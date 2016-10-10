#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import numpy as np
from image_converter import ToOpenCV, depthToOpenCV

objectFound = False
resetCount = 20

def spinWheels(u1, u2, time, cmd_vel):                   # Time in seconds
            radius = 0.03                               # wheel radius is 30mm
            length = 0.13                               # Distance between wheel and center is 13cm
            r = rospy.Rate(10)                          # Get sleep time

            linear_vel = (radius / 2) * (u1 + u2)
            ang_vel = (radius / (2 * length)) * (u1 - u2)

            twist_msg = Twist()
            twist_msg.linear.x = linear_vel
            twist_msg.angular.z = ang_vel

            for x in range(0, int(math.ceil(time * 10))):               # 10Hz per second, for x seconds
                cmd_vel.publish(twist_msg)
                r.sleep()                               # Try double sleep to make sure robot stops before wheels start next instruction
                r.sleep()

            cmd_vel.publish(Twist())               # Then stop robot
            r.sleep()
            r.sleep()

#this function does our image processing
#returns the location and "size" of the detected object
def process_image(image):
    #convert color space from BGR to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #create bounds for our color filter
    lower_bound = np.array([0, 10, 10])
    upper_bound = np.array([10,255,255])

    #execute the color filter, returns a binary black/white image
    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

    #display the results of the color filter
    cv2.imshow("image_mask", mask)

    #calculate the centroid of the results of the color filer
    M = cv2.moments(mask)
    location = None
    magnitude = 0
    if M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        magnitude = M['m00']
        location = (cx-320, cy-240) #scale so that 0,0 is center of screen
        #draw a circle image where we detected the centroid of the object
        cv2.circle(image, (cx,cy), 3, (0,0,255), -1)

    #display the original image with the centroid drawn on the image
    cv2.imshow("processing result", image)
    cv2.imshow("image_mask", image);

    #waitKey() is necessary for making all the cv2.imshow() commands work
    cv2.waitKey(1)
    return location, magnitude


class Node:
    def __init__(self):
        #register a subscriber callback that receives images
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback, queue_size=1)

        #create a publisher for sending commands to turtlebot
        self.movement_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    #this function wll get called every time a new image comes in
    #all logic occurs in this function
    def image_callback(self, ros_image):
        global objectFound
        global resetCount

        # convert the ros image to a format openCV can use
        cv_image = np.asarray(ToOpenCV(ros_image))

        #run our vision processing algorithm to pick out the object
        #returns the location (x,y) of the object on the screen, and the
        #"size" of the discovered object. Size can be used to estimate
        #distance
        #None/0 is returned if no object is seen
        location, magnitude = process_image(cv_image)

        #log the processing results
        rospy.logdebug("image location: {}\tmagnitude: {}".format(location, magnitude))

        ###########
        # Insert turtlebot controlling logic here!
        ###########          
        threshold = 10                                              # set pixel threshold to +- x for "centered" 
        wheelSpeed = 0.02
        moveTime = 0.1

        rospy.loginfo("L: " + str(location) + " | M: " + str(magnitude))

        if objectFound == True:                                             # Move forward for a bit so magnitude is reset to new object
            rospy.loginfo("Poop")
            if resetCount < 0:
                objectFound = False
                resetCount = 20
            else:
                rospy.loginfo("Going forward to reset dumb image")
                spinWheels(2, 2, 0.1, self.movement_pub)
                resetCount -= 1
        elif location is None:                                      # Keep rotating till we see something
            spinWheels(-0.1, 0.1, 1, self.movement_pub)
        elif magnitude >= 10000000:                                 # Stop when we magnitude gets to 10 million, which is one square length in gazebo = 1m?
            objectFound = True                                      
            rospy.loginfo("Object found. Turning 90 degrees")
            spinWheels(-1.744, 1.744, 2, self.movement_pub) 
        else:                                                       # Once we find something, try to center it
            x, y = location                                         # Break location into x and y values, we only really care about x right now
            if x < -threshold:                                      # If point is left of center, rotate right a little bit
                rospy.loginfo("Left")
                spinWheels(wheelSpeed, -wheelSpeed, moveTime, self.movement_pub)
            elif x > threshold:                                     # If point is right of center, rotate left
                rospy.loginfo("Right")
                spinWheels(-wheelSpeed, wheelSpeed, moveTime, self.movement_pub)
            else:                                                   # If centered, move forward!
                rospy.loginfo("Center! Full speed ahead!")
                spinWheels(wheelSpeed * 100, wheelSpeed * 100, moveTime, self.movement_pub)
                

if __name__ == "__main__":
    rospy.init_node("lab2_example")
    node = Node()

    #this function loops and returns when the node shuts down
    #all logic should occur in the callback function
    rospy.spin()