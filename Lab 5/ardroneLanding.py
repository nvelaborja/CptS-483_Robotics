#!/usr/bin/env python

"""
Author: James Irwin (james.irwin@wsu.edu)
Description:
    Example code for getting started with the ardrone
"""

import rospy
from sensor_msgs.msg import Image #for recieving video feed
from geometry_msgs.msg import Twist # controlling the movements
from std_msgs.msg import Empty #send empty message for takeoff and landing
import numpy 
import cv2
from intro_to_robotics.image_converter import ToOpenCV

class QuadcopterController:

    global arState 
    global searchCounter
    global landingCounter
    global shutdownCounter

    def __init__(self):
        #create message subscriber for receiving images
        self.image_sub = rospy.Subscriber('/ardrone/image_raw', Image, self.image_callback)
        #create message publisher for sending drone movement commands
        self.movement_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # Set global values
        global arState
        arState = "SEARCH"
        global searchCounter
        searchCounter = 200
        global landingCounter
        landingCounter = 50
        global shutdownCounter
        shutdownCounter = 50
    
    def image_callback(self, image):
        image = ToOpenCV(image)
        location = self.get_target_location(image)
        global arState
        global searchCounter
        global landingCounter
        global shutdownCounter
        rx = None
        ry = None

        #
        #control logic/commands here
        #location will be a None object if target is not visible
        #otherwise, location is a tuple (x,y)
        #
        

        #I'm just telling it to go straight up for now
        twist = Twist() #create new empty twist message

        # Get info on where landing pad is relative to center screen
        if (location is not None):
            x, y = location
            rx = x - 320
            ry = -(y - 180)         # y is upside down in the image apparently

        print ("Current State: " + str(arState) + " - Location: " + str(rx) + ", " + str(ry))

        # Going to run sort of a state machine here
        if (arState == "SEARCH"): # STATE: SEARCH - Rise until we see platform
            twist.linear.z = 1.0

            if (location is not None):
                searchCounter -= 1
            if (searchCounter == 0):
                arState = "ROTATE"

        if (arState == "ROTATE"): # STATE: ROTATE - Rotate until centered x ~ 0 and y > 0
            twist.angular.z = 0.2
            forwardCenter = range(-10, 10)

            if ry > 0 and rx in forwardCenter:
                arState = "FORWARD"
            
        if (arState == "FORWARD"): # STATE: FORWARD - Move until centered y ~ 0 and x ~ 0
            twist.linear.x = 0.2
            center = range(-10, 10)

            if ry in center:
                if rx not in center:
                    arState = "ROTATE"

                if rx in center and ry in center:
                    arState = "DESCEND"

        if (arState == "DESCEND"): # STATE: DESCEND - Drop!
            xGood = False
            yGood = False
            center = range(-10, 10)

            print "This is dumb"

            if rx in center and ry in center:
                arState = "SHUTDOWN"

            if (rx >= 10):
                twist.linear.x = -0.01
            elif (rx <= -10):
                twist.linear.x = 0.01
            if (ry >= 10):
                twist.linear.y = -0.01
            elif (ry <= -10):
                twist.linear.y = 0.01
                

        if (arState == "SHUTDOWN"):  #STATE: SHUTDOWN - Turn off motors and fall
            shutdownCounter -= 1

            if (shutdownCounter < 1):
                rospy.signal_shutdown("Currently above landing pad. Let's try to land!")



        self.movement_pub.publish(twist) #publish message

    def get_target_location(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_orange = numpy.array([10,10,10])
        upper_orange = numpy.array([255,255,255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        M = cv2.moments(mask)
        location = None
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            location = (cx, cy)
            cv2.circle(image, (cx,cy), 3, (0,0,255), -1)

        cv2.imshow("camera", image)
        cv2.waitKey(1)
        return location




#callback function that gets run when node shuts down
def shutdown_callback():
    print "shutting down..."
    drone_land_pub.publish(Empty()) #make the drone land
    cv2.destroyAllWindows() #cleanup opencv windows
    print "done!"


if __name__ == "__main__":
    rospy.init_node("quadcopter_controller")
    #ardrone uses specialized topics for landing and taking off
    drone_takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
    drone_land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
    controller = QuadcopterController()

    #register callback function to land drone if we kill the program
    rospy.on_shutdown(shutdown_callback) 

    rospy.sleep(1) #wait for a second to wait for node to fully connect
    drone_takeoff_pub.publish(Empty()) #command drone to takeoff

    #this function loops and waits for node to shutdown
    #all logic happens in the image_callback function
    rospy.spin()

