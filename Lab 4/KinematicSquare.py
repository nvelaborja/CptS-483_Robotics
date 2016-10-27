import rospy
from geometry_msgs.msg import Twist
from math import radians
import math

cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

def spinWheels(u1, u2, time):  # u1 and u2 are wheel rotation speed , time is in second
    r = 0.04
    l = 0.115
    linear_vel = (r / 2) * (u1 + u2) #compute liniear velocity
    ang_vel = (r / (2 * l)) * (u1 - u2) #compute angular velocity
    r = rospy.Rate(5)  # 5HZ = 0.2 second

    twist_msg = Twist()
    twist_msg.linear.x = linear_vel
    twist_msg.angular.z = ang_vel


    count = time / 0.2
    for x in range(0, int(count)):   #move turtlebot depends on given u1 and u2
        cmd_vel.publish(twist_msg)
        r.sleep()
    rospy.loginfo("Movement finish.")

    # while haven't reached time
    # publish twist_msg
    # publish a new twist message to make the robot stop

class RunSquare_01():
    def __init__(self):
        # initiliaze
        rospy.init_node('runsquare', anonymous=False)

        # What to do you ctrl + c
        rospy.on_shutdown(self.shutdown)

        #self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        #r=rospy.Rate(5) #5HZ = 0.2 second
        count=0;
        while(count<4):
            u1=10
            u2=10
            #go forward 2 meter
            rospy.loginfo("Going Straight")   #when two wheels roll in same positive direction, turtlebot goes forward
            spinWheels(u1,u2,5) #5 seconds
            #turn 90 degree
            #u3=1.15
            #u4=-1.15
	    #u3=0.45
	    #u4=-0.45
	    u3=0.575;
	    u4=-0.575;	                          #when two wheels roll in opposite direction, turtlebot turns anticlockwise, since u3-u4>0
            rospy.loginfo("Turning...")
            spinWheels(u3,u4,12)
            count=count+1

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop Drawing Squares")
        cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        RunSquare_01()
    except:
        rospy.loginfo("node terminated.")

