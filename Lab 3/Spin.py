#!/usr/bin/env python

#From: Mark Silliman
#https://github.com/markwsilliman/turtlebot/blob/master/goforward.py

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# A very basic TurtleBot script that moves TurtleBot forward indefinitely. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python goforward.py

import rospy
from geometry_msgs.msg import Twist
from math import radians;

def spinWheels(u1, u2, time, cmd_vel):                   # Time in seconds
            radius = 0.03                               # wheel radius is 30mm
            length = 0.13                               # Distance between wheel and center is 13cm
            r = rospy.Rate(10)                          # Get sleep time

            linear_vel = (radius / 2) * (u1 + u2)
            ang_vel = (radius / (2 * length)) * (u1 - u2)

            twist_msg = Twist()
            twist_msg.linear.x = linear_vel
            twist_msg.angular.z = ang_vel

            for x in range(0, time * 10):               # 10Hz per second, for x seconds
                cmd_vel.publish(twist_msg)
                r.sleep()                               # Try double sleep to make sure robot stops before wheels start next instruction
                r.sleep()

            cmd_vel.publish(Twist())               # Then stop robot
            r.sleep()
            r.sleep()

class Spin():
    def __init__(self):
        # initiliaze
        rospy.init_node('Spin', anonymous=False)

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)
    
        # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
     
        #TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10);

        # as long as you haven't ctrl + c keeping doing...
        lines = 0                                   # Keep track of how many lines / squares we've gone
        squares = 0

        while not rospy.is_shutdown():
            spinWheels(-10, 10, 1, self.cmd_vel)                         # Spin!

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
	# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
	# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        Spin()
    except:
        rospy.loginfo("Exception")
	pass
