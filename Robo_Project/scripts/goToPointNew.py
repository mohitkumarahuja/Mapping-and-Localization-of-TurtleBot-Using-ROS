#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# TurtleBot must have minimal.launch & amcl_demo.launch
# running prior to starting this script
# For simulation: launch gazebo world & amcl_demo prior to run this script

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from std_msgs.msg import Bool

class GoToPose():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)
        self.goal_sent = False
        initial_pose = PoseWithCovarianceStamped()
        initialPose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.header.frame_id = "map"
        initial_pose.pose.pose.position.x = 9.0
        initial_pose.pose.pose.position.y =  1.05
        initial_pose.pose.pose.orientation.z = 1.0
        initial_pose.pose.pose.orientation.w = 0.08
        initial_pose.pose.covariance[0] = 0.25
        initial_pose.pose.covariance[7] = 0.25
        initial_pose.pose.covariance[35] = 0.06

        rate = rospy.Rate(10)
        i = 1
        while i < 8:
            initialPose_pub.publish(initial_pose)
            i += 1
            rate.sleep()

	# What to do if shut down (e.g. Ctrl-C or failure)
	rospy.on_shutdown(self.shutdown)
	
	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

    

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(300)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        
        navigator = GoToPose()

        # Customize the following values so they are appropriate for your location
        position = {'x': 3.03, 'y' : -1.69}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.95, 'r4' : -0.15}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = navigator.goto(position, quaternion)

        if success:
            goal_pub = rospy.Publisher("centerReached_flag", Bool, queue_size=10)
            centerReached = True
            rate = rospy.Rate(10)
            i = 1
            while True:
            
                goal_pub.publish(centerReached)
                i += 1
                rate.sleep()
        else:
            rospy.loginfo("The base failed to reach the desired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

