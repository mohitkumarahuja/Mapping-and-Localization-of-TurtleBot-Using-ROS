#!/usr/bin/env python

#Node for goal status

# import required packages
import rospy, os
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool, String
import math 

global goalX, goalY


def goalStatus():
    
    rospy.init_node("goalStatus")
    
    rospy.Subscriber("target1_published", Bool, get_goToOnFlag)

    #subscribe move base goalget_turge
    rospy.Subscriber("move_base_simple/goal", PoseStamped, get_goal)

    rospy.spin()

def get_goToOnFlag(msg):
    if msg.data == True:

        #Subscribe pose from amcl_pose
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, get_turtlePose) 
        
    else:
        return
            
        

def get_turtlePose(msg):
    global turtleX, turtleY
    turtleX = msg.pose.pose.position.x
    turtleY = msg.pose.pose.position.y
    
    if (math.fabs(turtleX-goalX) < 0.2) and (math.fabs(turtleY-goalY) < 0.2):
        goal_pub = rospy.Publisher("nav_status", String, queue_size=10)
        goalStatus1 = "navdone"
        rate = rospy.Rate(10)
        i = 1
        while i < 500:
            goal_pub.publish(goalStatus1)
            i += 1
            rate.sleep()
        os.execvp('/bin/sh',['/bin/sh', '-c', "rosnode kill /my_goalStatus1"])
            

def get_goal(msg):
    
    global goalX, goalY
    goalX, goalY = 0.0, 0.0
    goalX = msg.pose.position.x
    goalY = msg.pose.position.y
        
    


if __name__ == '__main__':
    try:
        goalStatus1 = "nav"
        goalStatus()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("goalStatus node terminated.")
