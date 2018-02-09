#!/usr/bin/env python

# import required packages
import rospy, os
from geometry_msgs.msg import Point, PoseStamped, Twist
from std_msgs.msg import Bool

def stopAndGoToTarget():
    rospy.init_node("goToTarget1")
    rospy.Subscriber("target_pose1", PoseStamped, get_target_pose0)
    rospy.loginfo("subscribing")

    rospy.spin()

def get_target_pose0(msg3):
    rospy.loginfo("publishing goal")
    msg3.pose.position.x = msg3.pose.position.x + 0.5
    msg3.pose.position.y = msg3.pose.position.y + 0.5
    msg3.pose.orientation.z = -0.7
    msg3.pose.orientation.w = 0.7
    target0_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
    rate = rospy.Rate(10)
    i = 1
    while i < 8:
        target0_pub.publish(msg3)
        i += 1
        rate.sleep()
    os.execvp('/bin/sh',['/bin/sh', '-c', "rosnode kill /goToTarget1"])



if __name__ == '__main__':
    try:
        stopAndGoToTarget()
        rospy.loginfo("in main")
    except rospy.ROSInterruptException:
        rospy.loginfo("goToTarget node terminated.")
