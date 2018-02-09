#!/usr/bin/env python

#Node for subscribing target flags and when they are set, it wil publish taget ose 
# on move_base_simple/goal topic 

# import required packages
import rospy, os
from geometry_msgs.msg import Point, PoseStamped, Twist
from std_msgs.msg import Bool, String

global flagOnce

class GoToTarget():
    
    def __init__(self):

                

        rospy.init_node("goToTarget1")
        

        
        rospy.loginfo("subscribing goto flag")
        rospy.Subscriber("/relay/arm_status", String, self.get_goToOnFlag)

        rospy.loginfo("subscribed goto flag")


        rospy.spin()


    def get_goToOnFlag(self, msg3):
        global flagOnce
        if msg3.data == "ppdone" and flagOnce == False:
            rospy.loginfo("gotToOn flag true")
            self.stopAndGoToTarget()
            flagOnce = True
        else:
            rospy.loginfo("gotToOn flag false")
            return

    def stopAndGoToTarget(self):
        rospy.Subscriber("target_pose1", PoseStamped, self.get_target_pose0)

    def get_target_pose0(self, msg3):
        rospy.loginfo("publishing goal")
        msg3.pose.position.x = msg3.pose.position.x + 0.5
        msg3.pose.position.y = msg3.pose.position.y + 0.5
        msg3.pose.orientation.z = -0.7
        msg3.pose.orientation.w = 0.7
        target1_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
        rate = rospy.Rate(10)
        i = 1
        while i < 5:
            target1_pub.publish(msg3)
            i += 1
            rate.sleep()
        target1Flag_pub = rospy.Publisher("target1_published", Bool, queue_size=10)
        target1_flag = True
        while True:
             target1Flag_pub.publish(target1_flag)
             rate.sleep()
        os.execvp('/bin/sh',['/bin/sh', '-c', "rosnode kill /my_gototarget1"])
        
        rospy.loginfo("Publishing target1 pose to move_base_simple/goal topic")

if __name__ == '__main__':
    try:
        flagOnce = False
        GoToTarget()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("goToTarget node terminated.")
