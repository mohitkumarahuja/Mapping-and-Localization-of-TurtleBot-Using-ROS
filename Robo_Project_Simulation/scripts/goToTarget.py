#!/usr/bin/env python

#Node for subscribing target flags and when they are set, it wil publish taget ose 
# on move_base_simple/goal topic 

# import required packages
import rospy, os
from geometry_msgs.msg import Point, PoseStamped, Twist
from std_msgs.msg import Bool


class GoToTarget():
    def __init__(self):

                

        rospy.init_node("goToTarget")
        
        self.rotFlag = False
        self.flag = False
        
        rospy.loginfo("subscribing goto flag")
        rospy.Subscriber("goToOn_flag", Bool, self.get_goToOnFlag)

        rospy.Subscriber("rotate_flag", Bool, self.get_rotateFlag)

        rospy.loginfo("subscribed goto flag")


        rospy.spin()


    def get_goToOnFlag(self, msg3):
       
        if msg3.data == True and self.rotFlag == True:
            rospy.loginfo("gotToOn flag true")
            self.stopAndGoToTarget()
        else:
            rospy.loginfo("gotToOn flag false")
            return

    def stopAndGoToTarget(self):
        rospy.Subscriber("target_pose0", PoseStamped, self.get_target_pose0)

    def get_target_pose0(self, msg3):
        if self.flag == False:
            rospy.loginfo("publishing goal")
            msg3.pose.position.x = msg3.pose.position.x + 0.3
            msg3.pose.position.y = msg3.pose.position.y - 0.4
            msg3.pose.orientation.z = 0.7
            msg3.pose.orientation.w = 0.7
            target0_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
            rate = rospy.Rate(10)
            i = 1
            while i < 8:
                target0_pub.publish(msg3)
                i += 1
                rate.sleep()
            self.flag = True
        else:
            return
        #os.execvp('/bin/sh',['/bin/sh', '-c', "rosnode kill /my_gototarget"])
        
        rospy.loginfo("Publishing target0 pose to move_base_simple/goal topic")

    def get_rotateFlag(self, msg):
        self.rotFlag = msg.data

if __name__ == '__main__':
    try:
        GoToTarget()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("goToTarget node terminated.")
