#!/usr/bin/env python

#Node to rotate the turtlebot in place 360degrees

# import required packages
import rospy
from geometry_msgs.msg import Twist
from math import radians
from std_msgs.msg import Bool

global counter
counter = False

class RotateXY():
    def __init__(self):
        
        rospy.init_node("rotate")
        

        rospy.Subscriber("centerReached_flag", Bool, self.get_centerFlag)
        rospy.spin()

    def get_centerFlag(self, msg):
        global counter
        if msg.data == True and counter == False :
            self.rotate()
            counter = True
        else:
            return      

    def rotate(self):
        global goToOnFlag
        goToOnFlag = False
        
        cmd_vel = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        
        

        #create a Twist varaiable
        turn_cmd = Twist()
        turn_cmd.linear.x = 0
        turn_cmd.linear.y = 0
        turn_cmd.linear.z = 0
        turn_cmd.angular.x = 0
        turn_cmd.angular.y = 0
        turn_cmd.angular.z = radians(10)
        
        t0 = rospy.Time.now().to_sec()

        current_angle = 0
        relative_angle = radians(360)
        rospy.loginfo("Rotating...")

        while (current_angle < relative_angle):
            cmd_vel.publish(turn_cmd)
            t1 = rospy.Time.now().to_sec()
            current_angle = radians(8)*(t1-t0)

        turn_cmd.angular.z = 0
        cmd_vel.publish(turn_cmd)
        rospy.loginfo("Finished rotation...")

        rate = rospy.Rate(10)
        rotate_flag = rospy.Publisher('rotate_flag', Bool, queue_size=10)
        rot_flag = True
        rospy.loginfo("Publishing rotate set flag on topic /rotate_Flag")
        
        while True:
            
            #set the rotated flag on topic roate_flag
            rotate_flag.publish(rot_flag)
            rate.sleep()


if __name__ == '__main__':
    try:
        RotateXY()
    except rospy.ROSInterruptException:
        rospy.loginfo("rotate node terminated.")
