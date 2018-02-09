#!/usr/bin/env python

#Node for detecting AR Mrkers and calculating absolute pose of markers in map and publishing the pose 
# onto the target_pose0 and target_pose1 topics with related flags

# import required packages
import rospy, numpy, math, os
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Bool

class Tags():
       
    def __init__(self):
        
        self.counter0 = 0
        self.counter1 = 0
        #global t0_flag, t1_flag
        self.t0_flag, self.t1_flag = False, False
        self.xx0, self.yy0, self.zz0 = 0.0, 0.0, 0.0
        self.xx1, self.yy1, self.zz1 = 0.0, 0.0, 0.0

        global tag_target0, tag_target1
        #create PoseStamped variables to store the pose
        tag_target0 = PoseStamped()
        tag_target1 = PoseStamped()

        rospy.init_node("localization_new")

        #read optional list of tags to be detected
        self.tag_ids = rospy.get_param('~tag_ids', None)

        #Publish on /target_pose0 & 1 ,topic as a PoseStamped message
        self.tag_pub0 = rospy.Publisher("target_pose0", PoseStamped, queue_size=10)
        self.tag_pub1 = rospy.Publisher("target_pose1", PoseStamped, queue_size=10)

        #Subscribe pose from amcl_pose
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.get_turtlePose)

        #subscribe pose from alvar markers
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.get_tags)
        rospy.loginfo("Publishing marker pose and flags on topic target_pose0 target_pose1 target0_flag target1_flag")



        rospy.spin()

    def get_turtlePose(self, msg):
        # get the pose of turtlebot
        global turtleX, turtleY, turtleZ, turtleXrot, turtleZrot, turtleWrot, turtleAngle
        turtleX = msg.pose.pose.position.x
        turtleY = msg.pose.pose.position.y
        turtleZ = msg.pose.pose.position.z
        turtleZrot = msg.pose.pose.orientation.z 
        turtleWrot = msg.pose.pose.orientation.w  
        turtleAngle = math.atan2(2*turtleZrot*turtleWrot, 1-2*turtleZrot*turtleZrot)
 
    def get_tags(self, msg):
        #get pose of markers 0 and 1 and calculate absolute pose and publish onto target_pose0&1 topics
        
        #if self.counter == 2:
        #    return
        
        self.tag_pub0.publish(tag_target0)
        self.tag_pub1.publish(tag_target1)

        """rate = rospy.Rate(10)
        #publish target0 flag on target0_Flag
        target0_flag = rospy.Publisher("target0_Flag", Bool, queue_size=10)
        rospy.loginfo("publishing target0 flag set on /taget0_Flag topic")
        i = 1
        while i < 50:
            i += 1
            target0_flag.publish(self.t0_flag)
        
            
        rate = rospy.Rate(10)
        #publish target0 flag on target0_Flag
        target1_flag = rospy.Publisher("target1_Flag", Bool, queue_size=10)
        rospy.loginfo("publishing target1 flag set on /taget1_Flag topic")
        i = 1
        while i < 50:
            i += 1
            target1_flag.publish(self.t1_flag)"""

        if self.t0_flag == True and self.t1_flag == True:
            goToOn = rospy.Publisher("goToOn_flag", Bool, queue_size=10)
            goToOn_flag = True
            i = 1
            while i < 100:
                i += 1
                goToOn.publish(goToOn_flag)
            return
        else:
            goToOn = rospy.Publisher("goToOn_flag", Bool, queue_size=10)
            goToOn_flag = False
            goToOn.publish(goToOn_flag)
            

        #get number of amrkers detected
        n = len(msg.markers)

        if n == 0:
            return

        #for each markers (0,1) detected do the following
        for tag in msg.markers:
            if tag.id == 0:
                #counter if required
                if self.t0_flag == True:
                    continue

                affine_transf = [[math.cos(turtleAngle), -math.sin(turtleAngle), 0, turtleX], [math.sin(turtleAngle), math.cos(turtleAngle), 0, turtleY], [0, 0, 1, turtleZ], [0, 0, 0, 1]]
                """affine_transf = numpy.array([[1-2*turtleZrot*turtleZrot, 2*turtleZrot*turtleWrot, 0, turtleX], [-2*turtleZrot*turtleWrot, 1-2*turtleZrot*turtleZrot, 0, turtleY], [0, 0, 1, turtleZ], [0, 0, 0, 1]])"""
                point_in_baseframe = [[tag.pose.pose.position.x], [tag.pose.pose.position.y], [tag.pose.pose.position.z], [1]]
                point_in_mapframe = numpy.matmul(affine_transf, point_in_baseframe)

                self.xx0 = self.xx0 + point_in_mapframe[0]
                self.yy0 = self.yy0 + point_in_mapframe[1]
                self.zz0 = self.zz0 + point_in_mapframe[2] 
                self.counter0 += 1

                if self.counter0 == 1:
                    tag_target0.pose.position.x = self.xx0/1.0
                    tag_target0.pose.position.y = self.yy0/1.0               
                    tag_target0.pose.position.z = self.zz0/1.0
                   
                    tag_target0.pose.orientation.w = 1

                    #add time stamp and frameid
                    tag_target0.header.stamp = rospy.Time.now()
                    tag_target0.header.frame_id = "map"
                   
                    self.t0_flag = True



            elif tag.id == 1:
                #counter if required
                if self.t1_flag == True:
                    continue

                affine_transf = numpy.array([[math.cos(turtleAngle), -math.sin(turtleAngle), 0, turtleX], [math.sin(turtleAngle), math.cos(turtleAngle), 0, turtleY], [0, 0, 1, turtleZ], [0, 0, 0, 1]])
                """affine_transf = numpy.array([[1-2*turtleZrot*turtleZrot, 2*turtleZrot*turtleWrot, 0, turtleX], [-2*turtleZrot*turtleWrot, 1-2*turtleZrot*turtleZrot, 0, turtleY], [0, 0, 1, turtleZ], [0, 0, 0, 1]])"""
                point_in_baseframe = numpy.array([[tag.pose.pose.position.x], [tag.pose.pose.position.y], [tag.pose.pose.position.z], [1]])
                point_in_mapframe = numpy.matmul(affine_transf, point_in_baseframe)

                self.xx1 = self.xx1 + point_in_mapframe[0]
                self.yy1 = self.yy1 + point_in_mapframe[1]
                self.zz1 = self.zz1 + point_in_mapframe[2] 
                self.counter1 += 1

                if self.counter1 == 5:
                    tag_target1.pose.position.x = self.xx1/5.0
                    tag_target1.pose.position.y = self.yy1/5.0               
                    tag_target1.pose.position.z = self.zz1/5.0
                   
                    tag_target1.pose.orientation.w = 1

                    #add time stamp and frameid
                    tag_target1.header.stamp = rospy.Time.now()
                    tag_target1.header.frame_id = "map"
                   
                    self.t1_flag = True
            else:
                continue

if __name__ == '__main__':
    try:
        Tags()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Localization node terminated.")
        



        
        
        
               
        

