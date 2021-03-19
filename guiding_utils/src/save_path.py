#!/usr/bin/env python

import rospy
import actionlib
import sys
import json



from spencer_tracking_msgs.msg import TrackedPersons
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
import pickle
import math


class SavePath(object):


    def __init__(self) :

        rospy.Subscriber('/odom',Odometry , self.odom)
        rospy.Subscriber('/spencer/perception/tracked_persons',TrackedPersons,self.persons)
        rospy.Subscriber('/save_path',Empty,self.save)
        rospy.Subscriber('/reset',Empty,self.delete)
        self.path_robot=Path()
        self.path_persons=Path()
        self.last_pos=None
        self.pub_path=rospy.Publisher('/path_robot',Path,queue_size=1)
        self.min_dis=100



        #Subscribing to Localisation Topics
        #rospy.loginfo("Subscribing to Localisation Topics")
        #rospy.Subscriber('closest_node', String, self.closestNodeCallback)
        #rospy.Subscriber('current_node', String, self.currentNodeCallback)
        #rospy.loginfo(" ...done")

        rospy.spin()
       
    def odom(self,odom):
        pos=PoseStamped()
        pos.header=odom.header
        pos.pose=odom.pose.pose
        if len(self.path_robot.poses)<1:
            self.path_robot.poses.append(pos)
            self.last_pos=pos
        else:
            dis_now=math.sqrt((pos.pose.position.x-self.last_pos.pose.position.x)**2 +(pos.pose.position.y-self.last_pos.pose.position.y)**2)
            #print(dis_now)
            if dis_now > 0.1:
                self.last_pos=pos
                self.path_robot.poses.append(pos)
        #print(self.path_robot)
        

    def persons(self,person):

        for p in person.tracks:
            pos=PoseStamped()
            pos.header=person.header
            pos.pose=p.pose.pose
            dis_now=math.sqrt((pos.pose.position.x-self.last_pos.pose.position.x)**2 +(pos.pose.position.y-self.last_pos.pose.position.y)**2)
            if dis_now<self.min_dis:
                self.min_dis=dis_now
            self.path_persons.poses.append(pos)
        
        #print(self.path_persons)

    def save(self,_):
        print(self.path_robot)
        print(self.min_dis)
        self.path_robot.header.frame_id='odom'
        self.path_robot.header.stamp=rospy.Time.now()
        self.pub_path.publish(self.path_robot)
        #pickle.dump(self.path_persons,open( 'path_person.yaml', "wb" ))
        pickle.dump( self.path_robot, open( "robot_path.p", "wb" ) )
        

    def delete(self,_):
        
        self.path_persons=Path()
        self.path_robot=Path()


if __name__ == '__main__':
    mode="normal"
    rospy.init_node('SavePath')
    server = SavePath()
