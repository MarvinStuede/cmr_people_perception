#!/usr/bin/env python

import rospy
import actionlib
import sys
import json


from std_msgs.msg import Empty,Int64
import pickle
import math


str_pub = rospy.Publisher("/demo_rathaus/command", Int64, queue_size=1)
rospy.init_node('chat')

last=None
while True:
    inp = raw_input()
    print(inp)
    if inp.isdigit():
        inpo=int(inp)
        if(inpo==0 or inpo==1 or inpo==2 or inpo==3 or inpo==4 or inpo==5):
            if not(last==inpo):
                last=inpo
                str_pub.publish(int(inp))





