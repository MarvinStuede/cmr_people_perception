#!/usr/bin/env python

import rospy
import actionlib
import sys
import json


from std_msgs.msg import Empty,Int64
import pickle
import math
import actionlib
from cmr_msgs.msg import TextToSpeechAction,TextToSpeechGoal


client=actionlib.SimpleActionClient('/sobi/speech/tts',TextToSpeechAction)
rospy.init_node('chat2')
client.wait_for_server()
print("done")

last=None
while True:
    inp = raw_input()
    print(inp)
    goal=TextToSpeechGoal(inp)
    client.send_goal(goal)





