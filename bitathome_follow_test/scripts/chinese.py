#!/usr/bin/env python
# -*- coding: utf-8 -*
#__author__ = 'tony'

import roslib;
import rospy
from std_msgs.msg import String
import sys
import os
import subprocess

rc = None

class TalkBack:
    def __init__(self, script_path):
        rospy.init_node('chinese')
        rospy.Subscriber('/follow_me/talk_back', String, self.talk_back)

    def talk_back(self, msg):
        global rc
        if rc != None:
            rc.terminate()
        rospy.loginfo(msg.data)
        rc = subprocess.Popen("ekho " + msg.data,shell=True);


if __name__=="__main__":
    try:
        rc = subprocess.Popen("ekho 你好",shell=True);
        TalkBack(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("chinese node terminated.")