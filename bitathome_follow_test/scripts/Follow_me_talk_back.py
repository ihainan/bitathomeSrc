#!/usr/bin/env python
# -*- coding: utf-8 -*
#__author__ = 'tony'

import roslib;
import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
import sys

class TalkBack:
    def __init__(self, script_path):
        rospy.init_node('talk_back')
        rospy.on_shutdown(self.cleanup)
        self.voice = rospy.get_param("~voice", "voice_don_diphone")
        self.soundhandle = SoundClient()
        rospy.sleep(1)
        self.soundhandle.stopAll()
        rospy.sleep(1)
        rospy.Subscriber('/follow_me/talk_back', String, self.talk_back)

    def talk_back(self, msg):
        rospy.loginfo(msg.data)

        self.soundhandle.say(msg.data, self.voice)

    def cleanup(self):
        self.soundhandle.stopAll()
        rospy.loginfo("Shutting down talkback node...")

if __name__=="__main__":
    try:
        TalkBack(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Talkback node terminated.")
