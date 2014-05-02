#!/usr/bin/env python
#-*-encoding:utf-8-*-
# Filename : hc_codedisk_test.py
# Author : ihainan
# Created Date : 2014/04/25 21:12
# Descriptiont : 用于输出码盘数据，并测试里程数据是否正确

import roslib
import rospy
from voy_serialport_communication import VoySerialPort
from bitathome_hardware_control.srv import *
from bitathome_hardware_control.msg import *

node_name = "hc_codedisk_test"							# 节点名字
pub_of_codedisk = rospy.Publisher("/hc_cmd_interface/code_disk", CodeDisk)	# 码盘数据 topic
m2 = 2000000
m1 = 2000000
m3 = 2000000

if __name__ == "__main__":
	rospy.init_node(node_name)	# 初始化节点

	rate = rospy.Rate(5)
	c = CodeDisk()
	while not rospy.is_shutdown():
		# 前进
		'''
		m1 = m1 + 200
		if m1 > 4294967295:
			m1 = 0
		m2 = m2 - 200
		if m2 < 0:
			m2 = 4294967295
		m3 = m3
		'''
		c.m1 = m1
		c.m2 = m2
		c.m3 = m3
		pub_of_codedisk.publish(c)
		print "pub success"
		rate.sleep()

	rospy.spin()				# 等待
