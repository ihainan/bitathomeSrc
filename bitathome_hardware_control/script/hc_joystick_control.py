#!/usr/bin/env python
#-*-encoding:utf-8-*-
# Filename : hc_joystick_control.py
# Author : ihainan
# Created Date : 2014/04/11 09:26
# Descriptiont : 手柄运动控制

import rospy
from geometry_msgs.msg import *
from bitathome_hardware_control.srv import *
from sensor_msgs.msg import *

default_speed = 300					# 默认速度值，不宜太大

# JoyStick回调，用于监控手柄变化
def joy_callback(data):
	global joyData
	joyData = data

# 循环，用于调用 速度控制 服务
def joy_stick_loop():
	global joyData
	global pub, default_speed
	rate = rospy.Rate(3)
	while not rospy.is_shutdown():
		if joyData == None or len(joyData.axes) == 0:
			continue
		elif joyData.axes[1] == -1:
			resp = ser(-default_speed, default_speed, 0, 0, 0)
			if resp.result == 1:
					print "后退 : 成功"
			else:
					print "后退 : 失败"
		elif joyData.axes[1] == 1:
			resp = ser(default_speed, -default_speed, 0, 0, 0)
			if resp.result == 1:
					print "前进 : 成功"
			else:
					print "前进 : 失败"
		elif joyData.axes[0] == 1:
			resp = ser(-default_speed, -default_speed, -default_speed, 0, 0)
			if resp.result == 1:
					print "左旋 : 成功"
			else:
					print "左旋 : 失败"
		elif joyData.axes[0] == -1:
			resp = ser(default_speed, default_speed, default_speed, 0, 0)
			if resp.result == 1:
					print "右旋 : 成功"
			else:
					print "右旋 : 失败"
		else:
			resp = ser(0, 0, 0, 0, 0)
			if not resp.result == 1:
				print "停止 : 失败"
		rate.sleep()
						
if __name__ == "__main__":
		# 初始化节点
		rospy.init_node("hc_joystick_control")

		# 运动控制服务
		ser = rospy.ServiceProxy("/hc_cmd_interface/motor_speed", MotorSpeed)
		# 手柄数据
		joyData = Joy()
		pub = rospy.Subscriber("/joy", Joy, joy_callback)

		# 循环监控
		joy_stick_loop()
