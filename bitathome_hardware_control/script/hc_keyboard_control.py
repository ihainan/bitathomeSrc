#!/usr/bin/env python
#-*-encoding:utf-8-*-
# Filename : hc_keyboard_control.py
# Author : ihainan
# Created Date : 2014/04/11 09:26
# Descriptiont : 键盘运动控制

import os
import sys
import tty, termios
import rospy
from geometry_msgs.msg import *
from bitathome_hardware_control.srv import *

default_speed = 300					# 默认速度值，不宜太大

# 键盘监控循环
def keyboard_loop():
	global ser, default_speed
	rate = rospy.Rate(2)

	# 显示提示信息
	print "Reading from keyboard"  
	print "Use WASD keys to control the robot"
	print "Press q to quit"

	# 读取按键循环
	while not rospy.is_shutdown():
		# 无回显
		fd = sys.stdin.fileno()
		old_settings = termios.tcgetattr(fd) 
		old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO  
		try : 
			tty.setraw( fd )  
			ch = sys.stdin.read( 1 )  
		finally:
			termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

		# 控制运动
		if ch == "w":
			resp = ser(default_speed, -default_speed, 0, 0, 0)
		elif ch == "s":
			resp = ser(-default_speed, default_speed, 0, 0, 0)
		elif ch == "a":
			resp = ser(-default_speed, -default_speed, -default_speed, 0, 0)
		elif ch == "d":
			resp = ser(default_speed, default_speed, default_speed, 0, 0)
		elif ch == "e":
			resp = ser(0, 0, 0, 0, 0)
		elif ch == "q":
			exit()
		rate.sleep()
	pass
	
if __name__ == "__main__":
		# 初始化节点
		rospy.init_node("hc_keyboard_control")

		# 运动控制服务
		ser = rospy.ServiceProxy("/hc_cmd_interface/motor_speed", MotorSpeed)

		# 键盘监控循环
		keyboard_loop()
