#!/usr/bin/env python
#-*-encoding:utf-8-*-
# Filename : hc_odom_test.py
# Author : ihainan
# Created Date : 2014/04/22 09:26
# Descriptiont : 码盘数据测试

import rospy
from geometry_msgs.msg import *
from bitathome_hardware_control.srv import *
from bitathome_hardware_control.msg import *
from math import *
from numpy import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf

node_name = "hc_odom_test"			# 节点名字

# 上一次更新的码盘数据
pre_codedisk_position_of_M1 = 0
pre_codedisk_position_of_M1 = 0
pre_codedisk_position_of_M1 = 0
last_time = None

# 机器人相关参数
radius_of_wheel = 0.10356 / 2		# 轮子半径
radius_of_base = 0.44724 / 2		# 底座半径
dis_per_unit = 2 * pi * radius_of_wheel / 1895			# 每单元轮子运动距离
inverse_matrix = mat([[sin(60.0 / 180 * pi), cos(60.0 / 180 * pi), radius_of_base], [-sin(60.0 / 180 * pi), cos(60.0 / 180 * pi), radius_of_base], [0, -1, radius_of_base]]).I		# 逆矩阵，用于计算 vx, vy 和 Omega
x = 0
y = 0
th = 0

# 广播相关
odom_broadcaster = tf.TransformBroadcaster()
odom_pub = rospy.Publisher("odom", Odometry)

# 由于码盘数据是循环值，两次码盘数据之差并非是真正变化值
def get_diff(dis):
	if abs(dis) > 4000000000:
			if dis < 0:
				dis = dis + 4294967295
			else:
				dis = dis - 4294967295
	return dis

# 获取码盘数据，计算和更新历程数据
def handle_codedisk_data(codedisk_data):
	global inverse_matrix, last_time
	global x, y, th
	global pre_codedisk_position_of_M1, pre_codedisk_position_of_M2, pre_codedisk_position_of_M3
	global odom_broadcaster, odom_pub

	# 获取数据
	codedisk_position_of_M1 = codedisk_data.m1
	codedisk_position_of_M2 = codedisk_data.m2
	codedisk_position_of_M3 = codedisk_data.m3
	current_time = rospy.get_time()
	vx = 0
	vy = 0
	vth = 0

	# 并非是第一次获取码盘值
	if last_time != None:
		# 获取码盘变化值
		dis_value1 = get_diff(codedisk_position_of_M1 - pre_codedisk_position_of_M1)
		dis_value2 = get_diff(codedisk_position_of_M2 - pre_codedisk_position_of_M2)
		dis_value3 = get_diff(codedisk_position_of_M3 - pre_codedisk_position_of_M3)
		if dis_value1 != 0 or dis_value2 != 0 or dis_value3 != 0 :
			print "码盘一变化：", dis_value1
			print "码盘二变化：", dis_value2
			print "码盘三变化：", dis_value3

			# 根据轮子的半径和周长，计算运动的距离	
			dis1 = dis_per_unit * dis_value1		# dis_per_unit 绝逼有问题，待测试
			dis2 = dis_per_unit * dis_value2
			dis3 = dis_per_unit * dis_value3

			# 根据轮子的运动距离，计算 v1, v2 和 v3
			time_change = (current_time - last_time)
			v1 = dis1 / time_change
			v2 = dis2 / time_change
			v3 = dis3 / time_change
			print v1, v2, v3

			# 根据 v1, v2, v3 计算 vx, vy, omega
			v_m = inverse_matrix * mat([-v2, -v1, -v3]).T
			print v_m[0], v_m[1], v_m[2]
			print time_change

			# 根据 vx, vy, omega，计算 sx, sy，得出里程数据
			# 后面参数为矫正值
			sx = time_change * v_m[0] * 0.1 * 0.7714 * 0.95
			sy = time_change * v_m[1] * 0.1 * 0.7714 * 0.95
			sth = time_change * v_m[2] * 0.1 * 0.7714 * 0.95 / (2 * pi) * 360 / 0.9 / 1.07 * 1.08 / 180.0 * pi		# 弧度表示
			print "里程数据 : ", sx, sy, sth

			# 当前坐标，此处计算公式存在缺陷，仅考虑单纯平移和单纯旋转的情况
			x = x + sx * cos(th) - sy * sin(th) 
			y = y + sx * sin(th) + sy * cos(th)
			th = th + sth
			print "当前位置：", x, y, th * 180.0 / pi
			print ""

			# 重新计算，这是实际的 vx， vy 和 vth
			vx = v_m[0] * 0.1 * 0.7714 * 0.95
			vy = v_m[1] * 0.1 * 0.7714 * 0.95
			vth = v_m[2] * 0.1 * 0.7714 * 0.95 / (2 * pi) * 360 / 0.9 / 1.07 * 1.08 / 180.0 * pi

	ct = rospy.Time.now()			# 当前时间

	# 发送 tf
	quaternion = Quaternion()
	quaternion.x = 0.0
	quaternion.y = 0.0
	quaternion.z = sin(th / 2.0)	
	quaternion.w = cos(th / 2.0)	
	# odom_broadcaster.sendTransform((x, y, 0), (quaternion.x ,quaternion.y, quaternion.z, quaternion.w), ct, "odom", "base_link")
	odom_broadcaster.sendTransform((x, y, 0), (quaternion.x ,quaternion.y, quaternion.z, quaternion.w), ct, "base_link", "odom")

	# 发送里程 message
	odom = Odometry()
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";
	odom.header.stamp = ct
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = quaternion;
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.angular.z = vth;
	odom_pub.publish(odom)

	# 存储
	pre_codedisk_position_of_M1 = codedisk_position_of_M1
	pre_codedisk_position_of_M2 = codedisk_position_of_M2
	pre_codedisk_position_of_M3 = codedisk_position_of_M3
	last_time = current_time

if __name__ == "__main__":
	# 初始化节点
	rospy.init_node(node_name)

	# 监听码盘数据
	rospy.Subscriber("/hc_cmd_interface/code_disk", CodeDisk, handle_codedisk_data)

	# 等待
	rospy.spin()
