#!/usr/bin/env python
#-*-encoding:utf-8-*-
# Filename : hc_odom_control.py
# Author : ihainan
# Created Date : 2014/04/25 09:26
# Descriptiont : 根据码盘数据计算里程

import rospy
from geometry_msgs.msg import *
from bitathome_hardware_control.srv import *
from bitathome_hardware_control.msg import *
from math import *
from numpy import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
import tf
import threading

node_name = "hc_odom_control"		# 节点名字

# 上一次更新的码盘数据
pre_codedisk_position_of_M1 = 0
pre_codedisk_position_of_M2 = 0
pre_codedisk_position_of_M3 = 0
last_time = None

# 机器人相关参数
radius_of_wheel = 0.10356 / 2		# 轮子半径
radius_of_base = 0.44724 / 2		# 底座半径
dis_per_unit = 2 * pi * radius_of_wheel / 24557			# 每单元轮子运动距离

# 逆矩阵，用于计算 vx, vy 和 Omega
inverse_matrix = mat([[sin(60.0 / 180 * pi), -cos(60.0 / 180 * pi), -radius_of_base], [-sin(60.0 / 180 * pi), -cos(60.0 / 180 * pi), -radius_of_base], [0, 1, -radius_of_base]]).I

# 初始位置，该位置同时也决定了世界坐标系
x = 0.0
y = 0.0
th = 0.0
vx = 0.0
vy = 0.0
vth = 0.0


# 激光数据
laserScan = LaserScan()
# scan_pub = rospy.Publisher("scan", LaserScan)

# 广播相关
odom_broadcaster = tf.TransformBroadcaster()
odom_pub = rospy.Publisher("odom", Odometry)

# 由于码盘数据是循环值，两次码盘数据之差并不一定是真正的变化值
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
	global x, y, th, vx, vy, vth
	global pre_codedisk_position_of_M1, pre_codedisk_position_of_M2, pre_codedisk_position_of_M3

	# 获取数据
	codedisk_position_of_M1 = codedisk_data.m1
	codedisk_position_of_M2 = codedisk_data.m2
	codedisk_position_of_M3 = codedisk_data.m3
	current_time = rospy.get_time()

	# 并非是第一次获取码盘值
	if last_time != None:
		# 获取码盘变化值
		dis_value1 = get_diff(codedisk_position_of_M1 - pre_codedisk_position_of_M1)
		dis_value2 = get_diff(codedisk_position_of_M2 - pre_codedisk_position_of_M2)
		dis_value3 = get_diff(codedisk_position_of_M3 - pre_codedisk_position_of_M3)

		# 起码有一个码盘发生数据变化时候才计算里程
		if abs(dis_value1) < 3:
			dis_value1 = 0
		if abs(dis_value2) < 3:
			dis_value2 = 0
		if abs(dis_value3) < 3:
			dis_value3 = 0
		if abs(dis_value1) > 2 or abs(dis_value2) > 2 or abs(dis_value3) > 2 :
			print "码盘一变化：", dis_value1
			print "码盘二变化：", dis_value2
			print "码盘三变化：", dis_value3

			# 根据轮子的半径和周长，计算运动的距离	
			dis1 = dis_per_unit * dis_value1
			dis2 = dis_per_unit * dis_value2
			dis3 = dis_per_unit * dis_value3

			# 根据轮子的运动距离，计算 v1, v2 和 v3
			time_change = (current_time - last_time)
			v1 = dis1 / time_change
			v2 = dis2 / time_change
			v3 = dis3 / time_change
			print "v1 = ", v1, "v2 = ", v2, "v3 = ", v3

			# 重新计算 vx, vy 和 vz，后面参数叫矫正值
			v_m = inverse_matrix * (mat([v1, v2, v3]).T)
			vx = v_m[0, 0]
			vy = v_m[1, 0]
			omega = v_m[2, 0]
			print "机器人坐标系速度：", vx, vy, omega
			(dx, dy, dth) = cal_odom(vx, vy, omega, th, time_change)
			print "里程变化量：", dx, dy, dth * 180 / pi

			# 计算世界坐标系中的vx 和 vy
			vx_in_w = vx * cos(-th) - vy * sin(-th)
			vy_in_w = vx * sin(-th) + vy * cos(-th)
			print "世界坐标系中的速度：", vx_in_w, vy_in_w
			
			# 当前位置（里程）
			x = x + dx
			y = y + dy
			th = th + dth * 1.07
			print "当前位置：", x, y, th * 180 / pi
			print ""

	# 存储
	pre_codedisk_position_of_M1 = codedisk_position_of_M1
	pre_codedisk_position_of_M2 = codedisk_position_of_M2
	pre_codedisk_position_of_M3 = codedisk_position_of_M3
	last_time = current_time

# 里程数据计算
def cal_odom(vx, vy, omega, th, dt):

	vx_in_w = vx * cos(-th) - vy * sin(-th)
	vy_in_w = vx * sin(-th) + vy * cos(-th)
	dx = vx_in_w * dt
	dy = vy_in_w * dt
	dth = omega * dt
	return (dx, dy, dth)

	# 无自旋的情况
	if abs(omega) <= 0.1:
		rospy.logerr("[info] : 角速度 %lf 变化小，认识是直线", omega / pi * 180)
		# 在世界坐标系中的速度
		vx_in_w = vx * cos(-th) - vy * sin(-th)
		vy_in_w = vx * sin(-th) + vy * cos(-th)
		# 在世界坐标系中的位移
		dx = vx_in_w * dt
		dy = vy_in_w * dt
		dth = 0
		return (dx, dy, dth)
	# 有自旋的情况
	else:
		rospy.logerr("[info] : 角速度 %lf 变化大，认识是曲线", omega / pi * 180)
		r = sqrt((pow(vx, 2) + pow(vy, 2)) * 1.0 / pow(omega, 2))
		dth = omega * dt
		dx = r * sin(-(dth + th)) - r * sin(-(th))
		dy = r * cos(-th) - r * cos(-(dth + th))
		return (dx, dy, dth)

def broadcast_odom():
	global odom_broadcaster, odom_pub
	global x, y, th, vx, vy, vth
	global laserScan
	rate = rospy.Rate(20)				# 降低更新频率，期待更好的性能
	is_first = True;
	last_odom = None
	while not rospy.is_shutdown():
		# 发送 tf
		ct = rospy.Time.now()			# 当前时间
		q = tf.transformations.quaternion_from_euler(0, 0, th)
		quaternion = Quaternion(q[0], q[1], q[2], q[3])
		odom_broadcaster.sendTransform((x, y, 0), q, ct, "base_link", "odom")

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


		# 检测里程数据是否有更新
		'''
		if is_first or check_odom_update(odom, last_odom):
			rospy.logerr("更新激光：%lf %lf %lf", vx, vy, vth)
			# 发送激光数据
			scan_pub.publish(laserScan)	
			if check_odom_update(odom, last_odom):
				is_first = False
		'''
		#scan_pub.publish(laserScan)	

		# 休眠时间
		last_odom = odom
		rate.sleep()

# 检查里程是否有更新
def check_odom_update(odom, last_odom):
	if last_odom == None:
			return False
	x1 = odom.pose.pose.position.x
	y1 = odom.pose.pose.position.y
	z1 = odom.pose.pose.position.z
	x2 = last_odom.pose.pose.position.x
	y2 = last_odom.pose.pose.position.y
	z2 = last_odom.pose.pose.position.z
	if abs(x1 - x2) > 0 or abs(y1 - y2) > 0 or abs(z1 - z2) > 0:
			return True
	return False

'''
def handle_scan_data(data):
	global laserScan
	laserScan = data
'''

if __name__ == "__main__":
	# 初始化节点
	rospy.init_node(node_name)

	# 监听码盘数据，计算当前位置
	rospy.Subscriber("/hc_cmd_interface/code_disk", CodeDisk, handle_codedisk_data)
	# rospy.Subscriber("/scan_2", LaserScan, handle_scan_data)

	# 单开线程用于发布码盘数据
	thread = threading.Thread(target = broadcast_odom)
	thread.setDaemon(True)		# 防止子线程无法接收到终止信号
	thread.start()

	# 等待
	rospy.spin()
