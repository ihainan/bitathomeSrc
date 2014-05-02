#!/usr/bin/env python
#-*-encoding:utf-8-*-
# Filename : hc_cmd_interface.py
# Author : ihainan
# Created Date : 2014/04/17 16:22
# Descriptiont : 命令集合

import roslib
import rospy
from voy_serialport_communication import VoySerialPort
from bitathome_hardware_control.srv import *
from bitathome_hardware_control.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
import threading
from math import *

# 全局变量
node_name = "hc_cmd_interface"							# 节点名字
# vsp = VoySerialPort("/dev/pts/16", 2000000, 8)		# 串口通信对象
vsp = VoySerialPort("/dev/ttyUSB0", 2000000, 8)			# 串口通信对象
# delayTime_of_code_disk = 100							# 获取码盘数据间隔时间
delayTime_of_code_disk = 50							# 获取码盘数据间隔时间
max_speed = 800											# 最大速度
radius_of_base = 0.44724 / 2							# 轮子到中心的距离

# 数学相关
mysin = sin(60.0 / 180 * pi)
mycos = cos(60.0 / 180 * pi)

# topic 相关
pub_of_codedisk = rospy.Publisher("/hc_cmd_interface/code_disk", CodeDisk)	# 码盘数据 topic
cmd_vel = rospy.Publisher("/hc_cmd_interface/code_disk", Twist)

# 设置五个电机的速度
def set_five_motor_speed(speed):
	global vsp, node_name, max_speed
	# 为保证安全，检查速度在允许的范围之内
	for s in speed:
		if s > max_speed:
			rospy.logerr("Speed is beyond the allowable range")
			return False

	# 生成即将写入的数据
	speedBuf = vsp.split_to_bytes(speed)
	pBuffer = vsp.generate_sig_cmd(1, 0x38, 0x0a, 0x08, 0x70, speedBuf)

	# 写入数据
	if vsp.write_data(pBuffer, "hex"):
		return True
	else:
		rospy.logerr("Write to serialport failed")
		return False

# 根据特定向量和角速度运动
def vector_move(vx, vy, omega):
	global radius_of_base
	v1 = vx * mysin - vy * mycos  - radius_of_base * omega
	v2 = -vx * mysin - vy * mycos  - radius_of_base * omega
	v3 = vy - radius_of_base * omega

	print "speed = ", v1, v2, v3
	set_five_motor_speed([int(v1), int(v2), int(v3), 0, 0])

# 服务 - 根据 vx, vy 和 omega 运动
def handle_vector_move(data):
	global vsp, node_name
	# 获取数据
	vx = data.vx
	vy = data.vy
	omega = data.omega
	vector_move(vx, vy, omega)
	return VectorMoveResponse(1)

# 服务 - 控制电机运动
def handle_motor_speed(data):
	global vsp, node_name
	# 获取数据
	sFrontLeft = data.sFrontLeft
	sFrontRight = data.sFrontRight
	sBack = data.sBack
	sbLeft = data.sbLeft
	sbRight = data.sbRight
	speed = [sFrontLeft, sFrontRight, sBack, sbLeft, sbRight]

	# 设置电机速度
	if set_five_motor_speed(speed):
		return MotorSpeedResponse(1)
	else:
		rospy.logerr("Set motor speed failed")
		return MotorSpeedResponse(0)

# 线程 - 用于获取串口数据
def read_data_thread():
	global vsp, node_name, delayTime_of_code_disk

	# 要求底层发送码盘数据
	pSendBuf = vsp.generate_sig_cmd(1, 0x38, 0x02, 0x08, 0x60, vsp.split_to_bytes([delayTime_of_code_disk]))
	if vsp.write_data(pSendBuf, "hex"):
		rospy.loginfo("Get data from code disk success")
	else:
		rospy.loginfo("Get data from code disk failed")

	# 关闭 IO 返回
	pSendBuf = vsp.generate_sig_cmd(1, 0x38, 0x02, 0x06, 0x60, [0, 0])
	if vsp.write_data(pSendBuf, "hex"):
		rospy.loginfo("close I/O data from code disk success")
	else:
		rospy.loginfo("close I/O data from code disk failed")

	# 关闭 AD 返回
	pSendBuf = vsp.generate_sig_cmd(1, 0x38, 0x02, 0x07, 0x60, [0, 0])
	if vsp.write_data(pSendBuf, "hex"):
		rospy.loginfo("close AD data from code disk success")
	else:
		rospy.loginfo("close AD data from code disk failed")

	# 接受和解析数据
	buffer_data = ""
	rate = rospy.Rate(20)
	while True:
		if vsp.ser != None and vsp.ser.isOpen():
			num_to_read = vsp.ser.inWaiting()
			if num_to_read > 0:
				# 读出数据并放在缓冲区中
				data = vsp.ser.read(num_to_read)
				buffer_data = buffer_data + data
				buffer_data_byte = [ord(i) for i in buffer_data]	# str 转换为 Byte 数组
				# 解析数据
				if len(buffer_data) >= 8:
					# 检查标志位
					if buffer_data_byte[0] == 0x55 and buffer_data_byte[1] == 0xaa:
						# 数据位长度
						data_length = buffer_data_byte[3]
						if len(buffer_data) >= data_length + 8:
							print "本机节点：", hex(buffer_data_byte[2])
							print "模块编码：", hex(buffer_data_byte[4])
							print "方法编码：", hex(buffer_data_byte[5])
							print ""
							# 取出数据部分
							data_zone = buffer_data_byte[7 : 7 + data_length]
							# 判断是否为码盘数据
							if buffer_data_byte[2] == 0x38 and buffer_data_byte[3] == 0x12 and buffer_data_byte[4] == 0x08 and buffer_data_byte[5] == 0x60 and buffer_data_byte[6] == 0:
								handle_codedisk_data(data_zone)
							buffer_data = buffer_data[8 + data_length:]		# 获取数据成功，清除
					else:
						rospy.logerr("%s : 标志位不正确", node_name)
						buffer_data = buffer_data[1:]	# 标志位不正确，去除第一位

# 处理码盘数据
def handle_codedisk_data(data):
	codedisk_position_of_M1 = (data[0] << 24) + (data[1] << 16) + (data[2] << 8) + data[3]
	codedisk_position_of_M2 = (data[6] << 24) + (data[7] << 16) + (data[8] << 8) + data[9]
	codedisk_position_of_M3 = (data[12] << 24) + (data[13] << 16) + (data[14] << 8) + data[15]
	pub_data = CodeDisk()
	pub_data.m1 = codedisk_position_of_M1
	pub_data.m2 = codedisk_position_of_M2
	pub_data.m3 = codedisk_position_of_M3
	pub_of_codedisk.publish(pub_data)

# 主函数
if __name__ == "__main__":
	rospy.init_node(node_name)	# 初始化节点

	# 打开串口
	if vsp.open_serial_port():
		rospy.loginfo("%s : Open serialport succeess", node_name)
	else:
		rospy.logerr("%s : Open serialport failed", node_name)
		exit(0)

	# 服务列表
	rospy.Service("/hc_cmd_interface/motor_speed", MotorSpeed, handle_motor_speed)		# 设置五个电机的速度（家庭组中后面两个电机无）
	rospy.Service("/hc_cmd_interface/vector_move", VectorMove, handle_vector_move)		# 根据vx, vy 和 omega 运动

	# vector_move(300, 300, 500)

	# 单开线程用于码盘数据的读取
	thread = threading.Thread(target = read_data_thread)
	thread.setDaemon(True)		# 防止子线程无法接收到终止信号
	thread.start()

	rospy.spin()				# 等待
