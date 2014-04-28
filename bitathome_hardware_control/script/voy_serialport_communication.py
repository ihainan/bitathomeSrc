#!/usr/bin/env python
#-*-encoding:utf-8-*-
# Filename : voy_serialport_communication.py
# Author : ihainan
# Created Date : 2014/04/14 10:00
# Descriptiont : 旅行者三号串口通信类

# 这是一行注释
import serial
import threading

class VoySerialPort():
	# 构造函数，初始化串口、波特率、字节长度等
	def __init__(self, serialPort, baudrate, bytesize):
		self.is_debug = False					# 调试模式
		self.serialPort = serialPort			# 串口号
		self.baudrate = baudrate				# 波特率
		self.bytesize = bytesize				# 字节长度
		self.ser = None							# 串口对象

		# 单开线程用于数据的读取
		# self.read_data_buffer = ""				# 读取数据缓冲区
		# self.thread = threading.Thread(target = self.read_data_thread)
		# self.thread.setDaemon(True)				# 防止子线程无法接收到终止信号
		# self.thread.start()
	
	# 数据读取线程
	'''
	def read_data_thread(self):
		while True:
			if self.ser != None and self.ser.isOpen():
				# 检查是否有可以读的数据
				num_to_read = self.ser.inWaiting()
				if num_to_read > 0:
					# 读取数据并存储到缓冲区中
					data = self.ser.read(num_to_read)
					self.read_data_buffer = self.read_data_buffer + data
					# 缓冲区满，清空
					if len(self.read_data_buffer) > 10000:
						self.read_data_buffer = ""
	'''

	# 打开串口
	def open_serial_port(self):
		# 如果串口已经打开，关闭之
		if self.ser != None and self.ser.isOpen():
				self.ser.close()
		# 检查错误	
		try:
			self.ser = serial.Serial(self.serialPort, self.baudrate, self.bytesize)
		except:
			return False
		return True
	
	# 读取特定长度数据
	def read_data(self, data_length):
		# 串口尚未打开，则打开串口
		if self.ser == None or not self.ser.isOpen():
				if not self.open_serial_port():
						return None
		# 读取数据
		data_read = self.ser.read(data_length)
		return data_read
	

	# 往串口中写入数据
	def write_data(self, data, type = "ascii"):
		# 串口尚未打开，则打开串口
		if self.ser == None or not self.ser.isOpen():
				if not self.open_serial_port():
						return False

		# 十六进制或者字符串方式写入数据
		if type == "hex":
			write_buffer = bytearray([i for i in data])
		else:
			write_buffer = data

		# 输出写入信息
		if self.is_debug:
			print "写入数据 ："
			for i in write_buffer:
				print hex(i),
			print ""

		self.ser.write(write_buffer)
		return True
	
	# 生成写入数据
	def generate_sig_cmd(self, index, inID, inLen, inMode, inMethod, data):
		pSendBuf = []
		pSendBuf.append(0x55)
		pSendBuf.append(0xaa)
		pSendBuf.append(inID)
		pSendBuf.append(inLen)
		pSendBuf.append(inMode)
		pSendBuf.append(inMethod)
		if inLen > 0 and data != None:
			pSendBuf.extend(data)
		pSendBuf.append(self.cal_sum(pSendBuf))
		return pSendBuf

	# 计算校验码
	def cal_sum(self, sendBuf):
		sum = 0
		for item in sendBuf:
			sum = sum + item
		sum = sum & 0xff
		return sum

	# short 数组转换为 byte 数组
	def split_to_bytes(self, data):
		speedBuf = []
		for d in data:
			speedBuf.append((d & 0xff00) >> 8)
			speedBuf.append(d & 0xff)
		return speedBuf
