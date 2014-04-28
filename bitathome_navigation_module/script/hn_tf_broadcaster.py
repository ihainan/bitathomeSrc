#!/usr/bin/env python
#-*-encoding:utf-8-*-
# Filename : hn_tf_broadcaster.py
# Author : ihainan
# Created Date : 2014/04/22 22:38
# Descriptiont : 广播 tf 数据

import roslib
import rospy
import tf

# 全局变量
node_name = "hn_tf_broadcaster"							# 节点名字

# 主函数
if __name__ == "__main__":
	rospy.init_node(node_name)	# 初始化节点
	
	# 广播 tf
	r = rospy.Rate(100)
	br = tf.TransformBroadcaster()
	while True:
		br.sendTransform((0, 0, 0.3), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "base_link", "base_laser")
		r.sleep()

	rospy.spin()				# 等待
