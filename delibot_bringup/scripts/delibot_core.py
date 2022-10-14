#!/usr/bin/env python

import serial
import threading
import time
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class serialRobot:

	def __init__(self, port, baud):
		self.port = port
		self.baud = baud

		self.packet = [0,0,0,0,0,0]
		self.info = [0,0,0,0,0]
		self.memory = [0,0,0,0,0]

		self.array = ""

		self.write_thread = None
		self.read_thread = None
		self.read_error = 0

		rospy.init_node('delibot_core')
		rospy.Subscriber('cmd_vel', Twist, self.callback)
		self.odom_pub =  rospy.Publisher("odom", Odometry, queue_size=500)
		self.odom_broadcaster = tf.TransformBroadcaster()
		self.odom = Odometry()
		self.pub_rate = rospy.Rate(30)
		self.write_rate = rospy.Rate(20)
		
		while True:
			try:
				self.ser = serial.Serial(self.port, self.baud, timeout = 3)
			except serial.SerialException:
				print('disconnect')
				continue
			else:
				print('connect')
				break


	def callback(self, data):
		self.array = 'a'+str(data.linear.x)+'b'+str(data.angular.z)+'c\0'


	def write(self):
		while not rospy.is_shutdown():
			try:
				self.ser.write(self.array.encode())
				self.write_rate.sleep()
			except Exception:
				pass

	def read(self):
		while not rospy.is_shutdown():
			try:
				if self.ser.readable():
					self.ser.flushInput()
					arr = self.ser.readline()

				if arr[0] == 'a':
					for i in range(0, 6):
						self.packet[i] = arr.find(chr(97+i))
	
					for i in range(0, 5):
						self.info[i] = float(arr[self.packet[i]+1:self.packet[i+1]])
						self.memory = self.info
						self.read_error = 0
					print(self.info)

			except Exception:
				print("read Error")
				print(arr)
				self.array = 'a'+str(0.00)+'b'+str(0.00)+'c\0'
				self.ser.write(self.array.encode())
				self.ser.close()
			
				while True:
					try:
						self.ser = serial.Serial(self.port, self.baud, timeout = 3)
			
					except serial.SerialException:
						print("serial error")
						continue

					else:
						print("connect")
						break

	def run(self):
			self.write_thread = threading.Thread(target=self.write)
			self.write_thread.daemon = True
			self.read_thread = threading.Thread(target=self.read)
			self.read_thread.daemon = True

			self.write_thread.start()
			self.read_thread.start()

			while not rospy.is_shutdown():
				try:
					current_time = rospy.Time.now()
					odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.info[2])
					self.odom_broadcaster.sendTransform((self.info[0], self.info[1], 0.), odom_quat, current_time, "base_footprint", "odom")
					self.odom.header.stamp = current_time
					self.odom.header.frame_id = "odom"
					self.odom.pose.pose = Pose(Point(self.info[0], self.info[1], 0.), Quaternion(*odom_quat))
					self.odom.child_frame_id = "base_footprint"
					self.odom.twist.twist = Twist(Vector3(self.info[3], 0, 0), Vector3(0, 0, self.info[4]))
					self.odom_pub.publish(self.odom)
					self.pub_rate.sleep()

				except Exception:
					print("raise Error")
					self.ser.close()
			
					while True:
						try:
							self.ser = serial.Serial(self.port, self.baud, timeout = 3)
			
						except serial.SerialException:
							print("serial error")
							continue

						else:
							print("connect")
							break

	

if __name__ == '__main__':
	port_name = rospy.get_param('~port','/dev/ttyUSB0')
	baud = int(rospy.get_param('~baud','19200'))

	robot = serialRobot(port_name , baud)
	robot.run()