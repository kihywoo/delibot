#!/usr/bin/env python

import serial
import threading
import time
import rospy
from std_msgs.msg import Bool

receivePacket = ['a', 'b']
sendPacket = ['a', 'b', 'c', 'd']

class serialRobot:

	def __init__(self, port, baud):
		self.port = port
		self.baud = baud

		self.read_thread = None
		self.response = ""

		self.write_thread = None
		self.read_thread = None

		rospy.init_node('arm_core')
		rospy.Subscriber("delibot_state", Bool, self.callback)
		rospy.Subscriber("cali", Bool, self.cali)
		self.armState = rospy.Publisher('arm_state', Bool, queue_size=10)
		self.pub_rate = rospy.Rate(20)
		
		while True:
			try:
				self.ser = serial.Serial(self.port, self.baud, timeout = 3)
			except serial.SerialException:
				print('disconnect')
				continue
			else:
				print('connect')
				break

	def cali(self, data):
		if data.data:
			self.ser.write(sendPacket[2].encode())
			print("go to origin")
		if not data.data:
			self.ser.write(sendPacket[3].encode())
			print("go to ready")

	def callback(self, state):
		if state.data:
			try:
				self.ser.write(sendPacket[0].encode())
				#time.sleep(1.1)
				print("put up parcel")
			except Exception:
				print("write error")
		elif not state.data:
			try:
				self.ser.write(sendPacket[1].encode())
				print("put down parcel")
			except Exception:
				print("write error")


	def read(self):
			while not rospy.is_shutdown():
				try:
					if self.ser.readable():
						self.response = self.ser.readline()
				except Exception:
					print("error")

	def run(self):
			
			self.read_thread = threading.Thread(target=self.read)
			self.read_thread.daemon = True
			self.read_thread.start()

			while not rospy.is_shutdown():
				try:
					if self.response[0] == receivePacket[0]:
						print("complete put up")
						self.response=""
						self.armState.publish(True)
						self.pub_rate.sleep()
					elif self.response[0] == receivePacket[1]:
						print("complete put down")
						self.response=""
						self.armState.publish(False)
						self.pub_rate.sleep()
					
				except Exception:
					pass

	

if __name__ == '__main__':
	port_name = rospy.get_param('~port','/dev/ttyARM')
	baud = int(rospy.get_param('~baud','19200'))

	robot = serialRobot(port_name , baud)
	robot.run()
