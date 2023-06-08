#!/usr/bin/env python
'''
Bruno Sanchez Garcia				A01378960
Carlos Antonio Pazos Reyes 			A01378262
Manuel Agustin Diaz Vivanco			A01379673

Nodo para hover del dron hector
'''

import rospy
from geometry_msgs.msg import Twist, Vector3, Quaternion
from sensor_msgs.msg import Joy, Imu, Range
from hector_uav_msgs.srv import *
from std_srvs.srv import *

class HoverDrone():
	def __init__(self):
		rospy.init_node('hover_drone')
		t = 60.0
		self.rate = rospy.Rate(t)
		self.dt = 1.0/t
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
		rospy.Subscriber('/sonar_height', Range, self.altitude_callback)
		rospy.Subscriber('/raw_imu',Imu,self.imu_callback)
		rospy.Subscriber('/joy',Joy,self.controller_callback)
		self.enable_motors = rospy.ServiceProxy('/enable_motors', EnableMotors)
		self.calibrate_imu = rospy.ServiceProxy('/raw_imu/calibrate', Empty)
		self.rate = rospy.Rate(5)
		self.vel = Twist()
		self.controller_up, self.controller_down = 0.0, 0.0
		self.controller_roll, self.controller_pitch  = 0.0, 0.0
		self.controller_yawr, self.controller_yawl = 0.0, 0.0
		self.yaw = 0.0
		self.roll = 0.0
		self.pitch = 0.0
		self.height = 0.0

		self.desired_alt = 1.5
		self.k = 1.0
		self.drone_orientation, self.drone_linear_acc = Quaternion(), Vector3()
		self.position_x, self.position_y, self.position_z, self.orientation = 0.0, 0.0, 0.0, 0.0
		self.enable_motors(True)
		self.calibrate_imu()
		

	
	def controller_callback(self,msg):
		self.controller_up = msg.axes[5] # right trigger
		self.controller_down = msg.axes[2] # left trigger
		print("up: " + str(self.controller_up))
		print("down: " + str(self.controller_down))
		self.controller_yawl = msg.buttons[4] # rigth rt
		self.controller_yawr = msg.buttons[5] # left lt
		print("yaw right: " + str(self.controller_yawr))
		print("yaw left: " + str(self.controller_yawl))
		self.controller_roll = msg.axes[3] # right joystick ( left to right )
		self.controller_pitch = msg.axes[1] # left joystick ( up to down )
		print("roll: " + str(self.controller_roll))
		print("pitch: " + str(self.controller_pitch))


	def altitude_callback(self,msg):

		self.height = msg.range

	def imu_callback(self,msg):
		self.drone_orientation = msg.orientation
		self.drone_linear_acc = msg.linear_acceleration
		# print("\n")
		# print(self.drone_linear_acc)
		# print("\n")

	def hover(self):

		self.position_x += self.drone_linear_acc.x * self.dt + ((self.drone_linear_acc.x * (self.dt * self.dt)))
		#print("x: " + str(self.position_x))
		self.position_y += self.drone_linear_acc.y * self.dt + ((self.drone_linear_acc.y * (self.dt * self.dt)))
		#print("y: " + str(self.position_y))
		self.position_z = self.height
		#print("z: " + str(self.position_z))
		

		#conditions for LT & RT
		if self.controller_up < 0:
			self.desired_alt += abs(self.controller_up)
		elif self.controller_down < 0:
			self.desired_alt += self.controller_down
		#conditions for LB & RB
		if self.controller_yawl > 0:
			self.yaw = 1.5
		elif self.controller_yawr > 0:
			self.yaw = -1.5
		elif self.controller_yawr == 0 and self.controller_yawl == 0:
			self.yaw = 0.0



		#print(self.desired_alt)
		err = self.desired_alt - self.height
		self.vel.linear.z = err * self.k
		self.vel.angular.z = self.yaw
		# joysticks have ranges from -1 to 1, so it can be sended directly to de /cmd_vel topic 
		self.vel.linear.y = self.controller_roll * 2 #right joystick
		self.vel.linear.x = self.controller_pitch * 2 #left joystick
		self.cmd_vel_pub.publish(self.vel)
	
	def main(self):
		while not rospy.is_shutdown():
			try:
				print("hover and joy control enabled")
				self.hover() 
			except Exception as e:
				print(e)
			self.rate.sleep()

if __name__ == '__main__':
	try:
		hd = HoverDrone()
		hd.main()
	except (rospy.ROSInterruptException, rospy.ROSException):
		print("topic was closed during publish")
