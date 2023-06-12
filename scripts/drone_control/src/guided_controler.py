#!/usr/bin/env python
'''
Bruno Sanchez Garcia				A01378960
Carlos Antonio Pazos Reyes 			A01378262
Manuel Agustin Diaz Vivanco			A01379673

Nodo para hover del dron hector
'''

import rospy
from hector_uav_msgs.msg import Altimeter
from geometry_msgs.msg import Twist, Vector3, Quaternion
from std_msgs.msg import Float32, UInt32, Int32, String
from sensor_msgs.msg import Joy, Imu, Range
from tf.transformations import euler_from_quaternion
from hector_uav_msgs.srv import *
from std_srvs.srv import *

class SetPointControler():
    def __init__(self):
        rospy.init_node('guided_set_point_controler')
        t = 60.0
        self.rate = rospy.Rate(t)
        self.dt = 1.0/t
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        rospy.Subscriber('/sonar_height', Range, self.altitude_callback)
        rospy.Subscriber('/raw_imu',Imu,self.imu_callback)
        rospy.Subscriber('/mavros/setpoint_velocity/cmd_vel_unstamped',Twist, self.sp_vel_cb)
        self.set_point_vel = Twist()
        self.enable_motors = rospy.ServiceProxy('/enable_motors', EnableMotors)
        self.calibrate_imu = rospy.ServiceProxy('/raw_imu/calibrate', Empty)
        self.rate = rospy.Rate(5)
        self.vel = Twist()
        self.height = 0.0
        self.desired_alt = 2.0
        self.desired_x = 0.0
        self.desired_y = 0.0
        self.desired_yaw = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        self.yaw = 0.0
        self.max_height = False
        self.drone_orientation, self.drone_linear_acc = Quaternion(), Vector3()
        self.position_x, self.position_y, self.position_z, self.orientation = 0.0, 0.0, 0.0, 0.0
        self.enable_motors(True)
        self.calibrate_imu()
        rospy.on_shutdown(self.endCallback)

    def altitude_callback(self,msg):

        self.height = msg.range

    def sp_vel_cb(self,msg):

        self.set_point_vel = msg

    def imu_callback(self,msg):
        self.drone_orientation = msg.orientation
        self.drone_linear_acc = msg.linear_acceleration
        Q = [0.0,0.0,0.0,0.0]
        Q[0] = self.drone_orientation.x
        Q[1] = self.drone_orientation.y
        Q[2] = self.drone_orientation.z
        Q[3] = self.drone_orientation.w
        (roll,pitch,yaw) = euler_from_quaternion(Q)
        self.yaw = yaw
        # print("\n")
        # print(self.drone_linear_acc)
        # print("\n")

    def endCallback(self):
        print("Shutting down")
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.linear.z = 0
        self.cmd_vel_pub.publish(self.vel)


    def pose_controler(self):
        
        kpx, kpy, kpz, kpyw = 1.0, 1.0, 1.0, 0.5
        self.position_x += self.vel.linear.x * self.dt + (self.drone_linear_acc.x*self.dt**2)/2
        self.position_y += self.vel.linear.y * self.dt + (self.drone_linear_acc.y*self.dt**2)/2
        self.position_z = self.height
        self.desired_alt += self.set_point_vel.linear.z
        if self.desired_alt > 3.0:
            self.desired_alt = 2.9
        
        #error_x = self.desired_x - self.position_x
        #error_y = self.desired_y - self.position_y
        error_z = self.desired_alt - self.position_z
        error_yaw =self.desired_yaw - self.yaw

        self.vel.linear.y = self.set_point_vel.linear.x
        self.vel.linear.x = self.set_point_vel.linear.y
        self.vel.linear.z = kpz * error_z
        self.vel.angular.z = kpyw * error_yaw
        #print("yaw_vel: ", self.vel.angular.z, "error yaw: ", error_yaw)
        #print("x: ", self.position_x, "y: ", self.position_y, "z: ", self.position_z)
        #print("z: ", self.position_z, "desired z: ", self.desired_alt, self.set_point_vel.linear.z)
        self.cmd_vel_pub.publish(self.vel)
        
	
    def main(self):
        while not rospy.is_shutdown():
            try:
                self.pose_controler() 
            except Exception as e:
                print(e)
            self.rate.sleep()

if __name__ == '__main__':
	try:
		spc = SetPointControler()
		spc.main()
	except (rospy.ROSInterruptException, rospy.ROSException):
		print("topic was closed during publish")
