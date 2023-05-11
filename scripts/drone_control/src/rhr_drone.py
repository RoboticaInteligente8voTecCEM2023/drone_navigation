#!/usr/bin/env python
import sys
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from hector_uav_msgs.srv import *
from std_srvs.srv import *
from hector_uav_msgs.msg import Altimeter
from sensor_msgs.msg import Range
import follow_wall

class RightHandRuleController:
    def __init__(self, wall_dist=0.9, w_max = 0.5, v_max=0.2 ):
        """
        Arguments
        --------------
        wall_dist  : float
           Desired distance to wall
        w_max  : float
           Max angular velocity
        v_max  : float
           Max linear velocity
        """
        
        self.scan_listener = rospy.Subscriber('/laser/scan', LaserScan,
                                              self.scan_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel' , Twist, queue_size=1 )
        rospy.Subscriber('/sonar_height', Range, self.altitude_callback)
        self.rate = rospy.Rate(60.0)
        self.wall_dist = wall_dist
        self.w_max = w_max
        self.v_max = v_max
        self.scan = None

        self.k_h = 2
        self.desired_alt = 1
        self.height = 0.0

        self.enable_motors = rospy.ServiceProxy('/enable_motors', EnableMotors)
        self.calibrate_imu = rospy.ServiceProxy('/raw_imu/calibrate', Empty)
        self.enable_motors(True)


    def altitude_callback(self,msg):
        self.height = msg.range



    def scan_callback(self, msg):
        """Called when a new scan is available from the lidar. """
        self.scan = msg



    def follow_right_hand_wall(self):
        found_wall = False
        print("Enter right hand rule")
        while not rospy.is_shutdown():

            if self.scan is not None:
                #--------------------------------------------------------------
                # Your code here
                # 1) Check distance to wall/obstacle in front and to the right.
                # 2) Based on the information in 1) calculate a suitable angular
                #    velocity, w.
                # 3) Publish the Twist message to steer the puzzlebot.

                distance_ahead_left = follow_wall.get_distance_in_sector(self.scan, # scan from front to 15deg to the left
                                                                 -10*np.pi/180,
                                                                 0) # 15 deg left
                distance_ahead_right = follow_wall.get_distance_in_sector(self.scan,    # scan from front to 15 deg to the right
                                                                  0,
                                                                  10*np.pi/180) # 15 deg right
        
                distance_ahead = (distance_ahead_left + distance_ahead_right) / 2.0 # 30 deg front span


                distance_to_right = follow_wall.get_distance_in_sector(self.scan,
                                                           np.pi / 2 - 15 * np.pi / 180,
                                                           np.pi / 2 ,
                                                           )
                

                #--------------------------------------------------------------
                #--------------------------------------------------------------
                error = -follow_wall.find_wall_direction('right',self.scan)
                error_dist_side = self.wall_dist - distance_to_right
                error_dist_front = - self.wall_dist + distance_ahead
                msg = Twist()
                v = self.v_max
                k_ang = 0.9
                k_dist = 0.75

                yaw =  (error* k_ang) 
                linear_y = (error_dist_side * k_dist)
                linear_x = (error_dist_front * k_dist)

                if distance_ahead <= self.wall_dist * 1.5:
                    yaw = self.w_max

                
                print( "error: " + str("%.2f"%error), "distance to right: " +  "%.2f"%distance_to_right, "distance ahead " + "%.2f"%distance_ahead)
                msg.angular.z =  np.clip(yaw, -self.w_max,self.w_max )
                msg.linear.x = np.clip(linear_x, -self.v_max, self.v_max)
                msg.linear.y = np.clip(linear_y, -self.v_max, self.v_max)

                #HOVER
                print("z: " + str(self.height))
                print("\n")
                err = self.desired_alt - self.height
                msg.linear.z = err * self.k_h
                self.vel_pub.publish(msg)

            self.rate.sleep()




if __name__ == '__main__':
    if len(sys.argv) > 1:
        if sys.argv[1] == "--test":
            import doctest
            doctest.testmod()
            sys.exit(0)

    rospy.init_node('Follow_right_hand_wall')
    rhw = RightHandRuleController()
    rhw.follow_right_hand_wall()
