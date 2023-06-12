#!/usr/bin/env python
'''
Bruno Sanchez Garcia				A01378960
Carlos Antonio Pazos Reyes 			A01378262
Manuel Agustin Diaz Vivanco			A01379673

Nodo para seguimiento de objetos usando una camara
'''

#!/usr/bin/env python
import sys
import rospy
import csv
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Vector3, Quaternion
from std_msgs.msg import Float32, UInt32, Int32, String
from sensor_msgs.msg import Joy, Imu, Range
from tf.transformations import euler_from_quaternion
from std_srvs.srv import *
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool,CommandBoolRequest, SetMode,SetModeRequest, CommandTOL,CommandTOLRequest


class ObjectFollower:

    def __init__(self):

        # start video capture with video dev0
        """
        self.video = cv2.VideoCapture(0)
        # capture dimensions
        self.video.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.video.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        """

        self.imageM = None
        self.image = None
        self.mask = None

        self.crossArea = 500
        self.xcenter = [0,0]
        self.blob_pos = [(320,240),(320/2,240/2),(0,0)]
        # cv bridge
        self.bridge = CvBridge()


        self.crossArea = 500
        self.xcenter = [0,0]
        self.real_x = 320/2
        self.real_y = 240/2

        # cv bridge
        self.bridge = CvBridge()

        # ROS node initialization and config
        rospy.init_node("drone_follow_obj")
        self.state = 0
        self.current_state = State()
        self.state_pub = rospy.Subscriber('/mavros/state',State,callback=self.state_cb)
        #self.image_pub = rospy.Publisher("/video_source/raw",Image,queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped',Twist,queue_size=1)
        self.vel = Twist()
        rospy.Subscriber("/front_cam/camera/image", Image, self.image_cb)
        self.image_pub_comp = rospy.Publisher("/video_source/raw",Image,queue_size=1)
        rospy.Subscriber('/sonar_height', Range, self.height_cb) #Cambiar por el topico del sensor es el que se parece a como esta aqui
        self.height = Range()
        self.height.range = 2.0
        rospy.on_shutdown(self.endCallback)

        self.max_x = 0.5
        self.max_y = 0.5
        self.max_z = 0.5
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0
        self.rate = rospy.Rate(10)
        self.detection = [False, False, False] # Detection vector that tells the camera detects one of the blobs (Red, Green, Purple)
        self.state = 3
        self.landing_area = 0
        self.red_table = []
        self.green_table = []
        self.t0 = rospy.Time.now().to_sec()
        self.current_time = rospy.Time.now().to_sec()


        # services request
        # set mode service client
        """
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode',SetMode)
        
        # arming service client
        rospy.wait_for_service('/mavros/cmd/arming')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
        # takeoff service client
        rospy.wait_for_service('/mavros/cmd/takeoff')
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        # services setup
        self.offb_set_mode = SetModeRequest()
        self.offb_set_mode.custom_mode = 'GUIDED'
        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True
        self.takeoff_cmd = CommandTOLRequest()
        self.takeoff_cmd.altitude = 1.5
        
        self.land_cmd = SetModeRequest()
        self.land_cmd.custom_mode = 'LAND'
        """
        

    def state_cb(self, msg):
        self.current_state = msg

    def image_cb(self, msg):
        image = msg
        self.image = self.bridge.imgmsg_to_cv2(image, "bgr8")

    def height_cb(self, msg):
        self.height = msg

    def endCallback(self):
        print("Shutting down")
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.cmd_vel_pub.publish(self.vel)

        fields = ("x", "z", "x_ref", "z_ref", "t")
        self.csv_writer(fields, self.green_table, "follow_control.csv")
        self.csv_writer(fields, self.red_table, "avoid_control.csv")
        print(csv)

    def csv_writer(self, fields, rows, name):

        with open(name, 'w') as f:
        
            # using csv.writer method from CSV package
            write = csv.writer(f)
        
            write.writerow(fields)
            write.writerows(rows)
        return "Success"

    def blob_detection(self, blob_color):
        
        #Transforming image to HSV
        hsvImage = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        hsvImage = cv2.GaussianBlur(hsvImage, (5, 5), 0)
        # Aplying inRange Function
        # Masks for red blobs (maskR1,R2), Green blobs (maskG) and Purple blobs (maskP)
        kernel =np.ones((5,5),np.uint8)
        maskR = cv2.inRange(hsvImage, np.array([5, 120, 0]), np.array([15, 255, 255]))
        maskG  = cv2.inRange(hsvImage, np.array([70, 80, 70]), np.array([90, 255, 255]))
        maskY  = cv2.inRange(hsvImage, np.array([15, 40, 90]), np.array([20, 180, 180]))
        # Defining the color for the rectangles according to the borders
        if blob_color == "Red":
            color = (0, 0, 255)
            self.mask = maskR
            color_idx = 0 # Color idx is a variable for idx the dection vector

        elif blob_color == "Green":
            color = (0, 255, 0)
            self.mask= maskG
            color_idx = 1

        elif blob_color == "Yellow":
            color = (255, 0, 0)
            self.mask = maskY
            color_idx = 2

        self.mask = cv2.erode(self.mask, kernel, iterations=1)
        self.imageM = cv2.bitwise_and(self.image, self.image, mask=self.mask)
        
        #self.imageR = cv2.dilate(self.imageR, kernel, iterations=1)
        
        
        contours, hierarchy = cv2.findContours(self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        #cont, contours, cont = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #detection = cv2.drawContours(image, contours, -1, (255, 0, 0), 15)
        try:
            i = max(contours, key=cv2.contourArea)
            boundRect = cv2.boundingRect(i)
            
            MC = cv2.moments(i)

            if MC["m00"] == 0:
                x = 0
                y = 0
            else:
                x = int(MC["m10"] / MC["m00"])
                y = int(MC["m01"] / MC["m00"])
                
            
            # Get the dimensions of the bounding rect:
            rectX = boundRect[0]
            rectY = boundRect[1]
            rectWidth = boundRect[2]
            rectHeight = boundRect[3]
            rectArea = rectWidth * rectHeight

            minArea = self.crossArea
            final_area = rectArea
            
            if rectArea > minArea:
                print(self.landing_area)
                if blob_color == "Yellow":
                    self.landing_area = rectArea
                    
                
                self.xcenter = [x,y]
                boundRect = cv2.boundingRect(i)
                
                halfArea = rectArea / 2
                # Store and set bounding rect:

                self.image = cv2.rectangle(self.image, (int(rectX), int(rectY)),(int(rectX + rectWidth), int(rectY + rectHeight)), color, 2)
                cv2.circle(self.image, (self.xcenter[0], self.xcenter[1]), 5, (0, 255,255), -1)
                self.blob_pos[color_idx] = (self.xcenter[0], self.xcenter[1])
                
                #print(self.real_x, self.real_y)
                self.detection[color_idx] = True # If it detects a blob bigger than the minumum area, changes the dection state for the current color
            else:
                self.detection[color_idx] = False

        except:
            self.detection[color_idx] = False
            #print("nothing")

    def drone_control(self):
        kpx, kpy, kpz = self.max_x/(self.image.shape[1]/2), 1, self.max_z/(self.image.shape[0]/2)

        # Because landing make everything to stop definetly, we only go to land state if the camera detected a purple blob 50 times 
        

        if self.detection[0]:
            # If it detects a red blob, it will avoid it for the left right or above depending on where is the object

            # If it detects a red blob, it will avoid it for the left right or above depending on where is the object
            x_pos = self.blob_pos[0][0] 
            z_pos = self.blob_pos[0][1]
            # Calculates the distances to left, right and up 
            distance_to_left = x_pos
            distance_to_right = self.image.shape[1] - x_pos
            distance_up =  z_pos
            distances = (distance_to_left, distance_to_right, distance_up)
            avoid_dir = distances.index(max(distances))
            print(distances)
            #print(distances)
            # Avoids the object where there is more free space (avoid_dir)
            if avoid_dir == 2:
                ref_z = self.image.shape[0]*0.9
                ref_x = x_pos
            else:
                ref_z = z_pos

                if avoid_dir == 0:
                    ref_x = self.image.shape[1]
                    
                elif avoid_dir == 1:
                    ref_x = 0.0

            self.vel_z = kpz*(ref_z - z_pos)
            self.vel_x = 0.5*kpx*(ref_x - x_pos)
            self.vel_y = self.max_y
            self.red_table.append((x_pos, z_pos, ref_x, ref_z, self.current_time))
            print("Avoiding")

        elif self.detection[1]:
            # If ti detects a green blob, it will follow it
            x_pos = self.blob_pos[1][0]
            z_pos = self.blob_pos[1][1]
            ref_x = self.image.shape[1]/2
            ref_z = self.image.shape[0]/2
            error_x = ref_x - x_pos
            error_z = ref_z - z_pos

            x_control = kpx*error_x
            z_control = kpz*error_z

            #print(x_control, z_control)
            self.vel_x = x_control
            self.vel_y = self.max_y
            self.vel_z = z_control

            self.green_table.append((x_pos, z_pos, ref_x, ref_z, self.current_time))
            print("Following")

        elif self.landing_area > 30000 :
            self.state = 4
        
        else:
            # If there is no blobs detected, the drone stays on hover
            print("No objects detected")
            self.vel_x = 0.0
            self.vel_y = self.max_y
            self.vel_z = kpz*(2.0 - self.height.range)
        
        print(self.vel_x, self.vel_y, self.vel_z)
        #print(self.detection)
        self.vel.linear.x = np.clip(self.vel_x, -self.max_x, self.max_x)
        self.vel.linear.y = np.clip(self.vel_y, -self.max_y, self.max_y)
        self.vel.linear.z = np.clip(self.vel_z, -self.max_z, self.max_z) 
        self.cmd_vel_pub.publish(self.vel)


    def main(self):
        
        try:
            """
            if not self.video.isOpened():
                print("Unable to load camera")
                return
            """
            print("Publishing webcam image ...")
            while not rospy.is_shutdown():
                # Reading the image and searching for blobs
                #ret, self.image= self.video.read()
                #print(self.image)
                if self.image is not None:
                    self.current_time = rospy.Time.now().to_sec() - self.t0
                    #print(self.current_time)
                    self.blob_detection("Red")
                    self.blob_detection("Green")
                    self.blob_detection("Yellow")

                    if self.state == 3:
                        #State 3, flying
                        self.drone_control()
                    
                    if self.state == 4:
                        # State 4, landing
                        print("Landing")

                        self.vel.linear.x = 0
                        self.vel.linear.y = 0
                        self.vel.linear.z = np.clip(-self.height.range*0.25, -self.max_z, self.max_z) 
                        self.cmd_vel_pub.publish(self.vel)

                        

                    comp_frame = cv2.resize(self.image, (75,75))
                    #self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.image,"bgr8"))
                    #Publishing rescaled image
                    self.image_pub_comp.publish(self.bridge.cv2_to_imgmsg(self.image, "bgr8"))
                self.rate.sleep()

        except rospy.ROSInterruptException:
            rospy.logerr("ROS Interrupt Exception.")

if __name__ == "__main__":
    if len(sys.argv) == 1:
        obj_fol = ObjectFollower()
        obj_fol.main()
    else:
        print("Incorrect Usage")

