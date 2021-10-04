#!/usr/bin/env python2

import rospy
from std_msgs.msg import String, Float64
from sensor_msgs.msg import NavSatFix, Image
from mavros_msgs.srv import CommandTOL, SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist
from math import sqrt
from time import sleep
from cv_bridge import CvBridge, CvBridgeError
from cv_frame_detection import *
import tensorflow as tf
import pandas as pd
import cv2
import numpy as np
import ml_fxn as ml

model = tf.keras.models.load_model('modelv5.3')
maping = pd.read_csv('maping.txt', delimiter = ' ', header=None)
maping = np.array(maping)


class FLIGHT_CONTROLLER:

	def __init__(self):

		#DATA
		self.target = 'E'
		self.gps_lat = 0
		self.gps_long = 0

		self.curr_x = 0
		self.curr_y = 0
		self.curr_z = 0

		self.set_x = 0
		self.set_y = 0
		self.set_z = 0

		self.bridge = CvBridge()
		self.depth_bridge = CvBridge()

		self.delta = 0.05
		self.delta_z = 0.1
		self.waypoint_number = 0
		self.rgb_flag = 0
		self.depth_flag = 0
		self.frame_flag = 0
		self.distance = 0
		self.found_frame_flag = 0

		self.loc_tag=0

		#NODE
		rospy.init_node('iris_drone', anonymous = True)

		#SUBSCRIBERS
		self.gps_subscriber =  rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
		self.get_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_pose)
		self.get_rgb_image = rospy.Subscriber('/camera/rgb/image_raw', Image, self.get_rgb)
		self.get_depth_image = rospy.Subscriber('/camera/depth/image_raw', Image, self.get_depth)

		#PUBLISHERS
		self.publish_pose = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 10)

		#SERVICES
		self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
		self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
		self.flight_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

		rospy.loginfo('INIT')

	#MODE SETUP

	def toggle_arm(self, arm_bool):

		rospy.wait_for_service('/mavros/cmd/arming')

		try:
			self.arm_service(arm_bool)

		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: " %e)

	def takeoff(self, t_alt):

		self.gps_subscriber

		t_lat = self.gps_lat
		t_long = self.gps_long

		rospy.wait_for_service('/mavros/cmd/takeoff')

		try:
			self.takeoff_service(0.0,0,47.3977421,8.5455945,t_alt)
			rospy.loginfo('TAKEOFF')

		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: " %e)

	def land(self, l_alt):

		self.gps_subscriber

		l_lat = self.gps_lat
		l_long = self.gps_long

		rospy.wait_for_service('/mavros/cmd/land')

		try:
			self.land_service(0.0, 0.0, l_lat, l_long, l_alt)
			rospy.loginfo("LANDING")

		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: " %e)

	def set_offboard_mode(self):
		rate=rospy.Rate(20)
		#print('OFF')

		rospy.wait_for_service('/mavros/set_mode')
		PS = PoseStamped()

		PS.pose.position.x = 0
		PS.pose.position.y = 0
		PS.pose.position.z = 3.37



		for i in range(100):
			self.publish_pose.publish(PS)
			rate.sleep()
		try:
			self.flight_mode_service(0, 'OFFBOARD')
			rospy.loginfo('OFFBOARD')


		except rospy.ServiceException as e:
			rospy.loginfo("OFFBOARD Mode could not be set: " %e)

	#CALLBACKS

	def gps_callback(self, data):

		self.gps_lat = data.latitude
		self.gps_long = data.longitude

	def get_pose(self, location_data):

		self.curr_x = location_data.pose.position.x
		self.curr_y = location_data.pose.position.y
		self.curr_z = location_data.pose.position.z

	def get_rgb(self, rgb_data):

		if self.rgb_flag == 1:
			rospy.loginfo('image')
			rospy.loginfo("RGB IMAGE RECEIVED")
			cv2_img = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
			self.image = cv2_img
			rgb_filename = 'drone_img/rgb/rgb_camera_image'  + '.jpeg'
			cv2.imwrite(rgb_filename, cv2_img)
			self.rgb_flag = 0
			rospy.loginfo("RGB IMAGE SAVED: ")

	def click_rgb_image(self):
		rospy.loginfo('click')
		self.rgb_flag = 1
		self.get_rgb_image

	def click_depth_image(self):

		self.depth_flag = 1
		self.get_depth_image
    

	def get_cam(self):
        
		#print('1')
		if self.depth_flag == 1:
            
			
			rospy.loginfo("DEPTH IMAGE RECEIVED")
			
			#depth_data=depth_data*10.0
			cv2_img_depth =self.depth_bridge.imgmsg_to_cv2(self.depth87, "32FC1")
			#rospy.loginfo( self.depth_bridge.imgmsg_to_cv2(depth_data, "32FC1"))
			depth_filename = 'drone_img/depth/depth_camera_image.png'
			cv2.imwrite(depth_filename, cv2_img_depth)
			sleep(1)

			self.distance = frame_distance(depth_filename)
			print(self.distance)
			print("CAMERA CLICKED")

			if not self.distance == 1000:
				self.found_frame_flag = 1
				rospy.loginfo("found")
				self.navigate()
				#self.move_to_frame()

			else:
				self.found_frame_flag = 0
				rospy.loginfo("not found")
				self.navigate()

			self.depth_flag = 0
	

	def get_depth(self, depth_data):
		self.depth87=depth_data




	#PUBLISHERS

	def set_pose(self):                                     # Moves drone to given setpoint

		update_rate = rospy.Rate(20)
		PS = PoseStamped()

		PS.pose.position.x = self.set_x
		PS.pose.position.y = self.set_y
		PS.pose.position.z = self.set_z

		PS.pose.orientation.x = 0
		PS.pose.orientation.y = 0
		PS.pose.orientation.z = 0.707
		PS.pose.orientation.w = 0.707

		distance = sqrt((self.set_x - self.curr_x)**2 + (self.set_y - self.curr_y)**2 + (self.set_z - self.curr_z)**2)

		while (distance > self.delta): #and (abs(self.set_z - self.curr_z) > self.delta_z):

			self.publish_pose.publish(PS)
			self.get_pose_subscriber
			distance = sqrt((self.set_x - self.curr_x)**2 + (self.set_y - self.curr_y)**2 + (self.set_z - self.curr_z)**2)
			self.rgb_flag = 0
			self.depth_flag = 0
			update_rate.sleep()

		self.waypoint_number = self.waypoint_number + 1
		#self.depth_flag = 1
		rospy.loginfo('WAYPOINT REACHED: ' + str(self.waypoint_number))
	
	def move_to(self,x2,y2,z2):                             # function to efficiently reach given Setpoint
		
		
		a=self.curr_x
		b=self.curr_y
		c=self.curr_z

		self.set_waypoints((x2+a)/2,(y2+b)/2,(z2+c)/2)
		self.set_pose()
		self.set_waypoints(x2,y2,z2)
		self.set_pose()

		sleep(2)
		self.set_waypoints(x2,y2,z2)
		self.set_pose()

		#self.set_z=z2
		#self.set_pose()



	#MISSION CONTROL

	def set_waypoints(self, temp_x, temp_y, temp_z):            # sets waypoints

		self.set_x = temp_x
		self.set_y = temp_y
		self.set_z = temp_z

	

	def test_control(self):

		self.toggle_arm(True)
		self.takeoff(1.0)
		
		self.set_offboard_mode()


		self.move_to(0.7,1.2,3.3)
		sleep(2)
		self.depth_flag=1
		self.get_cam()




	def camera_test(self):
		self.toggle_arm(True)
		self.depth_flag = 1
		self.rgb_flag = 1
		self.get_depth_image
		self.get_rgb_image

	def cv_test(self):
		self.click_depth_image()
		#self.move_to_frame()

	def move_to_frame(self):

		if self.found_frame_flag == 1:
			self.set_x = self.distance+self.curr_x
			self.set_z=3.3
			self.set_pose()
			self.set_z=3.3
			self.set_y = self.set_y + 1
			self.set_pose()
			

		else:
			pass
	
	

	def activate_ml(self):
		self.move_to(0.1, 7.5, 2.6)
		self.click_rgb_image()
		sleep(20)
		try:
			rospy.loginfo('Neural Network activated')
			a = ml.main_prediction_fxn(self.image, model, maping, self.target)
		except:
			rospy.loginfo('Trying again')
			return self.activate_ml()
		rospy.loginfo('Landing position '+ str(a))
		if (a == -1 or a == 0 or a == 1):
			rospy.loginfo('Getting into landing position')
			if (a == -1):
				self.set_waypoints(-1.1,9.0,2.5)
			if (a == 0):
				self.set_waypoints(0.0,9.0,2.5)
			if (a == 1):
				self.set_waypoints(1.1,9.0,2.5)
			self.set_pose()
			rospy.loginfo('Landing against position '+ self.target)
			self.land(0.005)
			return 1
		else:
			return -1


	def navigate(self):                                    # control function to navigate drone and scan
		
		if self.found_frame_flag==1:
			print('Frame Detected')
		else:
			print('Frame NOT Detected')

		if (self.found_frame_flag==1):
			
			print('Moving inside Detected Rectangle')
			x_center=self.curr_x+self.distance

			self.set_x=x_center
			self.set_z=3.3
			self.set_pose()
			self.set_z=3.3
			self.set_y = self.set_y + 1
			self.set_pose()
			
			if abs(self.curr_x-1)<=0.2:
				self.loc_tag==1
			if abs(self.curr_x-(-1))<=0.2:
				self.loc_tag==-1
			if abs(self.curr_x-0)<=0.3:
				self.loc_tag==0
				
						

			self.move_to(self.loc_tag*0.7,self.set_y,self.set_z)
			
			
			sleep(2)
					
            
			self.frame_flag+=1
			self.found_frame_flag=0
			
			
				
		
		else:
			
			if abs(abs(self.curr_x)-1)<=0.4:
				print('Moving to Center')
				self.move_to(0,self.set_y,3.3)
				sleep(2)
				
					
			else:
				if self.loc_tag==-1:
					print('Moving to Right')
					self.move_to(0.7,self.set_y,3.3)
					sleep(2)
					
					self.loc_tag=1
					
				else:
					print('Moving to Left')
					self.move_to(-0.65,self.set_y,3.3)
					sleep(2)
					
					self.loc_tag=-1
					
			self.found_frame_flag=0
	
		
		print('Number of Frames Found :',self.frame_flag)
		if self.frame_flag==6:
			self.move_to(0.1,7.5,2.6)
			sleep(2)
			done = self.activate_ml()
			if done == -1:
				done = self.activate_ml()
			if done == -1:
				done = self.activate_ml()
			if done == -1:
				done = self.activate_ml()
				if done == -1:
					print('some internal error')
				self.land(0.005)
			sleep(5)
			print('\n\n------MISSION COMPLETED------')
		else:
			self.depth_flag=1
			self.get_cam()
		
				
				






if __name__ == '__main__':

	try:
		iris_controller = FLIGHT_CONTROLLER()
		#iris_controller.cv_test()
		iris_controller.test_control()

		rospy.spin()


	except rospy.ROSInterruptException:
		pass




