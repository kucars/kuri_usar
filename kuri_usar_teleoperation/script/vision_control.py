#!/usr/bin/env python

import rospy 
from geometry_msgs.msg import Pose, PoseStamped , TwistStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from keras.models import model_from_json
from keras.preprocessing import image

import numpy as np 
# Reading bag filename from command line or roslaunch parameter.
import os
import sys
import csv 
import time
import mavros
from math import *
from mavros.utils import *
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
import math
import tf

class VisionControl: 
	
	def __init__(self):
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.yaw = 0 
		self.roll = 0 
		self.pitch = 0 
		self.image_data=[]
		self.cvImage=[]
        	self.bridge = CvBridge()
		self.pose=PoseStamped()
		self.vel_msg = TwistStamped()
		self.camera_sub	=rospy.Subscriber("/uav_1/forward_cam/camera/image", Image, self.image_callback)
		self.odom_sub=rospy.Subscriber("/uav_1/mavros/local_position/pose", PoseStamped, self.pose_callback, queue_size=10)
		self.vel_pub=rospy.Publisher("/uav_1/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1000)
	
	def image_callback(self, data):
		self.image_data= data
            		
	def pose_callback(self, msg):
		self.pose= msg
		quaternion =( msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.roll = euler[0]
		self.pitch = euler[1]
		self.yaw = euler[2]
            	#tf::Quaternion q( msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
  		#tf::Matrix3x3(q).getRPY(self.roll, self.pitch, self.yaw);

	def image_to_feature_vector(self, image, size=(200, 200)):
		# resize the image to a fixed size, then flatten the image into
		# a list of raw pixel intensities
		return cv2.resize(image, size).flatten()

	def vision_control_demo(self) :
		#load the model 
		json_file = open('model.json', 'r')
		loaded_model_json = json_file.read()
		json_file.close()
		model = model_from_json(loaded_model_json)
		# load weights into new model
		model.load_weights("model.h5")
		print("Loaded model from disk")

		try:
		    while True:
			cvImage = self.bridge.imgmsg_to_cv2(self.image_data, "mono8")
			#test_img=cv2.cvtColor(cvImage, cv2.COLOR_BGR2GRAY)
			test_img_resize=cv2.resize(cvImage,(200,200))
			img_data = test_img_resize.astype('float32') #changed the type of data 
			img_data /= 255 # normalization 
			#print (img_data.shape)
			img_data= np.expand_dims(img_data, axis=0) 
			#print (img_data.shape)
			img_data= np.expand_dims(img_data, axis=4) 
			#print (img_data.shape)	
			preds = model.predict(img_data)
			#print "7" , preds
			movePrediction = model.predict_classes(img_data)
			print "MovePrediction" , movePrediction[0]
			
			
			if (movePrediction[0] == 0 ) :
				# mode forward 	
				print "Farward"	
				linear_x = 0.5 
				linear_y = 0 	
				self.vel_msg.twist.linear.z = 0
				self.vel_msg.twist.angular.z = 0
				self.vel_msg.twist.linear.x = linear_x*cos(self.yaw) - linear_y * sin(self.yaw)
				self.vel_msg.twist.linear.y = linear_x*sin (self.yaw) + linear_y * cos(self.yaw) 
				self.vel_pub.publish(self.vel_msg)	
			elif (movePrediction[0] == 1 ) :
				print "Turn"
				# turn left 
				angular_z=0.2625
		 		self.vel_msg.twist.linear.x = 0 
				self.vel_msg.twist.linear.y = 0
				self.vel_msg.twist.angular.z = angular_z
				self.vel_pub.publish(self.vel_msg)	
			elif (movePrediction[0] == 2 ):
				# turn right 
				angular_z = -1.57
		 		self.vel_msg.twist.linear.x = 0 
				self.vel_msg.twist.linear.y = 0
				self.vel_msg.twist.angular.z = angular_z
				self.vel_pub.publish(self.vel_msg)	
						

		except KeyboardInterrupt:
		    print('interrupted!')

			


if __name__ =='__main__':
	try: 
		rospy.Time(secs=0, nsecs=0) 
		rospy.init_node('visio_control', anonymous=True)
		rate=rospy.Rate(10)
		moveOption =  VisionControl()
		moveOption.vision_control_demo()

	except rospy.ROSInterruptException:
		pass

