#!/usr/bin/env python

import rospy 
from geometry_msgs.msg import Pose, PoseStamped , TwistStamped
from sensor_msgs.msg import Image
from kuri_usar_msgs.msg import ImageWithCommand
from cv_bridge import CvBridge, CvBridgeError
import cv2

class Recorder: 
	
	def __init__(self):
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.image_ID=0
		self.image_data=[]
		self.cvImage=[]
		self.bridge=CvBridge()
		self.image2Analyze=[]
		self.pose=PoseStamped()
		self.vel=TwistStamped()
		self.key=TwistStamped()
		#self.command = ImageWithCommand()
		self.data_pub=rospy.Publisher("/ImageWithCommandTopic", ImageWithCommand, queue_size=10)
		self.image_pub	=rospy.Publisher("/image_output", Image, queue_size=10)
		self.camera_sub	=rospy.Subscriber("/uav_1/rgbd_cam/rgb/image_raw", Image, self.image_callback)
		self.odom_sub=rospy.Subscriber("/uav_1/mavros/local_position/pose", PoseStamped, self.pose_callback, queue_size=10)
		self.vel_sub=rospy.Subscriber("/uav_1/mavros/setpoint_velocity/cmd_vel", TwistStamped, self.vel_callback, queue_size=10)
		self.key_sub=rospy.Subscriber("/keyboardKey", TwistStamped, self.key_callback, queue_size=10)
	
	def image_callback(self, data):
		self.image_data= data
            		
	def pose_callback(self, pose):
		self.pose= pose
            		
	def vel_callback(self, vel):
		self.vel= vel
	def key_callback(self, key):
		self.key= key

	def record(self):
		data=ImageWithCommand()  
		data.header.stamp = rospy.Time.now() 
		data.im=self.image_data 
		data.im_ID=self.image_ID
		self.image_ID+=1
		data.pose=self.pose
		data.vel =self.vel
		data.key =self.key
		self.data_pub.publish(data)

def recorder_demo():
        rospy.Time(secs=0, nsecs=0) 
	rospy.init_node('recorder', anonymous=True)
	rate=rospy.Rate(10)
	recorder=Recorder()
	for i in range (5000):
		recorder.record()
		rospy.sleep(0.1)
	rospy.sleep(1)

if __name__ =='__main__':
	try: 
		recorder_demo() 
	except rospy.ROSInterruptException:
		pass
