#!/usr/bin/env python
# vim:set ts=4 sw=4 et:
#
# Copyright 2015 UAVenture AG.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
# Updated: Tarek Taha : tarek.taha@kustar.ac.ae, Vladimir Ermakov
#    - Changed topic names after re-factoring : https://github.com/mavlink/mavros/issues/233
#    - Use mavros.setpoint module for topics

import rospy
import thread
import threading
import time
import mavros

from math import *
from mavros.utils import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler , euler_from_quaternion
from geometry_msgs.msg import Pose , PoseStamped

class SetpointPosition:
    """
    This class sends position targets to FCU's position controller
    """
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0  
        self.pitch = 0
        self.yaw = 0  
        self.quaternion = quaternion_from_euler(0, 0, 0)
        self.currentPoseX = 0 
        self.currentPoseY = 0
        self.currentPoseZ = 0
        self.currentYaw = 0 
        self.currentRoll = 0
        self.currentPitch = 0
        self.currentquaternion = quaternion_from_euler(0, 0, 0)

        # publisher for mavros/setpoint_position/local
        self.pub = rospy.Publisher('/uav_1/mavros/setpoint_position/local',PoseStamped, queue_size=10)

        # subscriber for mavros/local_position/pose
        self.sub = rospy.Subscriber("/uav_1/mavros/local_position/pose", PoseStamped, self.poseCallback)

        try:
            thread.start_new_thread(self.navigate, ())
        except:
            fault("Error: Unable to start thread")

        # TODO(simon): Clean this up.
        self.done = False
        self.done_evt = threading.Event()

    def navigate(self):
        rospy.loginfo("Navigate")
        rate = rospy.Rate(10)   # 10hz

        msg = PoseStamped()
        while not rospy.is_shutdown():
            msg.pose.position.x = self.x
            msg.pose.position.y = self.y
            msg.pose.position.z = self.z
            # convert from degree to R
            yaw_r = radians(self.yaw)
            self.quaternion = quaternion_from_euler(0, 0, yaw_r)

            #quaternion = tf.transformations.quaternion_from_euler(self.roll, self.pitch, yaw_r)
            #msg.pose.orientation = SP.Quaternion(*quaternion)
            msg.pose.orientation.x = self.quaternion[0]
            msg.pose.orientation.y = -self.quaternion[1]
            msg.pose.orientation.z = -self.quaternion[2]
            msg.pose.orientation.w = -self.quaternion[3]
            msg.header.frame_id="base_footprint"
            msg.header.stamp = rospy.Time.now()
            self.pub.publish(msg)
            rate.sleep()

    def setPose(self, x, y, z,roll,pitch,yaw, delay=0, wait=True):
        self.done = False
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll  
        self.pitch = pitch
        self.yaw = yaw 
        #print "Desired YAW" , self.yaw 

        if wait:
            rate = rospy.Rate(5)
            while not self.done and not rospy.is_shutdown():
                rate.sleep()
        time.sleep(delay)
        
    def takeoff(self, z, delay=0, wait=True):
        diff = z - self.currentPoseZ
        while not abs(diff)<0.2:
            diff = z - self.currentPoseZ
            print diff
            if diff>0:
                self.setPose(self.currentPoseX,self.currentPoseY,self.currentPoseZ + 1,0,0,90,2)
            else:
                self.setPose(self.currentPoseX,self.currentPoseY,self.currentPoseZ - 1,0,0,90,2)
    
    def land(self, delay=0, wait=True):
        altitude = self.currentPoseZ
        while altitude > 0.2:
            altitude = self.currentPoseZ
            self.setPose(self.currentPoseX,self.currentPoseY,self.currentPoseZ - 1,0,0,0,2)
    
    def poseCallback(self, topic):
        def is_near(msg, x, y):
            rospy.logdebug("Position %s: local: %d, target: %d, abs diff: %d",
                           msg, x, y, abs(x - y))
            return abs(x - y) < 0.1

        self.currentPoseX = topic.pose.position.x 
        self.currentPoseY = topic.pose.position.y
        self.currentPoseZ = topic.pose.position.z
        self.currentquaternion = (topic.pose.orientation.x,topic.pose.orientation.y,topic.pose.orientation.z,topic.pose.orientation.w)
        euler = euler_from_quaternion(self.currentquaternion)
        self.currentYaw = euler[2]
        #print "desired Yaw in radian " , self.yaw , "Orenitation z " , self.quaternion[2] , " ", self.quaternion[3], " desired Orentation " ,  self.currentquaternion[2] , "  " , self.currentquaternion[3] 
        #print "Desired Yaw : " , self.yaw ,"  " , "Actual yaw : " ,self.currentYaw , " Diff : " ,  self.yaw - self.currentYaw
        self.currentRoll = euler[0]
        self.currentPitch = euler[1]

        if is_near('X'  , topic.pose.position.x, self.x) and \
           is_near('Y'  , topic.pose.position.y, self.y) and \
           is_near('Z'  , topic.pose.position.z, self.z):
            self.done = True
            self.done_evt.set()


def setpoint_demo():
    rospy.init_node('setpoint_position_demo_1')
    #mavros.set_namespace()  # initialize mavros module with default namespace
    mavros.set_namespace('/uav_1/mavros')    
    rate = rospy.Rate(10)

    setpoint = SetpointPosition()
    
    time.sleep(1)
    
    rospy.loginfo("Climb")
    setpoint.takeoff(1.0)
   # setpoint.navigate()
    rospy.loginfo("Moving to Pose 1")
    setpoint.setPose(-9.74000263214,-38.3300018311,1,0,0,90,3)  
    rospy.loginfo("Moving to Pose 2")
    setpoint.setPose(-9.74000263214,-23.8746204376,1,0,0,90,3)  
    rospy.loginfo("Moving to Pose 3")
    setpoint.setPose(-9.74000263214,-18.2734966278,1,0,0,90,3)  
    rospy.loginfo("Moving to Pose 4")
    setpoint.setPose(-9.74000263214,-17.0077152252,1,0,0,90,3)  
    rospy.loginfo("Moving to Pose 5 ")
    setpoint.setPose(-9.74000263214,-16.5862216949,1,0,0,0,3)
    rospy.loginfo("Moving to Pose 5 turn ")
    setpoint.setPose(-9.74000263214,-16.5862216949,1,0,0,75,3)
    setpoint.setPose(-9.74000263214,-16.5862216949,1,0,0,60,3)
    setpoint.setPose(-9.74000263214,-16.5862216949,1,0,0,45,3)
    setpoint.setPose(-9.74000263214,-16.5862216949,1,0,0,30,3)
    setpoint.setPose(-9.74000263214,-16.5862216949,1,0,0,15,3)
    setpoint.setPose(-9.74000263214,-16.5862216949,1,0,0,0,3)
    rospy.loginfo("Moving to Pose 6 ")
    setpoint.setPose(-8.49420833588,-16.6697425842,1,0,0,0,3)
    rospy.loginfo("Moving to Pose 7 ")
    setpoint.setPose(-6.86303520203,-16.6697425842,1,0,0,0,3)
    rospy.loginfo("Moving to Pose 8 ")
    setpoint.setPose(-5.39108419418,-16.3141021729,1,0,0,0,3)
    rospy.loginfo("Moving to Pose 9 ")
    setpoint.setPose(-3.09930682182,-16.3141021729,1,0,0,0,3)
    rospy.loginfo("Moving to Pose 9 turn ")
    setpoint.setPose(-3.09930682182,-16.3141021729,1,0,0,15,3)
    setpoint.setPose(-3.09930682182,-16.3141021729,1,0,0,30,3)
    setpoint.setPose(-3.09930682182,-16.3141021729,1,0,0,45,3)
    setpoint.setPose(-3.09930682182,-16.3141021729,1,0,0,60,3)
    setpoint.setPose(-3.09930682182,-16.3141021729,1,0,0,75,3)
    setpoint.setPose(-3.09930682182,-16.3141021729,1,0,0,90,3)
    rospy.loginfo("Moving to Pose 10 ")
    setpoint.setPose(-3.03120684624,-15.2872591019,1,0,0,90,3)
    rospy.loginfo("Moving to Pose 11 ")
    setpoint.setPose(-3.03120684624,-13.4653873444,1,0,0,90,3)
    rospy.loginfo("Moving to Pose 12 ")
    setpoint.setPose(-3.03120684624,-11.8298950195,1,0,0,90,3)
    rospy.loginfo("Moving to Pose 12 trun ")
    setpoint.setPose(-3.03120684624,-11.8298950195,1,0,0,105,3)
    setpoint.setPose(-3.03120684624,-11.8298950195,1,0,0,120,3)
    setpoint.setPose(-3.03120684624,-11.8298950195,1,0,0,135,3)
    setpoint.setPose(-3.03120684624,-11.8298950195,1,0,0,150,3)
    setpoint.setPose(-3.03120684624,-11.8298950195,1,0,0,165,3)
    setpoint.setPose(-3.03120684624,-11.8298950195,1,0,0,180,3)
    rospy.loginfo("Moving to Pose 13 ")
    setpoint.setPose(-4.04095697403,-11.8370599747,1,0,0,180,3)
    rospy.loginfo("Moving to Pose 14 ")
    setpoint.setPose(-5.09890079498,-11.8370599747,1,0,0,180,3)
    rospy.loginfo("Moving to Pose 15 ")
    setpoint.setPose(-6.50557994843,-11.8370599747,1,0,0,180,3)
    rospy.loginfo("Moving to Pose 16 ")
    setpoint.setPose(-7.92576503754,-11.8370599747,1,0,0,180,3)
    rospy.loginfo("Moving to Pose 17 ")
    setpoint.setPose(-10.1049051285,-11.8370599747,1,0,0,180,3)
    rospy.loginfo("Moving to Pose 17 turn 1 ")
    setpoint.setPose(-10.1049051285,-11.8370599747,1,0,0,165,3)
    setpoint.setPose(-10.1049051285,-11.8370599747,1,0,0,150,3)
    setpoint.setPose(-10.1049051285,-11.8370599747,1,0,0,135,3)
    setpoint.setPose(-10.1049051285,-11.8370599747,1,0,0,120,3)
    setpoint.setPose(-10.1049051285,-11.8370599747,1,0,0,105,3)
    setpoint.setPose(-10.1049051285,-11.8370599747,1,0,0,90,3)
    rospy.loginfo("Moving to Pose 18 ")
    setpoint.setPose(-10.1092662811,-9.93856239319,1,0,0,90,3)
    rospy.loginfo("Moving to Pose 19 ")
    setpoint.setPose(-10.1092662811,-8.40193939209,1,0,0,90,3)
    rospy.loginfo("Moving to Pose 20 ")
    setpoint.setPose(-10.1092662811,-5.65100479126,1,0,0,90,3)
    rospy.loginfo("Moving to Pose 21 ")
    setpoint.setPose(-10.1092662811,-1.43230497837,1,0,0,90,3)
    rospy.loginfo("Moving to Pose 22 ")
    setpoint.setPose(-10.1092662811,1.42377030849,1,0,0,90,3)
    rospy.loginfo("Moving to Pose 22 ")
    setpoint.setPose(-10.1092662811,3.14487552643,1,0,0,90,3)
    rospy.loginfo("Moving to Pose 22 ")
    setpoint.setPose(-10.1092662811,3.58203005791,1,0,0,90,3)
    rospy.loginfo("Moving to Pose 22 turn ")
    setpoint.setPose(-10.1092662811,3.58203005791,1,0,0,75,3)
    setpoint.setPose(-10.1092662811,3.58203005791,1,0,0,60,3)
    setpoint.setPose(-10.1092662811,3.58203005791,1,0,0,45,3)
    setpoint.setPose(-10.1092662811,3.58203005791,1,0,0,30,3)
    setpoint.setPose(-10.1092662811,3.58203005791,1,0,0,15,3)
    setpoint.setPose(-10.1092662811,3.58203005791,1,0,0,0,3)
    rospy.loginfo("Moving to Pose 23  ")
    setpoint.setPose(-7.97584056854,3.58203005791,1,0,0,0,3)


    rospy.loginfo("Landing")
    setpoint.land()

    rospy.loginfo("Bye!")


if __name__ == '__main__':
    try:
        setpoint_demo()
    except rospy.ROSInterruptException:
        pass
