#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import sys
#rom featureless_and_high_dyamic_navigation.msg import Brick
from sensor_msgs.msg import Imu
from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_imu_v2 import BrickIMUV2    

class Brick():
    def __init__(self):
        rospy.init_node('brick_node', anonymous=True)
        rospy.loginfo("Start Initializing IMU")
        
        HOST = "localhost"
        PORT = 4223
        UID = "6dJCzE" # Change XXYYZZ to the UID of your IMU Brick 2.0
        self.deg_to_rad = np.pi / 180.0
        
        self.accx_list = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]    
        self.accy_list = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]    
        self.accz_list = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        
        self.eulx_list = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]    
        self.euly_list = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]    
        self.eulz_list = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        
        self.radx_list = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]    
        self.rady_list = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]    
        self.radz_list = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        
        self.q0 = 0.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        
        self.x_rad = 0.0
        self.y_rad = 0.0
        self.z_rad = 0.0
        
        self.x_acc = 0.0
        self.y_acc = 0.0
        self.z_acc = 0.0
        
        self.covariance_x_eul = 0.0
        self.covariance_y_eul = 0.0
        self.covariance_z_eul = 0.0
        
        self.covariance_x_acc = 0.0
        self.covariance_y_acc = 0.0
        self.covariance_z_acc = 0.0
        
        self.covariance_x_rad = 0.0
        self.covariance_y_rad = 0.0
        self.covariance_z_rad = 0.0
        
        ipcon = IPConnection() # Create IP connection
        self.imu = BrickIMUV2(UID, ipcon)
        ipcon.connect(HOST, PORT) # Connect to brickd
        
        self.pub = rospy.Publisher('imu', Imu, queue_size=5)
        self.brick_pub = Imu()
        
        self.imu.register_callback(self.imu.CALLBACK_QUATERNION, self.cb_quaternion)
        self.imu.register_callback(self.imu.CALLBACK_ORIENTATION, self.cb_euler)
        self.imu.register_callback(self.imu.CALLBACK_LINEAR_ACCELERATION, self.cb_linacc)
        self.imu.register_callback(self.imu.CALLBACK_ANGULAR_VELOCITY, self.cb_angvel)
        self.imu.set_quaternion_period(5)
        self.imu.set_orientation_period(5)
        self.imu.set_linear_acceleration_period(5)
        self.imu.set_angular_velocity_period(5)
        self.rate = rospy.Rate(50)
        rospy.loginfo("Initialization done")
        
        while not rospy.is_shutdown():
            self.publish()
            self.rate.sleep()
        if rospy.is_shutdown():
            rospy.loginfo("Shutting down brick node")
            ipcon.disconnect()
            rospy.signal_shutdown('brick_node')
            
        
    def cb_quaternion(self, w, x, y, z):
        self.q0 = x / 16383.0
        self.q1 = y / 16383.0
        self.q2 = z / 16383.0
        self.q3 = w / 16383.0
    
    def cb_euler(self, x, y, z):
        self.x_eul = x / 16.0
        self.y_eul = y / 16.0
        self.z_eul = z / 16.0
        
        self.eulx_list.append(self.x_eul)
        self.euly_list.append(self.y_eul)
        self.eulz_list.append(self.z_eul)
        
        self.eulx_list.pop(0)
        self.euly_list.pop(0)
        self.eulz_list.pop(0) 
        
        self.covariance_x_eul = np.cov(self.eulx_list)
        self.covariance_y_eul = np.cov(self.euly_list)
        self.covariance_z_eul = np.cov(self.eulz_list)
        
    def cb_linacc(self, x, y, z):
        self.x_acc = x / 100.0
        self.y_acc = y / 100.0
        self.z_acc = z / 100.0 
        
        self.accx_list.append(self.x_acc)
        self.accy_list.append(self.y_acc)
        self.accz_list.append(self.z_acc)
        
        self.accx_list.pop(0)
        self.accy_list.pop(0)
        self.accz_list.pop(0) 
        
        self.covariance_x_acc = np.cov(self.accx_list)
        self.covariance_y_acc = np.cov(self.accy_list)
        self.covariance_z_acc = np.cov(self.accz_list)
        
    def cb_angvel(self, x, y, z):
        self.x_rad = (x / 16.0) * self.deg_to_rad
        self.y_rad = (y / 16.0) * self.deg_to_rad
        self.z_rad = (z / 16.0) * self.deg_to_rad
        
        self.radx_list.append(self.x_rad)
        self.rady_list.append(self.y_rad)
        self.radz_list.append(self.z_rad)
        
        self.radx_list.pop(0)
        self.rady_list.pop(0)
        self.radz_list.pop(0) 
        
        self.covariance_x_rad = np.cov(self.radx_list)
        self.covariance_y_rad = np.cov(self.rady_list)
        self.covariance_z_rad = np.cov(self.radz_list)
    
    def publish(self):
        self.brick_pub.orientation.x = self.q0
        self.brick_pub.orientation.y = self.q1
        self.brick_pub.orientation.z = self.q2
        self.brick_pub.orientation.w = self.q3
        
        self.brick_pub.angular_velocity.x = self.x_rad 
        self.brick_pub.angular_velocity.y = self.y_rad
        self.brick_pub.angular_velocity.z = self.z_rad
        
        self.brick_pub.linear_acceleration.x = self.x_acc
        self.brick_pub.linear_acceleration.y = self.y_acc
        self.brick_pub.linear_acceleration.z = self.z_acc
        
        self.brick_pub.orientation_covariance = [self.covariance_x_eul, 0, 0,
                                                 0, self.covariance_y_eul, 0, 
                                                 0, 0, self.covariance_z_eul]
                                                    
        self.brick_pub.angular_velocity_covariance = [self.covariance_x_rad, 0, 0,
                                                      0, self.covariance_y_rad, 0, 
                                                      0, 0, self.covariance_z_rad]
                                                    
        self.brick_pub.linear_acceleration_covariance = [self.covariance_x_acc, 0, 0,
                                                         0, self.covariance_y_acc, 0, 
                                                         0, 0, self.covariance_z_acc]
        now = rospy.get_rostime()
        self.brick_pub.header.stamp.secs = now.secs
        self.brick_pub.header.stamp.nsecs = now.nsecs
        self.brick_pub.header.frame_id = "imu"
        
        self.pub.publish(self.brick_pub)
        
def main(args):
    brick = Brick()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown('brick_node')
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
