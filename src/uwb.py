#!/usr/bin/env python

from pypozyx import (PozyxSerial, POZYX_RANGE_PROTOCOL_FAST, POZYX_RANGE_PROTOCOL_PRECISION,
                        SingleRegister, DeviceRange, POZYX_SUCCESS, POZYX_FAILURE, get_first_pozyx_serial_port, get_serial_ports)
from filterpy.kalman import KalmanFilter
from geometry_msgs.msg import PoseStamped
import numpy as np
import rospy
import tf

class ReadyToRange(object):
    """Continuously performs ranging between the Pozyx and a destination and sets their LEDs"""

    def __init__(self, pozyx, destination_id, destination_coordinates, dt, protocol=POZYX_RANGE_PROTOCOL_FAST, remote_id=None):
        self.pozyx = pozyx
        self.destination_id = destination_id
        self.remote_id = remote_id
        self.protocol = protocol
        
        self.distance_prev_1 = 0
        self.distance_prev_2 = 0
        self.distance_prev_3 = 0
        self.distance_prev_4 = 0
        
        self.p1 = destination_coordinates[0] / 1000.
        self.p2 = destination_coordinates[1] / 1000.
        self.p3 = destination_coordinates[2] / 1000.
        self.p4 = destination_coordinates[3] / 1000.
        
        self.f1 = KalmanFilter(dim_x=1, dim_z=1, dim_u=1)
        self.f1.P = 100
        self.f1.H = np.array([[1.]])
        self.f1.F = np.array([[1.]])
        self.f1.B = np.array([[1.]])
        self.f1.Q = .02
        self.f1.R = 3
        
        self.f2 = KalmanFilter(dim_x=1, dim_z=1, dim_u=1)
        self.f2.P = 100
        self.f2.H = np.array([[1.]])
        self.f2.F = np.array([[1.]])
        self.f2.B = np.array([[1.]])
        self.f2.Q = .02
        self.f2.R = 3
        
        self.f3 = KalmanFilter(dim_x=1, dim_z=1, dim_u=1)
        self.f3.P = 100
        self.f3.H = np.array([[1.]])
        self.f3.F = np.array([[1.]])
        self.f3.B = np.array([[1.]])
        self.f3.Q = .02
        self.f3.R = 3
        
        self.f4 = KalmanFilter(dim_x=1, dim_z=1, dim_u=1)
        self.f4.P = 100
        self.f4.H = np.array([[1.]])
        self.f4.F = np.array([[1.]])
        self.f4.B = np.array([[1.]])
        self.f4.Q = .02
        self.f4.R = 3

    def setup(self):
        device_range = DeviceRange()
        #self.pozyx.printDeviceInfo(self.remote_id)
        self.pozyx.setRangingProtocol(self.protocol, self.remote_id)
        
#        for i in range(len(self.destination_id)):
#            device_range = DeviceRange()
#            self.pozyx.doRanging(self.destination_id[i], device_range)
#            distance = (float)(device_range.distance) * 0.001
#            
#            if i == 0:
#                self.distance_prev_1 = distance
#            elif i == 1:
#                self.distance_prev_2 = distance
#            elif i == 2:
#                self.distance_prev_3 = distance
#            elif i == 3:
#                self.distance_prev_4 = distance
        

    def loop(self):
        for i in range(len(self.destination_id)):
            device_range = DeviceRange()
            self.pozyx.doRanging(self.destination_id[i], device_range, self.remote_id)
            distance = (float)(device_range.distance) * 0.001

            if i == 0:
                distance_1 = distance
            elif i == 1:
                distance_2 = distance
            elif i == 2:
                distance_3 = distance
            elif i == 3:
                distance_4 = distance
                
        
        #Distance 1        
        if distance_1 == 0 :
            distance_1 = self.distance_prev_1

        self.f1.update(distance_1)
        self.f1.predict()
        
        
        #Distance 2
        if distance_2 == 0 :
            distance_2 = self.distance_prev_2

        self.f2.update(distance_2)
        self.f2.predict()
        
        
        #Distance 3
        if distance_3 == 0 :
            distance_3 = self.distance_prev_3

        self.f3.update(distance_3)
        self.f3.predict()
        
        
        #Distance 4
        if distance_4 == 0 :
            distance_4 = self.distance_prev_4
  
        self.f4.update(distance_4)
        self.f4.predict()
        
        
        self.distance_prev_1 = distance_1
        self.distance_prev_2 = distance_2
        self.distance_prev_3 = distance_3
        self.distance_prev_4 = distance_4
        
        self.distances = [distance_1, distance_2, distance_3, distance_4] 
        self.distances_kf = [self.f1.x[0], self.f2.x[0], self.f3.x[0], self.f4.x[0]]
        rospy.loginfo(self.distances)
        return self.trilaterate3D()
        
    def trilaterate3D(self):  
        r1 = self.distances_kf[0]
        r2 = self.distances_kf[1]
        r3 = self.distances_kf[2]
        r4 = self.distances_kf[3]
        
        e_x = (self.p2 - self.p1) / np.linalg.norm(self.p2 - self.p1)
        i = np.dot(e_x, (self.p3 - self.p1))
        e_y = (self.p3 - self.p1 - (i*e_x)) / (np.linalg.norm(self.p3 - self.p1 - (i*e_x)))
        e_z = np.cross(e_x, e_y)
        
        d = np.linalg.norm(self.p2 - self.p1)
        j = np.dot(e_y, (self.p3 - self.p1))
        x = ((r1**2) - (r2**2) + (d**2)) /( 2*d)
        y = (((r1**2) - (r3**2) + (i**2) + (j**2)) / (2*j)) - ((i/j) * (x))
        
        z1 = np.sqrt(np.abs(r1**2 - x**2 - y**2))
        z2 = np.sqrt(np.abs(r1**2 - x**2 - y**2)) * (-1)
        
        ans1 = self.p1 + (x * e_x) + (y * e_y) + (z1 * e_z)
        ans2 = self.p1 + (x * e_x) + (y * e_y) + (z2 * e_z)
        
        dist1 = np.linalg.norm(self.p4 - ans1)
        dist2 = np.linalg.norm(self.p4 - ans2)
        return ans2
#        if np.abs(r4 - dist1) < np.abs(r4 - dist2):
#            return ans1
#        else: 
#            return ans2
        
    def getCov(self):
        return
#        
if __name__ == "__main__":
    rospy.init_node('pozyx_node_trilateration')
    frequency = float(rospy.get_param('~frequency'))
    dt = 1/frequency
    serial_port = get_serial_ports()[2].device

    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    remote = False              
    if not remote:
        remote_id = None

    anchor0_id = int(rospy.get_param('~anchor0_id'), 16)
    anchor1_id = int(rospy.get_param('~anchor1_id'), 16)
    anchor2_id = int(rospy.get_param('~anchor2_id'), 16)
    anchor3_id = int(rospy.get_param('~anchor3_id'), 16)
    
    anchor0_coordinates = np.array(eval(rospy.get_param('~anchor0_coordinates')))
    anchor1_coordinates = np.array(eval(rospy.get_param('~anchor1_coordinates')))
    anchor2_coordinates = np.array(eval(rospy.get_param('~anchor2_coordinates')))
    anchor3_coordinates = np.array(eval(rospy.get_param('~anchor3_coordinates')))
    
    destination_id = [anchor0_id, anchor1_id, anchor2_id, anchor3_id]
    destination_coordinates = [anchor0_coordinates, anchor1_coordinates, anchor2_coordinates, anchor3_coordinates]

    ranging_protocol = POZYX_RANGE_PROTOCOL_FAST
    
    world_frame_id = rospy.get_param('~world_frame_id', 'world')
    tag_frame_id = rospy.get_param('~tag_frame_id', 'pozyx_tag')

    pozyx = PozyxSerial(serial_port)
    r = ReadyToRange(pozyx, destination_id, destination_coordinates, dt,
                     ranging_protocol, remote_id = remote_id)
    r.setup()
    rate = rospy.Rate(frequency)
    pose_pub = rospy.Publisher('/odom_uwb', PoseStamped, queue_size = 5)
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        position = r.loop()

        pwc = PoseStamped()
        pwc.header.stamp = rospy.get_rostime()
        pwc.header.frame_id = world_frame_id
        pwc.pose.position.x = position[0]
        pwc.pose.position.y = position[1]
        pwc.pose.position.z = position[2]   
        pose_pub.publish(pwc)
        br.sendTransform((position[0], position[1], 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     tag_frame_id,
                     world_frame_id)           
        rate.sleep()
