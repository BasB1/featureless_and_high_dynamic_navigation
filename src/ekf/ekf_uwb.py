#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from math import cos, sin, sqrt, atan2, tan
import numpy as np
from numpy import array, dot
from numpy.random import randn
from filterpy.kalman import ExtendedKalmanFilter as EKF
import matplotlib.pyplot as plt

class LocalizeEKF(EKF):
    def __init__(self, dt, wheelbase):
        EKF.__init__(self, 3, 2, 2)
        rospy.init_node('ekf', anonymous=True)
        #self.sub = rospy.Subscriber('/odom', Odometry, self.getOdom)
        self.pub = rospy.Publisher('/odom_ekf', Odometry, queue_size=0)
        self.tf_br = tf.TransformBroadcaster()
        self.dt = dt
        self.wheelbase = wheelbase
        self.OdomFiltered = Odometry()
        self.m = array([[0,0]])
        self.u = array([.0, .0])
        self.PP = np.mat(np.diag([0.0]*3))
        rospy.wait_for_message("/odom", Odometry)
        rospy.sleep(1)
    
    def getOdom(self):
        data = rospy.wait_for_message("/odom", Odometry)
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        self.original_quat = data.pose.pose.orientation
        
        b = np.array([[x,y]])        
        
        self.m = np.concatenate((self.m, b), axis=0)
        self.m = np.delete(self.m, 0, 0)
        self.u = array([data.twist.twist.linear.x, data.twist.twist.angular.z])
    
    def publishOdom(self, x, y):       
        self.OdomFiltered.header.stamp = rospy.Time.now()
        self.OdomFiltered.header.frame_id = '/odom_ekf'
        self.OdomFiltered.child_frame_id = '/map'

        self.OdomFiltered.pose.pose.position = Point(x.item(0), y.item(0), 0)
        self.OdomFiltered.pose.pose.orientation = self.original_quat

        
        p_cov = np.array([0.0]*36).reshape(6,6)

        # position covariance
        p_cov[0:2,0:2] = self.PP[0:2,0:2]
        # orientation covariance for Yaw
        # x and Yaw
        p_cov[5,0] = p_cov[0,5] = self.PP[2,0]
        # y and Yaw
        p_cov[5,1] = p_cov[1,5] = self.PP[2,1]
        # Yaw and Yaw
        p_cov[5,5] = self.PP[2,2]
    
        self.OdomFiltered.pose.covariance = tuple(p_cov.ravel().tolist())

        pos = (self.OdomFiltered.pose.pose.position.x,
               self.OdomFiltered.pose.pose.position.y,
               self.OdomFiltered.pose.pose.position.z)

        ori = (self.OdomFiltered.pose.pose.orientation.x,
               self.OdomFiltered.pose.pose.orientation.y,
               self.OdomFiltered.pose.pose.orientation.z,
               self.OdomFiltered.pose.pose.orientation.w)
        rospy.loginfo(self.OdomFiltered)
        self.pub.publish(self.OdomFiltered)

        self.tf_br.sendTransform(pos, ori, self.OdomFiltered.header.stamp, self.OdomFiltered.child_frame_id, self.OdomFiltered.header.frame_id)
        
    def getU(self):
        return self.u
        
    def getM(self):
        return self.m
        
    def predict(self, u):       
        self.x = self.move(self.x, u, self.dt)

        h = self.x[2, 0]
        v = u[0]
        steering_angle = u[1]

        dist = v*self.dt

        if abs(steering_angle) < 0.0001:
            r = 1.e-30
        else:
            r = self.wheelbase / tan(steering_angle)
            
        b = dist / self.wheelbase * tan(steering_angle)
        
        sinh = sin(h)
        sinhb = sin(h + b)
        cosh = cos(h)
        coshb = cos(h + b)

        F = array([[1., 0., -r*cosh + r*coshb],
                   [0., 1., -r*sinh + r*sinhb],
                   [0., 0., 1.]])

        w = self.wheelbase

        F = array([[1., 0., (-w*cosh + w*coshb)/tan(steering_angle)],
                   [0., 1., (-w*sinh + w*sinhb)/tan(steering_angle)],
                   [0., 0., 1.]])

        V = array(
            [[-r*sinh + r*sinhb, 0],
             [r*cosh + r*coshb, 0],
             [0, 0]])

        t2 = tan(steering_angle)**2
        V = array([[0, w*sinh*(-t2-1)/t2 + w*sinhb*(-t2-1)/t2],
                   [0, w*cosh*(-t2-1)/t2 - w*coshb*(-t2-1)/t2],
                   [0,0]])

        t2 = tan(steering_angle)**2

        a = steering_angle
        d = v*self.dt
        it = self.dt*v*tan(a)/w + h

        V[0,0] = self.dt*cos(d/w*tan(a) + h)
        V[0,1] = (self.dt*v*(t2+1)*cos(it)/tan(a) -
                  w*sinh*(-t2-1)/t2 +
                  w*(-t2-1)*sin(it)/t2)

        V[1,0] = self.dt*sin(it)

        V[1,1] = (d*(t2+1)*sin(it)/tan(a) + w*cosh/t2*(-t2-1) -
                  w*(-t2-1)*cos(it)/t2)

        V[2,0] = self.dt/w*tan(a)
        V[2,1] = d/w*(t2+1)

        M = array([[0.1*v**2, 0],
                   [0,         sigma_steer**2]])

        self.P = dot(F, self.P).dot(F.T) + dot(V, M).dot(V.T)
        
    def move(self, x, u, dt):
        dt = dt/10.
        h = x[2, 0]
        v = u[0]
        steering_angle = u[1]

        dist = v*dt

        if abs(steering_angle) < 0.0001:
            # approximate straight line with huge radius
            r = 1.e-30
        else:
            r = self.wheelbase / tan(steering_angle) # radius
            
        b = dist / self.wheelbase * tan(steering_angle)
        

        sinh = sin(h)
        sinhb = sin(h + b)
        cosh = cos(h)
        coshb = cos(h + b)
        to_return = x + array([[-r*sinh + r*sinhb],
                               [r*cosh - r*coshb],
                               [b]])
        #rospy.loginfo(to_return)
        return to_return      
                          
    def H_of(self, x, p):
        """ compute Jacobian of H matrix where h(x) computes the range and
        bearing to a landmark for state x """
    
        px = p[0]
        py = p[1]
        hyp = (px - x[0, 0])**2 + (py - x[1, 0])**2
        dist = np.sqrt(hyp)
    
        H = array(
            [[-(px - x[0, 0]) / dist, -(py - x[1, 0]) / dist, 0],
             [ (py - x[1, 0]) / hyp,  -(px - x[0, 0]) / hyp, -1]])
        return H

    def Hx(self, x, p):
        """ takes a state variable and returns the measurement that would
        correspond to that state.
        """
        px = p[0]
        py = p[1]
        dist = np.sqrt((px - x[0, 0])**2 + (py - x[1, 0])**2)
    
        Hx = array([[dist],
                    [atan2(py - x[1, 0], px - x[0, 0]) - x[2, 0]]])
        return Hx
        
    def normalize_angle(self, x, index):
        if x[index] > np.pi:
            x[index] -= 2*np.pi
        if x[index] < -np.pi:
            x[index] = 2*np.pi
    
    def residual(self, a,b):
        y = a - b
        self.normalize_angle(y, 1)
        return y

if __name__ == '__main__':
    sigma_r = 0.01
    sigma_h =  np.radians(1)
    sigma_steer =  np.radians(1)
    wheelbase = 0.1
    frequency = 10
    dt = 1/frequency
    
    ekf = LocalizeEKF(dt, wheelbase)
    
    ekf.P = np.diag([1., 1., 1.])
    ekf.R = np.diag([sigma_r**2, sigma_h**2])
    c = [0, 1, 2]
    
    xp = ekf.x.copy()
    
    rate = rospy.Rate(frequency)

    ekf.getOdom()
    
    ekf.x = array([[0, 0, 0]]).T
        
    while not rospy.is_shutdown():
        ekf.getOdom()
        u = ekf.getU()
        m = ekf.getM()
        ekf.predict(u)
        #rospy.loginfo(u)
        for lmark in m:
            d = sqrt((lmark[0] - xp[0, 0])**2 + (lmark[1] - xp[1, 0])**2)  + randn()*sigma_r
            a = atan2(lmark[1] - xp[1, 0], lmark[0] - xp[0, 0]) - xp[2, 0] + randn()*sigma_h
            z = np.array([[d], [a]])
    
            ekf.update(z, HJacobian=ekf.H_of, Hx=ekf.Hx, residual=ekf.residual, args=(lmark), hx_args=(lmark))

        ekf.publishOdom(ekf.x[0], ekf.x[1])
        rate.sleep()   

    if rospy.is_shutdown():
        rospy.signal_shutdown('ekf')