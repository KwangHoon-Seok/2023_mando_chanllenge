#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16
import rospy
import math
import numpy as np

from morai_msgs.msg import GPSMessage
from std_msgs.msg import Float32
from pyproj import Proj
from Kalman import Kalman

class VelocityEstimator():
    def __init__(self):
        rospy.init_node('velocity_estimator')
        rospy.Subscriber('/gps', GPSMessage, self.cb_gps, queue_size=1)
        self.pub_vel = rospy.Publisher('/velocity', Float32, queue_size=1)
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        self.x, self.y, self.velocity = 0, 0, 0
        self.flag_prev_no_gps = False

    def cb_gps(self, data):
        if data.longitude != 0:
            self.x, self.y = self.proj_UTM(data.longitude, data.latitude)
            if self.flag_prev_no_gps == True:
                self.kf_vel.x_esti = np.array([[self.x],[0],[self.y],[0]])
                self.flag_prev_no_gps = False
        else:
            self.x, self.y, self.velocity = 0, 0, 0
            self.flag_prev_no_gps = True

    def main(self):        
        loop_hz = 50
        rate = rospy.Rate(loop_hz)
        dt_gps = 1/loop_hz
        rospy.logwarn('velocity_estimator: waiting for gps topics')
        rospy.wait_for_message('/gps', GPSMessage)
        rospy.logwarn('velocity_estimator: begin')
        q1,q2,r=1,100,1000
        self.kf_vel = Kalman(A=np.array([[1,dt_gps,0,0],
                                                    [0,1,0,0],
                                                    [0,0,1,dt_gps],
                                                    [0,0,0,1]]),
                                        H=np.array([[1,0,0,0],
                                                    [0,0,1,0]]),
                                        P0=np.eye(4),
                                        x0=np.array([[self.x],[0],[self.y],[0]]),
                                        Q=np.array([[q1,0,0,0],
                                                    [0,q2,0,0],
                                                    [0,0,q1,0],
                                                    [0,0,0,q2]]),
                                        R=np.eye(2)*r)

        while not rospy.is_shutdown():
            if self.x != 0:
                xhat, _ = self.kf_vel.update(np.array([[self.x],[self.y]]))
                self.velocity = math.sqrt(xhat[1,0]**2+xhat[3,0]**2)*3.6
                self.pub_vel.publish(self.velocity)
            # rospy.loginfo(self.velocity)

            rate.sleep()


if __name__ == '__main__':
    ve = VelocityEstimator()
    ve.main()