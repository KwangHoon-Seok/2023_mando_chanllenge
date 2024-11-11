#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from morai_msgs.msg import CtrlCmd, Lamps, GPSMessage
from morai_msgs.srv import MoraiEventCmdSrv
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float32MultiArray
from Kalman_v import KalmanPos2Vel2D
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion
from pyproj import Proj
class EgoState():
    def __init__(self):
        self.velocity = 0
        self.x = 0
        self.y = 0
        self.heading = 0
        self.ax = 0

class RelativeSpeed():
    def __init__(self):
        rospy.init_node('Estimation_relative_speed')
        
        # Subscribers
        rospy.Subscriber('/gps',GPSMessage,self.cb_gps,queue_size=1)
        rospy.Subscriber('/imu', Imu, self.cb_imu, queue_size=1)
        rospy.Subscriber('/dist_ahead', Float32, self.cb_dist_ahead, queue_size=1)
        rospy.Subscriber('/obs_dist', Float32MultiArray, self.cb_obs_dist, queue_size=1)
        
        # Service
        self.srv_event_cmd = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)  
        
        # Kalman Filter
        self.filter = KalmanPos2Vel2D(P0=np.diag([1, 1, 1, 1]), x0=np.array([0, 0, 0, 0]))
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        
        self.ego = EgoState()
        self.distance_data = []
        self.dist_ahead = 0
        self.flag_dist_ahead_updated = False
        self.VRS = 0
        
    def main(self):
        loop_hz = 40
        rate = rospy.Rate(loop_hz)
        rospy.loginfo('Waiting for IMU topics')
        rospy.wait_for_message('/imu', Imu)
        rospy.loginfo('Estimation start')
        
        while not rospy.is_shutdown():
            self.VRS_speed()
            self.get_velocity()
            rate.sleep()
        
    def get_velocity(self):
        # Assuming 'self.filter.main' updates self.ego.velocity directly
        self.ego.velocity = self.filter.main(self.ego.x, self.ego.y, self.ego.ax)
        
    def VRS_speed(self):
        dt = 1.0 / 40
        if len(self.distance_data) > 1:
            distance_diff = np.diff(self.distance_data)
            speed = distance_diff / dt
            self.VRS = abs(speed[-1])
            rospy.loginfo("Ego Velocity: {}, VRS: {}".format(self.ego.velocity, self.VRS))
            #print(self.obs_dist)
            self.distance_data = [self.distance_data[-1]]
        else:
            self.VRS = 100  # Default or placeholder value
            
    def cb_imu(self, data):
        quaternion = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.ego.heading = euler[2]
        self.ego.heading = 2 * np.pi + self.ego.heading if -np.pi < self.ego.heading < -np.pi / 2 else self.ego.heading
        self.ego.ax = data.linear_acceleration.x        
    def cb_gps(self,data):
        if data.longitude != 0:
            # self.ego.x, self.ego.y = self.proj_UTM(data.longitude, data.latitude)
            x, y = self.proj_UTM(data.longitude, data.latitude)
            self.ego.x = x  - data.eastOffset
            self.ego.y = y  - data.northOffset
        else:
            self.ego.x, self.ego.y, self.ego.velocity = 0, 0, 0
            
    def cb_dist_ahead(self, data):
        self.dist_ahead = data.data
        self.flag_dist_ahead_updated = True
    
    def cb_obs_dist(self, data):
        obs_dists = data.data
        self.obs_dist = min(obs_dists)
        self.distance_data.append(self.obs_dist)

if __name__ == '__main__':
    try:
        relative_speed_estimator = RelativeSpeed()
        relative_speed_estimator.main()
    except rospy.ROSInterruptException:
        pass
