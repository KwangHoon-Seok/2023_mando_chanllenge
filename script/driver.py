#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16
from morai_msgs.msg import CtrlCmd, Lamps, GPSMessage
from morai_msgs.srv import MoraiEventCmdSrv
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Int16,Float32MultiArray
from Kalman_v import KalmanPos2Vel2D
import time
import rospy
import os
import sys
import numpy as np
from pyproj import Proj
from tf.transformations import euler_from_quaternion

# sys.path.append(os.path.dirname(os.path.abspath(__file__)) + 
#                 "/../")


from stanley import Stanley


class EgoState():
    velocity=0
    x=0
    y=0
    heading=0
    
class MoraiDriver():
    Course_1=1
    Course_2=2
    Course_3=3
    drive=0
    GEAR_P = 1
    GEAR_R = 2
    GEAR_N = 3
    GEAR_D = 4
    
    def __init__(self):
        rospy.init_node('dku_driver')
        
        #----------------- subscribers -----------------#
        rospy.Subscriber('/gps',GPSMessage,self.cb_gps,queue_size=1)
        rospy.Subscriber('/imu',Imu,self.cb_imu,queue_size=1)
        #rospy.Subscriber('/dist_ahead',Float32,self.cb_dist_ahead,queue_size=1)
        rospy.Subscriber('/obs_dist',Float32MultiArray,self.cb_obs_dist,queue_size=1)
        #rospy.Subscriber('/velocity',Float32,self.get_velocity,queue_size=1)
        
        #----------------- publishers ------------------#
        self.pub_ctrl_cmd=rospy.Publisher('ctrl_cmd',CtrlCmd,queue_size=1)

        #----------------- services --------------------#
        self.srv_event_cmd = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)        
        
        self.event_cmd_srv_type = MoraiEventCmdSrv()
        self.event_cmd_srv_type.ctrl_mode = 0
        self.event_cmd_srv_type.gear = -1
        self.event_cmd_srv_type.set_pause = False
        self.event_cmd_srv_type.lamps = Lamps()
        self.event_cmd_srv_type.option = 0
        
        self.filter = KalmanPos2Vel2D(P0 = np.diag([1,1,1,1]), x0 = np.array([np.transpose([0,0,0,0])]))
        self.ctrl_cmd_type=CtrlCmd()
        self.ctrl_cmd_type.longlCmdType=2
        
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        self.ego = EgoState()
        self.ref_tracker = Stanley()
        self.obs_dist = 0
        self.drive = 1
        self.distance_data = []
        
    def main(self):
        loop_hz=40
        rate = rospy.Rate(loop_hz)
        
        rospy.loginfo('dku_driver: waiting for gps topics')
        rospy.wait_for_message('/gps', GPSMessage)
        rospy.loginfo('dku_driver: begin')

        self.cur_Course = self.Course_1
        self.set_course_path()
        self.cur_Course_num = len(self.ref_tracker.ref_path)
        self.set_gear(self.GEAR_D)
        
        ref_velocity = 0
        steering = 0
        parking_phase = 0
        
        idx = 0
        while not rospy.is_shutdown():

            # print(self.cur_Course,self.cur_Course_num)           
            if idx == self.cur_Course_num-1:
                self.update_course()
            # if self.cur_Course == self.Course_1:
            #     rospy.loginfo('[index : %d, Phase : %d]', idx, 1)
            # elif self.cur_Course == self.Course_2:
            #     rospy.loginfo('[index : %d, Phase : %d]', idx, 2)
            # else:
            #     rospy.loginfo('[index : %d, Phase : %d]', idx, 3)
            self.VRE_speed()
            # 현재 차량 상태 업데이트
            self.ref_tracker.set_state(self.ego)
            ref_velocity, steering, idx = self.ref_tracker.main(self.drive,self.obs_dist,self.VRE)
            self.ego.veolocity = self.get_velocity()
            #print(self.ego.velocity)
            #print(self.ego.velocity)
            if (self.ego.velocity < 1 and self.drive == 3):
                self.set_gear(self.GEAR_P)
            # 제어 명령 발행
            self.set_cmd(steering, ref_velocity)
            
            rate.sleep()

    
    
    def set_course_path(self):
        if self.cur_Course == self.Course_1:
            self.ref_tracker.set_ref_path('smoothing_path1.csv', 1)
        elif self.cur_Course == self.Course_2:
            self.ref_tracker.set_ref_path('smoothing_path2.csv', 2)
        elif self.cur_Course == self.Course_3:
            self.ref_tracker.set_ref_path('smoothing_path3.csv', 3)
        else:
            rospy.logwarn("Invalid course number. Defaulting to Course 1.")
            self.ref_tracker.set_ref_path('test_gps.csv')
        return self.cur_Course

    def update_course(self):
        if self.cur_Course == self.Course_1:
            self.cur_Course = self.Course_2
            self.drive = 2
            self.set_course_path()
            self.cur_Course_num = len(self.ref_tracker.ref_path)

        elif self.cur_Course == self.Course_2:
            self.cur_Course = self.Course_3
            self.drive = 3
            self.set_course_path()
            self.cur_Course_num = len(self.ref_tracker.ref_path)


           
    def set_cmd(self,steer,vel):
        self.ctrl_cmd_type.velocity=vel
        self.ctrl_cmd_type.steering=steer
        self.pub_ctrl_cmd.publish(self.ctrl_cmd_type)

    def set_gear(self,gear):
        while abs(self.ego.velocity > 1):
            continue
        self.event_cmd_srv_type.option = 2
        self.event_cmd_srv_type.gear = gear
        _ = self.srv_event_cmd(self.event_cmd_srv_type)
    
    

   
    #----------------- callback 함수 --------------------#
    
    def cb_gps(self,data):
        if data.longitude != 0:
            # self.ego.x, self.ego.y = self.proj_UTM(data.longitude, data.latitude)
            x, y = self.proj_UTM(data.longitude, data.latitude)
            self.ego.x = x  - data.eastOffset
            self.ego.y = y  - data.northOffset

        else:
            self.ego.x, self.ego.y, self.ego.velocity = 0, 0, 0
            
    def get_velocity(self):
        self.ego.velocity = self.filter.main(self.ego.x,self.ego.y,self.ego.ax)
        
    def cb_imu(self, data):
        euler = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.ego.heading = euler[2]
        self.ego.heading = 2*np.pi + self.ego.heading if -np.pi < self.ego.heading < -np.pi/2  else self.ego.heading
        #rospy.loginfo('Heading : %.3f', self.ego.heading)
        self.ego.ax      = data.linear_acceleration.x


    
    def cb_obs_dist(self,data):
        obs_dists = data.data
        self.obs_dist = min(obs_dists)
        self.distance_data.append(self.obs_dist)
        # self.VRE()
        #print(self.obs_dist)
        
    def VRE_speed(self):
        dt = 1/40
        if len(self.distance_data) > 1:
            distance_diff = np.diff(self.distance_data)
            speed = distance_diff / dt
            self.VRE = abs(speed[-1])
            #print(self.ego.velocity,self.VRE)
            self.distance_data = [self.distance_data[-1]]
        else:
            self.VRE = 100
            
    def set_gear(self, gear):
        while abs(self.ego.velocity > 1):
            continue
        self.event_cmd_srv_type.option = 2
        self.event_cmd_srv_type.gear = gear
        _ = self.srv_event_cmd(self.event_cmd_srv_type)
        
if __name__=='__main__':
    morai_driver = MoraiDriver()
    morai_driver.main()