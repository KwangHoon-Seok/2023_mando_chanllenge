import numpy as np
from math import cos, sin, pi, sqrt, pow, atan2, hypot, tan
import csv
import os
import rospkg
from Kalman_v import KalmanPos2Vel2D
from Kalman_prev import KalmanPos2Vel
import rospy
import matplotlib.pyplot as plt
import math
import time
from std_msgs.msg import Float32, Int16,Float32MultiArray
from Kalman_prev1 import KalmanPos2Vel1
class Path_State():
    def __init__(self, a, b, c, Pose, cur_Waypoint, Next_Waypoint, slope):
        self.kind = None
        self.direction = None
        self.cur_pose = None
        self.a = a
        self.b = b
        self.c = c
        self.Pose = Pose
        self.cur_Waypoint = cur_Waypoint
        self.Next_Waypoint = Next_Waypoint
        self.slope = slope


    def Update_state(self):

        if self.slope == math.inf:
            self.kind = "Vertical"

        elif self.slope == -math.inf:
            self.kind = "Vertical"

        elif self.slope == 0:
            self.kind = "Horizontal"

        else:
            if self.slope > 0:
                self.kind = "Increase"
            else:
                self.kind = "Decrease"

        
        if (self.cur_Waypoint[0] < self.Next_Waypoint[0]) or (self.kind == "Vertical" and self.cur_Waypoint[1] < self.Next_Waypoint[1]):
            self.direction = "Positive"
        else:
            self.direction = "Negative"

        # if not self.kind == "Decrease":
        #     if (self.cur_Waypoint[0] < self.Next_Waypoint[0]) or (self.kind == "Vertical" and self.cur_Waypoint[1] < self.Next_Waypoint[1]):
        #         self.direction = "Positive"
        #     else:
        #         self.direction = "Negative"
        # else:
        #     if (self.cur_Waypoint[0] > self.Next_Waypoint[0]):
        #         self.direction = "Positive"
        #     else:
        #         self.direction = "Negative"
            


        if self.kind == "Horizontal":

            if self.Pose[1] > -self.c:
                self.cur_pose = "Left"
            else:
                self.cur_pose = "Right"

        elif self.kind == "Vertical":
            if self.Pose[0] < self.c:
                self.cur_pose = "Left"
            else:
                self.cur_pose = "Right"

        else:
            if self.Pose[1] > -((self.a * self.Pose[0] + self.c) / self.b):
                self.cur_pose = "Left"
            else:
                self.cur_pose = "Right"

class Stanley():
    def __init__(self, dt=1/40):
        self.dt = dt
        self.ref_path = []
        self.path_num = 0
        self.Next_Waypoint = []
        self.cur_Waypoint = []
        self.closest_idx = 0
        self.Front_wheel_x = 0
        self.Front_wheel_y = 0
        self.vehicle_length = 4.635
        self.vehicle_wheel_base = 3
        self.vehicle_sensor_x = 1.86
        self.vehicle_front_wheel_base = self.vehicle_wheel_base - self.vehicle_sensor_x
        self.k = 7
        self.filter = KalmanPos2Vel2D(P0 = np.diag([1,1,1,1]), x0 = np.array([np.transpose([0,0,0,0])]))
        self.filter_p1 = KalmanPos2Vel1(P0 = np.diag([1,1,1,1]), x0 = np.array([np.transpose([0,0,0,0])]))
        self.curve_speed = 20
        self.idx = 0
        self.flag = False
        self.ref_vel =25
        self.kf_dist = KalmanPos2Vel(P0 = np.array([[1,0],[0,1]]), x0=np.array([[0],[0]]))
        self.FLAG = False
        self.person_x = 0
        self.person_y = 0
        self.person_vx = 0
        self.person_vy = 0
        #-------------subscribers----------------#
  
    def set_state(self, ego):
        self.x = ego.x
        self.y = ego.y
        self.yaw = ego.heading
        # rospy.loginfo('%.6f', self.yaw)
        self.Front_wheel_x = ego.x + self.vehicle_front_wheel_base * cos(self.yaw)
        self.Front_wheel_y = ego.y + self.vehicle_front_wheel_base * sin(self.yaw)
        
        if abs(ego.ax) < 0.04:
            self.ax = 0
        else:
            self.ax = ego.ax




    def main(self,ref_course,obs_dist,VRE,x,y,cls,person_x,person_y):
        self.get_velocity()
        self.idx = self.find_min_idx([self.x, self.y])
        self.set_ref_vel(ref_course,obs_dist,VRE,x,y,cls,person_x,person_y)
        # rospy.loginfo('[Estimate velocity : %.3f]', self.velocity)F
        

        if self.idx < len(self.ref_path) - 1:
            self.cur_Waypoint = self.ref_path[self.idx, :]
            self.Next_Waypoint = self.ref_path[self.idx+1, :]
        else:
            self.cur_Waypoint = self.ref_path[-2, :]
            self.Next_Waypoint = self.ref_path[-1,:]

        # ax + by + c = 0
        self.path_slope = ( self.Next_Waypoint[1] - self.cur_Waypoint[1] ) / ( self.Next_Waypoint[0] - self.cur_Waypoint[0] )

        if self.path_slope == 0:
            a = 0
            b = 1
            c = -self.Next_Waypoint(2)
        else:
            a = -1
            b =  1/self.path_slope
            c = self.cur_Waypoint[0] - b*self.cur_Waypoint[1]

        Heading_error = self.cal_HE()

        if np.abs(Heading_error) >= pi/2:
            Cross_Track_Error = 0
        else:
            Cross_Track_Error = self.cal_CTE(a, b, c)


        # steering = Heading_error + atan2(self.k*Cross_Track_Error, self.velocity)
        if self.velocity < 1:
            steering = Heading_error + atan2(self.k*Cross_Track_Error, 10)
        else:
            steering = Heading_error + atan2(self.k*Cross_Track_Error, self.velocity)

        if steering > 40 * pi/180:
            steering = 40 * pi/180
        elif steering < -40 * pi/180:
            steering = -40 * pi/180

        
        return self.ref_vel, steering, self.idx



    def cal_HE(self):
        dx = self.Next_Waypoint[0] - self.cur_Waypoint[0]
        dy = self.Next_Waypoint[1] - self.cur_Waypoint[1]

        if (-pi < atan2(dy, dx) < -pi/2):
            
            return atan2(dy, dx) + 2*pi - self.yaw if self.yaw > 0 else atan2(dy, dx) - self.yaw
        
        return atan2(dy, dx) - self.yaw
    

    def cal_CTE(self, a, b, c):

        Path_state = Path_State(a, b, c, [self.x, self.y, self.yaw], self.cur_Waypoint, self.Next_Waypoint, self.path_slope)
        Path_state.Update_state()

        cte = np.abs(a*self.Front_wheel_x + b*self.Front_wheel_y + c) / sqrt(a**2 + b**2)

        if Path_state.direction == "Positive":
            if Path_state.cur_pose == "Left":
                cte = -cte

        else:
            if Path_state.cur_pose == "Right":
                cte = -cte

        return cte

    def set_ref_vel(self, drive ,obs_dist, VRE , x ,y, cls,person_x,person_y):
        # drive가 1일 때만 조건 검사dd
        dist_err = x - 10 # 가까워질때 음수값
        kfspeed, _=self.kf_dist.update(dist_err,1/40)
        kfspeed = np.clip(kfspeed,-0.1,0.1)
        state = self.state_update(self.idx)
        if drive == 1:
            # cls 2 자동차 , ACC 구간도 정해놔야할듯
            if state == -1:
                self.ref_vel = 25
            # cls 0 사람                
            if state == 0:
                if cls == 0 and obs_dist < 15:
                    self.ref_vel = 0
                    if self.FLAG == False:
                        if self.static(VRE) == 0 and abs(person_y) > 3.5:
                            self.ref_vel = 25
                            self.FLAG = True
                        else:
                            self.ref_vel = 0
                    else:
                        self.ref_vel = 25
                else:
                    self.FLAG = False
                    self.ref_vel = 25
            if state == 1:
                if cls == 2 :
                    if 0< x < 10 and abs(y) < 5:
                        self.ref_vel = self.ref_vel + kfspeed + dist_err/10
                    else:
                        self.ref_vel = self.ref_vel + kfspeed
                    self.ref_vel = np.clip(self.ref_vel,0,25)
                else:
                    self.ref_vel = 15
            

        #print(drive,VRE , self.ref_vel,obs_dist,person_x, person_y)
        # 차폭 y : 1.9m 도로폭 : 2.4m
        #print(f'ref_vel: {self.ref_vel}, idx: {self.idx}, dist: {obs_dist}, class: {cls}, y: {abs(person_y)}, state: {state}, VRE: {VRE}')
        print(self.x , self.y,state)
    def static(self,VRE):
        if VRE < 2:
            #print("정적장애물")
            return 0
        else:
            #print("동적장애물")
            return 1
    
    def get_velocity(self):
        self.velocity = self.filter.main(self.x, self.y, self.ax)
        
        
    def cal_dist(self, start_point, end_point):
        return hypot(end_point[0] - start_point[0], end_point[1] - start_point[1])

    def find_min_idx(self, vehicle_position):
        self.closest_idx = 2 + np.argmin(np.array([self.cal_dist(vehicle_position, d) for d in self.ref_path]))
        return self.closest_idx

    def normalize(self, theta):
        return theta + 2*pi if (-pi < theta < -pi/2) else theta
    
    def stop(self):
        self.ref_vel = 0
        return self.ref_vel

    def state_update(self,idx):
        if (idx > 18046 and idx < 19180) or (idx > 21400 and idx < 22712):
            return 0  # 보행자
        elif (idx > 0 and idx < 14510) or (idx > 24375 and idx < 26358):
            return 1  # ACC
        else:
            return -1
    def set_ref_path(self, filename):
            rospack = rospkg.RosPack()
            filename = os.path.join(rospack.get_path('dku_morai_driver'), 
                                                    f'ref_path/{filename}')        
            self.ref_path = []
            with open(filename, newline='') as csvfile:
                path_reader = csv.reader(csvfile, delimiter=',', quotechar='|')
                for row in path_reader:
                    self.ref_path.append([float(row[0]), float(row[1])])
                

            self.ref_path_list = self.ref_path
            self.ref_path = np.array(self.ref_path)
            self.closest_idx = 0
    