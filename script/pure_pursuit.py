import numpy as np
from math import cos, sin, pi, sqrt, pow, atan2, hypot, tan
import csv
import os
import rospkg
from Kalman_v import KalmanPos2Vel2D
import rospy
import matplotlib.pyplot as plt
import math


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
                
class P_P():
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
        self.curve_speed = 20
        self.idx = 0
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




    def main(self,ref_course):
        self.get_velocity()
        lookahead_distance = max(0.7,self.velocity*0.01)
        self.idx , lookahead_point = self.find_lookahead_point(lookahead_distance)
        self.set_ref_vel(ref_course)
        # rospy.loginfo('[Estimate velocity : %.3f]', self.velocity)
        
        

        if self.idx < len(self.ref_path) - 1:
            self.cur_Waypoint = self.ref_path[self.idx, :]
            self.Next_Waypoint = self.ref_path[self.idx+1, :]
        else:
            self.cur_Waypoint = self.ref_path[-2, :]
            self.Next_Waypoint = self.ref_path[-1,:]

        if lookahead_point is not None:
            L = np.sqrt((lookahead_point[0] - self.x)**2 + (lookahead_point[1] - self.y)**2)
            #alpha 값이 이상함
            alpha = self.cal_HE()
            steering = np.arctan2((2*L*np.sin(alpha)),L)
            
            max_steering_angle = 40 * np.pi/180
            steering = np.clip(steering, -max_steering_angle,max_steering_angle)
        else:
            steering = 0
        print(alpha)

        
        return self.ref_vel, steering, self.idx


    def cal_HE(self):
        dx = self.Next_Waypoint[0] - self.cur_Waypoint[0]
        dy = self.Next_Waypoint[1] - self.cur_Waypoint[1]

        if (-pi < atan2(dy, dx) < -pi/2):
            
            return atan2(dy, dx) + 2*pi - self.yaw if self.yaw > 0 else atan2(dy, dx) - self.yaw
        
        return atan2(dy, dx) - self.yaw

    def find_lookahead_point(self, lookahead_distance):
        # find_min_idx 함수를 호출하여 가장 가까운 경로점의 인덱스를 얻습니다.
        closest_idx = self.find_min_idx([self.x,self.y])  # 함수 호출 시 괄호 사용

        total_distance = 0
        lookahead_point = None
        for i in range(closest_idx, len(self.ref_path) - 1):
            start_point = self.ref_path[i]
            end_point = self.ref_path[i + 1]
            segment_length = np.sqrt((end_point[0] - start_point[0])**2 + (end_point[1] - start_point[1])**2)

            total_distance += segment_length  # 누적 거리에 세그먼트 길이를 더해야 합니다.
            if total_distance >= lookahead_distance:
                overshoot_distance = total_distance - lookahead_distance
                ratio = 1 - overshoot_distance / segment_length
                lookahead_x = ratio * end_point[0] + (1 - ratio) * start_point[0]
                lookahead_y = ratio * end_point[1] + (1 - ratio) * start_point[1]
                lookahead_point = (lookahead_x, lookahead_y)
                break  # 목표점을 찾았으므로 반복문을 빠져나옵니다.

        if lookahead_point is not None:
            return i, lookahead_point  # 목표점을 찾은 경우 인덱스와 함께 반환
        else:
            return closest_idx, None  # 목표점을 찾지 못한 경우 None 반환

            

    

    def set_ref_vel(self, drive):
        # drive가 1일 때만 조건 검사
        if drive == 1:
            # idx 값에 따른 조건 분기
            if (self.idx > 305 and self.idx < 447) or (self.idx > 455 and self.idx < 566) or (self.idx > 670 and self.idx < 778) or (self.idx > 800):
                self.ref_vel = 30
            
            else:
                self.ref_vel = 30
        if drive == 2:
            if (self.idx > 0 and self.idx < 37) or (self.idx > 404 and self.idx < 521):
                self.ref_vel = 20
            elif (self.idx >270 and self.idx < 405) or (self.idx > 610 and self.idx < 703):
                self.ref_vel = 15
            
            else:
                self.ref_vel = 25
        if drive == 3:
            if (self.idx > 0 and self.idx < 84) or (self.idx > 190 and self.idx < 300):
                self.ref_vel = 20
            elif self.idx >= 301:
                self.ref_vel = 0
            
            else:
                self.ref_vel = 25
        #print(f'mode: {drive}, idx: {self.idx}')
           

        
    
    def get_velocity(self):
        self.velocity = self.filter.main(self.x, self.y, self.ax)


    def normalize(self, theta):
        return theta + 2*pi if (-pi < theta < -pi/2) else theta
    
    def stop(self):
        self.ref_vel = 0
        return self.ref_vel

    def find_min_idx(self, vehicle_position):
        self.closest_idx = 2 + np.argmin(np.array([self.cal_dist(vehicle_position, d) for d in self.ref_path]))
        return self.closest_idx

    def cal_dist(self, start_point, end_point):
        return hypot(end_point[0] - start_point[0], end_point[1] - start_point[1])
    
    def set_ref_path(self, filename, path_num):
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
            self.path_num = path_num