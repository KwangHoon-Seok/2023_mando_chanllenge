import numpy as np
from math import cos, sin, pi, sqrt, pow, atan2, hypot, tan
import csv
import os
import rospkg
from Kalman_v import KalmanPos2Vel2D
import rospy
import matplotlib.pyplot as plt
import math
import time

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
        self.curve_speed = 20
        self.idx = 0
        self.flag = False
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




    def main(self,ref_course,obs_dist,VRE):
        self.get_velocity()
        self.idx = self.find_min_idx([self.x, self.y])
        self.set_ref_vel(ref_course,obs_dist,VRE)
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

    def set_ref_vel(self, drive,obs_dist,VRE):
        # drive가 1일 때만 조건 검사
        if drive == 1:
            # idx 값에 따른 조건 분기
            if (self.idx > 305 and self.idx < 447) or (self.idx > 455 and self.idx < 566) or (self.idx > 670 and self.idx < 778) or (self.idx > 800):
                self.ref_vel = 20
                if obs_dist < 14:
                    self.ref_vel = 0
                    # static 메소드의 결과에 따라 차량의 속도 조정
                    if self.flag == False:
                        if self.static(VRE) ==0 :
                            self.ref_vel = 20
                            self.flag = True
                        else:
                            self.ref_vel = 0
                    else:
                        self.ref_vel = 20
                else:
                    self.flag = False

            elif obs_dist < 0.42 * self.velocity:
                self.ref_vel = 0
            else:
                self.ref_vel = 50
        if drive == 2:
            if (self.idx > 0 and self.idx < 37) or (self.idx > 404 and self.idx < 521):
                self.ref_vel = 20
                if obs_dist < 10:
                    self.ref_vel = 0
                    self.static(VRE)
            elif (self.idx >270 and self.idx < 405) or (self.idx > 610 and self.idx < 703):
                self.ref_vel = 15
                if obs_dist < 10:
                    self.ref_vel = 0
                    self.static(VRE)
            elif obs_dist < 0.42 * self.velocity:
                self.ref_vel = 0
            else:
                self.ref_vel = 50
        if drive == 3:
            if (self.idx > 0 and self.idx < 84) or (self.idx > 190 and self.idx < 300):
                self.ref_vel = 20
                if obs_dist < 10:
                    self.ref_vel = 0
                    self.static(VRE)
            elif self.idx >= 301:
                self.ref_vel = 0
            elif obs_dist < 0.42 * self.velocity:
                self.ref_vel = 0
            else:
                self.ref_vel = 50
        print(f'mode: {drive}, idx: {self.idx}, 거리: {obs_dist}, 활성거리 : {0.42 * self.velocity}')
        #print(f'거리: {obs_dist}, 활성거리 : {0.42 * self.velocity}')

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
                





# if __name__ == '__main__':
#     driver = Stanley()
#     fn = 'smoothing_path2.csv'
#     driver.set_ref_path(fn)
#     driver.x = driver.ref_path[400, 0]
#     driver.y = driver.ref_path[400, 1]
#     driver.yaw = pi/2

#     plt.plot(driver.ref_path[:, 0], driver.ref_path[:, 1], 'b--')
#     plt.plot(driver.x, driver.y, 'ro')


#     for i in range(500):

#         print(i)
#         velocity, steering, _ = driver.main()
#         driver.yaw += driver.normalize(driver.dt * (velocity * tan(steering) / driver.vehicle_length))
#         driver.x += velocity * driver.dt * cos(driver.yaw)
#         driver.y += velocity * driver.dt * sin(driver.yaw)

#         plt.plot(driver.x, driver.y, 'ro')

#     # x_coords = [driver.ref_path[i, 0] for i in driver.curve_point]
#     # y_coords = [driver.ref_path[i, 1] for i in driver.curve_point]

#     # x_coords2 = [driver.ref_path[j, 0] for j in driver.linear_point]
#     # y_coords2 = [driver.ref_path[j, 1] for j in driver.linear_point]

#     # plt.scatter(x_coords, y_coords, color='red')
#     # plt.scatter(x_coords2, y_coords2, color='blue')
#     plt.show()