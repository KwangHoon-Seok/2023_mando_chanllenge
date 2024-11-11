#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16
import numpy as np
from numpy.linalg import inv

class Kalman():
    def __init__(self, A, H, P0, x0, Q, R, dt=0.02):
        self.A = A
        self.H = H
        self.Q = Q
        self.R = R
        self.P = P0
        self.dt = dt
        self.x_esti = x0

    
class KalmanPos2Vel():
    # 위치 값을 통한 속도 추정 -> 좌표계 일치 이슈..
    # -> x,y data를 모두 가져와서 올바른 선속도 계산
    # -> 좌표계 일치도 필요없음
    def __init__(self, P0, x0, dt = 1/40, q1 = 5, q2 = 5, r= 1000):

        self.x_cur  = np.transpose(x0)
        self.x_esti = np.transpose(x0)
        self.P = P0
        self.A = np.array([[1, 0],
                           [0, 1]])
        
        self.H = np.array([[1, 0],
                           [0, 1]])
        self.Q = np.array([[q1, 0],
                           [0, q2]])
        self.R = np.array([[r]])
        self.dt = dt
        self.firstrun = True

    def update(self, z):
        x_p = np.matmul(self.A, self.x_cur)
        P_p = np.matmul(np.matmul(self.A, self.P), np.transpose(self.A)) + self.Q

        K = np.matmul(np.matmul(P_p, np.transpose(self.H)), inv(np.matmul(np.matmul(self.H, P_p), np.transpose(self.H)) + self.R))

        self.x_esti = x_p + np.matmul(K, z - np.matmul(self.H, x_p))
        self.v_esti = [(self.x_esti[0][0]-self.x_cur[0][0])/self.dt, (self.x_esti[1][0]-self.x_cur[1][0])/self.dt]
        self.P      = P_p - np.matmul(K, np.matmul(self.H, P_p))
        
        self.x_cur = self.x_esti


        return 3.6 * np.sqrt(self.v_esti[0]**2 + self.v_esti[1]**2)
    
    def main(self, x, y, ax):

        if self.firstrun:
            # 초기값 -> 초기 측정값
            self.x_cur = np.array([[x], [y]])

            if ax == 0:
                return 0
            else:
                self.firstrun = False
            
        
        return self.update(np.array([[x], [y]]))
    
    


class KalmanPos2Vel2D():
    
    # 이거 쓰시면 됩니당

    def __init__(self, P0, x0, dt = 1/50, q1 = 1, q2 = 1, q3 = 10, q4 = 10, r = 5):

        self.x_esti = np.transpose(x0)
        self.P = P0
        self.A = np.array([[1, 0, dt, 0],
                           [0, 1, 0, dt],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
        
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])

        self.Q = np.diag([q1, q2, q3, q4])

        self.R = np.diag([r, r])

        self.dt = dt
        self.firstrun = True

    
    def update(self, z):
        x_p = np.matmul(self.A, self.x_esti)
        P_p = np.matmul(np.matmul(self.A, self.P), np.transpose(self.A)) + self.Q

        K = np.matmul(np.matmul(P_p, np.transpose(self.H)), inv(np.matmul(np.matmul(self.H, P_p), np.transpose(self.H)) + self.R))

        self.x_esti = x_p + np.matmul(K, z - np.matmul(self.H, x_p))
        self.P      = P_p - np.matmul(K, np.matmul(self.H, P_p))


        return 3.6 * np.sqrt(self.x_esti[2]**2 + self.x_esti[3]**2)
    

    def main(self, x, y, ax):

        if self.firstrun:
            # 초기값 -> 초기 측정값
            self.x_esti = np.array([[x], [y], [0], [0]])

            if ax == 0:
                return 0
            else:
                self.firstrun = False
            
        
        return self.update(np.array([[x], [y]]))



        

