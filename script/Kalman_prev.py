import numpy as np
from numpy.linalg import inv

class KalmanPos2Vel():
    
    def __init__(self, P0, x0, q1=1, q2=100, r=1000, dt=0.01):
        """칼만 필터 클래스 초기화 함수"""
        self.A = np.array([[1, dt],
                           [0, 1]])
        self.H = np.array([[1, 0]])
        self.Q = np.array([[q1, 0],
                           [0, q2]])
        self.R = np.array([[r]])
        self.P = P0
        self.dt = dt
        self.x_esti = x0
        self.firstrun = True
        
    def update(self, z_meas, dt):
        self.A[0,1] = dt
        x_pred = np.matmul(self.A, self.x_esti)
        P_pred = np.matmul(np.matmul(self.A, self.P), self.A.T) + self.Q
        K = np.matmul(P_pred,np.matmul(self.H.T, inv(np.matmul(np.matmul(self.H,P_pred), self.H.T) + self.R)))
        self.x_esti = x_pred + np.matmul(K, (z_meas - np.matmul(self.H, x_pred)))
        self.P = P_pred - np.matmul(K, np.matmul(self.H, P_pred))

        return self.x_esti[1][0]*3.6, self.P

    def main(self, x, y):
        if self.firstrun:
            self.x_esti = np.array([[x], [y], [0], [0]])
            self.firstrun = False
        
        vx, vy = self.update(np.array([[x], [y]]))
        return vx, vy
