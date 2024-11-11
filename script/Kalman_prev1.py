import numpy as np
from numpy.linalg import inv

class KalmanPos2Vel1():
    
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
        self.P = P_p - np.matmul(K, np.matmul(self.H, P_p))
        
        vx = self.x_esti[2, 0]
        vy = self.x_esti[3, 0]
        return vx, vy

    def main(self, x, y):
        if self.firstrun:
            self.x_esti = np.array([[x], [y], [0], [0]])
            self.firstrun = False
        
        vx, vy = self.update(np.array([[x], [y]]))
        return vx, vy
