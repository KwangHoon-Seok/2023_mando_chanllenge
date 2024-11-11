import numpy as np
import csv
import os
import rospkg
# 필요한 모듈 및 클래스 임포트
from PID import PIDController as PID

class RefTracker():
    def __init__(self, dt=1/50):
        self.dt = dt
        self.idx = 0
        self.stanley_k = 0.5
        self.stanley_error = 0
        self.vehicle_sensor_x = 1.86
        self.LOOK_AHEAD_DIST = 0.1
        self.slope_heading = 0
        self.ref_path = np.array([])

    def do(self, ego, drive):
        if ego.x == 0:
            return 0, 5, 0

        self.target_x, self.target_y = self.calc_ahead_point(ego)
        self.idx, theta = self.calc_nearest_point(self.target_x, self.target_y)
        theta = (theta + np.pi) % (2 * np.pi) - np.pi  # normalize_pi 사용

        dx, dy = ego.x - self.target_x, ego.y - self.target_y

        self.calc_slope_of_path()

        cte = -np.dot([dx, dy], [np.cos(ego.heading + np.pi / 2), np.sin(ego.heading + np.pi / 2)])

        cross_track_steering = np.arctan(self.stanley_k * cte / (ego.velocity + 1e-6))

        heading_error = self.normalize_pi(self.slope_heading - ego.heading)

        self.stanley_error = self.normalize_pi(cross_track_steering + heading_error)

        velocity = self.get_velocity(drive)
        steer = self.stanley_error
        print(ego.velocity)
        return steer, velocity, self.idx

    def calc_ahead_point(self, ego):
        dx = self.LOOK_AHEAD_DIST * np.cos(ego.heading)
        dy = self.LOOK_AHEAD_DIST * np.sin(ego.heading)

        ahead_x = ego.x + dx
        ahead_y = ego.y + dy

        return ahead_x, ahead_y

    def calc_slope_of_path(self):
        if len(self.ref_path) < 2:
            return
        idx_1 = self.idx
        idx_2 = (idx_1 + 1) % len(self.ref_path)

        dx = self.ref_path[idx_2, 0] - self.ref_path[idx_1, 0]
        dy = self.ref_path[idx_2, 1] - self.ref_path[idx_1, 1]

        self.slope_heading = np.arctan2(dy, dx)

    def calc_nearest_point(self, ahead_x, ahead_y):
        if len(self.ref_path) == 0:
            return 0, 0
        distances = np.sum((self.ref_path - (ahead_x, ahead_y)) ** 2, axis=1)
        closest_index = np.argmin(distances)

        idx_2 = (closest_index + 1) % len(self.ref_path)
        dx = self.ref_path[idx_2, 0] - self.ref_path[closest_index, 0]
        dy = self.ref_path[idx_2, 1] - self.ref_path[closest_index, 1]

        theta = np.arctan2(dy, dx)

        return closest_index, theta

    def set_ref_path(self, filename):
        rospack = rospkg.RosPack()
        filepath = os.path.join(rospack.get_path('dku_morai_driver'), f'ref_path/{filename}.csv')
        self.ref_path = []
        with open(filepath, newline='') as csvfile:
            path_reader = csv.reader(csvfile, delimiter=',', quotechar='|')
            
            for row in path_reader:
                self.ref_path.append([float(row[0]), float(row[1])])
        self.ref_path = np.array(self.ref_path)
        return self.ref_path

    def get_velocity(self, drive):
        if drive:
            return 20  # 임의의 속도 값
        else:
            return 0

    def normalize_pi(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

