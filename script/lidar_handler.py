#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2
import message_filters
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import ros_numpy
import numpy as np
import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as orh
from sklearn.cluster import DBSCAN

class LidarHandler():
    def __init__(self):
        rospy.init_node('dku_morai_lidar_hander')
        rospy.Subscriber('/imu', Imu, self.cb_imu, queue_size=1)
        rospy.Subscriber('/velodyne_points', PointCloud2, self.cb_lidar, queue_size=1)
        
        self.dist_pub = rospy.Publisher('/dist_ahead', Float32, queue_size=1)
        self.pc_pub = rospy.Publisher('/grd_removed_points', PointCloud2, queue_size=1)
        self.dbscan = DBSCAN(eps = 0.2, min_samples=10)

    def main(self):
        loop_hz = 10
        rate = rospy.Rate(loop_hz)
        rospy.logwarn('lidar_handler: waiting for lidar packet')
        rospy.wait_for_message('/velodyne_points',PointCloud2)
        rospy.logwarn('lidar_handler: waiting for imu packet')
        rospy.wait_for_message('/imu',Imu)
        rospy.loginfo('lidar_handler: begin')
        dist_ahead=0
        while not rospy.is_shutdown():
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(self.xyz)
            R = pcd.get_rotation_matrix_from_xyz([self.roll, self.pitch, 0])  
            
            pcd.rotate(R, center=(0, 0, 0))
            xyz = np.asarray(pcd.points)
            # 0 : y / 1 : x / 2 : z
            xyz = xyz[(xyz[:,0]<10) & (xyz[:,0]>-0) & (xyz[:,1]>-1) & (xyz[:,1]<1) & (xyz[:,2]>-1.2) & (xyz[:,2]<0)]
            xyz[(xyz[:,0] <2.5) & (xyz[:,0]>-2.5) & (xyz[:,1] <1.5) & (xyz[:,1]>-1.5)] = 0
            xyz = xyz[~np.all(xyz==0, axis=1)]
            removed_pcd = o3d.geometry.PointCloud()
            removed_pcd.points = o3d.utility.Vector3dVector(xyz)
            
            ros_pcd = orh.o3dpc_to_rospc(removed_pcd, 'velodyne')
            self.pc_pub.publish(ros_pcd)
            self.removed_xy = np.asarray(removed_pcd.points)[:, :2]
            if len(self.removed_xy)!=0:
                db = self.dbscan.fit_predict(self.removed_xy)
                n_cluster = np.max(db) + 1
                
                # log_str = ''
                obs_list = []
                for c in range(n_cluster):
                    c_tmp = np.mean(self.removed_xy[db==c, :], axis=0)
                    radius = np.max(np.linalg.norm(c_tmp-self.removed_xy[db==c,:], axis=1))
                    if radius < 0.05 and radius > 0.2:
                        continue

                    obs_list.append([c_tmp[0], c_tmp[1]])
                    # log_str+=f'({c_tmp[0]:0.1f}, {c_tmp[1]:0.1f}, {radius:0.1f}) '
                obs_list = np.array(obs_list)
                if len(obs_list)!=0:
                    dist_ahead = np.mean(obs_list,axis=0)[0]
                else:
                    dist_ahead = 0
            else:
                dist_ahead=0
            self.dist_pub.publish(dist_ahead)
                    # rospy.loginfo(dist_ahead)

            rate.sleep()

    def cb_imu(self, data):
        self.quaternion = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        self.roll, self.pitch, self.yaw = euler_from_quaternion(self.quaternion)
        
    def cb_lidar(self, msg):
        pc_arr = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
        self.xyz = pc_arr[:, :3]

if __name__ == '__main__':
    lh = LidarHandler()
    lh.main()