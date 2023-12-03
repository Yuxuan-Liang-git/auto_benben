#!/usr/bin/env python3
import rclpy
import numpy as np
import sys

import open3d as o3d 
import time
import tf2_ros
import copy

import ros2_numpy

from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from concurrent.futures import Future

import tf2_geometry_msgs

MAP_VOXEL_SIZE = 0.4
# FOV(rad), modify this according to your LiDAR type
FOV = 6.28319
# The farthest distance(meters) within FOV
FOV_FAR = 70

class GlobalLocalization(Node):
    def __init__(self,name):
        super().__init__(name)
        self.curscan = None   
        self.wait_for_global_map = True
        self.submap_pub = self.create_publisher(PointCloud2,'submap', 1)
        self.map_to_odom_pub = self.create_publisher(Odometry,'map_to_odom', 1)
        self.localization_success_pub = self.create_publisher(Bool,'localization_success', 1)    
        self.save_cur_scan_sub = self.create_subscription(PointCloud2,"cloud_registered",self.save_cur_scan_cb,1)
        self.save_cur_odom_sub = self.create_subscription(Odometry,"Odometry",self.save_cur_odom_cb,1)
        self.map3d_sub = self.create_subscription(PointCloud2,"global_map",self.global_map_cb,10)


    def save_cur_scan_cb(self,pc_msg):
        # 注意这里fastlio直接将scan转到odom系下了 不是lidar局部系
        pc_msg.header.frame_id = 'camera_init'
        pc_msg.header.stamp = self.clock.now()
        self.pub_pc_in_map.publish(pc_msg)
        # 转换为pcd
        # fastlio给的field有问题 处理一下
        pc_msg.fields = [pc_msg.fields[0], pc_msg.fields[1], pc_msg.fields[2],
                        pc_msg.fields[4], pc_msg.fields[5], pc_msg.fields[6],
                        pc_msg.fields[3], pc_msg.fields[7]]
        pc = self.msg_to_array(pc_msg)

        self.cur_scan = o3d.geometry.PointCloud()
        self.cur_scan.points = o3d.utility.Vector3dVector(pc[:, :3])

    def msg_to_array(self,pc_msg):
        pc_array = ros2_numpy.numpify(pc_msg)
        pc = np.zeros([len(pc_array['xyz']), 3])
        pc[:, 0] = pc_array['xyz'][:, 0]
        pc[:, 1] = pc_array['xyz'][:, 1]
        pc[:, 2] = pc_array['xyz'][:, 2]
        return pc

    def save_cur_odom_cb(self,odom_msg):
        self.cur_odom = odom_msg
            
    def voxel_down_sample(self,pcd, voxel_size):
        try:
            pcd_down = pcd.voxel_down_sample(voxel_size)
        except:
            # for opend3d 0.7 or lower
            pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
        return pcd_down

    def global_map_cb(self,global_map):
        if self.wait_for_global_map:  
            self.global_map = global_map
            global_map = o3d.geometry.PointCloud()
            global_map.points = o3d.utility.Vector3dVector(self.msg_to_array(self.global_map))
            global_map = self.voxel_down_sample(global_map, MAP_VOXEL_SIZE)
            initial_pose = self.create_initial_pose_with_quaternion()
            self.global_localize(initial_pose)
            self.get_logger().info("Global_map received and localization_init!!")   
            self.wait_for_global_map = False        
        
    def pose_to_mat(self,pose):
        # 创建一个4x4的单位矩阵
        matrix = np.identity(4)

        # 提取位姿消息的位置和四元数
        position = pose.position
        orientation = pose.orientation

        # 将位置信息填充到矩阵的前三列
        matrix[0:3, 3] = [position.x, position.y, position.z]

        # 计算旋转矩阵部分
        rotation_matrix = tf2_py.transformations.quaternion_matrix([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        matrix[0:3, 0:3] = rotation_matrix[0:3, 0:3]

        return matrix

    def global_localize(self,pose_estimation):
        self.get_logger().info("Global localization by scan-to-map matching......")    
        # scan_tobe_mapped = copy.copy(self.cur_scan)

    def crop_global_map_in_FOV(self):
        # 当前scan原点的位姿
        T_odom_to_base_link = self.pose_to_mat(self.cur_odom)
        T_map_to_base_link = np.matmul(self.initial_pos, T_odom_to_base_link)
        T_base_link_to_map = self.inverse_se3(T_map_to_base_link)

        # 把地图转换到lidar系下
        global_map_in_map = np.array(self.global_map.points)
        global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
        global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

        # 将视角内的地图点提取出来
        if FOV > 3.14:
            # 环状lidar 仅过滤距离
            indices = np.where(
                (global_map_in_base_link[:, 0] < FOV_FAR) &
                (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
            )
        else:
            # 非环状lidar 保前视范围
            # FOV_FAR>x>0 且角度小于FOV
            indices = np.where(
                (global_map_in_base_link[:, 0] > 0) &
                (global_map_in_base_link[:, 0] < FOV_FAR) &
                (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
            )
        global_map_in_FOV = o3d.geometry.PointCloud()
        global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

        # 发布fov内点云
        header = self.cur_odom.header
        header.frame_id = 'map'
        self.publish_point_cloud(pub_submap, header, np.array(global_map_in_FOV.points)[::10])

        return global_map_in_FOV       

    def inverse_se3(self,trans):
        trans_inverse = np.eye(4)
        # R
        trans_inverse[:3, :3] = trans[:3, :3].T
        # t
        trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
        return trans_inverse

    def publish_point_cloud(self,publisher, header, pc):
        data = np.zeros(len(pc), dtype=[
            ('x', np.float32),
            ('y', np.float32),pc_msg
            ('z', np.float32),
            ('intensity', np.float32),
        ])
        data['x'] = pc[:, 0]
        data['y'] = pc[:, 1]
        data['z'] = pc[:, 2]
        if pc.shape[1] == 4:
            data['intensity'] = pc[:, 3]
        msg = ros2_numpy.msgify(PointCloud2, data)
        msg.header = header
        publisher.publish(msg)

    def create_initial_pose_with_quaternion(self):
        # 创建 Quaternion
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = 0.0
        quaternion.w = 1.0  # 此处假设无旋转，因此 w 设置为 1.0

        # 创建 Pose
        pose = Pose()
        pose.orientation = quaternion
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.0

        return pose
    
def main(args=None):

    SCAN_VOXEL_SIZE = 0.1

    # Global localization frequency (HZ)
    FREQ_LOCALIZATION = 0.5

    # The threshold of global localization,
    # only those scan2map-matching with higher fitness than LOCALIZATION_TH will be taken
    LOCALIZATION_TH = 0.93





    rclpy.init(args=args) # 初始化rclpy
    node = GlobalLocalization("fast_lio_localization")  # 新建一个节点

    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy   

if __name__ == '__main__':
    main()