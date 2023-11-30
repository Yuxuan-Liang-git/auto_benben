#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from pathlib import Path
import open3d as o3d
import numpy as np
import os

# 获取当前文件的路径
current_file_path = Path(__file__).resolve()

# 获取上三级目录
three_levels_up = current_file_path.parents[4]

# 定义文件名
file_name = 'src/FAST_LIO/PCD/scans.pcd'  # 替换为实际的文件名

# 使用 os.path.join 组合路径
pcd_path = os.path.join(three_levels_up,file_name)



class PCDPublisher(Node):
    def __init__(self):
        super().__init__('pcd_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'point_cloud', 10)
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        pcd = o3d.io.read_point_cloud(pcd_path)

        # Convert Open3D point cloud to numpy array
        point_cloud_data = np.asarray(pcd.points)

        pc_msg = PointCloud2()
        pc_msg.header = Header()
        pc_msg.header.stamp = self.get_clock().now().to_msg()

        # Create PointCloud2 message
        pc_msg = pc2.create_cloud_xyz32(header=pc_msg.header,
                                         points=point_cloud_data)
        
        # 发布点云数据
        self.publisher_.publish(pc_msg)
        self.get_logger().info('Published point cloud')

def main(args=None):
    rclpy.init(args=args)
    pcd_publisher = PCDPublisher()
    rclpy.spin(pcd_publisher)
    pcd_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






