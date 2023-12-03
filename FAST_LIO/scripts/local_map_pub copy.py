#!/usr/bin/env python3
from sensor_msgs.msg import PointCloud2,PointField
from pathlib import Path
import sys
import os
import rclpy 
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import struct
import numpy as np
import open3d as o3d
import pcl
# 获取当前文件的路径
current_file_path = Path(__file__).resolve()

# 获取上三级目录
three_levels_up = current_file_path.parents[3]

# 定义文件名
file_name = 'src/FAST_LIO/PCD/scans.pcd'  # 替换为实际的文件名

# 使用 os.path.join 组合路径
pcd_path = os.path.join(three_levels_up,file_name)

class PCDPublisher(Node):
    def __init__(self):
        super().__init__('pcd_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'map3d', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        pcd = o3d.io.read_point_cloud(pcd_path)
        ros_cloud = self.o3d_to_ros(pcd)
        self.publisher_.publish(ros_cloud)
        self.get_logger().info('Publishing PointCloud2')

    def o3d_to_ros(self,pcd):
        """ Converts an Open3D PointCloud to a ROS PointCloud2 message """
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors) * 255  # Scale colors to 0-255 range

        ros_msg = PointCloud2()
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        ros_msg.header.frame_id = "map"

        ros_msg.height = 1
        ros_msg.width = points.shape[0]

        ros_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]

        ros_msg.is_bigendian = False
        ros_msg.point_step = 16
        ros_msg.row_step = 16 * ros_msg.width
        ros_msg.is_dense = True

        buffer = []
        # 通过struct模块将RGB颜色值打包为32位整数。
        # 然而，Python中的struct模块对字节顺序敏感，而ROS中通常使用小端字节顺序。
        for i in range(points.shape[0]):
            x, y, z = points[i]
        if pcd.has_colors():
            r, g, b = colors[i]
        else:
            r, g, b = 255, 255, 255

            rgb = struct.unpack('I', struct.pack('BBBB', int(b), int(g), int(r), 255))[0]
            buffer.append([x, y, z, rgb])

        ros_msg.data = np.array(buffer, dtype=np.float32).tobytes()

        return ros_msg

def main(args=None):
    rclpy.init(args=args)
    pcd_publisher = PCDPublisher()
    rclpy.spin(pcd_publisher)
    pcd_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
