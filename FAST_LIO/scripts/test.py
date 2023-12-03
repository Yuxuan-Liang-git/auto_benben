import pyntcloud
import rclpy
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
from pathlib import Path
import os

# 获取当前文件的路径
current_file_path = Path(__file__).resolve()

# 获取上三级目录
three_levels_up = current_file_path.parents[3]

# 定义文件名
file_name = 'src/FAST_LIO/PCD/scans.pcd'  # 替换为实际的文件名

# 使用 os.path.join 组合路径
pcd_path = os.path.join(three_levels_up,file_name)


def pcd_to_pointcloud2(pcd):
    header = std_msgs.msg.Header()
    header.stamp = rclpy.clock.Clock().now().to_msg()
    header.frame_id = "base_link"  # 修改为你的坐标系

    points = pcd.xyz
    intensity = pcd.points["intensity"]  # 使用强度信息字段

    fields = [
        PointCloud2.create_cloud_field('x', 0, PointCloud2.PointField.FLOAT32, 1),
        PointCloud2.create_cloud_field('y', 4, PointCloud2.PointField.FLOAT32, 1),
        PointCloud2.create_cloud_field('z', 8, PointCloud2.PointField.FLOAT32, 1),
        PointCloud2.create_cloud_field('intensity', 12, PointCloud2.PointField.FLOAT32, 1),
    ]

    pc2 = PointCloud2.create_cloud_xyz32(header, fields, points)
    pc2.header = header
    pc2.height = 1
    pc2.width = len(points)

    return pc2

def main():
    rclpy.init()

    node = rclpy.create_node('pointcloud_publisher')

    pcd = pyntcloud.PyntCloud.from_file(pcd_path)

    publisher = node.create_publisher(PointCloud2, 'pointcloud_topic', 10)

    while rclpy.ok():
        pc2_msg = pcd_to_pointcloud2(pcd)
        publisher.publish(pc2_msg)
        node.get_logger().info('PointCloud published')
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
