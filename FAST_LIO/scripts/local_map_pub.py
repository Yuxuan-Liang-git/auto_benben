import rclpy
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import Header
import pyntcloud
from pathlib import Path
import os
import numpy as np
from rclpy.node import Node
# 获取当前文件的路径
current_file_path = Path(__file__).resolve()

# 获取上三级目录
three_levels_up = current_file_path.parents[3]

# 定义文件名
file_name = 'src/FAST_LIO/PCD/scans.pcd'  # 替换为实际的文件名

# 使用 os.path.join 组合路径
pcd_path = os.path.join(three_levels_up,file_name)

class PCDPublisherNode(Node):
    def __init__(self):
        super().__init__('pcd_publisher')
        self.publisher = self.create_publisher(PointCloud2, 'pcd_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # Set the timer interval (1.0 second in this example)
        self.pcd_path = pcd_path
    def timer_callback(self):
        pcl_msg = self.pcd_to_ros2_pointcloud2()
        self.publisher.publish(pcl_msg)
        self.get_logger().info('Publishing PointCloud2')

    def pcd_to_ros2_pointcloud2(self):
        # Load PCD file using pyntcloud
        cloud = pyntcloud.PyntCloud.from_file(self.pcd_path)

        # Extract point cloud data
        points = cloud.points

        # Create PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"  # Change the frame_id as needed

        pcl_msg = PointCloud2()
        pcl_msg.header = header
        pcl_msg.height = 1
        pcl_msg.width = len(points)
        pcl_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="normal_x", offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name="normal_y", offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name="normal_z", offset=24, datatype=PointField.FLOAT32, count=1),
            PointField(name="curvature", offset=28, datatype=PointField.FLOAT32, count=1),
        ]

        pcl_msg.is_bigendian = False
        pcl_msg.point_step = 32
        pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width
        pcl_msg.is_dense = True

        # Flatten the points array and pack it into the PointCloud2 message
        pcl_msg.data = np.array(points, dtype=np.float32).tobytes()

        return pcl_msg

def main(args=None):
    rclpy.init(args=args)
    node = PCDPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
