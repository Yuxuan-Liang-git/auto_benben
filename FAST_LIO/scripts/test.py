import rclpy
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Header
from tf2_ros import TransformStamped
import tf2_geometry_msgs
import tf2_py as tf2
import numpy as np

def create_pose_with_quaternion():
    # 创建 Quaternion
    quaternion = Quaternion()
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = 0.0
    quaternion.w = 1.0  # 假设无旋转，因此 w 设置为 1.0

    # 创建 Pose
    pose = Pose()
    pose.orientation = quaternion
    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = 0.0

    return pose

def create_transform():
    # 创建 TransformStamped
    transform = TransformStamped()
    transform.header = Header()
    transform.header.stamp = rclpy.clock.Clock().now().to_msg()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'base_link'
    transform.transform.translation.x = 0.0
    transform.transform.translation.y = 1.0
    transform.transform.translation.z = 0.0
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0
    transform.transform.rotation.w = 1.0

    return transform

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('tf_transform_example_node')

    original_pose = create_pose_with_quaternion()
    transform = create_transform()

    # 调用 do_transform_pose 函数
    transformed_pose = tf2_geometry_msgs.do_transform_pose(original_pose, transform)

    # 打印结果
    node.get_logger().info(f'Original Pose: {original_pose}')
    node.get_logger().info(f'Transformed Pose: {transformed_pose}')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
