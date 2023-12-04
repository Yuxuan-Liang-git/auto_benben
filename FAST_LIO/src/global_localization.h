#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define MAP_VOXEL_SIZE = 0.4;
#define SCAN_VOXEL_SIZE = 0.1;
#define FREQ_LOCALIZATION = 0.5;
#define LOCALIZATION_TH = 0.93;
#define FOV = 6.28319;  // 360 degrees in radians
#define FOV_FAR = 70.0;

class GlobalLocalization: public rclcpp::Node
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GlobalLocalization();
  ~GlobalLocalization();

private:
    void cb_save_cur_scan(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg);
    void cb_save_cur_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
    pcl::PointCloud<pcl::PointXYZ> cur_scan_pc;

    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_in_map;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_submap;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_map_to_odom;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr localization_success_pub;

    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_registered;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry;


};
