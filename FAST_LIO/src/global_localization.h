#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/features/fpfh.h>

#include <iostream>
#include <string>

using namespace std;
const float MAP_VOXEL_SIZE = 0.4;
const float FREQ_LOCALIZATION = 0.5;
const float AVERAGE_DISTANTCE = 2;
const float FOV_FAR = 70.0;

class GlobalLocalization: public rclcpp::Node
{
  public:
    struct RegistrateResult{
      Eigen::Affine3f transformation;
      double fitness_score; 
    };
 
    GlobalLocalization();
    ~GlobalLocalization();

private:
    void cb_save_cur_scan(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg);
    void cb_save_cur_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
    void cb_locate_odom();
    void global_pc_init();
    void get_global_map_in_FOV();
    void locate();
    void voxelDownSample(const pcl::PointCloud<pcl::PointNormal>::Ptr& source_pc,
            const pcl::PointCloud<pcl::PointNormal>::Ptr& filtered_pc,float leafsize);
            
    bool odom_received_;
    bool odom_init_;  //  全局点云匹配标识符，匹配后置1，只匹配当前FOV内的点云；
    bool global_pc_received_;

    std_msgs::msg::Bool locate_state_;
    
    Eigen::Matrix4f sacCoarseRegister(
      const pcl::PointCloud<pcl::PointNormal>::Ptr& source,
      const pcl::PointCloud<pcl::PointNormal>::Ptr& target); 

    RegistrateResult icpFineRegister(
      const pcl::PointCloud<pcl::PointNormal>::Ptr& source,
      const pcl::PointCloud<pcl::PointNormal>::Ptr& target,
      const Eigen::Matrix4f &initial_guess);

    pcl::PointCloud<pcl::PointNormal>::Ptr cur_scan_pc;
    pcl::PointCloud<pcl::PointNormal>::Ptr global_pc;
    pcl::PointCloud<pcl::PointNormal>::Ptr global_pc_in_FOV;
    // 当前点相对于初始点的相对变换
    Eigen::Affine3f T_odom2body;
    // 当前点相对于地图原点的变换矩阵
    Eigen::Affine3f T_map2odom;
    rclcpp::TimerBase::SharedPtr timer;

    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_submap;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_in_map;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_locate_state;

    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_registered;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry;

    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
};
