#include "global_localization.h"

GlobalLocalization::GlobalLocalization(): Node("localization_node") {
    // 创建发布者
    pub_pc_in_map = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cur_scan_in_map", 1);
    pub_submap = this->create_publisher<sensor_msgs::msg::PointCloud2>("/submap", 1);
    pub_map_to_odom = this->create_publisher<nav_msgs::msg::Odometry>("/map_to_odom", 1);
    localization_success_pub = this->create_publisher<std_msgs::msg::Bool>("/localization_success", 1);

    // 创建订阅者
    sub_cloud_registered = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", 1, std::bind(&GlobalLocalization::cb_save_cur_scan, this, std::placeholders::_1));
    sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", 1, std::bind(&GlobalLocalization::cb_save_cur_odom, this, std::placeholders::_1));
    std::cout << "Succeed!" << std::endl;
}
GlobalLocalization::~GlobalLocalization()
{
}

    // 回调函数
void GlobalLocalization::cb_save_cur_scan(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 处理接收到的PointCloud2消息
}

void GlobalLocalization::cb_save_cur_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // 处理接收到的Odometry消息
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<GlobalLocalization>();
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
