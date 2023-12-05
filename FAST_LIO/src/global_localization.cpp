#include "global_localization.h"

GlobalLocalization::GlobalLocalization(): Node("localization_node")
{
    
    // 创建发布者
    pub_submap = this->create_publisher<sensor_msgs::msg::PointCloud2>("/submap", 1);
    pub_map_to_odom = this->create_publisher<nav_msgs::msg::Odometry>("/map_to_odom", 1);
    localization_success_pub = this->create_publisher<std_msgs::msg::Bool>("/localization_success", 1);

    // 创建订阅者
    sub_cloud_registered = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", 1, std::bind(&GlobalLocalization::cb_save_cur_scan, this, std::placeholders::_1));
    sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", 1, std::bind(&GlobalLocalization::cb_save_cur_odom, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Create pub&sub Succeed");

    this->global_pc_init();

}
GlobalLocalization::~GlobalLocalization()
{
}

    // 回调函数
void GlobalLocalization::cb_save_cur_scan(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg) {
    // 处理接收到的PointCloud2消息
    // 将ROS的PointCloud2转换为PCL的PointCloud并存储在类内
    // RCLCPP_INFO(this->get_logger(), "PointCloud Received!!");   
    pcl::PointCloud<pcl::PointNormal>::Ptr cur_scan_pc (new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg(*pc_msg, *cur_scan_pc);

}

void GlobalLocalization::cb_save_cur_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // 处理接收到的Odometry消息
    RCLCPP_INFO(this->get_logger(),"Odometry received!");
    T_map_to_odom.translation() << 
        msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z;

}

void GlobalLocalization::global_pc_init()
{   
    string file_name = string("scans.pcd");
    string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);

    pcl::PCDReader pcd_reader;
    pcl::PCLPointCloud2::Ptr temp_pc (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr temp_filtered_pc (new pcl::PCLPointCloud2 ());
    pcl::PointCloud<pcl::PointNormal>::Ptr global_pc (new pcl::PointCloud<pcl::PointNormal>);

    if (pcd_reader.read(all_points_dir,*temp_pc) != 0)  {
        RCLCPP_ERROR(this->get_logger(), "Error loading point cloud: %s", file_name.c_str());
    }
    else{
        // 体素体降采样
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(temp_pc);
        sor.setLeafSize(MAP_VOXEL_SIZE,MAP_VOXEL_SIZE,MAP_VOXEL_SIZE);
        sor.filter(*temp_filtered_pc);
        pcl::fromPCLPointCloud2 (*temp_filtered_pc, *global_pc);
        RCLCPP_INFO(this->get_logger(),"global_pc_init succeed!!");
    }

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
