#include "global_localization.h"

GlobalLocalization::GlobalLocalization(): Node("localization_node")
{
    odom_received_ = false;
    global_pc_received_ = false;
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
    timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000 / FREQ_LOCALIZATION)),
        std::bind(&GlobalLocalization::cb_locate_pos, this));

}
GlobalLocalization::~GlobalLocalization()
{
}

// 滤去可视范围外保存的点云
void GlobalLocalization::get_global_map_in_FOV()
{
    // 将保存的点云转到lidar坐标系下
    pcl::PointCloud<pcl::PointNormal>::Ptr temp_pc;
    pcl::transformPointCloud (*global_pc, *temp_pc, T_map_to_odom);

	//创建条件限定下的滤波器
	pcl::ConditionAnd<pcl::PointNormal>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointNormal>());//创建条件定义对象range_cond
	//为条件定义对象添加比较算子
	range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(new
		pcl::FieldComparison<pcl::PointNormal>("x", pcl::ComparisonOps::GT, -FOV_FAR)));//添加在x字段上大于 -.1 的比较算子
	range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(new
		pcl::FieldComparison<pcl::PointNormal>("x", pcl::ComparisonOps::LT, FOV_FAR)));//添加在x字段上小于 1.0 的比较算子
	range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(new
		pcl::FieldComparison<pcl::PointNormal>("y", pcl::ComparisonOps::GT, -FOV_FAR)));//添加在y字段上大于 -.1 的比较算子
	range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(new
		pcl::FieldComparison<pcl::PointNormal>("y", pcl::ComparisonOps::LT, FOV_FAR)));//添加在y字段上小于 1.0 的比较算子

	pcl::ConditionalRemoval<pcl::PointNormal> cr;	//创建滤波器对象
	cr.setCondition(range_cond);			//用条件定义对象初始化
	cr.setInputCloud(temp_pc);			//设置待滤波点云
	cr.setKeepOrganized(false);			//保持点云结构，即有序点云经过滤波后，仍能够保持有序性。(是否保留无效点云数据的位置信息)
	
	cr.filter(*global_pc_in_FOV);				//执行滤波，保存滤波结果于cloud_filtered

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*global_pc_in_FOV, laserCloudmsg);
    laserCloudmsg.header.stamp = this->get_clock()->now();
    laserCloudmsg.header.frame_id = "body";

    pub_submap->publish(laserCloudmsg)
}
   
// 回调函数
void GlobalLocalization::cb_save_cur_scan(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg) {
    // 处理接收到的PointCloud2消息
    // 将ROS的PointCloud2转换为PCL的PointCloud并存储在类内
    // RCLCPP_INFO(this->get_logger(), "PointCloud Received!!");   
    if(!global_pc_received_){
        global_pc_received_ = true;
    }
    pcl::PointCloud<pcl::PointNormal>::Ptr cur_scan_pc (new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg(*pc_msg, *cur_scan_pc);

}

void GlobalLocalization::cb_save_cur_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // 处理接收到的Odometry消息
    RCLCPP_INFO(this->get_logger(),"Odometry received!");
    if(!odom_received_){
        odom_received_ = true;
    }
    T_map_to_odom.translation() << 
        msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z;

    Eigen::Quaterniond quat
                (msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
    T_map_to_odom.rotate(quat);
}

void GlobalLocalization::cb_locate_pos()
{
    if(odom_received_&global_pc_received_){
        RCLCPP_INFO(this->get_logger(),"Global localization by scan-to-map matching......");
        get_global_map_in_FOV();
    }
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
