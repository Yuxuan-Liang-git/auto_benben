#include "global_localization.h"

GlobalLocalization::GlobalLocalization(): Node("localization_node")
{
    odom_received_ = false;
    global_pc_received_ = false;
    locate_state_.data = false;
    T_odom2body = Eigen::Affine3f::Identity();
    T_map2odom = Eigen::Affine3f::Identity();
    this->global_pc_init();
    // 创建发布者
    pub_submap = this->create_publisher<sensor_msgs::msg::PointCloud2>("/submap", 1);
    pub_odom_in_map = this->create_publisher<nav_msgs::msg::Odometry>("/odom_in_map", 1);
    pub_locate_state = this->create_publisher<std_msgs::msg::Bool>("/locate_state", 1);

    // 创建订阅者
    sub_cloud_registered = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", 1, std::bind(&GlobalLocalization::cb_save_cur_scan, this, std::placeholders::_1));
    sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", 1, std::bind(&GlobalLocalization::cb_save_cur_odom, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Create pub&sub Succeed");

    timer = this->create_wall_timer(
        500ms,
        std::bind(&GlobalLocalization::cb_locate_odom, this));
    

}
GlobalLocalization::~GlobalLocalization()
{
}

void GlobalLocalization::timer_callback()
{
        std::cout << "-------timer callback!-----------" << std::endl;
        // do something else;

} 

// 滤去可视范围外保存的点云，即雷达当前位置应该看到什么样子的点云数据
void GlobalLocalization::get_global_map_in_FOV()
{
    // 将map保存的点云转到lidar坐标系下
    pcl::PointCloud<pcl::PointNormal>::Ptr temp_pc(new pcl::PointCloud<pcl::PointNormal> ());
    pcl::transformPointCloudWithNormals(*global_pc, *temp_pc, T_odom2body*T_map2odom);
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
	cr.setKeepOrganized(false);			//不保留无效点云数据的位置信息
	cr.filter(*global_pc_in_FOV);				//执行滤波，保存滤波结果于global_pc_in_FOV

    // sensor_msgs::msg::PointCloud2 laserCloudmsg;
    // pcl::toROSMsg(*global_pc_in_FOV, laserCloudmsg);
    // laserCloudmsg.header.stamp = this->get_clock()->now();
    // laserCloudmsg.header.frame_id = "body";
    // pub_submap->publish(laserCloudmsg);
}
   
// 回调函数
void GlobalLocalization::cb_save_cur_scan(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg) {
    // 处理接收到的PointCloud2消息
    // 将ROS的PointCloud2转换为PCL的PointCloud并存储在类内
    // 此外初始化T_map2odom
    // RCLCPP_INFO(this->get_logger(), "PointCloud Received!!");   
    pcl::fromROSMsg(*pc_msg, *cur_scan_pc);
    if(!global_pc_received_){
        Eigen::Matrix4f coarse_transformation = sacCoarseRegister(cur_scan_pc,global_pc);
        RegistrateResult registrate_fine_result = icpFineRegister(cur_scan_pc,global_pc,coarse_transformation);
        T_map2odom = registrate_fine_result.transformation;
        RCLCPP_INFO(this->get_logger(), "Fitness score of the init T_map2odom %f", registrate_fine_result.fitness_score);
        global_pc_received_ = true;
    }
}

void GlobalLocalization::cb_save_cur_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // 处理接收到的Odometry消息

    T_odom2body.translation() << 
        (float) msg->pose.pose.position.x,
        (float) msg->pose.pose.position.y,
        (float) msg->pose.pose.position.z;

    Eigen::Quaternionf quat(
        (float) msg->pose.pose.orientation.x,
        (float) msg->pose.pose.orientation.y,
        (float) msg->pose.pose.orientation.z,
        (float) msg->pose.pose.orientation.w);
    T_odom2body.rotate(quat);

    if(!odom_received_){
        odom_received_ = true;
        RCLCPP_INFO(this->get_logger(),"Odometry received!");}

}

void GlobalLocalization::cb_locate_odom()
{
    if(odom_received_&global_pc_received_)
    {
        // 深拷贝并进行体素体降采样
        pcl::PointCloud<pcl::PointNormal>::Ptr cur_scan_pc_copy,scan_tobe_mapped;
        pcl::PCLPointCloud2::Ptr temp_pc (new pcl::PCLPointCloud2 ());
        pcl::PCLPointCloud2::Ptr temp_filtered_pc (new pcl::PCLPointCloud2 ());
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;

        scan_tobe_mapped = std::make_shared<pcl::PointCloud<pcl::PointNormal>>(*cur_scan_pc);
        // cur_scan_pc_copy = std::make_shared<pcl::PointCloud<pcl::PointNormal>>(*cur_scan_pc);
        pcl::toPCLPointCloud2(*scan_tobe_mapped, *temp_pc);
        sor.setInputCloud(temp_pc);
        sor.setLeafSize(MAP_VOXEL_SIZE,MAP_VOXEL_SIZE,MAP_VOXEL_SIZE);
        sor.filter(*temp_filtered_pc);
        pcl::fromPCLPointCloud2 (*temp_filtered_pc, *scan_tobe_mapped);

        // 滤去可视范围外的保存的点云地图
        get_global_map_in_FOV();
        Eigen::Matrix4f coarse_transformation = sacCoarseRegister(scan_tobe_mapped,global_pc_in_FOV);
        RegistrateResult registrate_fine_result = icpFineRegister(scan_tobe_mapped,global_pc_in_FOV,coarse_transformation);
        RCLCPP_INFO(this->get_logger(),"locate_odom fitness:%f",registrate_fine_result.fitness_score);

        // 全局定位成功才更新map2odom
        if(registrate_fine_result.fitness_score < AVERAGE_DISTANTCE)
        {
            T_map2odom = registrate_fine_result.transformation;
            nav_msgs::msg::Odometry odom_in_map;
            // 提取平移
            odom_in_map.pose.pose.position.x = T_map2odom(0, 3);
            odom_in_map.pose.pose.position.y = T_map2odom(1, 3);
            odom_in_map.pose.pose.position.z = T_map2odom(2, 3);

            // 提取旋转（假设矩阵是齐次变换矩阵）
            Eigen::Quaternionf quat(Eigen::Matrix3f(T_map2odom.block<3, 3>(0, 0)));
            odom_in_map.pose.pose.orientation.x = quat.x();
            odom_in_map.pose.pose.orientation.y = quat.y();
            odom_in_map.pose.pose.orientation.z = quat.z();
            odom_in_map.pose.pose.orientation.w = quat.w();
            
            odom_in_map.header.frame_id = "map";
            odom_in_map.header.stamp = this->get_clock()->now();

            pub_odom_in_map->publish(odom_in_map);
            RCLCPP_INFO(this->get_logger(),"Locate succeed,fitness score: %f",registrate_fine_result.fitness_score);
            locate_state_.data = true;
        }
        // else
        // {
        //     RCLCPP_ERROR(this->get_logger(),"Locate failed,fitness score: %f",registrate_fine_result.fitness_score);
        //     locate_state_.data = false;
        // }
        // pub_locate_state->publish(locate_state_);
    }
}

void GlobalLocalization::global_pc_init()
{   
    string file_name = string("scans.pcd");
    string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);

    pcl::PCDReader pcd_reader;
    pcl::PCLPointCloud2::Ptr temp_pc (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr temp_filtered_pc (new pcl::PCLPointCloud2 ());
    global_pc = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    global_pc_in_FOV = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    cur_scan_pc = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();

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

Eigen::Matrix4f GlobalLocalization::sacCoarseRegister
    (const pcl::PointCloud<pcl::PointNormal>::Ptr& source,
    const pcl::PointCloud<pcl::PointNormal>::Ptr& target) 
{
    // 计算FPFH特征
    pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features(new pcl::PointCloud<pcl::FPFHSignature33>());
    fpfh.setInputCloud(source);
    fpfh.setInputNormals(source);
    fpfh.setRadiusSearch(0.05);
    fpfh.compute(*source_features);
    fpfh.setInputCloud(target);
    fpfh.setInputNormals(target);
    fpfh.compute(*target_features);
    // SAC配准
    pcl::SampleConsensusPrerejective<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> sac;
    sac.setInputSource(source);
    sac.setSourceFeatures(source_features);
    sac.setInputTarget(target);
    sac.setTargetFeatures(target_features);
    pcl::PointCloud<pcl::PointNormal> aligned;
    sac.align(aligned);
    RCLCPP_INFO(this->get_logger(),"sacCoarseRegister fitness score: %f",sac.getFitnessScore());

    return sac.getFinalTransformation();
}

GlobalLocalization::RegistrateResult GlobalLocalization::icpFineRegister
    (const pcl::PointCloud<pcl::PointNormal>::Ptr& source,
    const pcl::PointCloud<pcl::PointNormal>::Ptr& target,
    const Eigen::Matrix4f &initial_guess) 
{
    pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    pcl::PointCloud<pcl::PointNormal> result;
    icp.align(result,initial_guess);
    RegistrateResult temp_result;
    temp_result.transformation = icp.getFinalTransformation();
    temp_result.fitness_score = icp.getFitnessScore();
    RCLCPP_INFO(this->get_logger(),"icpFineRegister fitness score: %f",icp.getFitnessScore());
    return temp_result;
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
