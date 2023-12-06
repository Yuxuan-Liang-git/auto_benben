#include <pcl/io/pcd_io.h>
#include <ctime>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/fpfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/fpfh_omp.h> //包含fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>


using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
	//法向量
	pointnormal::Ptr point_normal(new pointnormal);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal;
	est_normal.setInputCloud(input_cloud);
	est_normal.setSearchMethod(tree);
	est_normal.setKSearch(10);
	//est_normal.setRadiusSearch(0.03); 
    est_normal.compute(*point_normal);//计算法向量

	//fpfh 估计
	fpfhFeature::Ptr fpfh(new fpfhFeature);
	//pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> est_target_fpfh;
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
	est_fpfh.setNumberOfThreads(8); //指定4核计算
	// pcl::search::KdTree<pcl::PointXYZ>::Ptr tree4 (new pcl::search::KdTree<pcl::PointXYZ> ());
	est_fpfh.setInputCloud(input_cloud);
	est_fpfh.setInputNormals(point_normal);
	est_fpfh.setSearchMethod(tree);
	est_fpfh.setKSearch(10);
	//est_fpfh.setRadiusSearch(0.08); 
	est_fpfh.compute(*fpfh);

	return fpfh;

}

pointcloud::Ptr voxelGrid(pointcloud::Ptr cloud_in)
{
    pointcloud::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    //down sample
    std::cout << "begin downSample, cloud size: " << cloud_in->size() << std::endl;
    pcl::VoxelGrid<pcl::PointXYZ> downSampled;  //创建滤波对象
    downSampled.setInputCloud(cloud_in);            //设置需要过滤的点云给滤波对象
    downSampled.setLeafSize(0.5f, 0.5f, 0.5f);  //设置滤波时创建的体素体积为1cm的立方体（1为米，0.01就是1cm）
    downSampled.filter(*cloud_out);  //执行滤波处理，存储输出
    std::cout << "success downSample, cloud size: " << cloud_out->size() << std::endl;
    return cloud_out;
}

int main(int argc, char** argv)
{
    clock_t start, end, time;
	start = clock();
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_voxel(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_voxel(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_align(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPLYFile ("1.ply", *source);
//    pcl::io::loadPLYFile ("2.ply", *target);
//    pcl::io::loadPCDFile<pcl::PointXYZ>("1.pcd", *source);
//    pcl::io::loadPCDFile<pcl::PointXYZ>("2.pcd", *target);


    if(argc != 3)
    {
        cerr<<"输入点云数量不对!"<<endl;
        exit(1);
    }

    string input_filename = argv[1];
    string output_filename = argv[2];

    std::string format = input_filename.substr(input_filename.length()-4, 4);
    //std::cout<<"pointcloud format:"<<format<<std::endl;
    if(format == ".ply")
    {
        pcl::io::loadPLYFile(input_filename, *source);
        pcl::io::loadPLYFile(output_filename, *target);
    }
    else if(format == ".pcd")
    {
        pcl::io::loadPCDFile(input_filename, *source);
        pcl::io::loadPCDFile(output_filename, *target);
    }

    std:vector<int> index;
	pcl::removeNaNFromPointCloud(*source, *source, index);
	pcl::removeNaNFromPointCloud(*target, *target, index);

    //先对原始点云进行简化，对简化后的数据做配准计算, 将所获得的配准参数应用到原始点云，以提高计算效率
    source_voxel = voxelGrid(source);
    target_voxel = voxelGrid(target);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    //计算滤波后的特征点
    fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source_voxel, tree);
    fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target_voxel, tree);
//    end = clock();
//    cout << "calculate time is: " << float(end - start) / CLOCKS_PER_SEC << endl;

	//对齐(占用了大部分运行时间)
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(source_voxel);
	sac_ia.setSourceFeatures(source_fpfh);
    sac_ia.setInputTarget(target_voxel);
    sac_ia.setTargetFeatures(target_fpfh);
    pointcloud::Ptr align(new pointcloud);//对齐后的点云

    //对齐参数设置
//    sac_ia.setNumberOfSamples(20);//设置每次迭代计算中使用的样本数量（可省）,可节省时间
//    sac_ia.setCorrespondenceRandomness(50);//设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
//    //sac_ia.setMaximumIterations(100);
//    sac_ia.setEuclideanFitnessEpsilon(0.001);
//    sac_ia.setTransformationEpsilon(1e-10);
//    sac_ia.setRANSACIterations(30);

	sac_ia.align(*align);
    cout <<"has converged: "<< sac_ia.hasConverged() <<"\tscore: "<<sac_ia.getFitnessScore()<< endl;

    end = clock();
    cout << "calculate time is: " << float(end - start) / CLOCKS_PER_SEC << endl;

//    Eigen::Matrix4f T = sac_ia.getFinalTransformation ();
//    cout<<"T:"<<endl;
//    cout << T <<endl;

    //通过变换矩阵计算旋转矩阵和平移向量
    Eigen::Matrix4d T(4,4);
    T= sac_ia.getFinalTransformation().cast<double> ();//getFinalTransformation()输出的是float 转成double
    //cout << T.block<3,3>(0,0) << endl << endl;
    Eigen::Matrix3d R = T.block<3,3>(0,0);// = Eigen::Matrix3f::Identity();
    Eigen::Quaterniond q = Eigen::Quaterniond(R);
    //cout<<"R = \n"<<R<<endl;
    cout<<"Quaternion = \n"<<q.coeffs().transpose()<<endl;

    Eigen::MatrixXd t_mat  = T.topRightCorner(3, 1).transpose();
    Eigen::Vector3d t;
    t<<t_mat(0), t_mat(1), t_mat(2);
    cout<<"t = "<<endl<<t.transpose()<<endl;

    ofstream outfile("Transform.txt");
    if(!outfile)
    {
        cerr<<"open error!"<<endl;
        exit(1);
    }
    outfile<<"T:"<<endl<<T<<endl;
    outfile<<"q:"<<endl<<q.coeffs().transpose()<<endl;
    outfile<<"t:"<<endl<<t.transpose()<<endl;
    outfile.close();

    pcl::transformPointCloud(*source, *source_align, T);

    //将对齐后的点云保存
    pcl::io::savePCDFile("source_align.pcd", *source_align, false);
    //pcl::io::savePCDFile("align.pcd", *align, false);//结果是滤波后的

    //设置特征点对的关系
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33,pcl::FPFHSignature33> crude_cor_est;
    boost::shared_ptr<pcl::Correspondences> cru_correspondences (new pcl::Correspondences);//FPFP的点对的对应关系
    crude_cor_est.setInputSource(source_fpfh);
    crude_cor_est.setInputTarget(target_fpfh);
    //crude_cor_est.determineCorrespondences(*cru_correspondences);
    crude_cor_est.determineReciprocalCorrespondences(*cru_correspondences);
    cout<<"crude size is:"<<cru_correspondences->size()<<endl;

	//可视化
	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("fpfh test"));
	int v1;
	int v2;
	
	view->createViewPort(0, 0.0, 0.5, 1.0, v1);
	view->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	view->setBackgroundColor(0, 0, 0, v1);
	view->setBackgroundColor(0.05, 0, 0, v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(source, 250, 0, 0);
    view->addPointCloud(source, sources_cloud_color, "sources_cloud_v1", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(target, 0, 250, 0);
    view->addPointCloud(target, target_cloud_color, "target_cloud_v1", v1);
    //设置点的大小为2
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sources_cloud_v1");
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_v1");

    //v2
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>aligend_cloud_color(align, 255, 0, 0);
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>aligend_cloud_color(final, 255, 0, 0);

    view->addPointCloud(source_align, sources_cloud_color, "aligend_cloud_v2", v2);
    view->addPointCloud(target, target_cloud_color, "target_cloud_v2", v2);
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligend_cloud_v2");
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_v2");

    view->addCorrespondences<pcl::PointXYZ>(source,target,*cru_correspondences,"correspondence",v1);//添加显示对应点对
    //view->addCorrespondences<pcl::PointXYZ>(source_align,target,*cru_correspondences,"correspondence",v2);
	while (!view->wasStopped())
	{
		// view->spin();
		view->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}
