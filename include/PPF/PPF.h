#ifndef PPF
#define PPF

#include "pclFunction.h"
#include "meshSampling.h"
#include "HPR.h"
#include "MyPPFRegistration.hpp"
#include <mutex>

class DescriptorPPF
{
private:
	//IO
	std::string model_filename_;
	std::string type = "PPF";

	//Algorithm params
	double t_sampling = 0.04;
	float samp_rad;
	float norm_rad;
	double Lvoxel;
	float angle_discretization_step = 12.0f / 180.0f * float(M_PI);
	float distance_discretization_step = 0.005f;
	int scene_reference_point_sampling_rate = 10;
	int scene_referred_point_sampling_rate = 5;
	int icp_max_iter_ = 100;
	float icp_corr_distance_ = 0.01f;

	// Model 
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_sampling = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr model = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud <pcl::PointNormal>::Ptr model_keypoints_with_normals = pcl::PointCloud <pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>());
	std::shared_ptr<pcl::MyPPFHashMapSearch> ppf_hashmap_search = std::make_shared<pcl::MyPPFHashMapSearch>(angle_discretization_step, distance_discretization_step);

	//Others
	pcl::console::TicToc tt; // Tictoc for process-time calculation
	std::mutex mtx;
	PointCloudType::Ptr latestCloud = PointCloudType::Ptr(new PointCloudType());
	
public:
	CustomVisualizer customViewer;
	DescriptorPPF() {}
	std::string getType();
	void setModelPath(std::string model_path_);
	bool loadModel();
	bool prepareModelDescriptor();
	void storeLatestCloud(const PointCloudType::ConstPtr &cloud);
	void _3D_Matching();
};

#endif
