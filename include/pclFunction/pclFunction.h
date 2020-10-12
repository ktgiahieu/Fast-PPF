#ifndef PCLFUNCTION
#define PCLFUNCTION

#include "dataType.h"

#include <math.h>
#include <type_traits> 

// io
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/openni2_grabber.h>
//transform
#include <pcl/common/transforms.h>
// filter
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
// segment
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
// visualize
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/cloud_viewer.h>
// feature
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/common/centroid.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/ppf.h>
// recognition
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
// time
#include <pcl/common/time.h>
#include<pcl/console/time.h>
#include <pcl/console/parse.h>
// kdtree
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/surface/mls.h>
//registration
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/ppf_registration.h>
//ICP
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/registration/icp.h>

/**
* @brief The PCL Visualizer is used to visualize multiple points clearly.
* @param cloud_in - input cloud
* @param cloud_out - cloud after the application of the filter
* @return void
*/
class CustomVisualizer
{
public:
	bool ready = true;
	pcl::visualization::PCLVisualizer::Ptr viewer;
	CustomVisualizer() :viewer(pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("3D Viewer"))) { viewer->getRenderWindow()->GlobalWarningDisplayOff(); }
	void init();
};

/**
* @brief The PassThrough filter is used to identify and/or eliminate points
within a specific range of X, Y and Z values.
* @param cloud_in - input cloud
* @param cloud_out - cloud after the application of the filter
* @return void
*/
void passthrough(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in, std::vector<float> limit, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);

/*
* @brief The Statistical Outliner Remove filter is used to remove noisy measurements, e.g. outliers, from a point cloud dataset 
  using statistical analysis techniques.
* @param cloud_in - input cloud
* @param cloud_out - cloud after the application of the filter
* @return void
*/
void statisticalOutlinerRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, int numNeighbors, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);
/*
* @brief The VoxelGrid filter is used to simplify the cloud, by wrapping the point cloud
  with a three-dimensional grid and reducing the number of points to the center points within each bloc of the grid.
* @param cloud_in - input cloud
* @param cloud_vg_ptr - cloud after the application of the filter
* @return void
*/
void voxelgrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, float size_leaf, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);

void uniformsampling(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);

/*
* @brief The SACSegmentation and the ExtractIndices filters are used to identify and
remove the table from the point cloud leaving only the objects.
* @param cloud_in - input cloud
* @param cloud_out - cloud after the application of the filter - cloud after the application of the filter
* @return void
*/
void sacsegmentation_extindices(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, double dist_threshold, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);

/*
* @brief The RadiusOutlierRemoval filter is used to remove isolated point according to
the minimum number of neighbors desired.
* @param cloud_in - input cloud
* @param cloud_out - cloud after the application of the filter - cloud after the application of the filter
* @return void
*/
void radiusoutlierremoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, float radius, uint16_t min_neighbor, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);

/*
* @Tinh phap tuyen dam may diem
* @ cloud_in - cloud input pcl::PointCloud<pcl::PointXYZ>
* @ cloud_out - cloud output XYZRGBNormal
*/
void normal(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, int k, float r, char mode, PointCloudNormalType::Ptr& normal_out);

/*
* @brief The Cylinder model segmentation
* @param cloud_in - input cloud
* @param cloud_cylinder - cloud after the application of Cylinder segmentation
* @param cloud_normal - cloud normal of the cloud
* @return void
*/

double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

double computeCloudDiameter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

#endif
