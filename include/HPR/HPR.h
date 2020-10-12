#ifndef HIDDEN_POINT_REMOVAL
#define HIDDEN_POINT_REMOVAL

#include <string>
#include <vector>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Dense>

void HPR(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in, std::vector<float> camera_pos, int param, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out); // Hidden Point Removal

#endif