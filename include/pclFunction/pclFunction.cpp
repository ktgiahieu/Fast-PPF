#pragma once
#include "pclFunction.h"

using namespace std;
using namespace pcl;

void CustomVisualizer::init() {
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(0.05);
	viewer->initCameraParameters();
}

/**
* @brief The PassThrough filter is used to identify and/or eliminate points
within a specific range of X, Y and Z values.
* @param cloud_in - input cloud
* @param cloud_out - cloud after the application of the filter - cloud after the application of the filter
* @return void
*/

void passthrough(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in, vector<float> limits, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
	pcl::PassThrough<PointXYZ> pt;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptz_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pt.setInputCloud(cloud_in);
	pt.setFilterFieldName("z");
	//pt.setFilterLimits(0.0, 0.85);//0.81
	pt.setFilterLimits(limits[0], limits[1]);//0.81
	pt.filter(*cloud_ptz_ptr);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptx_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pt.setInputCloud(cloud_ptz_ptr);
	pt.setFilterFieldName("x");
	//pt.setFilterLimits(-0.3, 0.3);//0.37
	pt.setFilterLimits(limits[2], limits[3]);//0.81
	pt.filter(*cloud_ptx_ptr);

	pt.setInputCloud(cloud_ptx_ptr);
	pt.setFilterFieldName("y");
	//pt.setFilterLimits(-0.25, 0.25);//0.2
	pt.setFilterLimits(limits[4], limits[5]);//0.81
	pt.filter(*cloud_out);
}

/*
* @brief The Statistical Outliner Remove filter is used to remove noisy measurements, e.g. outliers, from a point cloud dataset
  using statistical analysis techniques.
* @param cloud_in - input cloud
* @param cloud_out - cloud after the application of the filter
* @return void
*/
void statisticalOutlinerRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, int numNeighbors, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out) {
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setMeanK(numNeighbors);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_out);
}

/*
* @brief The VoxelGrid filter is used to simplify the cloud, by wrapping the point cloud
  with a three-dimensional grid and reducing the number of points to the center points within each bloc of the grid.
* @param cloud_in - input cloud
* @param cloud_out - cloud after the application of the filter
* @return void
*/
void voxelgrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, float size_leaf, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
	// cout << "So diem truoc khi loc voxel " << cloud_in->points.size() << endl;
	pcl::VoxelGrid<PointXYZ> vg;
	vg.setInputCloud(cloud_in);
	vg.setLeafSize(size_leaf, size_leaf, size_leaf); // 3mm
	vg.setDownsampleAllData(true);
	vg.filter(*cloud_out);
	//  cout << "So diem sau khi loc voxel " << cloud_out->points.size() << endl;
}

/*
* @brief The VoxelGrid filter is used to simplify the cloud, by wrapping the point cloud
  with a three-dimensional grid and reducing the number of points to the center points within each bloc of the grid.
* @param cloud_in - input cloud
* @param cloud_out - cloud after the application of the filter
* @return void
*/
void uniformsampling(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
	pcl::UniformSampling<PointXYZ> uniform_sampling;
	uniform_sampling.setInputCloud(cloud_in);
	uniform_sampling.setRadiusSearch(radius);
	uniform_sampling.filter(*cloud_out);
}

/*
* @brief The SACSegmentation and the ExtractIndices filters are used to identify and
remove the table from the point cloud leaving only the objects.
* @param cloud_in - input cloud
* @param cloud_out - cloud after the application of the filter - cloud after the application of the filter
* @return void
*/
void sacsegmentation_extindices(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,double dist_threshold, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
	// Identify the table
	pcl::PointIndices::Ptr sacs_inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr sacs_coefficients(new pcl::ModelCoefficients);
	pcl::SACSegmentation<PointXYZ> sacs;
	sacs.setOptimizeCoefficients(true);
	sacs.setModelType(pcl::SACMODEL_PLANE);
	sacs.setMethodType(pcl::SAC_RANSAC);
	sacs.setMaxIterations(900);//900
	sacs.setDistanceThreshold(dist_threshold);//16mm
	sacs.setInputCloud(cloud_in);
	sacs.segment(*sacs_inliers, *sacs_coefficients);
	// Remove the table
	pcl::ExtractIndices<PointXYZ> ei;
	ei.setInputCloud(cloud_in);
	ei.setIndices(sacs_inliers);
	ei.setNegative(true);
	ei.filter(*cloud_out);
}

/*
* @brief The RadiusOutlierRemoval filter is used to remove isolated point according to
the minimum number of neighbors desired.
* @param cloud_in - input cloud
* @param cloud_out - cloud after the application of the filter - cloud after the application of the filter
* @return void
*/
void radiusoutlierremoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, float radius, uint16_t min_neighbor, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
	// Remove isolated points
	pcl::RadiusOutlierRemoval<PointXYZ> ror;
	ror.setInputCloud(cloud_in);
	ror.setRadiusSearch(radius);    //2cm        
	ror.setMinNeighborsInRadius(min_neighbor);   //150   
	ror.filter(*cloud_out);
}

/*
* @Tinh phap tuyen dam may diem
* @ cloud_in - cloud input PointType
* @ cloud_out - cloud output XYZRGBNormal
*/
void normal(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, int k, float r, char mode, PointCloudNormalType::Ptr& normal_out)
{
	pcl::NormalEstimationOMP<PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>);
	// Calculate all the normals of the entire surface
	ne.setInputCloud(cloud_in);
	ne.setNumberOfThreads(8);
	ne.setSearchMethod(tree);
	if (mode == 'K')
	{
		ne.setKSearch(k);//50
		ne.compute(*normal_out);
	}
	else if (mode == 'R')
	{
		ne.setRadiusSearch(r);//2cm
		ne.compute(*normal_out);
	}
}

double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<PointXYZ> tree;
	tree.setInputCloud(cloud);

	for (std::size_t i = 0; i < cloud->size(); ++i)
	{
		if (!std::isfinite((*cloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}

double computeCloudDiameter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::VoxelGrid<PointXYZ> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.005f, 0.005f, 0.005f);
	vg.setDownsampleAllData(false);
	vg.filter(*cloud_downsampled);

	double diameter_sqr = 0;
	for (size_t i = 0; i < cloud_downsampled->points.size(); i += 10)
	{
		for (size_t j = 0; j < cloud_downsampled->points.size(); j += 10)
		{
			if (i == j)
				continue;
			double distance_sqr = (cloud_downsampled->points[i].x - cloud_downsampled->points[j].x)*(cloud_downsampled->points[i].x - cloud_downsampled->points[j].x)
				+ (cloud_downsampled->points[i].y - cloud_downsampled->points[j].y)*(cloud_downsampled->points[i].y - cloud_downsampled->points[j].y)
				+ (cloud_downsampled->points[i].z - cloud_downsampled->points[j].z)*(cloud_downsampled->points[i].z - cloud_downsampled->points[j].z);
			if (distance_sqr > diameter_sqr)
			{
				diameter_sqr = distance_sqr;
			}
		}
	}
	return sqrt(diameter_sqr);
}
