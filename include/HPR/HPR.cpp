#include "HPR.h"

using namespace Eigen;
using namespace pcl;

void HPR(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in, std::vector<float> camera_pos, int param, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out) // Hidden Point Removal
{
	int dim = 3;
	int numPts = cloud_in->size();

	ArrayXXd p(numPts, dim);
	for (int i = 0; i < cloud_in->size(); ++i)
	{
		p(i, 0) = cloud_in->at(i).x;
		p(i, 1) = cloud_in->at(i).y;
		p(i, 2) = cloud_in->at(i).z;
	}
	
	ArrayXd C(3); C << camera_pos[0], camera_pos[1], camera_pos[2];
	p = p - C.transpose().replicate(numPts, 1);

	ArrayXd normp = (p*p).rowwise().sum().sqrt();
	ArrayXd maxNormp(1); maxNormp << normp.maxCoeff()*pow(10,param);
	ArrayXd R = maxNormp.replicate(numPts, 1);

	ArrayXXd P = p + 2 * (R - normp).replicate(1, dim) * p / normp.replicate(1, dim);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_P(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < P.rows(); ++i)
	{
		pcl::PointXYZ point;
		point.x = P(i, 0);
		point.y = P(i, 1);
		point.z = P(i, 2);
		cloud_P->push_back(point);
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::Vertices> polygons;
	pcl::ConvexHull<pcl::PointXYZ> chull;
	chull.setDimension(3);
	chull.setInputCloud(cloud_P);
	chull.reconstruct(*cloud_hull, polygons);

	pcl::PointIndices::Ptr indices(new pcl::PointIndices);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_P);

	for (int i = 0; i < cloud_hull->size(); ++i)
	{
		int K = 1;
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);
		if (kdtree.nearestKSearch(cloud_hull->at(i), K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			indices->indices.push_back(pointIdxNKNSearch[0]);
		}
	}

	// Extract the inliers
	pcl::ExtractIndices<PointXYZ> extract;
	extract.setInputCloud(cloud_in);
	extract.setIndices(indices);
	extract.setNegative(false);
	extract.filter(*cloud_out);
}