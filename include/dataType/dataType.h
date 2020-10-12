#ifndef DATATYPE
#define DATATYPE

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/ppf.h>

# define PI  3.1415926

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud<PointType> PointCloudType;
typedef pcl::Normal NormalType;
typedef pcl::PointCloud<NormalType> PointCloudNormalType;
typedef pcl::ReferenceFrame RFType;

typedef pcl::SHOT352 DescriptorTypeSHOT;
typedef pcl::SHOTEstimationOMP<pcl::PointXYZ, NormalType, DescriptorTypeSHOT> EstimatorTypeSHOT;

typedef pcl::PPFSignature DescriptorTypePPF;
typedef pcl::PPFEstimation<pcl::PointXYZ, NormalType, DescriptorTypePPF> EstimatorTypePPF;

typedef pcl::PointNormal PointXYZTangent;

struct CloudStyle
{
	double r;
	double g;
	double b;
	double size;

	CloudStyle(double r,
		double g,
		double b,
		double size) :
		r(r),
		g(g),
		b(b),
		size(size)
	{
	}
};

extern CloudStyle style_white;
extern CloudStyle style_red;
extern CloudStyle style_green;
extern CloudStyle style_cyan;
extern CloudStyle style_violet;

#endif