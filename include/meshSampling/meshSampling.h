#ifndef MESH_SAMPLING
#define MESH_SAMPLING

#include <string>
#include <vector>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <vtkVersion.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

extern const int default_number_samples;
extern const float default_leaf_size;
extern const bool default_write_cloud;

inline double uniform_deviate(int seed);

inline void randomPointTriangle(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3,
	float r1, float r2, Eigen::Vector3f& p);


inline void randPSurface(vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector3f& p,
	bool calcNormal, Eigen::Vector3f& n, bool calcColor, Eigen::Vector3f& c);

void uniform_sampling(vtkSmartPointer<vtkPolyData> polydata, std::size_t n_samples, bool calc_normal, bool calc_color,
	pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud_out);

void meshSampling(std::string filename, int num_samples, float leaf_size, bool write_pcd_file, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);

#endif