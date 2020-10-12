#include "MyPPFRegistration.hpp"

//#ifndef PCL_NO_PRECOMPILE
//#include <pcl/point_types.h>
//#include <pcl/impl/instantiate.hpp>
//#include <pcl/registration/impl/ppf_registration.hpp>

//PCL_INSTANTIATE_PRODUCT(PPFRegistration, (PCL_XYZ_POINT_TYPES)(PCL_NORMAL_POINT_TYPES));
//#endif    // PCL_NO_PRECOMPILE

void
pcl::MyPPFHashMapSearch::setInputFeatureCloud(PointCloud<PPFSignature>::ConstPtr feature_cloud)
{
	// Discretize the feature cloud and insert it in the hash map
	feature_hash_map_->clear();
	unsigned int n = static_cast<unsigned int> (std::sqrt(static_cast<float> (feature_cloud->points.size())));
	int d1, d2, d3, d4;
	max_dist_ = -1.0;
	alpha_m_.resize(n);
	for (std::size_t i = 0; i < n; ++i)
	{
		std::vector <float> alpha_m_row(n);
		for (std::size_t j = 0; j < n; ++j)
		{
			d1 = static_cast<int> (std::floor(feature_cloud->points[i*n + j].f1 / angle_discretization_step_));
			d2 = static_cast<int> (std::floor(feature_cloud->points[i*n + j].f2 / angle_discretization_step_));
			d3 = static_cast<int> (std::floor(feature_cloud->points[i*n + j].f3 / angle_discretization_step_));
			d4 = static_cast<int> (std::floor(feature_cloud->points[i*n + j].f4 / distance_discretization_step_));
			feature_hash_map_->insert(std::pair<HashKeyStruct, std::pair<std::size_t, std::size_t> >(HashKeyStruct(d1, d2, d3, d4), std::pair<std::size_t, std::size_t>(i, j)));
			alpha_m_row[j] = feature_cloud->points[i*n + j].alpha_m;

			if (max_dist_ < feature_cloud->points[i*n + j].f4)
				max_dist_ = feature_cloud->points[i*n + j].f4;
		}
		alpha_m_[i] = alpha_m_row;
	}

	internals_initialized_ = true;
}


//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::MyPPFHashMapSearch::nearestNeighborSearch(float &f1, float &f2, float &f3, float &f4,
	std::vector<std::pair<std::size_t, std::size_t> > &indices)
{
	if (!internals_initialized_)
	{
		PCL_ERROR("[pcl::PPFRegistration::nearestNeighborSearch]: input feature cloud has not been set - skipping search!\n");
		return;
	}

	int d1 = static_cast<int> (std::floor(f1 / angle_discretization_step_)),
		d2 = static_cast<int> (std::floor(f2 / angle_discretization_step_)),
		d3 = static_cast<int> (std::floor(f3 / angle_discretization_step_)),
		d4 = static_cast<int> (std::floor(f4 / distance_discretization_step_));

	indices.clear();
	HashKeyStruct key = HashKeyStruct(d1, d2, d3, d4);
	auto map_iterator_pair = feature_hash_map_->equal_range(key);
	for (; map_iterator_pair.first != map_iterator_pair.second; ++map_iterator_pair.first)
		indices.emplace_back(map_iterator_pair.first->second.first,
			map_iterator_pair.first->second.second);
}