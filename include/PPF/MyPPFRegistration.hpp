
#ifndef PCL_MY_PPF_REGISTRATION_H_
#define PCL_MY_PPF_REGISTRATION_H_

#include <pcl/common/common.h>
#include <pcl/registration/boost.h>
#include <pcl/registration/registration.h>
#include <pcl/features/ppf.h>
#include <pcl/common/transforms.h>
#include <numeric>

#include <unordered_map>

namespace pcl
{
  class MyPPFHashMapSearch
  {
    public:
      /** \brief Data structure to hold the information for the key in the feature hash map of the
        * MyPPFHashMapSearch class
        * \note It uses multiple pair levels in order to enable the usage of the boost::hash function
        * which has the std::pair implementation (i.e., does not require a custom hash function)
        */
      struct HashKeyStruct : public std::pair <int, std::pair <int, std::pair <int, int> > >
      {
		HashKeyStruct () = default;
        HashKeyStruct(int a, int b, int c, int d)
        {
          this->first = a;
          this->second.first = b;
          this->second.second.first = c;
          this->second.second.second = d;
        }
		std::size_t operator()(const HashKeyStruct& s) const noexcept
        {
            const std::size_t h1 = std::hash<int>{} (s.first);
            const std::size_t h2 = std::hash<int>{} (s.second.first);
            const std::size_t h3 = std::hash<int>{} (s.second.second.first);
            const std::size_t h4 = std::hash<int>{} (s.second.second.second);
            return h1 ^ (h2 << 1) ^ (h3 << 2) ^ (h4 << 3);
        }
      };
      /*typedef std::unordered_multimap<HashKeyStruct, std::pair<size_t, size_t> > FeatureHashMapType;
      typedef std::shared_ptr<FeatureHashMapType> FeatureHashMapTypePtr;
      typedef std::shared_ptr<MyPPFHashMapSearch> Ptr;*/

	  using FeatureHashMapType = std::unordered_multimap<HashKeyStruct, std::pair<std::size_t, std::size_t>, HashKeyStruct>;
      using FeatureHashMapTypePtr = std::shared_ptr<FeatureHashMapType>;
      using Ptr = std::shared_ptr<MyPPFHashMapSearch>;
      using ConstPtr = std::shared_ptr<const MyPPFHashMapSearch>;

      /** \brief Constructor for the MyPPFHashMapSearch class which sets the two step parameters for the enclosed data structure
       * \param angle_discretization_step the step value between each bin of the hash map for the angular values
       * \param distance_discretization_step the step value between each bin of the hash map for the distance values
       */
      MyPPFHashMapSearch (float angle_discretization_step = 12.0f / 180.0f * static_cast<float> (M_PI),
                        float distance_discretization_step = 0.01f)
        : alpha_m_ ()
        , feature_hash_map_ (new FeatureHashMapType)
        , internals_initialized_ (false)
        , angle_discretization_step_ (angle_discretization_step)
        , distance_discretization_step_ (distance_discretization_step)
        , max_dist_ (-1.0f)
      {
      }

      /** \brief Method that sets the feature cloud to be inserted in the hash map
       * \param feature_cloud a const smart pointer to the PPFSignature feature cloud
       */
      void
      setInputFeatureCloud (PointCloud<PPFSignature>::ConstPtr feature_cloud);

      /** \brief Function for finding the nearest neighbors for the given feature inside the discretized hash map
       * \param f1 The 1st value describing the query PPFSignature feature
       * \param f2 The 2nd value describing the query PPFSignature feature
       * \param f3 The 3rd value describing the query PPFSignature feature
       * \param f4 The 4th value describing the query PPFSignature feature
       * \param indices a vector of pair indices representing the feature pairs that have been found in the bin
       * corresponding to the query feature
       */
      void
      nearestNeighborSearch (float &f1, float &f2, float &f3, float &f4,
                             std::vector<std::pair<size_t, size_t> > &indices);

      /** \brief Convenience method for returning a copy of the class instance as a std::shared_ptr */
      Ptr
      makeShared() { return std::make_shared<MyPPFHashMapSearch>(*this); }

      /** \brief Returns the angle discretization step parameter (the step value between each bin of the hash map for the angular values) */
      inline float
      getAngleDiscretizationStep () { return angle_discretization_step_; }

      /** \brief Returns the distance discretization step parameter (the step value between each bin of the hash map for the distance values) */
      inline float
      getDistanceDiscretizationStep () { return distance_discretization_step_; }

      /** \brief Returns the maximum distance found between any feature pair in the given input feature cloud */
      inline float
      getModelDiameter () { return max_dist_; }

      std::vector <std::vector <float> > alpha_m_;
    private:
      FeatureHashMapTypePtr feature_hash_map_;
      bool internals_initialized_;

      float angle_discretization_step_, distance_discretization_step_;
      float max_dist_;
  };

  /** \brief Class that registers two point clouds based on their sets of PPFSignatures.
   * Please refer to the following publication for more details:
   *    B. Drost, M. Ulrich, N. Navab, S. Ilic
   *    Model Globally, Match Locally: Efficient and Robust 3D Object Recognition
   *    2010 IEEE Conference on Computer Vision and Pattern Recognition (CVPR)
   *    13-18 June 2010, San Francisco, CA
   *
   * \note This class works in tandem with the PPFEstimation class
   *
   * \author Alexandru-Eugen Ichim
   */
  template <typename PointSource, typename PointTarget>
  class MyPPFRegistration : public Registration<PointSource, PointTarget>
  {
    public:
      /** \brief Structure for storing a pose (represented as an Eigen::Affine3f) and an integer for counting votes
        * \note initially used std::pair<Eigen::Affine3f, unsigned int>, but it proved problematic
        * because of the Eigen structures alignment problems - std::pair does not have a custom allocator
        */
      struct PoseWithVotes
      {
        PoseWithVotes(Eigen::Affine3f &a_pose, unsigned int &a_votes)
        : pose (a_pose),
          votes (a_votes)
        {}

        Eigen::Affine3f pose;
        unsigned int votes;
      };
      typedef std::vector<PoseWithVotes, Eigen::aligned_allocator<PoseWithVotes> > PoseWithVotesList;

      /// input_ is the model cloud
      using Registration<PointSource, PointTarget>::input_;
      /// target_ is the scene cloud
      using Registration<PointSource, PointTarget>::target_;
      using Registration<PointSource, PointTarget>::converged_;
      using Registration<PointSource, PointTarget>::final_transformation_;
      using Registration<PointSource, PointTarget>::transformation_;

      typedef pcl::PointCloud<PointSource> PointCloudSource;
      typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
      typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

      typedef pcl::PointCloud<PointTarget> PointCloudTarget;
      typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
      typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;


      /** \brief Empty constructor that initializes all the parameters of the algorithm with default values */
	  MyPPFRegistration()
		  : Registration<PointSource, PointTarget>(),
		  search_method_(),
		  scene_reference_point_sampling_rate_(5),
		  scene_referred_point_sampling_rate_(5),
		  clustering_position_diff_threshold_(0.01f),
		  clustering_rotation_diff_threshold_(20.0f / 180.0f * static_cast<float> (M_PI)),
		  Lvoxel(0.001f)
      {}

      /** \brief Method for setting the position difference clustering parameter
       * \param clustering_position_diff_threshold distance threshold below which two poses are
       * considered close enough to be in the same cluster (for the clustering phase of the algorithm)
       */
      inline void
      setPositionClusteringThreshold (float clustering_position_diff_threshold) { clustering_position_diff_threshold_ = clustering_position_diff_threshold; }

      /** \brief Returns the parameter defining the position difference clustering parameter -
       * distance threshold below which two poses are considered close enough to be in the same cluster
       * (for the clustering phase of the algorithm)
       */
      inline float
      getPositionClusteringThreshold () { return clustering_position_diff_threshold_; }

      /** \brief Method for setting the rotation clustering parameter
       * \param clustering_rotation_diff_threshold rotation difference threshold below which two
       * poses are considered to be in the same cluster (for the clustering phase of the algorithm)
       */
      inline void
      setRotationClusteringThreshold (float clustering_rotation_diff_threshold) { clustering_rotation_diff_threshold_ = clustering_rotation_diff_threshold; }

      /** \brief Returns the parameter defining the rotation clustering threshold
       */
      inline float
      getRotationClusteringThreshold () { return clustering_rotation_diff_threshold_; }

      /** \brief Method for setting the scene reference point sampling rate
       * \param scene_reference_point_sampling_rate sampling rate for the scene reference point
       */
      inline void
      setSceneReferencePointSamplingRate (unsigned int scene_reference_point_sampling_rate) { scene_reference_point_sampling_rate_ = scene_reference_point_sampling_rate; }

      /** \brief Returns the parameter for the scene reference point sampling rate of the algorithm */
      inline unsigned int
      getSceneReferencePointSamplingRate () { return scene_reference_point_sampling_rate_; }

	  /** \brief Method for setting the scene reference point sampling rate
       * \param scene_reference_point_sampling_rate sampling rate for the scene reference point
       */
      inline void
      setSceneReferredPointSamplingRate (unsigned int scene_referred_point_sampling_rate) { scene_referred_point_sampling_rate_ = scene_referred_point_sampling_rate; }

      /** \brief Returns the parameter for the scene reference point sampling rate of the algorithm */
      inline unsigned int
      getSceneReferredPointSamplingRate () { return scene_referred_point_sampling_rate_; }


      /** \brief Function that sets the search method for the algorithm
       * \note Right now, the only available method is the one initially proposed by
       * the authors - by using a hash map with discretized feature vectors
       * \param search_method smart pointer to the search method to be set
       */
      inline void
      setSearchMethod (MyPPFHashMapSearch::Ptr search_method) { search_method_ = search_method; }

      /** \brief Getter function for the search method of the class */
      inline MyPPFHashMapSearch::Ptr
      getSearchMethod () { return search_method_; }

      /** \brief Provide a pointer to the input target (e.g., the point cloud that we want to align the input source to)
       * \param cloud the input point cloud target
       */
      void
      setInputTarget (const PointCloudTargetConstPtr &cloud);

	  inline void
      setLvoxel (double Lvoxel_input) { Lvoxel = Lvoxel_input; }

	  void
			computeFinalPoses(typename pcl::MyPPFRegistration<PointSource, PointTarget>::PoseWithVotesList &result);

	  void
	  verifyPoses (PoseWithVotesList &poses,
                    PoseWithVotesList &result);

    private:
      /** \brief Method that calculates the transformation between the input_ and target_ point clouds, based on the PPF features */
      void
      computeTransformation (PointCloudSource &output, const Eigen::Matrix4f& guess);


      /** \brief the search method that is going to be used to find matching feature pairs */
      MyPPFHashMapSearch::Ptr search_method_;

      /** \brief parameter for the sampling rate of the scene reference points */
      unsigned int scene_reference_point_sampling_rate_;

	  /** \brief parameter for the sampling rate of the scene reference points */
      unsigned int scene_referred_point_sampling_rate_;

      /** \brief position and rotation difference thresholds below which two
        * poses are considered to be in the same cluster (for the clustering phase of the algorithm) */
      float clustering_position_diff_threshold_, clustering_rotation_diff_threshold_;

	  float Lvoxel;

      /** \brief use a kd-tree with range searches of range max_dist to skip an O(N) pass through the point cloud */
      typename pcl::KdTreeFLANN<PointTarget>::Ptr scene_search_tree_;

      /** \brief static method used for the std::sort function to order two PoseWithVotes
       * instances by their number of votes*/
      static bool
      poseWithVotesCompareFunction (const PoseWithVotes &a,
                                    const PoseWithVotes &b);

	  static bool
		  scenePointSetCompareFunction(const std::vector <bool> &a, const std::vector <bool> &b);


  };
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::MyPPFRegistration<PointSource, PointTarget>::setInputTarget (const PointCloudTargetConstPtr &cloud)
{
  Registration<PointSource, PointTarget>::setInputTarget (cloud);

  scene_search_tree_ = typename pcl::KdTreeFLANN<PointTarget>::Ptr (new pcl::KdTreeFLANN<PointTarget>);
  scene_search_tree_->setInputCloud (target_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> bool
pcl::MyPPFRegistration<PointSource, PointTarget>::poseWithVotesCompareFunction (const typename pcl::MyPPFRegistration<PointSource, PointTarget>::PoseWithVotes &a,
                                                                              const typename pcl::MyPPFRegistration<PointSource, PointTarget>::PoseWithVotes &b )
{
  return (a.votes > b.votes);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> bool
pcl::MyPPFRegistration<PointSource, PointTarget>::scenePointSetCompareFunction (const std::vector <bool> &a,
                                                                              const std::vector <bool> &b )
{
	unsigned int a_votes  = 0, b_votes = 0;
	for (size_t i = 0; i < a.size(); ++i)
	{
		a_votes += a[i];
		b_votes += b[i];
	}
		
	return (a_votes > b_votes);
}

template <typename PointSource, typename PointTarget> void
pcl::MyPPFRegistration<PointSource, PointTarget>::computeFinalPoses(typename pcl::MyPPFRegistration<PointSource, PointTarget>::PoseWithVotesList &result)
{
	PoseWithVotesList voted_poses;
  std::vector <std::vector <unsigned int> > accumulator_array;
  accumulator_array.resize (input_->points.size ());

  size_t aux_size = static_cast<size_t> (floor (2 * M_PI / search_method_->getAngleDiscretizationStep ()));
  for (size_t i = 0; i < input_->points.size (); ++i)
  {
    std::vector<unsigned int> aux (aux_size);
    accumulator_array[i] = aux;
  }
  PCL_INFO ("Accumulator array size: %u x %u.\n", accumulator_array.size (), accumulator_array.back ().size ());

  // Consider every <scene_reference_point_sampling_rate>-th point as the reference point => fix s_r
  float f1, f2, f3, f4;
  for (size_t scene_reference_index = 0; scene_reference_index < target_->points.size (); scene_reference_index += scene_reference_point_sampling_rate_)
  {
    Eigen::Vector3f scene_reference_point = target_->points[scene_reference_index].getVector3fMap (),
        scene_reference_normal = target_->points[scene_reference_index].getNormalVector3fMap ();

    float rotation_angle_sg = acosf (scene_reference_normal.dot (Eigen::Vector3f::UnitX ()));
    bool parallel_to_x_sg = (scene_reference_normal.y() == 0.0f && scene_reference_normal.z() == 0.0f);
    Eigen::Vector3f rotation_axis_sg = (parallel_to_x_sg)?(Eigen::Vector3f::UnitY ()):(scene_reference_normal.cross (Eigen::Vector3f::UnitX ()). normalized());
    Eigen::AngleAxisf rotation_sg (rotation_angle_sg, rotation_axis_sg);
    Eigen::Affine3f transform_sg (Eigen::Translation3f ( rotation_sg * ((-1) * scene_reference_point)) * rotation_sg);

    // For every other point in the scene => now have pair (s_r, s_i) fixed
    std::vector<int> indices;
    std::vector<float> distances;
    scene_search_tree_->radiusSearch (target_->points[scene_reference_index],
                                     search_method_->getModelDiameter () /2,
                                     indices,
                                     distances);
    for(size_t i = 0; i < indices.size (); i+= scene_referred_point_sampling_rate_)
//    for(size_t i = 0; i < target_->points.size (); ++i)
    {
      //size_t scene_point_index = i;
      size_t scene_point_index = indices[i];
      if (scene_reference_index != scene_point_index)
      {
        if (/*pcl::computePairFeature*/pcl::computePPFPairFeature (target_->points[scene_reference_index].getVector4fMap (),
                                        target_->points[scene_reference_index].getNormalVector4fMap (),
                                        target_->points[scene_point_index].getVector4fMap (),
                                        target_->points[scene_point_index].getNormalVector4fMap (),
                                        f1, f2, f3, f4))
        {
          std::vector<std::pair<size_t, size_t> > nearest_indices;
          search_method_->nearestNeighborSearch (f1, f2, f3, f4, nearest_indices);

          // Compute alpha_s angle
          Eigen::Vector3f scene_point = target_->points[scene_point_index].getVector3fMap ();

          Eigen::Vector3f scene_point_transformed = transform_sg * scene_point;
          float alpha_s = atan2f ( -scene_point_transformed(2), scene_point_transformed(1));
          if (sin (alpha_s) * scene_point_transformed(2) < 0.0f)
            alpha_s *= (-1);
          alpha_s *= (-1);

          // Go through point pairs in the model with the same discretized feature
          for (std::vector<std::pair<size_t, size_t> >::iterator v_it = nearest_indices.begin (); v_it != nearest_indices.end (); ++ v_it)
          {
            size_t model_reference_index = v_it->first,
                model_point_index = v_it->second;
            // Calculate angle alpha = alpha_m - alpha_s
            float alpha = search_method_->alpha_m_[model_reference_index][model_point_index] - alpha_s;
			if (alpha < -M_PI)
				alpha += (M_PI * 2.0);
			if (alpha > M_PI)
				alpha -= (M_PI * 2.0);

			unsigned int alpha_discretized = static_cast<unsigned int> (floor(alpha + M_PI) / search_method_->getAngleDiscretizationStep());

            accumulator_array[model_reference_index][alpha_discretized] ++;
          }
        }
        else PCL_ERROR ("[pcl::MyPPFRegistration::computeTransformation] Computing pair feature vector between points %u and %u went wrong.\n", scene_reference_index, scene_point_index);
      }
    }

    size_t max_votes_i = 0, max_votes_j = 0;
    unsigned int max_votes = 0;

    for (size_t i = 0; i < accumulator_array.size (); ++i)
      for (size_t j = 0; j < accumulator_array.back ().size (); ++j)
      {
        if (accumulator_array[i][j] > max_votes)
        {
          max_votes = accumulator_array[i][j];
          max_votes_i = i;
          max_votes_j = j;
        }
        // Reset accumulator_array for the next set of iterations with a new scene reference point
        accumulator_array[i][j] = 0;
      }

    Eigen::Vector3f model_reference_point = input_->points[max_votes_i].getVector3fMap (),
        model_reference_normal = input_->points[max_votes_i].getNormalVector3fMap ();
    float rotation_angle_mg = acosf (model_reference_normal.dot (Eigen::Vector3f::UnitX ()));
    bool parallel_to_x_mg = (model_reference_normal.y() == 0.0f && model_reference_normal.z() == 0.0f);
    Eigen::Vector3f rotation_axis_mg = (parallel_to_x_mg)?(Eigen::Vector3f::UnitY ()):(model_reference_normal.cross (Eigen::Vector3f::UnitX ()). normalized());
    Eigen::AngleAxisf rotation_mg (rotation_angle_mg, rotation_axis_mg);
    Eigen::Affine3f transform_mg (Eigen::Translation3f ( rotation_mg * ((-1) * model_reference_point)) * rotation_mg);
    Eigen::Affine3f max_transform = 
      transform_sg.inverse () * 
      Eigen::AngleAxisf ((static_cast<float> (max_votes_j) - floorf (static_cast<float> (M_PI) / search_method_->getAngleDiscretizationStep ())) * search_method_->getAngleDiscretizationStep (), Eigen::Vector3f::UnitX ()) * 
      transform_mg;

    voted_poses.push_back (PoseWithVotes (max_transform, max_votes));
  }
  PCL_DEBUG ("Done with the Hough Transform ...\n");


  result.clear();
  sort(voted_poses.begin (), voted_poses.end (), poseWithVotesCompareFunction);
  //Only take poses with score > 0.2 x highest score
  for (int i = 0; i < voted_poses.size(); i++)
  {
	  result.push_back(voted_poses[i]);
	  if (voted_poses[i].votes < 0.3*voted_poses[0].votes || i > 50)
	  {
		  break;
	  }
  }

  //pcl::transformPointCloud (*input_, output, result.front ().pose);

  transformation_ = final_transformation_ = result.front ().pose.matrix ();
  converged_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::MyPPFRegistration<PointSource, PointTarget>::verifyPoses(typename pcl::MyPPFRegistration<PointSource, PointTarget>::PoseWithVotesList &poses,
	typename pcl::MyPPFRegistration<PointSource, PointTarget>::PoseWithVotesList &result)
{
	
	//Define voxel voting space
	PointTarget minPt, maxPt;
	pcl::getMinMax3D(*target_, minPt, maxPt);
	minPt.x -= 2 * Lvoxel; minPt.y -= 2 * Lvoxel; minPt.z -= 2 * Lvoxel;
	maxPt.x += 2 * Lvoxel; maxPt.y += 2 * Lvoxel; maxPt.z += 2 * Lvoxel;

	size_t x_size = static_cast<size_t> (std::ceil((maxPt.x - minPt.x)/Lvoxel));
	size_t y_size = static_cast<size_t> (std::ceil((maxPt.y - minPt.y)/Lvoxel));
	size_t z_size = static_cast<size_t> (std::ceil((maxPt.z - minPt.z)/Lvoxel));

	std::vector < std::vector <std::vector <int> > > voxel_voting_space(x_size, std::vector <std::vector <int> >(y_size, std::vector <int>(z_size, -1)));

	for (size_t scene_point = 0; scene_point < target_->points.size(); ++scene_point)
	{
		voxel_voting_space[static_cast<size_t>(std::floor((target_->points[scene_point].x - minPt.x) / Lvoxel))][static_cast<size_t>(std::floor((target_->points[scene_point].y - minPt.y) / Lvoxel))][static_cast<size_t>(std::floor((target_->points[scene_point].z - minPt.z) / Lvoxel))] = scene_point;

		voxel_voting_space[static_cast<size_t>(std::floor((target_->points[scene_point].x - minPt.x) / Lvoxel) - 1)][static_cast<size_t>(std::floor((target_->points[scene_point].y - minPt.y) / Lvoxel))][static_cast<size_t>(std::floor((target_->points[scene_point].z - minPt.z) / Lvoxel))] = scene_point;
		voxel_voting_space[static_cast<size_t>(std::floor((target_->points[scene_point].x - minPt.x) / Lvoxel) + 1)][static_cast<size_t>(std::floor((target_->points[scene_point].y - minPt.y) / Lvoxel))][static_cast<size_t>(std::floor((target_->points[scene_point].z - minPt.z) / Lvoxel))] = scene_point;
		voxel_voting_space[static_cast<size_t>(std::floor((target_->points[scene_point].x - minPt.x) / Lvoxel))][static_cast<size_t>(std::floor((target_->points[scene_point].y - minPt.y) / Lvoxel) - 1)][static_cast<size_t>(std::floor((target_->points[scene_point].z - minPt.z) / Lvoxel))] = scene_point;
		voxel_voting_space[static_cast<size_t>(std::floor((target_->points[scene_point].x - minPt.x) / Lvoxel))][static_cast<size_t>(std::floor((target_->points[scene_point].y - minPt.y) / Lvoxel) + 1)][static_cast<size_t>(std::floor((target_->points[scene_point].z - minPt.z) / Lvoxel))] = scene_point;
		voxel_voting_space[static_cast<size_t>(std::floor((target_->points[scene_point].x - minPt.x) / Lvoxel))][static_cast<size_t>(std::floor((target_->points[scene_point].y - minPt.y) / Lvoxel))][static_cast<size_t>(std::floor((target_->points[scene_point].z - minPt.z) / Lvoxel) - 1)] = scene_point;
		voxel_voting_space[static_cast<size_t>(std::floor((target_->points[scene_point].x - minPt.x) / Lvoxel))][static_cast<size_t>(std::floor((target_->points[scene_point].y - minPt.y) / Lvoxel))][static_cast<size_t>(std::floor((target_->points[scene_point].z - minPt.z) / Lvoxel) + 1)] = scene_point;
	}

	//Calculate new pose score
	std::vector <std::vector <bool> > scene_point_set(poses.size(), std::vector <bool>(target_->points.size(), 0));

	for (size_t pose_index = 0; pose_index < poses.size(); ++pose_index)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr instance(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::copyPointCloud(*input_, *instance);		
		pcl::transformPointCloud(*instance, *instance, poses[pose_index].pose);

		for (size_t model_point = 0; model_point < instance->points.size(); ++model_point)
		{
			int x_int = static_cast<int>(std::floor((instance->points[model_point].x - minPt.x) / Lvoxel));
			int y_int = static_cast<int>(std::floor((instance->points[model_point].y - minPt.y) / Lvoxel));
			int z_int = static_cast<int>(std::floor((instance->points[model_point].z - minPt.z) / Lvoxel));

			if (x_int < 0 || x_int >= x_size || y_int < 0 || y_int >= y_size || z_int < 0 || z_int >= z_size)
			{
				continue;
			}


			int scene_corr_index = voxel_voting_space[x_int][y_int][z_int];
			
			if (scene_corr_index != -1)
			{
				scene_point_set[pose_index][scene_corr_index] = 1;
			}
		}

		unsigned int initial_sum  = 0;
		for (size_t i = 0; i < scene_point_set.back().size(); ++i)
			initial_sum += scene_point_set[pose_index][i];
		poses[pose_index].votes = initial_sum;
	}

	
	if (poses.size() == 2)
	{
		if (poses[0].votes > poses[1].votes)
		{
			result.push_back(poses[0]);
			result.push_back(poses[1]);
		}
		else {
			result.push_back(poses[1]);
			result.push_back(poses[0]);
		}
		return;
	}
	else if (poses.size() == 1)
	{
		result.push_back(poses[0]);
		return;
	}
	sort(poses.begin(), poses.end(), poseWithVotesCompareFunction);
	sort(scene_point_set.begin(), scene_point_set.end(), scenePointSetCompareFunction);
	while (poses.size() > 2)
	{
		result.push_back(poses[0]);

		std::transform (scene_point_set[0].begin(), scene_point_set[0].end(), scene_point_set[0].begin(), std::logical_not<bool>());

		for (int pose_index = 1; pose_index < poses.size(); ++pose_index)
		{
			std::transform(scene_point_set[0].begin(), scene_point_set[0].end(),
				scene_point_set[pose_index].begin(), scene_point_set[pose_index].begin(), std::logical_and<bool>());

			unsigned int initial_sum  = 0;

			for (size_t i = 0; i < scene_point_set.back().size(); ++i)
				initial_sum += scene_point_set[pose_index][i];
			poses[pose_index].votes = initial_sum;
		}
		poses.erase(poses.begin(), poses.begin() + 1);
		scene_point_set.erase(scene_point_set.begin(), scene_point_set.begin() + 1);
		sort(poses.begin(), poses.end(), poseWithVotesCompareFunction);
		sort(scene_point_set.begin(), scene_point_set.end(), scenePointSetCompareFunction);
		if (poses[0].votes < 0.5 * result[0].votes)
			return;
	}

}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::MyPPFRegistration<PointSource, PointTarget>::computeTransformation (PointCloudSource &output, const Eigen::Matrix4f& guess)
{
	 //THIS METHOD IS NOT USED ANY MORE
	// DON'T ERASE
}
#endif 
