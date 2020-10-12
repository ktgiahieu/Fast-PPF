//Author: Khuong Thanh Gia Hieu
//Bach Khoa University - CK16KSCD

#include "PPF.h"
#include <thread>
#include <mutex>

int main()
{
	//In this project, we use PPF Descriptor from this paper, please read before continue:
	//Fast and Robust Pose Estimation Algorithm for Bin Picking Using Point Pair Feature
    //Mingyu Li and Koichi Hashimoto 
	//International Conference on Pattern Recognition (ICPR)
	DescriptorPPF* descr(new DescriptorPPF());
	std::cout << "Descriptor type: " << descr->getType() << endl;
	
	//Load model
	descr->setModelPath("../../data/model/bi/6914.STL");
	std::cout << "Done Preparation ... !" << std::endl;

	//Load scene
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("../../data/scene/bi/scene0.pcd", *cloud) == -1) //* load the file
    {
		PCL_ERROR ("Couldn't read file\n");
		return (-1);
    }
	descr->storeLatestCloud(cloud);

	//We MUST process and visualize from different thread, or the program will crash

	// Start processing from different thread
	auto _3D_Matching_Lambda = [&descr]() {
		descr->prepareModelDescriptor();
		descr->_3D_Matching();
	};
	std::thread _3D_Matching_Thread(_3D_Matching_Lambda);
	std::cout << "Processing Thread Started ... !" << std::endl;
	
	// Start visualizing from different thread
	while (!descr->customViewer.viewer->wasStopped()) {
		descr->customViewer.viewer->spinOnce(300);
		std::this_thread::sleep_for(std::chrono::microseconds(300000));
	}
	 
	//Wait for thread to finish before closing the program
	if (_3D_Matching_Thread.joinable())
		_3D_Matching_Thread.join();

	return 0;

}