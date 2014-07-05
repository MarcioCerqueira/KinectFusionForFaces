#ifndef COLORVOLUME_H
#define COLORVOLUME_H

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <vector>
#include <pcl/gpu/containers/device_array.hpp>
#include "openni_capture.h"
#include "internal.h"
#include "MyPointCloud.h"

using namespace pcl;
using namespace pcl::gpu;

class ColorVolume
{
public:
	ColorVolume(Eigen::Vector3f& tsdfSize, int maxWeight = 2);
	DeviceArray2D<int>& getColorVolume() { return colorVolume; }
	void updateColorVolume(device::Intr &intr, float trancDist, std::vector<Matrix3frm> &rmats_, std::vector<Eigen::Vector3f> &tvecs_, 
		std::vector<device::MapArr> &vmapsGPrev, KinfuTracker::View &rgbDevice);
	void fetchColors(const DeviceArray<PointXYZ>& cloud, DeviceArray<RGB>& colors);
	void reset();
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
	int maxWeight;
	Eigen::Vector3f volumeSize;
	DeviceArray2D<int> colorVolume;
	Eigen::Vector3i volumeResolution;
};

#endif