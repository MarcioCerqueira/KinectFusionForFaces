#ifndef SDF_H
#define SDF_H

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <vector>
#include <pcl/gpu/containers/device_array.hpp>
#include "internal.h"
#include "openni_capture.h"
#include "MyPointCloud.h"

using namespace Eigen;
using namespace pcl;
using namespace pcl::gpu;
typedef short2 volume_elem_type;

class TsdfVolume
{
public:
	TsdfVolume(Eigen::Vector3i& volumeSize);
	~TsdfVolume();

	void integrateVolume(std::vector<Matrix3frm>& rmats, std::vector<Vector3f>& tvecs, device::DepthMap &depthRaw, device::Intr& intrinsics,
		float trancDist, DeviceArray2D<float>& depthRawScaled, int globalTime);
	void raycast(std::vector<Matrix3frm>& rmats, std::vector<Vector3f>& tvecs, device::Intr& intrinsics, float trancDist, MyPointCloud *globalPreviousPointCloud,
		int globalTime);
	void raycastBasedOnClipping(std::vector<Matrix3frm>& rmats, std::vector<Vector3f>& tvecs, device::Intr& intrinsics, float trancDist, MyPointCloud *globalPreviousPointCloud,
		int globalTime);
	DeviceArray<PointXYZ> fetchCloud(DeviceArray<PointXYZ>& cloud_buffer);

	//10 * 1000 * 1000
	enum { DEFAULT_CLOUD_BUFFER_SIZE = 10 * 1000 * 1000 };

	Eigen::Vector3f getVolumeSize() { return volumeSize_; }
	void getHostErrorInRGB(KinfuTracker::View &errorInRGB);
	DeviceArray2D<int> getVolume() { return volume_; }
	unsigned char* getClippedRegion() { return clippingPlane.clippedRegion; }

	void incrementClippingPlaneLeftX() { 
		clippingPlane.leftX += velClipping; 
		if(clippingPlane.leftX > device::VOLUME_X) clippingPlane.leftX = device::VOLUME_X; 
	}
	void incrementClippingPlaneRightX() { 
		clippingPlane.rightX += velClipping; 
		if(clippingPlane.rightX > device::VOLUME_X) clippingPlane.rightX = device::VOLUME_X; 
	}
	void incrementClippingPlaneUpY() { 
		clippingPlane.upY += velClipping; 
		if(clippingPlane.upY > device::VOLUME_Y) clippingPlane.upY = device::VOLUME_Y;
	}
	void incrementClippingPlaneDownY() { 
		clippingPlane.downY += velClipping; 
		if(clippingPlane.downY > device::VOLUME_Y) clippingPlane.downY = device::VOLUME_Y;
	}
	void incrementClippingPlaneFrontZ() { 
		clippingPlane.frontZ += velClipping; 
		if(clippingPlane.frontZ > device::VOLUME_Z) clippingPlane.frontZ = device::VOLUME_Z;
	}
	void incrementClippingPlaneBackZ() { 
		clippingPlane.backZ += velClipping; 
		if(clippingPlane.backZ > device::VOLUME_Z) clippingPlane.backZ = device::VOLUME_Z;
	}
	
	void decrementClippingPlaneLeftX() { 
		clippingPlane.leftX -= velClipping; 
		if(clippingPlane.leftX < 0) clippingPlane.leftX = 0; 
	}
	void decrementClippingPlaneRightX() { 
		clippingPlane.rightX -= velClipping; 
		if(clippingPlane.rightX < 0) clippingPlane.rightX = 0;
	}
	void decrementClippingPlaneUpY() { 
		clippingPlane.upY -= velClipping; 
		if(clippingPlane.upY < 0) clippingPlane.upY = 0; 
	}
	void decrementClippingPlaneDownY() { 
		clippingPlane.downY -= velClipping; 
		if(clippingPlane.downY < 0) clippingPlane.downY = 0;
	}
	void decrementClippingPlaneFrontZ() { 
		clippingPlane.frontZ -= velClipping; 
		if(clippingPlane.frontZ < 0) clippingPlane.frontZ = 0;
	}
	void decrementClippingPlaneBackZ() { 
		clippingPlane.backZ -= velClipping; 
		if(clippingPlane.backZ < 0) clippingPlane.backZ = 0;
	}
	
	void setTSDFVisualization(bool hasTSDFVisualization) { hasTSDFVisualization_ = hasTSDFVisualization; }
	void reset() {  device::initVolume<volume_elem_type> (volume_); }
	
	bool hasClippingPlane;
private:
	Eigen::Vector3f volumeSize_;
	DeviceArray2D<int> volume_;
	DeviceArray2D<float> error_;
	bool hasTSDFVisualization_;

	//clipping
	pcl::device::ClippingPlane clippingPlane;
	int velClipping;
};

#endif