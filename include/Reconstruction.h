#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H

#define _CRT_SECURE_NO_DEPRECATE
#define NOMINMAX
#define USE_HEAD_POSE_ESTIMATION 1

#include "openni_capture.h"
#include "Image.h"
#include "MyPointCloud.h"
#include "TsdfVolume.h"

#if (USE_HEAD_POSE_ESTIMATION)
#include "Mediators/HeadPoseEstimationMediator.h"
#endif

#include <iostream>
//#include "pcl/gpu/kinfu/kinfu.h"
#include "pcl/gpu/containers/initialization.hpp"

#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/image_viewer.h>
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace pcl;
using namespace pcl::gpu;
using namespace Eigen;

#if (USE_HEAD_POSE_ESTIMATION)
	class HeadPoseEstimationMediator;
#endif

class Reconstruction
{
public:
	Reconstruction(Eigen::Vector3i& volumeSize);
	~Reconstruction();
	void run(Image *image);
	
	//HeadPoseEstimationMediator
	bool reRunICP();
	void reRunRaycasting();
	void transformGlobalPreviousPointCloud(Eigen::Matrix3f& Rinc, Eigen::Vector3f& tvec, Eigen::Vector3f& centerOfMass);
	
	void reset();
	void setErrorVisualization(bool hasErrorVisualization) { hasErrorVisualization_ = hasErrorVisualization; hasTsdfVolumeVisualization_ = false;}
	void enableOnlyTracking(bool stopFaceDetection = true);
	void stopTracking(bool stop) { stopTracking_ = stop; }

	void setThreshold(int threshold) { threshold_ = threshold; }
	void setHeadPoseEstimationMediatorPointer(void *pointer) { 
		headPoseEstimationMediator = pointer; 
		headPoseEstimationOk = true;
	}
	void setIntrinsics(pcl::device::Intr& intrinsics) { this->intrinsics = intrinsics; }
	void setTrancationDistance(float trancationDistance) { this->trancationDistance = trancationDistance; }
	 
	int getThreshold() { return threshold_; }
	unsigned short* getPreviousDepthMap() { return previousDepthData; }
	Eigen::Vector3f getCurrentTranslation() { return tvecs_[globalTime - 1]; }
	Eigen::Vector3f getPreviousTranslation() { return tvecs_[globalTime - 2]; }
	Matrix3frm getCurrentRotation() { return rmats_[globalTime - 1]; }
	Matrix3frm getPreviousRotation() { return rmats_[globalTime - 2]; }
	Eigen::Vector3f getInitialTranslation() { return init_tcam_; }
	pcl::device::Intr& getIntrinsics() { return intrinsics; }
	float getTrancationDistance() { return trancationDistance; }
	TsdfVolume* getTsdfVolume() { return tsdfVolume_; }
	Eigen::Vector3f& getVolumeSize() { return tsdfVolume_->getVolumeSize(); }
	std::vector<Matrix3frm>& getRotationMatrices() { return rmats_; }
	std::vector<Vector3f>& getTranslationVectors() { return tvecs_; }
	std::vector<device::MapArr>& getGlobalVertexMaps() { return globalPreviousPointCloud_->getVertexMaps(); }
	int getGlobalTime() { return globalTime; }
	void incrementGlobalTime() { globalTime++; }
	Eigen::Vector3f getGlobalCentroid() { return globalPreviousPointCloud_->getCentroid(); }
	MyPointCloud* getCurrentPointCloud() { return currentPointCloud_; }
	MyPointCloud* getGlobalPreviousPointCloud() { return globalPreviousPointCloud_; }
	
	bool hasImage() { return hasImage_; }
	bool hasErrorVisualization() { return hasErrorVisualization_; }
	bool isOnlyTrackingStopped() { return stopTracking_; }
	bool isOnlyTrackingOn() { return isOnlyTrackingOn_; }
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr extractFullPointCloud();
	void savePointCloud();
	
private:

	MyPointCloud *currentPointCloud_;
	MyPointCloud *globalPreviousPointCloud_;
	
	TsdfVolume *tsdfVolume_;

	bool hasErrorVisualization_;
	bool hasTsdfVolumeVisualization_;
	bool hasImage_;
	bool hasIncrement_;
	bool isOnlyTrackingOn_;
	bool stopTracking_;

	int threshold_;

	char *RAFileName_;
	
	std::vector<Matrix3frm> rmats_;
    std::vector<Vector3f> tvecs_;
	Matrix3frm init_Rcam_;
    Vector3f init_tcam_;

	int globalTime;
	
	unsigned short *previousDepthData;
	
	pcl::device::Intr intrinsics;
	float trancationDistance; 

	void *headPoseEstimationMediator;
	bool headPoseEstimationOk;
};

#endif
