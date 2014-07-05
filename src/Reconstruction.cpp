#include "Reconstruction.h"

#define _CRT_SECURE_NO_DEPRECATE
#define NOMINMAX

typedef pcl::ScopeTime ScopeTimeT;

Reconstruction::Reconstruction(Eigen::Vector3i& volumeSize) {

	hasImage_ = false;
	hasIncrement_ = true;
	hasErrorVisualization_ = false;
	hasTsdfVolumeVisualization_ = false;

	isOnlyTrackingOn_ = false;
	stopTracking_ = false;

	headPoseEstimationOk = false;

	tsdfVolume_ = new TsdfVolume(volumeSize);

	currentPointCloud_ = new MyPointCloud(640, 480);
	globalPreviousPointCloud_ = new MyPointCloud(640, 480);

	init_Rcam_ = Eigen::Matrix3f::Identity ();// * AngleAxisf(-30.f/180*3.1415926, Vector3f::UnitX());
	//init_tcam_ = Eigen::Vector3f::Zero ();
	init_tcam_ = tsdfVolume_->getVolumeSize() * 0.5f - Vector3f (0, 0, tsdfVolume_->getVolumeSize() (2) / 2 * 1.2f);
	
	rmats_.reserve (30000);
	tvecs_.reserve (30000);

	reset();

	previousDepthData = new unsigned short[640 * 480];

}

pcl::PointCloud<pcl::PointXYZ>::Ptr Reconstruction::extractFullPointCloud() {

	DeviceArray<pcl::PointXYZ> extractedCloudDevice;
	DeviceArray<PointXYZ> extracted =  tsdfVolume_->fetchCloud(extractedCloudDevice);

	pcl::PointCloud<pcl::PointXYZ>::Ptr hostCloud = pcl::PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);
	extracted.download(hostCloud->points);
	hostCloud->width = (int)hostCloud->points.size ();
	hostCloud->height = 1;

	return hostCloud;

}

void Reconstruction::savePointCloud() {
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr fullCloud = extractFullPointCloud();
	char fileToSave[1000];
	std::cout << "Write filename..." << std::endl;
	std::cin >> fileToSave;
	std::cout << fileToSave << std::endl;
	pcl::io::savePCDFile(fileToSave, *fullCloud); 
	std::cout << "Model saved..." << std::endl;

}

void Reconstruction::reset() {

	if(!isOnlyTrackingOn_) {
	
		rmats_.clear ();
		tvecs_.clear ();

		rmats_.push_back (init_Rcam_);
		tvecs_.push_back (init_tcam_);
		tsdfVolume_->reset();

		globalTime = 0;

		hasIncrement_ = false;

		std::cout << "Reset" << std::endl;
	
	}

}

void Reconstruction::run(Image *image) {

	if(globalTime == 0) {
		
		tsdfVolume_->integrateVolume(rmats_, tvecs_, image->getDepthDevice(), intrinsics, trancationDistance, image->getDepthRawScaled(), globalTime);
		currentPointCloud_->transformPointCloud(rmats_[0], tvecs_[0], globalPreviousPointCloud_->getVertexMaps(), globalPreviousPointCloud_->getNormalMaps());
				
	} else {
			
		if(!stopTracking_) {

			hasImage_ = currentPointCloud_->alignPointClouds(rmats_, tvecs_, globalPreviousPointCloud_, intrinsics, globalTime);

#if (USE_HEAD_POSE_ESTIMATION)
			if(!hasImage_ && !isOnlyTrackingOn_ && headPoseEstimationOk) 
				((HeadPoseEstimationMediator*)headPoseEstimationMediator)->runHeadPoseEstimationPlusICP((unsigned short*)image->getDepthMap().data, this);
#endif

			if(!hasImage_)
				reset();
			else {
				
				if(!isOnlyTrackingOn_)
					tsdfVolume_->integrateVolume(rmats_, tvecs_, image->getDepthDevice(), intrinsics, trancationDistance, image->getDepthRawScaled(), globalTime);
						
				tsdfVolume_->raycast(rmats_, tvecs_, intrinsics, trancationDistance, globalPreviousPointCloud_, globalTime);
				pcl::device::sync ();
						
			}

		} else
			hasIncrement_ = false;

	}

	if(hasIncrement_) {

		globalTime++;
		for(int pixel = 0; pixel < 640 * 480; pixel++)
			previousDepthData[pixel] = image->getDepthMap().data[pixel];
		
	} else
		hasIncrement_ = true;

}

void Reconstruction::enableOnlyTracking(bool stopFaceDetection) 
{
	isOnlyTrackingOn_ = true;	
}

bool Reconstruction::reRunICP() 
{

	hasImage_ = currentPointCloud_->alignPointClouds(rmats_, tvecs_, globalPreviousPointCloud_, intrinsics, globalTime);
	if(hasImage_)
		std::cout << "Error: " << currentPointCloud_->computeFinalError() << std::endl;
	else
		std::cout << "ICP Failed" << std::endl;
	return hasImage_;

}

void Reconstruction::reRunRaycasting() 
{	
	tsdfVolume_->raycast(rmats_, tvecs_, intrinsics, trancationDistance, globalPreviousPointCloud_, globalTime);
}

void Reconstruction::transformGlobalPreviousPointCloud(Eigen::Matrix3f& Rinc, Eigen::Vector3f& tvec, Eigen::Vector3f& centerOfMass)
{

	globalPreviousPointCloud_->transformPointCloud(Rinc, tvec, globalPreviousPointCloud_->getVertexMaps(), 
		globalPreviousPointCloud_->getNormalMaps(), init_tcam_, centerOfMass);

}

/*
void Reconstruction::transformGlobalPreviousToCurrentPointCloud()
{
	Matrix3frm invCurrRot = this->getCurrentRotation().inverse();
	globalPreviousPointCloud_->transformPointCloud(invCurrRot, this->getCurrentTranslation(), currentPointCloud_->getVertexMaps(), 
		currentPointCloud_->getNormalMaps(), true);
}
*/


Reconstruction::~Reconstruction() {

	delete tsdfVolume_;
	delete currentPointCloud_;
	delete globalPreviousPointCloud_;

	delete [] previousDepthData;

}

