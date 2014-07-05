#include "Mediators/HeadPoseEstimationMediator.h"

HeadPoseEstimationMediator::HeadPoseEstimationMediator(char *HPEConfigFileName) {
	
	this->HPEConfigFileName = HPEConfigFileName;
	headPoseEstimator = new HeadPoseEstimator(HPEConfigFileName);

}

HeadPoseEstimationMediator::~HeadPoseEstimationMediator() {
	delete headPoseEstimator;
}

void HeadPoseEstimationMediator::stopTracking(bool stop, unsigned short *currentDepthMap, Reconstruction *reconstruction) {
	
	headPoseEstimator->run(currentDepthMap);

	if(headPoseEstimator->hadSuccess()) {
		
		if(stop) {

			previousRotationMatrixEstimated = headPoseEstimator->getRotationMatrixEstimated();
			previousHeadCenter = headPoseEstimator->getHeadCenter() + reconstruction->getInitialTranslation();
		
		} else {
		
			Eigen::Vector3f tvec = Eigen::Vector3f::Zero();
			Eigen::Vector3f headCenter = headPoseEstimator->getHeadCenter();
			headCenter += reconstruction->getInitialTranslation();
			
			std::cout << "ICP running.." << std::endl;
			if(!reconstruction->reRunICP()) {
			
				Eigen::Vector3f diff = previousHeadCenter - headCenter;
			
				reconstruction->getTranslationVectors()[reconstruction->getGlobalTime()] = reconstruction->getTranslationVectors()[reconstruction->getGlobalTime() - 1] + diff;
				reconstruction->getRotationMatrices()[reconstruction->getGlobalTime()] = reconstruction->getRotationMatrices()[reconstruction->getGlobalTime() - 1];
		
				reconstruction->reRunRaycasting();
				reconstruction->incrementGlobalTime();

				Eigen::Matrix3f Rinc = headPoseEstimator->getRotationMatrixEstimated() * previousRotationMatrixEstimated.inverse();
				Eigen::Vector3f centroid = reconstruction->getGlobalCentroid();
				//diff = centroid - headCenter;
	
				headCenter = headPoseEstimator->getHeadCenter();
				reconstruction->transformGlobalPreviousPointCloud(Rinc, tvec, headCenter);

				std::cout << "ICP+HPE running..." << std::endl;
				reconstruction->reRunICP();
			
			}

		}

	} else
		std::cout << "Head Pose Estimation Failed" << std::endl;

}

void HeadPoseEstimationMediator::runHeadPoseEstimationPlusICP(unsigned short *currentDepthMap, Reconstruction *reconstruction) {

	std::cout << "Running ICP + HPE" << std::endl;
	
	headPoseEstimator->run(reconstruction->getPreviousDepthMap());
	previousRotationMatrixEstimated = headPoseEstimator->getRotationMatrixEstimated();
	previousHeadCenter = headPoseEstimator->getHeadCenter() + reconstruction->getInitialTranslation();
	headPoseEstimator->run(currentDepthMap);

	if(headPoseEstimator->hadSuccess()) {
		
		Eigen::Vector3f tvec = Eigen::Vector3f::Zero();
		Eigen::Vector3f headCenter = headPoseEstimator->getHeadCenter() + reconstruction->getInitialTranslation();
	
		Eigen::Vector3f diff = previousHeadCenter - headCenter;
		Eigen::Matrix3f Rinc = headPoseEstimator->getRotationMatrixEstimated() * previousRotationMatrixEstimated.inverse();
	
		reconstruction->getTranslationVectors()[reconstruction->getGlobalTime()] = reconstruction->getTranslationVectors()[reconstruction->getGlobalTime() - 1] + diff;
		reconstruction->getRotationMatrices()[reconstruction->getGlobalTime()] = reconstruction->getRotationMatrices()[reconstruction->getGlobalTime() - 1];
		
		reconstruction->reRunRaycasting();
		reconstruction->incrementGlobalTime();

		headCenter = headPoseEstimator->getHeadCenter();
		reconstruction->transformGlobalPreviousPointCloud(Rinc, tvec, headCenter);
		reconstruction->reRunICP();
			
	} else
		std::cout << "Head Pose Estimation Failed" << std::endl;

}