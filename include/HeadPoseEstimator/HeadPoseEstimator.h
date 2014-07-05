#ifndef HEADPOSEESTIMATOR_H
#define HEADPOSEESTIMATOR_H

#include <vector>
#include <fstream>
#include <string>
#include <iostream>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include "CRForestEstimator.h"
#include "MyCVPointCloud.h"

//Fanelli's Head Pose Estimator
class HeadPoseEstimator
{
public:
	HeadPoseEstimator(char *configFileName);
	~HeadPoseEstimator();
	void run(unsigned short *data);
	pcl::ModelCoefficients& getCylinder() { return cylinderCoeff; }
	Eigen::Matrix3f& getRotationMatrixEstimated() { return rotationMatrixEstimated; }
	Eigen::Vector3f& getHeadCenter() { return headCenter; }
	Eigen::Vector3f& getEulerAngles() { return eulerAngles; }
	bool hadSuccess() { return success; }
	void eulerToRotationMatrix(Eigen::Matrix3f& rotationMatrix, float x, float y, float z);
private:
	void loadDefaults();
	void loadConfigFile(char *configFileName);
	// Path to trees
	std::string g_treepath;
	// Number of trees
	int g_ntrees;
	// Patch width
	int g_p_width;
	// Patch height
	int g_p_height;
	//maximum distance form the sensor - used to segment the person
	int g_max_z;
	//head threshold - to classify a cluster of votes as a head
	int g_th;
	//threshold for the probability of a patch to belong to a head
	float g_prob_th;
	//threshold on the variance of the leaves
	float g_maxv;
	//stride (how densely to sample test patches - increase for higher speed)
	int g_stride;
	//radius used for clustering votes into possible heads
	float g_larger_radius_ratio;
	//radius used for mean shift
	float g_smaller_radius_ratio;
	std::vector< cv::Vec<float,POSE_SIZE> > means; //outputs
	std::vector< std::vector< Vote > > clusters; //full clusters of votes
	std::vector< Vote > votes; //all votes returned by the forest
	Eigen::Matrix3f rotationMatrixEstimated;
	Eigen::Vector3f headCenter;
	Eigen::Vector3f eulerAngles;
	bool success;
	pcl::ModelCoefficients cylinderCoeff;
	CRForestEstimator *crfe;
	MyCVPointCloud *pointCloud;
};
#endif
