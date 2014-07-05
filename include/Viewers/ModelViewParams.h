#ifndef MODELVIEW_PARAMS_H
#define MODELVIEW_PARAMS_H

#include <Eigen\Core>


typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;

typedef struct ModelViewParams
{
	float translationVector[3];
	float rotationAngles[3];
	Eigen::Vector3f gTrans;
	Matrix3frm gRot;
	Eigen::Vector3f initialTranslation;
	float rotationIndices[3];
	Eigen::Vector3f headCenter;
	Eigen::Vector3f headCenterRotated;
	Eigen::Vector3f headEulerAngles;
	bool useTextureRotation;
	bool useHeadPoseRotation;
} ModelViewParams;

#endif
