#include "HeadPoseEstimator/HeadPoseEstimator.h"

HeadPoseEstimator::HeadPoseEstimator(char *configFileName) {

	loadDefaults();
	loadConfigFile(configFileName);

	crfe = new CRForestEstimator();
	if( !crfe->loadForest(g_treepath.c_str(), g_ntrees)) {
		std::cout << "could not read forest!" << std::endl;
		return;
	}

	pointCloud = new MyCVPointCloud(640, 480);
	cylinderCoeff.values.resize(7);
	success = false;

	rotationMatrixEstimated = Eigen::Matrix3f::Identity();

}

HeadPoseEstimator::~HeadPoseEstimator() {
	
	delete crfe;
	delete pointCloud;

}

void HeadPoseEstimator::loadDefaults() {

	g_max_z = 0;
	g_th = 400;
	g_prob_th = 1.0f;
	g_maxv = 1000.f;
	g_stride = 5;
	g_larger_radius_ratio = 1.f;
	g_smaller_radius_ratio = 6.f;

}


void HeadPoseEstimator::loadConfigFile(char *configFileName) {
	
	std::ifstream in(configFileName);
	std::string dummy;

	if(in.is_open()) {

		// Path to trees
		in >> dummy;
		in >> g_treepath;

		// Number of trees
		in >> dummy;
		in >> g_ntrees;

		in >> dummy;
		in >> g_maxv;

		in >> dummy;
		in >> g_larger_radius_ratio;

		in >> dummy;
		in >> g_smaller_radius_ratio;

		in >> dummy;
		in >> g_stride;

		in >> dummy;
		in >> g_max_z;

		in >> dummy;
		in >> g_th;


	} else {
		std::cout << "File not found " << configFileName << std::endl;
		exit(-1);
	}

	in.close();

	std::cout << std::endl << "------------------------------------" << std::endl << std::endl;
	std::cout << "Estimation:       " << std::endl;
	std::cout << "Trees:            " << g_ntrees << " " << g_treepath << std::endl;
	std::cout << "Stride:           " << g_stride << std::endl;
	std::cout << "Max Variance:     " << g_maxv << std::endl;
	std::cout << "Max Distance:     " << g_max_z << std::endl;
	std::cout << "Head Threshold:   " << g_th << std::endl;

	std::cout << std::endl << "------------------------------------" << std::endl << std::endl;
	
}

void HeadPoseEstimator::run(unsigned short *data) {

	pointCloud->load(data, g_max_z);

	means.clear();
	votes.clear();
	clusters.clear();

	//do the actual estimation
	crfe->estimate( pointCloud->getCVPointCloud(), means, clusters, votes, g_stride, g_maxv, g_prob_th, g_larger_radius_ratio,
						g_smaller_radius_ratio, false, g_th );
		
	if(means.size() > 0) {
		
		float mult = 0.0174532925f;
		for(unsigned int i = 0; i < means.size(); i++) {
		
			Eigen::Vector3f headFront;
			Eigen::Vector3f faceDir(0, 0, -1);
			Eigen::Vector3f faceCurrDir;
			pcl::PointXYZ p1, p2, d;

			eulerToRotationMatrix(rotationMatrixEstimated, mult * means[i][3], mult * means[i][4], mult * means[i][5]);

			headCenter(0) = means[i][0];
			headCenter(1) = means[i][1];
			headCenter(2) = means[i][2];
			
			eulerAngles(0) = means[i][3];
			eulerAngles(1) = means[i][4];
			eulerAngles(2) = means[i][5];

			faceCurrDir = rotationMatrixEstimated * faceDir;

			headFront(0) = headCenter(0) + 150.f * faceCurrDir(0);
			headFront(1) = headCenter(1) + 150.f * faceCurrDir(1);
			headFront(2) = headCenter(2) + 150.f * faceCurrDir(2);
				
			p1.x = headCenter(0); p1.y = headCenter(1); p1.z = headCenter(2);
			p2.x = headFront(0); p2.y = headFront(1); p2.z = headFront(2);
			
			float n = sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
			
			cylinderCoeff.values[0] = p1.x;
			cylinderCoeff.values[1] = p1.y;
			cylinderCoeff.values[2] = p1.z;
			cylinderCoeff.values[3] = p2.x - p1.x;
			cylinderCoeff.values[4] = p2.y - p1.y;
			cylinderCoeff.values[5] = p2.z - p1.z;
			cylinderCoeff.values[6] = 8; //radius
			success = true;
		}

	} else {
		success = false;
	}

}

void HeadPoseEstimator::eulerToRotationMatrix(Eigen::Matrix3f& rotationMatrix, float x, float y, float z) {
	
	float A       = cos(x);
	float B       = sin(x);
	float C       = cos(y);
	float D       = sin(y);
	float E       = cos(z);
	float F       = sin(z);

	float AD      =   A * -D;
	float BD      =   B * -D;

	rotationMatrix(0, 0)  =   C * E;
	rotationMatrix(0, 1)  =  -C * F;
	rotationMatrix(0, 2)  =  D;

	rotationMatrix(1, 0)  = -BD * E + A * F;
	rotationMatrix(1, 1)  =  BD * F + A * E;
	rotationMatrix(1, 2)  =  -B * C;

	rotationMatrix(2, 0)  =  AD * E + B * F;
	rotationMatrix(2, 1)  = -AD * F + B * E;
	rotationMatrix(2, 2) =   A * C;

}
