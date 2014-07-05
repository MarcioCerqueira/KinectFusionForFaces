#ifndef MYCVPOINTCLOUD_H
#define MYCVPOINTCLOUD_H

#include <opencv2/opencv.hpp>

class MyCVPointCloud
{
public:
	MyCVPointCloud(int width, int height);
	void load(unsigned short *data, int max_z);
	cv::Mat getCVPointCloud() { return cvPointCloud; }
	float getFocalLength() { return f; }
private:
	cv::Mat cvPointCloud;
	float f;
};

#endif
