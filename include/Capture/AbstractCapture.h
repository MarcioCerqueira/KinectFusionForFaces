#ifndef ABSTRACT_CAPTURE_H
#define ABSTRACT_CAPTURE_H

#include <pcl/gpu/containers/device_array.hpp>
#include "openni_capture.h"

class AbstractCapture
{
protected: 
	pcl::gpu::PtrStepSz<const unsigned short> depthImage;
	pcl::gpu::PtrStepSz<const pcl::gpu::CaptureOpenNI::RGB> rgbImage;
	float f;
	std::vector<unsigned short> sourceDepthData;
	std::vector<pcl::gpu::KinfuTracker::RGB> sourceRgbData;
public:
	pcl::gpu::PtrStepSz<const pcl::gpu::CaptureOpenNI::RGB> getRGBImage() { return rgbImage; }
	pcl::gpu::PtrStepSz<const unsigned short> getDepthImage() { return depthImage; }
	int getImageWidth() { return rgbImage.cols; }
	int getImageHeight() { return rgbImage.rows; }
	float getFocalLength() { return f; }
	virtual bool grabFrame() = 0;
};

#endif