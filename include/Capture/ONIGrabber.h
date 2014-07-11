#ifndef ONIGRABBER_H
#define ONIGRABBER_H

#include <XnCppWrapper.h>
#include <pcl\io\openni_grabber.h>
#include <pcl\io\openni_camera\openni_image_rgb24.h>
#include "AbstractCapture.h"


class ONIGrabber : public AbstractCapture
{
public:
	ONIGrabber(char *file);
	~ONIGrabber();
	bool grabFrame();
private:
	XnUInt32 numberOfFrames;
	int currentFrame;
	int width;
	int height;
	xn::Context context;
	xn::Player player;
	xn::ImageGenerator imageGenerator;
	xn::DepthGenerator depthGenerator;
	xn::ImageMetaData imageMap;
	xn::DepthMetaData depthMap;
	XnPixelFormat pixelFormat;
	XnRGB24Pixel* imageData;
	XnDepthPixel* depthData;
	xn::ImageMetaData *teste;
};

#endif