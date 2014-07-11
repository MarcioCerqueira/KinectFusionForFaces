#include "Capture\ONIGrabber.h"

ONIGrabber::ONIGrabber(char *file) {
	
	context.Init();
	context.OpenFileRecording(file, player);
	player.SetRepeat(false);
	imageGenerator.Create(context);
	pixelFormat = imageGenerator.GetPixelFormat();
	if(pixelFormat != XN_PIXEL_FORMAT_RGB24) imageGenerator.SetPixelFormat(XN_PIXEL_FORMAT_RGB24);
	
	imageGenerator.GetMetaData(imageMap);
	XnUInt32 fps = imageMap.FPS();
	height = imageMap.YRes();
	width = imageMap.XRes();
	
	depthGenerator.Create(context);
	player.GetNumFrames(depthGenerator.GetName(), numberOfFrames);

	sourceRgbData.resize(width * height);
	sourceDepthData.resize(width * height);
	
	f = 525.f;
	currentFrame = 0;

}

ONIGrabber::~ONIGrabber() {

	context.StopGeneratingAll();
	context.Release();

}

bool ONIGrabber::grabFrame() {
	
	currentFrame++;
	if(currentFrame > numberOfFrames)
		return false;

	imageGenerator.WaitAndUpdateData();
	imageGenerator.GetMetaData(imageMap);
	
	rgbImage.rows = imageMap.YRes();
	rgbImage.cols = imageMap.XRes();
	rgbImage.step = rgbImage.cols * rgbImage.elemSize();

	unsigned char *uchartemp = (unsigned char*)&sourceRgbData[0];
	imageData = const_cast<XnRGB24Pixel*>(imageMap.RGB24Data());

	uchartemp = reinterpret_cast<unsigned char*>(imageData);
	for(int pixel = 0; pixel < (640 * 480); pixel++) {
		sourceRgbData[pixel].r = uchartemp[pixel * 3 + 0];
		sourceRgbData[pixel].g = uchartemp[pixel * 3 + 1];
		sourceRgbData[pixel].b = uchartemp[pixel * 3 + 2];
	}
	
	rgbImage.data = &sourceRgbData[0];

	depthGenerator.WaitAndUpdateData();
	depthGenerator.GetMetaData(depthMap);
	depthData = const_cast<XnDepthPixel*>(depthMap.Data());
	
	depthImage.rows = depthMap.YRes();
	depthImage.cols = depthMap.XRes();
	depthImage.step = depthImage.cols * depthImage.elemSize();

	unsigned short *ushortemp = sourceDepthData.data();
	ushortemp = reinterpret_cast<unsigned short*>(depthData);
	depthImage.data = ushortemp;

	return true;

}