#include "Capture/Kinect.h"

Kinect::Kinect()
{

	capture = new pcl::OpenNIGrabber();
	boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float constant)> callbackFunction
		= boost::bind (&Kinect::imageCallBack, this, _1, _2, _3);
	capture->registerCallback(callbackFunction);
	capture->start();

	f = 525.f;

}

Kinect::~Kinect()
{
	capture->stop();
	delete capture;
}

bool Kinect::grabFrame()
{
	boost::unique_lock<boost::mutex> lock(data_ready_mutex);
	return data_ready_cond.timed_wait (lock, boost::posix_time::millisec(200));
}

void Kinect::imageCallBack (const boost::shared_ptr<openni_wrapper::Image>& kRgbImage, const boost::shared_ptr<openni_wrapper::DepthImage>& kDepthImage, float constant) {

	boost::mutex::scoped_try_lock lock(data_ready_mutex);

	if (!lock)
		return;

	depthImage.cols = kDepthImage->getWidth();
	depthImage.rows = kDepthImage->getHeight();
	depthImage.step = depthImage.cols * depthImage.elemSize();
	
	sourceDepthData.resize(depthImage.cols * depthImage.rows);
	kDepthImage->fillDepthImageRaw(depthImage.cols, depthImage.rows, &sourceDepthData[0]);
	depthImage.data = &sourceDepthData[0];

	rgbImage.cols = kRgbImage->getWidth();
	rgbImage.rows = kRgbImage->getHeight();
	rgbImage.step = rgbImage.cols * rgbImage.elemSize();

	sourceRgbData.resize(rgbImage.cols * rgbImage.rows);
	kRgbImage->fillRGB(rgbImage.cols, rgbImage.rows, (unsigned char*)&sourceRgbData[0]);
	rgbImage.data = &sourceRgbData[0];
	
	data_ready_cond.notify_one();

}
