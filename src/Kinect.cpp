#include "Kinect.h"

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

void Kinect::imageCallBack (const boost::shared_ptr<openni_wrapper::Image>& rgbImage, const boost::shared_ptr<openni_wrapper::DepthImage>& depthImage, float constant) {

	boost::mutex::scoped_try_lock lock(data_ready_mutex);

	if (!lock)
		return;

	this->rgbImage = rgbImage;
	this->depthImage = depthImage;

	data_ready_cond.notify_one();

}
