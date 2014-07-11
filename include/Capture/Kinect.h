#ifndef KINECT_H
#define KINECT_H

#include <pcl/io/openni_grabber.h>
#include "AbstractCapture.h"

class Kinect : public AbstractCapture
{
public:
	Kinect();
	~Kinect();
	bool grabFrame();
private:
	void imageCallBack (const boost::shared_ptr<openni_wrapper::Image>& rgbImage, const boost::shared_ptr<openni_wrapper::DepthImage>& depthImage, float constant);
	
	pcl::Grabber *capture;
	boost::condition_variable data_ready_cond;
	boost::mutex data_ready_mutex;
	
};

#endif