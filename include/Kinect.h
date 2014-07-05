#ifndef KINECT_H
#define KINECT_H

#include <pcl/io/openni_grabber.h>

class Kinect
{
public:
	Kinect();
	~Kinect();
	bool grabFrame();
	boost::shared_ptr<openni_wrapper::Image>& getRGBImage() { return rgbImage; }
	boost::shared_ptr<openni_wrapper::DepthImage>& getDepthImage() { return depthImage; }
	int getImageWidth() { return rgbImage->getWidth(); }
	int getImageHeight() { return rgbImage->getHeight(); }
	float getFocalLength() { return f; }
private:
	void imageCallBack (const boost::shared_ptr<openni_wrapper::Image>& rgbImage, const boost::shared_ptr<openni_wrapper::DepthImage>& depthImage, float constant);
	
	pcl::Grabber *capture;
	boost::condition_variable data_ready_cond;
	boost::mutex data_ready_mutex;
	boost::shared_ptr<openni_wrapper::Image> rgbImage;
	boost::shared_ptr<openni_wrapper::DepthImage> depthImage;
	float f;
};

#endif