#ifndef FACEDETECTION_H
#define FACEDETECTION_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/gpu/gpu.hpp>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni_camera/openni_image_rgb24.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>
#include "Image.h"

class FaceDetection {
public:
	FaceDetection(char *cascadeFileName);
	~FaceDetection();
	bool run(Image *image);
	void segmentFace(Image *image);
private:
	//Given a cascade, it computes pt1 and pt2 and segments the face from the depth data
	bool detectFace(IplImage* img, unsigned short *depthData);	
	//Given pt1 and pt2, it segments the face from the depth data
	void segmentFace(IplImage *img, unsigned short *depthData);
	char cascade_name[100];
	CvPoint pt1, pt2;
	IplImage *rgbImage;
	unsigned short *depthData;
	bool isCascadeLoaded;
	cv::gpu::CascadeClassifier_GPU GPUCascade;
};

#endif
