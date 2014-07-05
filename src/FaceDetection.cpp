#include "FaceDetection.h"
#include <iostream>

FaceDetection::FaceDetection(char *cascadeFileName)
{
	std::ifstream in(cascadeFileName);
	std::string info;

	if(in.is_open())
	{

		in >> info;
		in >> cascade_name;
    
	} else {
		std::cout << "Cascade Config File not found" << std::endl;
	}
	in.close();

	isCascadeLoaded = false;
	rgbImage = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
	depthData = (unsigned short*)malloc(640 * 480 * sizeof(unsigned short));

}

FaceDetection::~FaceDetection()
{
	cvReleaseImage(&rgbImage);
	delete [] depthData;
}

void FaceDetection::segmentFace(IplImage *img, unsigned short *depthData)
{
	// Pintando tudo fora do retângulo de preto
	int height = img->height;
	int width  = img->width;
	int x, y;

	for(int pixel = 0; pixel < width * height; pixel++)
	{
		x = pixel % width;
		y = pixel / width;
		if(x < pt1.x || y < pt1.y || x > pt2.x || y > pt2.y)
		{
			img->imageData[pixel * 3 + 0] = 0;
			img->imageData[pixel * 3 + 1] = 0;
			img->imageData[pixel * 3 + 2] = 0;
		}
	}

	// Draw the rectangle in the input image
    cvRectangle( img, pt1, pt2, CV_RGB(255,0,0), 3, 8, 0 );
		
	//We update the depth map based on the region of the face
	for(int pixel = 0; pixel < img->width * img->height; pixel++)
		if(img->imageData[pixel * 3 + 0] == 0 && img->imageData[pixel * 3 + 1] == 0 && img->imageData[pixel * 3 + 2] == 0)
			depthData[pixel] = 0;

}

bool FaceDetection::detectFace(IplImage* img, unsigned short *depthData)
{

	
	IplImage *grayImg = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);
	cvCvtColor(img, grayImg, CV_BGR2GRAY);
	cvEqualizeHist(grayImg, grayImg);

	cv::gpu::GpuMat GPUImage(cv::Mat(grayImg, true)), GPUFaces;
	cv::Mat objHost;
	int detected = GPUCascade.detectMultiScale(GPUImage, GPUFaces, 1.2, 2, cv::Size(40, 40));
	GPUFaces.colRange(0, detected).download(objHost);
	cv::Rect* cfaces = objHost.ptr<cv::Rect>();

	int scale = 1;
	int i;
	cvReleaseImage(&grayImg);
    // Loop the number of faces found.
	for(i = 0; i < detected; i++)
	{
		pt1.x = cfaces[i].tl().x * scale - 10;
		pt2.x = (cfaces[i].tl().x + cfaces[i].size().width) * scale + 10;
		pt1.y = cfaces[i].tl().y * scale - 10;
		pt2.y = (cfaces[i].tl().y + cfaces[i].size().height) * scale + 10;
 		return true;
		
	}
	return false;
}
// Function to detect and draw any faces that is present in an image
bool FaceDetection::run(Image *image)
{

	if(!isCascadeLoaded) {
		GPUCascade.load(cascade_name);
		isCascadeLoaded = true;
	}

	// Check whether the cascade has loaded successfully. Else report and error and quit
	if(!isCascadeLoaded)
    {
		fprintf( stderr, "ERROR: Could not load classifier cascade\n" );
		return false;
    }

	static CvMemStorage* storage = 0;
	// Allocate the memory storage
	if(!storage)
		storage = cvCreateMemStorage(0);
	
	// Convert the data
	this->rgbImage->imageData = (char*)image->getRGBMap().data;
	this->depthData = (unsigned short*)image->getDepthMap().data;

	// Find whether the cascade is loaded, to find the faces. If yes, then:
	if(isCascadeLoaded)
    {
		if(detectFace(this->rgbImage, depthData))
		{
			segmentFace(this->rgbImage, depthData);
			memcpy((void*)image->getRGBMap().data, this->rgbImage->imageData, this->rgbImage->width * this->rgbImage->height * 3 * sizeof(char));
			memcpy((void*)image->getDepthMap().data, this->depthData, this->rgbImage->width * this->rgbImage->height * sizeof(unsigned short));
			image->updateDeviceData();
			return true;
		}
	}
	
	// Clear the memory storage which was used before
    cvClearMemStorage( storage );
	return false;

}

void FaceDetection::segmentFace(Image *image)
{
	// Convert the data
	this->rgbImage->imageData = (char*)image->getRGBMap().data;
	this->depthData = (unsigned short*)image->getDepthMap().data;

	segmentFace(this->rgbImage, this->depthData);
	
	memcpy((void*)image->getRGBMap().data, this->rgbImage->imageData, this->rgbImage->width * this->rgbImage->height * 3 * sizeof(char));
	memcpy((void*)image->getDepthMap().data, this->depthData, this->rgbImage->width * this->rgbImage->height * sizeof(unsigned short));
	image->updateDeviceData();
	
}