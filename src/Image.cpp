#include "Image.h"

template<class D, class Matx> D&
device_cast (Matx& matx)
{
  return (*reinterpret_cast<D*>(matx.data ()));
}

Image::Image(int cols, int rows) {

	cols_ = cols;
	rows_ = rows;
	trancationDistance_ = 30; //mm
	allocateBuffers(cols, rows);

}

void Image::setDepthIntrinsics (float fx, float fy, float cx, float cy)  {

  intrinsics.fx = fx;
  intrinsics.fy = fy;
  intrinsics.cx = (cx == -1) ? cols_/2 : cx;
  intrinsics.cy = (cy == -1) ? rows_/2 : cy;

}

void Image::setTrancationDistance(Eigen::Vector3i volumeSize) {

	float cx = volumeSize (0) / device::VOLUME_X;
	float cy = volumeSize (1) / device::VOLUME_Y;
	float cz = volumeSize (2) / device::VOLUME_Z;

	trancationDistance_ = max (trancationDistance_, 2.1f * max (cx, max (cy, cz)));

}

void Image::applyBilateralFilter() {

	device::bilateralFilter (depthDevice_, depths_curr_[0]);
	//depthDevice_.copyTo(depths_curr_[0]);
}

void Image::applyDepthTruncation(float truncValue) {

	int cols;
	device::truncateDepth(depths_curr_[0], truncValue);
	depths_curr_[0].download(sourceDepthData, cols);
	depthMap_.data = &sourceDepthData[0];

}

void Image::applyDepthTruncation(device::DepthMap& depthMap, float truncValue) {

	device::truncateDepth(depthMap, truncValue);

}

void Image::applyPyrDown() {

	for (int i = 1; i < LEVELS; ++i)
      device::pyrDown (depths_curr_[i-1], depths_curr_[i]);

}

void Image::convertToPointCloud(MyPointCloud *currentPointCloud) {

	for (int i = 0; i < LEVELS; ++i)
    {
      device::createVMap (intrinsics(i), depths_curr_[i], currentPointCloud->getVertexMaps()[i]);
      //device::createNMap(currentPointCloud->getVertexMaps()[i], currentPointCloud->getNormalMaps()[i]);
      device::computeNormalsEigen (currentPointCloud->getVertexMaps()[i], currentPointCloud->getNormalMaps()[i]);
    }

}

void Image::load(boost::shared_ptr<openni_wrapper::Image>& rgbImage, boost::shared_ptr<openni_wrapper::DepthImage>& depthImage) {

	depthMap_.cols = depthImage->getWidth();
	depthMap_.rows = depthImage->getHeight();
	depthMap_.step = depthMap_.cols * depthMap_.elemSize();
	
	sourceDepthData.resize(depthMap_.cols * depthMap_.rows);
	depthImage->fillDepthImageRaw(depthMap_.cols, depthMap_.rows, &sourceDepthData[0]);
	depthMap_.data = &sourceDepthData[0];

	rgbMap_.cols = rgbImage->getWidth();
	rgbMap_.rows = rgbImage->getHeight();
	rgbMap_.step = rgbMap_.cols * rgbMap_.elemSize();

	sourceRgbData.resize(rgbMap_.cols * rgbMap_.rows);
	rgbImage->fillRGB(rgbMap_.cols, rgbMap_.rows, (unsigned char*)&sourceRgbData[0]);
	rgbMap_.data = &sourceRgbData[0];
	
	updateDeviceData();

}

void Image::updateDeviceData() {
	
	rgbDevice_.upload(rgbMap_.data, rgbMap_.step, rgbMap_.rows, rgbMap_.cols);
	depthDevice_.upload(depthMap_.data, depthMap_.step, depthMap_.rows, depthMap_.cols);
	
}
	
void Image::allocateBuffers(int cols, int rows) {

	depths_curr_.resize (LEVELS);

	for (int i = 0; i < LEVELS; ++i)
	{

		int pyr_rows = rows >> i;
		int pyr_cols = cols >> i;

		depths_curr_[i].create (pyr_rows, pyr_cols);

	}

	depthRawScaled_.create (rows, cols);

}

unsigned char* Image::getRaycastImage(Eigen::Vector3f volumeSize, MyPointCloud *globalPreviousPointCloud) {
	
	int cols;
	const Eigen::Vector3f& lightSourcePose = volumeSize * (-3.f);

	device::LightSource light;
	light.number = 1;
	light.pos[0] = device_cast<const float3>(lightSourcePose);

	viewDevice.create(rows_, cols_);
	generateImage (globalPreviousPointCloud->getVertexMaps()[0], globalPreviousPointCloud->getNormalMaps()[0], light, viewDevice);
	viewDevice.download (viewHost, cols);

	return (unsigned char*)viewHost.data();

}