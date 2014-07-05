#include "ColorVolume.h"

template<class D, class Matx> D&
device_cast (Matx& matx)
{
  return (*reinterpret_cast<D*>(matx.data ()));
}

ColorVolume::ColorVolume(Eigen::Vector3f& tsdfSize, int maxWeight) {

	this->maxWeight = maxWeight < 0 ? maxWeight : maxWeight;
	this->maxWeight = this->maxWeight > 255 ? 255 : this->maxWeight;

	volumeResolution(0) = device::VOLUME_X;
	volumeResolution(1) = device::VOLUME_Y;
	volumeResolution(2) = device::VOLUME_Z;

	volumeSize = tsdfSize;
	colorVolume.create(volumeResolution(1) * volumeResolution(2), volumeResolution(0));
	reset();
}

void ColorVolume::updateColorVolume(device::Intr &intr, float trancDist, std::vector<Matrix3frm> &rmats_, std::vector<Eigen::Vector3f> &tvecs_, 
	std::vector<device::MapArr> &vmapsGPrev, KinfuTracker::View &rgbDevice) {

	float3 device_volume_size = device_cast<float3>(volumeSize);

	Matrix3frm R_inv = rmats_.back().inverse();
    Eigen::Vector3f t = tvecs_.back();

	device::Mat33&  device_Rcurr_inv = device_cast<device::Mat33> (R_inv);
    float3& device_tcurr = device_cast<float3> (t);
	//std::cout << volumeSize(0) << " " << volumeSize(1) << " " << volumeSize(2) << std::endl;
	
	device::updateColorVolume(intr, trancDist, device_Rcurr_inv, device_tcurr, vmapsGPrev[0], rgbDevice, device_cast<const float3>(volumeSize), colorVolume, maxWeight);
}

void ColorVolume::fetchColors(const DeviceArray<PointXYZ>& cloud, DeviceArray<RGB>& colors) {

	colors.create(cloud.size());
	device::exctractColors(colorVolume, device_cast<const float3>(volumeSize), cloud, (uchar4*)colors.ptr()/*bgra*/); 

}


void ColorVolume::reset() {

	device::initColorVolume(colorVolume);

}