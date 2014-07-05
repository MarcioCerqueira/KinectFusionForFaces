#include "TsdfVolume.h"

template<class D, class Matx> D&
device_cast (Matx& matx) {
  return (*reinterpret_cast<D*>(matx.data ()));
}

TsdfVolume::TsdfVolume(Eigen::Vector3i& volumeSize) {

	volumeSize_(0) = volumeSize(0);
	volumeSize_(1) = volumeSize(1);
	volumeSize_(2) = volumeSize(2);
	volume_.create (device::VOLUME_Y * device::VOLUME_Z, device::VOLUME_X);
	cudaMalloc((void**)&clippingPlane.clippedRegion, 640 * 480 * sizeof(unsigned char));

	hasTSDFVisualization_ = false;
	hasClippingPlane = false;
	clippingPlane.leftX = 0;
	clippingPlane.rightX = device::VOLUME_X;
	clippingPlane.downY = device::VOLUME_Y;
	clippingPlane.upY = 0;
	clippingPlane.frontZ = 0;
	clippingPlane.backZ = device::VOLUME_Z;
	velClipping = 5;

}

TsdfVolume::~TsdfVolume() {
	
	cudaFree(clippingPlane.clippedRegion);

}

void TsdfVolume::integrateVolume(std::vector<Matrix3frm>& rmats, std::vector<Vector3f>& tvecs, device::DepthMap &depthRaw, device::Intr& intrinsics, float trancDist, 
	DeviceArray2D<float>& depthRawScaled, int globalTime) {

	Matrix3frm Rcurr = rmats[globalTime]; //  [Ri|ti] - pos of camera, i.e.
    Vector3f tcurr = tvecs[globalTime]; //  transform from camera to global coo space for (i-1)th camera pose

	device::Mat33&  device_Rcam = device_cast<device::Mat33> (Rcurr);
    float3& device_tcam = device_cast<float3>(tcurr);

    Matrix3frm Rcurr_inv = (Eigen::Matrix3f)Rcurr.inverse();
    device::Mat33&   device_Rcam_inv = device_cast<device::Mat33> (Rcurr_inv);
    float3 device_volume_size = device_cast<float3>(volumeSize_);

	device::integrateVolume(depthRaw, intrinsics, device_volume_size, device_Rcam_inv, device_tcam, trancDist, volume_, depthRawScaled);

}

void TsdfVolume::raycast(std::vector<Matrix3frm>& rmats, std::vector<Vector3f>& tvecs, device::Intr& intrinsics, float trancDist, MyPointCloud *globalPreviousPointCloud,
		int globalTime) {

	if(hasTSDFVisualization_)
		error_.create(480, 640);

	Matrix3frm Rcurr = rmats[globalTime]; //  [Ri|ti] - pos of camera, i.e.
    Vector3f tcurr = tvecs[globalTime]; //  transform from camera to global coo space for (i-1)th camera pose

	device::Mat33& device_Rcurr = device_cast<device::Mat33> (Rcurr);
    float3& device_tcurr = device_cast<float3>(tcurr);

	float3 device_volume_size = device_cast<float3>(volumeSize_);

	if(hasTSDFVisualization_) {

		device::raycast(intrinsics, device_Rcurr, device_tcurr, trancDist, device_volume_size, volume_, globalPreviousPointCloud->getVertexMaps()[0], 
			globalPreviousPointCloud->getNormalMaps()[0], error_);

	} else if(hasClippingPlane) {
	
		device::raycast(intrinsics, device_Rcurr, device_tcurr, trancDist, device_volume_size, volume_, globalPreviousPointCloud->getVertexMaps()[0], 
			globalPreviousPointCloud->getNormalMaps()[0], clippingPlane);

	} else {

		device::raycast(intrinsics, device_Rcurr, device_tcurr, trancDist, device_volume_size, volume_, globalPreviousPointCloud->getVertexMaps()[0], 
			globalPreviousPointCloud->getNormalMaps()[0]);

	}

	for (int i = 1; i < LEVELS; ++i) {

      device::resizeVMap (globalPreviousPointCloud->getVertexMaps()[i-1], globalPreviousPointCloud->getVertexMaps()[i]);
      device::resizeNMap (globalPreviousPointCloud->getNormalMaps()[i-1], globalPreviousPointCloud->getNormalMaps()[i]);

    }

}

void TsdfVolume::raycastBasedOnClipping(std::vector<Matrix3frm>& rmats, std::vector<Vector3f>& tvecs, device::Intr& intrinsics, float trancDist, 
	MyPointCloud *globalPreviousPointCloud, int globalTime) {

	Matrix3frm Rcurr = rmats[globalTime - 1]; //  [Ri|ti] - pos of camera, i.e.
    Vector3f tcurr = tvecs[globalTime - 1]; //  transform from camera to global coo space for (i-1)th camera pose

	device::Mat33& device_Rcurr = device_cast<device::Mat33> (Rcurr);
    float3& device_tcurr = device_cast<float3>(tcurr);

	float3 device_volume_size = device_cast<float3>(volumeSize_);

	device::raycast(intrinsics, device_Rcurr, device_tcurr, trancDist, device_volume_size, volume_, globalPreviousPointCloud->getVertexMaps()[0], 
		globalPreviousPointCloud->getNormalMaps()[0], clippingPlane);

}

DeviceArray<PointXYZ> TsdfVolume::fetchCloud(DeviceArray<PointXYZ>& cloud_buffer) {
	
	if (cloud_buffer.empty ())
		cloud_buffer.create (DEFAULT_CLOUD_BUFFER_SIZE);

	float3 device_volume_size = device_cast<const float3> (volumeSize_);
	size_t size = device::extractCloud (volume_, device_volume_size, cloud_buffer);
	return (DeviceArray<PointXYZ> (cloud_buffer.ptr (), size));
}

void TsdfVolume::getHostErrorInRGB(KinfuTracker::View &errorInRGB) {

	errorInRGB.create(480, 640);
	device::generateTSDFErrorImage(error_, errorInRGB);

}