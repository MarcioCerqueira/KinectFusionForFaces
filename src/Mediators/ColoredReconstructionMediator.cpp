#include "Mediators/ColoredReconstructionMediator.h"

ColoredReconstructionMediator::ColoredReconstructionMediator(Eigen::Vector3f& volumeSize) {
	colorVolume = new ColorVolume(volumeSize);
}

void ColoredReconstructionMediator::savePointCloud(TsdfVolume *tsdfVolume) {
	
	DeviceArray<RGB> colorDevice;
	DeviceArray<pcl::PointXYZ> extractedCloudDevice;

	pcl::PointCloud<PointXYZ>::Ptr hostExtractedCloud;
	pcl::PointCloud<RGB>::Ptr hostExtractedColor;
	pcl::PointCloud<PointXYZRGB>::Ptr hostCloudColor;

	hostExtractedCloud = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);
	hostExtractedColor = PointCloud<RGB>::Ptr (new PointCloud<RGB>);
	hostCloudColor = PointCloud<PointXYZRGB>::Ptr (new PointCloud<PointXYZRGB>);

	DeviceArray<PointXYZ> extracted = tsdfVolume->fetchCloud(extractedCloudDevice);
	
	extracted.download(hostExtractedCloud->points);
	hostExtractedCloud->width = (int)hostExtractedCloud->points.size ();
	hostExtractedCloud->height = 1;

	colorVolume->fetchColors(extracted, colorDevice);
	colorDevice.download(hostExtractedColor->points);
	hostExtractedColor->width = (int)hostExtractedColor->points.size ();
	hostExtractedColor->height = 1;
	
	if(hostExtractedCloud->points.size() == hostExtractedColor->points.size()) {
		
		hostCloudColor->points.resize(hostExtractedCloud->points.size());
		hostCloudColor->width = (int)hostCloudColor->points.size ();
		hostCloudColor->height = 1;
		
		for(int p = 0; p < hostCloudColor->points.size (); p++) {

			hostCloudColor->points[p].x = hostExtractedCloud->points[p].x;
			hostCloudColor->points[p].y = hostExtractedCloud->points[p].y;
			hostCloudColor->points[p].z = hostExtractedCloud->points[p].z;
			hostCloudColor->points[p].r = hostExtractedColor->points[p].r;
			hostCloudColor->points[p].g = hostExtractedColor->points[p].g;
			hostCloudColor->points[p].b = hostExtractedColor->points[p].b;
		
		}

		char fileToSave[1000];
	
		std::cout << "Write filename..." << std::endl;
		std::cin >> fileToSave;
		std::cout << fileToSave << std::endl;
		pcl::io::savePCDFile(fileToSave, *hostCloudColor); 
		std::cout << "Model saved..." << std::endl;

	}

}

void ColoredReconstructionMediator::updateColorVolume(KinfuTracker::View rgbDevice, Reconstruction *reconstruction) {
	
	colorVolume->updateColorVolume(reconstruction->getIntrinsics(), reconstruction->getTrancationDistance(), reconstruction->getRotationMatrices(), 
			reconstruction->getTranslationVectors(), reconstruction->getGlobalVertexMaps(), rgbDevice);

}