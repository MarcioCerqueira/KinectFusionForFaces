#ifndef COLORED_RECONSTRUCTION_MEDIATOR_H
#define COLORED_RECONSTRUCTION_MEDIATOR_H

#include "ColorVolume.h"
#include "TsdfVolume.h"
#include "Reconstruction.h"
#include <pcl/io/pcd_io.h>

class ColoredReconstructionMediator
{
public:
	ColoredReconstructionMediator(Eigen::Vector3f& volumeSize);
	void savePointCloud(TsdfVolume* tsdfVolume);
	void updateColorVolume(KinfuTracker::View rgbDevice, Reconstruction *reconstruction);
private:
	ColorVolume* colorVolume;
	
};

#endif