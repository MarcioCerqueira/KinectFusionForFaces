#ifndef MESHGENERATIONMEDIATOR_H
#define MESHGENERATIONMEDIATOR_H

#include <pcl/io/ply_io.h>
#include "MarchingCubes.h"
#include "Mesh.h"
#include "TsdfVolume.h"

class MeshGenerationMediator
{
public:
	MeshGenerationMediator();
	~MeshGenerationMediator();
	void saveMesh(TsdfVolume *tsdfVolume);
private:
	MarchingCubes *marchingCubes;
};

#endif