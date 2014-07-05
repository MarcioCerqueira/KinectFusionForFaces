#ifndef MESH_H
#define MESH_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/PolygonMesh.h>
#include "MarchingCubes.h"
#include "TsdfVolume.h"

using namespace Eigen;

class Mesh
{
public:
	Mesh();
	void extractFromGrid(MarchingCubes *marchingCubes, TsdfVolume *tsdfVolume_);
	void computePolygonMesh();
	void convertToCameraCoordinates(Vector3f& init_tcam);
	void convertToGlobalCoordinates(Vector3f& init_tcam);
	void applyTransformation(std::vector<Matrix3frm>& rmat, std::vector<Vector3f>& tvec, int globalTime);
	pcl::PolygonMesh getPolygonMesh() { return mesh_; }
private:

	DeviceArray<PointXYZ> trianglesDevice_;
	DeviceArray<PointXYZ> trianglesBufferDevice_;
	pcl::PolygonMesh mesh_;

};

#endif