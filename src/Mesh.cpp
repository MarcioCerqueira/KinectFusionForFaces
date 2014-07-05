#include "Mesh.h"

Mesh::Mesh() {
}

void Mesh::extractFromGrid(MarchingCubes *marchingCubes, TsdfVolume *tsdfVolume) {

	trianglesDevice_ = marchingCubes->run(tsdfVolume, trianglesBufferDevice_);

}

void Mesh::computePolygonMesh() {
	
    if (!trianglesDevice_.empty()) {

		pcl::PointCloud<pcl::PointXYZ> cloud;
		cloud.width  = (int)trianglesDevice_.size();
		cloud.height = 1;
		trianglesDevice_.download(cloud.points);

		pcl::toROSMsg(cloud, mesh_.cloud);  
      
		mesh_.polygons.resize (trianglesDevice_.size() / 3);
		for (size_t i = 0; i < mesh_.polygons.size (); ++i)
		{
			pcl::Vertices v;
			v.vertices.push_back(i*3+0);
			v.vertices.push_back(i*3+2);
			v.vertices.push_back(i*3+1);              
			mesh_.polygons[i] = v;
		}    
	
	}

}

void Mesh::convertToCameraCoordinates(Vector3f& init_tcam) {
	
	pcl::PointCloud<pcl::PointXYZ> pointCloud;
	pcl::fromROSMsg(mesh_.cloud, pointCloud);
	
	for(int point = 0; point < pointCloud.points.size(); point++) {

		pointCloud.points[point].x -= init_tcam(0);
		pointCloud.points[point].y -= init_tcam(1);
		pointCloud.points[point].z -= init_tcam(2);

	}

	pcl::toROSMsg(pointCloud, mesh_.cloud);

}

void Mesh::convertToGlobalCoordinates(Vector3f& init_tcam) {
	
	pcl::PointCloud<pcl::PointXYZ> pointCloud;
	pcl::fromROSMsg(mesh_.cloud, pointCloud);
	
	for(int point = 0; point < pointCloud.points.size(); point++) {

		pointCloud.points[point].x += init_tcam(0);
		pointCloud.points[point].y += init_tcam(1);
		pointCloud.points[point].z += init_tcam(2);

	}

	pcl::toROSMsg(pointCloud, mesh_.cloud);

}

void Mesh::applyTransformation(std::vector<Matrix3frm>& rmat, std::vector<Vector3f>& tvec, int globalTime) {
	
	pcl::PointCloud<pcl::PointXYZ> pointCloud;
	pcl::fromROSMsg(mesh_.cloud, pointCloud);

	Matrix3frm rmatPrev = rmat[globalTime - 1];
	Vector3f tvecPrev = tvec[globalTime - 1];

	Matrix3frm rmatCurr = rmat[globalTime];
	Vector3f tvecCurr = tvec[globalTime];

	Matrix3frm rmatPrevInverse = rmatPrev.inverse();
	Matrix3frm rmatCurrInverse = rmatCurr.inverse();
	float x, y, z;
	for(int point = 0; point < pointCloud.points.size(); point++) 
	{
		
		x = pointCloud.points[point].x * rmatCurr(0, 0) + pointCloud.points[point].y * rmatCurr(0, 1) + 
			pointCloud.points[point].z * rmatCurr(0, 2) + tvecCurr(0);
		y = pointCloud.points[point].x * rmatCurr(1, 0) + pointCloud.points[point].y * rmatCurr(1, 1) + 
			pointCloud.points[point].z * rmatCurr(1, 2) + tvecCurr(1);
		z = pointCloud.points[point].x * rmatCurr(2, 0) + pointCloud.points[point].y * rmatCurr(2, 1) + 
			pointCloud.points[point].z * rmatCurr(2, 2) + tvecCurr(2);
		
		x -= tvecPrev(0);
		y -= tvecPrev(1);
		z -= tvecPrev(2);
		
		pointCloud.points[point].x = x * rmatPrevInverse(0, 0) + y * rmatPrevInverse(0, 1) + 
			z * rmatPrevInverse(0, 2);
		pointCloud.points[point].y = x * rmatPrevInverse(1, 0) + y * rmatPrevInverse(1, 1) + 
			z * rmatPrevInverse(1, 2);
		pointCloud.points[point].z = x * rmatPrevInverse(2, 0) + y * rmatPrevInverse(2, 1) + 
			z * rmatPrevInverse(2, 2);
		
	}
	
	pcl::toROSMsg(pointCloud, mesh_.cloud);
}