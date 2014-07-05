#ifndef MYGLCLOUDVIEWER_H
#define MYGLCLOUDVIEWER_H

#include <GL/glew.h>
#include <stdlib.h>
#include <iostream>
#include <GL/glut.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "Viewers/glm.h"
#include "Viewers/ModelViewParams.h"
#include "MyPointCloud.h"

class MyGLCloudViewer
{
public:
	MyGLCloudViewer();
	void configureAmbient(int threshold);
	void configureQuadAmbient(int threshold);
	void configureLight();
	void configureOBJAmbient(int threshold);
	void configureARAmbientWithBlending(int threshold);
	void computeARModelCentroid(float *centroid);

	void drawAxis();
	void drawMesh(GLuint* VBOs, Eigen::Vector3f gTrans, Matrix3frm gRot, Eigen::Vector3f initialTranslation, float *rotationAngles, bool useShader, bool globalCoordinates);
	void drawOBJ(ModelViewParams modelViewParams);
	void drawQuad(GLuint *VBO);
	
	void loadARModel(char *fileName);
	void loadIndices(int *indices, float *pointCloud);
	void loadVBOs(GLuint *meshVBO, int *indices, float *pointCloud, float *normalVector);
	void loadVBOQuad(GLuint *VBO, float x, float y, float z);
	void setEyePosition(int xEye, int yEye, int zEye);
	float* getEyePosition() { return eyePos; }
	void setOBJScale(float* scale);
	void setProgram(GLuint shaderProg);
	void setAmbientIntensity(float ambientIntensity) { this->ambientIntensity = ambientIntensity; }
	void setDiffuseIntensity(float diffuseIntensity) { this->diffuseIntensity = diffuseIntensity; }
	void setSpecularIntensity(float specularIntensity) { this->specularIntensity = specularIntensity; }
	//void configure3DTexture(
	void updateModelViewMatrix(ModelViewParams modelViewParams);
private:
	GLMmodel* ARModel;
	float eyePos[3];
	float eyePosSpecial;
	GLuint shaderProg;
	float diffuseIntensity;
	float ambientIntensity;
	float specularIntensity;
};
#endif
