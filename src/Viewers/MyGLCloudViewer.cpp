#include "Viewers/MyGLCloudViewer.h"

MyGLCloudViewer::MyGLCloudViewer()
{
	ARModel = NULL;
	ambientIntensity = 0;
	diffuseIntensity = 0.9;
	specularIntensity = 0.85;
}

void MyGLCloudViewer::configureAmbient(int threshold)
{

	glDisable(GL_LIGHTING);
	//drawAxis();
	glEnable(GL_LIGHTING);
	
	gluPerspective(41, 640.f/480.f, 0.01, threshold * 2);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(eyePos[0], eyePos[1], eyePos[2], 0, 0, 0, 0, -1, 1);
	
	// setup light
	float lightdir[3];
	lightdir[0] = 0;
	lightdir[1] = 0;
	lightdir[2] = 1;

	float lightPosition[3];
	lightPosition[0] = eyePos[0];
	lightPosition[1] = eyePos[1];
	lightPosition[2] = -eyePos[2];

	GLfloat light0_position[] = { lightPosition[0], lightPosition[1], lightPosition[2], 0 };
	GLfloat light1_position[] = { -lightPosition[0], -lightPosition[1], -lightPosition[2], 0 };
	glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
	glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
	
	configureLight();

	glDisable(GL_CULL_FACE);
}

void MyGLCloudViewer::configureLight() 
{
	// set light parameters
	GLfloat mat_specular[4] = { 0.18f, 0.18f, 0.18f, 1.f };
	GLfloat mat_shininess[] = { 64.f };
	GLfloat global_ambient[] = { 0.05f, 0.05f, 0.05f, 1.f };
	
	GLfloat light0_ambient[] = { ambientIntensity, ambientIntensity, ambientIntensity, 1.f };
	GLfloat light0_diffuse[] = { diffuseIntensity, diffuseIntensity, diffuseIntensity, 1.f };
	GLfloat light0_specular[] = { specularIntensity, specularIntensity, specularIntensity, 1.f };

	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, light0_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, light0_ambient);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, light0_specular);

	glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light0_specular);
	
	glLightfv(GL_LIGHT1, GL_AMBIENT, light0_ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light0_diffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, light0_specular);
	
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, global_ambient);
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_FALSE);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);

	glLightf(GL_LIGHT0, GL_SPOT_EXPONENT, 0.0);
	glLightf(GL_LIGHT1, GL_SPOT_EXPONENT, 0.0);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,GL_TRUE);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);// todo include this into spotlight node

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_NORMALIZE);
}

void MyGLCloudViewer::configureOBJAmbient(int threshold)
{
	
	gluPerspective(41.0, 640.f/480.f, 0.01, threshold * 2);
	gluLookAt(eyePos[0], eyePos[1], eyePos[2], 0, 0, 0, 0, -1, 1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	glEnable(GL_LIGHTING);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_FRONT);
	
}

void MyGLCloudViewer::configureQuadAmbient(int threshold)
{

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

    gluPerspective(41.0, 640.f/480.f, 0.01, threshold * 2);
    gluLookAt(eyePos[0], eyePos[1], eyePos[2], 0, 0, 0, 0, -1, 1);
    
	glMatrixMode(GL_TEXTURE);
	glLoadIdentity();

	glDisable(GL_LIGHTING);
	glDisable(GL_ALPHA_TEST);
	glDisable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);

	glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

}

void MyGLCloudViewer::configureARAmbientWithBlending(int threshold)
{

	glEnable(GL_DEPTH_TEST);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(41.0, 640.f/480.f, 0.01, threshold * 2);
	gluLookAt(eyePos[0], eyePos[1], eyePos[2], 0, 0, 0, 0, -1, 1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glEnable( GL_ALPHA_TEST );
    glAlphaFunc( GL_GREATER, 0.1f );

    glEnable(GL_BLEND);
	glBlendFunc( GL_ONE, GL_ONE_MINUS_SRC_ALPHA );

	glEnable(GL_LIGHTING);

}

void MyGLCloudViewer::computeARModelCentroid(float *centroid) 
{
	centroid[0] = 0;
	centroid[1] = 0;
	centroid[2] = 0;

	for(int i = 1; i <= ARModel->numvertices; i++)
		for(int axis = 0; axis < 3; axis++)
			centroid[axis] += ARModel->vertices[3 * i + axis];

	centroid[0] /= ARModel->numvertices;
	centroid[1] /= ARModel->numvertices;
	centroid[2] /= ARModel->numvertices;
}

void MyGLCloudViewer::drawAxis()
{
	
    glColor3f (0.5, 0.5, 0.5);
    glBegin (GL_LINES);
        glColor3f (1, 0.0, 0.0);
        glVertex3f (-300.0, 0.0, 0.0);
        glVertex3f (300.0, 0.0, 0.0);

        glColor3f (0.0, 1, 0.0);
        glVertex3f (0.0, 300.0, 0.0);
        glVertex3f (0.0, -300.0, 0.0);

        glColor3f (0.0, 0.0, 1);
        glVertex3f (0.0, 0.0, -300.0);
        glVertex3f (0.0, 0.0, 300.0);
    glEnd ();
}

void MyGLCloudViewer::drawMesh(GLuint* VBOs, Eigen::Vector3f gTrans, Matrix3frm gRot, Eigen::Vector3f initialTranslation, float *rotationAngles, bool useShader, 
	bool globalCoordinates)
{

	if (useShader)
		glUseProgram(shaderProg);

	glPushMatrix();	
	//OpenGL correction
	
	glRotatef(180, 0, 1, 0);
	GLfloat mat[16] = {gRot(0, 0), gRot(0, 1), gRot(0, 2), 0,
					   gRot(1, 0), gRot(1, 1), gRot(1, 2), 0,
					   gRot(2, 0), gRot(2, 1), gRot(2, 2), 0,
					   0, 0, 0, 1};
	
	if(globalCoordinates)
	{
		glMultMatrixf(mat);
		glTranslatef(-gTrans(0), -gTrans(1), -gTrans(2));
	}

	glColor3f(1, 1, 1);

	glBindBuffer(GL_ARRAY_BUFFER, VBOs[0]); 
	glEnableClientState(GL_VERTEX_ARRAY); 		
	glVertexPointer(3, GL_FLOAT, 0, 0); 		 
	
	glBindBuffer(GL_ARRAY_BUFFER, VBOs[1]); 		
	glEnableClientState(GL_NORMAL_ARRAY); 		
	glNormalPointer(GL_FLOAT, 0, 0); 		 
	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, VBOs[2]); 
	
	glDrawElements(GL_TRIANGLES, 640 * 480 * 6, GL_UNSIGNED_INT, 0);
	
	glDisableClientState(GL_VERTEX_ARRAY); 
	glDisableClientState(GL_NORMAL_ARRAY); 

	glBindBuffer(GL_ARRAY_BUFFER, 0); 
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0); 

	glPopMatrix();
	glDisable(GL_LIGHTING);

	if (useShader)
    	glUseProgram(0);
}

void MyGLCloudViewer::drawOBJ(ModelViewParams modelViewParams)
{

	glEnable(GL_LIGHTING);
	glFrontFace(GL_CW);


	updateModelViewMatrix(modelViewParams);
	//glMultMatrixf(mat);
	//glMultMatrixf(mat);
	
	glmDraw(ARModel, GLM_SMOOTH | GLM_MATERIAL);
	glPopMatrix();
	
	glDisable(GL_LIGHTING);
	
	glFrontFace(GL_CCW);
}

void MyGLCloudViewer::drawQuad(GLuint *VBO)
{

	glBindBuffer(GL_ARRAY_BUFFER, VBO[0]); 
	glEnableClientState(GL_VERTEX_ARRAY); 		
	glVertexPointer(3, GL_FLOAT, 0, 0); 

	glBindBuffer(GL_ARRAY_BUFFER, VBO[1]); 		
	glEnableClientState(GL_COLOR_ARRAY); 		
	glColorPointer(3, GL_FLOAT, 0, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, VBO[2]); 
	
	glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
	
	glDisableClientState(GL_VERTEX_ARRAY); 
	glDisableClientState(GL_COLOR_ARRAY); 

	glBindBuffer(GL_ARRAY_BUFFER, 0); 
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0); 

}

void MyGLCloudViewer::loadARModel(char *fileName)
{

	ARModel = glmReadOBJ(fileName);
    if (!ARModel) exit(0);
    glmUnitize(ARModel);
    glmFacetNormals(ARModel);
    glmVertexNormals(ARModel, 90.0);
    
}

void MyGLCloudViewer::loadIndices(int *indices, float *pointCloud)
{
	
	int x, y, pixel, pixelX1, pixelY1, pixelX1Y1;
	for(int point = 0; point < 640 * 480; point++)
	{

		indices[point * 6 + 0] = 0;
		indices[point * 6 + 1] = 0;
		indices[point * 6 + 2] = 0;
		indices[point * 6 + 3] = 0;
		indices[point * 6 + 4] = 0;
		indices[point * 6 + 5] = 0;

		x = point % 640;
		y = point / 640;

		if(x == 640 - 1 || y == 480 - 1)
			continue;

		pixel = y * 640 + x;
		pixelX1 = y * 640 + x + 1;
		pixelY1 = (y + 1) * 640 + x;
		pixelX1Y1 = (y + 1) * 640 + x + 1;

		if(pointCloud[pixel * 3 + 2] == 0 || pointCloud[pixelX1 * 3 + 2] == 0 || pointCloud[pixelY1 * 3 + 2] == 0 || 
			pointCloud[pixel * 3 + 2] != pointCloud[pixel * 3 + 2] || pointCloud[pixelX1 * 3 + 2] != pointCloud[pixelX1 * 3 + 2] || pointCloud[pixelY1 * 3 + 2]
			!= pointCloud[pixelY1 * 3 + 2])
				continue;

		indices[point * 6 + 0] = pixel;
		indices[point * 6 + 1] = pixelY1;
		indices[point * 6 + 2] = pixelX1;
		
		if(pointCloud[pixelX1 * 3 + 2] == 0 || pointCloud[pixelY1 * 3 + 2] == 0 || pointCloud[pixelX1Y1 * 3 + 2] == 0 ||
			pointCloud[pixelX1 * 3 + 2] != pointCloud[pixelX1 * 3 + 2] || pointCloud[pixelY1 * 3 + 2] != pointCloud[pixelY1 * 3 + 2] || 
			pointCloud[pixelX1Y1 * 3 + 2] != pointCloud[pixelX1Y1 * 3 + 2])
				continue;

		indices[point * 6 + 3] = pixelY1;
		indices[point * 6 + 4] = pixelX1Y1;
		indices[point * 6 + 5] = pixelX1;

	}
}
	
void MyGLCloudViewer::loadVBOs(GLuint *VBOs, int *indices, float *pointCloud, float *normalVector)
{

	glBindBuffer(GL_ARRAY_BUFFER, VBOs[0]);
	glEnableClientState(GL_VERTEX_ARRAY);
	glBufferData(	GL_ARRAY_BUFFER, 640 * 480 * 3 * sizeof(float), 
						pointCloud, GL_DYNAMIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, VBOs[1]);
	glEnableClientState(GL_NORMAL_ARRAY);
	glBufferData(	GL_ARRAY_BUFFER, 640 * 480 * 3 * sizeof(float), 
						normalVector, GL_DYNAMIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, VBOs[2]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 640 * 480 * 6 * sizeof(int), 
						indices, GL_DYNAMIC_DRAW);
		
	glDisableClientState(GL_VERTEX_ARRAY); 
	glDisableClientState(GL_NORMAL_ARRAY);
}

void MyGLCloudViewer::loadVBOQuad(GLuint *VBO, float x, float y, float z)
{

	float pointCloud[24];
	float color[24];
	int indices[36];

	pointCloud[0] = -x;	pointCloud[1] = y;	pointCloud[2] = z;
	pointCloud[3] = -x;	pointCloud[4] = -y;	pointCloud[5] = z;
	pointCloud[6] = x;	pointCloud[7] = -y;	pointCloud[8] = z;
	pointCloud[9] = x;	pointCloud[10] = y;	pointCloud[11] = z;

	pointCloud[12] = -x;	pointCloud[13] = y;	pointCloud[14] = -z;
	pointCloud[15] = -x;	pointCloud[16] = -y;	pointCloud[17] = -z;
	pointCloud[18] = x;	pointCloud[19] = -y;	pointCloud[20] = -z;
	pointCloud[21] = x;	pointCloud[22] = y;	pointCloud[23] = -z;

	color[0] = 0;	color[1] = 1;	color[2] = 1;
	color[3] = 0;	color[4] = 0;	color[5] = 1;
	color[6] = 1;	color[7] = 0;	color[8] = 1;
	color[9] = 1;	color[10] = 1;	color[11] = 1;

	color[12] = 0;	color[13] = 1;	color[14] = 0;
	color[15] = 0;	color[16] = 0;	color[17] = 0;
	color[18] = 1;	color[19] = 0;	color[20] = 0;
	color[21] = 1;	color[22] = 1;	color[23] = 0;
	
	indices[0] = 0;	indices[1] = 1;	indices[2] = 2;
	indices[3] = 0;	indices[4] = 2;	indices[5] = 3;
	indices[6] = 4;	indices[7] = 5;	indices[8] = 1;
	indices[9] = 4;	indices[10] = 1;	indices[11] = 0;
	indices[12] = 7;	indices[13] = 6;	indices[14] = 5;
	indices[15] = 7;	indices[16] = 5;	indices[17] = 4;
	indices[18] = 3;	indices[19] = 2;	indices[20] = 6;
	indices[21] = 3;	indices[22] = 6;	indices[23] = 7;
	indices[24] = 4;	indices[25] = 0;	indices[26] = 3;
	indices[27] = 4;	indices[28] = 3;	indices[29] = 7;
	indices[30] = 6;	indices[31] = 2;	indices[32] = 1;
	indices[33] = 6;	indices[34] = 1;	indices[35] = 5;
	
	glBindBuffer(GL_ARRAY_BUFFER, VBO[0]);
	glEnableClientState(GL_VERTEX_ARRAY);
	glBufferData(GL_ARRAY_BUFFER, 24 * sizeof(float), pointCloud, GL_DYNAMIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, VBO[1]);
	glEnableClientState(GL_COLOR_ARRAY);
	glBufferData(GL_ARRAY_BUFFER, 24 * sizeof(float), color, GL_DYNAMIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, VBO[2]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 36 * sizeof(unsigned int), indices, GL_DYNAMIC_DRAW);

	glDisableClientState(GL_VERTEX_ARRAY); 
	glDisableClientState(GL_COLOR_ARRAY); 

}

void MyGLCloudViewer::setEyePosition(int xEye, int yEye, int zEye) 
{
	eyePos[0] = xEye;
	eyePos[1] = yEye;
	eyePos[2] = zEye;
}

void MyGLCloudViewer::setOBJScale(float* scale)
{
	glmScale(ARModel, scale);
}

void MyGLCloudViewer::setProgram(GLuint shaderProg)
{
	this->shaderProg = shaderProg;
}

void MyGLCloudViewer::updateModelViewMatrix(ModelViewParams modelViewParams) 
{

	glPushMatrix();
	
	glRotatef(180, 0, 1, 0);

	Matrix3frm gRot = modelViewParams.gRot;
	
	GLfloat mat2[16] = {gRot(0, 0), gRot(0, 1), gRot(0, 2), 0,
					   gRot(1, 0), gRot(1, 1), gRot(1, 2), 0,
					   gRot(2, 0), gRot(2, 1), gRot(2, 2), 0,
					   0, 0, 0, 1};
	
	glMultMatrixf(mat2);
	glTranslatef(-modelViewParams.gTrans(0), -modelViewParams.gTrans(1), -modelViewParams.gTrans(2));
	glTranslatef(modelViewParams.initialTranslation(0), modelViewParams.initialTranslation(1), modelViewParams.initialTranslation(2));
	glTranslatef(modelViewParams.translationVector[0], modelViewParams.translationVector[1], modelViewParams.translationVector[2]);
	
	if(!modelViewParams.useTextureRotation) {

		glRotatef(modelViewParams.rotationAngles[0], 1, 0, 0);
		glRotatef(modelViewParams.rotationAngles[1], 0, 1, 0);
		glRotatef(modelViewParams.rotationAngles[2], 0, 0, 1);

	} else {

		int rotationX = modelViewParams.rotationIndices[0];
		int rotationY = modelViewParams.rotationIndices[1];
		int rotationZ = modelViewParams.rotationIndices[2];
		glRotatef(modelViewParams.rotationAngles[rotationX], 1, 0, 0);
		glRotatef(modelViewParams.rotationAngles[rotationY], 0, 1, 0);
		glRotatef(modelViewParams.rotationAngles[rotationZ], 0, 0, 1);

	}

	//My solution to the orientation problem	
	glRotatef(180, 1, 0, 0);

	if(modelViewParams.useHeadPoseRotation) {
		int rotationX = modelViewParams.rotationIndices[0];
		int rotationY = modelViewParams.rotationIndices[1];
		int rotationZ = modelViewParams.rotationIndices[2];
		glTranslatef(-modelViewParams.headCenterRotated(0), -modelViewParams.headCenterRotated(1), -modelViewParams.headCenterRotated(2));
		glRotatef(-modelViewParams.headEulerAngles(rotationX), 1, 0, 0);
		glRotatef(modelViewParams.headEulerAngles(rotationY), 0, 1, 0);
		glRotatef(modelViewParams.headEulerAngles(rotationZ), 0, 0, 1);
		glTranslatef(modelViewParams.headCenter(0), modelViewParams.headCenter(1), modelViewParams.headCenter(2));
	}
	
}