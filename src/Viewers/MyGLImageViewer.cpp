#include "Viewers/MyGLImageViewer.h"

MyGLImageViewer::MyGLImageViewer()
{
	depthData = (unsigned char*)malloc(640 * 480 * 3 * sizeof(unsigned char));
	frameBuffer = (unsigned char*)malloc(640 * 480 * 3 * sizeof(unsigned char));
}

MyGLImageViewer::~MyGLImageViewer()
{
	delete [] depthData;
	delete [] frameBuffer;
}

void MyGLImageViewer::loadDepthTexture(unsigned short *data, GLuint *texVBO, int index, int threshold, int imageWidth, int imageHeight)
{
	
	//Normalize to (0..255)
	for(int p = 0; p < (640 * 480); p++) {
		depthData[p * 3 + 0] = data[p] / 20;
		depthData[p * 3 + 1] = depthData[p * 3 + 0];
		depthData[p * 3 + 2] = depthData[p * 3 + 0];
	}

	glBindTexture(GL_TEXTURE_2D, texVBO[index]);
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imageWidth, imageHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, depthData);
}
	
void MyGLImageViewer::loadDepthComponentTexture(float *data, GLuint *texVBO, int index, int windowWidth, int windowHeight)
{

	glBindTexture(GL_TEXTURE_2D, texVBO[index]);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, windowWidth/2, windowHeight/2, 0, GL_DEPTH_COMPONENT, GL_FLOAT, data);

}

void MyGLImageViewer::loadRGBTexture(const unsigned char *data, GLuint *texVBO, int index, int imageWidth, int imageHeight)
{

	glBindTexture(GL_TEXTURE_2D, texVBO[index]);
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imageWidth, imageHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, data);

}

void MyGLImageViewer::loadRGBTextureWithMipMaps(const unsigned char *data, GLuint *texVBO, int index, int imageWidth, int imageHeight)
{

	glBindTexture(GL_TEXTURE_2D, texVBO[index]);
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, imageWidth, imageHeight, GL_RGB, GL_UNSIGNED_BYTE, data);

}

void MyGLImageViewer::loadRGBATexture(unsigned char *data, GLuint *texVBO, int index, int width, int height) {
	
	glBindTexture(GL_TEXTURE_2D, texVBO[index]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);

}

void MyGLImageViewer::loadFrameBufferTexture(GLuint *texVBO, int index, int x, int y, int width, int height)
{

	glReadPixels(x, y, width, height, GL_RGB, GL_UNSIGNED_BYTE, frameBuffer);
	
	glBindTexture(GL_TEXTURE_2D, texVBO[index]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, frameBuffer);

}

void MyGLImageViewer::loadFrameBufferTexture(int x, int y, int width, int height)
{
	glReadPixels(x, y, width, height, GL_RGB, GL_UNSIGNED_BYTE, frameBuffer);	
}

void MyGLImageViewer::loadARTexture(const unsigned char *rgbMap, unsigned char *data, GLuint *texVBO, int index, int windowWidth, 
	int windowHeight) {
	
	for(int pixel = 0; pixel < (640 * 480); pixel++) {
		if(data[pixel * 3 + 0] == 0)
			for(int color = 0; color < 3; color++)
				data[pixel * 3 + color] = rgbMap[pixel * 3 + color];
		else 
			for(int color = 0; color < 3; color++)
				data[pixel * 3 + color] = data[pixel * 3 + color] * 0.5 + rgbMap[pixel * 3 + color] * 0.5;
	}

	glBindTexture(GL_TEXTURE_2D, texVBO[index]);
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, windowWidth/2, windowHeight/2, 0, GL_RGB, GL_UNSIGNED_BYTE, data);

}

void MyGLImageViewer::load2DNoiseTexture(GLuint *texVBO, int index, int width, int height) {
	
	unsigned char *noise = (unsigned char*)malloc(width * height * sizeof(unsigned char));

	srand((unsigned)time(NULL));
	for(int pixel = 0; pixel < (width * height); pixel++)
		noise[pixel] = 255.f * rand()/(float)RAND_MAX;

	glBindTexture(GL_TEXTURE_2D, texVBO[index]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE8, width, height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, noise);

	delete [] noise;

}

void MyGLImageViewer::load3DTexture(unsigned char *data, GLuint *texVBO, int index, int volumeWidth, int volumeHeight, int volumeDepth)
{
	
	glBindTexture(GL_TEXTURE_3D, texVBO[index]);
	//glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA, volumeWidth, volumeHeight, volumeDepth, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
	glBindTexture(GL_TEXTURE_3D, 0);
}

void MyGLImageViewer::drawDepthTexture(GLuint *texVBO, int index, int windowWidth, int windowHeight)
{
	gluOrtho2D( 0, windowWidth, windowHeight/2, 0 ); 
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texVBO[index]);

	glBegin(GL_QUADS);
		glTexCoord2f(0.0f, 0.0f); 
		glVertex2f(0.0f, 0.0f);
		glTexCoord2f(1.0f, 0.0f); 
		glVertex2f(windowWidth, 0.0f);
		glTexCoord2f(1.0f, 1.0f); 
		glVertex2f(windowWidth, windowHeight);
		glTexCoord2f(0.0f, 1.0f); 
		glVertex2f(0.0f, windowHeight);
	glEnd();

	glDisable(GL_TEXTURE_2D);
}

void MyGLImageViewer::drawRGBTexture(GLuint *texVBO, int index, int windowWidth, int windowHeight)
{

	//if(index == 2)
	//	glUseProgram(shaderProg);
	
	gluOrtho2D( 0, windowWidth/2, windowHeight/2, 0 ); 
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();

	glMatrixMode(GL_TEXTURE);
	glLoadIdentity();

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texVBO[index]);
	
	glBegin(GL_QUADS);
		glTexCoord2f(0.0f, 0.0f); 
		glVertex2f(0.0f, 0.0f);
		glTexCoord2f(1.0f, 0.0f); 
		glVertex2f(windowWidth/2, 0.0f);
		glTexCoord2f(1.0f, 1.0f); 
		glVertex2f(windowWidth/2, windowHeight/2);
		glTexCoord2f(0.0f, 1.0f); 
		glVertex2f(0.0f, windowHeight/2);
	glEnd();

	glDisable(GL_TEXTURE_2D);
	
	//if(index == 2)
	//	glUseProgram(0);
}

void MyGLImageViewer::drawRGBTextureOnShader(GLuint *texVBO, int index, int windowWidth, int windowHeight)
{

	glUseProgram(shaderProg);
	
	gluOrtho2D( 0, windowWidth/2, windowHeight/2, 0 ); 
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();

	GLuint texLoc = glGetUniformLocation(shaderProg, "image");
	glUniform1i(texLoc, 0);
	
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, texVBO[index]);

	glBegin(GL_QUADS);
		glTexCoord2f(0.0f, 0.0f); 
		glVertex2f(0.0f, 0.0f);
		glTexCoord2f(1.0f, 0.0f); 
		glVertex2f(windowWidth/2, 0.0f);
		glTexCoord2f(1.0f, 1.0f); 
		glVertex2f(windowWidth/2, windowHeight/2);
		glTexCoord2f(0.0f, 1.0f); 
		glVertex2f(0.0f, windowHeight/2);
	glEnd();

	glActiveTexture(GL_TEXTURE0);
	glDisable(GL_TEXTURE_2D);
	
	glUseProgram(0);

}

void MyGLImageViewer::setProgram(GLuint shaderProg) 
{
	this->shaderProg = shaderProg;
}
