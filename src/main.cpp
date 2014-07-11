
#include <iostream>
#include <pcl/console/parse.h>
#include <GL/glew.h>
#include <GL/glut.h>
#include "pcl/gpu/containers/initialization.hpp"
#include "Reconstruction.h"
#include "Viewers/MyGLImageViewer.h"
#include "Viewers/MyGLCloudViewer.h"
#include "Viewers/shader.h"
#include "Viewers/ModelViewParams.h"
#include "Mediators/MeshGenerationMediator.h"
#include "Mediators/ColoredReconstructionMediator.h"
#include "Mediators/HeadPoseEstimationMediator.h"
#include "Capture/AbstractCapture.h"
#include "Capture/Kinect.h"
#include "Capture/ONIGrabber.h"
#include "Image.h"
#include "FaceDetection.h"

using namespace std;
using namespace pcl;
using namespace pcl::gpu;
using namespace Eigen;

//Window's size
int windowWidth = 1280;
int windowHeight = 960;

//Our Objects
Image *imageCollection;
AbstractCapture *capture;
Reconstruction *reconstruction;
MyGLImageViewer *myGLImageViewer;
MyGLCloudViewer *myGLCloudViewer;
ColoredReconstructionMediator *coloredReconstructionMediator;
HeadPoseEstimationMediator *headPoseEstimationMediator;
FaceDetection *faceDetector;
ModelViewParams modelViewParams;

GLuint texVBO[25]; 
GLuint spt[2];
GLuint meshVBO[4];

enum
{
	REAL_DEPTH_FROM_DEPTHMAP_BO = 0,
	REAL_RGB_BO = 1,
	RAYCAST_BO = 2,
	AR_FROM_VOLUME_KINECTFUSION_BO = 3,
	VIRTUAL_DEPTH_BO = 4,
	VIRTUAL_RGB_BO = 5,
	REAL_DEPTH_FROM_DEPTHBUFFER_BO = 6,
	AR_FROM_VOLUME_RENDERING_BO = 7,
	REAL_RGB_FROM_FBO = 8, 
	MIN_MAX_OCTREE_BO = 9,
	TRANSFER_FUNCTION_BO = 10,
	NOISE_BO = 11,
	FRONT_QUAD_RGB_FBO = 12,
	FRONT_QUAD_DEPTH_FBO = 13,
	BACK_QUAD_RGB_FBO = 14,
	BACK_QUAD_DEPTH_FBO = 15,
	CURVATURE_MAP_FBO = 16, 
	CONTOURS_FBO = 17,
	BACKGROUND_SCENE_FBO = 18,
	SUBTRACTION_MASK_FBO = 19, 
	SUBTRACTION_MASK_DEPTH_FBO = 20
};

int indices[640 * 480 * 6];
float pointCloud[640 * 480 * 3];
float normalVector[640 * 480 * 3];
float depthData[640 * 480];

//AR (General attributes)
int vel = 4;
float scale[3];
float translationVector[3];
float rotationAngles[3];
bool translationOn = false;
bool rotationOn = false;
bool scaleOn = false;

bool integrateColors = false;
bool isHeadPoseEstimationEnabled = false;
bool hasFaceDetection = false;
bool faceDetected = false;
bool shader=true;

bool showCloud = false;
bool showRaycasting = true;
bool showDepthMap = true;
bool showRGBMap = true;
bool showCurvatureMap = false;
bool showContoursMap = false;

//
// Global handles for the currently active program object, with its two shader objects
//
GLuint ProgramObject = 0;
GLuint VertexShaderObject = 0;
GLuint FragmentShaderObject = 0;

GLuint shaderVS, shaderFS, shaderProg[10];   // handles to objects
GLint  linked;

int w1 = 1, w2 = 0, w3 = 120; 
int workAround = 0;

//  The number of frames
int frameCount = 0;
float fps = 0;
int currentTime = 0, previousTime = 0;
std::vector<unsigned short> sourceDepthData;
unsigned char *clippedImage;

IplImage *image;
IplImage *grayImage;

void calculateFPS() {

	frameCount++;
	currentTime = glutGet(GLUT_ELAPSED_TIME);

    int timeInterval = currentTime - previousTime;

    if(timeInterval > 1000) {
        fps = frameCount / (timeInterval / 1000.0f);
        previousTime = currentTime;
        frameCount = 0;
		std::cout << "FPS: " << fps << std::endl;
    }

}

void printHelp() {

	std::cout << "Help " << std::endl;
	std::cout << "--cloud: Show Cloud " << std::endl;
	std::cout << "--mesh: Show Mesh extracted from MC " << std::endl;
	std::cout << "--color: Enable color integration " << std::endl;
	std::cout << "--hpefile config.txt: Head Pose Estimation Config File " << std::endl; 
	std::cout << "--threshold value: Depth Threshold for depth map truncation " << std::endl;
	std::cout << "--face cascades.txt: Enable face detection " << std::endl;
	
	std::cout << "On the fly.. " << std::endl;
	std::cout << "Press 'h' to enable head pose tracking " << std::endl;
	std::cout << "Press 'p' to stop the head pose tracking " << std::endl;
	std::cout << "Press 'c' to continue the head pose tracking " << std::endl;
	std::cout << "Press 'a' to enable AR application " << std::endl;

}

void saveModel()
{
	
	int op;
	std::cout << "Saving Model..." << std::endl;
	std::cout << "Do you want to save a point cloud (.pcd) or a mesh (.ply)? (0: point cloud; 1: mesh)" << std::endl;
	std::cin >> op;
	if(op == 0) {
		if(integrateColors)
			coloredReconstructionMediator->savePointCloud(reconstruction->getTsdfVolume());
		else
			reconstruction->savePointCloud();
	} else {
		MeshGenerationMediator mgm;
		mgm.saveMesh(reconstruction->getTsdfVolume());
	}
	
}

void loadArguments(int argc, char **argv, Reconstruction *reconstruction)
{
	//Default arguments
	char fileName[100];
	char hpeConfigFileName[100];
	char cascadeFileName[100];
	char aux[5];
	int begin = 0;
	int end = 0;
	int threshold = 5000;
	
	//AR Configuration
	translationVector[0] = 0;
	translationVector[1] = 0;
	translationVector[2] = 0;
	rotationAngles[0] = 0;
	rotationAngles[1] = 0;
	rotationAngles[2] = 0;
	scale[0] = 1;
	scale[1] = 1;
	scale[2] = 1;

	if(pcl::console::find_argument(argc, argv, "--cloud") >= 0) {
	showCloud = true;
	}
	if(pcl::console::find_argument(argc, argv, "--color") >= 0) {
	integrateColors = true;
	coloredReconstructionMediator = new ColoredReconstructionMediator(reconstruction->getVolumeSize());
	}
	if(pcl::console::find_argument(argc, argv, "-h") >= 0) {
	printHelp();
	}
	if(pcl::console::parse(argc, argv, "--threshold", aux) >= 0) {
	threshold = atoi(aux);
	}
	if(pcl::console::parse(argc, argv, "--hpefile", hpeConfigFileName) >= 0) {
	isHeadPoseEstimationEnabled = true;
	headPoseEstimationMediator = new HeadPoseEstimationMediator(hpeConfigFileName);
	reconstruction->setHeadPoseEstimationMediatorPointer((void*)headPoseEstimationMediator);
	}
	if(pcl::console::parse(argc, argv, "--face", cascadeFileName) >= 0) {
	hasFaceDetection = true;
	faceDetector = new FaceDetection(cascadeFileName);
	}

	//Initialize reconstruction with arguments
	reconstruction->setThreshold(threshold);

}

void loadARDepthDataBasedOnDepthMaps() 
{

	pcl::console::TicToc clock2;
	glPixelTransferf(GL_DEPTH_SCALE, 1.0/reconstruction->getThreshold());
	for(int p = 0; p < 640 * 480; p++)
		depthData[p] = (float)imageCollection->getDepthMap().data[p];///(float)reconstruction->getThreshold();
	myGLImageViewer->loadDepthComponentTexture(depthData, texVBO, REAL_DEPTH_FROM_DEPTHBUFFER_BO, windowWidth, windowHeight);

	reconstruction->getGlobalPreviousPointCloud()->getHostPointCloud(pointCloud);
	Matrix3frm rotInverse = reconstruction->getCurrentRotation().inverse();
	for(int point = 0; point < (640 * 480); point++) {
		if(pointCloud[point * 3 + 2] > 0 && pointCloud[point * 3 + 2] == pointCloud[point * 3 + 2]) {

			pointCloud[point * 3 + 0] -= reconstruction->getCurrentTranslation()[0];
            pointCloud[point * 3 + 1] -= reconstruction->getCurrentTranslation()[1];
            pointCloud[point * 3 + 2] -= reconstruction->getCurrentTranslation()[2];
			pointCloud[point * 3 + 2] = rotInverse(2, 0) * pointCloud[point * 3 + 0] + rotInverse(2, 1) * pointCloud[point * 3 + 1] +
                rotInverse(2, 2) * pointCloud[point * 3 + 2];

        }
    }
	
	for(int p = 0; p < 640 * 480; p++)
		depthData[p] = pointCloud[p * 3 + 2];///(float)reconstruction->getThreshold();
	
	myGLImageViewer->loadDepthComponentTexture(depthData, texVBO, VIRTUAL_DEPTH_BO, windowWidth, windowHeight);
	glPixelTransferf(GL_DEPTH_SCALE, 1);
	
}

void reshape(int w, int h)
{
	windowWidth = w;
	windowHeight = h;

	glViewport( 0, 0, windowWidth, windowHeight );
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluOrtho2D( 0, windowWidth, 0, windowHeight );
	glMatrixMode( GL_MODELVIEW );

}

void displayContoursData()
{

	glViewport(windowWidth/2, windowHeight/2, windowWidth/2, windowHeight/2);
	glMatrixMode(GL_PROJECTION);          
	glLoadIdentity();    
	
	myGLImageViewer->setProgram(shaderProg[7]);
	myGLImageViewer->drawRGBTextureOnShader(texVBO, SUBTRACTION_MASK_FBO, windowWidth, windowHeight);

}

void displayDepthData()
{
	
	glViewport(0, windowHeight/2, windowWidth/2, windowHeight/2);
	glMatrixMode(GL_PROJECTION);          
	glLoadIdentity();    
	
	myGLImageViewer->loadDepthTexture((unsigned short*)imageCollection->getDepthMap().data, texVBO, REAL_DEPTH_FROM_DEPTHMAP_BO, 
		reconstruction->getThreshold(), capture->getImageWidth(), capture->getImageHeight());
	myGLImageViewer->drawRGBTexture(texVBO, REAL_DEPTH_FROM_DEPTHMAP_BO, windowWidth, windowHeight);

}

void displayRGBData()
{
	glViewport(windowWidth/2, windowHeight/2, windowWidth/2, windowHeight/2);
	glMatrixMode(GL_PROJECTION);          
	glLoadIdentity(); 

	myGLImageViewer->loadRGBTexture((const unsigned char*)imageCollection->getRGBMap().data, texVBO, REAL_RGB_BO, capture->getImageWidth(), 
		capture->getImageHeight());
	myGLImageViewer->drawRGBTexture(texVBO, REAL_RGB_BO, windowWidth, windowHeight);

}

void displayRaycastedData()
{

	glViewport(windowWidth/2, 0, windowWidth/2, windowHeight/2);
	glMatrixMode(GL_PROJECTION);          
	glLoadIdentity();    

	myGLImageViewer->loadRGBTexture(imageCollection->getRaycastImage(reconstruction->getVolumeSize(), 
		reconstruction->getGlobalPreviousPointCloud()), texVBO, RAYCAST_BO, capture->getImageWidth(), capture->getImageHeight());
	myGLImageViewer->drawRGBTexture(texVBO, RAYCAST_BO, windowWidth, windowHeight);

}

void displayCloud(bool globalCoordinates = true)
{
	
	if(globalCoordinates) {
		reconstruction->getGlobalPreviousPointCloud()->getHostPointCloud(pointCloud);
		reconstruction->getGlobalPreviousPointCloud()->getHostNormalVector(normalVector, 1);
	} else {
		reconstruction->getCurrentPointCloud()->getHostPointCloud(pointCloud);
		reconstruction->getCurrentPointCloud()->getHostNormalVector(normalVector, -1);
	}
	
	myGLCloudViewer->loadIndices(indices, pointCloud);
	myGLCloudViewer->loadVBOs(meshVBO, indices, pointCloud, normalVector);
	
	//glViewport(0, 0, windowWidth/2, windowHeight/2);
	glViewport(windowWidth/2, 0, windowWidth/2, windowHeight/2);
	
	glMatrixMode(GL_PROJECTION);          
	glLoadIdentity(); 
	
	myGLCloudViewer->configureAmbient(reconstruction->getThreshold());
	if(reconstruction->getGlobalTime() > 1)
		myGLCloudViewer->drawMesh(meshVBO, reconstruction->getCurrentTranslation(), reconstruction->getCurrentRotation(), reconstruction->getInitialTranslation(), 
			rotationAngles, shader, globalCoordinates);
	if(workAround == 1) {
		myGLCloudViewer->drawMesh(meshVBO, Eigen::Vector3f::Zero(), Eigen::Matrix3f::Identity(), reconstruction->getInitialTranslation(), rotationAngles, shader, 
			globalCoordinates);
		workAround = 2;
	}
}

void display()
{
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	if(workAround == 1)
		displayCloud();
	if(showDepthMap)
		displayDepthData();
	if(showRGBMap)
		displayRGBData();
	if(showRaycasting && reconstruction->hasImage())
		displayRaycastedData();
	if(showCloud && reconstruction->hasImage())
		displayCloud();
	
	
	glutSwapBuffers();
	glutPostRedisplay();

}

void idle()
{
	
	bool preGlobalTimeGreaterThanZero;
	if(capture->grabFrame()) {

		imageCollection->load(capture->getRGBImage(), capture->getDepthImage());

		if(hasFaceDetection) {
			if(faceDetected) {
				reconstruction->stopTracking(false);
				faceDetector->segmentFace(imageCollection);
			} else {
				reconstruction->reset();
				reconstruction->stopTracking(true);
				faceDetected = faceDetector->run(imageCollection);
				if(faceDetected)
					faceDetector->segmentFace(imageCollection);
			}
			//to check if the reconstruction was reseted
			if(reconstruction->getGlobalTime() > 0) preGlobalTimeGreaterThanZero = true;
			else preGlobalTimeGreaterThanZero = false;
		}

		imageCollection->applyBilateralFilter();
		imageCollection->applyDepthTruncation(reconstruction->getThreshold());
		imageCollection->applyPyrDown();
		imageCollection->convertToPointCloud(reconstruction->getCurrentPointCloud());
		imageCollection->applyDepthTruncation(imageCollection->getDepthDevice(), reconstruction->getThreshold());
		pcl::device::sync ();
		reconstruction->run(imageCollection); 
		
		//if the reconstruction was reseted, the face is no more detected
		if(hasFaceDetection)
			if(reconstruction->getGlobalTime() == 0 && preGlobalTimeGreaterThanZero)
				faceDetected = false;
		if(integrateColors)
			coloredReconstructionMediator->updateColorVolume(imageCollection->getRgbDevice(), reconstruction);
		if(workAround != 2)
			workAround = 1;

	}
	calculateFPS();

}

void keyboard(unsigned char key, int x, int y)
{
	switch(key) {
	case 27:
		exit(0);
		break;
	case (int)'i' : case (int)'I':
		std::cout << "Head Pose Tracking Activated..." << std::endl;
		reconstruction->enableOnlyTracking();
		hasFaceDetection = false;
		break;
	case (int)'p' : case (int)'P':
		std::cout << "Pause..." << std::endl;
		reconstruction->stopTracking(true);
		if(isHeadPoseEstimationEnabled)
			headPoseEstimationMediator->stopTracking(true, (unsigned short*)imageCollection->getDepthMap().data, reconstruction);
		break;
	case (int)'c' : case (int)'C':
		std::cout << "Continue..." << std::endl;
		reconstruction->stopTracking(false);
		if(isHeadPoseEstimationEnabled)
			headPoseEstimationMediator->stopTracking(false, (unsigned short*)imageCollection->getDepthMap().data, reconstruction);
		break;
	case (int)'r' : case (int)'R':
		reconstruction->reset();
		faceDetected = false;
		break;
	case (int)'u':
		shader = !shader;
		break;
	case (int)'h':
		std::cout << rotationAngles[0] << " " << rotationAngles[1] << " " << rotationAngles[2] << std::endl;
		break;
	default:
		break;
	}
	glutPostRedisplay();
}

void specialKeyboard(int key, int x, int y)
{

	switch (key)
	{
	case GLUT_KEY_UP:
		if(translationOn)
			translationVector[1] += vel;
		if(rotationOn)
			rotationAngles[1] += vel;
		break;
	case GLUT_KEY_DOWN:
		if(translationOn)
			translationVector[1] -= vel;
		if(rotationOn)
			rotationAngles[1] -= vel;
		break;
	case GLUT_KEY_LEFT:
		if(translationOn)
			translationVector[0] -= vel;
		if(rotationOn)
			rotationAngles[0] -= vel;
		break;
	case GLUT_KEY_RIGHT:
		if(translationOn)
			translationVector[0] += vel;
		if(rotationOn)
			rotationAngles[0] += vel;
		break;
	case GLUT_KEY_PAGE_UP:
		if(translationOn)
			translationVector[2] += vel;
		if(rotationOn)
			rotationAngles[2] += vel;
		break;
	case GLUT_KEY_PAGE_DOWN:
		if(translationOn)
			translationVector[2] -= vel;
		if(rotationOn)
			rotationAngles[2] -= vel;
		break;
	default:
		break;
	}
	glutPostRedisplay();
}

void mainMenu(int id)
{
}

void resetBooleans()
{
	
	translationOn = false;
	rotationOn = false;
	scaleOn = false;

}


void transformationMenu(int id)
{

	resetBooleans();
	switch(id)
	{
	case 0:
		translationOn = true;
		break;
	case 1:
		rotationOn = true;
		break;
	case 2:
		scaleOn = true;
		break;
	}
}

void otherFunctionsMenu(int id)
{
	switch(id)
	{
	case 0:
		saveModel();
		break;
	case 1:
		showCurvatureMap = !showCurvatureMap;
		if(showCurvatureMap) showDepthMap = false;
		else showDepthMap = true;
		break;
	case 2:
		showContoursMap = !showContoursMap;
		break;
	}
}

void init()
{
	
	//initialize some conditions
	glClearColor( 0.0f, 0.0f, 0.0f, 0.0 );
	glShadeModel(GL_SMOOTH);
	glPixelStorei( GL_UNPACK_ALIGNMENT, 1);  
	
	//buffer objects
	if(texVBO[0] == 0)
		glGenTextures(25, texVBO);
	if(meshVBO[0] == 0)
		glGenBuffers(4, meshVBO);

	myGLImageViewer = new MyGLImageViewer();
	myGLCloudViewer = new MyGLCloudViewer();
	myGLCloudViewer->setEyePosition(1, 0, 120);

	myGLImageViewer->loadDepthComponentTexture(NULL, texVBO, VIRTUAL_DEPTH_BO, windowWidth, windowHeight);
	myGLImageViewer->loadDepthComponentTexture(NULL, texVBO, REAL_DEPTH_FROM_DEPTHBUFFER_BO, windowWidth, windowHeight);
	myGLImageViewer->loadDepthComponentTexture(NULL, texVBO, FRONT_QUAD_DEPTH_FBO, windowWidth, windowHeight);
	myGLImageViewer->loadDepthComponentTexture(NULL, texVBO, BACK_QUAD_DEPTH_FBO, windowWidth, windowHeight);
	myGLImageViewer->loadDepthComponentTexture(NULL, texVBO, SUBTRACTION_MASK_DEPTH_FBO, windowWidth, windowHeight);

	myGLImageViewer->loadRGBTexture(NULL, texVBO, VIRTUAL_RGB_BO, windowWidth/2, windowHeight/2);
	myGLImageViewer->loadRGBTexture(NULL, texVBO, REAL_RGB_FROM_FBO, windowWidth/2, windowHeight/2);
	myGLImageViewer->loadRGBTexture(NULL, texVBO, FRONT_QUAD_RGB_FBO, windowWidth/2, windowHeight/2);
	myGLImageViewer->loadRGBTexture(NULL, texVBO, BACK_QUAD_RGB_FBO, windowWidth/2, windowHeight/2);
	myGLImageViewer->loadRGBTexture(NULL, texVBO, SUBTRACTION_MASK_FBO, windowWidth/2, windowHeight/2);

	
	clippedImage = (unsigned char*)malloc(640 * 480 * sizeof(unsigned char));
	image = cvCreateImage(cvSize(windowWidth/2, windowHeight/2), IPL_DEPTH_8U, 3);
	grayImage = cvCreateImage(cvSize(windowWidth/2, windowHeight/2), IPL_DEPTH_8U, 1);

}

void releaseObjects() {

  delete capture;
  delete imageCollection;
  delete reconstruction;
  delete myGLImageViewer;
  delete myGLCloudViewer;
  if(integrateColors)
	  delete coloredReconstructionMediator;
  if(isHeadPoseEstimationEnabled)
	  delete headPoseEstimationMediator;
  if(hasFaceDetection)
	  delete faceDetector;
  delete [] clippedImage;
  cvReleaseImage(&image);
  cvReleaseImage(&grayImage);

}

int main(int argc, char **argv) {

  pcl::gpu::setDevice (0);
  pcl::gpu::printShortCudaDeviceInfo (0);

  //This argument is an exception. It is loaded first because it is necessary to instantiate the Reconstruction object
  Eigen::Vector3i volumeSize(3000, 3000, 3000); //mm
  if(pcl::console::parse_3x_arguments(argc, argv, "--volumesize", volumeSize(0), volumeSize(1), volumeSize(2)) >= 0) {
  }
  
  try
  {
	//Initialize some objects
	reconstruction = new Reconstruction(volumeSize);
	//capture = new ONIGrabber("teste.oni");
	capture = new Kinect();
	imageCollection = new Image(640, 480);

	imageCollection->setDepthIntrinsics(capture->getFocalLength(), capture->getFocalLength());
	imageCollection->setTrancationDistance(volumeSize);

	reconstruction->setIntrinsics(imageCollection->getIntrinsics());
	reconstruction->setTrancationDistance(imageCollection->getTrancationDistance());

	loadArguments(argc, argv, reconstruction);
	
	//Initialize the GL window
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_ALPHA);
	glutInitWindowSize(windowWidth, windowHeight);
	glutInit(&argc, argv);
	glutCreateWindow("My KinFu");

	//Initialize glew
	GLenum err = glewInit();
	if (GLEW_OK != err) {
		std::cout << "Error: " << glewGetErrorString(err) << std::endl;
		exit(0);
	}
	init();

	glutReshapeFunc(reshape);
	glutDisplayFunc(display);
	glutIdleFunc(idle);
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(specialKeyboard);

	initShader("Shaders/Phong", 0);
	initShader("Shaders/Occlusion", 1);

	myGLCloudViewer->setProgram(shaderProg[0]);
	myGLImageViewer->setProgram(shaderProg[1]);
	
	glutMainLoop();

  } 
  catch (const std::bad_alloc& /*e*/)
  {
    cout << "Bad alloc" << endl;
  }
  catch (const std::exception& /*e*/)
  {
    cout << "Exception" << endl;
  }

  releaseObjects();
  return 0;

}
