//Libfreenect, etc
#include <libfreenect.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <math.h>
#include <pthread.h>
#include <time.h> 

//Local headers
#include "defs.h"
#include "myfreenectdevice.h"

//OpenCV
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

//PCL
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include "pcl/registration/ia_ransac.h"
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transforms.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include "pcl/features/normal_3d.h"

//EIGEN

// OpenGL
#if defined(__APPLE__)
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif


using namespace cv;
using namespace std;
using namespace pcl;
using namespace Eigen;

// helper methods :)
void displayPolygon();
void register3DScene();
void drawRGBScene();
void idle();
void InitGL();
void defaultGL();
void constructScene();
void showAxis();
void mainDisplay();
void drawDepthScene();
void rotateCamera();
void getInterestPoints();
void getPointDescriptors();

/*
 * PCL variables
 */
//PointCloud<PointXYZRGB>::Ptr cloud_in (new PointCloud<PointXYZRGB>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

PointCloud<PointXYZRGB>::Ptr cloud_prev (new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloud_final (new PointCloud<PointXYZRGB>);

int number_of_points   = 0;

int MAX_Z              = 200; // in MM

float SIFT_SCALE       = 0.010f;
float LEAF_SIZE        = 0.001f;
float NORMAL_RADIUS    = 0.05f; //0.05
float FEATURE_RADIUS   = 0.25f; //0.15
float SAC_MAX_COR_DIST = 0.01; //.01
float SAC_MIN_SAMPLE   = 0.25f; //.25
int SAC_MAX_ITER       = 300;

/*
 * gluLookAt variables
 */
static float eyeX = 0.0f;
static float eyeY = 0.0f;
static float eyeZ = 0.0f;

// sight vector 
static float dx = 0.0f;
static float dy = 0.0f;
static float dz = 1.0f;

//angle
static float angle = 0.0f;
 
/*
 * Basic OpenGL and logic variables defined here
 *
 */
GLuint gl_depth_tex;
GLuint gl_rgb_tex;
int g_argc;
char **g_argv;
int got_frames(0);
int window(0);
int subWindow1(0);
int subWindow2(0);
int subWindow3(0);

int showScene = 0;
int registerScene = 0;

vector<uint16_t> sceneDepth(640*480*4);
vector<uint8_t> sceneRGB(640*480*4);

/*
 * OpenCV variables
 */
static int total_points = 0;
static Mat prevImage(480, 640, CV_8UC1);
static int prevReg = 0;

int minHessian = 400;
SurfFeatureDetector detector( minHessian );
SurfDescriptorExtractor extractor;
FlannBasedMatcher matcher;

/*
 * Libfreenect variables
 */
Freenect::Freenect freenect;
MyFreenectDevice* device;
double freenect_angle(0);
freenect_video_format requested_video_format(FREENECT_VIDEO_IR_8BIT);//FREENECT_VIDEO_RGB
freenect_depth_format requested_depth_format(FREENECT_DEPTH_REGISTERED);
