#include <libfreenect.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <math.h>
#include <pthread.h>


//Local headers
#include "defs.h"
#include "myfreenectdevice.h"


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

using namespace std;

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

std::vector<uint16_t> sceneDepth(640*480*4);
std::vector<uint8_t> sceneRGB(640*480*4);

/*
 * Libfreenect variables
 */
Freenect::Freenect freenect;
MyFreenectDevice* device;
double freenect_angle(0);
freenect_video_format requested_video_format(FREENECT_VIDEO_IR_8BIT);//FREENECT_VIDEO_RGB
freenect_depth_format requested_depth_format(FREENECT_DEPTH_REGISTERED);


/*
 * TESTING OpenGL
 * Simple method to draw simple polygons 
 */
void displayPolygon() {

  // Set every pixel in the frame buffer to the current clear color.
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();//load identity matrix
	glDisable(GL_TEXTURE_2D);

    //glTranslatef(0.0f,0.0f,-4.0f);//move forward 4 units
    
    glColor3f(0.0f,0.0f,1.0f); //blue color
        
    glBegin(GL_POLYGON);//begin drawing of polygon
      glVertex3f(-0.5f,0.5f,0.0f);//first vertex
      glVertex3f(0.5f,0.5f,0.0f);//second vertex
      glVertex3f(1.0f,0.0f,0.0f);//third vertex
      glVertex3f(0.5f,-0.5f,0.0f);//fourth vertex
      glVertex3f(-0.5f,-0.5f,0.0f);//fifth vertex
      glVertex3f(-1.0f,0.0f,0.0f);//sixth vertex
    glEnd();//end drawing of polygon

    // glBegin(GL_POLYGON);//begin drawing of polygon
    //   //glVertex3f(-0.5f,0.5f,0.0f);//first vertex
    //   //glVertex3f(0.5f,0.5f,0.0f);//second vertex
    //   glVertex3f(320,0,0.0f);//third vertex
    //   glVertex3f(640,240,0.0f);//fourth vertex
    //   glVertex3f(320,480,0.0f);//fifth vertex
    //   glVertex3f(0,240,0.0f);//sixth vertex
    // glEnd();//end drawing of polygon
	glutSwapBuffers();
}


/*
 * Rotates the camera to show 3D points
 */
void rotateCamera() {
	static double angle = 0.;
	static double radius = 3.;
	double x = radius*sin(angle);
	double z = radius*(1-cos(angle)) - radius/2;
	glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
	gluLookAt(x,0,z,0,0,radius/2,0,-1,0);
	angle += 0.05;
}


/*
 * Define Kinect's keyboard controls
 * glutKeyboardFunc Handler
 */
void keyPressed(unsigned char key, int x, int y) {
	if (key == 27) {
		device->setLed(LED_OFF);
		freenect_angle = 0;
		glutDestroyWindow(window);
	}
	if (key == '1') {
		device->setLed(LED_GREEN);
	}
	if (key == '2') {
		device->setLed(LED_RED);
	}
	if (key == '3') {
		device->setLed(LED_YELLOW);
	}
	if (key == '4') {
		device->setLed(LED_BLINK_GREEN);
	}
	if (key == '5') {
		// 5 is the same as 4
		device->setLed(LED_BLINK_GREEN);
	}
	if (key == '6') {
		device->setLed(LED_BLINK_RED_YELLOW);
	}
	if (key == '0') {
		device->setLed(LED_OFF);
	}
	if (key == 'd') {
		device->setDepthFormat(requested_depth_format);
	}
	if (key == 'f') {
		if (requested_video_format == FREENECT_VIDEO_IR_8BIT) {
			requested_video_format = FREENECT_VIDEO_RGB;
		} else if (requested_video_format == FREENECT_VIDEO_RGB){
			requested_video_format = FREENECT_VIDEO_YUV_RGB;
		} else {
			requested_video_format = FREENECT_VIDEO_IR_8BIT;
		}
		device->setVideoFormat(requested_video_format);
	}

	if (key == 'w') {
		freenect_angle++;
		if (freenect_angle > 30) {
			freenect_angle = 30;
		}
	}
	if (key == 's' || key == 'd') {
		freenect_angle = 10;
	}
	if (key == 'x') {
		freenect_angle--;
		if (freenect_angle < -30) {
			freenect_angle = -30;
		}
	}
	if (key == 'e') {
		freenect_angle = 10;
	}
	if (key == 'c') {
		freenect_angle = -10;
	}

	if (key == 'v') {
		register3DScene();
		showScene = 1;
	}

	device->setTiltDegrees(freenect_angle);
}



//show the current kinect RGB scene
void drawRGBScene() {
	//static std::vector<uint8_t> depth(640*480*4);
	static std::vector<uint8_t> rgb(640*480*4);

	// using getTiltDegs() in a closed loop is unstable
	/*if(device->getState().m_code == TILT_STATUS_STOPPED){
		freenect_angle = device->getState().getTiltDegs();
	}*/
	device->updateState();
	//cout << "Device tilt angle: " << device->getState().getTiltDegs();
	printf("\r demanded tilt angle: %+4.2f device tilt angle: %+4.2f", freenect_angle, device->getState().getTiltDegs());
	fflush(stdout);

	//bool depth_registered = false;
	bool rgb_registered = false;

	while(!rgb_registered)
		rgb_registered = device->getRGB(rgb);

	got_frames = 0;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	//glViewport(0, (float)HEIGHT/2, (float)WIDTH/2, HEIGHT);
	glEnable(GL_TEXTURE_2D);

	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glTexImage2D(GL_TEXTURE_2D, 0, 4, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, &rgb[0]);

	glBegin(GL_TRIANGLE_FAN);
		glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
		glTexCoord2f(0, 0); glVertex3f(0,0,0);
		glTexCoord2f(1, 0); glVertex3f(640,0,0);
		glTexCoord2f(1, 1); glVertex3f(640,480,0);
		glTexCoord2f(0, 1); glVertex3f(0,480,0);
	glEnd();

	glutSwapBuffers();
}

//show the current kinect depth scene
void drawDepthScene() {

	static std::vector<uint8_t> depth(640*480*4);

	device->updateState();
	printf("\r demanded tilt angle: %+4.2f device tilt angle: %+4.2f", freenect_angle, device->getState().getTiltDegs());
	fflush(stdout);

	bool depth_registered = false;

	while(!depth_registered)
		depth_registered = device->getDepth(depth);

	got_frames = 0;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	//glViewport(0, (float)HEIGHT/2, (float)WIDTH/2, HEIGHT);
	glEnable(GL_TEXTURE_2D);

	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glTexImage2D(GL_TEXTURE_2D, 0, 4, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, &depth[0]);

	glBegin(GL_TRIANGLE_FAN);
		glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
		glTexCoord2f(0, 0); glVertex3f(0,0,0);
		glTexCoord2f(1, 0); glVertex3f(640,0,0);
		glTexCoord2f(1, 1); glVertex3f(640,480,0);
		glTexCoord2f(0, 1); glVertex3f(0,480,0);
	glEnd();

	glutSwapBuffers();
}



// Registers 3D depth points from the Kinect
void register3DScene() {
	bool depth_registered = false;
	bool rgb_registered = false;

	while(!depth_registered)
		depth_registered = device->getDepthInMM(sceneDepth);
	while(!rgb_registered)
		rgb_registered = device->getRGB(sceneRGB);

	printf("\n depth was registered: %d", depth_registered);
	printf("\n rgb was registered: %d", rgb_registered);
}

//Shows the axis alignment
void showAxis() {
	 //draw axis lines//
	 
	 //x-axis - RED
	 glBegin(GL_LINES);
	 glColor3f(1.0f,0.0f,0.0f);
	 glVertex3f(0.0f,0.0f,0.0f);
	 glVertex3f(100.0f, 0.0f,0.0f);
	 glEnd();

	 //y-axis - GREEN
	 glBegin(GL_LINES);
	 glColor3f(0.0f,1.0f,0.0f);
 	 glVertex3f(0.0f, 100.0f,0.0f);
	 glVertex3f(0.0f,0.0f,0.0f);
	 glEnd();

	 //z-axis - BLUE
	 glBegin(GL_LINES);
	 glColor3f(0.0f,0.0f,1.0f);
	 glVertex3f(0.0f,0.0f,0.0f);
	 glVertex3f(0.0f, 0.0f,100.0f);
	 glEnd();

	 glColor3f(1,1,1);

}

// Constructs 3D point cloud scene
void constructScene() {
		//displayPolygon();
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glLoadIdentity();
		//glViewport(0, 0, (float)WIDTH, HEIGHT/2);

	    rotateCamera();

    	if(showScene) {
	    	glBegin(GL_POINTS);
		  //   for (int i = 0; i < WIDTH*HEIGHT; ++i) {
				// if(i < 50) {
				// 	printf("\n color 1: %d \n color 2: %d \n color 3: %d \n", sceneRGB[i*3], sceneRGB[i*3+1], sceneRGB[i*3+2]);
				// 	printf("\n depth 1: %d \n depth 2: %d \n depth 3: %d \n", sceneDepth[i*3], sceneDepth[i*3+1], sceneDepth[i*3+2]);
		  //       }
		  //       glColor3f((float)sceneRGB[i*3]/255, (float)sceneRGB[i*3+1]/255, (float)sceneRGB[i*3+2]/255);
		  //       //glVertex3f((float)sceneDepth[i*3]/255, (float)sceneDepth[i*3+1]/255, (float)sceneDepth[i*3+2]/255);
		  //       glVertex3f((float)sceneDepth[i*3]/255, (float)sceneDepth[i*3+1]/255, (float)sceneDepth[i*3+2]/255);
		  //   }
	  		//glColor3f(0.5, 0.5, 0.5);
		    for (int i = 0; i < FREENECT_FRAME_PIX; i++) {
		  	  		glColor3f((float)sceneRGB[i*3]/255, (float)sceneRGB[i*3+1]/255, (float)sceneRGB[i*3+2]/255);
		    		//if((float)sceneDepth[i] != 0 || (float)sceneDepth[i] != 10000)
		    		float ii = i % FREENECT_FRAME_W;
		    		float j = floor(i/FREENECT_FRAME_W);
		    		float z = (float)sceneDepth[i]/10;
		    		float x = (ii - FREENECT_FRAME_W/2) * (z - 10) * .0021;
		    		float y = (j - FREENECT_FRAME_H/2) * (z - 10) * .0021;
  	  				glVertex3f(x/20, y/20, z/20);
			}
		    glEnd();
		}else {
			showAxis();
		}

	    glutSwapBuffers();
}

/*
 * Initialize OpenGL variables for 3D perspective projection
 * Currently: used for botton "point cloud" window 
 * 
 */
void defaultGL() {
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0);
	glDepthFunc(GL_LESS);
	glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_MODELVIEW);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//glOrtho (0, 640, 480, 0, 0.0f, 10000);
	gluPerspective(150.0f, (float)FREENECT_FRAME_W/FREENECT_FRAME_H, 0.1f, 100);
	glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
	gluLookAt(0,0,0,0,0,1,0,0,0);
}

/*
 * Initialize OpenGL variables for 2D texture drawings
 * Currently: used for top 2 windows
 */
void InitGL() {
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0);
	glDepthFunc(GL_LESS);
	glEnable(GL_DEPTH_TEST);
	//glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glShadeModel(GL_SMOOTH);
	glGenTextures(1, &gl_depth_tex);
	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho (0, 640, 480, 0, 0.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
}

/*
 * Glut idle function
 * Redraws all of the windows regardless of window focus
 */
void idle() {
	int currentWindow = glutGetWindow();
	glutSetWindow(window);
	glutPostRedisplay();
	glutSetWindow(subWindow1);
	glutPostRedisplay();
	glutSetWindow(subWindow2);
	glutPostRedisplay();
	glutSetWindow(subWindow3);
	glutPostRedisplay();
	glutSetWindow(currentWindow);
}

// Main Window that carries the subwindows
void mainDisplay() {
	glClearColor(0.8, 0.8, 0.8, 0.0);  
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glutSwapBuffers();
}

/*
 * Sets up the GLUT display
 * Currently: 3 Windows
 */
void setUpDisplay(MyFreenectDevice* device){
	glutInit(&g_argc, g_argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
	glutInitWindowSize(WIDTH, HEIGHT);
	glutInitWindowPosition(INIT_POS_X, INIT_POS_Y);

	//
	window = glutCreateWindow("Kinect 3D Scene");
		glutDisplayFunc(mainDisplay);
		//glutDisplayFunc(&DrawGLScene);
		//glutIdleFunc(&DrawGLScene);
		glutKeyboardFunc(keyPressed);
		defaultGL();
	subWindow1 = glutCreateSubWindow(window, BORDER, BORDER, WIDTH/2-3/2*BORDER, HEIGHT/2-3/2*BORDER);
		glutDisplayFunc(drawRGBScene);
		//glutIdleFunc(&DrawGLScene);
		InitGL();
	subWindow2 = glutCreateSubWindow(window, WIDTH/2+BORDER/2, BORDER, WIDTH/2-3/2*BORDER, HEIGHT/2-3/2*BORDER);
		glutDisplayFunc(drawDepthScene);
		//glutIdleFunc(&DrawGLScene);
		InitGL();
	subWindow3 = glutCreateSubWindow(window, BORDER, HEIGHT/2+BORDER/2, WIDTH-2*BORDER, HEIGHT/2-3/2*BORDER);
		glutDisplayFunc(constructScene);
		//glutIdleFunc(&constructScene);
		//InitGL();
		defaultGL();
	glutIdleFunc(idle);
	glutMainLoop();
}

/*
 * MAIN: main loop
 * Init libfreenect, OpenGL (GLUT windows)
 */
int main(int argc, char **argv) {
	//Get Kinect Device
	device = &freenect.createDevice<MyFreenectDevice>(0);
	//Start Kinect Device
	//device->setTiltDegrees(10);
	device->startVideo();
	device->startDepth();
	
	//handle Kinect Device Data
	device->setLed(LED_GREEN);
	setUpDisplay(device);
	//cout << "Focal Length: " << reference_distance;


	//Stop Kinect Device
	device->stopVideo();
	device->stopDepth();
	device->setLed(LED_OFF);

	//RETURN
	return 1;
}