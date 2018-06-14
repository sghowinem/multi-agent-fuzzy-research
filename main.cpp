// texturebfly.cpp : Defines the entry point for the console application.
//
#include <windows.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>


#include "glut.h"
#include "bitmap.h"
#include "UCFL_Utils2.h"
#include "CFLSlib\\CFLS.h"
#include "maincode.h"
#include "sim.h"


//timing
/* Startup */
__int64 countsPerSec = 0;
__int64 prevTimeStamp = 0;
float secsPerCount;
/* Per frame */
__int64 currTimeStamp;
float deltatime;

int frameCount = 0, prvframeCount = 0;
int framePerSecond=0;

int runMode=1; // 0=test mode  1=sim mode
int timerInterval=50; // how fast

GLfloat Near=0.5;
GLfloat Far=20.0;
GLfloat aspect; // screen aspect ratio 

char VersionString[] = "Fuzzy Explorer V0.8.0.1 alpha";
int  window1,window2;

enum {  // enumerations for menu
  cmdNull,
  cmdExit
};

static int	AppMenu;

bool timeForNextFrame=false;
bool simHasRun=false;

int err;
const GLubyte *oglVersion; 

int windowWidth,windowHeight;


static void display ()
{
  frameCount++;
  if(frameCount%20 == 0)
  {
	  //fps timing
	  QueryPerformanceCounter((LARGE_INTEGER*)&currTimeStamp);
      deltatime = (currTimeStamp - prevTimeStamp)*secsPerCount; 
      secsPerCount = 1.0 / (double)countsPerSec;
      prevTimeStamp = currTimeStamp;framePerSecond = 20/deltatime;
	  prvframeCount++;
  } 

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

 // Do the 2D drawing bit
  glDisable(GL_DEPTH_TEST);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(0.0, (GLfloat)windowWidth, 0.0, (GLfloat)windowHeight);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  simRender();
  
  // Check for errors and write a diagnostic string if detected.
  // On error, the app will freeze (but not exit), as the buffers
  // won't be swapped...
  glFlush();
  err = glGetError();
  if (err == GL_NO_ERROR)
    glutSwapBuffers();
  else
    printf("GL Error >>%s\n", gluErrorString(err));
}

static void display2 ()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Do the 2D drawing bit
  glDisable(GL_DEPTH_TEST);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(0.0, (GLfloat)windowWidth, 0.0, (GLfloat)windowHeight);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  mainRender();

  
  // Check for errors and write a diagnostic string if detected.
  // On error, the app will freeze (but not exit), as the buffers
  // won't be swapped...
  glFlush();
  err = glGetError();
  if (err == GL_NO_ERROR)
    glutSwapBuffers();
  else
    printf("GL Error >>%s\n", gluErrorString(err));
}

static void updateIdle (void)
{
  if (timeForNextFrame && simHasRun)
  {
	  glutSetWindow(window1);
      glutPostRedisplay();
	  glutSetWindow(window2);
      glutPostRedisplay();
      timeForNextFrame=false;
      simHasRun=false;
  }
  else
	  if (!simHasRun) simIdle();
}

static void menuChoice (int item)
{
  switch (item) {
    case cmdExit:
      exit(0);
      break;
    default:
      break;
  }
}

void myExitfn()
{
  printf("Exiting\n");
  if (runMode==1) simExit();
  printf("Exit complete\n");
}

static void initGraphics (void)
{
	//timing 
	QueryPerformanceFrequency((LARGE_INTEGER*)&countsPerSec);
	secsPerCount = 1.0 / (double)countsPerSec;
	QueryPerformanceCounter((LARGE_INTEGER*)&prevTimeStamp);

	glClearColor(0.9, 0.9, 1, 0.0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	simInit();

	AppMenu = glutCreateMenu(menuChoice);
	glutSetMenu(AppMenu);
	glutAddMenuEntry("----", cmdNull);
	glutAddMenuEntry("Exit", cmdExit);
	glutAttachMenu(GLUT_RIGHT_BUTTON);

	atexit(myExitfn);
}
static void initGraphics2 (void)
{
  glClearColor(0.9, 0.9, 1, 0.0);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	mainCode();

  AppMenu = glutCreateMenu(menuChoice);
  glutSetMenu(AppMenu);
  glutAddMenuEntry("----", cmdNull);
  glutAddMenuEntry("Exit", cmdExit);
  glutAttachMenu(GLUT_RIGHT_BUTTON);

  atexit(myExitfn);
}

static void resize (int width, int height)
{
  windowWidth=width;
  windowHeight=height;
  glViewport(0, 0, width, height);
}



static void asciiKey (unsigned char key, int x, int y)
{
  simKey(key,x,y);
  key_generic(key);

  if (key==27) exit(0); /* 27 is ESC */
 
}

void timer(int value)
{
	timeForNextFrame=true;
	glutTimerFunc(timerInterval,timer,0);
	simTimer(value);
	mainTimer(value);
}

int main (int argc, char * argv[])
{
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(600, 600);

  //create first window
  glutInitWindowPosition(50, 50);
  window1 = glutCreateWindow(VersionString);
  initGraphics();
  glutDisplayFunc(display);
  glutReshapeFunc(resize);
 
  //create second window
  glutInitWindowPosition(655, 50);
  window2 = glutCreateWindow("Second");
  initGraphics2();  
  glutDisplayFunc(display2);
  glutReshapeFunc(resize);


  oglVersion=glGetString(GL_VERSION);
  printf("openGl version %s \n",oglVersion);
 

  
  glutKeyboardFunc(asciiKey);
  glutTimerFunc(timerInterval,timer,0);
  glutIdleFunc(updateIdle);
  glutMainLoop();
  return 0;
}


