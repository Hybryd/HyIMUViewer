
/*

 IMU viewer
 Draws a colored cube whose orientation follows the one of the IMU

 The mouse cas be used to modify manually the orientation.
 Press 'P' and 'M' to zoom in and out.
 
 The orientation will stabilize after a few second at the beginning.

 Compilae with : g++ *.cpp -o main -lGL -lGLU -lSDL -lserial -lm -lglut

*/


#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <GL/glut.h>
#include <SDL/SDL.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <cstdlib>

#include <SerialStream.h>
#include <stdlib.h>     /* atof */
#include <iostream>
#include <string>
#include <cmath>

#define PORT "/dev/ttyUSB0"

using namespace std;
using namespace LibSerial;


SerialStream ardu;

float ORIG[3] = {0,0,0};
float X[3]    = {1,0,0};
float Y[3]    = {0,1,0};
float Z[3]    = {0,0,1};
double sizeCube = 0.5;


/* Viewer state */

float sphi=90.0, stheta=45.0;
float sdepth = 10;
float zNear=1.0, zFar=100.0;
float aspect = 5.0/4.0;

float zoom = 70;

long xsize, ysize;
int downX, downY;
bool leftButton = false, middleButton = false;
int i,j;

GLfloat light0Position[] = { 0, 1, 0, 1.0};


void MyIdleFunc(void) { glutPostRedisplay();} /* things to do while idle */
void RunIdleFunc(void) {   glutIdleFunc(MyIdleFunc); }
void PauseIdleFunc(void) {   glutIdleFunc(NULL); }

// Split the string according to the delimiter ' '
std::vector<double> splitData(std::string s);
void DrawFromEuler(std::vector<double> & data);
void DrawFromQuaternion(std::vector<double> & data);


void ReshapeCallback(int width, int height)
{
  xsize = width;
  ysize = height;
  aspect = (float)xsize/(float)ysize;
  glViewport(0, 0, xsize, ysize);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glutPostRedisplay();
}


void UpdatePosition(std::vector<double> & data)
{
  double ax=data[4];
  double ay=data[5];
  double az=data[6];
//  std::cerr << ax << " " << ay << " " << az << std::endl;
  for( int k=0;k< data.size();++k)
    std::cerr << data[k] << " ";
  std::cerr <<  std::endl;

}
 
void DisplayCallback(void)
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(zoom, aspect, zNear, zFar);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(0.0,0.0,-sdepth);
  glRotatef(-stheta, 1.0, 0.0, 0.0);
  glRotatef(sphi, 0.0, 0.0, 1.0);
  
  // read data from IMU
  char c;
  std::string s("");
  ardu.get( c ) ;
  while(c != '#'){ardu.get( c );} // do nothing until we read the end of a block
  
  ardu.get( c );
  while(c != '#')
  {
    s += c;
    ardu.get( c );
  }
  
  // Convert string to std::vector<double>
  std::vector<double> data = splitData(s);

//  for(int k=0;k<data.size();++k)
//    std::cerr << data[k] << std::endl;
  
  // Orient the cube
  DrawFromQuaternion(data);
  
  // Move the cube
  UpdatePosition(data);


  glutSwapBuffers();
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

 

void KeyboardCallback(unsigned char ch, int x, int y)
{
  switch (ch)
  {
    case 27: exit(0);
    break;
    case 'q' : exit(0);
    break;
    case 'p' :
    {
      --zoom; // Or whatever you want the step to be
      glMatrixMode(GL_PROJECTION); // You had GL_MODELVIEW
      glOrtho(-1.5 + zoom, 1.0 - zoom, -2.0 + zoom, 0.5 - zoom, -1.0, 3.5); // Changed some of the signs here
    }
    break;
    
    case 'm' :
    {
      ++zoom; // Or whatever you want the step to be
      glMatrixMode(GL_PROJECTION); // You had GL_MODELVIEW
      glOrtho(-1.5 + zoom, 1.0 - zoom, -2.0 + zoom, 0.5 - zoom, -1.0, 3.5); // Changed some of the signs here
    }
    break;
  }
  glutPostRedisplay();
}

 

void MouseCallback(int button, int state, int x, int y)
{
  downX = x; downY = y;
  leftButton = ((button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN));
  middleButton = ((button == GLUT_MIDDLE_BUTTON) &&  (state == GLUT_DOWN));
  glutPostRedisplay();
}


//void mouseWheel(int button, int dir, int x, int y)
//{
//  
//  return;
//}


void MotionCallback(int x, int y)
{
   // rotate
  if (leftButton)
  {
    sphi+=(float)(x-downX)/4.0;stheta+=(float)(downY-y)/4.0;
  }
  
   // scale
  if (middleButton)
  {
    sdepth += (float)(downY - y) / 10.0;
  }
  downX = x;   downY = y;
  glutPostRedisplay();
}
 
SDL_Event event;


void InitGL()
{


//  SDL_Init(SDL_INIT_VIDEO);
//  atexit(SDL_Quit);
//  SDL_WM_SetCaption("IMU viewer", NULL);
//  SDL_SetVideoMode(640, 480, 32, SDL_OPENGL);

  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(500, 500);
  glutCreateWindow("cs175 Triangle Viewer");
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glPolygonOffset(1.0, 1.0);
  gluLookAt(0,-5,0,0,0,0,0,0,1);
  glDisable(GL_CULL_FACE);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
  glEnable(GL_COLOR_MATERIAL);
  glColorMaterial(GL_FRONT, GL_DIFFUSE);
  glLightfv (GL_LIGHT0, GL_POSITION, light0Position);
  glEnable(GL_LIGHT0);
  glutReshapeFunc(ReshapeCallback);
  glutDisplayFunc(DisplayCallback);
//  glutMouseWheelFunc(mouseWheel);
  glutIdleFunc(DisplayCallback);
  glutKeyboardFunc(KeyboardCallback);
  glutMouseFunc(MouseCallback);
  glutMotionFunc(MotionCallback);
}


int main(int argc, char **argv)
{
  ardu.Open(PORT);    
  ardu.SetBaudRate(SerialStreamBuf::BAUD_38400);  
  ardu.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);

  glutInit(&argc, argv);

  InitGL();

  glutMainLoop(); 
  return 0;
}


void DrawFromQuaternion(std::vector<double> & data)
{
    
//    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
// 
//    glMatrixMode( GL_MODELVIEW );
//    glLoadIdentity( );
 
 
//    gluLookAt(0,-5,0,0,0,0,0,0,1);

    double w=data[0];
    double x=data[1];
    double y=data[2];
    double z=data[3];

    GLfloat matrix[16];
    // Column 1                  // Column 2                  // Column 3                  // Column 4
    matrix[0] = 1-2*(y*y+z*z);   matrix[4] = 2*(x*y-w*z);     matrix[8] = 2*(x*z+w*y);     matrix[12] = 0;
    matrix[1] = 2*(x*y+w*z);     matrix[5] = 1-2*(x*x+z*z);   matrix[9] = 2*(y*z-w*x);     matrix[13] = 0;
    matrix[2] = 2*(x*z-w*y);     matrix[6] = 2*(y*z+w*x);     matrix[10] = 1-2*(x*x+y*y);  matrix[14] = 0;
    matrix[3] = 0;               matrix[7] = 0;               matrix[11] = 0;              matrix[15] = 1;


    glMultMatrixf(matrix);
//    glScaled (2 ,2 ,2);
    
    
    glBegin(GL_QUADS);
    
 
    glColor3ub(255,0,0); //face rouge
    glVertex3d(sizeCube,sizeCube,sizeCube);
    glVertex3d(sizeCube,sizeCube,-sizeCube);
    glVertex3d(-sizeCube,sizeCube,-sizeCube);
    glVertex3d(-sizeCube,sizeCube,sizeCube);
 
    glColor3ub(0,255,0); //face verte
    glVertex3d(sizeCube,-sizeCube,sizeCube);
    glVertex3d(sizeCube,-sizeCube,-sizeCube);
    glVertex3d(sizeCube,sizeCube,-sizeCube);
    glVertex3d(sizeCube,sizeCube,sizeCube);
 
    glColor3ub(0,0,255); //face bleue
    glVertex3d(-sizeCube,-sizeCube,sizeCube);
    glVertex3d(-sizeCube,-sizeCube,-sizeCube);
    glVertex3d(sizeCube,-sizeCube,-sizeCube);
    glVertex3d(sizeCube,-sizeCube,sizeCube);
 
    glColor3ub(255,255,0); //face jaune
    glVertex3d(-sizeCube,sizeCube,sizeCube);
    glVertex3d(-sizeCube,sizeCube,-sizeCube);
    glVertex3d(-sizeCube,-sizeCube,-sizeCube);
    glVertex3d(-sizeCube,-sizeCube,sizeCube);
 
    glColor3ub(0,255,255); //face cyan
    glVertex3d(sizeCube,sizeCube,-sizeCube);
    glVertex3d(sizeCube,-sizeCube,-sizeCube);
    glVertex3d(-sizeCube,-sizeCube,-sizeCube);
    glVertex3d(-sizeCube,sizeCube,-sizeCube);
 
    glColor3ub(255,0,255); //face magenta
    glVertex3d(sizeCube,-sizeCube,sizeCube);
    glVertex3d(sizeCube,sizeCube,sizeCube);
    glVertex3d(-sizeCube,sizeCube,sizeCube);
    glVertex3d(-sizeCube,-sizeCube,sizeCube);

    glEnd();


    // Plot axes
    glLineWidth (2.0);

    glBegin (GL_LINES);
      glColor3f (1,0,0); // X axis is red.
      glVertex3fv (ORIG);
      glVertex3fv (X);
      glColor3f (0,1,0); // Y axis is green.
      glVertex3fv (ORIG);
      glVertex3fv (Y);
      glColor3f (0,0,1); // z axis is blue.
      glVertex3fv (ORIG);
      glVertex3fv (Z);
    glEnd();
 
//    glFlush();
//    SDL_GL_SwapBuffers();
}


std::vector<double> splitData(std::string s)
{

  std::vector<double> res;
  std::string tmp("");
  int k=0;
  while(k<s.size())
  {
    if(s[k]!=' ')
      tmp+=s[k];
    else
    {
      res.push_back(atof(tmp.c_str()));
      tmp=" ";
    }
    ++k;
  }
  res.push_back(atof(tmp.c_str()));
  return res;
}
 

