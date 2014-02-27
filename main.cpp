/*

IMU viewer
Draws a colored cube whose orientation follows the one of the IMU

The mouse cas be used to modify manually the orientation.
Press 'P' and 'M' to zoom in and out.
The orientation will stabilize after a few second at the beginning.

Compile with:
g++ *.cpp -o imuviewer -lGL -lGLU -lSDL -lserial -lm -lglut -std=c++0x -fpermissive

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
#include <stdlib.h> /* atof */
#include <iostream>
#include <string>
#include <cmath>

#include <chrono>
#include <ctime>

#define PORT "/dev/ttyUSB0"

using namespace std;
using namespace LibSerial;


SerialStream ardu;

std::vector<double> data;

float ORIG[3] = {0,0,0};
float X[3] = {1,0,0};
float Y[3] = {0,1,0};
float Z[3] = {0,0,1};
double sizeCube = 0.5;

std::chrono::time_point<std::chrono::system_clock> prevtime;

double grav[3] = {0,0,0}; // gravitation of earth

double quat[4] = {0,0,0,0};
double acc2[3] = {0,0,0};
double acc1[3] = {0,0,0};
double acc0[3] = {0,0,0};

double pos[3] = {0,0,0};


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
void RunIdleFunc(void) { glutIdleFunc(MyIdleFunc); }
void PauseIdleFunc(void) { glutIdleFunc(NULL); }

// Split the string according to the delimiter ' '
void splitData(std::string s);
void DrawFromEuler(std::vector<double> & data);
void DrawFromQuaternion(std::vector<double> & data);


void measureGravitation()
{
  double mean[3] = {0,0,0};
  int nbIter = 100;
  
  for(int k=0;k<nbIter;++k)
  {
    std::cerr << k << "/" << nbIter << "\r";
    // read data from IMU
    char c; // character
    std::string s("");
    ardu.get( c ) ;
    while(c != '#'){ardu.get( c );} // do nothing until we read the end of a block
    
    ardu.get( c );
    while(c != '#')
    {
      s += c;
      ardu.get( c );
    }
    splitData(s);
    
    mean[0] += data[4];
    mean[1] += data[5];
    mean[2] += data[6];
    
  }
  for(int j=0;j<3;++j)
    grav[j] = mean[j]/nbIter;
  
  std::cerr << std::endl;
}

void ReshapeCallback(int width, int height)
{
  xsize = width;
  ysize = height;
  aspect = (float)xsize/(float)ysize;
  glViewport(0, 0, xsize, ysize);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glutPostRedisplay();
}


//void UpdatePosition(std::vector<double> & data)
void UpdatePosition()
{
  for(int k=0;k<4;++k)
    quat[k] = data[k];

  
//  // remove gravitation from acceleration
//  double g[3] = {0,0,0};
//  
//  g[0] = 2 * (quat[1] * quat[3] - quat[0] * quat[2]);
//  g[1] = 2 * (quat[0] * quat[1] + quat[2] * quat[3]);
//  g[2] = quat[0] * quat[0] - quat[1] * quat[1] - quat[2] * quat[2] + quat[3] * quat[3];
//  
//  // update acceleration values in data vector
//  data[4] = data[4]-g[0]*3675;
//  data[5] = data[5]-g[1];
//  data[6] = data[6]-g[2];
//  
//  acc0[0] = data[4];
//  acc0[1] = data[5];
//  acc0[2] = data[6];
  
  // compute dt
  std::chrono::time_point<std::chrono::system_clock> currtime;
  currtime = std::chrono::system_clock::now();
  double dt = std::chrono::duration_cast<std::chrono::microseconds> (currtime-prevtime).count();
  prevtime = currtime;
//  std::cerr << "dt: " << dt << std::endl;
  
  // compute position
  for(int k=0;k<3;++k)
  {
    pos[k] = dt*dt*(acc0[k] + 2*acc1[k] + acc2[k]);
    acc2[k] = acc1[k];
    acc1[k] = acc0[k];
  }
  
  
  
  
  
// std::cerr << ax << " " << ay << " " << az << std::endl;
  for( int k=0;k< data.size();++k)
    std::cerr << data[k] << " ";
  std::cerr << std::endl;

}
 
void DisplayCallback(void)
{
  std::cerr << "sidpp" << std::endl;
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(zoom, aspect, zNear, zFar);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(0.0,0.0,-sdepth);
  glRotatef(-stheta, 1.0, 0.0, 0.0);
  glRotatef(sphi, 0.0, 0.0, 1.0);
  
  // read data from IMU
  
  char c; // character
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
//  std::vector<double> data = splitData(s);
  splitData(s);

// for(int k=0;k<data.size();++k)
// std::cerr << data[k] << std::endl;
  
  
  // Move the cube
  UpdatePosition();
  
  // Orient the cube
  DrawFromQuaternion(data);
  
  


  glutSwapBuffers();
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

 

void KeyboardCallback(unsigned char ch, int x, int y)
{
  std::cerr << "key" << std::endl;
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
    
    // Measure the gravitation. The device must not move
    case 'g' :
    {
      std::cerr << "Before : " << grav[0] << " " << grav[1] << " " << grav[2] << std::endl;
      measureGravitation();
      std::cerr << "After : " << grav[0] << " " << grav[1] << " " << grav[2] << std::endl;
    }
    break;
    
  }
  glutPostRedisplay();
}

 

void MouseCallback(int button, int state, int x, int y)
{
  std::cerr << "mouse" << std::endl;
  downX = x; downY = y;
  leftButton = ((button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN));
  middleButton = ((button == GLUT_MIDDLE_BUTTON) && (state == GLUT_DOWN));
  glutPostRedisplay();
}


//void mouseWheel(int button, int dir, int x, int y)
//{
//
// return;
//}


void MotionCallback(int x, int y)
{
  std::cerr << "motion" << std::endl;
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
  downX = x; downY = y;
  glutPostRedisplay();
}
 
SDL_Event event;


void InitGL()
{


// SDL_Init(SDL_INIT_VIDEO);
// atexit(SDL_Quit);
// SDL_WM_SetCaption("IMU viewer", NULL);
// SDL_SetVideoMode(640, 480, 32, SDL_OPENGL);

  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(500, 500);
  glutCreateWindow("IMU Viewer");
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
// glutMouseWheelFunc(mouseWheel);
  glutIdleFunc(DisplayCallback);
  glutKeyboardFunc(KeyboardCallback);
  glutMouseFunc(MouseCallback);
  glutMotionFunc(MotionCallback);
  std::cerr << "Inint" << std::endl;
}


int main(int argc, char **argv)
{
  // initialize global variables
  prevtime = std::chrono::system_clock::now();
  std::cerr << "koko1" << std::endl;
  for(int k=0;k<7;++k)
    data.push_back(0);
  
  ardu.Open(PORT);
  std::cerr << "koko2" << std::endl;
  ardu.SetBaudRate(SerialStreamBuf::BAUD_38400);
  ardu.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
  std::cerr << "koko3" << std::endl;

  glutInit(&argc, argv);
  std::cerr << "koko4" << std::endl;
  InitGL();
  std::cerr << "koko5" << std::endl;
  glutMainLoop();

  return 0;
}


void DrawFromQuaternion(std::vector<double> & data)
{
    std::cerr << "quat" << std::endl;
// glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
//
// glMatrixMode( GL_MODELVIEW );
// glLoadIdentity( );
 
 
// gluLookAt(0,-5,0,0,0,0,0,0,1);

    // Quaternion
    double w=data[0];
    double x=data[1];
    double y=data[2];
    double z=data[3];
    
    // Acceleration vector
    double ax=data[4];
    double ay=data[5];
    double az=data[6];
    
    


    GLfloat matrix[16];
    // Column 1 // Column 2 // Column 3 // Column 4
    matrix[0] = 1-2*(y*y+z*z); matrix[4] = 2*(x*y-w*z); matrix[8] = 2*(x*z+w*y); matrix[12] = 0;
    matrix[1] = 2*(x*y+w*z); matrix[5] = 1-2*(x*x+z*z); matrix[9] = 2*(y*z-w*x); matrix[13] = 0;
    matrix[2] = 2*(x*z-w*y); matrix[6] = 2*(y*z+w*x); matrix[10] = 1-2*(x*x+y*y); matrix[14] = 0;
    matrix[3] = 0; matrix[7] = 0; matrix[11] = 0; matrix[15] = 1;


    glMultMatrixf(matrix);
// glScaled (2 ,2 ,2);
    
    
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




//    // Plot axes
//    glLineWidth (2.0);

//    glBegin (GL_LINES);
//      glColor3f (1,0,0); // X axis is red.
//      glVertex3fv (ORIG);
//      glVertex3fv (X);
//      glColor3f (0,1,0); // Y axis is green.
//      glVertex3fv (ORIG);
//      glVertex3fv (Y);
//      glColor3f (0,0,1); // z axis is blue.
//      glVertex3fv (ORIG);
//      glVertex3fv (Z);
//    glEnd();

    
    // Plot acceleration vector
    double normacc=sqrt(ax*ax+ay*ay+az*az);
    float Acc[3] = {3*ax/normacc,-3*ay/normacc,-3*az/normacc};
    glLineWidth (2.0);
    
    glBegin (GL_LINES);
      glColor3f (1,0,0); // X axis is red.
      glVertex3fv (ORIG);
      glVertex3fv (Acc);
    glEnd();
//    glMultTransposeMatrixf(matrix);
    

 
// glFlush();
// SDL_GL_SwapBuffers();
}


//std::vector<double> splitData(std::string s)
void splitData(std::string s)
{
  std::cerr << "split" << std::endl;

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
  
  if(res.size() == data.size())
  {
    for(int k=0;k<data.size();++k)
      data[k]=res[k];
  }
  else
  {
    std::cerr << "ERROR in splitData : different sizes" << std::endl;
  }
//  return res;
}
 


