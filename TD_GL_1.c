////////////////////////////////
// 2012	                      //
// 3D animation		      //
// Univ. of Paris-Sud	      //  
// Mehdi AMMI - ammi@limsi.fr //
////////////////////////////////

#include <GL/glut.h>    // Header for GLUT 
#include <GL/gl.h>	// Header for OpenGL
#include <GL/glu.h>	// Header for GLu
#include <stdlib.h>     // Heard  Utilitaire général

/* light parameters */

GLfloat light_position[] = { 1.0F,0.0F,1.0F,0.0F };
GLfloat lightambiant[] = { 1.0f,1.0f,1.0f,0.0};

/*  ASCII code for escape key*/
#define ESCAPE 27

/* GLUT ID for the window */
int window; 

/* initialisation*/
void InitGL(int Width, int Height)	        
{
// ..
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);		

// ..
  glClearDepth(1.0);	
  glDepthFunc(GL_LESS);	
  glEnable(GL_DEPTH_TEST);	

//  lighting enabling **
//  glEnable(GL_LIGHTING);
//  glEnable(GL_LIGHT0); 	

  glLightfv(GL_LIGHT0,GL_POSITION,light_position);	
  glLightfv(GL_LIGHT0,GL_AMBIENT,lightambiant);


// ..
  glShadeModel(GL_SMOOTH);			

// ..
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();				
  gluPerspective(45.0f,(GLfloat)Width/(GLfloat)Height,0.1f,100.0f);	

//  Orthogonal projection without aspect ratio
//..

//  Orthogonal projection with aspect ratio
//..

  glMatrixMode(GL_MODELVIEW);

}

/* resize (rechape)  function*/
void ReSizeGLScene(int Width, int Height)
{
  if (Height==0)				
    Height=1;
  glViewport(0, 0, Width, Height);		

// ..
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();				
  gluPerspective(45.0f,(GLfloat)Width/(GLfloat)Height,0.1f,100.0f);	

//  Orthogonal projection without aspect ratio
//..

//  Orthogonal projection with aspect ratio
//..


  glMatrixMode(GL_MODELVIEW);

}

/* draw function*/
void DrawGLScene()
{
// ..
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		
  glLoadIdentity();		


  glLoadIdentity();
  gluLookAt(8.0,6.0,4.0,0.0,0.0,0.0,0.0,1.0,0.0); 

  // Drawing points and lines
	glBegin(GL_POINTS);
	//glPointSize(4);
	glVertex2i(1,3);
	glVertex2i(5,5);
	glVertex2i(0,0);
	glEnd();
  //..
	glBegin(GL_LINES);
	glColor3f(0,1,0); // y
	glVertex3f(0,0,0);
	glVertex3f(0,1,0);
	glColor3f(1,0,0); //x	
	glVertex3f(0,0,0);
	glVertex3f(1,0,0);
	glColor3f(0,0,1); //z
	glVertex3f(0,0,0);
	glVertex3f(0,0,1);
	glEnd();

  // Drawing triangles
	/*glBegin(GL_TRIANGLES);
	glColor3f(2,1,2);
	glVertex3f(2,2,0);
	glVertex3f(1,2,0);
	glVertex3f(1,1,0);
	// interpoler
	glColor3f(1,0,0);
	glVertex3f(0,1,2);
	glColor3f(0,1,0);
	glVertex3f(1,-1,2);
	glColor3f(0,0,1);
	glVertex3f(-1,-1,2);
	glEnd();
	*/
  //..


  // Drawing triangles with unifom color

  //..


  //Drawing triangles with interpolated color

  //..				



  // Cube 

  //Translation 
	glRotatef(45,1,1,0);
  //..

  //Face 1
	glBegin(GL_QUADS);
	glColor3f(0.5,0.7,0.5);
	glVertex3f(0,2,2);
	glVertex3f(2,2,2);
	glVertex3f(2,0,2);
	glVertex3f(0,0,2);
	glEnd();
  //..


  // Face 2
	glBegin(GL_QUADS);
	glColor3f(0.5,0.7,0.5);
	glVertex3f(0,2,0);
	glVertex3f(2,2,0);
	glVertex3f(2,0,0);
	glVertex3f(0,0,0);
	glEnd();
  //..


  // Face 3
	glBegin(GL_QUADS);
	glColor3f(0.5,0.5,0.7);
	glVertex3f(0,0,0);
	glVertex3f(2,0,0);
	glVertex3f(2,0,2);
	glVertex3f(0,0,2);
	glEnd();
  //..


  // Face 4
	glBegin(GL_QUADS);
	glColor3f(0.5,0.5,0.7);
	glVertex3f(0,2,0);
	glVertex3f(2,2,0);
	glVertex3f(2,2,2);
	glVertex3f(0,2,2);
	glEnd();
  //..


  // Face 5
	glBegin(GL_QUADS);
	glColor3f(0.7,0.5,0.5);
	glVertex3f(0,0,0);
	glVertex3f(0,2,0);
	glVertex3f(0,2,2);
	glVertex3f(0,0,2);
	glEnd();
  //..


  // Face 4
  	glBegin(GL_QUADS);
	glColor3f(0.7,0.5,0.5);
	glVertex3f(2,0,0);
	glVertex3f(2,2,0);
	glVertex3f(2,2,2);
	glVertex3f(2,0,2);
	glEnd();
  //..		






  // Cube with colors

  //Translation 

  //..		

  //Face 1
  //..


  // Face 2
  //..


  // Face 3
  //..



  // Face 4
  //..


  // Face 5
  //..


  // Face 4
  //..



  // Cube with noramles

  //Translation 
  //..		

  //Face 1
  //..


  // Face 2
  //..


  // Face 3
  //..


  // Face 4
  //..

  // Face 5
  //..

  // Face 6
  //..


  // Cube with noramles

  //..		

 
  /* .. */
  glutSwapBuffers();
}

/* .. */
void keyPressed(unsigned char key, int x, int y) 
{

    if (key == ESCAPE) 
    { 
	/* .. */
	glutDestroyWindow(window); 

	/* .. */
	exit(0);                   
    }
}

/* .. */  
int main(int argc, char **argv) 
{
  /* .. */  
  glutInit(&argc, argv);

  /* init different buffers */  
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);  

  /* .. */
  glutInitWindowSize(640, 480);  

  /* .. */
  glutInitWindowPosition(0, 0);  

  /* .. */  
  window = glutCreateWindow("TD Informatique Graphique");  

  /* .. */
  glutDisplayFunc(&DrawGLScene);  

  /* .. */
  glutReshapeFunc(&ReSizeGLScene);

  /* .. */
  glutKeyboardFunc(&keyPressed);

  /* .. */
  InitGL(640, 480);
  
  /* .. */  
  glutMainLoop();  

  return 1;
}
