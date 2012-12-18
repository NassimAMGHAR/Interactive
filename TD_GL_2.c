////////////////////////////////
// 3D Animation 3D             //
// Univ. of Paris-Sud	      //  
// Mehdi AMMI - ammi@limsi.fr //
////////////////////////////////

#include <GL/glut.h>    // Header for GLUT 
#include <GL/gl.h>	// Header for OpenGL
#include <GL/glu.h>	// Header for GLu
#include <stdlib.h>     // Header for genral functions
#include <math.h>       // Header for math functions

/* light */

GLfloat light_position_1[] = {0.0f,20.0f,-15.0f, 0.0f};
GLfloat light_diffuse_1[] = {1.0, 1.0, 1.0, 0.0};

GLfloat light_position_2[] = {0.0f,30.0f,30.0f, 0.0f};
GLfloat light_diffuse_2[] = {1.0, 1.0, 1.0, 0.0};

/* material */

GLfloat mat_specularBLUE[] ={0.05,0.05,0.75,1.0};
GLfloat mat_ambientBLUE[] ={0,0,1,1.0};
GLfloat mat_diffuseBLUE[] ={0.50,0.50,0.50,1.0};
GLfloat mat_shininessBLUE[] ={128.0};

GLfloat mat_specularDarkBLUE[] ={0.05,0.05,0.75,1.0};
GLfloat mat_ambientDarkBLUE[] ={0,0,1,1.0};
GLfloat mat_diffuseDarkBLUE[] ={0.40,0.40,0.40,1.0};
GLfloat mat_shininessDarkBLUE[] ={128.0};

GLfloat mat_specularLightBLUE[] ={0.05,0.05,0.75,1.0};
GLfloat mat_ambientLightBLUE[] ={0,0,1,1.0};
GLfloat mat_diffuseLightBLUE[] ={0.90,0.90,0.90,1.0};
GLfloat mat_shininessLightBLUE[] ={128.0};


GLfloat mat_specularGREEN[] ={0.633, 0.727811, 0.633,1.0};
GLfloat mat_ambientGREEN[] ={0.1215, 0.2745, 0.1215,1.0};
GLfloat mat_diffuseGREEN[] ={0.27568, 0.31424, 0.27568,1.0};
GLfloat mat_shininessGREEN[] ={128.0};

GLfloat mat_specularYELLOW[] ={0.0,0.0,0.0,1.0};
GLfloat mat_ambientYELLOW[] ={1,0.7,0.,1.0};
GLfloat mat_diffuseYELLOW[] ={0.50,0.50,0.50,1.0};
GLfloat mat_shininessYELLOW[] ={128.0};

GLfloat mat_specularRED[] ={0.75,0.75,0.75,1.0};
GLfloat mat_ambientRED[] ={1.0,0.0,0.0,1.0};
GLfloat mat_diffuseRED[] ={0.8,0.50,0.50,1.0};
GLfloat mat_shininessRED[] ={128.0};

GLfloat mat_specularORANGE[] ={0.75,0.75,0.75,1.0};
GLfloat mat_ambientORANGE[] ={0.8,0.5,0.0,1.0};
GLfloat mat_diffuseORANGE[] ={1.0,0.5,0.0,1.0};
GLfloat mat_shininessORANGE[] ={128.0};


/* Parameters for DoF (degree of freedom )*/

float base_x = 0.0,base_y = 0.0,base_z = 0.0;
float base_rot = 0.0;
float first_x = 0.0,first_z = 20.0;
float second_x = 0.0,second_z = 20.0;
float third_x = 0.0,third_z = 20.0;
float finger_x = 0.0;

/* Camera paramters */

float up_down = 0.0, left_right = -1.57;
float cam_pos_x = 0.0;
float cam_pos_z = 30.0;
float cam_look_x = 0.0;
float cam_look_z = 0.0;
float vect_x = 0.0;
float vect_z = 0.0;

/* Animation parameters */

double rot_1 = 0.1;
double rot_2 = 0.1;
double rot_3 = 0.1;

/* Escape code ASCII*/

#define ESCAPE 27
#define SPACE 32

/* GLUT window ID*/

int window; 


/* Headers */

void Special_key(int key, int x, int y);
void Keyboard_key(unsigned char key, int x, int y);
void ground();
void SetMaterial(GLfloat spec[], GLfloat amb[], GLfloat diff[], GLfloat shin[]);
void base();
void first_arm();
void second_arm();
void third_arm();
void finger_1();
void finger_2();
void roue();

void move_camera(double speed);
void rotate_camera(double speed);


/* Init function*/
void InitGL(int Width, int Height)	        
{
// Clear color
  glClearColor(0.3f, 0.3f, 0.4f, 0.0f);		

// Z-buffer settings
  glClearDepth(1.0);	
  glDepthFunc(GL_LESS);	
  glEnable(GL_DEPTH_TEST);	

// Light activation
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0); 	
  glEnable(GL_LIGHT1); 	

// Light paramters
  glLightfv(GL_LIGHT0,GL_DIFFUSE,light_diffuse_1);
  glLightfv(GL_LIGHT0,GL_POSITION,light_position_1);

  glLightfv(GL_LIGHT1,GL_DIFFUSE,light_diffuse_2);
  glLightfv(GL_LIGHT1,GL_POSITION,light_position_2);

// Normalization of normales
  glEnable(GL_NORMALIZE);

// activation of smoothing
  glShadeModel(GL_SMOOTH);			

// Perceptive projection
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();				
  gluPerspective(45.0f,(GLfloat)Width/(GLfloat)Height,0.1f,1000.0f);	

  glMatrixMode(GL_MODELVIEW);

}

/* Reshape function */
void ReSizeGLScene(int Width, int Height)
{
  if (Height==0)				
    Height=1;

  glViewport(0, 0, Width, Height);		

// Perceptive projection
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();				
  gluPerspective(45.0f,(GLfloat)Width/(GLfloat)Height,0.1f,1000.0f);	

  glMatrixMode(GL_MODELVIEW);

}

/* Draw function */
void DrawGLScene()
{
// Clear of color and z-buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		
  glLoadIdentity();		


//////////////////////////////////////////////////
// cemera

gluLookAt(0.0,20.0,100.0,0.0,5.0,0.0,0.0,1.0,0.0); // 0.0,5.0,30.0

// Navigation 
//...

//////////////////////////////////////////////////
// Ground

  glPushMatrix();
  ground();
  glPopMatrix();

//////////////////////////////////////////////////
// The scene



//////////////////////////////////////////////////
// The robot

// The base
//..
	glTranslatef(base_x,1.0,base_z);
 	base();
	
// Articulated arm
//..
	
	
	glRotatef(first_x,1.0,0.0,0.0);
	glRotatef(first_z,0.0,0.0,1.0);
	first_arm();

	glTranslatef(0.0,4.0,0.0);
	glRotatef(second_x,1.0,0.0,0.0);
	glRotatef(second_z,0.0,0.0,1.0);
	second_arm();

	glTranslatef(0.0,4.0,0.0);
	glRotatef(third_x,1.0,0.0,0.0);
	glRotatef(third_z,0.0,0.0,1.0);
	third_arm();

	glTranslatef(0.0,4.0,0.0);	
	
		GLUquadricObj *q;
		q = gluNewQuadric();
		gluSphere(q, 0.6, 30, 30);

	glPushMatrix();
	glTranslatef(0.5,0.0,0.0);
	glRotatef(finger_x,0.0,0.0,1.0);
	finger_1();
	glPopMatrix();
	glTranslatef(-0.5,0.0,0.0);
	glRotatef(-finger_x,0.0,0.0,1.0);
	finger_2();
//////////////////////////////////////////////////


   glutSwapBuffers();
   glutPostRedisplay();
}

/*keyboard function for standard keys*/
void keyPressed(unsigned char key, int x, int y) 
{

    if (key == ESCAPE) 
    { 
	glutDestroyWindow(window); 

	exit(0);               
    }
}



/* IDLE function */
void idle_function()
{

double incr = 0.05;

// incrementation of rotation
//..


}


int main(int argc, char **argv) 
{
  
  glutInit(&argc, argv);

  /*  Activation of buffers  */  
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);  

  /* Creation of the window */
  glutInitWindowSize(1024, 765);  

  /* Position of the winfow */
  glutInitWindowPosition(200, 200);  

  /* Opening of the window */  
  window = glutCreateWindow("TD Animation 3D");  

  /* Draw function */
  glutDisplayFunc(DrawGLScene);  

  /* IDLE function */

  glutIdleFunc(idle_function);

  /* Reshape function */
  glutReshapeFunc(ReSizeGLScene);

  /* Keyboard function for standard keys */
  glutKeyboardFunc(Keyboard_key);

  /*  Keyboard function for special keys */

  glutSpecialFunc(Special_key);

  /* Init function */
  InitGL(640, 480);
  
  /* OpenGL loop */  
  glutMainLoop();  

  return 1;
}



void Special_key(int key, int x, int y) 
{

// Interaction with the base

  switch (key) 
  {

//	case ...:  		
//	...
//		break; 

//..
	case GLUT_KEY_RIGHT:	
	base_x +=0.5;
	break;
	case GLUT_KEY_LEFT:
	base_x -=0.5;
	break;
	case GLUT_KEY_UP:
	base_z -=0.5;
	break;
	case GLUT_KEY_DOWN:
	base_z +=0.5;
	break;
          
        default:
        break;
  }    



   glutSwapBuffers();
   glutPostRedisplay();
}

void Keyboard_key(unsigned char key, int x, int y)
{

  switch (key) 
  {    

///////////////////////////////////////////
// Interaction with the articulated arm 

//	case '...':  		 
//		...
//		...
//		break; 

//
//
//
//
 	case 'a':
	if(first_x < 45)
	first_x += 0.5;
	break;
	case 'q':
	if(first_x > -45)
	first_x -= 0.5;
	break;
	case 'z':
	if(first_z < 45)
	first_z += 0.5;
	break;
	case 's':
	if(first_x > -45)
	first_z -= 0.5;
	break;

	case 'e':
	if(second_x < 45)
	second_x += 0.5;
	break;
	case 'd':
	if(second_x > -45)
	second_x -= 0.5;
	break;
	case 'r':
	if(second_z < 45)
	second_z += 0.5;
	break;
	case 'f':
	if(second_x > -45)
	second_z -= 0.5;
	break;

	case 't':
	if(third_x < 45)
	third_x += 0.5;
	break;
	case 'g':
	if(third_x > -45)
	third_x -= 0.5;
	break;
	case 'y':
	if(third_z < 45)
	third_z += 0.5;
	break;
	case 'h':
	if(third_x > -45)
	third_z -= 0.5;
	break;

	case 'w':	
	finger_x = 0.0;
	break;
	case 'x':
	finger_x = -20.0;
	break;
		
        case ESCAPE :
               { 
	       glutDestroyWindow(window); 
	       exit(0);                   
               }

        default: 
        break;
  }    

   glutPostRedisplay();
   glutSwapBuffers();

}


void ground()
{

  glDisable(GL_LIGHTING);
  glColor3f(0.5,0.5,0.0);

	


        float i;
	for( i = -50; i <= 50; i += 5)
	{
		glBegin(GL_LINES);

			glVertex3f(-50, 0, i);
			glVertex3f(50, 0, i);

			glVertex3f(i, 0, -50);
			glVertex3f(i, 0, 50);

		glEnd();
	}

  glEnable(GL_LIGHTING);
}

void SetMaterial(GLfloat spec[], GLfloat amb[], GLfloat diff[], GLfloat shin[])
{
  glMaterialfv(GL_FRONT, GL_SPECULAR, spec);
  glMaterialfv(GL_FRONT, GL_SHININESS, shin);
  glMaterialfv(GL_FRONT, GL_AMBIENT, amb);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, diff);
}

void base()
{
	SetMaterial(mat_specularDarkBLUE, mat_ambientDarkBLUE, mat_diffuseDarkBLUE, mat_shininessDarkBLUE);

	glPushMatrix();

		glPushMatrix();

		glRotatef(90.0,1.0,0.0,0.0);

		GLUquadricObj *p;
		p = gluNewQuadric();
		gluCylinder(p, 3.0, 3.0,1.0, 30, 30);
		gluDisk(p, 0.0,3.0, 50, 5);
		glPopMatrix();

        	glTranslatef(0.0,0.0,0.0); 
		gluSphere(p, 1.0, 30, 30);

        	gluDeleteQuadric(p);

	glPopMatrix();
}


void first_arm()
{
//..
	//GLUquadricObj *p;
	//p = gluNewQuadric();
	glPushMatrix();	

		glTranslatef(0.0,2.0,0.0);
		
		glScalef(1.2,4.0,1.2);

		glutSolidCube(1); 
	
	glPopMatrix();

}



void second_arm()
{
//..
	glPushMatrix();	

		GLUquadricObj *p;
		p = gluNewQuadric();
		gluSphere(p, 1.0, 30, 30);

		glTranslatef(0.0,2.0,0.0);
		
		glScalef(0.8,4.0,0.8);

		glutSolidCube(1); 
	
	glPopMatrix();

}




void third_arm()
{
//..
	glPushMatrix();	

		GLUquadricObj *p;
		p = gluNewQuadric();
		gluSphere(p, 0.8, 30, 30);

		glTranslatef(0.0,2.0,0.0);
		
		glScalef(0.6,4.0,0.6);

		glutSolidCube(1); 
	
	glPopMatrix();	
		
}



void finger_1()
{
//..
	glPushMatrix();	

		glTranslatef(0.0,0.5,0.0);
		
		glRotatef(-32.0,0.0,0.0,1.0);

		glScalef(0.2,1.0,0.2);

		glutSolidCube(1);

	
	glPopMatrix();

	glPushMatrix();	

		glTranslatef(0.0,1.5,0.0);

		GLUquadricObj *p;
		p = gluNewQuadric();
		gluSphere(p, 0.3, 30, 30);
	
		glRotatef(32.0,0.0,0.0,1.0);
		
		glScalef(0.2,1.0,0.2);

		glutSolidCube(1);
	glPopMatrix();
}


void finger_2()
{
//..
	glPushMatrix();	

		glTranslatef(0.0,0.5,0.0);
		
		glRotatef(32.0,0.0,0.0,1.0);

		glScalef(0.2,1.0,0.2);

		glutSolidCube(1);

	
	glPopMatrix();

	glPushMatrix();			

		glTranslatef(0.0,1.5,0.0);

		GLUquadricObj *p;
		p = gluNewQuadric();
		gluSphere(p, 0.3, 30, 30);
	
		glRotatef(-32.0,0.0,0.0,1.0);
		
		glScalef(0.2,1.0,0.2);

		glutSolidCube(1);
	glPopMatrix();
}



void roue()
{
	glPushMatrix();

	SetMaterial(mat_specularGREEN, mat_ambientGREEN, mat_diffuseGREEN, mat_shininessGREEN);

	GLUquadricObj *p;
	p = gluNewQuadric();
	gluCylinder(p, 1.0, 1.0,1.0, 10, 10);
	gluDisk(p, 0.0,1.0, 10, 5);
        gluDeleteQuadric(p);

	glPopMatrix();
}


void move_camera(double speed)
{
//..

}


void rotate_camera(double speed)
{
//..

}
