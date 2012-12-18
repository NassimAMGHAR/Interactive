////////////////////////////////
// 2007			      //
// TD Informatique Graphique  //
// Université Paris 11	      //  
// Mehdi AMMI - ammi@limsi.fr //
////////////////////////////////

#include <GL/glut.h>    // Header pour GLUT 
#include <GL/gl.h>	// Header pour OpenGL
#include <GL/glu.h>	// Header pour GLu
#include <stdlib.h>     // Header utilitaire général
#include <math.h>       // Header pour les fonctions mathèmatiques
#include <stdio.h>      
/* Définition des paramètes dynamiques */
#define G 9.8 // m/s^2
#define B 0.8 // Damping


/* Source de lumière */
GLfloat light_diffuse[] = {1.0, 1.0, 1.0, 0.0};   /*  composante diffuse */
GLfloat light_position[] = {0.0,10.0,10.0, 0.0};  /*  position */


/* Definitions des matériaux */
GLfloat mat_specularWHITE[] ={255.0,255.0,255.0,1.0};
GLfloat mat_ambientWHITE[] ={255.0,255.0,255.0,1.0};
GLfloat mat_diffuseWHITE[] ={255.0,255.0,255.0,1.0};
GLfloat mat_shininessWHITE[] ={128.0 * 0.4};

GLfloat mat_specularGRAY[] ={0.75,0.75,0.75,1.0};
GLfloat mat_ambientGRAY[] ={0.5,0.5,0.5,1.0};
GLfloat mat_diffuseGRAY[] ={0.50,0.50,0.50,1.0};
GLfloat mat_shininessGRAY[] ={128.0 * 0.6};

GLfloat mat_specularBLUE[] ={0.75,0.75,0.75,1.0};
GLfloat mat_ambientBLUE[] ={0,0,1,1.0};
GLfloat mat_diffuseBLUE[] ={0.50,0.50,0.50,1.0};
GLfloat mat_shininessBLUE[] ={128.0 };

GLfloat mat_specularGREEN[] ={0.633, 0.727811, 0.633,1.0};
GLfloat mat_ambientGREEN[] ={0.0215, 0.1745, 0.0215,1.0};
GLfloat mat_diffuseGREEN[] ={0.07568, 0.61424, 0.07568,1.0};
GLfloat mat_shininessGREEN[] ={128.0};

GLfloat mat_specularYELLOW[] ={0.75,0.75,0.75,1.0};
GLfloat mat_ambientYELLOW[] ={1,1,0,1.0};
GLfloat mat_diffuseYELLOW[] ={0.50,0.50,0.50,1.0};
GLfloat mat_shininessYELLOW[] ={128.0};

GLfloat mat_specularRED[] ={0.75,0.75,0.75,1.0};
GLfloat mat_ambientRED[] ={1.0,0.0,0.0,1.0};
GLfloat mat_diffuseRED[] ={0.8,0.50,0.50,1.0};
GLfloat mat_shininessRED[] ={128.0};

/* Variables d'animation */
float up_down = 0.0, left_right = -1.57;
float cam_pos_x = 0.0;
float cam_pos_z = 50.0;
float cam_look_x = 0.0;
float cam_look_z = 0.0;
float vect_x = 0.0;
float vect_z = 0.0;
double d = 30;

// Postion de la sphère : t & t-1
float sphere_x = -20.0;
float sphere_y = 20.0;
float sphere_z = 0.0;

float sphere_ix = -20.0;
float sphere_iy = 20.0;
float sphere_iz = 0.0;

// Vitesse de la sphère : t & t-1
float sphere_vx;
float sphere_vy;
float sphere_vz;

float sphere_ivx = 10.0;
float sphere_ivy = 0.0;
float sphere_ivz = 0.0;

// accélération de la sphère
float sphere_ax;
float sphere_ay;
float sphere_az;

float sphere_iax = 0.0;
float sphere_iay = -G;
float sphere_iaz = 0.0;

// Pas de temps
double dt = 0.02;

/* code ASCII pour la touche escape*/
#define ESCAPE 27

/* Idantifiant de la fenêtre GLUT */
int window; 

/* Déclaration des en-têtes */

void Draw3DSGrid();
void rotate_camera(double speed);
void move_camera(double speed);


/* Fonction d'initialisation */
void InitGL(int Width, int Height)	        
{
// Couleur d'effacement du buffer de couleur
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);		

// paramètrage du Z-buffer
  glClearDepth(1.0);	
  glDepthFunc(GL_LESS);	
  glEnable(GL_DEPTH_TEST);	

// Activation de l'éclairage
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);

  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHTING);


// Activation du lissage
  glShadeModel(GL_SMOOTH);			

// Projection perceptive
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();				
  gluPerspective(45.0f,(GLfloat)Width/(GLfloat)Height,0.1f,100.0f);	

  glMatrixMode(GL_MODELVIEW);

}

/* Fonction de redimensionnement de la fenêtre */
void ReSizeGLScene(int Width, int Height)
{
  if (Height==0)				
    Height=1;

  glViewport(0, 0, Width, Height);		

// Projection perceptive
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();				
  gluPerspective(45.0f,(GLfloat)Width/(GLfloat)Height,0.1f,100.0f);	

  glMatrixMode(GL_MODELVIEW);

}

/* Focntion de dessin */
void DrawGLScene()
{
// Effacement du buffer de couleur et de profondeur
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		
  glLoadIdentity();		
  gluLookAt(cam_pos_x,10.0,cam_pos_z,cam_look_x,5.0,cam_look_z,0.0,1.0,0.0); 


// Dessin de la grille
  glPushMatrix();
  Draw3DSGrid();
  glPopMatrix();


//////////////////////////////////////////////////
// Dessin de la sphere
  glTranslatef(sphere_x,sphere_y,sphere_z);
  glutSolidSphere(3,40,50);
//..

//////////////////////////////////////////////////


// Permutation des buffers
   glutPostRedisplay();
   glutSwapBuffers();
}

/* Fonction de gestion du clavier */
void keyPressed(unsigned char key, int x, int y) 
{

    if (key == ESCAPE) 
    { 
	/* Eteindre la fenêtre */
	glutDestroyWindow(window); 

	/* Sortire du programme */
	exit(0);                   
    }
}



/* Fonction de gestion du clavier spetial */
void Special_key(int key, int x, int y) 
{

  switch (key) 
  {
	case GLUT_KEY_RIGHT:  		
		left_right+= 0.03;
		rotate_camera(left_right);
		break; 

	case GLUT_KEY_LEFT:  		
		left_right-= 0.03;
		rotate_camera(left_right);
		break; 
	
	case GLUT_KEY_UP:  		 
		move_camera(0.02);
		break; 
        
 	case GLUT_KEY_DOWN:  		    
		move_camera(-0.02);
		break;    

  default:
    break;
  }   

   glutPostRedisplay();
   glutSwapBuffers();

}


/* Encapsulation des fonctions matériaux */
void SetMaterial(GLfloat spec[], GLfloat amb[], GLfloat diff[], GLfloat shin[])
{
  glMaterialfv(GL_FRONT, GL_SPECULAR, spec);
  glMaterialfv(GL_FRONT, GL_SHININESS, shin);
  glMaterialfv(GL_FRONT, GL_AMBIENT, amb);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, diff);
}


/* Dessin d'une grille 3D */
void Draw3DSGrid()
{

	SetMaterial(mat_specularYELLOW, mat_ambientYELLOW, mat_diffuseYELLOW, mat_shininessYELLOW);

	// Draw a 1x1 grid along the X and Z axis'
        float i;
	for( i = -50; i <= 50; i += 5)
	{
		// Start drawing some lines
		glBegin(GL_LINES);

			// Do the horizontal lines (along the X)
			glVertex3f(-50, 0, i);
			glVertex3f(50, 0, i);

			// Do the vertical lines (along the Z)
			glVertex3f(i, 0, -50);
			glVertex3f(i, 0, 50);

		// Stop drawing lines
		glEnd();
	}
}



/* Gestion du déplacement de la caméra  */
void move_camera(double speed)
{
	vect_x = cam_look_x - cam_pos_x;
	vect_z = cam_look_z - cam_pos_z;

	cam_pos_x += vect_x * speed;
	cam_pos_z += vect_z * speed;

	cam_look_x += vect_x * speed;
	cam_look_z += vect_z * speed;
}

/* Gestion de l'orientation de la caméra */
void rotate_camera(double speed)
{
	vect_x = cam_look_x - cam_pos_x;
	vect_z = cam_look_z - cam_pos_z;

	// Calculate the sine and cosine of the angle once
	float cosTheta = (float)cos(speed);
	float sinTheta = (float)sin(speed);
 
	double new_x = sqrt(pow(vect_x,2) + pow(vect_z,2)) * cosTheta;
	double new_z = sqrt(pow(vect_x,2) + pow(vect_z,2)) * sinTheta;

	cam_look_x = cam_pos_x + new_x;
	cam_look_z = cam_pos_z + new_z;
}

/* Défintion de la fonction IDLE */
void idle_function()
{

	////////////////////////////////////
	// Numerical integration
	
	sphere_vx = sphere_ivx + sphere_iax * dt;
	sphere_vy = sphere_ivy + sphere_iay * dt;
	sphere_vz = sphere_ivz + sphere_iaz * dt;

	sphere_x = sphere_ix + sphere_vx * dt;
	sphere_y = sphere_iy + sphere_vy * dt;
	sphere_z = sphere_iz + sphere_vz * dt;

//..



	////////////////////////////////////
	// Collision test
	float distance = (sphere_y - 3/* radius */);

	if (distance < 0){

		sphere_vy = -sphere_vy * 0.8;
		//sphere_vy = -sphere_vy;
		sphere_y = 3.0;

	}


//..



	////////////////////////////////////
	// Acceleration calculation
	
	sphere_ax = 0 - B*sphere_vx; 
	sphere_ay = -G - B*sphere_vy;
	sphere_az = 0 - B*sphere_vz;
//..



	////////////////////////////////////
	// System update

	sphere_ix = sphere_x;
	sphere_iy = sphere_y;
	sphere_iz = sphere_z;
	
	sphere_ivx = sphere_vx;
	sphere_ivy = sphere_vy;
	sphere_ivz = sphere_vz;

	sphere_iax = sphere_ax;
	sphere_iay = sphere_ay;
	sphere_iaz = sphere_az;

	glutPostRedisplay();
	//glutSwapBuffers();

//..



	////////////////////////////////////
}

int main(int argc, char **argv) 
{
  /* Initialize GLUT state - glut will take any command line arguments that pertain to it or 
     X Windows - look at its documentation at http://reality.sgi.com/mjk/spec3/spec3.html */  
  glutInit(&argc, argv);

  /*  Activation des buffers :   
     Double buffer 
     RGBA color
     Alpha
     Depth buffer */  
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);  

  /* Création de la fenêtre */
  glutInitWindowSize(640, 480);  

  /* Positionnement de la fenêtre */
  glutInitWindowPosition(0, 0);  

  /* Ouverture de la fenêtre */  
  window = glutCreateWindow("TD Animation 3D");  

  /* Spécification de la fontion de dessin */
  glutDisplayFunc(&DrawGLScene);  

  /* Spécification de la routine de fond */

  glutIdleFunc(idle_function);

  /* Spécification de la fontion de redimensionnement */
  glutReshapeFunc(&ReSizeGLScene);

  /* Spécification de la fontion de de gestion du clavier */
  glutKeyboardFunc(&keyPressed);

  /* Spécification de la fontion special de gestion du clavier */
  glutSpecialFunc(Special_key);

  /* Intitialisation des paramètres de l'affichage et de la fenêtre */
  InitGL(640, 480);
  
  /* Lancement de la boucle OpenGL */  
  glutMainLoop();  

  return 1;
}
