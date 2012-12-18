////////////////////////////////
// 2008			      //
// TD Animation 3D	      //
// Université Paris 11	      //  
// Mehdi AMMI - ammi@limsi.fr //
//////////////////////////////// 
    
#include <sys/time.h>
#include <GL/gl.h>
#include <GL/glut.h>#include "kinematic.cpp"
   
 
/* ############################################################################################################################
   # Declaration des variables
   ############################################################################################################################ */
 
int   MouseState=0;                                // Etat de la souris
      
float fov=45.0;                                    // Ouverture de l'angle de vue
float near_plane=10;                               // Plan proche et éloigné
float far_plane=500.0;   
                      	   
int   WindowWidth;                                 // Taille de la fenètre d'affichage
int   WindowHeight;
 
int keyUp		 = 0;                      // Etats des boutons clavier
int keyDown		 = 0;
int keyLeft		 = 0;
int keyRight	 = 0;
int keyPgUp		 = 0;
int keyPgDown	 = 0;

int selected = -1;                                 // Etat de la sélection
 
/* ############################################################################################################################
   # En-têtes des fonctions
   ############################################################################################################################ */

void MakeCallLists();
void GLInit(void);
void ReLoad();
void motion(int x, int y);
void mouse(int button, int state, int x, int y);
int PickLink(int x, int y);
GLuint GetNearestHitName(GLint hits, GLuint buffer[]);
void DisplayHits (GLint hits, GLuint buffer[]);
void keyboardSpecUp(int key, int x, int y);
void keyboardUp(unsigned char key, int x, int y);
void keyboardSpecDown(int key, int x, int y);
void keyboardDown(unsigned char key, int x, int y);
void HelpMessage();
void InfoMessage();
void timeTick(void);
double getTime(void);  
void display(void);
void reshape(int w, int h);
void DrawEEOrientation();
void DrawSkelet(void);
void DrawEE ();
void MoveToOrigin();
void SetEEOrientView(int w, int h);
void SetGlobalView(int w, int h);
MDBool IsEEOrientViewClick(int w, int h, int x, int y);
void Draw3DSGrid();
void DirectKinematics(double deltaT);

/* ############################################################################################################################
   # Fonction principale
   ############################################################################################################################ */

int main (int argc, char **argv)
{

/* **************************************************************************************************************************** */

 	/* Pointeurs vers l'application */
	glutInit(&argc, argv);         

  	/*  Activation des buffers :   
     	Double buffer 
     	RGBA color
     	Depth buffer */               
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH); 

  	/* Création de la fenêtre */
	glutInitWindowSize(1024, 768);

	/* Positionnement de la fenêtre */
	glutInitWindowPosition(100, 100);

	/* Ouverture de la fenêtre */  
	glutCreateWindow("TD_Animation_3D");

	/* Spécification de la fontion de dessin */
	glutDisplayFunc(display);

  	/* Spécification de la fontion de redimensionnement */
	glutReshapeFunc(reshape);

  	/* Spécification des fontions de gestion du clavier */
	glutKeyboardFunc(keyboardDown);
	glutKeyboardUpFunc(keyboardUp);
	glutSpecialFunc(keyboardSpecDown);
	glutSpecialUpFunc(keyboardSpecUp);

 	/* Spécification des fontions de gestion de la souris */
	glutMouseFunc(mouse);
	glutMotionFunc(motion);

 	/* Spécification de la fonction de mise-à-jour */
	glutIdleFunc(timeTick);

/* **************************************************************************************************************************** */

	/* Affichage des fonctions clavier */
	HelpMessage();

/* **************************************************************************************************************************** */

	/* Intitialisation des paramètres de l'affichage et de la fenêtre */
	GLInit();

	/* Intitialisation des paramètres du squelette */ 
	SkeletInit();

	/* Intitialisation de la scène cinématique */
	IKInit();

/* **************************************************************************************************************************** */

  	/* Lancement de la boucle OpenGL */  
	glutMainLoop();

/* **************************************************************************************************************************** */

	return 0;

/* **************************************************************************************************************************** */
}


     	  
/* ############################################################################################################################
   # Intitialisation des paramètres de l'affichage et de la fenêtre
   ############################################################################################################################ */

void GLInit(void)
{
/* **************************************************************************************************************************** */
	glClearColor(0.2, 0.2, 0.2, 0.0);
	glShadeModel(GL_SMOOTH);
	glClearDepth(1.0f);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);
	glLightModeli(GL_LIGHT_MODEL_COLOR_CONTROL, GL_SEPARATE_SPECULAR_COLOR);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

	glPointSize(5);

	glEnable(GL_NORMALIZE);
/* **************************************************************************************************************************** */

	LinkList	 = glGenLists(4);
	JointList	 = LinkList+1;
	EEList		 = JointList+1;
	AxesList	 = EEList+1;

	MakeCallLists();
/* **************************************************************************************************************************** */
} 



/* ############################################################################################################################
   # Fonction principale de dessin
   ############################################################################################################################ */

void display(void) 
{
/* **************************************************************************************************************************** */

        /* Effacement du buffer de couleur et de profondeur */
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

/* **************************************************************************************************************************** */

	/* Positionnement du point de vue initial */
	SetGlobalView(WindowWidth, WindowHeight);

/* **************************************************************************************************************************** */

        /* Dessin du squelette */
	DrawSkelet();

/* **************************************************************************************************************************** */

	/* Dessin de la grille */
	Draw3DSGrid();

/* **************************************************************************************************************************** */

	/* Dessin des axes */
	glCallList(AxesList);

/* **************************************************************************************************************************** */

	/* Dessin de l'orientation de l'effecteur final */
	DrawEEOrientation();

/* **************************************************************************************************************************** */

	/* Permutation des buffers et mise-à-jour */
	glFlush();         
	glutSwapBuffers(); 

/* **************************************************************************************************************************** */
}


/* ############################################################################################################################
   # Fonction de redimensionnement
   ############################################################################################################################ */

void reshape(int w, int h) 
{
/* **************************************************************************************************************************** */
	WindowWidth  = w;                         
	WindowHeight = h;
/* **************************************************************************************************************************** */
}
  


/* ############################################################################################################################
   # Dessin d'une grille 3D
   ############################################################################################################################ */

void Draw3DSGrid()
{

	glColor3f(0.0, 0.0,0.0);

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

 
/* ############################################################################################################################
   # Fonction clavier : boutons principaux (intercation avec la fenètre et la scène)
   ############################################################################################################################ */

void keyboardDown(unsigned char key, int x, int y) 
{
/* **************************************************************************************************************************** */
	if (key>='A' && key<='Z')                   
		key+='a'-'A';
/* **************************************************************************************************************************** */
	switch (key) {                              
	case 13:                                  
		FindConfiguration();
		break;
	case 'f':
		glutFullScreen();                       
		break;
	case 'h':
		HelpMessage();                       
		break;
	case 'i':
		InnerIterate();
		break;
	case 'l':
		AngleLimits	 = !AngleLimits;
		if (AngleLimits)
			fprintf(stderr, "Angle Limits: ON\n");
		else
			fprintf(stderr, "Angle Limits: OFF\n");
		break;
	case 27:                                 
	case 'q':
	case 'x':
		exit(0);                               
		break;
	case 'r':
		ReLoad();
		break;
	case 'w':
		glutReshapeWindow(1024, 768);            
		glutPositionWindow(100, 100);
		break;
	default:
		break;
	}
/* **************************************************************************************************************************** */
}


/********************************************************************/
/* Fonction clavier : boutons spéciaux 	(ddl du robot / activé)	    */
/********************************************************************/

void keyboardSpecDown(int key, int x, int y) 
{
/* **************************************************************************************************************************** */
  switch (key) {                            
    case GLUT_KEY_LEFT:
		keyLeft		 = 1;
		break;
    case GLUT_KEY_RIGHT:
		keyRight	 = 1;
		break;
    case GLUT_KEY_UP:
		keyUp		 = 1;
		break;
    case GLUT_KEY_DOWN:
		keyDown		 = 1;
		break;
    case GLUT_KEY_PAGE_UP:
		keyPgUp		 = 1;
		break;
    case GLUT_KEY_PAGE_DOWN:
		keyPgDown	 = 1;
		break;
    default:
		break;
  }
/* **************************************************************************************************************************** */
}


/********************************************************************/
/* Fonction de convertion majuscule -> minuscule                    */
/********************************************************************/
void keyboardUp(unsigned char key, int x, int y) 
{
/* **************************************************************************************************************************** */
  if (key>='A' && key<='Z')                   
    key+='a'-'A';
/* **************************************************************************************************************************** */
}


/********************************************************************/
/* Fonction clavier : boutons spéciaux 	(ddl du robot / désactivé)  */
/********************************************************************/

void keyboardSpecUp(int key, int x, int y) 
{
/* **************************************************************************************************************************** */
  switch (key) {                             
    case GLUT_KEY_LEFT:
		keyLeft		 = 0;
		break;
    case GLUT_KEY_RIGHT:
		keyRight	 = 0;
		break;
    case GLUT_KEY_UP:
		keyUp		 = 0;
		break;
    case GLUT_KEY_DOWN:
		keyDown		 = 0;
		break;
    case GLUT_KEY_PAGE_UP:
		keyPgUp		 = 0;
		break;
    case GLUT_KEY_PAGE_DOWN:
		keyPgDown	 = 0;
		break;
    default:
		break;
  }
/* **************************************************************************************************************************** */
}


/* ############################################################################################################################
   # Fonction souris : boutons activés
   ############################################################################################################################ */

void mouse(int button, int state, int x, int y) 
{
/* **************************************************************************************************************************** */
	int		Modifiers;
/* **************************************************************************************************************************** */
	Modifiers	 = glutGetModifiers();
/* **************************************************************************************************************************** */
	if (Modifiers == GLUT_ACTIVE_CTRL) { 
		if (button==GLUT_LEFT_BUTTON) { 
			if (state==GLUT_UP) { 
				MouseState	 = 0;
				glutPostRedisplay();
			} else {

				selected	 = PickLink(x, y);

				int i;
				for (i=0; i<_NO_OF_LINKS_; i++) Skelet[i].UnSelect();

				if (selected < _NO_OF_LINKS_) Skelet[selected].Select();
			
				glMatrixMode(GL_PROJECTION);              
				glPopMatrix();

				GLdouble	 MViewMatrix[4][4];

				if ( (selected == _DESIRED_EE_ID_) || (selected == _EE_ID_) ) {

					SetGlobalView(WindowWidth, WindowHeight);
					MoveToOrigin();

					glGetDoublev (GL_MODELVIEW_MATRIX, (MDFloat*)MViewMatrix);

					GetXYPlaneEqn(MViewMatrix, PNew, &a, &b, &c, &d);
					MouseState	 = 4;
				}

			}
		}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
	} else if (Modifiers == GLUT_ACTIVE_SHIFT) { 
		if (IsEEOrientViewClick(WindowWidth, WindowHeight, x, y)) {
			if (state==GLUT_DOWN) {
				MouseState	 = 6;
				xx1			 = x;       
				xold		 = (int)ONew[1];   
			}
			else {           
				MouseState	 = 0;
			}
		} else if (button==GLUT_LEFT_BUTTON) { 
			if (state==GLUT_UP) { 
				MouseState	 = 0;
				glutPostRedisplay();
			} else {

				selected	 = PickLink(x, y);

				int i;
				for (i=0; i<_NO_OF_LINKS_; i++) Skelet[i].UnSelect();

				if (selected < _NO_OF_LINKS_) Skelet[selected].Select();
			

				glMatrixMode(GL_PROJECTION);             
				glPopMatrix();

				GLdouble	 MViewMatrix[4][4];

				if ( (selected == _DESIRED_EE_ID_) || (selected == _EE_ID_) ) {

					SetGlobalView(WindowWidth, WindowHeight);
					MoveToOrigin();
					
					glGetDoublev (GL_MODELVIEW_MATRIX, (MDFloat*)MViewMatrix);

					GetXYPlaneEqn(MViewMatrix, PNew, &a, &b, &c, &d);
					MouseState	 = 3;
				}

			}
		}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
	} else {
		if (IsEEOrientViewClick(WindowWidth, WindowHeight, x, y)) {
			if (state==GLUT_DOWN) {
				MouseState	 = 7;
				xold		 = (int)ONew[2];   
				xx1			 = x;       
			}
			else {           
				MouseState=0;
			}
		} else {
			if (button==GLUT_LEFT_BUTTON) {             
				if (state==GLUT_DOWN) {                
					MouseState=1;                           
					xold=xnew;                       
					yold=ynew;
					xx1=x;                              
					yy1=y;
				}
				else {                                
					MouseState=0;                           
				}
			}

			if (button==GLUT_RIGHT_BUTTON) {          
				if (state==GLUT_DOWN) {                
					MouseState=2;                           
					zold=znew;                         
					zz1=y;                              
				}
				else {
					MouseState=0;
				}
			}
		}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
	}
/* **************************************************************************************************************************** */
  glutPostRedisplay();                       
/* **************************************************************************************************************************** */
}


/* ############################################################################################################################
   # Fonction souris : déplacement
   ############################################################################################################################ */

void motion(int x, int y) 
{
/* **************************************************************************************************************************** */
	int			 TempRes;
	MDFloat		 t;
	GLint		 ViewPort[4];
	MDFloat		 Q[4];
	MDFloat		 P[4];
	MDFloat		 s[4];
	GLdouble	 MM[16];
	GLdouble	 PM[16];
/* **************************************************************************************************************************** */
    if (MouseState==1) {                           
        xnew=xold+x-xx1;                         
        ynew=yold+y-yy1;
        glutPostRedisplay();                    
    }
/* **************************************************************************************************************************** */
    if (MouseState==2) {                              
        znew=zold+y-zz1;                         
        glutPostRedisplay();                    
    }
/* **************************************************************************************************************************** */
    if ( (MouseState==3) || (MouseState==4) ) {

		SetGlobalView(WindowWidth, WindowHeight);
		MoveToOrigin();

		glGetDoublev(GL_MODELVIEW_MATRIX, MM);
		glGetDoublev(GL_PROJECTION_MATRIX, PM);
		glGetIntegerv (GL_VIEWPORT, ViewPort);

		/*
		 * Obtain 2 points on line going from cursor
		 */

		TempRes	 = gluUnProject( x, ViewPort[3]-y, -1/*z*/,
			MM, PM, ViewPort,
			&Q[0], &Q[1], &Q[2] );
		TempRes	 = gluUnProject( x, ViewPort[3]-y, 0/*z*/,
			MM, PM, ViewPort,
			&P[0], &P[1], &P[2] );

		/* Find directional vector of this line */
		VectSub(P, Q, s);

		/* Find intersection with plane in Selected node paralel with projection plane */
		t	 = (-a*Q[0] - b*Q[1] - c*Q[2] - d) / (a*s[0] + b*s[1] + c*s[2]);
		PNew[0]	 = Q[0] + t*s[0];
		PNew[1]	 = Q[1] + t*s[1];
		PNew[2]	 = Q[2] + t*s[2];

		if (MouseState==3) FindConfiguration();

		glutPostRedisplay();

    }
/* **************************************************************************************************************************** */
    if (MouseState==6) {   
        ONew[1]	 = xold+x-xx1; 
        if (ONew[1]>= 75)	ONew[1]	 =  74.99;
        if (ONew[1]<=-75)	ONew[1]	 = -74.99;
        glutPostRedisplay();
    }
    if (MouseState==7) {   
        ONew[2]	 = xold+x-xx1; 
        if (ONew[2]>= 180)	ONew[2]	 =  179.99;
        if (ONew[2]<=-80)	ONew[2]	 = -79.99;
        glutPostRedisplay();
    }
/* **************************************************************************************************************************** */
}



/* ############################################################################################################################
   # Retourne le temps actuel en [sec]
   ############################################################################################################################ */

double getTime(void) 
{
/* **************************************************************************************************************************** */
  // enregistrement du temps actuel
  struct timeval tv;
  if (gettimeofday(&tv, NULL) == -1) {        
    assert(0 && "gettimeofday does not work.");  
    exit(-1);
  }
  // revois du temps
  return (double)tv.tv_sec + (double)tv.tv_usec/1000000.;  

/* **************************************************************************************************************************** */
}


/* ############################################################################################################################
   # Fonction de mise-à-jour
   ############################################################################################################################ */

void timeTick(void)
{
/* **************************************************************************************************************************** */
	static double lastTime = 0.;                
	double deltaT;                              
/* **************************************************************************************************************************** */
	// calcul du pas
	if (lastTime == 0.) {                      
		lastTime = getTime();                    
		deltaT = 0.;
	} else {                                    
		double currentTime = getTime();          
		deltaT = currentTime - lastTime;          
		lastTime = currentTime;                   
	}
/* **************************************************************************************************************************** */
	// mise-à-jour du squelette : 

	//..

/* **************************************************************************************************************************** */
	glutPostRedisplay();                        
/* **************************************************************************************************************************** */
}


/* ############################################################################################################################
   #  Fonction de calcul de la cinématique directe
   ############################################################################################################################ */


void DirectKinematics(double deltaT)
{

// ..


}


/* ############################################################################################################################
   #  Consturuction des listes des elements de la scène
   ############################################################################################################################ */


void MakeCallLists() 
{

GLUquadricObj	*gluLinkObj;
GLUquadricObj	*gluJointObj;
GLUquadricObj	*gluAxesObj;

gluLinkObj	= gluNewQuadric();
gluJointObj	= gluNewQuadric();
gluAxesObj	= gluNewQuadric();

/* **************************************************************************************************************************** */
	// Axes
	glNewList(AxesList, GL_COMPILE);
		glMatrixMode(GL_MODELVIEW);

		glPushMatrix();
		glColor3f(1, 0, 0);
		glTranslatef(-50.0, 0, 0);
		glScalef(100, 0.5, 0.5);
		glRotatef(90, 0, 1, 0);
		gluCylinder(gluAxesObj, 0.5, 0.5, 1.0, 15, 15);
		glPopMatrix();
		glPushMatrix();
		glTranslatef(50.0+3, 0, 0);
		glRotatef(-90, 0, 1, 0);
		gluCylinder(gluAxesObj, 0.0, 1.5, 3.0, 15, 15);
		glPopMatrix();

		glPushMatrix();
		glColor3f(0, 1, 0);
		glTranslatef(0.0, 50.0, 0);
		glScalef(0.5, 100.0, 0.5);
		glRotatef(90, 0, 1, 0);
		glRotatef(90, 1, 0, 0);
		gluCylinder(gluAxesObj, 0.5, 0.5, 1.0, 15, 15);
		glPopMatrix();
		glPushMatrix();
		glTranslatef(0.0, 50.0+3, 0);
		glRotatef(90, 0, 1, 0);
		glRotatef(90, 1, 0, 0);
		gluCylinder(gluAxesObj, 0.0, 1.5, 3.0, 15, 15);
		glPopMatrix();

		glPushMatrix();
		glColor3f(0, 0, 1);
		glTranslatef(0.0, 0.0, -50.0);
		glScalef(0.5, 0.5, 100.0);
		gluCylinder(gluAxesObj, 0.5, 0.5, 1.0, 15, 15);
		glPopMatrix();
		glPushMatrix();
		glTranslatef(0.0, 0.0, 50.0+3);
		glRotatef(180, 1, 0, 0);
		gluCylinder(gluAxesObj, 0.0, 1.5, 3.0, 15, 15);
		glPopMatrix();
 
	glEndList();
/* **************************************************************************************************************************** */
	// Segment
	glNewList(LinkList, GL_COMPILE);
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glRotatef(90, 0, 1, 0);
		gluCylinder(gluLinkObj, 1.0, 1.0, 1.0, 30, 30);
		glPopMatrix();
	glEndList();
/* **************************************************************************************************************************** */
	// Articulation
	glNewList(JointList, GL_COMPILE);
		glMatrixMode(GL_MODELVIEW);
		gluSphere(gluJointObj, 1.0, 30, 30);
	glEndList();
/* **************************************************************************************************************************** */
	// Effecteur final
	glNewList(EEList, GL_COMPILE);
		glPushMatrix();
		glTranslatef(1*thickness, 0, 0);

		glPushMatrix();
		glScalef(2*thickness, thickness/3, thickness/2);
		glutSolidCube(1);
		glPopMatrix();

		glTranslatef(1*thickness+thickness/8, 0, 0);

		glPushMatrix();
		glScalef(thickness/4, 2*thickness, 2*thickness);
		glutSolidCube(1);
		glPopMatrix();

		glPushMatrix();
		glTranslatef(thickness/2 + thickness/8, 0, 0);
		glTranslatef(0, thickness - thickness/8, 0);
		glScalef(thickness, thickness/4, 2*thickness);
		glutSolidCube(1);
		glPopMatrix();
		
		glPushMatrix();
		glTranslatef(thickness/2 + thickness/8, 0, 0);
		glTranslatef(0, -thickness + thickness/8, 0);
		glScalef(thickness, thickness/4, 2*thickness);
		glutSolidCube(1);
		glPopMatrix();
		
		glPopMatrix();
	glEndList();
/* **************************************************************************************************************************** */
}





/* ############################################################################################################################
   # Initialiser le squelette
   ############################################################################################################################ */

void ReLoad() {
/* **************************************************************************************************************************** */
	xnew=0;
	ynew=0;
	znew=-100;
	xold=0;
	yold=0;
	zold=-100;
	MouseState=0;   
/* **************************************************************************************************************************** */
	SkeletInit();
	IKInit();
/* **************************************************************************************************************************** */
}


/* ############################################################################################################################
   # 
   ############################################################################################################################ */

void SetGlobalView(int w, int h) {
/* **************************************************************************************************************************** */
	glViewport(0, 0, w, h);                     
	glMatrixMode(GL_PROJECTION);                
	glLoadIdentity();
	gluPerspective(fov, (double)w/(double)h, near_plane, far_plane);
	glMatrixMode(GL_MODELVIEW);                 
/* **************************************************************************************************************************** */
}




/* ############################################################################################################################
   # Positionnement et orientation de la scène par rapport à la caméra
   ############################################################################################################################ */

void MoveToOrigin() {
/* **************************************************************************************************************************** */
	glMatrixMode(GL_MODELVIEW);

	glLoadIdentity();
    	glTranslatef(0.0f,-30.0f,-50.0f);
    	glTranslatef(0.0f,0.0f,znew);
    	glRotatef(ynew,1.0,0.0,0.0);
    	glRotatef(xnew,0.0,1.0,0.0);

/* **************************************************************************************************************************** */
}





/* ############################################################################################################################
   # Dessin de l'effecteur final
   ############################################################################################################################ */

void DrawEE()
{
/* **************************************************************************************************************************** */
	glColor3f(1.f, 0.f, 0.f);
/* **************************************************************************************************************************** */
	glLoadName(_EE_ID_);                           
/* **************************************************************************************************************************** */
	glCallList(EEList);
/* **************************************************************************************************************************** */
}



 

 


/* ############################################################################################################################
   # Dessin du squelette
   ############################################################################################################################ */

void DrawSkelet(void)
{
/* **************************************************************************************************************************** */
	int			 i;
	MDFloat		 tempT[4][4];
	MDFloat		 ZTi[4][4];
	MDFloat		*ptrZTi		 = (MDFloat*)ZTi;
	MDFloat		*ptrtempT	 = (MDFloat*)tempT;
/* **************************************************************************************************************************** */
	MoveToOrigin();
/* **************************************************************************************************************************** */

	glLoadName(_DESIRED_EE_ID_);

	glDisable(GL_LIGHTING);
	glMatrixMode(GL_MODELVIEW);
	glBegin(GL_POINTS);
		glColor3f(1.0f, 1.0f, 0.2f);
		glVertex3dv(PNew);
	glEnd();
	glEnable(GL_LIGHTING);
/* **************************************************************************************************************************** */
	glMatrixMode(GL_MODELVIEW);
/* **************************************************************************************************************************** */
	glPushMatrix();
/* **************************************************************************************************************************** */
	LoadIdentity(ptrZTi);
	for (i=0; i<_NO_OF_LINKS_; i++) {

		Skelet[i].Draw();

		glPopMatrix();
		glPushMatrix();

		MultMatrix4x4((MDFloat(*)[4])Skelet[i].Get_im1Ti(), ZTi, tempT);
		CopyMatrix4x4(ptrtempT, ptrZTi);

		glMultTransposeMatrixd(ptrZTi);
	}
/* **************************************************************************************************************************** */
	DrawEE();
/* **************************************************************************************************************************** */
	glPopMatrix();
/* **************************************************************************************************************************** */
}



 
/* ############################################################################################################################
   # Positionnement du vecteur de direction de l'effecteur final
   ############################################################################################################################ */

void SetEEOrientView(int w, int h) {
/* **************************************************************************************************************************** */
#define	_EEW_W_	(100)
#define	_EEW_H_	(100)
#define	_EEW_B_	(2)
/* **************************************************************************************************************************** */
	glViewport(WindowWidth-_EEW_W_, 0 , _EEW_W_, _EEW_H_);                    
	glMatrixMode(GL_PROJECTION);                
	glLoadIdentity();
	gluPerspective(fov, (double)_EEW_W_/(double)_EEW_H_, 10, 40);
/* **************************************************************************************************************************** */
}





/* ############################################################################################################################
   # Test de clique dans la zone du vecteur direction de l'effecteur final
   ############################################################################################################################ */

MDBool IsEEOrientViewClick(int w, int h, int x, int y) {
/* **************************************************************************************************************************** */
	return( (x > (w-_EEW_W_)) && ((h-y)<_EEW_H_) );
/* **************************************************************************************************************************** */
}






/* ############################################################################################################################
   # Dessin du vecteur direction de l'effecteur final
   ############################################################################################################################ */

void DrawEEOrientation() {
/* **************************************************************************************************************************** */
	/*
	 * Draw Grey border
	 */
	SetEEOrientView(WindowWidth, WindowHeight); 
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, _EEW_W_, 0, _EEW_H_, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glDisable(GL_LIGHTING);
	glColor3f(1, 1, 1);
	glColor3f(0.5, 0.5, 0.5);
	glBegin(GL_TRIANGLE_STRIP);
		glVertex2f(0, 0);
		glVertex2f(_EEW_B_, _EEW_B_);
		glVertex2f(0, _EEW_H_);
		glVertex2f(_EEW_B_, _EEW_H_-_EEW_B_);
		glVertex2f(_EEW_W_, _EEW_H_);
		glVertex2f(_EEW_W_-_EEW_B_, _EEW_H_-_EEW_B_);
		glVertex2f(_EEW_W_, 0);
		glVertex2f(_EEW_W_-_EEW_B_, _EEW_B_);
		glVertex2f(0, 0);
		glVertex2f(_EEW_B_, _EEW_B_);
	glEnd();
	glEnable(GL_LIGHTING);
/* **************************************************************************************************************************** */
	/* Set persp. proj. */
	SetEEOrientView(WindowWidth, WindowHeight); 
/* **************************************************************************************************************************** */
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
    glTranslatef(0.0f,0.0f,-25.0f);
    glRotatef(ynew,1.0,0.0,0.0);
    glRotatef(xnew,0.0,1.0,0.0);
    glRotatef(ONew[0], 1.0, 0.0, 0.0);
    glRotatef(ONew[1], 0.0, 1.0, 0.0);
    glRotatef(ONew[2], 0.0, 0.0, 1.0);
/* **************************************************************************************************************************** */
#define _ARROW_L_	(5)
/* **************************************************************************************************************************** */
	/* 
	 * Display Arrow
	 */

	GLUquadricObj	*gluAxesObj_1;
	gluAxesObj_1	= gluNewQuadric();

	glPushMatrix();
	glColor3f(1, 1, 0);
	glScalef(_ARROW_L_, 0.5, 0.5);
	glRotatef(90, 0, 1, 0);
	gluCylinder(gluAxesObj_1, 1.0, 1.0, 1.0, 15, 15);
	glPopMatrix();
	glPushMatrix();
	glTranslatef(_ARROW_L_, 0, 0);
	glRotatef(-90, 0, 1, 0);
	gluDisk(gluAxesObj_1, 0.5, 1.5, 15, 15);
	glTranslatef(0, 0, -2);
	gluCylinder(gluAxesObj_1, 0.0, 1.5, 2.0, 15, 15);
	glPopMatrix();
/* **************************************************************************************************************************** */
}







/* ############################################################################################################################
   # Messages d'indications des fonctions des touches claviers
   ############################################################################################################################ */

void HelpMessage() {
/* **************************************************************************************************************************** */
	fprintf(stderr, "  ----------------------\n");
	fprintf(stderr, "  Usage:\n");
	fprintf(stderr, "  ======\n");
	fprintf(stderr, "  <LEFT_mouse_button>+<mouse_move> ............ Manipulation de l'orientation de la scène\n");
	fprintf(stderr, "  <RIGHT_mouse_button>+<mouse_move> ........... Manipulation de la distance de la caméra\n");
	fprintf(stderr, "  <CTRL>+<LEFT_mouse_button>+<mouse_move> ..... Déplacer le point désiré\n");
	fprintf(stderr, "  <SHIFT>+<LEFT_mouse_button>+<mouse_move> .... Déplacer interactivement l'effecteur\n");
	fprintf(stderr, "  terminal ou manipulation directe des DDLs du squelette (cinématique directe)\n");
	fprintf(stderr, "  'i','I' ... Appliquer la boucle interne des calculs de CI\n");
	fprintf(stderr, "  <ENTER> ... Appliquer la boucle externe de CI pour atteindre la position désirée\n");
	fprintf(stderr, "  'l','L' ... Limiter les angles\n");
	fprintf(stderr, "  'r','R' ... Initialiser la scène         'h','H' ...Aide\n");
	fprintf(stderr, "  'f','F' ... Mode full screen         'w','W' ... Mode window\n");
	fprintf(stderr, "  <ESC>,'q','Q','x','X' ..................... exit\n");
	fprintf(stderr, "  Manipulation de l'orientation désirée de l'effecteur terminal:\n");
	fprintf(stderr, "    <mouse_button>+<mouse_move> ............... Manipulation de l'angle Gamma\n");
	fprintf(stderr, "    <SHIFT>+<mouse_button>+<mouse_move> ....... Manipulation de l'angle Beta\n");
	fprintf(stderr, "  ----------------------\n");
/* **************************************************************************************************************************** */
}
 


    

/* ############################################################################################################################
   # Sélection : Fonction de calcul de la primitive la plus proche
   ############################################################################################################################ */
 
GLuint GetNearestHitName(GLint hits, GLuint buffer[]) {
/* **************************************************************************************************************************** */
	unsigned int	 i/*, j*/;
	GLuint			 names,
					 MinVal,
		  			 z1,
					*ptr;
	int				 MinName;
/* **************************************************************************************************************************** */
	MinVal	 = 0xffffffff;
	MinName	 = -1;
/* **************************************************************************************************************************** */
	ptr = (GLuint *) buffer;
	for (i = 0; (int)i < hits; i++) {	// Pour chaque element sélectioné
		names = *ptr;			// Nombre d'élements retournés par la procédure de séléction
		
		if (names > 1) {
			fprintf(stderr, "You have some problems in selection rendering mode.\nI should get always only one hit!\nI got %d hits\n", names); 
			exit(2);
		};

		if (names != 0) {
			ptr++;			// -> z1 (zmin)

			z1	 = *ptr;
			ptr++;			// -> z2 (zmax)

			ptr++;			// -> Identifiant

			if (z1 < MinVal) { 	// Comparaison des prondeurs 
				MinVal	 = z1;
				MinName	 = *ptr;
			}
			ptr++;

		} else {
			ptr++;
			ptr++;
			ptr++;
		}
   }
/* **************************************************************************************************************************** */
	return(MinName);
/* **************************************************************************************************************************** */
}


/* ############################################################################################################################
   #  Sélection : Routine de sélection : zone 5x5 autour du curseur
   ############################################################################################################################ */

int PickLink(int x, int y) {
/* **************************************************************************************************************************** */
	GLuint	 record[64];                      // Défintion du buffer de selection (noms des pritives sélectionnées)
						  //	structure du buffer :
						  //	<Number of hits>
						  //		for every hit:
						  //		<number of names in hit> 
						  //		<minimum z coordinate>
						  //		<maximum z coordinate>
						  //			for every name:
						  //			<name>

	GLint	 viewport[4];                     // paramètres du Viewport pour la sélection
	GLuint	 numHits;                         // Nombre des objets selectionnés
/* **************************************************************************************************************************** */
	glSelectBuffer(64, record);               // Attribution du buffer au mécanisme de selection
	glRenderMode(GL_SELECT);                  // Activation du mode de sélection (la scène n'est pas dessiné)
	glPushName(0);                            // ..

	SetGlobalView(WindowWidth, WindowHeight);

	glMatrixMode(GL_PROJECTION);              // Défintion d'une matrice de projection qui restreint les calcul de rendu à une région : 5x5
	glPushMatrix();
	glLoadIdentity();                       
	glGetIntegerv(GL_VIEWPORT, viewport);     
	gluPickMatrix(x, viewport[3]-y, 5.f, 5.f, viewport);  
	gluPerspective(fov, (double)WindowWidth/(double)WindowHeight, near_plane, far_plane); // Multiplication par la matrice de projection en cours
									 
/* **************************************************************************************************************************** */
	glMatrixMode(GL_MODELVIEW);               // Calcul de rendu de la scène
	glLoadIdentity();                       
	DrawSkelet();                               
	glFlush();               
              
/* **************************************************************************************************************************** */

	numHits = glRenderMode(GL_RENDER);       // Renvoi du nombre de primitives selectionés et remplissage de la structure "record"
						 // avec les paramètres (nom, zmin, zmax)

/* **************************************************************************************************************************** */

	DisplayHits(numHits, record );		 // Affichage des elements selectionnés

/* **************************************************************************************************************************** */

	return( GetNearestHitName(numHits, record) );

/* **************************************************************************************************************************** */
}






/* ############################################################################################################################
   #  Sélection : Affichage des informations générales relatives à la sélelction
   ############################################################################################################################ */

void DisplayHits (GLint hits, GLuint buffer[]) {
/* **************************************************************************************************************************** */
	unsigned int	 i, j;
	GLuint			 names, *ptr;
/* **************************************************************************************************************************** */
	if (hits > 0) {
		printf ("===========================\n"); 
		printf ("Nombre de sélections = %d\n", hits);
	}
	ptr = (GLuint *) buffer;
	for (i = 0; (int)i < hits; i++) {	
		names = *ptr;

		if (names != 0) {
			printf ("\n Nombre de noms pour cette sélection = %d\n", names);
			ptr++;
			printf("  zmin is %g\n", (float) *ptr/0x7fffffff);
			ptr++;
			printf("  zmax is %g\n", (float) *ptr/0x7fffffff);
			ptr++;
			printf ("   Identifiant ");
			for (j = 0; j < names; j++) { 
				printf ("%d ", *ptr);
				ptr++;
			}
		} else {
			ptr++;
			ptr++;
			ptr++;
		}
		printf ("\n\n");
   }
/* **************************************************************************************************************************** */
}




