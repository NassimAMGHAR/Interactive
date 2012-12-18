////////////////////////////////
// 2008			      //
// TD Animation 3D	      //
// Université Paris 11	      //  
// Mehdi AMMI - ammi@limsi.fr //
////////////////////////////////

#include <sys/time.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "MDVectors.h"


/* ############################################################################################################################
   # Declaration des constantes
   ############################################################################################################################ */


#define	_NO_OF_LINKS_		(9)
#define	_EE_ID_			(_NO_OF_LINKS_+1)
#define	_DESIRED_EE_ID_		(_NO_OF_LINKS_+2)

#define _INNER_EPSILON_		(15.0)
#define _MAX_INNER_ITER_	(16)
#define _OUTER_EPSILON_		(2.5)
#define _MAX_OUTER_ITER_	(64)

#define _L_			(10.0)
#define _THETA_			(25.0)
#define _MIN_THETA_		(0.0)
#define _MAX_THETA_		(45.0)
#define _THICKNESS_		(1.0)


/* ############################################################################################################################
   # Declaration des variables
   ############################################################################################################################ */


_LOCAL_	MDFloat	 a, b, c, d;

int   xnew=0, ynew=0, znew=-100;                   // Position désirée de l'effecteur terminal
int   xold=0, yold=0, zold=-100;                   // Position actuelle de l'effecteur final
int   xx1=0, yy1=0, zz1=0;                         


float thickness = 3.f;                          

float rotationSpeed = 180.f;                     

MDBool	AngleLimits	 = false;


/* ############################################################################################################################
   # Identifiants des listes représentant les différents éléments de la scène
   ############################################################################################################################ */

int	 LinkList;
int	 JointList;
int	 EEList;
int	 AxesList;

/* ############################################################################################################################
   # Classe squelette
   ############################################################################################################################ */

class CLink {
	int		 idx; 			// Identifiant pour l'opération de sélelction
	/* *** */
			MDFloat	 Length;	// Longeur du segment
	/* *** */
			MDFloat	 Alpha;		// angles
			MDFloat	 Beta;
			MDFloat	 Gamma;
	/* *** */
			MDFloat	 MinAlpha;	// angles limites des articulation
			MDFloat	 MaxAlpha;
			MDFloat	 MinBeta;
			MDFloat	 MaxBeta;
			MDFloat	 MinGamma;
			MDFloat	 MaxGamma;
	/* *** */
			MDFloat	 Thick;		// Epaisseur du segment
			MDFloat	 im1Ti[4][4];   // Matrice de transformation locale ML[R:T]
			MDFloat	 ZTi[4][4];	// Matrice de transformation du noeud racine au noeud actuel MG[R:T] = MLn * MLn-1 * ... ML0
	/* *** */
			MDBool	 Selected;	// Etat de sélection du segment en cour
			static	MDBool	 *Limits; // Activation/Désactivation des limites angulaires
	/* *** */
	
			void	 Updateim1Ti(); // mise-à-jour de la matrice de transformation locale

public:
	/* *** */
	CLink();
	/* *** */
	// Dessin du squelette
	void Draw();
	/* *** */
	// Fonction de sélection
	void	 Select()				{	Selected	 = true;	 }
	void	 UnSelect()				{	Selected	 = false;	 }
	MDBool	 IsSelected()			{	return(Selected);		 }
	/* *** */
	// Fonctions de manipulation d'angles
	MDFloat	 GetAlpha()				{	return(Alpha);			 }
	MDFloat	 GetBeta()				{	return(Beta);			 }
	MDFloat	 GetGamma()				{	return(Gamma);			 }

	void	 SetAlpha(MDFloat t)	{	Alpha	 = t;
										ClampAngles();
										Updateim1Ti();			 }

	void	 SetBeta(MDFloat t)		{	Beta	 = t;
										ClampAngles();
										Updateim1Ti();			 }

	void	 SetGamma(MDFloat t)	{	Gamma	 = t;
										ClampAngles();
										Updateim1Ti();			 }

	void	 ClampAngles();

	MDFloat	 GetLength()			{	return(Length);			 }

	void	 SetLength(MDFloat l)	{	Length	 = l;
		 								Updateim1Ti();			 }

	/* *** */
	// Fonctions de manipulation des matrices
	MDFloat	*Get_im1Ti()			{	return((MDFloat*)im1Ti); }


	MDFloat	*Get_ZTi()				{	return((MDFloat*)ZTi);	 }

	void	 Set_ZTi(MDFloat (*M)[4] )
									{	memcpy(ZTi, M, 4*4*sizeof(MDFloat)); }


	void	 Updateim1Ti(MDFloat ThetaX, MDFloat	ThetaY, MDFloat	ThetaZ);

	// Fonction d'attribution d'identifiant (sélection)
	void	 SetIdx(int i)			{	idx	 = i;				 }
};  


/* ############################################################################################################################
   # Intilisation des paramètres du sequelette
   ############################################################################################################################ */


CLink::CLink(){ 
/* **************************************************************************************************************************** */
	idx		 = -1;
	Length		 =  _L_;
	Alpha		 =  0;
	Beta		 =  0;
	Gamma		 =  _THETA_;
	MinAlpha	 = -_MAX_THETA_/5;
	MinBeta		 =  _MIN_THETA_;
	MinGamma	 =  _MIN_THETA_;
	MaxAlpha	 =  _MAX_THETA_/5;
	MaxBeta		 =  _MIN_THETA_;
	MaxGamma	 =  _MAX_THETA_;
	Thick		 =  _THICKNESS_;
	Selected	 = false;
/* **************************************************************************************************************************** */
	Updateim1Ti();
/* **************************************************************************************************************************** */
}


/* ############################################################################################################################
   # Fonction d'application de limites aux angles
   ############################################################################################################################ */



void CLink::ClampAngles() {
/* **************************************************************************************************************************** */
	if (!*Limits) return;
	if (idx == (_NO_OF_LINKS_-1)) return;
/* **************************************************************************************************************************** */
	if (Alpha<MinAlpha)	 Alpha	 = MinAlpha;
	if (Alpha>MaxAlpha)	 Alpha	 = MaxAlpha;
	if (Beta<MinBeta)	 Beta	 = MinBeta;
	if (Beta>MaxBeta)	 Beta	 = MaxBeta;
	if (Gamma<MinGamma)	 Gamma	 = MinGamma;
	if (Gamma>MaxGamma)	 Gamma	 = MaxGamma;
/* **************************************************************************************************************************** */
}


/* ############################################################################################################################
   # Fonction de mise-à-jour de la matrice de transformation locale
   ############################################################################################################################ */


void CLink::Updateim1Ti(){
/* **************************************************************************************************************************** */
	MDFloat	 RotX[4][4];
	MDFloat	 RotY[4][4];
	MDFloat	 RotZ[4][4];
	MDFloat	 Tr[4][4];
	MDFloat	 TempM[4][4];

    	MDRotatexf((MDFloat*)RotX, Alpha);
    	MDRotateyf((MDFloat*)RotY, Beta);
    	MDRotatezf((MDFloat*)RotZ, Gamma);

    	MDTranslatef((MDFloat*)Tr, Length, 0.0, 0.0);

	MultMatrix4x4(RotX, RotY, TempM);
	MultMatrix4x4(TempM, RotZ, RotX);
	MultMatrix4x4(RotX, Tr, im1Ti);
/* **************************************************************************************************************************** */
}

/* ############################################################################################################################
   # Fonction de mise-à-jour de la matrice de transformation locale (avec limites sur les angles)
   ############################################################################################################################ */


void CLink::Updateim1Ti(MDFloat	ThetaX, MDFloat	ThetaY, MDFloat	ThetaZ){
/* **************************************************************************************************************************** */
	if (ThetaX > 5.0) ThetaX = 5.0;
	if (ThetaY > 5.0) ThetaY = 5.0;
	if (ThetaZ > 5.0) ThetaZ = 5.0;
	if (ThetaX < -5.0) ThetaX = -5.0;
	if (ThetaY < -5.0) ThetaY = -5.0;
	if (ThetaZ < -5.0) ThetaZ = -5.0;
/* **************************************************************************************************************************** */
	Alpha	+= ThetaX;
	Beta	+= ThetaY;
	Gamma	+= ThetaZ;
/* **************************************************************************************************************************** */
	ClampAngles();
/* **************************************************************************************************************************** */
	Updateim1Ti();
/* **************************************************************************************************************************** */
}


/* ############################################################################################################################
   # Fonction de dessin des segments
   ############################################################################################################################ */

void CLink::Draw(){
/* **************************************************************************************************************************** */
	if (Selected)
		glColor3f(0.f, 0.7f, 0.8f);
	else
		glColor3f(0.8, 0.8, 0.8);
/* **************************************************************************************************************************** */
	glLoadName(idx);                           	// Identifiant GL pour la sélection
/* **************************************************************************************************************************** */
	glPushMatrix();					// Dessin du segment
	glScalef(Length, Thick, Thick);
	glCallList(LinkList);
	glPopMatrix();
/* **************************************************************************************************************************** */
	glPushMatrix();					// Dessin de l'articulation
	glTranslatef(Length, 0, 0);
	glScalef(1.25*Thick, 1.25*Thick, 1.25*Thick);
	glCallList(JointList);
	glPopMatrix();
/* **************************************************************************************************************************** */
}








/* ############################################################################################################################
   # Variables des calculs de la cinématique inverse
   ############################################################################################################################ */

#define	_JAC_M_		(3*_NO_OF_LINKS_)
#define	_JAC_N_		(3*2)

CLink	 Skelet[_NO_OF_LINKS_];

typedef		MDFloat		 (*TJacobian) [_JAC_N_];
typedef		MDFloat		 (*TJacobianT)[_JAC_M_];
typedef		MDFloat		 (*TK)[_JAC_M_];

MDFloat	 J [_JAC_M_][_JAC_N_];
MDFloat	 Jt[_JAC_N_][_JAC_M_];
MDFloat	 K [_JAC_M_][_JAC_M_];

MDFloat	 ONew[4];							// desired orientation of EE
MDFloat	 PNew[4];							// desired position of EE
MDFloat	 PnIter[4];							// position of EE after IK
MDFloat	 OnIter[4];

MDBool	(*CLink::Limits)	 = &AngleLimits;



/* ############################################################################################################################
   # Fonctions des calculs de la cinématique inverse
   ############################################################################################################################ */



/* ############################################################################################################################
   # Affichage la Jacobienne
   ############################################################################################################################ */

void PrintJacobian(FILE *File, TJacobian J) {
/* **************************************************************************************************************************** */
	int	 i, j;
/* **************************************************************************************************************************** */
	fprintf(File, "J[%d][%d]:\n", _JAC_M_, _JAC_N_);
	for (j=0; j<_JAC_N_; j++) {
		fprintf(File, " ");
		for (i=0; i<_JAC_M_; i++) {
			fprintf(File, "%4.2f\t", J[i][j]);
		}
		fprintf(File, "\n");
	}
/* **************************************************************************************************************************** */
}

/* ############################################################################################################################
   # Affichage de la Jacobienne transposée
   ############################################################################################################################ */

void PrintJacobianT(FILE *File, TJacobianT Jt) {
/* **************************************************************************************************************************** */
	int	 i, j;
/* **************************************************************************************************************************** */
	fprintf(File, "Jt[%d][%d]:\n", _JAC_N_, _JAC_M_);
	for (j=0; j<_JAC_M_; j++) {
		fprintf(File, " ");
		for (i=0; i<_JAC_N_; i++) {
			fprintf(File, "%4.2f\t", Jt[i][j]);
		}
		fprintf(File, "\n");
	}
/* **************************************************************************************************************************** */
}

/* ############################################################################################################################
   # Fonction de calcul de la Jacobienne transposée
   ############################################################################################################################ */

void TransposeJacobian(TJacobian J, TJacobianT Jt) {
/* **************************************************************************************************************************** */
	int		 i, j;
/* **************************************************************************************************************************** */
	for (j=0; j<_JAC_N_; j++) {
		for (i=0; i<_JAC_M_; i++) {
			Jt[j][i]	 = J[i][j];
		}
	}
/* **************************************************************************************************************************** */
}

/* ############################################################################################################################
   # Fonction de multiplication : dTheta = Jt * dX
   ############################################################################################################################ */

void MultJacobianT(TJacobianT Jt, MDFloat dX[_JAC_N_], MDFloat dTheta[_JAC_M_]) {
/* **************************************************************************************************************************** */
	int		 i, j;
/* **************************************************************************************************************************** */
	for (i=0; i<_JAC_M_; i++) {
		dTheta[i]	 = 0.0;
		for (j=0; j<_JAC_N_; j++) {
			dTheta[i]	+= Jt[j][i]*dX[j];
		}
	}
/* **************************************************************************************************************************** */
}

/* ############################################################################################################################
   # Fonction de Multiplication : JtRes = K * Jt (K : matrice de raideur)
   ############################################################################################################################ */

void MultKJacobianT(TK K, TJacobianT Jt, TJacobianT JtRes) {
/* **************************************************************************************************************************** */
	int		 i, j, k;
/* **************************************************************************************************************************** */
	for (i=0; i<_JAC_M_; i++) {
		for (j=0; j<_JAC_N_; j++) {
			JtRes[j][i]	 = 0.0;
			for (k=0; k<_JAC_M_; k++) {
				JtRes[j][i]	+= K[k][i]*Jt[j][k];
			}
		}
	}
/* **************************************************************************************************************************** */
}

/* ############################################################################################################################
   # Fonction de calcul de la Jacobienne
   ############################################################################################################################ */

void MakeJacobian(CLink *Skelet, TJacobian J) {
/* **************************************************************************************************************************** */
	int		 i;

	MDFloat		 ai[4],
			 bi[4],
			 v[4],
			 Pn[4],
			 Pj[4];

	MDFloat		 tempT[4][4];
	MDFloat		 tempZTi[4][4];

	MDFloat		*ptrZTi		 = (MDFloat*)tempZTi;
	MDFloat		*ptrtempT	 = (MDFloat*)tempT;
/* **************************************************************************************************************************** */
	// Calcul de la matrice de transformation globale de l'effecteur terminal (ZTi)
	LoadIdentity(ptrZTi);
	for (i=0; i<_NO_OF_LINKS_; i++) {
	// .. 
		MultMatrix4x4((MDFloat(*)[4])Skelet[i].Get_im1Ti(),tempZTi,tempT);
		CopyMatrix4x4(ptrtempT,ptrZTi);
		Skelet[i].Set_ZTi(tempZTi);
	}
/* **************************************************************************************************************************** */

	MDFloat	 (*ZTi)[4];
	MDFloat	 (*ZTN)[4];

	// Extraction de la position de l'effecteur terminal
	// .. 
		
		ZTN = (MDFloat(*)[4])Skelet[_NO_OF_LINKS_-1].Get_ZTi();
		Pn[0] = ZTN[0][3];
		Pn[1] = ZTN[1][3];
		Pn[2] = ZTN[2][3];
		Pn[3] = 1.0;

	// Calcul de la Jacobienne (3 lignes)
	for (i=0; i<_NO_OF_LINKS_; i++) {

		// Caclul du vecteur reliant l'effecteur terminal aux différents repères (locaux) des segments du squelette : v = Pn - Pj
		// .. 
		ZTi = (MDFloat(*)[4])Skelet[i].Get_ZTi();
		Pj[0] = ZTi[0][3];
		Pj[0] = ZTi[1][3];
		Pj[0] = ZTi[2][3];
		Pj[0] = 1.0;
		VectSub(Pn,Pj,v);

		// Extraction du vecteur de rotation suivant l'axe oX (1ere colonne)
		// .. 
		ai[0] = ZTi[0][0];
		ai[1] = ZTi[1][0];
		ai[2] = ZTi[2][0];
		ai[3] = 1.0;
		// Calcul du vacteur orthogonal au vecteur v et au vecteur de rotation suivant l'axe oX : bi = ai X v
		// .. 
		VectProd(ai,v,bi);
		
		// Calcul de la 1ere colonne de la Jacobienne (DDL : oX) : J[0][x]t = [bi[0],bi[1],bi[2],ai[0],ai[1],ai[2]]
		// .. 
         	J[i*3][0] = bi[0];
		J[i*3][1] = bi[1];
		J[i*3][2] = bi[2];
		J[i*3][3] = ai[0];
		J[i*3][4] = ai[1];
		J[i*3][5] = ai[2];


		// Extraction du vecteur de rotation suivant l'axe oY (2eme colonne)
		// .. 
		ai[0] = ZTi[0][1];
		ai[1] = ZTi[1][1];
		ai[2] = ZTi[2][1];
		ai[3] = 1.0;
		// Calcul du vacteur orthogonal au vecteur v et au vecteur de rotation suivant l'axe oY : bi = ai X v
		// .. 
		VectProd(ai,v,bi);

		// Calcul de la 2eme colonne de la Jacobienne (DDL : oY) : J[1][x]t = [bi[0],bi[1],bi[2],ai[0],ai[1],ai[2]]
		// .. 
		J[i*3+1][0] = bi[0];
		J[i*3+1][1] = bi[1];
		J[i*3+1][2] = bi[2];
		J[i*3+1][3] = ai[0];
		J[i*3+1][4] = ai[1];
		J[i*3+1][5] = ai[2];
	


		// Extraction du vecteur de rotation suivant l'axe oZ (3eme colonne)
		// .. 
		ai[0] = ZTi[0][2];
		ai[1] = ZTi[1][2];
		ai[2] = ZTi[2][2];
		ai[3] = 1.0;

		// Calcul du vacteur orthogonal au vecteur v et au vecteur de rotation suivant l'axe oZ : bi = ai X v
		// .. 
		VectProd(ai,v,bi);


		// Calcul de la 3eme colonne de la Jacobienne (DDL : oZ) : J[2][x]t = [bi[0],bi[1],bi[2],ai[0],ai[1],ai[2]]
		// .. 
		J[i*3+2][0] = bi[0];
		J[i*3+2][1] = bi[1];
		J[i*3+2][2] = bi[2];
		J[i*3+2][3] = ai[0];
		J[i*3+2][4] = ai[1];
		J[i*3+2][5] = ai[2];
	} FILE *f ;
	 f = fopen ( "test.jac", "w" ) ;PrintJacobian(f,J);

/* **************************************************************************************************************************** */
}

/* ############################################################################################################################
   # Mise-à-jour de transformation de noeud racine au noeud actuel
   ############################################################################################################################ */

void UpdateZTiMatrices(CLink *Skelet) {
/* **************************************************************************************************************************** */
	int			 i;
	MDFloat		 tempT[4][4];
	MDFloat		 tempZTi[4][4];
	MDFloat		*ptrZTi		 = (MDFloat*)tempZTi;
	MDFloat		*ptrtempT	 = (MDFloat*)tempT;
/* **************************************************************************************************************************** */
	LoadIdentity(ptrZTi);
	for (i=0; i<_NO_OF_LINKS_; i++) {
		MultMatrix4x4((MDFloat(*)[4])Skelet[i].Get_im1Ti(), tempZTi, tempT);
		CopyMatrix4x4(ptrtempT, ptrZTi);
		Skelet[i].Set_ZTi(tempZTi);
	}
/* **************************************************************************************************************************** */
}

/* ############################################################################################################################
   # Fonction de calcul du vecteur de rotation à partir d'une matrice de transformation M[R:T] -> Angle[ox,oy,oz] 
   ############################################################################################################################ */

void GetEulerAngles(MDFloat (*T)[4], MDFloat *Angle) {
/* **************************************************************************************************************************** */
	int		 i;
/* **************************************************************************************************************************** */
	Angle[1]	 = asin(T[0][2]);
/* **************************************************************************************************************************** */
	if (Angle[1] == _PI_/2.0) {
		fprintf(stderr, "Angle[1] == _PI_/2.0\n");
		exit(3);
	}
/* **************************************************************************************************************************** */
	if ( (T[2][2] * cos(Angle[1])) > 0) {
		Angle[0]	 = atan(-T[1][2]/T[2][2]);
	} else {
		Angle[0]	 = atan(-T[1][2]/T[2][2]) + _PI_;
	}
/* **************************************************************************************************************************** */
	if ( (T[1][1] * cos(Angle[1])) > 0) {
		Angle[2]	 = atan(-T[0][1]/T[1][1]);
	} else {
		Angle[2]	 = atan(-T[0][1]/T[1][1]) + _PI_;
	}
/* **************************************************************************************************************************** */
	/* Convertion en degrées*/
	for (i=0; i<3; i++) 
		Angle[i]	 = 180*Angle[i]/_PI_;
/* **************************************************************************************************************************** */
}

/* ############################################################################################################################
   # Fonction de calcul de la position et de l'orientation de l'effecteur terminal : PnIter & OnIter
   ############################################################################################################################ */

void GetEEPosition(CLink *Skelet, MDFloat *PnIter, MDFloat *OnIter) {
/* **************************************************************************************************************************** */
	MDFloat		 (*ZTN)[4];
/* **************************************************************************************************************************** */
	ZTN	 = (MDFloat(*)[4])Skelet[_NO_OF_LINKS_-1].Get_ZTi();
/* **************************************************************************************************************************** */
	PnIter[0]	 = ZTN[0][3];
	PnIter[1]	 = ZTN[1][3];
	PnIter[2]	 = ZTN[2][3];
	PnIter[3]	 = 1.0;
/* **************************************************************************************************************************** */
	GetEulerAngles(ZTN, OnIter);
	OnIter[3]	 = 1.0;
/* **************************************************************************************************************************** */
}
 
/* ############################################################################################################################
   # Fonction de calcul de l'équation du plan parallèle au plan de projection
   ############################################################################################################################ */

MDBool GetXYPlaneEqn(MDFloat (*MViewMatrix)[4], MDFloat *Origin, MDFloat *a, MDFloat *b, MDFloat *c, MDFloat *d) {
/* **************************************************************************************************************************** */
	MDFloat	 v[3];
	MDFloat	 n[3];
/* **************************************************************************************************************************** */
	v[0]	 = 0;
	v[1]	 = 0;
	v[2]	 = 1;
	n[0]	 = 0;
	n[1]	 = 0;
	n[2]	 = 0;
/* **************************************************************************************************************************** */
	// Original matrix is transposed => code below is enough
	MultMatrixVec4(MViewMatrix, v, n);
	*a	 = n[0];
	*b	 = n[1];
	*c	 = n[2];
	*d	 = - (*a)*Origin[0] - (*b)*Origin[1] - (*c)*Origin[2];
/* **************************************************************************************************************************** */
	return(false);
/* **************************************************************************************************************************** */
}

/* ############################################################################################################################
   # Fonction de calcul de la cinématique inverse : une itération
   ############################################################################################################################ */

void InnerIterate() {
/* **************************************************************************************************************************** */
	int	i;
	int	NoOfIter;

	MDFloat	 dX[_JAC_N_];			// Vecteur d'entrée (position & orientation désirées) 
	MDFloat	 dTheta[_JAC_M_];		// Vecteur de sortie (DDL : oX1,oY1,oZ1,oX2,oY2,oZ2,etc.)
	MDFloat	 (*ZTN)[4];

	MDFloat	 TempJt[_JAC_N_][_JAC_M_];	
		
	MDFloat	 JdThetaP[4];			// Vecteurs d'erreur
	MDFloat	 JdThetaO[4];

	MDFloat	 ErrVectP[4];			 
	MDFloat	 ErrVectO[4];

	MDFloat	 Pn[4];				// Position actuelle de l'effecteur 
	MDFloat	 dXPn[4];			// Variation désirée en position : dXPn = PNew - Pn
	MDFloat	 On[4];				// Orientation actuelle de l'effecteur
	MDFloat	 dXOn[4];			// Variation désirée en orientation : dXOn = ONew - On

	MDFloat	 Err; 				// Erreur

	CLink	 BackUpSkelet[_NO_OF_LINKS_];
/* **************************************************************************************************************************** */
	NoOfIter	 = 0;
	// Sauvegarde du squelette **
	memcpy(BackUpSkelet, Skelet, _NO_OF_LINKS_*sizeof(CLink));
/* **************************************************************************************************************************** */

	/////////////////////////////
	// Calcul du vecteur d'entrée
	/////////////////////////////

	// Intilisation du vecteur d'entrée
	// .. 
	for(i = 0; i<_JAC_N_;i++)dX[i] = 0.0;
	// Calcul de la variation de position "dXPn" du vecteur d'entrée "dX"
	// .. 

	ZTN = (MDFloat(*)[4])Skelet[_NO_OF_LINKS_-1].Get_ZTi();
	Pn[0] = ZTN[0][3];
	Pn[1] = ZTN[1][3];
	Pn[2] = ZTN[2][3];
	Pn[3] = 1.0;

	VectSub(PNew,Pn,dXPn);

	// Calcul de la variation en orientation "dXOn" du vecteur d'entrée "dX"
	// .. 

	GetEulerAngles(ZTN, On);

	On[3] = 1.0;

	VectSub(ONew, On, dXOn);

	dX[0] = dXPn[0]*0.5;
	dX[1] = dXPn[1]*0.5;
	dX[2] = dXPn[2]*0.5;
	dX[3] = dXOn[0]*0.5;
	dX[4] = dXOn[1]*0.5;
	dX[5] = dXOn[2]*0.5;

/* **************************************************************************************************************************** */

	///////////////////////////////////////
	// Calcul du vecteur de sortie "dTheta"
	///////////////////////////////////////

	do {	// Améliore la convergence de l'algorithme **
		memcpy(Skelet, BackUpSkelet, _NO_OF_LINKS_*sizeof(CLink));

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

		// Calcul de la Jacobienne
		// .. 
		MakeJacobian(Skelet,J);
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

		// Calcul de la Jacobienne transposée
		// .. 
		TransposeJacobian(J, Jt);
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

		// Incorporation d'une raideur à la Jacobienne : poids forts aux articulations éloignée de l'effecteur.
		// .. 
		memcpy(TempJt, Jt, _JAC_N_*_JAC_M_*sizeof(MDFloat));
		MultKJacobianT(K , TempJt , Jt);
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */


		// Calcul du vecteur de sortie : dTheta = Jt * dX
		// .. 
		MultJacobianT(Jt, dX, dTheta );
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

		// Ajout de dTheta aux angles et mise-à-jour des matrices de transformation locales "m1Ti" du squelette
		// .. 
		for (i=0; i<_NO_OF_LINKS_; i++)
		Skelet[i].Updateim1Ti(dTheta[i*3+0], dTheta[i*3+1], dTheta[i*3+2]);
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

		// Estimation de l'erreur : Err² = (ErrVectP)² + (ErrVectO)²
		// .. 
		UpdateZTiMatrices(Skelet);
		GetEEPosition(Skelet, PnIter, OnIter);

		VectSub(PnIter , Pn , JdThetaP);
		VectSub(JdThetaP, dX , ErrVectP);

		VectSub(OnIter , On , JdThetaO);
		VectSub(JdThetaO, &dX[3], ErrVectO);		

		Err = sqrt ( ScalProd(ErrVectP,ErrVectP) + ScalProd(ErrVectO,ErrVectO) );
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

		//Calcul du nouveau vecteur d'entrée : division par deux pour fluidifier le mouvement **
		// ..
		dX[0] *=0.5;
		dX[1] *=0.5;
		dX[2] *=0.5;
		dX[3] *=0.5;
		dX[4] *=0.5; 
		dX[5] *=0.5;
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
		NoOfIter++;
	} while ( (Err > _INNER_EPSILON_) && (NoOfIter < _MAX_INNER_ITER_) );
#ifdef _DEBUG
	fprintf(stderr, "INNER loop: %d\n", NoOfIter);
#endif
	
/* **************************************************************************************************************************** */
}

/* ############################################################################################################################
   # 
   ############################################################################################################################ */


void FindConfiguration () {
/* **************************************************************************************************************************** */
	MDFloat	 Err;
	MDFloat	 ErrVectP[4];
	MDFloat	 ErrVectO[4];
	int		 NoOfIter;
/* **************************************************************************************************************************** */
	NoOfIter	 = 0;
/* **************************************************************************************************************************** */
	// Calcul de l'erreur
	// .. 
		VectSub(PNew, PnIter , ErrVectP);
		VectSub(ONew, OnIter, ErrVectO);
		Err = sqrt ( ScalProd(ErrVectP,ErrVectP) + ScalProd(ErrVectO,ErrVectO) );

	while ( (Err > _OUTER_EPSILON_) && (NoOfIter<_MAX_OUTER_ITER_) ) {
	// .. 
		InnerIterate();
		VectSub(PNew, PnIter , ErrVectP);
		VectSub(ONew, OnIter, ErrVectO);
		Err = sqrt ( ScalProd(ErrVectP,ErrVectP) + ScalProd(ErrVectO,ErrVectO) );

		NoOfIter++;
	}
/* **************************************************************************************************************************** */
#ifdef _DEBUG
	fprintf(stderr, "OUTER loop: %d\n", NoOfIter);
#endif
/* **************************************************************************************************************************** */
}


/* ############################################################################################################################
   # Fonction d'initialisation du squelette
   ############################################################################################################################ */

void SkeletInit() {
/* **************************************************************************************************************************** */
	int	 i;
	MDFloat	 (*ZTN)[4];
/* **************************************************************************************************************************** */
	// Initialisation des paramètres du squelette 
	Skelet[0].SetLength(0);
	Skelet[0].SetAlpha(0);
	Skelet[0].SetBeta(0);
	Skelet[0].SetGamma(_THETA_);
	for (i=0; i< _NO_OF_LINKS_; i++) {
		Skelet[i].SetIdx(i);
		if (i>0) {
			Skelet[i].SetLength(_L_);
			Skelet[i].SetAlpha(0);
			Skelet[i].SetBeta(0);
			Skelet[i].SetGamma(_THETA_);
		}
	}
/* **************************************************************************************************************************** */
	MDFloat		 tempT[4][4];
	MDFloat		 tempZTi[4][4];
	MDFloat		*ptrZTi		 = (MDFloat*)tempZTi;
	MDFloat		*ptrtempT	 = (MDFloat*)tempT;
/* **************************************************************************************************************************** */
	// Initialisation des matrices de transformation du noeud racine au noeud actuel  : ZTi
	LoadIdentity(ptrZTi);
	for (i=0; i<_NO_OF_LINKS_; i++) {
		MultMatrix4x4((MDFloat(*)[4])Skelet[i].Get_im1Ti(), tempZTi, tempT);
		CopyMatrix4x4(ptrtempT, ptrZTi);
		Skelet[i].Set_ZTi(tempZTi);
	}

	ZTN	 = (MDFloat(*)[4])Skelet[_NO_OF_LINKS_-1].Get_ZTi();
	PnIter[0]	 = ZTN[0][3];
	PnIter[1]	 = ZTN[1][3];
	PnIter[2]	 = ZTN[2][3];
	PnIter[3]	 = 1.0;

/* **************************************************************************************************************************** */
}

/* ############################################################################################################################
   # Fonction permettant de mettre le squelette en configuration initiale 
   ############################################################################################################################ */

void IKInit() {
/* **************************************************************************************************************************** */
	MDFloat	 (*ZTN)[4];
	MDFloat	 w;
	int		 i, j;
/* **************************************************************************************************************************** */
	// Initialisation des objectifs, en position et en orientation, à atteindre par l'effecteur terminal
	ZTN	 = (MDFloat(*)[4])Skelet[_NO_OF_LINKS_-1].Get_ZTi();
	PNew[0]	 = ZTN[0][3];
	PNew[1]	 = ZTN[1][3];
	PNew[2]	 = ZTN[2][3];
	PNew[3]	 = 1.0;

	PNew[0]	-= 15.0;
	PNew[1]	-= 15.0;

	ONew[0]	 = 0.0;
	ONew[1]	 = 0.0;
	ONew[2]	 = 180.0;
	ONew[3]	 = 1.0;

/* **************************************************************************************************************************** */
	// Calcul de la matrice de raideurs
	for (j=0; j<_NO_OF_LINKS_; j++) {
		for (i=0; i<_NO_OF_LINKS_; i++) {
			if (i==j) {
				w	 = _NO_OF_LINKS_ - i;
				if (Skelet[i].GetLength() == 0) {
					K[i*3+0][j*3+0]	 = 1.0/(w*5*Skelet[1].GetLength());
					K[i*3+1][j*3+1]	 = 1.0/(w*5*Skelet[1].GetLength());
					K[i*3+2][j*3+2]	 = 1.0/(w*5*Skelet[1].GetLength());
				} else {
					K[i*3+0][j*3+0]	 = 1.0/(w*Skelet[i].GetLength());
					K[i*3+1][j*3+1]	 = 1.0/(w*Skelet[i].GetLength());
					K[i*3+2][j*3+2]	 = 1.0/(w*Skelet[i].GetLength());
				}

			} else {
				K[i*3+0][j*3+0]	 = 0.0;
				K[i*3+1][j*3+1]	 = 0.0;
				K[i*3+2][j*3+2]	 = 0.0;
			}
		}
	}
/* **************************************************************************************************************************** */
}
