

/* ############################################################################################################################
   #
   # MDVectors.h: Vector operations
   #
   ############################################################################################################################ */

#ifndef _MDVECTORS_H_
#define _MDVECTORS_H_

/* ############################################################################################################################ */

 

#include "MDdfn.h"

/* ############################################################################################################################
   # MACROS
   ############################################################################################################################ */

/* Scalar product MACRO. U and V are pointers */
#define ScalProd(U,V)	((U)[0]*(V)[0] + (U)[1]*(V)[1] + (U)[2]*(V)[2])
/* Vector addition W = U+V */
#define VectAdd(U,V,W)	(W)[0]=(U)[0]+(V)[0]; (W)[1]=(U)[1]+(V)[1]; (W)[2]=(U)[2]+(V)[2]; (W)[3]=1.0f;
/* Vector subtraction W = U-V */
#define VectSub(U,V,W)	(W)[0]=(U)[0]-(V)[0]; (W)[1]=(U)[1]-(V)[1]; (W)[2]=(U)[2]-(V)[2]; (W)[3]=1.0f;
/* size */
#define VectSize(v)			sqrt(ScalProd(v,v))
/* maximum */
#define MAX(A,B)		((A>B)?A:B)


/* ############################################################################################################################
   # EXPORTED FUNCTIONS
   ############################################################################################################################ */

_EXPORT_ void		 MDTransposef(MDFloat *M);
_EXPORT_ void		 MDTranslatef(MDFloat *M, MDFloat dx, MDFloat dy, MDFloat dz);
_EXPORT_ void		 MDRotatexf(MDFloat *M, MDFloat alpha);
_EXPORT_ void		 MDRotateyf(MDFloat *M, MDFloat alpha);
_EXPORT_ void		 MDRotatezf(MDFloat *M, MDFloat alpha);
_EXPORT_ void		 MultVecMatrix4(MDFloat v[4], MDFloat (*A)[4], MDFloat res[4]);
_EXPORT_ void		 MultMatrix4x4(MDFloat (*A)[4], MDFloat (*B)[4], MDFloat (*res)[4]);
_EXPORT_ void		 CopyMatrix4x4(MDFloat *Src, MDFloat *Dst);
_EXPORT_ void		 LoadIdentity(MDFloat *M);

_EXPORT_ void		 VectProd(MDFloat *u, MDFloat *v, MDFloat *w);
_EXPORT_ void		 VectProdNorm	(MDfloat *u, MDfloat *v, MDfloat *w);
_EXPORT_ void		 Normalize		(MDfloat *v);
_EXPORT_ void		 MultMatrixVec4(MDFloat (*a)[4], MDFloat v[4], MDFloat res[4]);
_EXPORT_ void		 MultMatrixTVec4(MDFloat (*a)[4], MDFloat v[4], MDFloat res[4]);

/* ############################################################################################################################ */

#endif

