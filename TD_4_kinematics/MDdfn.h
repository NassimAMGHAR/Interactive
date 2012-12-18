
/* ############################################################################################################################
   #
   # MDdfn.h: Definition of common types and constants
   #
   ############################################################################################################################ */

 


#ifndef _MDDFN_H_
#define _MDDFN_H_




#ifdef _MD_TEST_FAKE_BND_
#define _FAKE_WEIGHT_	(0.00000001)
#define _FAKE_ETHA_		(0.000001)
#endif



#define _MD_DOUBLE_

#ifndef _MD_DOUBLE_
#define _MD_FLOAT_
#endif


#define _PI_ 3.1415926535897932384626433832795

#define _LINE_SHIFT_	(1e-2)
#define _NEAR_ZERO_		(1e-8)
#define _NEAR_INF_		(1e+30)
 
#define _GLOBAL_				 
#define _LOCAL_					 static
#define _IMPORT_				 extern
#define _EXPORT_				 extern

#define _MD_MAX_LONG_			 2147483647
#define _MD_MAX_INT_			 2147483647
#define _MD_MAX_MDINT_			 2147483647
#define _MD_MIN_MDINT_			 -2147483648


#ifdef __cplusplus
typedef	bool				MDbool;
typedef	bool				MDBool;
#else
typedef	unsigned char		MDbool;
typedef	unsigned char		MDBool;
#endif

typedef	int					MDint;
typedef	int					MDInt;
typedef	int					MDLong;
typedef	int					MDlong;
typedef	unsigned long int	MDuint;
typedef	unsigned long int	MDUInt;
typedef	unsigned int		MDDWORD;



#ifdef _MD_DOUBLE_
typedef	double				MDfloat;
typedef	double				MDFloat;
#else
typedef	float				MDfloat;
typedef	float				MDFloat;
#endif


#define CHECK_GL_ERROR(str)													\
{																			\
	GLenum	error;															\
	if ( error = glGetError() )												\
		fprintf(stderr, "GL error: %s (%s)", gluErrorString(error), str);	\
}


#endif 
