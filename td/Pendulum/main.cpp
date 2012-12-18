#include "PendulumApplication.h"
#include "GlutStuff.h"

int main(int argc,char** argv)
{

	PendulumApplication pendulum;
	pendulum.initPhysics();


#ifdef CHECK_MEMORY_LEAKS
	ccdDemo.exitPhysics();
#else
	return glutmain(argc, argv,1024,600,"Bullet Physics Demo. http://bulletphysics.org",&pendulum);
#endif

	//default glut doesn't return from mainloop
	return 0;
}

