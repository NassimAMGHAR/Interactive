
#include "FourWheelDrive.h"
#include "OpenGL/GlutStuff.h"
#include "OpenGL/GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"

GLDebugDrawer gDebugDrawer;

int main(int argc,char* argv[])
{
    FourWheelDrive fwd;

    fwd.initPhysics();
	fwd.getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);

    return glutmain(argc, argv,800,600,"4WD",&fwd);
}
