#include "PendulumApplication.h"

#include "GLDebugDrawer.h"
static GLDebugDrawer	gDebugDrawer;
const int maxProxies = 32766;

PendulumApplication::PendulumApplication(void)
{

}


PendulumApplication::~PendulumApplication(void)
{
	exitPhysics();
}


void PendulumApplication::initPhysics()
{
	setCameraDistance(8.);
	m_debugMode |= btIDebugDraw::DBG_NoHelpText;

	///register some softbody collision algorithms on top of the default btDefaultCollisionConfiguration
	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();


	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
	m_softBodyWorldInfo.m_dispatcher = m_dispatcher;

	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax(1000,1000,1000);

	m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);

	m_softBodyWorldInfo.m_broadphase = m_broadphase;

	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	m_solver = solver;

	btSoftBodySolver* softBodySolver = 0;

	btDiscreteDynamicsWorld* world = new btSoftRigidDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration,softBodySolver);
	m_dynamicsWorld = world;
	m_dynamicsWorld->setDebugDrawer(&gDebugDrawer);

	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));
	m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);

	m_softBodyWorldInfo.m_sparsesdf.Initialize();

	//create a few basic rigid bodies
	btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0));
	localCreateRigidBody(0,groundTransform,groundShape);

	buildDoublePendulumRigid();
	//buildDoublePendulumSoft();
	//buildFlyingTrapeze();

	m_softBodyWorldInfo.m_sparsesdf.Reset();

	m_softBodyWorldInfo.air_density		=	(btScalar)1.2;
	m_softBodyWorldInfo.water_density	=	0;
	m_softBodyWorldInfo.water_offset	=	0;
	m_softBodyWorldInfo.water_normal	=	btVector3(0,0,0);
	m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);

}

void PendulumApplication::buildDoublePendulumRigid()
{
    //TODO
}

void PendulumApplication::buildDoublePendulumSoft()
{
    //TODO
}

void PendulumApplication::buildFlyingTrapeze()
{
    //TODO
}

void PendulumApplication::exitPhysics()
{
	//cleanup in the reverse order of creation/initialization
	//remove the rigid and soft bodies from the dynamics world and delete them
	for(int i=m_dynamicsWorld->getNumCollisionObjects()-1;i>=0;i--)
	{
		btCollisionObject*	obj=m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody*		body=btRigidBody::upcast(obj);
		if(body&&body->getMotionState())
		{
			delete body->getMotionState();
		}

		//remove the constraint from the dynamics world and delete them
		while(m_dynamicsWorld->getNumConstraints())
		{
			btTypedConstraint*	pc=m_dynamicsWorld->getConstraint(0);
			m_dynamicsWorld->removeConstraint(pc);
			delete pc;
		}

		btSoftBody* softBody = btSoftBody::upcast(obj);
		if (softBody)
		{
			getSoftDynamicsWorld()->removeSoftBody(softBody);
		} else
		{
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body)
				m_dynamicsWorld->removeRigidBody(body);
			else
				m_dynamicsWorld->removeCollisionObject(obj);
		}
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	delete m_dynamicsWorld;
	delete m_solver;
	delete m_broadphase;
	delete m_dispatcher;

	delete m_collisionConfiguration;
}

void PendulumApplication::renderme()
{

	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	m_dynamicsWorld->debugDrawWorld();

	btSoftRigidDynamicsWorld* softWorld = getSoftDynamicsWorld();

	for (  int i=0;i<softWorld->getSoftBodyArray().size();i++)
	{
		btSoftBody*	psb=(btSoftBody*)softWorld->getSoftBodyArray()[i];
		if (softWorld->getDebugDrawer() && !(softWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
		{
			btSoftBodyHelpers::DrawFrame(psb,softWorld->getDebugDrawer());
			btSoftBodyHelpers::Draw(psb,softWorld->getDebugDrawer(),softWorld->getDrawFlags());
		}
	}

	DemoApplication::renderme();
}

void PendulumApplication::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//simple dynamics world doesn't handle fixed-time-stepping
	float dt = getDeltaTimeMicroseconds()/1000000.f;
	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(dt);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}

	renderme();
	swapBuffers();
}

void PendulumApplication::displayCallback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	swapBuffers();
}

void PendulumApplication::clientResetScene()
{
	exitPhysics();
	initPhysics();
}

void PendulumApplication::keyboardCallback(unsigned char key, int x, int y)
{
    //TODO : catch the key event
    switch(key)
    {
        case(27):exitPhysics();
        exit(0);
        break;
        default:DemoApplication::keyboardCallback(key,x,y);
    }
}
