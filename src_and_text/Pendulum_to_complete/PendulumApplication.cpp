
#include "PendulumApplication.h"


PendulumApplication::PendulumApplication(void)
{
}


PendulumApplication::~PendulumApplication(void)
{
	exitPhysics();
}


void PendulumApplication::initPhysics()
{
	setTexturing(true);
	setShadows(true);
	m_debugMode |= btIDebugDraw::DBG_NoHelpText;
	setCameraDistance(btScalar(20.));

	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

	///create a few basic rigid bodies
	btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0));
	localCreateRigidBody(0,groundTransform,groundShape);

	btSphereShape* sphereShape = new btSphereShape(1.);
	m_collisionShapes.push_back(sphereShape);

	btTransform pendulumTransform;
	pendulumTransform.setIdentity();

    m_theta.m_value = SIMD_HALF_PI;
    m_theta.m_dot = 1.;
    m_r.m_value = 10;
    m_r.m_dot = 0;
    temp = 0;
	btVector3 to_Remplace; // to replace with te pendulum position

	pendulumTransform.setOrigin( to_Remplace);
	m_pendulumBody = localCreateRigidBody(1,pendulumTransform,sphereShape);
	m_pendulumBody->setCollisionFlags( m_pendulumBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
}

void PendulumApplication::exitPhysics()
{
	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
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

void PendulumApplication::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//simple dynamics world doesn't handle fixed-time-stepping
	m_dt = getDeltaTimeMicroseconds()/1000000.f;
	temp += m_dt;
	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(m_dt);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}
	btTransform trans;
	m_pendulumBody->getMotionState()->getWorldTransform(trans);
	Integrator_rk4();
	trans.getOrigin() = btVector3(0,20,0)+pol2cart(m_r.m_value,m_theta.m_value);
	//printf("%f %f %f\n",m_position.x(),m_po sition.y(),m_position.z());
	m_pendulumBody->getMotionState()->setWorldTransform(trans);
    m_r.m_value = m_r.m_value + 0.1 * sin (temp);
	//draw the rope
	btVector3 anchor(0,20,0);
	glLineWidth(2.0);
	glBegin(GL_LINES);
	glVertex3fv(anchor.m_floats);
	glVertex3fv(trans.getOrigin());
	glEnd();

	renderme();
	swapBuffers();
}

void PendulumApplication::Integrator_rk4()
{
/* here is the integrator part. Fill with the dynamical function */
    Pair    k1 = m_dt*f_rk4(m_theta);
    Pair	k2 = m_dt*f_rk4(m_theta+0.5*k1);
    Pair	k3 = m_dt*f_rk4(m_theta+0.5*k2);
    Pair	k4 = m_dt*f_rk4(m_theta+k3);

	m_theta=m_theta+(k1+2*k2+2*k3+k4)*(1/6.);

}


Pair PendulumApplication::f_rk4(Pair tetha)
{
	Pair kdt;
	btScalar m = 1; //mass of the pendulum
	btScalar g = 9.81; // gravity constant
	btScalar lambda = 0.2; // air viscosity

	kdt.m_value = tetha.m_dot;
	kdt.m_dot = ((2*(m_r.m_dot/m_r.m_value) - ((1/(m_r.m_value * m))))*lambda)*tetha.m_dot + (g/m_r.m_value) * (sin (tetha.m_value));

	return kdt;

}
/*
void PendulumApplication::update_r(float dt)
{
    m_r.m_value = m_r.m_value * cos (dt);
    return;
}*/

btVector3 PendulumApplication::pol2cart(btScalar r, btScalar theta)
{
    // usefull for transforming polar coordinate r, theta to cartesian coordinate x y z

	return  btVector3(r*cos(theta+SIMD_HALF_PI),r*sin(theta+SIMD_HALF_PI),0) ;
}

void PendulumApplication::mouseMotionFunc( int x, int y)
{
    if(m_mouseButtons & 1)
    {
        float dty;

/*
use m_mouseOldY or m_mouseOldX if you want to controle the length with the mouse !
*/
    //m_r.m_value +=0.1*(y - m_mouseOldY) + 0.1;

    }
    DemoApplication::mouseMotionFunc(x, y);
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
