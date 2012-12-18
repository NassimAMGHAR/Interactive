
#include "PendulumApplication.h"
/**
    Amghar Nassim
    TP double pendule
*/

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

    /// pendule 1
	btSphereShape* sphereShape = new btSphereShape(1.);
	m_collisionShapes.push_back(sphereShape);

	btTransform pendulumTransform;
	pendulumTransform.setIdentity();

    btSphereShape* sphere2 = new btSphereShape(1.);
	m_collisionShapes.push_back(sphere2);

    btTransform pendulum2;
	pendulum2.setIdentity();

	m_theta.m_value = SIMD_HALF_PI *2;
    m_theta.m_dot = 1.;
    m_r.m_value = 5;
    m_r.m_dot = 0;

    m_theta2.m_value = SIMD_HALF_PI *2;
    m_theta2.m_dot = 1.;
    m_r2.m_value = 10;
    m_r2.m_dot = 0;

	pendulumTransform.setOrigin(pol2cart(m_r.m_value,m_theta.m_value));
	pendulum2.setOrigin(pol2cart(m_r2.m_value,m_theta2.m_value));

	m_pendulumBody = localCreateRigidBody(1,pendulumTransform,sphereShape);
	m_pendulumBody2 = localCreateRigidBody(1,pendulum2,sphere2);

	m_pendulumBody->setCollisionFlags( m_pendulumBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    m_pendulumBody2->setCollisionFlags( m_pendulumBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	/// pendulum 2----------------------------------------------------------------------------------------------------------------------------

    btSphereShape* sphereShapep2 = new btSphereShape(1.);
	m_collisionShapes.push_back(sphereShapep2);

	btTransform pendulumTransformp2;
	pendulumTransformp2.setIdentity();

    btSphereShape* sphere2p2 = new btSphereShape(1.);
	m_collisionShapes.push_back(sphere2p2);

    btTransform pendulum2p2;
	pendulum2p2.setIdentity();

	m_2theta.m_value = SIMD_HALF_PI *2;
    m_2theta.m_dot = 1.;
    m_2r.m_value = 5;
    m_2r.m_dot = 0;

    m_2theta2.m_value = SIMD_HALF_PI *2;
    m_2theta2.m_dot = 1.;
    m_2r2.m_value = 10;
    m_2r2.m_dot = 0;

	pendulumTransformp2.setOrigin(pol2cart(m_2r.m_value,m_2theta.m_value));
	pendulum2p2.setOrigin(pol2cart(m_2r2.m_value,m_2theta2.m_value));


	m_pendulum2Body = localCreateRigidBody(1,pendulumTransformp2,sphereShapep2);
	m_pendulum2Body2 = localCreateRigidBody(1,pendulum2p2,sphere2p2);

	m_pendulum2Body->setCollisionFlags( m_pendulum2Body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    m_pendulum2Body2->setCollisionFlags( m_pendulum2Body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
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
	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(m_dt);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}

	// pnodule 1 range kutta
	btTransform trans;
	m_pendulumBody->getMotionState()->getWorldTransform(trans);
	Integrator_rk4();

	trans.getOrigin() = btVector3(20,20,0)+pol2cart(m_r.m_value,m_theta.m_value);
	//printf("%f %f %f\n",m_position.x(),m_po sition.y(),m_position.z());
	m_pendulumBody->getMotionState()->setWorldTransform(trans);

	//draw the rope
	btVector3 anchor(20,20,0);
	glLineWidth(2.0);
	glBegin(GL_LINES);
	glVertex3fv(anchor.m_floats);
	glVertex3fv(trans.getOrigin());
	glEnd();

    //----------------------------------------------------------------------------------
    btTransform trans2;
	m_pendulumBody2->getMotionState()->getWorldTransform(trans2);
	//Integrator_rk4();

	trans2.getOrigin() = trans.getOrigin()+ pol2cart(m_r2.m_value,m_theta2.m_value);
    m_pos = trans2.getOrigin();
	printf(" Range Kutta :  %f  %f  %f\n",m_pos.x(),m_pos.y(),m_pos.z());
	m_pendulumBody2->getMotionState()->setWorldTransform(trans2);


	//draw the rope
	//btVector3 temp = pol2cart(m_r.m_value,m_theta.m_value);
	glLineWidth(2.0);
	glBegin(GL_LINES);
	glVertex3fv(trans.getOrigin());
	glVertex3fv(trans2.getOrigin());
	glEnd();


    ///-------------------------- pnodule 2 euler
	btTransform transp2;
	m_pendulum2Body->getMotionState()->getWorldTransform(transp2);


	transp2.getOrigin() = btVector3(-20,20,0)+pol2cart(m_2r.m_value,m_2theta.m_value);
	//printf("%f %f %f\n",m_position.x(),m_po sition.y(),m_position.z());
	m_pendulum2Body->getMotionState()->setWorldTransform(transp2);

	//draw the rope
	btVector3 anchor2(-20,20,0);
	glLineWidth(2.0);
	glBegin(GL_LINES);
	glVertex3fv(anchor2.m_floats);
	glVertex3fv(transp2.getOrigin());
	glEnd();

    //----------------------------------------------------------------------------------
    btTransform trans2p2;
	m_pendulum2Body2->getMotionState()->getWorldTransform(trans2p2);
	//Integrator_rk4();

	trans2p2.getOrigin() = transp2.getOrigin()+ pol2cart(m_2r2.m_value,m_2theta2.m_value);
	m_pos2 = trans2p2.getOrigin();
	printf(" Euler :       %f  %f  %f\n",m_pos2.x(),m_pos2.y(),m_pos2.z());
	m_pendulum2Body2->getMotionState()->setWorldTransform(trans2p2);


	//draw the rope
	glLineWidth(2.0);
	glBegin(GL_LINES);
	glVertex3fv(transp2.getOrigin());
	glVertex3fv(trans2p2.getOrigin());
	glEnd();

	renderme();
	swapBuffers();
}

void PendulumApplication::Integrator_rk4()
{
	Pair y,r,y1,r1;
	Dpair k1,k2,k3,k4,ke;
	btScalar rp =0;

    /// range kutta pour pendule 1
	k1 = m_dt*f_rk4(m_theta,m_theta2);
	y = k1.getp1();
	r = k1.getp2();
	k2 = m_dt*f_rk4(m_theta+0.5*y,m_theta2+0.5*r);
	y = k2.getp1();
	r = k2.getp2();
	k3 = m_dt*f_rk4(m_theta+0.5*y,m_theta2+0.5*r);
	y = k3.getp1();
	r = k3.getp2();
	k4 = m_dt*f_rk4(m_theta+y,m_theta2+r);

	m_theta=m_theta+((k1.getp1())+2*(k2.getp1())+2*(k3.getp1())+(k4.getp1()))*(1/6.);
	m_theta2=m_theta2+((k1.getp2())+2*(k2.getp2())+2*(k3.getp2())+(k4.getp2()))*(1/6.);
    //printf(" 1 -> %f\n",m_theta2.m_dot);
	/// euler pour pendule 2

	ke = m_dt*f_rk4(m_2theta,m_2theta2);
	y1 = ke.getp1();
	r1 = ke.getp2();

	m_2theta=m_2theta + y1;
	m_2theta2=m_2theta2 + r1;

	//printf(" Euler -> %f\n",m_2theta2.m_dot);

}


Dpair PendulumApplication::f_rk4(Pair tetha1,Pair tetha2)
{
	Pair p1,p2;
	btScalar m1 = 1;
	btScalar m2 = 1;
	btScalar l1,l2;
	btScalar g = -9.81;
	//btScalar lambda = 0.2;
    l1 = m_r.m_value;
    l2 = m_r2.m_value;

    btScalar x1,x2,y1,y2;

    x1 = tetha1.m_value;
    x2 = tetha1.m_dot;

    y1 = tetha2.m_value;
    y2 = tetha2.m_dot;

    //btScalar lit = (cos(x1 - y1)*cos(x1 - y1));
    btScalar lit = cos(2*x1 - 2*y1);

    btScalar div1 = l1*(2*m1+m2-m2*lit);

	p1.m_value = x2;
	p1.m_dot = (- g * (2 * m1 + m2) * sin (x1) - m2 * g * sin (x1 - 2 * y1) - 2 *sin (x1 - y1) * m2 * (y2*y2 *l2 + x2*x2 * l1 * cos(x1 - y1)) ) / div1 ;
	//p1.m_dot = - (((y2*y2)*m2*l2 * sin(x1-y1) )+ (m2*l1*x2*x2 * sin (x1 - y1)*cos (x1-y1) )+ ((m1+m2)*g * sin(x1)) - (m2*g*cos(x1-y1)*sin(y1))) / div1 ;
    //p1.m_dot =0.1;


    btScalar div2 = l2*(2*m1+m2-m2*lit);

    p2.m_value = y2;
    p2.m_dot = (2 * sin(x1 - y1) * (x2*x2 *l1 * (m1 + m2) + g*(m1 + m2) * cos (x1) + y2*y2 *l2 * m2 * cos(x1 - y1)))/ div2;
    //p2.m_dot = - (((y2*y2)*m2*l2 * sin(x1-y1) )+( x2*x2*l1*(m1+m2) * tan(x1 - y1) )+ (g*(m1+m2)* sin(x1) )- (g*(m1+m2)* sin(y1)*cos(x1 - y1))) / div2;
    //p2.m_dot =-0.2;

	return Dpair(p1,p2);

}

/*
void PendulumApplication::update_r(float dt)
{
    return;
}*/

btVector3 PendulumApplication::pol2cart(btScalar r, btScalar theta)
{
	btVector3 cartcoord(0.,0.,0.);
	cartcoord.setX(r*sin(theta));
	cartcoord.setY(r*cos(theta));
	cartcoord.setZ(0);
	return cartcoord;
}

void PendulumApplication::mouseMotionFunc( int x, int y)
{
    if(m_mouseButtons & 1)
    {
        float dty;
        dty = 0.01*(y-m_mouseOldY);
        m_r.m_value+= dty;
        m_r.m_dot = dty;
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
