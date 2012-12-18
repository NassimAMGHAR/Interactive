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

	//buildDoublePendulumRigid();
	//buildDoublePendulumSoft();
	buildFlyingTrapeze();

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
    btTransform trans;
    trans.setIdentity();
    trans.setOrigin(btVector3(0,10,0));

    btSphereShape * sphere = new btSphereShape(1.);
    btRigidBody * pendule = localCreateRigidBody(1.,trans,sphere);
    m_cameraTarget = pendule;
    m_collisionShapes.push_back(sphere);

    btPoint2PointConstraint * constraint = new btPoint2PointConstraint (*pendule, btVector3(0,5,0));
    m_dynamicsWorld->addConstraint(constraint);
    constraint->setBreakingImpulseThreshold(5.);

    pendule->applyCentralImpulse(btVector3(3,0,0));

    ///------------------------------------------------------
    //btTransform trans2;
    //trans2.setIdentity();
    //trans2.setOrigin(btVector3(0,1,0));
    trans.setOrigin(btVector3(0,5,0));
    //btSphereShape * sphere2 = new btSphereShape(1.);
    btRigidBody * pendule2 = localCreateRigidBody(1.,trans,sphere);
    //m_collisionShapes.push_back(sphere2);
    btVector3 v = pendule->getWorldTransform().getOrigin() - trans.getOrigin();
    btPoint2PointConstraint * constraint2 = new btPoint2PointConstraint (*pendule, *pendule2, btVector3(0,0,0), v);
    m_dynamicsWorld->addConstraint(constraint2);
    constraint2->setDbgDrawSize(2.f);
}

void PendulumApplication::buildDoublePendulumSoft()
{
    //TODO
    btTransform trans;
    trans.setIdentity();
    trans.setOrigin(btVector3(0,10,0));

    btSphereShape * sphere = new btSphereShape(1.);
    btRigidBody * pendule = localCreateRigidBody(1.,trans,sphere);
    m_cameraTarget = pendule;
    m_collisionShapes.push_back(sphere);

    btSoftBody * corde =
        btSoftBodyHelpers::CreateRope(m_softBodyWorldInfo, btVector3(0,15,0),pendule->getWorldTransform().getOrigin(),7,1);

    corde->setTotalMass(1.);
    this->getSoftDynamicsWorld()->addSoftBody(corde,1,1) ;

    //corde->appendAnchor(1,corde,false,1);
    m_corde = corde;

    corde->appendAnchor(corde->m_nodes.size()-1,pendule,btVector3(0,0.5,0),true,1);

    btVector3 beg = corde->m_nodes[0].m_x;
    btVector3 end = corde->m_nodes[corde->m_nodes.size()-1].m_x;

    m_size_corde = beg - end;


    ///------------------------------------------------------

    trans.setOrigin(btVector3(0,5,0));

    btRigidBody * pendule2 = localCreateRigidBody(1.,trans,sphere);

    btVector3 v = pendule->getWorldTransform().getOrigin() - trans.getOrigin();

    btSoftBody * corde2 =
        btSoftBodyHelpers::CreateRope(m_softBodyWorldInfo, pendule->getWorldTransform().getOrigin(),pendule2->getWorldTransform().getOrigin(),7,0);

    m_corde2 = corde2;

    beg = corde2->m_nodes[0].m_x;
    end = corde2->m_nodes[corde2->m_nodes.size()-1].m_x;

    m_size_corde2 = beg - end;

    corde2->setTotalMass(1.);
    this->getSoftDynamicsWorld()->addSoftBody(corde2,1,1) ;

    corde2->appendAnchor(0,pendule,btVector3(0,-0.5,0),true,1);

    corde2->appendAnchor(corde2->m_nodes.size()-1,pendule2,btVector3(0,0.5,0),true,1);

}

void PendulumApplication::buildFlyingTrapeze()
{
    /// Actu

    RagDoll * ragdoll = new RagDoll (m_dynamicsWorld, btVector3(0,12,0));

    btRigidBody* right_hand = ragdoll->getBodyPart(ragdoll->BODYPART_RIGHT_LOWER_ARM);
    btRigidBody* left_hand  = ragdoll->getBodyPart(ragdoll->BODYPART_LEFT_LOWER_ARM);

    btSoftBody * corde =
        btSoftBodyHelpers::CreateRope(m_softBodyWorldInfo, btVector3(2,17,0),right_hand->getWorldTransform().getOrigin(),7,1);

    btSoftBody * corde2 =
        btSoftBodyHelpers::CreateRope(m_softBodyWorldInfo, btVector3(-2,17,0),left_hand->getWorldTransform().getOrigin(),7,1);

    corde->appendAnchor(corde->m_nodes.size()-1,right_hand,btVector3(0,0.1,0),true,1);
    corde2->appendAnchor(corde2->m_nodes.size()-1,left_hand,btVector3(0,0.1,0),true,1);

    corde->setTotalMass(8.);
    this->getSoftDynamicsWorld()->addSoftBody(corde,1,1) ;

    corde2->setTotalMass(8.);
    this->getSoftDynamicsWorld()->addSoftBody(corde2,1,1) ;

    m_corde = corde;
    m_corde2 = corde2;

    /// fillet
    btVector3 c_00(-3,10,-3);
    btVector3 c_10(3,10,-3);
    btVector3 c_01(-3,10,3);
    btVector3 c_11(3,10,3);

    btSoftBody * fillet =
        btSoftBodyHelpers::CreatePatch(m_softBodyWorldInfo,c_00,c_10,c_01,c_11,20,20,15,true);

    //fillet->appendAnchor(fillet->m_nodes.size()-1,right_hand,false,1);

    fillet->setTotalMass(150.);
    getSoftDynamicsWorld()->addSoftBody(fillet,1,1) ;

    fillet->generateBendingConstraints(2,fillet->appendMaterial());

    fillet->getCollisionShape()->setMargin (0.6)	;
    struct myFn :public btSoftBody::ImplicitFn
    {

    };

    //myFn * ifn();

    //fillet->refine(,2,true);
   // applyTorqueImpulse()

    /// camera
    btRigidBody* head  = ragdoll->getBodyPart(ragdoll->BODYPART_HEAD);
    m_cameraTarget = head;

    m_ragdall = ragdoll;
}

void PendulumApplication::exitPhysics()
{
	//cleanup in the reverse order of creation/initialization
	//remove the rigid and soft bodies from the dynamics world and delete them
	if(m_ragdall == NULL)
        delete m_ragdall;
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

    /// follow the object
    if(m_cameraTarget != NULL)
    m_cameraTargetPosition = m_cameraTarget->getWorldTransform().getOrigin();
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

    /*
    if(m_corde != NULL && m_corde2 != NULL)
    {
    /// contrainte p1
    btVector3 beg = m_corde->m_nodes[0].m_x;
    btVector3 end = m_corde->m_nodes[m_corde->m_nodes.size()-1].m_x;

    btVector3 res = beg - end;

    btScalar mid = m_corde->m_nodes.size()/2;

    if(res.x() > m_size_corde.x()+4 || res.y() > m_size_corde.y()+4||res.z() > m_size_corde.z()+4)
    m_corde->cutLink(mid,mid+1,0.5);
    /// contrainte p2
    beg = m_corde2->m_nodes[0].m_x;
    end = m_corde2->m_nodes[m_corde2->m_nodes.size()-1].m_x;

    res = beg - end;

    mid = m_corde2->m_nodes.size()/2;

    if(res.x() > m_size_corde2.x()+3 || res.y() > m_size_corde2.y()+3 || res.z() > m_size_corde2.z()+3)
    m_corde2->cutLink(mid,mid+1,0.5);
    }
    */
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

void PendulumApplication::drop_dead()
{
	m_corde->cutLink(m_corde->m_nodes.size()-2,m_corde->m_nodes.size()-1,0.5);
	m_corde2->cutLink(m_corde2->m_nodes.size()-2,m_corde2->m_nodes.size()-1,0.5);
}


void PendulumApplication::move_legs(bool left)
{

    btRigidBody* right_leg = m_ragdall->getBodyPart(m_ragdall->BODYPART_RIGHT_LOWER_LEG);
    btRigidBody* left_leg  = m_ragdall->getBodyPart(m_ragdall->BODYPART_LEFT_LOWER_LEG);
    /// move the legs
    if(left)
    {
        right_leg->applyTorqueImpulse(btVector3(2,0,0));
        left_leg->applyTorqueImpulse(btVector3(2,0,0));
    }
    else
    {
        right_leg->applyTorqueImpulse(btVector3(-2,0,0));
        left_leg->applyTorqueImpulse(btVector3(-2,0,0));
    }

}

void PendulumApplication::keyboardCallback(unsigned char key, int x, int y)
{
    //TODO : catch the key event
    switch(key)
    {
        case(27):exitPhysics();
        exit(0);
        break;
        case(13):
        drop_dead();
        break;
        case(66): // b B left
        case(98):
        move_legs(true);
        break;
        case(78): // n N right
        case(110):
        move_legs(false);
        break;
        default:DemoApplication::keyboardCallback(key,x,y);
    }
}
