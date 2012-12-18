#include "FourWheelDrive.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btIDebugDraw.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "TorusMesh.h"

FourWheelDrive::FourWheelDrive(void)
{
	m_debugMode |= btIDebugDraw::DBG_NoHelpText;
}

void FourWheelDrive::initPhysics()
{
	// Setup the basic world
	setTexturing(true);
	setShadows(true);





	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	m_softBodyWorldInfo.m_dispatcher = m_dispatcher;

	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax,maxProxies);
	m_softBodyWorldInfo.m_broadphase = m_broadphase;

	m_solver = new btSequentialImpulseConstraintSolver;

	m_dynamicsWorld = new btSoftRigidDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));
	m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);
	m_softBodyWorldInfo.m_sparsesdf.Initialize();

	m_azi = 135;
	m_ele = 30;
	setCameraDistance(15.f);

	clientResetScene();
}

void FourWheelDrive::buildScene()
{
	// Setup a big ground box

	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(10.),btScalar(200.)));
	m_collisionShapes.push_back(groundShape);
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-10,0));
	localCreateRigidBody(btScalar(0.),groundTransform,groundShape);


	m_motorImpulse = 0;
	float width = 2.2;
	float length = 3.5;
	float height = 4;
	float space = 1;

	//Create Wheels
	btVector3 poss [4] = {btVector3(width+space,height,length),btVector3(-width-space,height,length),btVector3(-width-space,height,-length),btVector3(width+space,height,-length)};
	for(int i=0;i<4;i++)
		m_rims[i] = createWheel(poss[i]);

	//Create Frame
	btBoxShape*	pshp=new btBoxShape(btVector3(width,0.2,length+0.5));
	btRigidBody* frame=localCreateRigidBody(50,btTransform(btQuaternion(0,0,0),btVector3(0,height,0)),pshp);

	m_cameraTarget = frame;
	//Constraint wheels on frame with hinges
	btVector3 pIA [4] = {btVector3(0,space,0),btVector3(0,-space,0),btVector3(0,-space,0),btVector3(0,space,0)};
	btVector3 pIB [4] = {btVector3(width,0,length),btVector3(-width,0,length),btVector3(-width,0,-length),btVector3(width,0,-length)};
	for(int i=0;i<4;i++)
	{
		m_hinges[i]= new btHingeConstraint(*m_rims[i],*frame,pIA[i],pIB[i],btVector3(0,1,0),btVector3(0,0,0));
		m_dynamicsWorld->addConstraint(m_hinges[i]);
		m_hinges[i]->enableAngularMotor(true,maxMotorVelocity,m_motorImpulse);
		m_hinges[i]->setBreakingImpulseThreshold(90);
	}


	//Create Obstacles
	float step =0.1;
	float size = 5;
	float stepLength = 50;
	float stepHeight = 0;
	float offset =
	height = 0;
	for(int i =0;i<10;i++)
	{
		stepHeight = (i+1)*step;
		if(i<9)
			localCreateRigidBody(0,btTransform(btQuaternion(0,0,0),btVector3(0,height+stepHeight,-size*i-25)),new btBoxShape(btVector3(stepLength,stepHeight,size)));
		else
			localCreateRigidBody(0,btTransform(btQuaternion(0,0,0),btVector3(0,height+stepHeight,-size*i-50)),new btBoxShape(btVector3(stepLength,stepHeight,30)));

		height+= stepHeight*2;
	}
}

btRigidBody * FourWheelDrive::createWheel(const btVector3& pos)
{
	//Rim
	btCylinderShape*	pshp=new btCylinderShape(btVector3(1.6f,0.3,10));
	btRigidBody*		rim=localCreateRigidBody(10,btTransform(btQuaternion(0,0,SIMD_HALF_PI),pos),pshp);

    //TODO
	//Add a Tire

	btSoftBody* tire =
        btSoftBodyHelpers::CreateFromTriMesh(m_softBodyWorldInfo,gVertices,&gIndices[0][0],NUM_TRIANGLES,false);

    getSoftDynamicsWorld()->addSoftBody(tire);

    //tire->appendAnchor(0,rim,btVector3(0,0.1,0),true,1);

    //tire->
    //tire->
    btVector3 a = btVector3(0,0,SIMD_HALF_PI);
    btSoftBody::Material*	pm=tire->appendMaterial();
    pm->m_kAST = 1;
    pm->m_kLST = 1;
   // pm->m_kVST = 100;
    pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
    tire->generateBendingConstraints(2,pm);
	tire->m_cfg.piterations	=	1;
	tire->m_cfg.collisions	=1;
	//for (int i= 0 ; i<tire->m_nodes.size();i+=4)
    //    tire->appendAnchor(0,rim,btVector3(1,0,i/100),true,1);

	tire->randomizeConstraints();
	tire->scale(btVector3(1,1.5,1));
	tire->rotate(btQuaternion(a[0],a[1],a[2]));
	tire->translate(pos);
    //tire->setMass(1,0.01);
	tire->setTotalMass(2,false);
	tire->generateClusters(64);

    btSoftBody::LJoint::Specs	lspecs;
	lspecs.cfm		=	1;
	lspecs.erp		=	1;
	lspecs.position	=	pos;

    tire->appendLinearJoint(lspecs,rim);

    btSoftBody::AJoint::Specs	aspecs;
	aspecs.cfm		=	1;
	aspecs.erp		=	1;
	aspecs.axis		=	btVector3(1,0,0);

    tire->appendAngularJoint(aspecs,rim);

    aspecs.cfm		=	1;
	aspecs.erp		=	1;
	aspecs.axis		=	btVector3(0,1,0);
	tire->appendAngularJoint(aspecs,rim);


	return rim;
}

void FourWheelDrive::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();

	float minFPS = 1000000.f/60.f;
	if (ms > minFPS)
		ms = minFPS;

	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);

		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}

	m_cameraTargetPosition = m_cameraTarget->getWorldTransform().getOrigin();
	renderme();

	glutSwapBuffers();
}


void FourWheelDrive::displayCallback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glutSwapBuffers();
}

void FourWheelDrive::keyboardCallback(unsigned char key, int x, int y)
{
	int i;
	switch (key)
	{
	case('S'):
	case('s'):	if(m_motorImpulse==0)
					m_motorImpulse=maxMotorImpulse;
				else
					m_motorImpulse=0;
				break;
		default:
			DemoApplication::keyboardCallback(key, x, y);
	}

	for( i=0;i<4;i++)
		m_hinges[i]->setMaxMotorImpulse(m_motorImpulse);
}


void FourWheelDrive::renderme()
{
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	m_dynamicsWorld->debugDrawWorld();

	btSoftRigidDynamicsWorld* softWorld = (btSoftRigidDynamicsWorld*)m_dynamicsWorld;

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

void FourWheelDrive::exitPhysics()
{

	int i;

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them

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

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;
}

void FourWheelDrive::clientResetScene()
{
	DemoApplication::clientResetScene();
	/* Clean up	*/
	for(int i=m_dynamicsWorld->getNumCollisionObjects()-1;i>=0;i--)
	{
		btCollisionObject*	obj=m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody*		body=btRigidBody::upcast(obj);
		if(body&&body->getMotionState())
		{
			delete body->getMotionState();
		}
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

	m_softBodyWorldInfo.m_sparsesdf.Reset();

	buildScene();

	m_softBodyWorldInfo.air_density		=	(btScalar)1.2;
	m_softBodyWorldInfo.water_density	=	0;
	m_softBodyWorldInfo.water_offset	=	0;
	m_softBodyWorldInfo.water_normal	=	btVector3(0,0,0);
	m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);

}
