#pragma once

#include "OpenGL/GlutDemoApplication.h"
#include <LinearMath/btAlignedObjectArray.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletDynamics/ConstraintSolver/btHingeConstraint.h>

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

//engine power
const int maxMotorImpulse = 30; //<= change this if you want
const int maxMotorVelocity = 5; //<= change this if you want

const int maxProxies = 32766;

class FourWheelDrive : public GlutDemoApplication
{
public:
	FourWheelDrive(void);
	~FourWheelDrive(void){	exitPhysics();}

	void initPhysics();

	void exitPhysics();

	void renderme();

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void keyboardCallback(unsigned char key, int x, int y);

	virtual void buildScene();

	virtual void clientResetScene();

	btRigidBody * createWheel(const btVector3& pos);

	virtual const btSoftRigidDynamicsWorld*	getSoftDynamicsWorld() const
	{
		///just make it a btSoftRigidDynamicsWorld please
		///or we will add type checking
		return (btSoftRigidDynamicsWorld*) m_dynamicsWorld;
	}

	virtual btSoftRigidDynamicsWorld*	getSoftDynamicsWorld()
	{
		///just make it a btSoftRigidDynamicsWorld please
		///or we will add type checking
		return (btSoftRigidDynamicsWorld*) m_dynamicsWorld;
	}

private:

	btRigidBody * m_rims[4];
	btHingeConstraint * m_hinges[4];
	btScalar m_motorImpulse;
	btRigidBody * m_cameraTarget;

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
	btBroadphaseInterface*	m_broadphase;

	btSoftBodyWorldInfo	m_softBodyWorldInfo;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

};
