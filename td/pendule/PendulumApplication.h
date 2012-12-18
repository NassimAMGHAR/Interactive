#pragma once

#include "GlutDemoApplication.h"
#include "btBulletDynamicsCommon.h"
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBody.h>
#include "RagDoll.h"

/// final code in /net/K3/enseignant/nagirard/public

class PendulumApplication : public GlutDemoApplication
{
public:
	PendulumApplication(void);
	~PendulumApplication(void);

	void initPhysics();

	void exitPhysics();

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();
	virtual void clientResetScene();

	virtual void keyboardCallback(unsigned char key, int x, int y);

	void buildDoublePendulumRigid();
	void buildDoublePendulumSoft();
	void buildFlyingTrapeze();
    void drop_dead();
    void move_legs(bool left);

	virtual void renderme();

	///just make it a btSoftRigidDynamicsWorld please
	///or we will add type checking
	btSoftRigidDynamicsWorld*	getSoftDynamicsWorld() const
	{
		return (btSoftRigidDynamicsWorld*) m_dynamicsWorld;
	}

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	btSoftBodyWorldInfo	m_softBodyWorldInfo;

    //TODO:  add some attributes or member functions

    btVector3 m_size_corde;
    btSoftBody * m_corde;

    btVector3 m_size_corde2;
    btSoftBody * m_corde2;

    RagDoll * m_ragdall;

    btRigidBody * m_cameraTarget;
};
