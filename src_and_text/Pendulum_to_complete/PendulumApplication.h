#pragma once

#include "OpenGL/GlutDemoApplication.h"
#include "btBulletDynamicsCommon.h"

class Pair
{
	public:
	Pair(){ m_value=m_dot=0;}
	Pair(btScalar value, btScalar dot){ m_value=value;m_dot=dot;}
/* here you will overload operator * and +
*/
	Pair operator*(btScalar k) { return Pair(m_value*k,m_dot*k);}
    friend Pair operator*(btScalar k,Pair p) { return p*k;}
    Pair operator+(Pair p1) { return Pair(p1.m_value+m_value,p1.m_dot+m_dot);}
	btScalar m_value,m_dot;
};

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
	virtual void keyboardCallback(unsigned char key, int x, int y){if(key==27)exit(0);else DemoApplication::keyboardCallback(key,x,y);}
    virtual void mouseMotionFunc(int x, int y);

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
	btBroadphaseInterface*	m_broadphase;
	btCollisionDispatcher*	m_dispatcher;
	btConstraintSolver*	m_solver;
	btDefaultCollisionConfiguration* m_collisionConfiguration;
	btRigidBody * m_pendulumBody;


    void Integrator_rk4();
	Pair f_rk4(Pair k);
	btVector3 pol2cart(btScalar r, btScalar theta);

    Pair m_theta;
    Pair m_r;
    btScalar m_dt;
    btScalar temp;
};

