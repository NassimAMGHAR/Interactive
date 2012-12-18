#pragma once

#include "OpenGL/GlutDemoApplication.h"
#include "btBulletDynamicsCommon.h"

class Pair
{
	public:
	Pair(){ m_value=m_dot=0;}
	Pair(btScalar value, btScalar dot){ m_value=value;m_dot=dot;}
	Pair operator*(btScalar k) { return Pair(k*m_value,k*m_dot);}
	Pair operator+(Pair p) { return Pair(p.m_value+m_value,p.m_dot+m_dot);}

	friend Pair operator*(btScalar k,Pair p) { return p*k;}
	btScalar m_value,m_dot;
};
class Dpair
{
    public:
    Dpair(){ m_p1 = Pair(); m_p2 = Pair();}
    Dpair(Pair pe1,Pair pe2) { m_p1 = pe1; m_p2 = pe2 ;}

    Pair getp1(){ return m_p1; }
    Pair getp2(){ return m_p2; }

    Dpair operator*(btScalar k) { return Dpair(k*m_p1,k*m_p2);}

    friend Dpair operator*(btScalar k,Dpair p) { return p*k;}
    Pair m_p1,m_p2;

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
    //pondule 1
	btRigidBody * m_pendulumBody;
    btRigidBody * m_pendulumBody2;
    //pondule 2
	btRigidBody * m_pendulum2Body;
    btRigidBody * m_pendulum2Body2;

    void Integrator_rk4();
	Dpair f_rk4(Pair k, Pair j);
	btVector3 pol2cart(btScalar r, btScalar theta);
    /// pondule 1 avec range kutta
    Pair m_theta;
    Pair m_r;
    Pair m_theta2;
    Pair m_r2;
    //pos
    btVector3 m_pos;
    /// pondule 2 avec euler
    Pair m_2theta;
    Pair m_2r;
    Pair m_2theta2;
    Pair m_2r2;
    // pos
    btVector3 m_pos2;


    btScalar m_dt;
};

