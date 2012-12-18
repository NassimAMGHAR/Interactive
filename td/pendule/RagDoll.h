#pragma once

/*
Bullet Continuous Collision Detection and Physics Library
Class extract from the Ragdoll Demo by Marten Svanfeldt
*/

#include "LinearMath/btIDebugDraw.h"
#include "btBulletDynamicsCommon.h"

#define SIMD_PI_2      (SIMD_PI * btScalar(0.5))
#define SIMD_PI_4      (SIMD_PI * btScalar(0.25))
#define CONSTRAINT_DEBUG_SIZE 0.2f

class RagDoll
{
public:

	enum BodyPart
	{
		BODYPART_PELVIS = 0,
		BODYPART_SPINE,
		BODYPART_HEAD,

		BODYPART_LEFT_UPPER_LEG,
		BODYPART_LEFT_LOWER_LEG,

		BODYPART_RIGHT_UPPER_LEG,
		BODYPART_RIGHT_LOWER_LEG,

		BODYPART_LEFT_UPPER_ARM,
		BODYPART_LEFT_LOWER_ARM,

		BODYPART_RIGHT_UPPER_ARM,
		BODYPART_RIGHT_LOWER_ARM,

		BODYPART_COUNT
	};

	enum Joint
	{
		JOINT_PELVIS_SPINE = 0,
		JOINT_SPINE_HEAD,

		JOINT_LEFT_HIP,
		JOINT_LEFT_KNEE,

		JOINT_RIGHT_HIP,
		JOINT_RIGHT_KNEE,

		JOINT_LEFT_SHOULDER,
		JOINT_LEFT_ELBOW,

		JOINT_RIGHT_SHOULDER,
		JOINT_RIGHT_ELBOW,

		JOINT_COUNT
	};

	RagDoll (btDynamicsWorld* ownerWorld, const btVector3& positionOffset);
	virtual	~RagDoll ();
	btRigidBody* getBodyPart(const BodyPart & bodyPart);

private:
	btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape);

	btDynamicsWorld* m_ownerWorld;
	btCollisionShape* m_shapes[BODYPART_COUNT];
	btRigidBody* m_bodies[BODYPART_COUNT];
	btTypedConstraint* m_joints[JOINT_COUNT];
};