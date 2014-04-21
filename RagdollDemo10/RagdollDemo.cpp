/*
Bullet Continuous Collision Detection and Physics Library
Ragdoll Demo
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt
*/

#define CONSTRAINT_DEBUG_SIZE 0.2f


#include "btBulletDynamicsCommon.h"
#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"

#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"
#include "RagdollDemo.h"
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */


// Enrico: Shouldn't these three variables be real constants and not defines?

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif

class RagDoll
{
	enum
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
        BODYPART_BOX,

		BODYPART_COUNT
        
	};

	enum
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

	btDynamicsWorld* m_ownerWorld;
	btCollisionShape* m_shapes[BODYPART_COUNT];
	btRigidBody* m_bodies[BODYPART_COUNT];
	btTypedConstraint* m_joints[JOINT_COUNT];

	btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
	{
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			shape->calculateLocalInertia(mass,localInertia);

		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		m_ownerWorld->addRigidBody(body);

		return body;
	}
    
    /*void CreateBox(int index,
                   double x, double y, double z,
                   double length, double width, double height)
    {
        
        btCollisionShape* box;
        btRigidBody* boxBody;
        
        // Setup the geometry
        box = new btBoxShape(btVector3(length, width, height));
        
        //btCollisionShape* geom[9];
        //geom[index]= box;
        
        // Setup all the rigid bodies
        btTransform offset;
        offset.setIdentity();
        
        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(btVector3(x, y, z));
        boxBody = localCreateRigidBody(btScalar(1.), offset*transform, box);
        //btRigidBody* body[9];
        //body[index]= boxBody;
        
        
    }
*/
    
public:
	RagDoll (btDynamicsWorld* ownerWorld, const btVector3& positionOffset)
		: m_ownerWorld (ownerWorld)
	{
        
        
  
        
        // Setup the geometry
		m_shapes[BODYPART_PELVIS] = new btCapsuleShape(btScalar(0.15), btScalar(0.20));
		m_shapes[BODYPART_SPINE] = new btCapsuleShape(btScalar(0.15), btScalar(0.28));
		m_shapes[BODYPART_HEAD] = new btCapsuleShape(btScalar(0.40), btScalar(0.20));
		m_shapes[BODYPART_LEFT_UPPER_LEG] = new btCapsuleShape(btScalar(0.07), btScalar(0.45));
		m_shapes[BODYPART_LEFT_LOWER_LEG] = new btCapsuleShape(btScalar(0.05), btScalar(0.37));
		m_shapes[BODYPART_RIGHT_UPPER_LEG] = new btCapsuleShape(btScalar(0.07), btScalar(0.45));
		m_shapes[BODYPART_RIGHT_LOWER_LEG] = new btCapsuleShape(btScalar(0.05), btScalar(0.37));
		m_shapes[BODYPART_LEFT_UPPER_ARM] = new btCapsuleShape(btScalar(0.05), btScalar(0.33));
		m_shapes[BODYPART_LEFT_LOWER_ARM] = new btCapsuleShape(btScalar(0.04), btScalar(0.25));
		m_shapes[BODYPART_RIGHT_UPPER_ARM] = new btCapsuleShape(btScalar(0.05), btScalar(0.33));
		m_shapes[BODYPART_RIGHT_LOWER_ARM] = new btCapsuleShape(btScalar(0.04), btScalar(0.25));

		// Setup all the rigid bodies
		btTransform offset; offset.setIdentity();
		offset.setOrigin(positionOffset);

		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(1.), btScalar(0.)));
		m_bodies[BODYPART_PELVIS] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_PELVIS]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(1.2), btScalar(0.)));
		m_bodies[BODYPART_SPINE] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_SPINE]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(1.6), btScalar(0.)));
		m_bodies[BODYPART_HEAD] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_HEAD]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.18), btScalar(0.65), btScalar(0.)));
		m_bodies[BODYPART_LEFT_UPPER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_UPPER_LEG]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.18), btScalar(0.2), btScalar(0.)));
		m_bodies[BODYPART_LEFT_LOWER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_LOWER_LEG]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.18), btScalar(0.65), btScalar(0.)));
		m_bodies[BODYPART_RIGHT_UPPER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_UPPER_LEG]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.18), btScalar(0.2), btScalar(0.)));
		m_bodies[BODYPART_RIGHT_LOWER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_LOWER_LEG]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.35), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,M_PI_2);
		m_bodies[BODYPART_LEFT_UPPER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_UPPER_ARM]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.7), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,M_PI_2);
		m_bodies[BODYPART_LEFT_LOWER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_LOWER_ARM]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.35), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,-M_PI_2);
		m_bodies[BODYPART_RIGHT_UPPER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_UPPER_ARM]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.7), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,-M_PI_2);
		m_bodies[BODYPART_RIGHT_LOWER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_LOWER_ARM]);

		// Setup some damping on the m_bodies
		for (int i = 0; i < BODYPART_COUNT; ++i)
		{
			m_bodies[i]->setDamping(0.05, 0.85);
			m_bodies[i]->setDeactivationTime(0.8);
			m_bodies[i]->setSleepingThresholds(1.6, 2.5);
		}

		// Now setup the constraints
		btHingeConstraint* hingeC;
		btConeTwistConstraint* coneC;

		btTransform localA, localB;

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.15), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.15), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_SPINE], localA, localB);
		hingeC->setLimit(btScalar(-M_PI_4), btScalar(M_PI_2));
		m_joints[JOINT_PELVIS_SPINE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_PELVIS_SPINE], true);


		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,M_PI_2); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.30), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_HEAD], localA, localB);
		coneC->setLimit(M_PI_4, M_PI_4, M_PI_2);
		m_joints[JOINT_SPINE_HEAD] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_SPINE_HEAD], true);


		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,-M_PI_4*5); localA.setOrigin(btVector3(btScalar(-0.18), btScalar(-0.10), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,-M_PI_4*5); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.225), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_LEFT_UPPER_LEG], localA, localB);
		coneC->setLimit(M_PI_4, M_PI_4, 0);
		m_joints[JOINT_LEFT_HIP] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_HIP], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.185), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_LEFT_UPPER_LEG], *m_bodies[BODYPART_LEFT_LOWER_LEG], localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_LEFT_KNEE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_KNEE], true);


		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,M_PI_4); localA.setOrigin(btVector3(btScalar(0.18), btScalar(-0.10), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_4); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.225), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_RIGHT_UPPER_LEG], localA, localB);
		coneC->setLimit(M_PI_4, M_PI_4, 0);
		m_joints[JOINT_RIGHT_HIP] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_HIP], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.185), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_RIGHT_UPPER_LEG], *m_bodies[BODYPART_RIGHT_LOWER_LEG], localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_RIGHT_KNEE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_KNEE], true);


		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,M_PI); localA.setOrigin(btVector3(btScalar(-0.2), btScalar(0.15), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_LEFT_UPPER_ARM], localA, localB);
		coneC->setLimit(M_PI_2, M_PI_2, 0);
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_joints[JOINT_LEFT_SHOULDER] = coneC;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_SHOULDER], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_LEFT_UPPER_ARM], *m_bodies[BODYPART_LEFT_LOWER_ARM], localA, localB);
//		hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_LEFT_ELBOW] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_ELBOW], true);



		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,0); localA.setOrigin(btVector3(btScalar(0.2), btScalar(0.15), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_RIGHT_UPPER_ARM], localA, localB);
		coneC->setLimit(M_PI_2, M_PI_2, 0);
		m_joints[JOINT_RIGHT_SHOULDER] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_SHOULDER], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_RIGHT_UPPER_ARM], *m_bodies[BODYPART_RIGHT_LOWER_ARM], localA, localB);
//		hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_RIGHT_ELBOW] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_ELBOW], true);
	}

	virtual	~RagDoll ()
	{
		int i;

		// Remove all constraints
		for ( i = 0; i < JOINT_COUNT; ++i)
		{
			m_ownerWorld->removeConstraint(m_joints[i]);
			delete m_joints[i]; m_joints[i] = 0;
		}

		// Remove all bodies and shapes
		for ( i = 0; i < BODYPART_COUNT; ++i)
		{
			m_ownerWorld->removeRigidBody(m_bodies[i]);
			
			delete m_bodies[i]->getMotionState();

			delete m_bodies[i]; m_bodies[i] = 0;
			delete m_shapes[i]; m_shapes[i] = 0;
		}
	}
};
static RagdollDemo* ragdollDemo;

bool myContactProcessedCallback(btManifoldPoint& cp, void* body0, void* body1)
{
    int *ID1, *ID2;
    btCollisionObject* o1 = static_cast<btCollisionObject*>(body0);
    btCollisionObject* o2 = static_cast<btCollisionObject*>(body1);
    
    //int groundID = 9;
    
    ID1 = static_cast<int*>(o1->getUserPointer());
    ID2 = static_cast<int*>(o2->getUserPointer());

    //printf("ID1 = %d, ID2 = %d\n", *ID1, *ID2);
    ragdollDemo->touches[*ID1] = 1;
    ragdollDemo->touches[*ID2] = 1;
    ragdollDemo->touchPoints[*ID1] = cp.m_positionWorldOnB;
    ragdollDemo->touchPoints[*ID2] = cp.m_positionWorldOnB;
    
    /* Your code will go here. See the next step. */
    return false;
}


void RagdollDemo::initPhysics()
{
	ragdollDemo = this;
    // Setup the basic world
    gContactProcessedCallback = myContactProcessedCallback;
	setTexturing(true);
	setShadows(true);

	setCameraDistance(btScalar(5.));

	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);

	m_solver = new btSequentialImpulseConstraintSolver;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	//m_dynamicsWorld->getDispatchInfo().m_useConvexConservativeDistanceUtil = true;
	//m_dynamicsWorld->getDispatchInfo().m_convexConservativeDistanceThreshold = 0.01f;



	// Setup a big ground box
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(10.),btScalar(200.)));
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-10,0));

#define CREATE_GROUND_COLLISION_OBJECT 1
#ifdef CREATE_GROUND_COLLISION_OBJECT
		btCollisionObject* fixedGround = new btCollisionObject();
		fixedGround->setCollisionShape(groundShape);
		fixedGround->setWorldTransform(groundTransform);
        fixedGround->setUserPointer(&IDs[9]);
		m_dynamicsWorld->addCollisionObject(fixedGround);
#else
		localCreateRigidBody(btScalar(0.),groundTransform,groundShape);
#endif //CREATE_GROUND_COLLISION_OBJECT

	}

	// Spawn one ragdoll
	btVector3 startOffset(1,0.5,0);
	//spawnRagdoll(startOffset);
	startOffset.setValue(-1,0.5,0);
	//spawnRagdoll(startOffset);
    
    CreateBox(0, 0., 2., 0., 1., 0.2, 1.); // Create the box
    
    //leg components
    CreateCylinder(1, 2. , 2., 0., .2, 1., 0., 0., M_PI_2); //xleft horizontal
    CreateCylinder(2, -2., 2., 0., .2, 1., 0., 0., M_PI_2); //xright (negative) horizontal
    CreateCylinder(3, 0, 2., 2., .2, 1., M_PI_2, 0., 0.);    //zpositive (into screen)
    CreateCylinder(4, 0, 2., -2., .2, 1., M_PI_2, 0., 0.);   //znegative (out of screen)
    CreateCylinder(5, 3.0, 1., 0., .2, 1., 0., 0., 0.);       //xleft vertical
    CreateCylinder(6, -3.0, 1., 0., .2, 1., 0., 0., 0.);      //xright vertical
    CreateCylinder(7, 0., 1., 3.0, .2, 1., 0., 0., 0.);     //zpositive vertical
    CreateCylinder(8, 0., 1., -3.0, .2, 1., 0., 0., 0.);    //znegative vertical
    
    //The axisworldtolocal defines a vector perpendicular to the plane that you want the bodies to rotate in
    
    
    //hinge the legs together
    //xnegative -- right
    //two bodies should rotate in y-plane through x--give axisworldtolocal it a z vector
    btVector3 local2 = PointWorldToLocal(2, btVector3(-3., 2., 0));
    btVector3 local6 = PointWorldToLocal(6, btVector3(-3., 2., 0));
    btVector3 axis2 = AxisWorldToLocal(2, btVector3(0., 0., -1.));
    btVector3 axis6 = AxisWorldToLocal(6, btVector3( 0., 0., -1.));
    
    
    CreateHinge(0, *body[2], *body[6], local2, local6, axis2, axis6);
    joints[0]->setLimit(btScalar(-M_PI_4-M_PI_2), btScalar(-M_PI_4));
    
   
    //positive -- left
    //give axisworldtolocal a z vector
    btVector3 local1 = PointWorldToLocal(1, btVector3(3., 2., 0));
    btVector3 local5 = PointWorldToLocal(5, btVector3(3., 2., 0));
    btVector3 axis1 = AxisWorldToLocal(1, btVector3( 0., 0., 1.));
    btVector3 axis5 = AxisWorldToLocal(5, btVector3(0., 0., 1.));
    
    CreateHinge(1, *body[1], *body[5], local1, local5, axis1, axis5);
    joints[1]->setLimit(btScalar(M_PI_4), btScalar(M_PI_2+M_PI_4));
    
    //zpositive -- back
    //rotates in y-plane through z--give it an x vector
    btVector3 local3 = PointWorldToLocal(3, btVector3(0, 2., 3.));
    btVector3 local7 = PointWorldToLocal(7, btVector3(0, 2., 3.));
    btVector3 axis3 = AxisWorldToLocal(3, btVector3(-1., 0., 0.));
    btVector3 axis7 = AxisWorldToLocal(7, btVector3(-1., 0., 0.));
    
    CreateHinge(2, *body[3], *body[7], local3, local7, axis3, axis7);
    joints[2]->setLimit(btScalar(-M_PI_4-M_PI_2), btScalar(-M_PI_4));
    
    //znegative -- front
    
    btVector3 local4 = PointWorldToLocal(4, btVector3(0, 2., -3.));
    btVector3 local8 = PointWorldToLocal(8, btVector3(0, 2., -3.));
    btVector3 axis4 = AxisWorldToLocal(4, btVector3(1., 0., 0.));
    btVector3 axis8 = AxisWorldToLocal(8, btVector3(1., 0., 0.));
    
    CreateHinge(3, *body[4], *body[8], local4, local8, axis4, axis8);
    joints[3]->setLimit(btScalar(M_PI_4), btScalar(M_PI_2+M_PI_4));
    
    //hinge the legs to the body
    //xright to main
    btVector3 localHinge2 = PointWorldToLocal(2, btVector3(-1, 2., 0));
    btVector3 mainHinge2 = PointWorldToLocal(0, btVector3(-1, 2., 0));
    btVector3 localAxis2 = AxisWorldToLocal(2, btVector3(0., 0., -1.));
    btVector3 mainAxis2 = AxisWorldToLocal(0, btVector3( 0., 0., -1.));
    
    CreateHinge(4, *body[0], *body[2], mainHinge2, localHinge2, mainAxis2, localAxis2);
    joints[4]->setLimit(btScalar(M_PI_4), btScalar(M_PI_2+M_PI_4));
    
    
    //xleft to main
    btVector3 localHinge1 = PointWorldToLocal(1, btVector3(1, 2., 0));
    btVector3 mainHinge1 = PointWorldToLocal(0, btVector3(1, 2., 0));
    btVector3 localAxis1 = AxisWorldToLocal(1, btVector3(0., 0., -1.));
    btVector3 mainAxis1 = AxisWorldToLocal(0, btVector3( 0., 0., -1.));
    
    
    CreateHinge(5, *body[0], *body[1], mainHinge1, localHinge1, mainAxis1, localAxis1);
    joints[5]->setLimit(btScalar(M_PI_4), btScalar(M_PI_4 +M_PI_2));
    
    //zpositive (back) to main
    btVector3 localHinge3 = PointWorldToLocal(3, btVector3(0., 2., 1));
    btVector3 mainHinge3 = PointWorldToLocal(0, btVector3(0., 2., 1));
    btVector3 localAxis3 = AxisWorldToLocal(3, btVector3(-1., 0., 0.));
    btVector3 mainAxis3 = AxisWorldToLocal(0, btVector3( -1., 0., 0.));
    
    CreateHinge(6, *body[0], *body[3], mainHinge3, localHinge3, mainAxis3, localAxis3);
    joints[6]->setLimit(btScalar(M_PI_4), btScalar(M_PI_2+M_PI_4));
    
    //znegative (front) to main body
    btVector3 localHinge4 = PointWorldToLocal(4, btVector3(0., 2., -1));
    btVector3 mainHinge4= PointWorldToLocal(0, btVector3(0., 2., -1));
    btVector3 localAxis4 = AxisWorldToLocal(4, btVector3(-1., 0., 0. ));
    btVector3 mainAxis4 = AxisWorldToLocal(0, btVector3( -1., 0., 0.));
    
    
    CreateHinge(7, *body[0], *body[4], mainHinge4, localHinge4, mainAxis4, localAxis4);
    joints[7]->setLimit(btScalar(M_PI_4), btScalar(M_PI_4+M_PI_2));
    
	clientResetScene();
}

void RagdollDemo::spawnRagdoll(const btVector3& startOffset)
{
	RagDoll* ragDoll = new RagDoll (m_dynamicsWorld, startOffset);
	m_ragdolls.push_back(ragDoll);
}	

void RagdollDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();

	float minFPS = 100000.f/60.f;
	if (ms > minFPS)
		ms = minFPS;

	if (m_dynamicsWorld)
	{
        int count = 0;
        if(!pause || (pause && oneStep))
        {
            /*
            ActuateJoint2(0, M_PI_4, M_PI_2, ms / 1000000.f);
            ActuateJoint2(1, -3.*M_PI/4, M_PI_2, ms / 1000000.f);
            ActuateJoint2(2, M_PI_4, M_PI_2, ms / 1000000.f);
            ActuateJoint2(3, -3.*M_PI/4, M_PI_2, ms / 1000000.f);
            ActuateJoint2(4, -M_PI_4, M_PI_2, ms / 1000000.f);
            ActuateJoint2(5, 3.*M_PI/4, M_PI_2, ms / 1000000.f);
            ActuateJoint2(6, -M_PI_4, M_PI_2, ms / 1000000.f);
            ActuateJoint2(7, 3.*M_PI/4, M_PI_2, ms / 1000000.f);
            */
            
           double random[8];
            for( int i = 0; i< 8; i++)
            {
                //srand(time(NULL));
                random[i] = rand()/double(RAND_MAX)*M_PI_4;
                double randNeg = double(rand()/double(RAND_MAX));
                if(randNeg < .5)
                {
                    random[i] = -1.*random[i];
                }
            
            }
            
            /*for(int k = 0; k<8; k++)
                printf("%f\n", random[k]);*/
            
            if(count %1000 == 0)
            {
            //Hinge main body to legs: The offset is negative if the axis vectors are positive => the joint limits are negative
            ActuateJoint(4, random[0], M_PI_2, ms / 1000000.f);
            ActuateJoint(5, random[1], M_PI_2, ms / 1000000.f);
            ActuateJoint(6, random[2], M_PI_2, ms / 1000000.f);
            ActuateJoint(7, random[3], M_PI_2, ms / 1000000.f);
            
            //Jeg joints: offset negative if the axis vectors are negative => joint limits are negative
            ActuateJoint(0, random[4], M_PI_2, ms / 1000000.f);
            ActuateJoint(1, random[5], M_PI_2, ms / 1000000.f);
            ActuateJoint(2, random[6], M_PI_2, ms / 1000000.f);
            ActuateJoint(3, random[7], M_PI_2, ms / 1000000.f);
            
            
            }
            
            
            for(int k = 0; k<10; k++)
            {
                touches[k] = 0;
                //printf("touches[k] = %d\n", touches[k]);
            }
            

            
            m_dynamicsWorld->stepSimulation(ms / 100000.f);
            
            for(int i = 0; i<9; i++)
            {
                printf("%d", touches[i]);
                
            }
            
            printf("\n");
            /*for(int k = 0; k<10; k++);
            {
                printf("%d", touches[k]);
                
            }
             */
            oneStep = !oneStep;
            count++;
        
            
            //Hinge main body to legs: The offset is negative if the axis vectors are positive => the joint limits are negative
            //ActuateJoint(4, -M_PI_4, M_PI_2, ms / 1000000.f);
           // ActuateJoint(5, M_PI_4, M_PI_2 , ms / 1000000.f);
            //ActuateJoint(6, -M_PI_4, M_PI_2, ms / 1000000.f);
            //ActuateJoint(7, M_PI_4, M_PI_2, ms / 1000000.f);
            
            //Jeg joints: offset negative if the axis vectors are negative => joint limits are negative
            //ActuateJoint(0, M_PI_4, M_PI_2, ms / 1000000.f);
            //ActuateJoint(1, -M_PI_4, M_PI_2, ms / 1000000.f);
            //ActuateJoint(2, M_PI_4, M_PI_2, ms / 1000000.f);
            //ActuateJoint(3, -M_PI_4, M_PI_2, ms / 1000000.f);
        
        
        }
            
        
        
        /*if(!pause)
        {
            m_dynamicsWorld->stepSimulation(ms / 1000000.f);
        }
        if (pause && oneStep)
            {
                m_dynamicsWorld->stepSimulation(ms / 1000000.f);
                oneStep = !oneStep;
            }
        
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();*/

	}

	renderme(); 

	glFlush();

	glutSwapBuffers();
}

void RagdollDemo::displayCallback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}

void RagdollDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'e':
		{
		btVector3 startOffset(0,2,0);
		spawnRagdoll(startOffset);
		break;
		}
    case 'p':
        {
            pause = !pause;
        }
    case 'o':
        {
            oneStep = !oneStep;
        }
	default:
		DemoApplication::keyboardCallback(key, x, y);
	}

	
}



void	RagdollDemo::exitPhysics()
{
    DestroyHinge(0);
    
    DeleteObject(0);
	int i;
	for (i=0;i<m_ragdolls.size();i++)
	{
		RagDoll* doll = m_ragdolls[i];
		delete doll;
	}

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





