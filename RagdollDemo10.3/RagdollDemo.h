/*
Bullet Continuous Collision Detection and Physics Library
RagdollDemo
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

#ifndef RAGDOLLDEMO_H
#define RAGDOLLDEMO_H
#define CONSTRAINT_DEBUG_SIZE 0.2f

#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletDynamicsCommon.h"
#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"

#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

class RagdollDemo : public GlutDemoApplication
{

	btAlignedObjectArray<class RagDoll*> m_ragdolls;

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;
    
    btRigidBody* body[13]; // one main body, 4x2 leg segments
    
    btCollisionShape* geom[13];
    
    
    bool pause;
    
    btHingeConstraint* joints[12];
    
    
    bool oneStep;
    
    int IDs[14];
    int timeStep;
    
    public: btVector3 touchPoints[14];
    
    public: int touches[14];

    public: double weights[6][12];
    public: double weights2[32];
    

public:
	
    void CreateBox(int index,
                   double x, double y, double z,
                   double length, double width, double height);
    

    
    void CreateCylinder(int index,
                        double x, double y, double z,
                        double radius, double height, double eulerX, double eulerY, double eulerZ);

    void DeleteObject( int index );
    
    btVector3 PointWorldToLocal(int index, btVector3 p);
    
    btVector3 AxisWorldToLocal(int index, btVector3 a);
    
    void CreateHinge(int index, btRigidBody& rbA, btRigidBody& rbB, btVector3& pivotInA, btVector3& pivotInB, btVector3& axisInA, btVector3& axisInB);
    
    void ActuateJoint(int jointIndex,
                      double desiredAngle, double jointOffset, double timeStep);
    
    void DestroyHinge(int index);
    
    void initPhysics();

	void exitPhysics();
    
    virtual void renderme();
    
	virtual ~RagdollDemo()
	{
		exitPhysics();
	}

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void keyboardCallback(unsigned char key, int x, int y);

	static DemoApplication* Create()
	{
		RagdollDemo* demo = new RagdollDemo();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}
	
};


#endif
