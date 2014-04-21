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
#define M_PI_2     1.57079632679489661923
#define M_PI_4     0.785398163397448309616

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
    
    btRigidBody* body[9]; // one main body, 4x2 leg segments
    
    btCollisionShape* geom[9];
    
    
    bool pause;
    
    btHingeConstraint* joints[8];
    
    
    bool oneStep;
    
    int IDs[10];
    
public: btVector3 touchPoints[10];
    
public: int touches[10];
    
    
    
    
    /*for (int i = 0; i<9; i++)
    {
        IDs[i] = i;
    }
     */
    
    
    virtual void renderme()
    {
        extern GLDebugDrawer gDebugDrawer;
        // Call the parent method.
        GlutDemoApplication::renderme();
        // Make a circle with a 0.9 radius at (0,0,0)
        // with RGB color (1,0,0).
        //gDebugDrawer.drawSphere(btVector3(0.,0.,0.),
                                //0.9, btVector3(1., 0., 0.));
        
        for(int i = 5; i<9; i++)
        {
            if(touches[i] == 1)
            {
               gDebugDrawer.drawSphere(touchPoints[i], 0.2, btVector3(1., 0., 0.));
            }
        }
    }
    void CreateBox(int index,
                   double x, double y, double z,
                   double length, double width, double height)
    {
        
        
        for(int i = 0; i<10; i++)
        {
            IDs[i] = i;
            //printf("id = %d", i);
        }
        
        
        btCollisionShape* boxGeom;
        btRigidBody* boxBody;
        
        // Setup the geometry
        boxGeom = new btBoxShape(btVector3(length, width, height));
        
        //btCollisionShape* geom[9];
        geom[index]= boxGeom;
        
        // Setup all the rigid bodies
        btTransform offset;
        offset.setIdentity();
        
        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(btVector3(x, y, z));
        boxBody = localCreateRigidBody(btScalar(1.), offset*transform, boxGeom);
        //btRigidBody* body[9];
        body[index]= boxBody;
        body[index]->setUserPointer(&IDs[index]);
        
        
    }

    void CreateCylinder(int index,
                   double x, double y, double z,
                   double radius, double height, double eulerX, double eulerY, double eulerZ)
    {
        
        btCollisionShape* cylGeom;
        btRigidBody* cylBody;
        
        // Setup the geometry
        cylGeom = new btCylinderShape(btVector3(radius, height, radius));
        
        //btCollisionShape* geom[9];
        geom[index]= cylGeom;
        
        // Setup all the rigid bodies
        btTransform offset;
        offset.setIdentity();
        
        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(btVector3(x, y, z));
        transform.getBasis().setEulerZYX(eulerX, eulerY, eulerZ);
        cylBody = localCreateRigidBody(btScalar(1.), offset*transform, geom[index]);
        //btRigidBody* body[9];
        body[index]= cylBody;
        body[index]->setUserPointer(&IDs[index]);
        
    }
    
    void DeleteObject( int index ){
        delete(geom[index]);
        delete(body[index]);
    }
    
    
    btVector3 PointWorldToLocal(int index, btVector3 p) {
        btTransform local1 = body[index]->getCenterOfMassTransform().inverse();
        return local1 * p;
    }
    
    btVector3 AxisWorldToLocal(int index, btVector3 a) {
        btTransform local1 = body[index]->getCenterOfMassTransform().inverse();
        btVector3 zero(0,0,0);
        local1.setOrigin(zero);
        return local1 * a; 
    }
    
    
    
    
    void CreateHinge(int index, btRigidBody& rbA, btRigidBody& rbB, btVector3& pivotInA, btVector3& pivotInB, btVector3& axisInA, btVector3& axisInB){
        
        btHingeConstraint* hinge;
        
        hinge = new btHingeConstraint(rbA, rbB, pivotInA, pivotInB, axisInA, axisInB);
        
        //hinge->setLimit(btScalar(-M_PI_4), btScalar(M_PI_2));
        
        joints[index] = hinge;
        
        //M_PI_4, M_PI_4, M_PI_2
        
        hinge->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
        
        m_dynamicsWorld->addConstraint(joints[index], true);
    }
    
    void ActuateJoint(int jointIndex,
                      double desiredAngle, double jointOffset, double timeStep) {

        joints[jointIndex]->enableMotor(true);
        joints[jointIndex]->setMaxMotorImpulse(btScalar(1.25));
        joints[jointIndex]->setMotorTarget(btScalar(desiredAngle+jointOffset), btScalar(timeStep));
        
    }
    
    void ActuateJoint2(int jointIndex,
                       double desiredAngle, double jointOffset, double timeStep){
        
        joints[jointIndex]->enableMotor(true);
        joints[jointIndex]->setMaxMotorImpulse(btScalar(10.));
        btScalar hingeAngle = joints[jointIndex]->getHingeAngle();
        
        
        btScalar diff = btScalar(desiredAngle) + btScalar(jointOffset) - btScalar(hingeAngle);
        joints[jointIndex]->enableAngularMotor(true, btScalar(diff), btScalar(10.)); //maxImpulse is last parameter
        
        
    }
    
    void DestroyHinge(int index){
        delete(joints[index]);
    }
    

public:
	
    void CreateBox();
    
    void initPhysics();

	void exitPhysics();

	virtual ~RagdollDemo()
	{
		exitPhysics();
	}

	void spawnRagdoll(const btVector3& startOffset);

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
