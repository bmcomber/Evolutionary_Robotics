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

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>


#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif




static RagdollDemo* ragdollDemo;
double jointOffset[8];

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
    
    return false;
}

void RagdollDemo::initPhysics()
{
    ragdollDemo = this;
    timeStep = 0;
    using namespace std;
    
    //open the synaptic weights file
    ifstream inFile;
    inFile.open("/Users/bmcomber/Assignment10/weights.dat");
	string line;
    for(int j=0; j<4; j++){
		for(int i=0; i<8; i++){
            
			getline (inFile, line);
			stringstream convert(line);
			convert >> weights[j][i];
		}
	}
	inFile.close();
    
    
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
    
    //create my robot
    
    CreateBox(0, 0., 2., 0., 1., 0.2, 1.); // Create the box
    
    //leg components
    //x-left
    CreateCylinder(1, 2. , 2., -.6, .2, 1., 0., 0., M_PI_2); //xleft horizontal
    CreateCylinder(5, 3.0, 1., -.6, .2, 1., 0., 0., 0.);       //xleft vertical
    
    CreateCylinder(3, 2., 2., .6, .2, 1., 0., 0., M_PI_2);    //zpositive (into screen)
    CreateCylinder(7, 3., 1., .6, .2, 1., 0., 0., 0.);     //zpositive vertical
    
    //x-right
    CreateCylinder(2, -2., 2., -.6, .2, 1., 0., 0., M_PI_2); //xright (negative) horizontal
    CreateCylinder(6, -3., 1., -.6, .2, 1., 0., 0., 0.);      //xright vertical
    
    CreateCylinder(4, -2., 2., .6, .2, 1., 0., 0., M_PI_2);   //znegative (out of screen)
    CreateCylinder(8, -3., 1., .6, .2, 1., 0., 0., 0.);    //znegative vertical
    
    
    
    
    
    
    
    //The axisworldtolocal defines a vector perpendicular to the plane that you want the bodies to rotate in
    
    
    //hinge the legs together
    //xnegative -- right
    //two bodies should rotate in y-plane through x--give axisworldtolocal it a z vector
    btVector3 local2 = PointWorldToLocal(2, btVector3(-3., 2., 0));
    btVector3 local6 = PointWorldToLocal(6, btVector3(-3., 2., 0));
    btVector3 axis2 = AxisWorldToLocal(2, btVector3(0., 0., 1.));
    btVector3 axis6 = AxisWorldToLocal(6, btVector3( 0., 0., 1.));
    
    
    CreateHinge(0, *body[2], *body[6], local2, local6, axis2, axis6);
    //joints[0]->setLimit(btScalar(-M_PI_4-M_PI_2), btScalar(-M_PI_4));
    
    //znegative -- front
    
    btVector3 local4 = PointWorldToLocal(4, btVector3(-3., 2., 0.));
    btVector3 local8 = PointWorldToLocal(8, btVector3(-3., 2., 0.));
    btVector3 axis4 = AxisWorldToLocal(4, btVector3(0., 0., 1.));
    btVector3 axis8 = AxisWorldToLocal(8, btVector3(0., 0., 1.));
    
    CreateHinge(3, *body[4], *body[8], local4, local8, axis4, axis8);
    //joints[3]->setLimit(btScalar(M_PI_4), btScalar(M_PI_2+M_PI_4));
    
    
    //positive -- left
    //give axisworldtolocal a z vector
    btVector3 local1 = PointWorldToLocal(1, btVector3(3., 2., 0));
    btVector3 local5 = PointWorldToLocal(5, btVector3(3., 2., 0));
    btVector3 axis1 = AxisWorldToLocal(1, btVector3( 0., 0., -1.));
    btVector3 axis5 = AxisWorldToLocal(5, btVector3(0., 0., -1.));
    
    CreateHinge(1, *body[1], *body[5], local1, local5, axis1, axis5);
    //joints[1]->setLimit(btScalar(M_PI_4), btScalar(M_PI_2+M_PI_4));
    
    //zpositive -- back
    //rotates in y-plane through z--give it an x vector
    btVector3 local3 = PointWorldToLocal(3, btVector3(3., 2., 0.));
    btVector3 local7 = PointWorldToLocal(7, btVector3(3., 2., 0.));
    btVector3 axis3 = AxisWorldToLocal(3, btVector3(0., 0., -1.));
    btVector3 axis7 = AxisWorldToLocal(7, btVector3(0., 0., -1.));
    
    CreateHinge(2, *body[3], *body[7], local3, local7, axis3, axis7);
    //joints[2]->setLimit(btScalar(-M_PI_4-M_PI_2), btScalar(-M_PI_4));
    

    
    //hinge the legs to the body
    //xright to main
    btVector3 localHinge2 = PointWorldToLocal(2, btVector3(-1, 2., 0));
    btVector3 mainHinge2 = PointWorldToLocal(0, btVector3(-1, 2., 0));
    btVector3 localAxis2 = AxisWorldToLocal(2, btVector3(0., 0., 1.));
    btVector3 mainAxis2 = AxisWorldToLocal(0, btVector3( 0., 0., 1.));
    
    CreateHinge(4, *body[0], *body[2], mainHinge2, localHinge2, mainAxis2, localAxis2);
    //joints[4]->setLimit(btScalar(M_PI_4), btScalar(M_PI_2+M_PI_4));
    
    
    //znegative (front) to main body
    btVector3 localHinge4 = PointWorldToLocal(4, btVector3(-1., 2., 0.));
    btVector3 mainHinge4= PointWorldToLocal(0, btVector3(-1., 2., 0.));
    btVector3 localAxis4 = AxisWorldToLocal(4, btVector3(0., 0., 1. ));
    btVector3 mainAxis4 = AxisWorldToLocal(0, btVector3( 0., 0., 1.));
    
    
    CreateHinge(7, *body[0], *body[4], mainHinge4, localHinge4, mainAxis4, localAxis4);
    //joints[7]->setLimit(btScalar(M_PI_4), btScalar(M_PI_4+M_PI_2));

    
    
    //xleft to main
    btVector3 localHinge1 = PointWorldToLocal(1, btVector3(1, 2., 0));
    btVector3 mainHinge1 = PointWorldToLocal(0, btVector3(1, 2., 0));
    btVector3 localAxis1 = AxisWorldToLocal(1, btVector3(0., 0., -1.));
    btVector3 mainAxis1 = AxisWorldToLocal(0, btVector3( 0., 0., -1.));
    
    
    CreateHinge(5, *body[0], *body[1], mainHinge1, localHinge1, mainAxis1, localAxis1);
    //joints[5]->setLimit(btScalar(M_PI_4), btScalar(M_PI_4 +M_PI_2));
    
    //zpositive (back) to main
    btVector3 localHinge3 = PointWorldToLocal(3, btVector3(1., 2., 0.));
    btVector3 mainHinge3 = PointWorldToLocal(0, btVector3(1., 2., 0.));
    btVector3 localAxis3 = AxisWorldToLocal(3, btVector3(0., 0., -1.));
    btVector3 mainAxis3 = AxisWorldToLocal(0, btVector3( 0., 0., -1.));
    
    CreateHinge(6, *body[0], *body[3], mainHinge3, localHinge3, mainAxis3, localAxis3);
    //joints[6]->setLimit(btScalar(M_PI_4), btScalar(M_PI_2+M_PI_4));
    
    
	clientResetScene();
}


void RagdollDemo::clientMoveAndDisplay()
{
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();
    
	float minFPS = 100000.f/60.f;
	if (ms > minFPS)
        
		ms = minFPS;
    
    
    if(m_dynamicsWorld)
    {
        if(!pause)
        {
            //set touches to 0
            for(int k = 0; k<10; k++)
            {
                touches[k] = 0;
                //printf("touches[k] = %d\n", touches[k]);
            }
            
            m_dynamicsWorld->stepSimulation(ms / 100000.f);
            
        }
        else if(oneStep)
        {
            m_dynamicsWorld->stepSimulation(ms / 100000.f);
            oneStep = false;
        }
        
        if(timeStep %2 == 0)
        {
            
            for(int i=0; i<8; i++)
            {
                
                double motorCommand = 0.0;
                
                for(int j=0; j<4; j++)
                {
                    
                    //Did j+5 because my 4 feet touch sensors start at the 5th index. May change depending on your configuration
                    motorCommand = motorCommand + touches[(j+5)]*weights[j][i];
                    //std::cout << touches[(j+5)] << std::endl;
                }
                
                //Keep motorcommand between -1 and 1
                motorCommand = tanh(motorCommand);
                
                //Expand it to be between -45 and 45
                //motorCommand = motorCommand*M_PI_4;
                motorCommand = -1;
                //motorCommand = M_PI_4;
                //printf("%f", motorCommand);
                
                ActuateJoint(i, motorCommand, jointOffset[i], ms / 100000.f);
            }
            //oneStep = !oneStep;
            
        }
        
    }
    timeStep++;
    if(timeStep == 1000)
    {
        using namespace std;
        btVector3 pos = body[0]->getCenterOfMassPosition();
        //printf("%f %f %f\n", pos.x(), pos.y(), pos.z());
        ofstream myfile;
        myfile.open ("/Users/bmcomber/Assignment10/fits.dat");
        myfile << pos.z();
        myfile.close();
        exit(0);
    }
    
	//optional but useful: debug drawing
    m_dynamicsWorld->debugDrawWorld();
    
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




void RagdollDemo:: CreateBox(int index,
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



void RagdollDemo:: CreateCylinder(int index,
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



void RagdollDemo:: DeleteObject( int index ){
    delete(geom[index]);
    delete(body[index]);
}




btVector3 RagdollDemo:: PointWorldToLocal(int index, btVector3 p) {
    btTransform local1 = body[index]->getCenterOfMassTransform().inverse();
    return local1 * p;
}



btVector3 RagdollDemo:: AxisWorldToLocal(int index, btVector3 a) {
    btTransform local1 = body[index]->getCenterOfMassTransform().inverse();
    btVector3 zero(0,0,0);
    local1.setOrigin(zero);
    return local1 * a;
}




void RagdollDemo:: CreateHinge(int index, btRigidBody& rbA, btRigidBody& rbB, btVector3& pivotInA, btVector3& pivotInB, btVector3& axisInA, btVector3& axisInB){
    
    btHingeConstraint* hinge;
    
    hinge = new btHingeConstraint(rbA, rbB, pivotInA, pivotInB, axisInA, axisInB);
    
    jointOffset[index]= hinge->getHingeAngle();
    
    //printf("%f\n", jointOffset[index]);
    
    
    joints[index] = hinge;
    joints[index]->setLimit(jointOffset[index]-M_PI_4, jointOffset[index]+M_PI_4);
    joints[index]->enableMotor(true);
    joints[index]->setMaxMotorImpulse(btScalar(1.));
    
    //M_PI_4, M_PI_4, M_PI_2
    
    hinge->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
    
    m_dynamicsWorld->addConstraint(joints[index], true);
}



void RagdollDemo:: ActuateJoint(int jointIndex,
                                double desiredAngle, double jointOffset, double timeStep) {
    
    
    //joints[jointIndex]->enableAngularMotor(true, btScalar(.1), btScalar(1.));
    joints[jointIndex]->setMotorTarget(btScalar(desiredAngle+jointOffset), btScalar(timeStep));
    
}



void RagdollDemo:: DestroyHinge(int index){
    delete(joints[index]);
}




void RagdollDemo:: renderme()
{
    extern GLDebugDrawer gDebugDrawer;
    // Call the parent method.
    GlutDemoApplication::renderme();
    
    for(int i = 5; i<9; i++)
    {
        if(touches[i] == 1)
        {
            gDebugDrawer.drawSphere(touchPoints[i], 0.2, btVector3(1., 0., 0.));
        }
    }
    
}

void RagdollDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
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
    for(int j = 0; j<8; j++){
        DestroyHinge(j);
    }
    
    
    for(int k = 0; k<10; k++){
        DeleteObject(k);
    }
    
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
