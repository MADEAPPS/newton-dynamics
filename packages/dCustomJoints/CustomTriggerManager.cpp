/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// NewtonPlayerControllerManager.h: interface for the NewtonPlayerControllerManager class.
//
//////////////////////////////////////////////////////////////////////
#include <CustomJointLibraryStdAfx.h>
#include <CustomJoint.h>
#include <CustomTriggerManager.h>


/*
static int aabbCollisionCallback (const NewtonBody * const otherBody, void * const userData)
{   
	NewtonBody* const myBody = (NewtonBody*)userData;
	dAssert (myBody);

	// not interested in self collision
	if ( myBody == otherBody ) {
		return 1;
	}

	NewtonWorld* const world = NewtonBodyGetWorld(myBody);
	NewtonCollision* const collisionA = NewtonBodyGetCollision(myBody);
	NewtonCollision* const collisionB = NewtonBodyGetCollision(otherBody);
	
	dMatrix poseA;
	NewtonBodyGetMatrix(myBody, &poseA[0][0]);

	dMatrix poseB;
	NewtonBodyGetMatrix(otherBody,&poseB[0][0]);

	if( NewtonCollisionIntersectionTest(world, collisionA, &poseA[0][0], collisionB, &poseB[0][0],0) )
	{
		// ignore contact with no penetration
		const int maxSize = 2;
		dFloat contacts[maxSize * 3];
		dFloat normals[maxSize * 3];
		dFloat penetrations[maxSize];
		dLong attrbA[maxSize * 3];
		dLong attrbB[maxSize * 3];
		int contactCount = NewtonCollisionCollide(world, maxSize,
			collisionA, &poseA[0][0], 
			collisionB, &poseB[0][0],
			&contacts[0],
			&normals[0],
			&penetrations[0],
			&attrbA[0],
			&attrbB[0],
			0);
		if(contactCount) {
//			dAssert (0);
			contactCount*=1;
			//entity->bodyDesc.collisionInfo.collisionList.push_back( (PhysicsBody*)(otherEntity) );

		}
	}
	return 1;
}


static bool testForCollision (NewtonBody * const pBody)
{
	// this body has NewtonCollisionSetCollisonMode set to 0
	dVector min;
	dVector max;
	NewtonBodyGetAABB (pBody, &min.m_x, &max.m_x);
	NewtonWorld* const world = NewtonBodyGetWorld(pBody);
	NewtonWorldForEachBodyInAABBDo(world, &min.m_x, &max.m_x, aabbCollisionCallback, pBody);
	return true;
}
*/

CustomTriggerManager::CustomTriggerManager(NewtonWorld* const world)
	:CustomControllerManager<CustomTriggerController>(world, TRIGGER_PLUGIN_NAME)
	,m_lock(0)
{
}

CustomTriggerManager::~CustomTriggerManager()
{
}

	
CustomTriggerController* CustomTriggerManager::CreateTrigger (const dMatrix& matrix, NewtonCollision* const convexShape, void* const userData)
{
	CustomTriggerController* const trigger = (CustomTriggerController*) CreateController();
	trigger->Init (convexShape, matrix, userData);
	return trigger;
}

void CustomTriggerManager::PreUpdate(dFloat timestep)
{
	for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
		NewtonDispachThreadJob (m_world, UpdateTrigger, &node->GetInfo());
	}
	NewtonSyncThreadJobs(m_world);

	// bypass the entire Post Update call by not calling the base class
	//CustomControllerManager<CustomTriggerController>::PreUpdate(timestep);
}


void CustomTriggerManager::UpdateTrigger (CustomTriggerController* const controller)
{
	NewtonBody* const triggerBody = controller->GetBody();
	dTree<NewtonBody*,NewtonBody*>& manifest = controller->m_manifest;

//bool xxx = testForCollision (triggerBody);

	for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint (triggerBody); joint; joint = NewtonBodyGetNextContactJoint (triggerBody, joint)) {

		int isActive = NewtonJointIsActive (joint);
		NewtonBody* const body0 = NewtonJointGetBody0(joint);
		NewtonBody* const body1 = NewtonJointGetBody1(joint);
		NewtonBody* const passangerBody = (body0 != triggerBody) ? body0 : body1; 
		
		dTree<NewtonBody*,NewtonBody*>::dTreeNode* const passengerNode = manifest.Find (passangerBody);
		if (isActive) {
			if (passengerNode) {
				EventCallback (controller, m_inTrigger, passangerBody);

			} else {
				CustomScopeLock lock (&m_lock);
				manifest.Insert (passangerBody, passangerBody);
				EventCallback (controller, m_enterTrigger, passangerBody);
			} 
		} else {
			if (passengerNode) {
				EventCallback (controller, m_exitTrigger, passangerBody);

				CustomScopeLock lock (&m_lock);
				manifest.Remove (passengerNode);
			}
		}
	}

}


void CustomTriggerManager::UpdateTrigger (NewtonWorld* const world, void* const context, int threadIndex)
{
	CustomTriggerController* const controller = (CustomTriggerController*)context;
	CustomTriggerManager* const me = (CustomTriggerManager*)controller->GetManager();
	me->UpdateTrigger(controller);
}



CustomTriggerController::CustomTriggerController()
	:CustomControllerBase()
	,m_manifest()
{
}

CustomTriggerController::~CustomTriggerController()
{
	NewtonDestroyBody(m_body);
}


void CustomTriggerController::Init (NewtonCollision* const convexShape, const dMatrix& matrix, void* const userData)
{
	m_userData = userData;

	NewtonWorld* const world = ((CustomTriggerManager*)GetManager())->GetWorld();

	// create a trigger body and place in the scene
	m_body = NewtonCreateKinematicBody(world, convexShape, &matrix[0][0]);
	
	// set this shape do not collide with other bodies
	NewtonCollision* const collision = NewtonBodyGetCollision (m_body);
	NewtonCollisionSetCollisionMode(collision, 0);
}


void CustomTriggerController::PostUpdate(dFloat timestep, int threadIndex)
{
}

void CustomTriggerController::PreUpdate(dFloat timestep, int threadIndex)
{
}
