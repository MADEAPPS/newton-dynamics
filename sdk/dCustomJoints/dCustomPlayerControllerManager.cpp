/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// NewtonCustomJoint.cpp: implementation of the NewtonCustomJoint class.
//
//////////////////////////////////////////////////////////////////////
#include "dCustomJointLibraryStdAfx.h"
#include "dCustomJoint.h"
#include "dCustomPlayerControllerManager.h"


#define D_DESCRETE_MOTION_STEPS		4
#define D_MAX_COLLIONSION_STEPS		8
#define D_MAX_COLLISION_PENTRATION	dFloat (2.0e-3f)

dCustomPlayerControllerManager::dCustomPlayerControllerManager(NewtonWorld* const world)
	:dCustomParallelListener(world, PLAYER_PLUGIN_NAME)
	,m_playerList()
{
}

dCustomPlayerControllerManager::~dCustomPlayerControllerManager()
{
	m_playerList.RemoveAll();
	dAssert(m_playerList.GetCount() == 0);
}

void dCustomPlayerControllerManager::PreUpdate(dFloat timestep, int threadID)
{
	D_TRACKTIME();
	NewtonWorld* const world = GetWorld();
	const int threadCount = NewtonGetThreadsCount(world);

	dList<dCustomPlayerController>::dListNode* node = m_playerList.GetFirst();
	for (int i = 0; i < threadID; i++) {
		node = node ? node->GetNext() : NULL;
	}
	if (node) {
		dCustomPlayerController* const controller = &node->GetInfo();
		controller->PreUpdate(timestep);
		do {
			for (int i = 0; i < threadCount; i++) {
				node = node ? node->GetNext() : NULL;
			}
		} while (node);
	}
}

void dCustomPlayerControllerManager::PostUpdate(dFloat timestep, int threadID)
{
	D_TRACKTIME();
	NewtonWorld* const world = GetWorld();
	const int threadCount = NewtonGetThreadsCount(world);

	dList<dCustomPlayerController>::dListNode* node = m_playerList.GetFirst();
	for (int i = 0; i < threadID; i++) {
		node = node ? node->GetNext() : NULL;
	}
	if (node) {
		dCustomPlayerController* const controller = &node->GetInfo();
		controller->PostUpdate(timestep);
		do {
			for (int i = 0; i < threadCount; i++) {
				node = node ? node->GetNext() : NULL;
			}
		} while (node);
	}
}

int dCustomPlayerControllerManager::ProcessContacts(const dCustomPlayerController* const controller, NewtonWorldConvexCastReturnInfo* const contacts, int count) const
{
	dAssert(0);
	return 0;
}


dCustomPlayerController* dCustomPlayerControllerManager::CreatePlayerController(const dMatrix& location, const dMatrix& localAxis, dFloat mass, dFloat radius, dFloat height)
{
//	dAssert(stairStep >= 0.0f);
//	dAssert(innerRadius >= 0.0f);
//	dAssert(outerRadius >= innerRadius);
//	dAssert(height >= stairStep);
//	dAssert(localAxis[0].m_w == dFloat(0.0f));
//	dAssert(localAxis[1].m_w == dFloat(0.0f));
//	dFloat mass, dFloat outerRadius, dFloat innerRadius, dFloat height, dFloat stairStep, const dMatrix& localAxis)

	NewtonWorld* const world = GetWorld();

//	SetRestrainingDistance(0.0f);
//	m_outerRadio = outerRadius;
//	m_innerRadio = innerRadius;
//	m_height = height;
//	m_stairStep = stairStep;
//	SetClimbSlope(45.0f * dDegreeToRad);
//	m_upVector = localAxis[0];
//	m_frontVector = localAxis[1];
//
//	m_groundPlane = dVector(0.0f);
//	m_groundVelocity = dVector(0.0f);
//
//	const int steps = 12;
//	dVector convexPoints[2][steps];
//
//	// create an inner thin cylinder
//	dFloat shapeHigh = height;
//	dAssert(shapeHigh > 0.0f);
//	dVector p0(0.0f, m_innerRadio, 0.0f, 0.0f);
//	dVector p1(shapeHigh, m_innerRadio, 0.0f, 0.0f);
//	for (int i = 0; i < steps; i++) {
//		dMatrix rotation(dPitchMatrix(i * 2.0f * dPi / steps));
//		convexPoints[0][i] = localAxis.RotateVector(rotation.RotateVector(p0));
//		convexPoints[1][i] = localAxis.RotateVector(rotation.RotateVector(p1));
//	}
//	NewtonCollision* const supportShape = NewtonCreateConvexHull(world, steps * 2, &convexPoints[0][0].m_x, sizeof(dVector), 0.0f, 0, NULL);
//
//	// create the outer thick cylinder
//	dMatrix outerShapeMatrix(localAxis);
//	dFloat capsuleHigh = m_height - stairStep;
//	dAssert(capsuleHigh > 0.0f);
//	m_sphereCastOrigin = capsuleHigh * 0.5f + stairStep;
//	outerShapeMatrix.m_posit = outerShapeMatrix[0].Scale(m_sphereCastOrigin);
//	outerShapeMatrix.m_posit.m_w = 1.0f;
//	NewtonCollision* const bodyCapsule = NewtonCreateCapsule(world, 0.25f, 0.25f, 0.5f, 0, &outerShapeMatrix[0][0]);
//	NewtonCollisionSetScale(bodyCapsule, capsuleHigh, m_outerRadio * 4.0f, m_outerRadio * 4.0f);

	dMatrix shapeMatrix(localAxis);
	shapeMatrix.m_posit = shapeMatrix.m_front.Scale (height * 0.5f);
	shapeMatrix.m_posit.m_w = 1.0f;
	height = dMax (height - 2.0f * radius, dFloat (0.1f));
	NewtonCollision* const bodyCapsule = NewtonCreateCapsule(world, radius, radius, height, 0, &shapeMatrix[0][0]);

	// compound collision player controller
//	NewtonCollision* const playerShape = NewtonCreateCompoundCollision(world, 0);
//	NewtonCompoundCollisionBeginAddRemove(playerShape);
//	NewtonCompoundCollisionAddSubCollision(playerShape, supportShape);
//	NewtonCompoundCollisionAddSubCollision(playerShape, bodyCapsule);
//	NewtonCompoundCollisionEndAddRemove(playerShape);

	// create the kinematic body
	NewtonBody* const body = NewtonCreateKinematicBody(world, bodyCapsule, &location[0][0]);

	// players must have weight, otherwise they are infinitely strong when they collide
	NewtonCollision* const shape = NewtonBodyGetCollision(body);
	NewtonBodySetMassProperties(body, mass, shape);

	// make the body collidable with other dynamics bodies, by default
	NewtonBodySetCollidable(body, 1);

/*
	dFloat castHigh = capsuleHigh * 0.4f;
	dFloat castRadio = (m_innerRadio * 0.5f > 0.05f) ? m_innerRadio * 0.5f : 0.05f;

	dVector q0(0.0f, castRadio, 0.0f, 0.0f);
	dVector q1(castHigh, castRadio, 0.0f, 0.0f);
	for (int i = 0; i < steps; i++) {
		dMatrix rotation(dPitchMatrix(i * 2.0f * dPi / steps));
		convexPoints[0][i] = localAxis.RotateVector(rotation.RotateVector(q0));
		convexPoints[1][i] = localAxis.RotateVector(rotation.RotateVector(q1));
	}
	m_castingShape = NewtonCreateConvexHull(world, steps * 2, &convexPoints[0][0].m_x, sizeof(dVector), 0.0f, 0, NULL);

	m_supportShape = NewtonCompoundCollisionGetCollisionFromNode(shape, NewtonCompoundCollisionGetNodeByIndex(shape, 0));
	m_upperBodyShape = NewtonCompoundCollisionGetCollisionFromNode(shape, NewtonCompoundCollisionGetNodeByIndex(shape, 1));
	
	NewtonDestroyCollision(supportShape);
	NewtonDestroyCollision(playerShape);

	m_isJumping = false;
*/
	NewtonDestroyCollision(bodyCapsule);

	dCustomPlayerController& controller = m_playerList.Append()->GetInfo();

	controller.m_mass = mass;
	controller.m_invMass = 1.0f / mass;
	controller.m_manager = this;
	controller.m_kinematicBody = body;
	return &controller;
}

void dCustomPlayerController::PreUpdate(dFloat timestep)
{
	m_impulse = dVector(0.0f);
	m_manager->ApplyPlayerMove(this, timestep);

	dVector veloc(m_veloc + m_impulse.Scale(m_invMass));
	NewtonBodySetVelocity(m_kinematicBody, &veloc[0]);
}


void dCustomPlayerController::PostUpdate(dFloat timestep)
{
	dFloat timeLeft = timestep;
	const dFloat timeEpsilon = timestep * (1.0f / 16.0f);

	for (int i = 0; (i < D_DESCRETE_MOTION_STEPS) && (timeLeft > timeEpsilon); i++) {
		if (timeLeft > timeEpsilon) {
			ResolveCollision();
		}

		dFloat predicetdTime = PredictTimestep(timestep);
		NewtonBodyIntegrateVelocity(m_kinematicBody, predicetdTime);
		timeLeft -= predicetdTime;
	}

//	dAssert(timeLeft < timeEpsilon);
	NewtonBodyGetVelocity(m_kinematicBody, &m_veloc[0]);
}


unsigned dCustomPlayerController::PrefilterCallback(const NewtonBody* const body, const NewtonCollision* const collision, void* const userData)
{
	dCustomPlayerController* const controller = (dCustomPlayerController*)userData;
	if (controller->GetBody() == body) {
		return false;
	}
	return 1;
}

void dCustomPlayerController::ResolveCollision()
{
	dMatrix matrix;
	NewtonWorldConvexCastReturnInfo info[16];

	NewtonWorld* const world = m_manager->GetWorld();
		
	NewtonBodyGetMatrix(m_kinematicBody, &matrix[0][0]);
	NewtonCollision* const shape = NewtonBodyGetCollision(m_kinematicBody);

	int contactCount = NewtonWorldCollide(world, &matrix[0][0], shape, this, PrefilterCallback, info, 4, 0);
	if (contactCount) {
		m_veloc = dVector(0.0f);
		NewtonBodySetVelocity(m_kinematicBody, &m_veloc[0]);
		dTrace(("implment collsion rsolution !!!\n"));
	}
}

dFloat dCustomPlayerController::PredictTimestep(dFloat timestep)
{
	dMatrix matrix;
	dMatrix predicMatrix;
	NewtonWorld* const world = m_manager->GetWorld();
	
	NewtonWorldConvexCastReturnInfo info[16];
	NewtonBodyGetMatrix(m_kinematicBody, &matrix[0][0]);
	NewtonCollision* const shape = NewtonBodyGetCollision(m_kinematicBody);
	
	NewtonBodyIntegrateVelocity(m_kinematicBody, timestep);
	NewtonBodyGetMatrix(m_kinematicBody, &predicMatrix[0][0]);
	int contactCount = NewtonWorldCollide(world, &predicMatrix[0][0], shape, this, PrefilterCallback, info, 4, 0);
	NewtonBodySetMatrix(m_kinematicBody, &matrix[0][0]);

	if (contactCount) {
		dFloat t0 = 0.0f;
		dFloat t1 = timestep;
		dFloat dt = (t1 + t0) * 0.5f;
		timestep = dt;
		for (int i = 0; i < D_MAX_COLLIONSION_STEPS; i++) {
			NewtonBodyIntegrateVelocity(m_kinematicBody, timestep);
			NewtonBodyGetMatrix(m_kinematicBody, &predicMatrix[0][0]);
			contactCount = NewtonWorldCollide(world, &predicMatrix[0][0], shape, this, PrefilterCallback, info, 4, 0);
			NewtonBodySetMatrix(m_kinematicBody, &matrix[0][0]);

			dt *= 0.5f;
			if (contactCount) {
				dFloat penetration = 0.0f;
				for (int j = 0; j < contactCount; j++) {
					penetration = dMax(penetration, info[j].m_penetration);
				}
				if (penetration < D_MAX_COLLISION_PENTRATION) {
					break;
				}
				timestep -= dt;
			} else {
				timestep += dt;
			}
		}
	}

	return timestep;
}
