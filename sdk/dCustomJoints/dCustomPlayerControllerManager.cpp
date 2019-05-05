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

#define D_MAX_ROWS					8
#define D_MAX_COLLISION_PENETRATION	dFloat (5.0e-3f)

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
	NewtonWorld* const world = GetWorld();

	dMatrix shapeMatrix(localAxis);
	shapeMatrix.m_posit = shapeMatrix.m_front.Scale (height * 0.5f);
	shapeMatrix.m_posit.m_w = 1.0f;
	height = dMax (height - 2.0f * radius, dFloat (0.1f));
	NewtonCollision* const bodyCapsule = NewtonCreateCapsule(world, radius, radius, height, 0, &shapeMatrix[0][0]);

	// create the kinematic body
	NewtonBody* const body = NewtonCreateKinematicBody(world, bodyCapsule, &location[0][0]);

	// players must have weight, otherwise they are infinitely strong when they collide
	NewtonCollision* const shape = NewtonBodyGetCollision(body);
	NewtonBodySetMassProperties(body, mass, shape);

	// make the body collidable with other dynamics bodies, by default
	NewtonBodySetCollidable(body, 1);
	NewtonDestroyCollision(bodyCapsule);

	dCustomPlayerController& controller = m_playerList.Append()->GetInfo();

	controller.m_localFrame = localAxis;
	controller.m_mass = mass;
	controller.m_invMass = 1.0f / mass;
	controller.m_manager = this;
	controller.m_kinematicBody = body;
	return &controller;
}

dVector dCustomPlayerController::GetVelocity() const
{ 
	dVector veloc(0.0);
	NewtonBodyGetVelocity(m_kinematicBody, &veloc[0]);
	return veloc; 
}

void dCustomPlayerController::SetVelocity(const dVector& veloc) 
{ 
	NewtonBodySetVelocity(m_kinematicBody, &veloc[0]);
}

void dCustomPlayerController::PreUpdate(dFloat timestep)
{
	m_impulse = dVector(0.0f);
	m_manager->ApplyPlayerMove(this, timestep);

	dVector veloc(GetVelocity() + m_impulse.Scale(m_invMass));
	NewtonBodySetVelocity(m_kinematicBody, &veloc[0]);
}

unsigned dCustomPlayerController::PrefilterCallback(const NewtonBody* const body, const NewtonCollision* const collision, void* const userData)
{
	dCustomPlayerController* const controller = (dCustomPlayerController*)userData;
	if (controller->GetBody() == body) {
		return false;
	}
	return 1;
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
				if (penetration < D_MAX_COLLISION_PENETRATION) {
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

dVector dCustomPlayerController::CalculateImpulse(
	int rows, 
	const dFloat* const rhs,
	const dFloat* const low,
	const dFloat* const high,
	const int* const normalIndex,
	const dComplementaritySolver::dJacobian* const jt) const
{
	dMatrix invInertia;
	dFloat impulseMag[D_MAX_ROWS];
	dFloat massMatrix[D_MAX_ROWS][D_MAX_ROWS];

	NewtonBodyGetInvInertiaMatrix(m_kinematicBody, &invInertia[0][0]);
	for (int i = 0; i < rows; i++) {
		dComplementaritySolver::dJacobian jInvMass(jt[i]);

		jInvMass.m_linear = jInvMass.m_linear.Scale (m_invMass);
		jInvMass.m_angular = invInertia.RotateVector (jInvMass.m_angular);

		dVector tmp(jInvMass.m_linear * jt[i].m_linear + jInvMass.m_angular * jt[i].m_angular);

		dFloat a00 = (tmp.m_x + tmp.m_y + tmp.m_z) * 1.0001f;
		massMatrix[i][i] = a00;

		for (int j = i + 1; j < rows; j++) {
			dVector tmp1(jInvMass.m_linear * jt[j].m_linear + jInvMass.m_angular * jt[j].m_angular);
			dFloat a01 = tmp1.m_x + tmp1.m_y + tmp1.m_z;
			massMatrix[i][j] = a01;
			massMatrix[j][i] = a01;
		}
	}

	dGaussSeidelLcpSor(rows, D_MAX_ROWS, &massMatrix[0][0], impulseMag, rhs, normalIndex, low, high, 1.0e-2f, 32, 1.1f);

	dVector netImpulse(0.0f);
	for (int i = 0; i < rows; i++) {
		netImpulse += jt[i].m_linear.Scale(impulseMag[i]);
	}
	return netImpulse;
}

int dCustomPlayerController::ResolveInterpenetrations(int contactCount, NewtonWorldConvexCastReturnInfo* const contactArray)
{
	dVector zero (0.0f);
	dVector veloc (0.0f);
	dVector savedVeloc (0.0f);

	NewtonBodyGetVelocity(m_kinematicBody, &savedVeloc[0]);
	NewtonBodySetVelocity(m_kinematicBody, &veloc[0]);

	dFloat timestep = 0.1f;
	dFloat invTimestep = 1.0f / timestep;

	dComplementaritySolver::dJacobian jt[D_MAX_ROWS];
	dFloat rhs[D_MAX_ROWS];
	dFloat low[D_MAX_ROWS];
	dFloat high[D_MAX_ROWS];
	int normalIndex[D_MAX_ROWS];

	NewtonWorld* const world = m_manager->GetWorld();
	for (int i = 0; i < 3; i++) {
		jt[i].m_linear = zero;
		jt[i].m_angular = zero;
		jt[i].m_angular[i] = dFloat(1.0f);
		rhs[i] = 0.0f;
		low[i] = -1.0e12f;
		high[i] = 1.0e12f;
		normalIndex[i] = 0;
	}

	dFloat penetration = D_MAX_COLLISION_PENETRATION * 10.0f;
	for (int i = 0; (i < 8) && (penetration > D_MAX_COLLISION_PENETRATION) ; i ++) {
		dMatrix matrix;
		dVector com(0.0f);
		NewtonBodyGetMatrix(m_kinematicBody, &matrix[0][0]);
		NewtonBodyGetCentreOfMass(m_kinematicBody, &com[0]);
		com = matrix.TransformVector(com);
		com.m_w = 0.0f;

		int rowCount = 3;
		for (int i = 0; i < contactCount; i++) {
			NewtonWorldConvexCastReturnInfo& contact = contactArray[i];

			dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], 0.0f);
			dVector normal(contact.m_normal[0], contact.m_normal[1], contact.m_normal[2], 0.0f);

			jt[rowCount].m_linear = normal;
			jt[rowCount].m_angular = (point - com).CrossProduct(normal);

			low[rowCount] = 0.0f;
			high[rowCount] = 1.0e12f;
			normalIndex[rowCount] = 0;
			dFloat penetration = dClamp(contact.m_penetration - D_MAX_COLLISION_PENETRATION * 0.5f, dFloat(0.0f), dFloat(0.5f));
			rhs[rowCount] = penetration * invTimestep;
			rowCount++;
		}

		dVector impulse (CalculateImpulse(rowCount, rhs, low, high, normalIndex, jt));

		impulse = impulse.Scale(m_invMass);
		NewtonBodySetVelocity(m_kinematicBody, &impulse[0]);
		NewtonBodyIntegrateVelocity(m_kinematicBody, timestep);

		NewtonBodyGetMatrix(m_kinematicBody, &matrix[0][0]);
		NewtonCollision* const shape = NewtonBodyGetCollision(m_kinematicBody);

		contactCount = NewtonWorldCollide(world, &matrix[0][0], shape, this, PrefilterCallback, contactArray, 4, 0);

		penetration = 0.0f;
		for (int i = 0; i < contactCount; i++) {
			penetration = dMax(contactArray[i].m_penetration, penetration);
		}
	}

	NewtonBodySetVelocity(m_kinematicBody, &savedVeloc[0]);
	return contactCount;
}

void dCustomPlayerController::ResolveCollision()
{
	dMatrix matrix;
	NewtonWorldConvexCastReturnInfo info[D_MAX_ROWS];
	NewtonWorld* const world = m_manager->GetWorld();

	NewtonBodyGetMatrix(m_kinematicBody, &matrix[0][0]);
	NewtonCollision* const shape = NewtonBodyGetCollision(m_kinematicBody);

	int contactCount = NewtonWorldCollide(world, &matrix[0][0], shape, this, PrefilterCallback, info, 4, 0);
	if (!contactCount) {
		return;
	}

	dFloat maxPenetration = 0.0f;
	for (int i = 0; i < contactCount; i ++) {
		maxPenetration = dMax (info[i].m_penetration, maxPenetration);
	}

	if (maxPenetration > D_MAX_COLLISION_PENETRATION) {
		ResolveInterpenetrations(contactCount, info);
		NewtonBodyGetMatrix(m_kinematicBody, &matrix[0][0]);
	}
	
	int rowCount = 0;
	dVector zero(0.0f);

	dMatrix invInertia;
	dVector com(0.0f);
	dVector veloc(0.0f);
	dComplementaritySolver::dJacobian jt[D_MAX_ROWS];
	dFloat rhs[D_MAX_ROWS];
	dFloat low[D_MAX_ROWS];
	dFloat high[D_MAX_ROWS];
	dFloat impulseMag[D_MAX_ROWS];
	int normalIndex[D_MAX_ROWS];
	
	NewtonBodyGetVelocity(m_kinematicBody, &veloc[0]);
	NewtonBodyGetCentreOfMass(m_kinematicBody, &com[0]);
	NewtonBodyGetInvInertiaMatrix(m_kinematicBody, &invInertia[0][0]);

	const dMatrix localFrame (dPitchMatrix(m_headingAngle) * m_localFrame * matrix);
	

	com = matrix.TransformVector(com);
	com.m_w = 0.0f;
	for (int i = 0; i < contactCount; i++) {
		NewtonWorldConvexCastReturnInfo& contact = info[i];

		dVector point (contact.m_point[0], contact.m_point[1], contact.m_point[2], 0.0f);
		dVector normal (contact.m_normal[0], contact.m_normal[1], contact.m_normal[2], 0.0f);

		jt[rowCount].m_linear = normal;
		jt[rowCount].m_angular = (point - com).CrossProduct(normal);

		low[rowCount] = 0.0f;
		high[rowCount] = 1.0e12f;
		normalIndex[rowCount] = 0;
		dVector tmp (veloc * jt[rowCount].m_linear.Scale (1.001f));
		rhs[rowCount] = - (tmp.m_x + tmp.m_y + tmp.m_z);
		rowCount ++;
		dAssert (rowCount < (D_MAX_ROWS - 3));

		dFloat updir = localFrame.m_front.DotProduct3(normal);
		if (updir > 0.1f)
		{
			// add lateral traction friction
			dVector sideDir (localFrame.m_up.CrossProduct(normal).Normalize());

			jt[rowCount].m_linear = sideDir;
			jt[rowCount].m_angular = (point - com).CrossProduct(sideDir);

			low[rowCount] = -m_friction;
			high[rowCount] = m_friction;
			normalIndex[rowCount] = -1;
			rhs[rowCount] = m_lateralSpeed;
			rowCount++;
			dAssert (rowCount < (D_MAX_ROWS - 3));

			// add longitudinal  traction friction
			dVector frontDir (normal.CrossProduct(sideDir));
			jt[rowCount].m_linear = frontDir;
			jt[rowCount].m_angular = (point - com).CrossProduct(frontDir);

			low[rowCount] = -m_friction;
			high[rowCount] = m_friction;
			normalIndex[rowCount] = -2;
			rhs[rowCount] = m_forwardSpeed;
			rowCount++;
			dAssert(rowCount < (D_MAX_ROWS - 3));

		}
	}

	for (int i = 0; i < 3; i++) {
		jt[rowCount].m_linear = zero;
		jt[rowCount].m_angular = zero;
		jt[rowCount].m_angular[i] = dFloat(1.0f);
		rhs[rowCount] = 0.0f;
		impulseMag[rowCount] = 0;
		low[rowCount] = -1.0e12f;
		high[rowCount] = 1.0e12f;
		normalIndex[rowCount] = 0;

		rowCount ++;
		dAssert (rowCount < D_MAX_ROWS);
	}

	dVector impulse (veloc.Scale (m_mass) + CalculateImpulse(rowCount, rhs, low, high, normalIndex, jt));
	veloc = impulse.Scale(m_invMass);
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
}
