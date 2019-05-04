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
	dAssert(timeLeft < timeEpsilon);
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


void dCustomPlayerController::ResolveCollision()
{
	dMatrix matrix;
	NewtonWorldConvexCastReturnInfo info[D_MAX_ROWS - 3];

	NewtonWorld* const world = m_manager->GetWorld();

	NewtonBodyGetMatrix(m_kinematicBody, &matrix[0][0]);
	NewtonCollision* const shape = NewtonBodyGetCollision(m_kinematicBody);

	int contactCount = NewtonWorldCollide(world, &matrix[0][0], shape, this, PrefilterCallback, info, 4, 0);
	if (!contactCount) {
		return;
	}

	int rowCount = 0;
	dVector zero(0.0f);

	dMatrix invInertia;
	dVector com(0.0f);
	dVector veloc(0.0f);
	dComplementaritySolver::dJacobian jt[D_MAX_ROWS];
	dComplementaritySolver::dJacobian jInvMass[D_MAX_ROWS];
	dFloat rhs[D_MAX_ROWS];
	dFloat low[D_MAX_ROWS];
	dFloat high[D_MAX_ROWS];
	dFloat impulseMag[D_MAX_ROWS];
	int normalIndex[D_MAX_ROWS];
	dFloat massMatrix[D_MAX_ROWS][D_MAX_ROWS];
	
	NewtonBodyGetVelocity(m_kinematicBody, &veloc[0]);
	NewtonBodyGetCentreOfMass(m_kinematicBody, &com[0]);
	NewtonBodyGetInvInertiaMatrix(m_kinematicBody, &invInertia[0][0]);

	com = matrix.TransformVector(com);

	for (int i = 0; i < contactCount; i++) {
		NewtonWorldConvexCastReturnInfo& contact = info[i];
		dVector point (contact.m_point[0], contact.m_point[1], contact.m_point[2], 0.0f);

		jt[rowCount].m_linear = contact.m_normal;
		jt[rowCount].m_angular = (point - com).CrossProduct(contact.m_normal);
		jt[rowCount].m_angular.m_w = 0.0f;

		jInvMass[rowCount].m_linear = jt[rowCount].m_linear.Scale (m_invMass);
		jInvMass[rowCount].m_angular = invInertia.UnrotateVector(jt[rowCount].m_angular);

		low[rowCount] = 0.0f;
		high[rowCount] = 1.0e12f;
		normalIndex[rowCount] = 0;

		dVector tmp (m_impulse * jInvMass[rowCount].m_linear + veloc * jt[rowCount].m_linear);
		rhs[rowCount] = - (tmp.m_x + tmp.m_y + tmp.m_z);
		rowCount ++;
	}

	for (int i = 0; i < 3; i++) {
		jt[rowCount].m_linear = zero;
		jt[rowCount].m_angular = zero;
		jt[rowCount].m_angular[i] = dFloat(1.0f);

		jInvMass[rowCount].m_linear = zero;
		jInvMass[rowCount].m_angular = invInertia.UnrotateVector(jt[rowCount].m_angular);

		rhs[rowCount] = 0.0f;
		impulseMag[rowCount] = 0;
		low[rowCount] = -1.0e12f;
		high[rowCount] = 1.0e12f;
		normalIndex[rowCount] = 0;

		rowCount ++;
		dAssert (rowCount < D_MAX_ROWS);
	}

	for (int i = 0; i < rowCount; i++) {
		const dComplementaritySolver::dJacobian& J0 = jInvMass[i];
		dVector tmp(J0.m_linear * jt[i].m_linear + J0.m_angular * jt[i].m_angular);

		dFloat a00 = (tmp.m_x + tmp.m_y + tmp.m_z) * 1.0001f;
		massMatrix[i][i] = a00;

		for (int j = i + 1; j < rowCount; j++) {
			dVector tmp1(J0.m_linear * jt[j].m_linear + J0.m_angular * jt[j].m_angular);
			dFloat a01 = tmp1.m_x + tmp1.m_y + tmp1.m_z;
			massMatrix[i][j] = a01;
			massMatrix[j][i] = a01;
		}
	}

	dGaussSeidelLcpSor(rowCount, D_MAX_ROWS, &massMatrix[0][0], impulseMag, rhs, normalIndex, low, high, 1.0e-2f, 32, 1.1f);

	dVector netImpulse (m_impulse + veloc.Scale (m_mass));
	for (int i = 0; i < contactCount; i++) {
		netImpulse += jt[i].m_linear.Scale (impulseMag[i]);
	}

	veloc = netImpulse.Scale (m_invMass);
	NewtonBodySetVelocity(m_kinematicBody, &veloc[0]);
}
