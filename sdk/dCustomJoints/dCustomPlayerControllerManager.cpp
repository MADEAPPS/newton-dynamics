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

#define D_MAX_ROWS					16
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


dCustomPlayerController* dCustomPlayerControllerManager::CreateController(const dMatrix& location, const dMatrix& localAxis, dFloat mass, dFloat radius, dFloat height, dFloat stepHeight)
{
	NewtonWorld* const world = GetWorld();

	dMatrix shapeMatrix(localAxis);
	shapeMatrix.m_posit = shapeMatrix.m_front.Scale (height * 0.5f);
	shapeMatrix.m_posit.m_w = 1.0f;

	dFloat scale = 3.0f;
	height = dMax(height - 2.0f * radius / scale, dFloat(0.1f));
	NewtonCollision* const bodyCapsule = NewtonCreateCapsule(world, radius / scale, radius / scale, height, 0, &shapeMatrix[0][0]);
	NewtonCollisionSetScale(bodyCapsule, 1.0f, scale, scale);

	// create the kinematic body
	NewtonBody* const body = NewtonCreateKinematicBody(world, bodyCapsule, &location[0][0]);

	// players must have weight, otherwise they are infinitely strong when they collide
	NewtonCollision* const shape = NewtonBodyGetCollision(body);
	NewtonBodySetMassProperties(body, mass, shape);

	// make the body collide with other dynamics bodies, by default
	NewtonBodySetCollidable(body, 1);
	NewtonDestroyCollision(bodyCapsule);

	dCustomPlayerController& controller = m_playerList.Append()->GetInfo();

	shapeMatrix.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);
	controller.m_localFrame = shapeMatrix;
	controller.m_mass = mass;
	controller.m_invMass = 1.0f / mass;
	controller.m_manager = this;
	controller.m_kinematicBody = body;
	controller.m_contactPatch = radius / scale;
	controller.m_stepHeight = dMax (stepHeight, controller.m_contactPatch * 2.0f);

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


void dCustomPlayerController::SetFrame(const dMatrix& frame)
{
	dAssert (frame.TestOrthogonal());
	m_localFrame = frame;
	m_localFrame.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);

	NewtonCollision* const capsule = NewtonBodyGetCollision(m_kinematicBody);

	dMatrix oldMatrix;
	dMatrix newMatrix(m_localFrame);
	NewtonCollisionGetMatrix(capsule, &oldMatrix[0][0]);

	newMatrix.m_posit = oldMatrix.m_posit;
	NewtonCollisionSetMatrix(capsule, &newMatrix[0][0]);
}

unsigned dCustomPlayerController::PrefilterCallback(const NewtonBody* const body, const NewtonCollision* const collision, void* const userData)
{
	dCustomPlayerController* const controller = (dCustomPlayerController*)userData;
	if (controller->GetBody() == body) {
		return false;
	}
	return 1;
}

void dCustomPlayerController::ResolveStep(dFloat timestep)
{
	dMatrix matrix;
	dMatrix stepMatrix;
	dVector veloc(0.0f);
	dVector zero(0.0f);
	NewtonWorldConvexCastReturnInfo info[16];

	NewtonBodyGetMatrix(m_kinematicBody, &matrix[0][0]);
	NewtonBodyGetVelocity(m_kinematicBody, &veloc[0]);

	dMatrix coodinateMatrix (m_localFrame * matrix);

	dComplementaritySolver::dJacobian jt[3];
	dFloat rhs[3];
	dFloat low[3];
	dFloat high[3];
	int normalIndex[3];

	jt[0].m_linear = coodinateMatrix[0];
	jt[0].m_angular = zero;
	low[0] = 0.0f;
	high[0] = 1.0e12f;
	normalIndex[0] = 0;
	rhs[0] = -m_impulse.DotProduct3(jt[0].m_linear) * m_invMass;

	// add lateral traction friction
	jt[1].m_linear = coodinateMatrix[1];
	jt[1].m_angular = zero;
	low[1] = -m_friction;
	high[1] = m_friction;
	normalIndex[1] = -1;
	dVector tmp1(veloc * jt[1].m_linear);
	rhs[1] = -m_lateralSpeed - (tmp1.m_x + tmp1.m_y + tmp1.m_z);

	// add longitudinal  traction friction
	jt[2].m_linear = coodinateMatrix[2];
	jt[2].m_angular = zero;
	low[2] = -m_friction;
	high[2] = m_friction;
	normalIndex[2] = -2;
	dVector tmp2(veloc * jt[2].m_linear);
	rhs[2] = -m_forwardSpeed - (tmp2.m_x + tmp2.m_y + tmp2.m_z);
	
	dVector impulse(veloc.Scale(m_mass) + CalculateImpulse(3, rhs, low, high, normalIndex, jt));

	impulse = impulse.Scale(m_invMass);
	NewtonBodySetVelocity(m_kinematicBody, &impulse[0]);
	NewtonBodyIntegrateVelocity(m_kinematicBody, timestep);

	NewtonWorld* const world = m_manager->GetWorld();
	NewtonCollision* const shape = NewtonBodyGetCollision(m_kinematicBody);
	
	NewtonBodyGetMatrix(m_kinematicBody, &stepMatrix[0][0]);
	int contactCount = NewtonWorldCollide(world, &stepMatrix[0][0], shape, this, PrefilterCallback, info, 4, 0);

	NewtonBodySetMatrix(m_kinematicBody, &matrix[0][0]);
	NewtonBodySetVelocity(m_kinematicBody, &veloc[0]);

	dFloat maxHigh = 0.0f;
	for (int i = 0; i < contactCount; i++) {
		NewtonWorldConvexCastReturnInfo& contact = info[i];
		dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], 0.0f);
		point = m_localFrame.UntransformVector (stepMatrix.UntransformVector(point));
		maxHigh = dMax (point.m_x, maxHigh);
	}
	if ((maxHigh < m_stepHeight) && (maxHigh > m_contactPatch)) {
		dVector step (stepMatrix.RotateVector(m_localFrame.RotateVector (dVector(maxHigh, dFloat(0.0f), dFloat(0.0f), dFloat(0.0f)))));
		matrix.m_posit += step;
		NewtonBodySetMatrix(m_kinematicBody, &matrix[0][0]);
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

		impulseMag[i] = 0.0f;
		for (int j = i + 1; j < rows; j++) {
			dVector tmp1(jInvMass.m_linear * jt[j].m_linear + jInvMass.m_angular * jt[j].m_angular);
			dFloat a01 = tmp1.m_x + tmp1.m_y + tmp1.m_z;
			massMatrix[i][j] = a01;
			massMatrix[j][i] = a01;
		}
	}

	dGaussSeidelLcpSor(rows, D_MAX_ROWS, &massMatrix[0][0], impulseMag, rhs, normalIndex, low, high, dFloat(1.0e-6f), 32, dFloat(1.1f));

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
	for (int j = 0; (j < 8) && (penetration > D_MAX_COLLISION_PENETRATION) ; j ++) {
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
			penetration = dClamp(contact.m_penetration - D_MAX_COLLISION_PENETRATION * 0.5f, dFloat(0.0f), dFloat(0.5f));
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

	const dMatrix localFrame (m_localFrame * matrix);

	com = matrix.TransformVector(com);
	com.m_w = 0.0f;
	for (int i = 0; i < contactCount; i ++) {
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

		//dFloat updir = localFrame.m_front.DotProduct3(normal);
		dFloat friction = m_manager->ContactFriction(this, point, normal, int (contact.m_contactID), contact.m_hitBody);
		if (friction > 0.0f)
		{
			// add lateral traction friction
			dVector sideDir (localFrame.m_up.CrossProduct(normal).Normalize());

			jt[rowCount].m_linear = sideDir;
			jt[rowCount].m_angular = (point - com).CrossProduct(sideDir);

			low[rowCount] = -friction;
			high[rowCount] = friction;
			normalIndex[rowCount] = -1;

			dVector tmp1 (veloc * jt[rowCount].m_linear);
			rhs[rowCount] =  -m_lateralSpeed - (tmp1.m_x + tmp1.m_y + tmp1.m_z);
			rowCount++;
			dAssert (rowCount < (D_MAX_ROWS - 3));

			// add longitudinal  traction friction
			dVector frontDir (normal.CrossProduct(sideDir));
			jt[rowCount].m_linear = frontDir;
			jt[rowCount].m_angular = (point - com).CrossProduct(frontDir);

			low[rowCount] = -friction;
			high[rowCount] = friction;
			normalIndex[rowCount] = -2;
			dVector tmp2 (veloc * jt[rowCount].m_linear);
			rhs[rowCount] = -m_forwardSpeed - (tmp2.m_x + tmp2.m_y + tmp2.m_z);
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

void dCustomPlayerController::PreUpdate(dFloat timestep)
{
	dFloat timeLeft = timestep;
	const dFloat timeEpsilon = timestep * (1.0f / 16.0f);

	m_impulse = dVector(0.0f);
	m_manager->ApplyMove(this, timestep);

//SetForwardSpeed(1.0f);
//SetLateralSpeed(0.0f);
//SetHeadingAngle(45.0f*dDegreeToRad);

	// set player orientation
	dMatrix matrix(dYawMatrix(GetHeadingAngle()));
	NewtonBodyGetPosition(m_kinematicBody, &matrix.m_posit[0]);
	NewtonBodySetMatrix(m_kinematicBody, &matrix[0][0]);

	// set play desired velocity
	dVector veloc(GetVelocity() + m_impulse.Scale(m_invMass));
	NewtonBodySetVelocity(m_kinematicBody, &veloc[0]);

	// determine if player has to step over obstacles lower than step hight
	ResolveStep(timestep);

	// advance player until it hit a collision point, until there is not more time left
	for (int i = 0; (i < D_DESCRETE_MOTION_STEPS) && (timeLeft > timeEpsilon); i++) {
		if (timeLeft > timeEpsilon) {
			ResolveCollision();
		}

		dFloat predicetdTime = PredictTimestep(timestep);
		NewtonBodyIntegrateVelocity(m_kinematicBody, predicetdTime);
		timeLeft -= predicetdTime;
	}
}
