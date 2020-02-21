/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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

#include "dStdafxVehicle.h"
#include "dVehicle.h"
#include "dVehicleNode.h"
#include "dVehicleManager.h"
#include "dPlayerController.h"
#include "dPlayerControllerContactSolver.h"

dPlayerController::dPlayerController(NewtonWorld* const world, const dMatrix& location, const dMatrix& localAxis, dFloat mass, dFloat radius, dFloat height, dFloat stepHeight)
	:dVehicle(NULL, localAxis, 10.0f)
	,m_impulse(0.0f)
	,m_mass(mass)
	,m_invMass(1.0f / mass)
	,m_headingAngle(0.0f)
	,m_forwardSpeed(0.0f)
	,m_lateralSpeed(0.0f)
	,m_stepHeight(0.0f)
	,m_contactPatch(0.0f)
	,m_height(height)
	,m_weistScale(3.0f)
	,m_crouchScale(0.5f)
	,m_userData(NULL)
	,m_isAirbone(false)
	,m_isOnFloor(false)
	,m_isCrouched(false)
{
	dMatrix shapeMatrix(localAxis);
	shapeMatrix.m_posit = shapeMatrix.m_front.Scale(height * 0.5f);
	shapeMatrix.m_posit.m_w = 1.0f;

	height = dMax(height - 2.0f * radius / m_weistScale, dFloat(0.1f));
	NewtonCollision* const bodyCapsule = NewtonCreateCapsule(world, radius / m_weistScale, radius / m_weistScale, height, 0, &shapeMatrix[0][0]);
	NewtonCollisionSetScale(bodyCapsule, 1.0f, m_weistScale, m_weistScale);

	// create the kinematic body
	m_newtonBody = NewtonCreateKinematicBody(world, bodyCapsule, &location[0][0]);

	// players must have weight, otherwise they are infinitely strong when they collide
	NewtonCollision* const shape = NewtonBodyGetCollision(m_newtonBody);
	NewtonBodySetMassProperties(m_newtonBody, mass, shape);

	// make the body collide with other dynamics bodies, by default
	NewtonBodySetCollidable(m_newtonBody, 1);
	NewtonDestroyCollision(bodyCapsule);

	shapeMatrix.m_posit = dVector(0.0f, dFloat(0.0f), dFloat(0.0f), 1.0f);
	m_localFrame = shapeMatrix;
	m_contactPatch = radius / m_weistScale;
	m_stepHeight = dMax(stepHeight, m_contactPatch * dFloat(2.0f));
}

dPlayerController::~dPlayerController()
{
	if (m_managerNode) {
		m_manager->RemoveRoot(this);
	}
}

void dPlayerController::ToggleCrouch()
{
	dMatrix matrix;
	m_isCrouched = !m_isCrouched;

	NewtonCollision* const shape = NewtonBodyGetCollision(m_newtonBody);
	NewtonCollisionGetMatrix(shape, &matrix[0][0]);
	if (m_isCrouched) {
		matrix.m_posit = matrix.m_front.Scale(m_height * m_crouchScale * 0.5f);
		NewtonCollisionSetScale(shape, m_crouchScale, m_weistScale, m_weistScale);
	} else {
		matrix.m_posit = matrix.m_front.Scale(m_height * 0.5f);
		NewtonCollisionSetScale(shape, dFloat(1.0f), m_weistScale, m_weistScale);
	}
	matrix.m_posit.m_w = 1.0f;
	NewtonCollisionSetMatrix(shape, &matrix[0][0]);
}

void dPlayerController::PreUpdate(dFloat timestep)
{
	dPlayerControllerContactSolver contactSolver(this);

	dFloat timeLeft = timestep;
	const dFloat timeEpsilon = timestep * (1.0f / 16.0f);

#if 0
	m_impulse = dVector(0.0f);
	m_manager->ApplyMove(this, timestep);

#if 0
	#if 0
		static FILE* file = fopen("log.bin", "wb");
		if (file) {
			fwrite(&m_headingAngle, sizeof(m_headingAngle), 1, file);
			fwrite(&m_forwardSpeed, sizeof(m_forwardSpeed), 1, file);
			fwrite(&m_lateralSpeed, sizeof(m_lateralSpeed), 1, file);
			fflush(file);
		}
	#else 
		static FILE* file = fopen("log.bin", "rb");
		if (file) {
			fread(&m_headingAngle, sizeof(m_headingAngle), 1, file);
			fread(&m_forwardSpeed, sizeof(m_forwardSpeed), 1, file);
			fread(&m_lateralSpeed, sizeof(m_lateralSpeed), 1, file);
		}
	#endif
#endif

	// set player orientation
	dMatrix matrix(dYawMatrix(GetHeadingAngle()));
	NewtonBodyGetPosition(m_kinematicBody, &matrix.m_posit[0]);
	NewtonBodySetMatrix(m_kinematicBody, &matrix[0][0]);

	// set play desired velocity
	dVector veloc(GetVelocity() + m_impulse.Scale(m_invMass));
	SetVelocity(veloc);

	// determine if player has to step over obstacles lower than step hight
	ResolveStep(timestep, contactSolver);

	// advance player until it hit a collision point, until there is not more time left
	for (int i = 0; (i < D_DESCRETE_MOTION_STEPS) && (timeLeft > timeEpsilon); i++) {
		if (timeLeft > timeEpsilon) {
			ResolveCollision(contactSolver, timestep);
		}

		dFloat predicetdTime = PredictTimestep(timeLeft, contactSolver);
		NewtonBodyIntegrateVelocity(m_kinematicBody, predicetdTime);
		timeLeft -= predicetdTime;
	}

	UpdatePlayerStatus(contactSolver);
#endif
}

