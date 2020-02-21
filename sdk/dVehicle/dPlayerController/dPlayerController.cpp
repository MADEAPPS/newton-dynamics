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

dPlayerController::dPlayerController(NewtonWorld* const world, const dMatrix& location, const dMatrix& localAxis, dFloat mass, dFloat radius, dFloat height, dFloat stepHeight)
	:dVehicle(NULL, localAxis, 10.0f)
//	,m_localFrame(dGetIdentityMatrix())
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
	,m_kinematicBody(NULL)
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
	m_kinematicBody = NewtonCreateKinematicBody(world, bodyCapsule, &location[0][0]);

	// players must have weight, otherwise they are infinitely strong when they collide
	NewtonCollision* const shape = NewtonBodyGetCollision(m_kinematicBody);
	NewtonBodySetMassProperties(m_kinematicBody, mass, shape);

	// make the body collide with other dynamics bodies, by default
	NewtonBodySetCollidable(m_kinematicBody, 1);
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

void dPlayerController::PreUpdate(dFloat timestep)
{
	//dAssert (0);
	dTrace (("implemet this\n"));
}

void dPlayerController::PostUpdate(dFloat timestep)
{
	dTrace (("implemet this\n"));
}
