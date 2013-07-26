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


// NewtonCustomJoint.cpp: implementation of the NewtonCustomJoint class.
//
//////////////////////////////////////////////////////////////////////
#include "CustomJointLibraryStdAfx.h"
#include "CustomPlayerControllerManager.h"


#define D_DESCRETE_MOTION_STEPS				8
#define D_PLAYER_MAX_INTERGRATION_STEPS		8
#define D_PLAYER_MAX_SOLVER_ITERATIONS		16


CustomPlayerControllerManager::CustomPlayerControllerManager(NewtonWorld* const world)
	:CustomControllerManager<CustomPlayerController> (world, PLAYER_PLUGIN_NAME)
{
}

CustomPlayerControllerManager::~CustomPlayerControllerManager()
{
}

CustomPlayerControllerManager::CustomController* CustomPlayerControllerManager::CreatePlayer (dFloat mass, dFloat outerRadius, dFloat innerRadius, dFloat height, dFloat stairStep, const dMatrix& localAxis)
{
	CustomPlayerControllerManager::CustomController* const controller = CreateController();
	controller->Init (mass, outerRadius, innerRadius, height, stairStep, localAxis);
	return controller;
}

void CustomPlayerControllerManager::DestroyController (CustomController* const controller)
{
	controller->Cleanup();
}

int CustomPlayerControllerManager::ProcessContacts (const CustomPlayerController* const controller, NewtonWorldConvexCastReturnInfo* const contacts, int contactCount) const 
{
	//	for (int i = 0; i < contactCount; i ++) {
	//		const dgVector normal = contacts[i].m_normal;
	//		const dgVector position = contacts[i].m_normal;
	//	}
	return contactCount;
}


void CustomPlayerController::Init(dFloat mass, dFloat outerRadius, dFloat innerRadius, dFloat height, dFloat stairStep, const dMatrix& localAxis)
{
	dAssert (stairStep >= 0.0f);
	dAssert (innerRadius >= 0.0f);
	dAssert (outerRadius >= innerRadius);
	dAssert (height >= stairStep);
	dAssert (localAxis[0].m_w == dFloat (0.0f));
	dAssert (localAxis[1].m_w == dFloat (0.0f));

	CustomPlayerControllerManager* const manager = (CustomPlayerControllerManager*) GetManager();
	NewtonWorld* const world = manager->GetWorld();

	SetRestrainingDistance (0.0f);

	m_outerRadio = outerRadius;
	m_innerRadio = innerRadius;
	m_height = height;
	m_stairStep = stairStep;
	SetClimbSlope(45.0f * 3.1416f/ 180.0f);
	m_upVector = localAxis[0];
	m_frontVector = localAxis[1];

	m_groundPlane = dVector (0.0f, 0.0f, 0.0f, 0.0f);
	m_groundVelocity = dVector (0.0f, 0.0f, 0.0f, 0.0f);

	const int steps = 12;
	dVector convexPoints[2][steps];

	// create an inner thin cylinder
	dFloat shapeHigh = height;
	dAssert (shapeHigh > 0.0f);
	dVector p0 (0.0f, m_innerRadio, 0.0f, 0.0f);
	dVector p1 (shapeHigh, m_innerRadio, 0.0f, 0.0f);
	for (int i = 0; i < steps; i ++) {
		dMatrix rotation (dPitchMatrix (i * 2.0f * 3.141592f / steps));
		convexPoints[0][i] = localAxis.RotateVector(rotation.RotateVector(p0));
		convexPoints[1][i] = localAxis.RotateVector(rotation.RotateVector(p1));
	}
	NewtonCollision* const supportShape = NewtonCreateConvexHull(world, steps * 2, &convexPoints[0][0].m_x, sizeof (dVector), 0.0f, 0, NULL); 

	// create the outer thick cylinder
	dMatrix outerShapeMatrix (localAxis);
	dFloat capsuleHigh = m_height - stairStep;
	dAssert (capsuleHigh > 0.0f);
	m_sphereCastOrigin = capsuleHigh * 0.5f + stairStep;
	outerShapeMatrix.m_posit = outerShapeMatrix[0].Scale(m_sphereCastOrigin);
	outerShapeMatrix.m_posit.m_w = 1.0f;
	NewtonCollision* const bodyCapsule = NewtonCreateCapsule(world, 0.25f, 0.5f, 0, &outerShapeMatrix[0][0]);
	NewtonCollisionSetScale(bodyCapsule, capsuleHigh, m_outerRadio * 4.0f, m_outerRadio * 4.0f);

	// compound collision player controller
	NewtonCollision* const playerShape = NewtonCreateCompoundCollision(world, 0);
	NewtonCompoundCollisionBeginAddRemove(playerShape);	
	NewtonCompoundCollisionAddSubCollision (playerShape, supportShape);
	NewtonCompoundCollisionAddSubCollision (playerShape, bodyCapsule);
	NewtonCompoundCollisionEndAddRemove (playerShape);	

	// create the kinematic body
	dMatrix locationMatrix (GetIdentityMatrix());
	NewtonBody* const body = NewtonCreateKinematicBody(world, playerShape, &locationMatrix[0][0]);

	// players must have weight, otherwise they are infinitely strong when they collide
	NewtonCollision* const shape = NewtonBodyGetCollision(body);
	NewtonBodySetMassProperties(body, mass, shape);

	// make the body collidable with other dynamics bodies, by default
	NewtonSetBodyCollidable (body, true);
	
	SetBody (body);

	dFloat castHigh = capsuleHigh * 0.4f;
	dFloat castRadio = (m_innerRadio * 0.5f > 0.05f) ? m_innerRadio * 0.5f : 0.05f;

	dVector q0 (0.0f, castRadio, 0.0f, 0.0f);
	dVector q1 (castHigh, castRadio, 0.0f, 0.0f);
	for (int i = 0; i < steps; i ++) {
		dMatrix rotation (dPitchMatrix (i * 2.0f * 3.141592f / steps));
		convexPoints[0][i] = localAxis.RotateVector(rotation.RotateVector(q0));
		convexPoints[1][i] = localAxis.RotateVector(rotation.RotateVector(q1));
	}
	m_castingShape = NewtonCreateConvexHull(world, steps * 2, &convexPoints[0][0].m_x, sizeof (dVector), 0.0f, 0, NULL); 

	
	m_supportShape = NewtonCompoundCollisionGetCollisionFromNode (shape, NewtonCompoundCollisionGetNodeByIndex (shape, 0));
	m_upperBodyShape = NewtonCompoundCollisionGetCollisionFromNode (shape, NewtonCompoundCollisionGetNodeByIndex (shape, 1));

	NewtonDestroyCollision (bodyCapsule);
	NewtonDestroyCollision (supportShape);
	NewtonDestroyCollision (playerShape);

	m_isJumping = false;
}

void CustomPlayerController::Cleanup()
{
	if (m_castingShape) {
		NewtonDestroyCollision (m_castingShape);
	}
	m_castingShape = NULL;
}

void CustomPlayerController::SetPlayerOrigin (dFloat originHigh)
{
	NewtonCollision* const playerShape = NewtonBodyGetCollision(GetBody());
	NewtonCompoundCollisionBeginAddRemove(playerShape);	

		dMatrix supportShapeMatrix (GetIdentityMatrix());
		supportShapeMatrix[0] = m_upVector;
		supportShapeMatrix[1] = m_frontVector;
		supportShapeMatrix[2] = supportShapeMatrix[0] * supportShapeMatrix[1];
		supportShapeMatrix.m_posit = supportShapeMatrix[0].Scale(m_height * 0.5f - originHigh);
		supportShapeMatrix.m_posit.m_w = 1.0f;
		NewtonCollisionSetMatrix (m_supportShape, &supportShapeMatrix[0][0]);

		dMatrix collisionShapeMatrix (supportShapeMatrix);
		dFloat cylinderHeight = m_height - m_stairStep;
		dAssert (cylinderHeight > 0.0f);
		collisionShapeMatrix.m_posit = collisionShapeMatrix[0].Scale(cylinderHeight * 0.5f + m_stairStep - originHigh);
		collisionShapeMatrix.m_posit.m_w = 1.0f;
		NewtonCollisionSetMatrix (m_upperBodyShape, &collisionShapeMatrix[0][0]);

	NewtonCompoundCollisionEndAddRemove (playerShape);	

	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	NewtonBodyGetMassMatrix(GetBody(), &mass, &Ixx, &Iyy, &Izz);
	NewtonBodySetMassProperties(GetBody(), mass, playerShape);
}

void CustomPlayerController::PreUpdate(dFloat timestep, int threadIndex)
{
}


dVector CustomPlayerController::CalculateDesiredOmega (dFloat headingAngle, dFloat timestep) const
{
	dQuaternion playerRotation;
	dQuaternion targetRotation (m_upVector, headingAngle);
	NewtonBodyGetRotation(GetBody(), &playerRotation.m_q0);
	return playerRotation.CalcAverageOmega (targetRotation, 0.5f / timestep);
}

dVector CustomPlayerController::CalculateDesiredVelocity (dFloat forwardSpeed, dFloat lateralSpeed, dFloat verticalSpeed, const dVector& gravity, dFloat timestep) const
{
	NewtonBody* const body = GetBody();
	dMatrix matrix;
	NewtonBodyGetMatrix(body, &matrix[0][0]);
	dVector updir (matrix.RotateVector(m_upVector));
	dVector frontDir (matrix.RotateVector(m_frontVector));
	dVector rightDir (frontDir * updir);
	

	dVector veloc (0.0f, 0.0f, 0.0f, 0.0f);
	if ((verticalSpeed <= 0.0f) && (m_groundPlane % m_groundPlane) > 0.0f) {
		// plane is supported by a ground plane, apply the player input velocity
		if ((m_groundPlane % updir) >= m_maxSlope) {
			// player is in a legal slope, he is in full control of his movement
			dVector bodyVeloc;
			NewtonBodyGetVelocity(body, &bodyVeloc[0]);
			veloc = updir.Scale(bodyVeloc % updir) + gravity.Scale (timestep) + frontDir.Scale (forwardSpeed) + rightDir.Scale (lateralSpeed) + updir.Scale(verticalSpeed);
			veloc += (m_groundVelocity - updir.Scale (updir % m_groundVelocity));
			dFloat normalVeloc = m_groundPlane % (veloc - m_groundVelocity);
			if (normalVeloc < 0.0f) {
				veloc -= m_groundPlane.Scale (normalVeloc);
			}
			dFloat speedLimitMag2 = forwardSpeed * forwardSpeed + lateralSpeed * lateralSpeed + verticalSpeed * verticalSpeed;
			dFloat speedMag2 = veloc % veloc;
			if (speedMag2 > speedLimitMag2) {
				if (speedLimitMag2 < dFloat (1.0e-6f)) {
					veloc = dVector (0.0f, 0.0f, 0.0f, 0.0f);
				} else {
					veloc = veloc.Scale (dSqrt (speedLimitMag2 / speedMag2));
				}
			}
		} else {
			// player is in an illegal ramp, he slides down hill an loses control of his movement 
			NewtonBodyGetVelocity(body, &veloc[0]);
			veloc += updir.Scale(verticalSpeed);
			veloc += gravity.Scale (timestep);
			dFloat normalVeloc = m_groundPlane % (veloc - m_groundVelocity);
			if (normalVeloc < 0.0f) {
				veloc -= m_groundPlane.Scale (normalVeloc);
			}
		}
	} else {
		// player is on free fall, only apply the gravity
		NewtonBodyGetVelocity(body, &veloc[0]);
		veloc += updir.Scale(verticalSpeed);
		veloc += gravity.Scale (timestep);
	}
	return veloc;
}

void CustomPlayerController::SetPlayerVelocity (dFloat forwardSpeed, dFloat lateralSpeed, dFloat verticalSpeed, dFloat headingAngle, const dVector& gravity, dFloat timestep)
{
	dVector omega (CalculateDesiredOmega (headingAngle, timestep));
	dVector veloc (CalculateDesiredVelocity (forwardSpeed, lateralSpeed, verticalSpeed, gravity, timestep));			

	NewtonBody* const body = GetBody();
	NewtonBodySetOmega(body, &omega[0]);
	NewtonBodySetVelocity(body, &veloc[0]);

	if ((verticalSpeed > 0.0f)) {
		m_isJumping = true;
	}
}

dFloat CustomPlayerController::CalculateContactKinematics(const dVector& veloc, const NewtonWorldConvexCastReturnInfo* const contactInfo) const
{
	dVector contactVeloc;
	NewtonBodyGetPointVelocity (contactInfo->m_hitBody, contactInfo->m_point, &contactVeloc[0]);

	const dFloat restitution = 0.0f;
	dVector normal (contactInfo->m_normal);
	dFloat reboundVelocMag = -((veloc - contactVeloc)% normal) * (1.0f + restitution);
	return (reboundVelocMag > 0.0f) ? reboundVelocMag : 0.0f; 
}



void CustomPlayerController::UpdateGroundPlane (dMatrix& matrix, const dMatrix& castMatrix, const dVector& dst, int threadIndex)
{
	CustomPlayerControllerManager* const manager = (CustomPlayerControllerManager*) GetManager();
	NewtonWorld* const world = manager->GetWorld();

	CustomControllerConvexRayFilter filter(GetBody());
	NewtonWorldConvexRayCast (world, m_castingShape, &castMatrix[0][0], &dst[0], CustomControllerConvexRayFilter::Filter, &filter, CustomControllerConvexRayFilter::Prefilter, threadIndex);

	m_groundPlane = dVector (0.0f, 0.0f, 0.0f, 0.0f);
	m_groundVelocity = dVector (0.0f, 0.0f, 0.0f, 0.0f);

	if (filter.m_hitBody) {
		m_isJumping = false;
		dVector supportPoint (castMatrix.m_posit + (dst - castMatrix.m_posit).Scale (filter.m_intersectParam));
		m_groundPlane = filter.m_hitNormal;
		m_groundPlane.m_w = - (supportPoint % filter.m_hitNormal);
		NewtonBodyGetPointVelocity (filter.m_hitBody, &supportPoint.m_x, &m_groundVelocity[0]);
		matrix.m_posit = supportPoint;
		matrix.m_posit.m_w = 1.0f;
	}
}

void CustomPlayerController::PostUpdate(dFloat timestep, int threadIndex)
{
	dMatrix matrix; 
	dQuaternion bodyRotation;
	dVector veloc(0.0f, 0.0f, 0.0f, 0.0f); 
	dVector omega(0.0f, 0.0f, 0.0f, 0.0f);  

	CustomPlayerControllerManager* const manager = (CustomPlayerControllerManager*) GetManager();
	NewtonWorld* const world = manager->GetWorld();
	NewtonBody* const body = GetBody();

	// apply the player motion, by calculation the desired plane linear and angular velocity
	manager->ApplyPlayerMove (this, timestep);

	// get the body motion state 
	NewtonBodyGetMatrix(body, &matrix[0][0]);
	NewtonBodyGetVelocity(body, &veloc[0]);
	NewtonBodyGetOmega(body, &omega[0]);

	// integrate body angular velocity
	NewtonBodyGetRotation (body, &bodyRotation.m_q0); 
	bodyRotation = bodyRotation.IntegrateOmega(omega, timestep);
	matrix = dMatrix (bodyRotation, matrix.m_posit);

	// integrate linear velocity
	dFloat normalizedTimeLeft = 1.0f; 
	dFloat step = timestep * dSqrt (veloc % veloc) ;
	dFloat descreteTimeStep = timestep * (1.0f / D_DESCRETE_MOTION_STEPS);
	int prevContactCount = 0;
	CustomControllerConvexCastPreFilter castFilterData (body);
	NewtonWorldConvexCastReturnInfo prevInfo[PLAYER_CONTROLLER_MAX_CONTACTS];

	dVector scale;
	NewtonCollisionGetScale (m_upperBodyShape, &scale.m_x, &scale.m_y, &scale.m_z);
//	NewtonCollisionSetScale (m_upperBodyShape, m_height - m_stairStep, (m_outerRadio + m_restrainingDistance) * 4.0f, (m_outerRadio + m_restrainingDistance) * 4.0f);
	NewtonCollisionSetScale (m_upperBodyShape, m_height - m_stairStep, m_outerRadio * 4.0f, m_outerRadio * 4.0f);
	for (int i = 0; (i < D_PLAYER_MAX_INTERGRATION_STEPS) && (normalizedTimeLeft > 1.0e-5f); i ++ ) {
		if ((veloc % veloc) < 1.0e-6f) {
			break;
		}

		dFloat timetoImpact;
		NewtonWorldConvexCastReturnInfo info[PLAYER_CONTROLLER_MAX_CONTACTS];
		dVector destPosit (matrix.m_posit + veloc.Scale (timestep));
		int contactCount = NewtonWorldConvexCast (world, &matrix[0][0], &destPosit[0], m_upperBodyShape, &timetoImpact, &castFilterData, CustomControllerConvexCastPreFilter::Prefilter, info, sizeof (info) / sizeof (info[0]), threadIndex);
		if (contactCount) {
			contactCount = manager->ProcessContacts (this, info, contactCount);
		}

		if (contactCount) {
			if (timetoImpact > 0.0f) {
				matrix.m_posit += veloc.Scale (timetoImpact * timestep);
			}
			normalizedTimeLeft -= timetoImpact;

			dFloat speed[PLAYER_CONTROLLER_MAX_CONTACTS * 2];
			dFloat bounceSpeed[PLAYER_CONTROLLER_MAX_CONTACTS * 2];
			dVector bounceNormal[PLAYER_CONTROLLER_MAX_CONTACTS * 2];

			int count = 0;
			for (int i = 0; i < contactCount; i ++) {
				speed[count] = 0.0f;
				bounceNormal[count] = dVector (info[i].m_normal);
				bounceSpeed[count] = CalculateContactKinematics(veloc, &info[i]);
				count ++;
			}

			for (int i = 0; i < prevContactCount; i ++) {
				speed[count] = 0.0f;
				bounceNormal[count] = dVector (prevInfo[i].m_normal);
				bounceSpeed[count] = CalculateContactKinematics(veloc, &prevInfo[i]);
				count ++;
			}

			dFloat residual = 10.0f;
			dVector auxBounceVeloc (0.0f, 0.0f, 0.0f, 0.0f);
			for (int i = 0; (i < D_PLAYER_MAX_SOLVER_ITERATIONS) && (residual > 1.0e-3f); i ++) {
				residual = 0.0f;
				for (int k = 0; k < count; k ++) {
					dVector normal (bounceNormal[k]);
					dFloat v = bounceSpeed[k] - normal % auxBounceVeloc;
					dFloat x = speed[k] + v;
					if (x < 0.0f) {
						v = 0.0f;
						x = 0.0f;
					}

					if (dAbs (v) > residual) {
						residual = dAbs (v);
					}

					auxBounceVeloc += normal.Scale (x - speed[k]);
					speed[k] = x;
				}
			}

			dVector velocStep (0.0f, 0.0f, 0.0f, 0.0f);
			for (int i = 0; i < count; i ++) {
				dVector normal (bounceNormal[i]);
				velocStep += normal.Scale (speed[i]);
			}
			veloc += velocStep;

			dFloat velocMag2 = velocStep % velocStep;
			if (velocMag2 < 1.0e-6f) {
				dFloat advanceTime = dMin (descreteTimeStep, normalizedTimeLeft * timestep);
				matrix.m_posit += veloc.Scale (advanceTime);
				normalizedTimeLeft -= advanceTime / timestep;
			}

			prevContactCount = contactCount;
			memcpy (prevInfo, info, prevContactCount * sizeof (NewtonWorldConvexCastReturnInfo));

		} else {
			matrix.m_posit = destPosit;
			matrix.m_posit.m_w = 1.0f;
			break;
		}
	}
	NewtonCollisionSetScale (m_upperBodyShape, scale.m_x, scale.m_y, scale.m_z);

	// determine if player is standing on some plane
	if (step > 1.0e-6f) {
		dMatrix supportMatrix (matrix);
		dVector updir (matrix.RotateVector(m_upVector));
		supportMatrix.m_posit += updir.Scale (m_sphereCastOrigin);

		if (m_isJumping) {
			dVector dst (matrix.m_posit);
			UpdateGroundPlane (matrix, supportMatrix, dst, threadIndex);
		} else {
			step = dAbs (updir % veloc.Scale (timestep));
			dFloat castDist = ((m_groundPlane % m_groundPlane) > 0.0f) ? m_stairStep : step;
			dVector dst (matrix.m_posit - updir.Scale (castDist * 2.0f));
			UpdateGroundPlane (matrix, supportMatrix, dst, threadIndex);
		}
	}

	// set player velocity, position and orientation
	NewtonBodySetVelocity(body, &veloc[0]);
	NewtonBodySetMatrix (body, &matrix[0][0]);
}

