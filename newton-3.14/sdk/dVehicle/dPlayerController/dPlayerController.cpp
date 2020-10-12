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
#include "dPlayerControllerImpulseSolver.h"

#define D_DESCRETE_MOTION_STEPS		4
#define D_MAX_COLLIONSION_STEPS		8
#define D_MAX_COLLISION_PENETRATION	dFloat (5.0e-3f)

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
//	,m_userData(NULL)
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

dVector dPlayerController::GetVelocity() const
{
	dVector veloc(0.0f);
	NewtonBodyGetVelocity(m_newtonBody, &veloc[0]);
	return veloc;
}

void dPlayerController::SetVelocity(const dVector& veloc)
{
	dAssert(veloc.DotProduct3(veloc) < 10000.0f);
	NewtonBodySetVelocity(m_newtonBody, &veloc[0]);
}

void dPlayerController::ResolveStep(dFloat timestep, dPlayerControllerContactSolver& contactSolver)
{
	dMatrix matrix;
	dVector zero(0.0f);
	dVector saveVeloc(0.0f);

	//static int xxx;
	//xxx++;

	NewtonBodyGetMatrix(m_newtonBody, &matrix[0][0]);
	NewtonBodyGetVelocity(m_newtonBody, &saveVeloc[0]);

	dMatrix startMatrix(matrix);
	dPlayerControllerImpulseSolver impulseSolver(this);

	dFloat invTimeStep = 1.0f / timestep;
	bool hasStartMatrix = false;

	for (int j = 0; !hasStartMatrix && (j < 4); j++) {
		hasStartMatrix = true;
		contactSolver.CalculateContacts();
		int count = contactSolver.m_contactCount;
		for (int i = count - 1; i >= 0; i--) {
			NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
			dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat(0.0f));
			dVector localPointpoint(m_localFrame.UntransformVector(startMatrix.UntransformVector(point)));
			if (localPointpoint.m_x < m_stepHeight) {
				count--;
				contactSolver.m_contactBuffer[i] = contactSolver.m_contactBuffer[count];
			}
		}

		if (count) {
			dVector com(zero);
			hasStartMatrix = false;

			SetVelocity(zero[0]);
			impulseSolver.Reset(this);
			NewtonBodyGetCentreOfMass(m_newtonBody, &com[0]);
			com = startMatrix.TransformVector(com);
			for (int i = 0; i < count; i++) {
				NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
				dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat(0.0f));
				dVector normal(contact.m_normal[0], contact.m_normal[1], contact.m_normal[2], dFloat(0.0f));
				dFloat speed = dMin((contact.m_penetration + D_MAX_COLLISION_PENETRATION), dFloat(0.25f)) * invTimeStep;
				impulseSolver.AddLinearRow(normal, point - com, speed, 0.0f, 1.0e12f);
			}

			impulseSolver.AddAngularRows();
			dVector veloc(impulseSolver.CalculateImpulse().Scale(m_invMass));
			SetVelocity(veloc);
			NewtonBodyIntegrateVelocity(m_newtonBody, timestep);
			NewtonBodyGetMatrix(m_newtonBody, &startMatrix[0][0]);
		}
	}

	//dAssert (hasStartMatrix);
	if (hasStartMatrix) {
		// clip player velocity along the high contacts

		dMatrix coodinateMatrix(m_localFrame * startMatrix);

		dFloat scaleSpeedFactor = 1.5f;
		dFloat fowardSpeed = m_forwardSpeed * scaleSpeedFactor;
		dFloat lateralSpeed = m_lateralSpeed * scaleSpeedFactor;
		dFloat maxSpeed = dMax(dAbs(fowardSpeed), dAbs(lateralSpeed));
		dFloat stepFriction = 1.0f + m_mass * maxSpeed;

		SetVelocity(saveVeloc);
		impulseSolver.Reset(this);
		int index = impulseSolver.AddLinearRow(coodinateMatrix[0], impulseSolver.m_zero, 0.0f, 0.0f, 1.0e12f);
		impulseSolver.AddLinearRow(coodinateMatrix[1], impulseSolver.m_zero, -fowardSpeed, -stepFriction, stepFriction, index);
		impulseSolver.AddLinearRow(coodinateMatrix[2], impulseSolver.m_zero, lateralSpeed, -stepFriction, stepFriction, index);
		dVector veloc(saveVeloc + impulseSolver.CalculateImpulse().Scale(m_invMass));

		bool advanceIsBlocked = true;
		for (int j = 0; advanceIsBlocked && (j < 4); j++) {

			advanceIsBlocked = false;
			SetVelocity(veloc);
			NewtonBodyIntegrateVelocity(m_newtonBody, timestep);

			contactSolver.CalculateContacts();
			if (contactSolver.m_contactCount) {
				dMatrix stepMatrix;
				dVector com(zero);

				NewtonBodyGetMatrix(m_newtonBody, &stepMatrix[0][0]);
				int count = contactSolver.m_contactCount;

				// filter by position
				for (int i = count - 1; i >= 0; i--) {
					NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
					dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat(0.0f));
					dVector localPointpoint(m_localFrame.UntransformVector(stepMatrix.UntransformVector(point)));
					if (localPointpoint.m_x < m_stepHeight) {
						count--;
						contactSolver.m_contactBuffer[i] = contactSolver.m_contactBuffer[count];
					}
				}

				if (count) {
					NewtonBodyGetCentreOfMass(m_newtonBody, &com[0]);
					com = stepMatrix.TransformVector(com);
					advanceIsBlocked = true;

					impulseSolver.Reset(this);
					for (int i = 0; i < count; i++) {
						NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
						dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat(0.0f));
						dVector normal(contact.m_normal[0], contact.m_normal[1], contact.m_normal[2], dFloat(0.0f));
						impulseSolver.AddLinearRow(normal, point - com, 0.0f, 0.0f, 1.0e12f);
					}

					impulseSolver.AddAngularRows();
					veloc += impulseSolver.CalculateImpulse().Scale(m_invMass);
					NewtonBodySetMatrix(m_newtonBody, &startMatrix[0][0]);
				}
			}
		}

		SetVelocity(veloc);
		NewtonBodySetMatrix(m_newtonBody, &startMatrix[0][0]);
		NewtonBodyIntegrateVelocity(m_newtonBody, timestep);
		contactSolver.CalculateContacts();
		if (contactSolver.m_contactCount) {
			dMatrix stepMatrix;
			NewtonBodyGetMatrix(m_newtonBody, &stepMatrix[0][0]);

			dFloat maxHigh = 0.0f;
			for (int i = 0; i < contactSolver.m_contactCount; i++) {
				NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];

				dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat(0.0f));
				point = m_localFrame.UntransformVector(stepMatrix.UntransformVector(point));
				if ((point.m_x < m_stepHeight) && (point.m_x > m_contactPatch)) {
					dVector normal(contact.m_normal[0], contact.m_normal[1], contact.m_normal[2], dFloat(0.0f));
					dFloat relSpeed = normal.DotProduct3(veloc);
					if (relSpeed < dFloat(-1.0e-2f)) {
						maxHigh = dMax(point.m_x, maxHigh);
					}
				}
			}

			if (maxHigh > 0.0f) {
				dVector step(stepMatrix.RotateVector(m_localFrame.RotateVector(dVector(maxHigh, dFloat(0.0f), dFloat(0.0f), dFloat(0.0f)))));
				matrix.m_posit += step;
			}
		}
	}

	SetVelocity(saveVeloc);
	NewtonBodySetMatrix(m_newtonBody, &matrix[0][0]);
}

void dPlayerController::ResolveInterpenetrations(dPlayerControllerContactSolver& contactSolver, dPlayerControllerImpulseSolver& impulseSolver)
{
	dVector zero(0.0f);
	dVector savedVeloc(0.0f);
	NewtonBodyGetVelocity(m_newtonBody, &savedVeloc[0]);

	dFloat timestep = 0.1f;
	dFloat invTimestep = 1.0f / timestep;

	dFloat penetration = D_MAX_COLLISION_PENETRATION * 10.0f;
	for (int j = 0; (j < 8) && (penetration > D_MAX_COLLISION_PENETRATION); j++) {
		dMatrix matrix;
		dVector com(0.0f);

		SetVelocity(zero);
		NewtonBodyGetMatrix(m_newtonBody, &matrix[0][0]);
		NewtonBodyGetCentreOfMass(m_newtonBody, &com[0]);
		com = matrix.TransformVector(com);
		com.m_w = 0.0f;

		impulseSolver.Reset(this);
		for (int i = 0; i < contactSolver.m_contactCount; i++) {
			NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];

			dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat(0.0f));
			dVector normal(contact.m_normal[0], contact.m_normal[1], contact.m_normal[2], dFloat(0.0f));
			int index = impulseSolver.AddContactRow(&contact, normal, point - com, 0.0f, 0.0f, 1.0e12f);

			dFloat impulse = invTimestep * dClamp(contact.m_penetration - D_MAX_COLLISION_PENETRATION * 0.5f, dFloat(0.0f), dFloat(0.5f));
			impulseSolver.m_rhs[index] = impulse;
		}
		impulseSolver.AddAngularRows();

		dVector veloc(impulseSolver.CalculateImpulse().Scale(m_invMass));
		SetVelocity(veloc);
		NewtonBodyIntegrateVelocity(m_newtonBody, timestep);

		penetration = 0.0f;
		contactSolver.CalculateContacts();
		for (int i = 0; i < contactSolver.m_contactCount; i++) {
			penetration = dMax(contactSolver.m_contactBuffer[i].m_penetration, penetration);
		}
	}

	SetVelocity(savedVeloc);
}
void dPlayerController::ResolveCollision(dPlayerControllerContactSolver& contactSolver, dFloat timestep)
{
	dMatrix matrix;
	NewtonBodyGetMatrix(m_newtonBody, &matrix[0][0]);

	contactSolver.CalculateContacts();
	if (!contactSolver.m_contactCount) {
		return;
	}

	dFloat maxPenetration = 0.0f;
	for (int i = 0; i < contactSolver.m_contactCount; i++) {
		maxPenetration = dMax(contactSolver.m_contactBuffer[i].m_penetration, maxPenetration);
	}

	dPlayerControllerImpulseSolver impulseSolver(this);
	if (maxPenetration > D_MAX_COLLISION_PENETRATION) {
		ResolveInterpenetrations(contactSolver, impulseSolver);
		NewtonBodyGetMatrix(m_newtonBody, &matrix[0][0]);
	}

	dVector zero(0.0f);
	dVector com(0.0f);
	dVector veloc(0.0f);

	NewtonBodyGetVelocity(m_newtonBody, &veloc[0]);
	NewtonBodyGetCentreOfMass(m_newtonBody, &com[0]);

	const dMatrix frameMatrix(m_localFrame * matrix);
	com = matrix.TransformVector(com);

	impulseSolver.Reset(this);
	dVector surfaceVeloc(0.0f);
	const dFloat contactPatchHigh = m_contactPatch * dFloat(0.995f);
	for (int i = 0; i < contactSolver.m_contactCount; i++) {
		NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
		dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat(0.0f));
		dVector normal(contact.m_normal[0], contact.m_normal[1], contact.m_normal[2], dFloat(0.0f));
		const int normalIndex = impulseSolver.AddContactRow(&contact, normal, point - com, 0.0f, 0.0f, 1.0e12f);
		dVector localPoint(frameMatrix.UntransformVector(point));

		if (localPoint.m_x < contactPatchHigh) {
			if (impulseSolver.m_contactPoint[normalIndex]) {
				impulseSolver.m_jacobianPairs[normalIndex].m_jacobian_J10.m_linear = impulseSolver.m_zero;
				impulseSolver.m_jacobianPairs[normalIndex].m_jacobian_J10.m_angular = impulseSolver.m_zero;
			}

			dFloat friction = ContactFrictionCallback(point, normal, int(contact.m_contactID), contact.m_hitBody);
			if (friction > 0.0f) {
				// add lateral traction friction
				dVector sideDir(frameMatrix.m_up.CrossProduct(normal).Normalize());
				impulseSolver.AddContactRow(&contact, sideDir, point - com, -m_lateralSpeed, -friction, friction, normalIndex);

				// add longitudinal  traction friction
				dVector frontDir(normal.CrossProduct(sideDir));
				impulseSolver.AddContactRow(&contact, frontDir, point - com, -m_forwardSpeed, -friction, friction, normalIndex);
			}
		}
	}

	impulseSolver.AddAngularRows();

	veloc += impulseSolver.CalculateImpulse().Scale(m_invMass);
	impulseSolver.ApplyReaction(timestep);

	SetVelocity(veloc);
}

dPlayerController::dCollisionState dPlayerController::TestPredictCollision(const dPlayerControllerContactSolver& contactSolver, const dVector& veloc) const
{
	for (int i = 0; i < contactSolver.m_contactCount; i++) {
		const NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
		if (contact.m_penetration >= D_MAX_COLLISION_PENETRATION) {
			return m_deepPenetration;
		}
	}

	for (int i = 0; i < contactSolver.m_contactCount; i++) {
		const NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
		dFloat projecSpeed = veloc.DotProduct3(contact.m_normal);
		if (projecSpeed < dFloat(0.0f)) {
			return m_colliding;
		}
	}
	return m_freeMovement;
}

dFloat dPlayerController::PredictTimestep(dFloat timestep, dPlayerControllerContactSolver& contactSolver)
{
	dMatrix matrix;
	dVector veloc(0.0f);

	NewtonBodyGetVelocity(m_newtonBody, &veloc[0]);
	NewtonBodyGetMatrix(m_newtonBody, &matrix[0][0]);

	NewtonBodyIntegrateVelocity(m_newtonBody, timestep);
	dCollisionState playerCollide = TestPredictCollision(contactSolver, veloc);
	NewtonBodySetMatrix(m_newtonBody, &matrix[0][0]);
	if (playerCollide == m_deepPenetration) {
		dFloat savedTimeStep = timestep;
		timestep *= 0.5f;
		dFloat dt = timestep;
		for (int i = 0; i < D_MAX_COLLIONSION_STEPS; i++) {
			NewtonBodyIntegrateVelocity(m_newtonBody, timestep);
			contactSolver.CalculateContacts();
			NewtonBodySetMatrix(m_newtonBody, &matrix[0][0]);

			dt *= 0.5f;
			playerCollide = TestPredictCollision(contactSolver, veloc);
			if (playerCollide == m_colliding) {
				return timestep;
			}
			if (playerCollide == m_deepPenetration) {
				timestep -= dt;
			}
			else {
				timestep += dt;
			}
		}
		if (timestep > dt * 2.0f) {
			return timestep;
		}

		dt = savedTimeStep / D_MAX_COLLIONSION_STEPS;
		timestep = dt;
		for (int i = 1; i < D_MAX_COLLIONSION_STEPS; i++) {
			NewtonBodyIntegrateVelocity(m_newtonBody, timestep);
			contactSolver.CalculateContacts();
			NewtonBodySetMatrix(m_newtonBody, &matrix[0][0]);
			playerCollide = TestPredictCollision(contactSolver, veloc);
			if (playerCollide != m_freeMovement) {
				return timestep;
			}
			timestep += dt;
		}
		dAssert(0);
	}

	return timestep;
}

const void dPlayerController::Debug(dCustomJoint::dDebugDisplay* const debugContext) const
{
}

void dPlayerController::UpdatePlayerStatus(dPlayerControllerContactSolver& contactSolver)
{
	dMatrix matrix;
	NewtonBodyGetMatrix(m_newtonBody, &matrix[0][0]);

	m_isAirbone = true;
	m_isOnFloor = false;
	matrix = m_localFrame * matrix;
	contactSolver.CalculateContacts();
	for (int i = 0; i < contactSolver.m_contactCount; i++) {
		m_isAirbone = false;
		NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
		dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat(0.0f));
		dVector localPoint(matrix.UntransformVector(point));
		if (localPoint.m_x < m_contactPatch) {
			dVector normal(contact.m_normal[0], contact.m_normal[1], contact.m_normal[2], dFloat(0.0f));
			dVector localNormal(matrix.UnrotateVector(normal));
			if (localNormal.m_x > 0.95f) {
				m_isOnFloor = true;
			}
		}
	}
}

void dPlayerController::PreUpdate(dFloat timestep)
{
	dPlayerControllerContactSolver contactSolver(this);

	dFloat timeLeft = timestep;
	const dFloat timeEpsilon = timestep * (1.0f / 16.0f);

	m_impulse = dVector(0.0f);
	m_manager->ApplyInputs(this, timestep);

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
	NewtonBodyGetPosition(m_newtonBody, &matrix.m_posit[0]);
	NewtonBodySetMatrix(m_newtonBody, &matrix[0][0]);

dTrace(("player matrix: heading %f front: (%f %f)\n", GetHeadingAngle() * dRadToDegree, matrix[0][0], matrix[0][2]));

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
		NewtonBodyIntegrateVelocity(m_newtonBody, predicetdTime);
		timeLeft -= predicetdTime;
	}

	UpdatePlayerStatus(contactSolver);
}

