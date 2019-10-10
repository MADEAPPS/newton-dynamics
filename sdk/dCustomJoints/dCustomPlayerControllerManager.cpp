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
#include "dCustomJointLibraryStdAfx.h"
#include "dCustomJoint.h"
#include "dCustomPlayerControllerManager.h"

#define D_DESCRETE_MOTION_STEPS		4
#define D_MAX_COLLIONSION_STEPS		8
#define D_MAX_CONTACTS				6
#define D_MAX_ROWS					(3 * D_MAX_CONTACTS) 
#define D_MAX_COLLISION_PENETRATION	dFloat (5.0e-3f)

class dCustomPlayerController::dContactSolver
{
	public: 
	dContactSolver(dCustomPlayerController* const controller)
		:m_controller(controller)
		,m_contactCount(0)
	{
	}

	void CalculateContacts()
	{
		dMatrix matrix;
		NewtonWorld* const world = m_controller->m_manager->GetWorld();
		NewtonCollision* const shape = NewtonBodyGetCollision(m_controller->m_kinematicBody);

		NewtonBodyGetMatrix(m_controller->m_kinematicBody, &matrix[0][0]);
		m_contactCount = NewtonWorldCollide(world, &matrix[0][0], shape, m_controller, PrefilterCallback, m_contactBuffer, D_MAX_CONTACTS, 0);
	}

	static unsigned PrefilterCallback(const NewtonBody* const body, const NewtonCollision* const collision, void* const userData)
	{
		dCustomPlayerController* const controller = (dCustomPlayerController*)userData;
		if (controller->GetBody() == body) {
			return false;
		}
		return 1;
	}

	NewtonWorldConvexCastReturnInfo m_contactBuffer[D_MAX_ROWS];
	dCustomPlayerController* m_controller;
	int m_contactCount;
};

class dCustomPlayerController::dImpulseSolver 
{
	public: 
	dImpulseSolver (dCustomPlayerController* const controller)
		:m_zero(0.0f)
	{
		m_mass = controller->m_mass;
		m_invMass = controller->m_invMass;
		NewtonBodyGetInvInertiaMatrix(controller->m_kinematicBody, &m_invInertia[0][0]);
		Reset(controller);
	}

	void Reset (dCustomPlayerController* const controller)
	{
		m_rowCount = 0;
		NewtonBodyGetVelocity(controller->m_kinematicBody, &m_veloc[0]);
	}

	void AddAngularRows ()
	{
		for (int i = 0; i < 3; i ++) {
			m_contactPoint[m_rowCount] = NULL;
			m_jacobianPairs[m_rowCount].m_jacobian_J10.m_linear = m_zero;
			m_jacobianPairs[m_rowCount].m_jacobian_J10.m_angular = m_zero;
			m_jacobianPairs[m_rowCount].m_jacobian_J01.m_linear = m_zero;
			m_jacobianPairs[m_rowCount].m_jacobian_J01.m_angular = m_zero;
			m_jacobianPairs[m_rowCount].m_jacobian_J01.m_angular[i] = dFloat(1.0f);
			m_rhs[m_rowCount] = 0.0f;
			m_low[m_rowCount] = -1.0e12f;
			m_high[m_rowCount] = 1.0e12f;
			m_impulseMag[m_rowCount] = 0.0f;
			m_normalIndex[m_rowCount] = 0;
			m_rowCount++;
			dAssert(m_rowCount < D_MAX_ROWS);
		}
	}

	int AddLinearRow (const dVector& dir, const dVector& r, dFloat speed, dFloat low, dFloat high, int normalIndex = -1)
	{
		m_contactPoint[m_rowCount] = NULL;
		m_jacobianPairs[m_rowCount].m_jacobian_J01.m_linear = dir;
		m_jacobianPairs[m_rowCount].m_jacobian_J01.m_angular = r.CrossProduct(dir);
		m_jacobianPairs[m_rowCount].m_jacobian_J10.m_linear = m_zero;
		m_jacobianPairs[m_rowCount].m_jacobian_J10.m_angular = m_zero;

		m_low[m_rowCount] = low;
		m_high[m_rowCount] = high;
		m_normalIndex[m_rowCount] = (normalIndex == -1) ? 0 : normalIndex - m_rowCount;
		m_rhs[m_rowCount] = speed - m_veloc.DotProduct3(m_jacobianPairs[m_rowCount].m_jacobian_J01.m_linear);
		m_rowCount++;
		dAssert(m_rowCount < D_MAX_ROWS);
		return m_rowCount - 1;
	}

	int AddContactRow (const NewtonWorldConvexCastReturnInfo* const contact, const dVector& dir, const dVector& r, dFloat speed, dFloat low, dFloat high, int normalIndex = -1)
	{
		dFloat invIxx;
		dFloat invIyy;
		dFloat invIzz;
		dFloat invMass;
		dAssert (contact->m_hitBody);
		NewtonBodyGetInvMass (contact->m_hitBody, &invMass, &invIxx, &invIyy, &invIzz);
		if (invMass == 0.0f) {
			return AddLinearRow(dir, r, speed, low, high, normalIndex);
		}

		dMatrix matrix;
		dVector com;
		dVector veloc;
		dVector omega;

		NewtonBodyGetOmega(contact->m_hitBody, &omega[0]);
		NewtonBodyGetVelocity(contact->m_hitBody, &veloc[0]);
		NewtonBodyGetMatrix(contact->m_hitBody, &matrix[0][0]);
		NewtonBodyGetCentreOfMass(contact->m_hitBody, &com[0]);
		com = matrix.TransformVector(com);

		dVector p1(contact->m_point[0], contact->m_point[1], contact->m_point[2], dFloat(0.0f));
		dVector r1 (p1 - com);
		dVector dir1 (dir.Scale (-1.0f));

		m_contactPoint[m_rowCount] = contact;
		m_jacobianPairs[m_rowCount].m_jacobian_J01.m_linear = dir;
		m_jacobianPairs[m_rowCount].m_jacobian_J01.m_angular = r.CrossProduct(dir);
		m_jacobianPairs[m_rowCount].m_jacobian_J10.m_linear = dir1;
		m_jacobianPairs[m_rowCount].m_jacobian_J10.m_angular = r1.CrossProduct(dir1);

		m_low[m_rowCount] = low;
		m_high[m_rowCount] = high;
		m_normalIndex[m_rowCount] = (normalIndex == -1) ? 0 : normalIndex - m_rowCount;

		dVector s (m_veloc * m_jacobianPairs[m_rowCount].m_jacobian_J01.m_linear + 
				   veloc * m_jacobianPairs[m_rowCount].m_jacobian_J10.m_linear + 
				   omega * m_jacobianPairs[m_rowCount].m_jacobian_J10.m_angular);
		m_rhs[m_rowCount] = speed - s.m_x - s.m_y - s.m_z;
									
		m_rowCount++;
		dAssert(m_rowCount < D_MAX_ROWS);
		return m_rowCount - 1;
	}

	dVector CalculateImpulse()
	{
		dFloat massMatrix[D_MAX_ROWS][D_MAX_ROWS];
		const NewtonBody* bodyArray[D_MAX_ROWS];
		for (int i = 0; i < m_rowCount; i++) {
			bodyArray[i] = m_contactPoint[i] ? m_contactPoint[i]->m_hitBody : NULL; 
		}

		for (int i = 0; i < m_rowCount; i++) {
			dComplementaritySolver::dJacobianPair jInvMass(m_jacobianPairs[i]);

			jInvMass.m_jacobian_J01.m_linear = jInvMass.m_jacobian_J01.m_linear.Scale(m_invMass);
			jInvMass.m_jacobian_J01.m_angular = m_invInertia.RotateVector(jInvMass.m_jacobian_J01.m_angular);
			if (bodyArray[i]) {
				dMatrix invInertia;
				dFloat invIxx;
				dFloat invIyy;
				dFloat invIzz;
				dFloat invMass;

				NewtonBodyGetInvMass(bodyArray[i], &invMass, &invIxx, &invIyy, &invIzz);
				NewtonBodyGetInvInertiaMatrix(bodyArray[i], &invInertia[0][0]);
				jInvMass.m_jacobian_J10.m_linear = jInvMass.m_jacobian_J10.m_linear.Scale (invMass);
				jInvMass.m_jacobian_J10.m_angular = invInertia.RotateVector(jInvMass.m_jacobian_J10.m_angular);

			} else {
				jInvMass.m_jacobian_J10.m_linear = m_zero;
				jInvMass.m_jacobian_J10.m_angular = m_zero;
			}

			dVector tmp(jInvMass.m_jacobian_J01.m_linear * m_jacobianPairs[i].m_jacobian_J01.m_linear + 
						jInvMass.m_jacobian_J01.m_angular * m_jacobianPairs[i].m_jacobian_J01.m_angular +
						jInvMass.m_jacobian_J10.m_linear * m_jacobianPairs[i].m_jacobian_J10.m_linear + 
						jInvMass.m_jacobian_J10.m_angular * m_jacobianPairs[i].m_jacobian_J10.m_angular);
			dFloat a00 = (tmp.m_x + tmp.m_y + tmp.m_z) * 1.0004f;

			massMatrix[i][i] = a00;

			m_impulseMag[i] = 0.0f;
			for (int j = i + 1; j < m_rowCount; j++) {
				dVector tmp1(jInvMass.m_jacobian_J01.m_linear * m_jacobianPairs[j].m_jacobian_J01.m_linear +
							 jInvMass.m_jacobian_J01.m_angular * m_jacobianPairs[j].m_jacobian_J01.m_angular);
				if (bodyArray[i] == bodyArray[j]) {
					tmp1 += jInvMass.m_jacobian_J10.m_linear * m_jacobianPairs[j].m_jacobian_J10.m_linear;
					tmp1 += jInvMass.m_jacobian_J10.m_angular * m_jacobianPairs[j].m_jacobian_J10.m_angular;
				}

				dFloat a01 = tmp1.m_x + tmp1.m_y + tmp1.m_z;
				massMatrix[i][j] = a01;
				massMatrix[j][i] = a01;
			}
		}

		dAssert (dTestPSDmatrix(m_rowCount, D_MAX_ROWS, &massMatrix[0][0]));
		dGaussSeidelLcpSor(m_rowCount, D_MAX_ROWS, &massMatrix[0][0], m_impulseMag, m_rhs, m_normalIndex, m_low, m_high, dFloat(1.0e-6f), 32, dFloat(1.1f));

		dVector netImpulse(0.0f);
		for (int i = 0; i < m_rowCount; i++) {
			netImpulse += m_jacobianPairs[i].m_jacobian_J01.m_linear.Scale(m_impulseMag[i]);
		}
		return netImpulse;
	}

	void ApplyReaction(dFloat timestep)
	{
		dFloat invTimeStep = 0.1f / timestep;
		for (int i = 0; i < m_rowCount; i++) {
			if (m_contactPoint[i]) {
				dVector force (m_jacobianPairs[i].m_jacobian_J10.m_linear.Scale (m_impulseMag[i] * invTimeStep));
				dVector torque (m_jacobianPairs[i].m_jacobian_J10.m_angular.Scale (m_impulseMag[i] * invTimeStep));
				NewtonBodyAddForce(m_contactPoint[i]->m_hitBody, &force[0]);
				NewtonBodyAddTorque(m_contactPoint[i]->m_hitBody, &torque[0]);
			}
		}
	}
	
	dMatrix m_invInertia;
	dVector m_veloc;
	dVector m_zero;
	dComplementaritySolver::dJacobianPair m_jacobianPairs[D_MAX_ROWS];
	const NewtonWorldConvexCastReturnInfo* m_contactPoint[D_MAX_ROWS];
	dFloat m_rhs[D_MAX_ROWS];
	dFloat m_low[D_MAX_ROWS];
	dFloat m_high[D_MAX_ROWS];
	dFloat m_impulseMag[D_MAX_ROWS];
	int m_normalIndex[D_MAX_ROWS];
	dFloat m_mass;	
	dFloat m_invMass;	
	int m_rowCount;
};

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
		do {
			dCustomPlayerController* const controller = &node->GetInfo();
			controller->PreUpdate(timestep);
			for (int i = 0; i < threadCount; i++) {
				node = node ? node->GetNext() : NULL;
			}
		} while (node);
	}
}


dCustomPlayerController::dCustomPlayerController()
	:m_localFrame(dGetIdentityMatrix())
	,m_impulse(0.0f)
	,m_mass(0.0f)
	,m_invMass(0.0f)
	,m_headingAngle(0.0f)
	,m_forwardSpeed(0.0f)
	,m_lateralSpeed(0.0f)
	,m_stepHeight(0.0f)
	,m_contactPatch(0.0f)
	,m_userData(NULL)
	,m_kinematicBody(NULL)
	,m_manager(NULL)
	,m_isAirbone(false)
	,m_isOnFloor(false)
{
}

dCustomPlayerController::~dCustomPlayerController()
{
	NewtonDestroyBody(m_kinematicBody);
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

	shapeMatrix.m_posit = dVector (0.0f, dFloat (0.0f), dFloat (0.0f), 1.0f);
	controller.m_localFrame = shapeMatrix;
	controller.m_mass = mass;
	controller.m_invMass = 1.0f / mass;
	controller.m_manager = this;
	controller.m_kinematicBody = body;
	controller.m_contactPatch = radius / scale;
	controller.m_stepHeight = dMax (stepHeight, controller.m_contactPatch * 2.0f);

	return &controller;
}

void dCustomPlayerControllerManager::DestroyController(dCustomPlayerController* const player)
{
	dList<dCustomPlayerController>::dListNode* const node = m_playerList.GetNodeFromInfo(*player);
	dAssert(node);
	m_playerList.Remove(node);
}

dVector dCustomPlayerController::GetVelocity() const
{ 
	dVector veloc(0.0);
	NewtonBodyGetVelocity(m_kinematicBody, &veloc[0]);
	return veloc; 
}

void dCustomPlayerController::SetVelocity(const dVector& veloc) 
{ 
	dAssert(veloc.DotProduct3(veloc) < 10000.0f);
	NewtonBodySetVelocity(m_kinematicBody, &veloc[0]);
}

void dCustomPlayerController::SetFrame(const dMatrix& frame)
{
	dAssert (frame.TestOrthogonal());
	m_localFrame = frame;
	m_localFrame.m_posit = dVector (0.0f, dFloat (0.0f), dFloat (0.0f), 1.0f);

	NewtonCollision* const capsule = NewtonBodyGetCollision(m_kinematicBody);

	dMatrix oldMatrix;
	dMatrix newMatrix(m_localFrame);
	NewtonCollisionGetMatrix(capsule, &oldMatrix[0][0]);

	newMatrix.m_posit = oldMatrix.m_posit;
	NewtonCollisionSetMatrix(capsule, &newMatrix[0][0]);
}


//#define USE_OLD_STEP_ALGORITHM

#ifdef USE_OLD_STEP_ALGORITHM
void dCustomPlayerController::ResolveStep(dFloat timestep, dContactSolver& contactSolver)
{
	dMatrix matrix;
	dVector saveVeloc(0.0f);

	NewtonBodyGetMatrix(m_kinematicBody, &matrix[0][0]);
	NewtonBodyGetVelocity(m_kinematicBody, &saveVeloc[0]);

	dMatrix coodinateMatrix (m_localFrame * matrix);

	// clip player velocity along the high contacts
	bool applyStep = false; 
	dImpulseSolver impulseSolver(this);

	dFloat scaleSpeedFactor = 2.0f;
	dFloat fowardSpeed = m_forwardSpeed * scaleSpeedFactor;
	dFloat lateralSpeed = m_lateralSpeed * scaleSpeedFactor;
	dFloat maxSpeed = dMax(dAbs(fowardSpeed), dAbs(lateralSpeed));
	dFloat stepFriction = 1.0f + m_mass * maxSpeed;
	
	int index = impulseSolver.AddLinearRow(coodinateMatrix[0], impulseSolver.m_zero, 0.0f, 0.0f, 1.0e12f);
	impulseSolver.AddLinearRow(coodinateMatrix[1], impulseSolver.m_zero, -m_forwardSpeed, -stepFriction, stepFriction, index);
	impulseSolver.AddLinearRow(coodinateMatrix[2], impulseSolver.m_zero, m_lateralSpeed, -stepFriction, stepFriction, index);
	dVector veloc (saveVeloc + impulseSolver.CalculateImpulse().Scale(m_invMass));

	for (int j = 0; !applyStep && (j < 4); j ++) {
		SetVelocity(veloc);
		NewtonBodySetMatrix(m_kinematicBody, &matrix[0][0]);
		NewtonBodyIntegrateVelocity(m_kinematicBody, timestep);

		applyStep = true;
		contactSolver.CalculateContacts();
		if (contactSolver.m_contactCount) {
			dMatrix stepMatrix;
			NewtonBodyGetMatrix(m_kinematicBody, &stepMatrix[0][0]);
			int highContactCount = contactSolver.m_contactCount;
			for (int i = contactSolver.m_contactCount - 1; i >= 0; i--) {
				NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
				dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat(0.0f));
				dVector localPointpoint(m_localFrame.UntransformVector(stepMatrix.UntransformVector(point)));
				if (localPointpoint.m_x <= m_stepHeight) {
					highContactCount --;
					contactSolver.m_contactBuffer[i] = contactSolver.m_contactBuffer[highContactCount];
				}
			}
			if (highContactCount) {
				dVector com;
				NewtonBodyGetCentreOfMass(m_kinematicBody, &com[0]);
				com = stepMatrix.TransformVector(com);

				impulseSolver.Reset(this);
				for (int i = 0; i < highContactCount; i++) {
					NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
					dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat(0.0f));
					dVector normal(contact.m_normal[0], contact.m_normal[1], contact.m_normal[2], dFloat(0.0f));
					impulseSolver.AddContactRow(&contact, normal, point - com, 0.0f, 0.0f, 1.0e12f);
				}

				applyStep = false;
				impulseSolver.AddAngularRows();
				veloc += impulseSolver.CalculateImpulse().Scale(m_invMass);
			}
		}
	}

	if (applyStep) {
		contactSolver.CalculateContacts();
		if (contactSolver.m_contactCount) {
			dMatrix stepMatrix;
			NewtonBodyGetMatrix(m_kinematicBody, &stepMatrix[0][0]);

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

	NewtonBodySetMatrix(m_kinematicBody, &matrix[0][0]);
	SetVelocity(saveVeloc);
}

#else

void dCustomPlayerController::ResolveStep(dFloat timestep, dContactSolver& contactSolver)
{
	dMatrix matrix;
	dVector zero(0.0f);
	dVector saveVeloc(0.0f);

//static int xxx;
//xxx++;

	NewtonBodyGetMatrix(m_kinematicBody, &matrix[0][0]);
	NewtonBodyGetVelocity(m_kinematicBody, &saveVeloc[0]);

	dMatrix startMatrix(matrix);
	dImpulseSolver impulseSolver(this);

	dFloat invTimeStep = 1.0f / timestep;
	bool hasStartMatrix = false;

	for (int i = 0; !hasStartMatrix && (i < 4); i ++) {
		hasStartMatrix = true;
		contactSolver.CalculateContacts();
		int count = contactSolver.m_contactCount;
		for (int i = count - 1; i >= 0; i--) {
			NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
			dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat(0.0f));
			dVector localPointpoint(m_localFrame.UntransformVector(startMatrix.UntransformVector(point)));
			if (localPointpoint.m_x < m_stepHeight) {
				count --;
				contactSolver.m_contactBuffer[i] = contactSolver.m_contactBuffer[count];
			}
		}

		if (count) {
			dVector com(zero);
			hasStartMatrix = false;
		
			SetVelocity(zero[0]);
			impulseSolver.Reset(this);
			NewtonBodyGetCentreOfMass(m_kinematicBody, &com[0]);
			com = startMatrix.TransformVector(com);
			for (int i = 0; i < count; i++) {
				NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
				dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat(0.0f));
				dVector normal(contact.m_normal[0], contact.m_normal[1], contact.m_normal[2], dFloat(0.0f));
				dFloat speed = (contact.m_penetration + D_MAX_COLLISION_PENETRATION) * invTimeStep;
				impulseSolver.AddLinearRow(normal, point - com, speed, 0.0f, 1.0e12f);
			}

			impulseSolver.AddAngularRows();
			dVector veloc (impulseSolver.CalculateImpulse().Scale(m_invMass));
			SetVelocity(veloc);
			NewtonBodyIntegrateVelocity(m_kinematicBody, timestep);
			NewtonBodyGetMatrix(m_kinematicBody, &startMatrix[0][0]);
		}
	}

	dAssert (hasStartMatrix);
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
			NewtonBodyIntegrateVelocity(m_kinematicBody, timestep);

			contactSolver.CalculateContacts();
			if (contactSolver.m_contactCount) {
				dMatrix stepMatrix;
				dVector com(zero);

				NewtonBodyGetMatrix(m_kinematicBody, &stepMatrix[0][0]);
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
					NewtonBodyGetCentreOfMass(m_kinematicBody, &com[0]);
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
					NewtonBodySetMatrix(m_kinematicBody, &startMatrix[0][0]);
				}
			}
		}
	
		SetVelocity(veloc);
		NewtonBodySetMatrix(m_kinematicBody, &startMatrix[0][0]);
		NewtonBodyIntegrateVelocity(m_kinematicBody, timestep);
		contactSolver.CalculateContacts();
		if (contactSolver.m_contactCount) {
			dMatrix stepMatrix;
			NewtonBodyGetMatrix(m_kinematicBody, &stepMatrix[0][0]);

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
	NewtonBodySetMatrix(m_kinematicBody, &matrix[0][0]);
}
#endif

dCustomPlayerController::dCollisionState dCustomPlayerController::TestPredictCollision(const dContactSolver& contactSolver, const dVector& veloc) const
{
	for (int i = 0; i < contactSolver.m_contactCount; i++) {
		const NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
		if (contact.m_penetration >= D_MAX_COLLISION_PENETRATION) {
			return m_deepPenetration;
		}
	}

	for (int i = 0; i < contactSolver.m_contactCount; i ++) {
		const NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
		dFloat projecSpeed = veloc.DotProduct3(contact.m_normal);
		if (projecSpeed < dFloat (0.0f)) {
			return m_colliding;
		}
	}
	return m_freeMovement;
}

dFloat dCustomPlayerController::PredictTimestep(dFloat timestep, dContactSolver& contactSolver)
{
	dMatrix matrix;
	dVector veloc(0.0f);
	
	NewtonBodyGetVelocity(m_kinematicBody, &veloc[0]);
	NewtonBodyGetMatrix(m_kinematicBody, &matrix[0][0]);

	NewtonBodyIntegrateVelocity(m_kinematicBody, timestep);
	dCollisionState playerCollide = TestPredictCollision(contactSolver, veloc);
	NewtonBodySetMatrix(m_kinematicBody, &matrix[0][0]);
	if (playerCollide == m_deepPenetration) {
		dFloat savedTimeStep = timestep;
		timestep *= 0.5f;
		dFloat dt = timestep;
		for (int i = 0; i < D_MAX_COLLIONSION_STEPS; i++) {
			NewtonBodyIntegrateVelocity(m_kinematicBody, timestep);
			contactSolver.CalculateContacts();
			NewtonBodySetMatrix(m_kinematicBody, &matrix[0][0]);

			dt *= 0.5f;
			playerCollide = TestPredictCollision(contactSolver, veloc);
			if (playerCollide == m_colliding) {
				return timestep;
			} 
			if (playerCollide == m_deepPenetration) {
				timestep -= dt;
			} else {
				timestep += dt;
			}
		}
		if (timestep > dt * 2.0f) {
			return timestep;
		}

		dt = savedTimeStep / D_MAX_COLLIONSION_STEPS;
		timestep = dt;
		for (int i = 1; i < D_MAX_COLLIONSION_STEPS; i++) {
			NewtonBodyIntegrateVelocity(m_kinematicBody, timestep);
			contactSolver.CalculateContacts();
			NewtonBodySetMatrix(m_kinematicBody, &matrix[0][0]);
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

void dCustomPlayerController::ResolveInterpenetrations(dContactSolver& contactSolver, dImpulseSolver& impulseSolver)
{
	dVector zero (0.0f);
	dVector savedVeloc (0.0f);
	NewtonBodyGetVelocity(m_kinematicBody, &savedVeloc[0]);

	dFloat timestep = 0.1f;
	dFloat invTimestep = 1.0f / timestep;

	dFloat penetration = D_MAX_COLLISION_PENETRATION * 10.0f;
	for (int j = 0; (j < 8) && (penetration > D_MAX_COLLISION_PENETRATION) ; j ++) {
		dMatrix matrix;
		dVector com(0.0f);

		SetVelocity(zero);
		NewtonBodyGetMatrix(m_kinematicBody, &matrix[0][0]);
		NewtonBodyGetCentreOfMass(m_kinematicBody, &com[0]);
		com = matrix.TransformVector(com);
		com.m_w = 0.0f;

		impulseSolver.Reset(this);
		for (int i = 0; i < contactSolver.m_contactCount; i++) {
			NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];

			dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat (0.0f));
			dVector normal(contact.m_normal[0], contact.m_normal[1], contact.m_normal[2], dFloat (0.0f));
			int index = impulseSolver.AddContactRow(&contact, normal, point - com, 0.0f, 0.0f, 1.0e12f);

			dFloat impulse = invTimestep * dClamp(contact.m_penetration - D_MAX_COLLISION_PENETRATION * 0.5f, dFloat(0.0f), dFloat(0.5f));
			impulseSolver.m_rhs[index] = impulse;
		}
		impulseSolver.AddAngularRows();

		dVector veloc (impulseSolver.CalculateImpulse().Scale (m_invMass));
		SetVelocity(veloc);
		NewtonBodyIntegrateVelocity(m_kinematicBody, timestep);

		penetration = 0.0f;
		contactSolver.CalculateContacts();
		for (int i = 0; i < contactSolver.m_contactCount; i++) {
			penetration = dMax(contactSolver.m_contactBuffer[i].m_penetration, penetration);
		}
	}

	SetVelocity(savedVeloc);
}

void dCustomPlayerController::ResolveCollision(dContactSolver& contactSolver, dFloat timestep)
{
	dMatrix matrix;
	NewtonBodyGetMatrix(m_kinematicBody, &matrix[0][0]);

	contactSolver.CalculateContacts();
	if (!contactSolver.m_contactCount) {
		return;
	}

	dFloat maxPenetration = 0.0f;
	for (int i = 0; i < contactSolver.m_contactCount; i ++) {
		maxPenetration = dMax (contactSolver.m_contactBuffer[i].m_penetration, maxPenetration);
	}

	dImpulseSolver impulseSolver(this);
	if (maxPenetration > D_MAX_COLLISION_PENETRATION) {
		ResolveInterpenetrations(contactSolver, impulseSolver);
		NewtonBodyGetMatrix(m_kinematicBody, &matrix[0][0]);
	}

	dVector zero(0.0f);
	dVector com(0.0f);
	dVector veloc(0.0f);
	
	NewtonBodyGetVelocity(m_kinematicBody, &veloc[0]);
	NewtonBodyGetCentreOfMass(m_kinematicBody, &com[0]);

	const dMatrix frameMatrix (m_localFrame * matrix);
	com = matrix.TransformVector(com);

	impulseSolver.Reset(this);
	dVector surfaceVeloc(0.0f);
	const dFloat contactPatchHigh = m_contactPatch * dFloat (0.995f);
	for (int i = 0; i < contactSolver.m_contactCount; i ++) {
		NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
		dVector point (contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat (0.0f));
		dVector normal (contact.m_normal[0], contact.m_normal[1], contact.m_normal[2], dFloat (0.0f));
		const int normalIndex = impulseSolver.AddContactRow(&contact, normal, point - com, 0.0f, 0.0f, 1.0e12f);
		dVector localPoint (frameMatrix.UntransformVector(point));

		if (localPoint.m_x < contactPatchHigh) {
			if (impulseSolver.m_contactPoint[normalIndex]) {
				impulseSolver.m_jacobianPairs[normalIndex].m_jacobian_J10.m_linear = impulseSolver.m_zero;
				impulseSolver.m_jacobianPairs[normalIndex].m_jacobian_J10.m_angular = impulseSolver.m_zero;
			}

			dFloat friction = m_manager->ContactFriction(this, point, normal, int (contact.m_contactID), contact.m_hitBody);
			if (friction > 0.0f) {
				// add lateral traction friction
				dVector sideDir (frameMatrix.m_up.CrossProduct(normal).Normalize());
				impulseSolver.AddContactRow(&contact, sideDir, point - com, -m_lateralSpeed, -friction, friction, normalIndex);

				// add longitudinal  traction friction
				dVector frontDir (normal.CrossProduct(sideDir));
				impulseSolver.AddContactRow(&contact, frontDir, point - com, -m_forwardSpeed, -friction, friction, normalIndex);
			}
		}
	}

	impulseSolver.AddAngularRows();

	veloc += impulseSolver.CalculateImpulse().Scale(m_invMass);
	impulseSolver.ApplyReaction(timestep);

	SetVelocity(veloc);
}


void dCustomPlayerController::UpdatePlayerStatus(dContactSolver& contactSolver)
{
	dMatrix matrix;
	NewtonBodyGetMatrix(m_kinematicBody, &matrix[0][0]);

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
			if (localNormal.m_x > 0.99f) {
				m_isOnFloor = true;
			}
		}
	}
}

void dCustomPlayerController::PreUpdate(dFloat timestep)
{
	dContactSolver contactSolver (this);

	dFloat timeLeft = timestep;
	const dFloat timeEpsilon = timestep * (1.0f / 16.0f);

	m_impulse = dVector(0.0f);
	m_manager->ApplyMove(this, timestep);

#if 0
	#if 1
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
}
