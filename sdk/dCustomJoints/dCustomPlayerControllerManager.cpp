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
#define D_MAX_CONTACTS				6
#define D_MAX_ROWS					(3 * D_MAX_CONTACTS) 
#define D_STEP_FRICTION				dFloat (2.0f)
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
	{
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
		dVector zero (0.0f);
		for (int i = 0; i < 3; i ++) {
			m_jacobian[m_rowCount].m_linear = zero;
			m_jacobian[m_rowCount].m_angular = zero;
			m_jacobian[m_rowCount].m_angular[i] = dFloat(1.0f);
			m_rhs[m_rowCount] = 0.0f;
			m_impulseMag[m_rowCount] = 0;
			m_low[m_rowCount] = -1.0e12f;
			m_high[m_rowCount] = 1.0e12f;
			m_normalIndex[m_rowCount] = 0;
			m_rowCount++;
			dAssert(m_rowCount < D_MAX_ROWS);
		}
	}

	int AddLinearRow (const dVector& dir, const dVector& r, dFloat accel, dFloat low, dFloat high, int normalIndex = -1)
	{
		m_jacobian[m_rowCount].m_linear = dir;
		m_jacobian[m_rowCount].m_angular = r.CrossProduct(dir);
		m_low[m_rowCount] = low;
		m_high[m_rowCount] = high;
		m_normalIndex[m_rowCount] = (normalIndex == -1) ? 0 : normalIndex - m_rowCount;
		m_rhs[m_rowCount] = accel - m_veloc.DotProduct3(m_jacobian[m_rowCount].m_linear);
		m_rowCount++;
		dAssert(m_rowCount < D_MAX_ROWS);
		return m_rowCount - 1;
	}

	dVector CalculateImpulse()
	{
		dFloat massMatrix[D_MAX_ROWS][D_MAX_ROWS];
		for (int i = 0; i < m_rowCount; i++) {
			dComplementaritySolver::dJacobian jInvMass(m_jacobian[i]);

			jInvMass.m_linear = jInvMass.m_linear.Scale(m_invMass);
			jInvMass.m_angular = m_invInertia.RotateVector(jInvMass.m_angular);

			dVector tmp(jInvMass.m_linear * m_jacobian[i].m_linear + jInvMass.m_angular * m_jacobian[i].m_angular);

			dFloat a00 = (tmp.m_x + tmp.m_y + tmp.m_z) * 1.0001f;
			massMatrix[i][i] = a00;

			m_impulseMag[i] = 0.0f;
			for (int j = i + 1; j < m_rowCount; j++) {
				dVector tmp1(jInvMass.m_linear * m_jacobian[j].m_linear + jInvMass.m_angular * m_jacobian[j].m_angular);
				dFloat a01 = tmp1.m_x + tmp1.m_y + tmp1.m_z;
				massMatrix[i][j] = a01;
				massMatrix[j][i] = a01;
			}
		}

		dGaussSeidelLcpSor(m_rowCount, D_MAX_ROWS, &massMatrix[0][0], m_impulseMag, m_rhs, m_normalIndex, m_low, m_high, dFloat(1.0e-6f), 32, dFloat(1.1f));

		dVector netImpulse(0.0f);
		for (int i = 0; i < m_rowCount; i++) {
			netImpulse += m_jacobian[i].m_linear.Scale(m_impulseMag[i]);
		}
		return netImpulse;
	}
	
	dMatrix m_invInertia;
	dVector m_veloc;
	dComplementaritySolver::dJacobian m_jacobian[D_MAX_ROWS];
	dFloat m_rhs[D_MAX_ROWS];
	dFloat m_low[D_MAX_ROWS];
	dFloat m_high[D_MAX_ROWS];
	dFloat m_impulseMag[D_MAX_ROWS];
	int m_normalIndex[D_MAX_ROWS];
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
	m_localFrame.m_posit = dVector (0.0f, dFloat (0.0f), dFloat (0.0f), 1.0f);

	NewtonCollision* const capsule = NewtonBodyGetCollision(m_kinematicBody);

	dMatrix oldMatrix;
	dMatrix newMatrix(m_localFrame);
	NewtonCollisionGetMatrix(capsule, &oldMatrix[0][0]);

	newMatrix.m_posit = oldMatrix.m_posit;
	NewtonCollisionSetMatrix(capsule, &newMatrix[0][0]);
}

void dCustomPlayerController::ResolveStep(dFloat timestep, dContactSolver& contactSolver)
{
	dMatrix matrix;
	dVector zero(0.0f);
	dVector saveVeloc(0.0f);

	NewtonBodyGetMatrix(m_kinematicBody, &matrix[0][0]);
	NewtonBodyGetVelocity(m_kinematicBody, &saveVeloc[0]);

	dMatrix coodinateMatrix (m_localFrame * matrix);

	dImpulseSolver impulseSolver(this);
	int index = impulseSolver.AddLinearRow(coodinateMatrix[0], zero, 0.0f, 0.0f, 1.0e12f);
	impulseSolver.AddLinearRow(coodinateMatrix[1], zero, -m_forwardSpeed, -D_STEP_FRICTION, D_STEP_FRICTION, index);
	impulseSolver.AddLinearRow(coodinateMatrix[2], zero,  m_lateralSpeed, -D_STEP_FRICTION, D_STEP_FRICTION, index);
	dVector veloc (saveVeloc + impulseSolver.CalculateImpulse().Scale(m_invMass));

	NewtonBodySetVelocity(m_kinematicBody, &veloc[0]);
	NewtonBodyIntegrateVelocity(m_kinematicBody, timestep);
	contactSolver.CalculateContacts();

	if (contactSolver.m_contactCount) {
		dMatrix stepMatrix;
		dVector com;

		NewtonBodyGetMatrix(m_kinematicBody, &stepMatrix[0][0]);
		NewtonBodyGetCentreOfMass(m_kinematicBody, &com[0]);
		com = stepMatrix.TransformVector(com);

		impulseSolver.Reset(this);
		for (int i = 0; i < contactSolver.m_contactCount; i++) {
			NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
			dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat(0.0f));
			dVector localPointpoint (m_localFrame.UntransformVector(stepMatrix.UntransformVector(point)));
			if (localPointpoint.m_x > m_stepHeight) {
				dVector normal(contact.m_normal[0], contact.m_normal[1], contact.m_normal[2], dFloat(0.0f));
				impulseSolver.AddLinearRow(normal, point - com, 0.0f, 0.0f, 1.0e12f);
			}
		}

		if (impulseSolver.m_rowCount) {
			impulseSolver.AddAngularRows();
			veloc += impulseSolver.CalculateImpulse().Scale(m_invMass);

			NewtonBodySetMatrix(m_kinematicBody, &matrix[0][0]);
			NewtonBodySetVelocity(m_kinematicBody, &veloc[0]);
			NewtonBodyIntegrateVelocity(m_kinematicBody, timestep);
			contactSolver.CalculateContacts();
			NewtonBodyGetMatrix(m_kinematicBody, &stepMatrix[0][0]);
		}

		bool applyStep = false;
		dFloat maxHigh = 0.0f;
		for (int i = 0; i < contactSolver.m_contactCount; i++) {
			NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
			dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat(0.0f));
			point = m_localFrame.UntransformVector(stepMatrix.UntransformVector(point));
			if ((point.m_x < m_stepHeight) && (point.m_x > m_contactPatch)) {
				dVector normal(contact.m_normal[0], contact.m_normal[1], contact.m_normal[2], dFloat(0.0f));
				dFloat relSpeed = normal.DotProduct3(veloc);
				if (relSpeed < dFloat(-1.0e-2f)) {
					applyStep = true;
					maxHigh = dMax(point.m_x, maxHigh);
				}
			}
		}

		if (applyStep) {
			dVector step (stepMatrix.RotateVector(m_localFrame.RotateVector (dVector(maxHigh, dFloat(0.0f), dFloat(0.0f), dFloat(0.0f)))));
			matrix.m_posit += step;
		}
	}

	NewtonBodySetMatrix(m_kinematicBody, &matrix[0][0]);
	NewtonBodySetVelocity(m_kinematicBody, &saveVeloc[0]);
}

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
	//dMatrix predicMatrix;
	dVector veloc(0.0f);
	
	NewtonBodyGetVelocity(m_kinematicBody, &veloc[0]);
	NewtonBodyGetMatrix(m_kinematicBody, &matrix[0][0]);

	//NewtonBodyIntegrateVelocity(m_kinematicBody, timestep);
	//NewtonBodyGetMatrix(m_kinematicBody, &predicMatrix[0][0]);
	//contactSolver.CalculateContacts();
	//NewtonBodySetMatrix(m_kinematicBody, &matrix[0][0]);

	NewtonBodyIntegrateVelocity(m_kinematicBody, timestep);
	dCollisionState playerCollide = TestPredictCollision(contactSolver, veloc);
	NewtonBodySetMatrix(m_kinematicBody, &matrix[0][0]);
	if (playerCollide == m_deepPenetration) {
		dFloat savedTimeStep = timestep;
		timestep *= 0.5f;
		dFloat dt = timestep;
		for (int i = 0; i < D_MAX_COLLIONSION_STEPS; i++) {
			NewtonBodyIntegrateVelocity(m_kinematicBody, timestep);
			//NewtonBodyGetMatrix(m_kinematicBody, &predicMatrix[0][0]);
			//contactCount = NewtonWorldCollide(world, &predicMatrix[0][0], shape, this, PrefilterCallback, contactBuffer, D_MAX_CONTACTS, 0);
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

		NewtonBodySetVelocity(m_kinematicBody, &zero[0]);
		NewtonBodyGetMatrix(m_kinematicBody, &matrix[0][0]);
		NewtonBodyGetCentreOfMass(m_kinematicBody, &com[0]);
		com = matrix.TransformVector(com);
		com.m_w = 0.0f;

		impulseSolver.Reset(this);
		impulseSolver.AddAngularRows();
		for (int i = 0; i < contactSolver.m_contactCount; i++) {
			NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];

			dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat (0.0f));
			dVector normal(contact.m_normal[0], contact.m_normal[1], contact.m_normal[2], dFloat (0.0f));

			penetration = dClamp(contact.m_penetration - D_MAX_COLLISION_PENETRATION * 0.5f, dFloat(0.0f), dFloat(0.5f));
			int index = impulseSolver.AddLinearRow(normal, point - com, 0.0f, 0.0f, 1.0e12f);
			impulseSolver.m_rhs[index] = penetration * invTimestep;
		}

		dVector veloc (impulseSolver.CalculateImpulse().Scale (m_invMass));
		NewtonBodySetVelocity(m_kinematicBody, &veloc[0]);
		NewtonBodyIntegrateVelocity(m_kinematicBody, timestep);

		penetration = 0.0f;
		contactSolver.CalculateContacts();
		for (int i = 0; i < contactSolver.m_contactCount; i++) {
			penetration = dMax(contactSolver.m_contactBuffer[i].m_penetration, penetration);
		}
	}

	NewtonBodySetVelocity(m_kinematicBody, &savedVeloc[0]);
}

void dCustomPlayerController::ResolveCollision(dContactSolver& contactSolver)
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
		const int normalIndex = impulseSolver.AddLinearRow(normal, point - com, 0.0f, 0.0f, 1.0e12f);

		NewtonBodyGetPointVelocity (contact.m_hitBody, &point[0], &surfaceVeloc[0]);
		impulseSolver.m_rhs[impulseSolver.m_rowCount-1] += surfaceVeloc.DotProduct3(normal);

		m_isAirbone = false;
		dVector localPooint (frameMatrix.UntransformVector(point));
		if (localPooint.m_x < contactPatchHigh) {
			m_isOnFloor = true;
			dFloat friction = m_manager->ContactFriction(this, point, normal, int (contact.m_contactID), contact.m_hitBody);
			if (friction > 0.0f) {
				// add lateral traction friction
				dVector sideDir (frameMatrix.m_up.CrossProduct(normal).Normalize());
				impulseSolver.AddLinearRow(sideDir, point - com, -m_lateralSpeed, -friction, friction, normalIndex);
				impulseSolver.m_rhs[impulseSolver.m_rowCount-1] += surfaceVeloc.DotProduct3(sideDir);

				// add longitudinal  traction friction
				dVector frontDir (normal.CrossProduct(sideDir));
				impulseSolver.AddLinearRow(frontDir, point - com, -m_forwardSpeed, -friction, friction, normalIndex);
				impulseSolver.m_rhs[impulseSolver.m_rowCount-1] += surfaceVeloc.DotProduct3(frontDir);
			}
		}
	}

	impulseSolver.AddAngularRows();

	veloc += impulseSolver.CalculateImpulse().Scale(m_invMass);
	NewtonBodySetVelocity(m_kinematicBody, &veloc[0]);
}

void dCustomPlayerController::PreUpdate(dFloat timestep)
{
	dContactSolver contactSolver (this);

	dFloat timeLeft = timestep;
	const dFloat timeEpsilon = timestep * (1.0f / 16.0f);

	m_impulse = dVector(0.0f);
	m_manager->ApplyMove(this, timestep);

#if 0
static int xxxx;
xxxx++;
if (xxxx > 6800)
xxxx *= 1;

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
	ResolveStep(timestep, contactSolver);

	m_isAirbone = true;
	m_isOnFloor = false;
	// advance player until it hit a collision point, until there is not more time left
	for (int i = 0; (i < D_DESCRETE_MOTION_STEPS) && (timeLeft > timeEpsilon); i++) {
		if (timeLeft > timeEpsilon) {
			ResolveCollision(contactSolver);
		}

		dFloat predicetdTime = PredictTimestep(timeLeft, contactSolver);
		NewtonBodyIntegrateVelocity(m_kinematicBody, predicetdTime);
		timeLeft -= predicetdTime;
	}

//veloc = GetVelocity();
//dTrace (("%d %f %f %f\n", xxxx, veloc.m_x, veloc.m_y, veloc.m_z));
}
