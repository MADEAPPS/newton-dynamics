/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "dCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndContact.h"
#include "ndShapeCapsule.h"
#include "ndBodyPlayerCapsule.h"

#define D_DESCRETE_MOTION_STEPS		4
#define D_PLAYER_MAX_CONTACTS		8
#define D_PLAYER_MAX_ROWS			(3 * D_PLAYER_MAX_CONTACTS)

#define D_MAX_COLLIONSION_STEPS		8

D_MSV_NEWTON_ALIGN_32
class ndBodyPlayerCapsuleContactSolver
{
	public:
	ndBodyPlayerCapsuleContactSolver(ndBodyPlayerCapsule* const player);
	void CalculateContacts();

	//static unsigned PrefilterCallback(const NewtonBody* const body, const NewtonCollision* const collision, void* const userData);
	//NewtonWorldConvexCastReturnInfo m_contactBuffer[D_PLAYER_MAX_ROWS];
	//ndBodyPlayerCapsule* m_player;
	//dInt32 m_contactCount;

	ndBodyPlayerCapsule* m_player;
	dInt32 m_contactCount;
} D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class ndBodyPlayerCapsuleImpulseSolver
{
	public:
	ndBodyPlayerCapsuleImpulseSolver(ndBodyPlayerCapsule* const controller);

	dVector CalculateImpulse();
	void Reset(ndBodyPlayerCapsule* const controller);

	dInt32 AddLinearRow(const dVector& dir, const dVector& r, dFloat32 speed, dFloat32 low, dFloat32 high, dInt32 normalIndex = -1);
/*
	void AddAngularRows();
	
	dInt32 AddContactRow(const NewtonWorldConvexCastReturnInfo* const contact, const dVector& dir, const dVector& r, dFloat32 speed, dFloat32 low, dFloat32 high, dInt32 normalIndex = -1);
	
	void ApplyReaction(dFloat32 timestep);
	
	dComplementaritySolver::dJacobianPair m_jacobianPairs[D_PLAYER_MAX_ROWS];
	const NewtonWorldConvexCastReturnInfo* m_contactPoint[D_PLAYER_MAX_ROWS];
	
	
	
	
	
	dFloat32 m_mass;
	dFloat32 m_invMass;
	dInt32 m_rowCount;
*/
	dMatrix m_invInertia;
	dVector m_veloc;
	ndJacobianPair m_jacobianPairs[D_PLAYER_MAX_ROWS];
	ndContactPoint* m_contactPoint[D_PLAYER_MAX_ROWS];
	dFloat32 m_rhs[D_PLAYER_MAX_ROWS];
	dFloat32 m_low[D_PLAYER_MAX_ROWS];
	dFloat32 m_high[D_PLAYER_MAX_ROWS];
	dInt32 m_normalIndex[D_PLAYER_MAX_ROWS];
	dFloat32 m_impulseMag[D_PLAYER_MAX_ROWS];
	dFloat32 m_mass;
	dFloat32 m_invMass;
	dInt32 m_rowCount;
} D_GCC_NEWTON_ALIGN_32;


ndBodyPlayerCapsule::ndBodyPlayerCapsule(const dMatrix& localAxis, dFloat32 mass, dFloat32 radius, dFloat32 height, dFloat32 stepHeight)
	:ndBodyKinematic()
{
	m_impulse = dVector::m_zero;
	m_headingAngle = dFloat32(0.0f);
	m_forwardSpeed = dFloat32(0.0f);
	m_lateralSpeed = dFloat32(0.0f);
	m_stepHeight = dFloat32(0.0f);
	m_contactPatch = dFloat32(0.0f);
	m_height = height;
	m_weistScale = dFloat32(3.0f);
	m_crouchScale = dFloat32(0.5f);
	m_isAirbone = false;
	m_isOnFloor = false;
	m_isCrouched = false;

	dMatrix shapeMatrix(localAxis);
	shapeMatrix.m_posit = shapeMatrix.m_front.Scale(height * 0.5f);
	shapeMatrix.m_posit.m_w = 1.0f;
	
	height = dMax(height - 2.0f * radius / m_weistScale, dFloat32(0.1f));
	ndShapeInstance instance(new ndShapeCapsule(radius / m_weistScale, radius / m_weistScale, height));
	instance.SetLocalMatrix(shapeMatrix);
	instance.SetScale(dVector (dFloat32 (1.0f), m_weistScale, m_weistScale, dFloat32 (0.0f)));
	ndBodyKinematic::SetCollisionShape(instance);
	
	SetMassMatrix(mass, instance);
	m_invMass = GetInvMass();
	m_mass = ndBodyKinematic::m_mass.m_w;
		
	m_localFrame = shapeMatrix;
	m_localFrame.m_posit = dVector::m_wOne;
	m_contactPatch = radius / m_weistScale;
	m_stepHeight = dMax(stepHeight, m_contactPatch * dFloat32(2.0f));
}

ndBodyPlayerCapsule::~ndBodyPlayerCapsule()
{
}

void ndBodyPlayerCapsule::ResolveStep(ndBodyPlayerCapsuleContactSolver& contactSolver, dFloat32 timestep)
{
	dMatrix matrix(m_matrix);
	dVector saveVeloc(m_veloc);

	dMatrix startMatrix(matrix);
	ndBodyPlayerCapsuleImpulseSolver impulseSolver(this);

	bool hasStartMatrix = false;
	//dFloat32 invTimeStep = 1.0f / timestep;
	for (dInt32 j = 0; !hasStartMatrix && (j < 4); j++) 
	{
		hasStartMatrix = true;
		contactSolver.CalculateContacts();
		dInt32 count = contactSolver.m_contactCount;
		for (dInt32 i = count - 1; i >= 0; i--) 
		{
			dAssert(0);
		//	NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
		//	dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat32(0.0f));
		//	dVector localPointpoint(m_localFrame.UntransformVector(startMatrix.UntransformVector(point)));
		//	if (localPointpoint.m_x < m_stepHeight) {
		//		count--;
		//		contactSolver.m_contactBuffer[i] = contactSolver.m_contactBuffer[count];
		//	}
		}
		
		if (count) 
		{
			dAssert(0);
		//	dVector com(zero);
		//	hasStartMatrix = false;
		//
		//	SetVelocity(zero[0]);
		//	impulseSolver.Reset(this);
		//	NewtonBodyGetCentreOfMass(m_newtonBody, &com[0]);
		//	com = startMatrix.TransformVector(com);
		//	for (dInt32 i = 0; i < count; i++) {
		//		NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
		//		dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat32(0.0f));
		//		dVector normal(contact.m_normal[0], contact.m_normal[1], contact.m_normal[2], dFloat32(0.0f));
		//		dFloat32 speed = dMin((contact.m_penetration + D_MAX_COLLISION_PENETRATION), dFloat32(0.25f)) * invTimeStep;
		//		impulseSolver.AddLinearRow(normal, podInt32 - com, speed, 0.0f, 1.0e12f);
		//	}
		//
		//	impulseSolver.AddAngularRows();
		//	dVector veloc(impulseSolver.CalculateImpulse().Scale(m_invMass));
		//	SetVelocity(veloc);
		//	NewtonBodyIntegrateVelocity(m_newtonBody, timestep);
		//	NewtonBodyGetMatrix(m_newtonBody, &startMatrix[0][0]);
		}
	}

	if (hasStartMatrix) 
	{
		// clip player velocity along the high contacts
		dMatrix coodinateMatrix(m_localFrame * startMatrix);
		dFloat32 scaleSpeedFactor = 1.5f;
		dFloat32 fowardSpeed = m_forwardSpeed * scaleSpeedFactor;
		dFloat32 lateralSpeed = m_lateralSpeed * scaleSpeedFactor;
		dFloat32 maxSpeed = dMax(dAbs(fowardSpeed), dAbs(lateralSpeed));
		dFloat32 stepFriction = 1.0f + m_mass * maxSpeed;
		
		SetVelocity(saveVeloc);
		impulseSolver.Reset(this);
		dInt32 index = impulseSolver.AddLinearRow(coodinateMatrix[0], dVector::m_zero, dFloat32 (0.0f), dFloat32(0.0f), dFloat32(1.0e12f));
		impulseSolver.AddLinearRow(coodinateMatrix[1], dVector::m_zero, -fowardSpeed, -stepFriction, stepFriction, index);
		impulseSolver.AddLinearRow(coodinateMatrix[2], dVector::m_zero, lateralSpeed, -stepFriction, stepFriction, index);
		dVector veloc(saveVeloc + impulseSolver.CalculateImpulse().Scale(m_invMass));
		
		bool advanceIsBlocked = true;
		for (dInt32 j = 0; advanceIsBlocked && (j < 4); j++) 
		{
			advanceIsBlocked = false;
			SetVelocity(veloc);
			IntegrateVelocity(timestep);
			contactSolver.CalculateContacts();
			if (contactSolver.m_contactCount) 
			{
				dAssert(0);
		//		dMatrix stepMatrix;
		//		dVector com(zero);
		//
		//		NewtonBodyGetMatrix(m_newtonBody, &stepMatrix[0][0]);
		//		dInt32 count = contactSolver.m_contactCount;
		//
		//		// filter by position
		//		for (dInt32 i = count - 1; i >= 0; i--) {
		//			NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
		//			dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat32(0.0f));
		//			dVector localPointpoint(m_localFrame.UntransformVector(stepMatrix.UntransformVector(point)));
		//			if (localPointpoint.m_x < m_stepHeight) {
		//				count--;
		//				contactSolver.m_contactBuffer[i] = contactSolver.m_contactBuffer[count];
		//			}
		//		}
		//
		//		if (count) {
		//			NewtonBodyGetCentreOfMass(m_newtonBody, &com[0]);
		//			com = stepMatrix.TransformVector(com);
		//			advanceIsBlocked = true;
		//
		//			impulseSolver.Reset(this);
		//			for (dInt32 i = 0; i < count; i++) {
		//				NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
		//				dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat32(0.0f));
		//				dVector normal(contact.m_normal[0], contact.m_normal[1], contact.m_normal[2], dFloat32(0.0f));
		//				impulseSolver.AddLinearRow(normal, podInt32 - com, 0.0f, 0.0f, 1.0e12f);
		//			}
		//
		//			impulseSolver.AddAngularRows();
		//			veloc += impulseSolver.CalculateImpulse().Scale(m_invMass);
		//			NewtonBodySetMatrix(m_newtonBody, &startMatrix[0][0]);
		//		}
			}
		}
		
		//SetVelocity(veloc);
		//NewtonBodySetMatrix(m_newtonBody, &startMatrix[0][0]);
		//NewtonBodyIntegrateVelocity(m_newtonBody, timestep);
		//contactSolver.CalculateContacts();
		//if (contactSolver.m_contactCount) {
		//	dMatrix stepMatrix;
		//	NewtonBodyGetMatrix(m_newtonBody, &stepMatrix[0][0]);
		//
		//	dFloat32 maxHigh = 0.0f;
		//	for (dInt32 i = 0; i < contactSolver.m_contactCount; i++) {
		//		NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
		//
		//		dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat32(0.0f));
		//		podInt32 = m_localFrame.UntransformVector(stepMatrix.UntransformVector(point));
		//		if ((point.m_x < m_stepHeight) && (point.m_x > m_contactPatch)) {
		//			dVector normal(contact.m_normal[0], contact.m_normal[1], contact.m_normal[2], dFloat32(0.0f));
		//			dFloat32 relSpeed = normal.DotProduct3(veloc);
		//			if (relSpeed < dFloat32(-1.0e-2f)) {
		//				maxHigh = dMax(point.m_x, maxHigh);
		//			}
		//		}
		//	}
		//
		//	if (maxHigh > 0.0f) {
		//		dVector step(stepMatrix.RotateVector(m_localFrame.RotateVector(dVector(maxHigh, dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f)))));
		//		matrix.m_posit += step;
		//	}
		//}
	}
	
	SetVelocity(saveVeloc);
	SetMatrix(matrix);
}

void ndBodyPlayerCapsule::IntegrateExternalForce(dFloat32 timestep)
{
	ndBodyPlayerCapsuleContactSolver contactSolver(this);
	
	dFloat32 timeLeft = timestep;
	const dFloat32 timeEpsilon = timestep * (1.0f / 16.0f);
	
	m_impulse = dVector::m_zero;
	ApplyInputs(timestep);
	
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
	matrix.m_posit = m_matrix.m_posit;
	//NewtonBodyGetPosition(m_newtonBody, &matrix.m_posit[0]);
	//NewtonBodySetMatrix(m_newtonBody, &matrix[0][0]);
	SetMatrix(matrix);
	
	//// set play desired velocity
	dVector veloc(GetVelocity() + m_impulse.Scale(m_invMass));
	SetVelocity(veloc);
	
	// determine if player has to step over obstacles lower than step hight
	ResolveStep(contactSolver, timestep);

	// advance player until it hit a collision point, until there is not more time left
	for (dInt32 i = 0; (i < D_DESCRETE_MOTION_STEPS) && (timeLeft > timeEpsilon); i++) 
	{
		if (timeLeft > timeEpsilon) 
		{
			ResolveCollision(contactSolver, timestep);
		}
	
		dFloat32 predicetdTime = PredictTimestep(contactSolver, timeLeft);
		IntegrateVelocity(predicetdTime);
		timeLeft -= predicetdTime;
	}
	
	UpdatePlayerStatus(contactSolver);
}

void ndBodyPlayerCapsule::UpdatePlayerStatus(ndBodyPlayerCapsuleContactSolver& contactSolver)
{
	m_isAirbone = true;
	m_isOnFloor = false;
	contactSolver.CalculateContacts();
	dMatrix matrix(m_localFrame * GetMatrix());
	for (int i = 0; i < contactSolver.m_contactCount; i++) 
	{
		dAssert(0);
		//m_isAirbone = false;
		//NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
		//dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat(0.0f));
		//dVector localPoint(matrix.UntransformVector(point));
		//if (localPoint.m_x < m_contactPatch) {
		//	dVector normal(contact.m_normal[0], contact.m_normal[1], contact.m_normal[2], dFloat(0.0f));
		//	dVector localNormal(matrix.UnrotateVector(normal));
		//	if (localNormal.m_x > 0.95f) {
		//		m_isOnFloor = true;
		//	}
		//}
	}
}

ndBodyPlayerCapsule::dCollisionState ndBodyPlayerCapsule::TestPredictCollision(const ndBodyPlayerCapsuleContactSolver& contactSolver, const dVector& veloc) const
{
	for (int i = 0; i < contactSolver.m_contactCount; i++) 
	{
		dAssert(0);
		//const NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
		//if (contact.m_penetration >= D_MAX_COLLISION_PENETRATION) {
		//	return m_deepPenetration;
		//}
	}

	for (int i = 0; i < contactSolver.m_contactCount; i++) 
	{
		dAssert(0);
		//const NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
		//dFloat projecSpeed = veloc.DotProduct3(contact.m_normal);
		//if (projecSpeed < dFloat(0.0f)) {
		//	return m_colliding;
		//}
	}
	return m_freeMovement;
}

dFloat32 ndBodyPlayerCapsule::PredictTimestep(ndBodyPlayerCapsuleContactSolver& contactSolver, dFloat32 timestep)
{
	dMatrix matrix(m_matrix);
	dVector veloc(GetVelocity());
	IntegrateVelocity(timestep);
	dCollisionState playerCollide = TestPredictCollision(contactSolver, veloc);
	SetMatrix(matrix);

	if (playerCollide == m_deepPenetration) 
	{
		dAssert(0);
		//dFloat32 savedTimeStep = timestep;
		//timestep *= 0.5f;
		//dFloat32 dt = timestep;
		//for (int i = 0; i < D_MAX_COLLIONSION_STEPS; i++) 
		//{
		//	NewtonBodyIntegrateVelocity(m_newtonBody, timestep);
		//	contactSolver.CalculateContacts();
		//	NewtonBodySetMatrix(m_newtonBody, &matrix[0][0]);
		//
		//	dt *= 0.5f;
		//	playerCollide = TestPredictCollision(contactSolver, veloc);
		//	if (playerCollide == m_colliding) {
		//		return timestep;
		//	}
		//	if (playerCollide == m_deepPenetration) {
		//		timestep -= dt;
		//	}
		//	else {
		//		timestep += dt;
		//	}
		//}
		//if (timestep > dt * 2.0f) {
		//	return timestep;
		//}
		//
		//dt = savedTimeStep / D_MAX_COLLIONSION_STEPS;
		//timestep = dt;
		//for (int i = 1; i < D_MAX_COLLIONSION_STEPS; i++) {
		//	NewtonBodyIntegrateVelocity(m_newtonBody, timestep);
		//	contactSolver.CalculateContacts();
		//	NewtonBodySetMatrix(m_newtonBody, &matrix[0][0]);
		//	playerCollide = TestPredictCollision(contactSolver, veloc);
		//	if (playerCollide != m_freeMovement) {
		//		return timestep;
		//	}
		//	timestep += dt;
		//}
		//dAssert(0);
	}

	return timestep;
}

void ndBodyPlayerCapsule::ResolveCollision(ndBodyPlayerCapsuleContactSolver& contactSolver, dFloat32 timestep)
{
	//dMatrix matrix;
	//NewtonBodyGetMatrix(m_newtonBody, &matrix[0][0]);

	contactSolver.CalculateContacts();
	if (!contactSolver.m_contactCount) 
	{
		return;
	}

dAssert(0);
/*

	dFloat32 maxPenetration = 0.0f;
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
	const dFloat32 contactPatchHigh = m_contactPatch * dFloat32(0.995f);
	for (int i = 0; i < contactSolver.m_contactCount; i++) {
		NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
		dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat32(0.0f));
		dVector normal(contact.m_normal[0], contact.m_normal[1], contact.m_normal[2], dFloat32(0.0f));
		const int normalIndex = impulseSolver.AddContactRow(&contact, normal, point - com, 0.0f, 0.0f, 1.0e12f);
		dVector localPoint(frameMatrix.UntransformVector(point));

		if (localPoint.m_x < contactPatchHigh) {
			if (impulseSolver.m_contactPoint[normalIndex]) {
				impulseSolver.m_jacobianPairs[normalIndex].m_jacobian_J10.m_linear = impulseSolver.m_zero;
				impulseSolver.m_jacobianPairs[normalIndex].m_jacobian_J10.m_angular = impulseSolver.m_zero;
			}

			dFloat32 friction = ContactFrictionCallback(point, normal, int(contact.m_contactID), contact.m_hitBody);
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
*/
}


ndBodyPlayerCapsuleImpulseSolver::ndBodyPlayerCapsuleImpulseSolver(ndBodyPlayerCapsule* const controller)
{
	m_mass = controller->GetMassMatrix().m_w;
	m_invMass = controller->GetInvMass();
	//NewtonBodyGetInvInertiaMatrix(controller->m_newtonBody, &m_invInertia[0][0]);
	m_invInertia = controller->GetInvInertiaMatrix();
	Reset(controller);
}

void ndBodyPlayerCapsuleImpulseSolver::Reset(ndBodyPlayerCapsule* const controller)
{
	m_rowCount = 0;
	m_veloc = controller->GetVelocity();
}


ndBodyPlayerCapsuleContactSolver::ndBodyPlayerCapsuleContactSolver(ndBodyPlayerCapsule* const player)
	:m_player(player)
	,m_contactCount(0)
{
}

void ndBodyPlayerCapsuleContactSolver::CalculateContacts()
{
	//dMatrix matrix;
	//NewtonBody* const body = m_player->GetBody();
	//NewtonWorld* const world = m_player->GetManager()->GetWorld();
	//NewtonCollision* const shape = NewtonBodyGetCollision(body);
	//
	//NewtonBodyGetMatrix(body, &matrix[0][0]);
	//m_contactCount = NewtonWorldCollide(world, &matrix[0][0], shape, m_player, PrefilterCallback, m_contactBuffer, D_PLAYER_MAX_CONTACTS, 0);

	//const ndBodyKinematic::ndContactMap& map = m_player->GetContactMap();

	m_contactCount = 0;
	ndBodyKinematic::ndContactMap::Iterator it(m_player->GetContactMap());
	for (it.Begin(); it; it++)
	{
		ndContact* const contact = *it;
		if (contact->IsActive())
		{
			dAssert(0);
		}
	}
}

dInt32 ndBodyPlayerCapsuleImpulseSolver::AddLinearRow(const dVector& dir, const dVector& r, dFloat32 speed, dFloat32 low, dFloat32 high, dInt32 normalIndex)
{
	m_contactPoint[m_rowCount] = nullptr;
	m_jacobianPairs[m_rowCount].m_jacobianM0.m_linear = dir;
	m_jacobianPairs[m_rowCount].m_jacobianM0.m_angular = r.CrossProduct(dir);
	m_jacobianPairs[m_rowCount].m_jacobianM1.m_linear = dVector::m_zero;
	m_jacobianPairs[m_rowCount].m_jacobianM1.m_angular = dVector::m_zero;

	m_low[m_rowCount] = low;
	m_high[m_rowCount] = high;
	m_normalIndex[m_rowCount] = (normalIndex == -1) ? 0 : normalIndex - m_rowCount;
	m_rhs[m_rowCount] = speed - m_veloc.DotProduct(m_jacobianPairs[m_rowCount].m_jacobianM0.m_linear).GetScalar();
	m_rowCount++;
	dAssert(m_rowCount < D_PLAYER_MAX_ROWS);
	return m_rowCount - 1;
}

dVector ndBodyPlayerCapsuleImpulseSolver::CalculateImpulse()
{
	dFloat32 massMatrix[D_PLAYER_MAX_ROWS][D_PLAYER_MAX_ROWS];
	const ndBodyKinematic* bodyArray[D_PLAYER_MAX_ROWS];
	for (int i = 0; i < m_rowCount; i++) 
	{
		//bodyArray[i] = m_contactPoint[i] ? m_contactPoint[i]->m_hitBody : NULL;
		bodyArray[i] = m_contactPoint[i] ? m_contactPoint[i]->m_body0 : nullptr;
	}

	for (int i = 0; i < m_rowCount; i++) 
	{
		ndJacobianPair jInvMass(m_jacobianPairs[i]);

		jInvMass.m_jacobianM0.m_linear = jInvMass.m_jacobianM0.m_linear.Scale(m_invMass);
		jInvMass.m_jacobianM0.m_angular = m_invInertia.RotateVector(jInvMass.m_jacobianM0.m_angular);
		if (bodyArray[i]) 
		{
			dAssert(0);
			//dMatrix invInertia;
			//dFloat32 invIxx;
			//dFloat32 invIyy;
			//dFloat32 invIzz;
			//dFloat32 invMass;
			//
			//NewtonBodyGetInvMass(bodyArray[i], &invMass, &invIxx, &invIyy, &invIzz);
			//NewtonBodyGetInvInertiaMatrix(bodyArray[i], &invInertia[0][0]);
			//jInvMass.m_jacobian_J10.m_linear = jInvMass.m_jacobian_J10.m_linear.Scale(invMass);
			//jInvMass.m_jacobian_J10.m_angular = invInertia.RotateVector(jInvMass.m_jacobian_J10.m_angular);
		}
		else 
		{
			jInvMass.m_jacobianM1.m_linear = dVector::m_zero;
			jInvMass.m_jacobianM1.m_angular = dVector::m_zero;
		}

		dVector tmp(
			jInvMass.m_jacobianM0.m_linear * m_jacobianPairs[i].m_jacobianM0.m_linear +
			jInvMass.m_jacobianM0.m_angular * m_jacobianPairs[i].m_jacobianM0.m_angular +
			jInvMass.m_jacobianM1.m_linear * m_jacobianPairs[i].m_jacobianM1.m_linear +
			jInvMass.m_jacobianM1.m_angular * m_jacobianPairs[i].m_jacobianM1.m_angular);
		dFloat32 a00 = (tmp.m_x + tmp.m_y + tmp.m_z) * dFloat32 (1.0004f);

		massMatrix[i][i] = a00;

		m_impulseMag[i] = 0.0f;
		for (int j = i + 1; j < m_rowCount; j++) 
		{
			dVector tmp1(
				jInvMass.m_jacobianM0.m_linear * m_jacobianPairs[j].m_jacobianM0.m_linear +
				jInvMass.m_jacobianM0.m_angular * m_jacobianPairs[j].m_jacobianM0.m_angular);
			if (bodyArray[i] == bodyArray[j]) 
			{
				tmp1 += jInvMass.m_jacobianM1.m_linear * m_jacobianPairs[j].m_jacobianM1.m_linear;
				tmp1 += jInvMass.m_jacobianM1.m_angular * m_jacobianPairs[j].m_jacobianM1.m_angular;
			}

			dFloat32 a01 = tmp1.m_x + tmp1.m_y + tmp1.m_z;
			massMatrix[i][j] = a01;
			massMatrix[j][i] = a01;
		}
	}

	dAssert(dTestPSDmatrix(m_rowCount, D_PLAYER_MAX_ROWS, &massMatrix[0][0]));
	dGaussSeidelLcpSor(m_rowCount, D_PLAYER_MAX_ROWS, &massMatrix[0][0], m_impulseMag, m_rhs, m_normalIndex, m_low, m_high, dFloat32(1.0e-6f), 32, dFloat32(1.1f));

	dVector netImpulse(0.0f);
	for (int i = 0; i < m_rowCount; i++) 
	{
		netImpulse += m_jacobianPairs[i].m_jacobianM0.m_linear.Scale(m_impulseMag[i]);
	}
	return netImpulse;
}