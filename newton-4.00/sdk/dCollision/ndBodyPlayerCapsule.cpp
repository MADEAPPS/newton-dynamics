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
#include "ndShapeCapsule.h"
#include "ndBodyPlayerCapsule.h"

D_MSV_NEWTON_ALIGN_32
class ndBodyPlayerCapsuleContactSolver
{
	public:
	ndBodyPlayerCapsuleContactSolver(ndBodyPlayerCapsule* const player)
		:m_controller(player)
		//,m_contactCount(0)
	{
	}

	//void CalculateContacts();
	//static unsigned PrefilterCallback(const NewtonBody* const body, const NewtonCollision* const collision, void* const userData);

	//NewtonWorldConvexCastReturnInfo m_contactBuffer[D_PLAYER_MAX_ROWS];
	//ndBodyPlayerCapsule* m_controller;
	//int m_contactCount;

	ndBodyPlayerCapsule* m_controller;
} D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class ndBodyPlayerCapsuleImpulseSolver
{
	public:
	ndBodyPlayerCapsuleImpulseSolver(ndBodyPlayerCapsule* const controller)
	{
		m_mass = controller->GetMassMatrix().m_w;
		m_invMass = controller->GetInvMass();
		//NewtonBodyGetInvInertiaMatrix(controller->m_newtonBody, &m_invInertia[0][0]);
		m_invInertia = controller->GetInvInertiaMatrix();
		Reset(controller);
	}

	void Reset(ndBodyPlayerCapsule* const controller)
	{
		m_rowCount = 0;
		m_veloc = controller->GetVelocity();
	}

/*
	void AddAngularRows();
	int AddLinearRow(const dVector& dir, const dVector& r, dFloat32 speed, dFloat32 low, dFloat32 high, int normalIndex = -1);
	int AddContactRow(const NewtonWorldConvexCastReturnInfo* const contact, const dVector& dir, const dVector& r, dFloat32 speed, dFloat32 low, dFloat32 high, int normalIndex = -1);
	dVector CalculateImpulse();
	void ApplyReaction(dFloat32 timestep);
	
	dComplementaritySolver::dJacobianPair m_jacobianPairs[D_PLAYER_MAX_ROWS];
	const NewtonWorldConvexCastReturnInfo* m_contactPoint[D_PLAYER_MAX_ROWS];
	dFloat32 m_rhs[D_PLAYER_MAX_ROWS];
	dFloat32 m_low[D_PLAYER_MAX_ROWS];
	dFloat32 m_high[D_PLAYER_MAX_ROWS];
	dFloat32 m_impulseMag[D_PLAYER_MAX_ROWS];
	int m_normalIndex[D_PLAYER_MAX_ROWS];
	dFloat32 m_mass;
	dFloat32 m_invMass;
	int m_rowCount;
*/
	dMatrix m_invInertia;
	dVector m_veloc;
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
		
	m_localFrame = shapeMatrix;
	m_localFrame.m_posit = dVector::m_wOne;
	m_contactPatch = radius / m_weistScale;
	m_stepHeight = dMax(stepHeight, m_contactPatch * dFloat32(2.0f));
}

ndBodyPlayerCapsule::~ndBodyPlayerCapsule()
{
}

void ndBodyPlayerCapsule::ResolveStep(dFloat32 timestep, ndBodyPlayerCapsuleContactSolver& contactSolver)
{
	//dMatrix matrix;
	//dVector zero(0.0f);
	//dVector saveVeloc(0.0f);

	dMatrix matrix(m_matrix);
	dVector saveVeloc(m_veloc);

	dMatrix startMatrix(matrix);
	ndBodyPlayerCapsuleImpulseSolver impulseSolver(this);

	bool hasStartMatrix = false;
	dFloat32 invTimeStep = 1.0f / timestep;
	for (int j = 0; !hasStartMatrix && (j < 4); j++) 
	{
		//hasStartMatrix = true;
		//contactSolver.CalculateContacts();
		//int count = contactSolver.m_contactCount;
		//for (int i = count - 1; i >= 0; i--) {
		//	NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
		//	dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat32(0.0f));
		//	dVector localPointpoint(m_localFrame.UntransformVector(startMatrix.UntransformVector(point)));
		//	if (localPointpoint.m_x < m_stepHeight) {
		//		count--;
		//		contactSolver.m_contactBuffer[i] = contactSolver.m_contactBuffer[count];
		//	}
		//}
		//
		//if (count) {
		//	dVector com(zero);
		//	hasStartMatrix = false;
		//
		//	SetVelocity(zero[0]);
		//	impulseSolver.Reset(this);
		//	NewtonBodyGetCentreOfMass(m_newtonBody, &com[0]);
		//	com = startMatrix.TransformVector(com);
		//	for (int i = 0; i < count; i++) {
		//		NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];
		//		dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat32(0.0f));
		//		dVector normal(contact.m_normal[0], contact.m_normal[1], contact.m_normal[2], dFloat32(0.0f));
		//		dFloat32 speed = dMin((contact.m_penetration + D_MAX_COLLISION_PENETRATION), dFloat32(0.25f)) * invTimeStep;
		//		impulseSolver.AddLinearRow(normal, point - com, speed, 0.0f, 1.0e12f);
		//	}
		//
		//	impulseSolver.AddAngularRows();
		//	dVector veloc(impulseSolver.CalculateImpulse().Scale(m_invMass));
		//	SetVelocity(veloc);
		//	NewtonBodyIntegrateVelocity(m_newtonBody, timestep);
		//	NewtonBodyGetMatrix(m_newtonBody, &startMatrix[0][0]);
		//}
	}

	
/*
	//dAssert (hasStartMatrix);
	if (hasStartMatrix) {
		// clip player velocity along the high contacts

		dMatrix coodinateMatrix(m_localFrame * startMatrix);

		dFloat32 scaleSpeedFactor = 1.5f;
		dFloat32 fowardSpeed = m_forwardSpeed * scaleSpeedFactor;
		dFloat32 lateralSpeed = m_lateralSpeed * scaleSpeedFactor;
		dFloat32 maxSpeed = dMax(dAbs(fowardSpeed), dAbs(lateralSpeed));
		dFloat32 stepFriction = 1.0f + m_mass * maxSpeed;

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
					dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat32(0.0f));
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
						dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat32(0.0f));
						dVector normal(contact.m_normal[0], contact.m_normal[1], contact.m_normal[2], dFloat32(0.0f));
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

			dFloat32 maxHigh = 0.0f;
			for (int i = 0; i < contactSolver.m_contactCount; i++) {
				NewtonWorldConvexCastReturnInfo& contact = contactSolver.m_contactBuffer[i];

				dVector point(contact.m_point[0], contact.m_point[1], contact.m_point[2], dFloat32(0.0f));
				point = m_localFrame.UntransformVector(stepMatrix.UntransformVector(point));
				if ((point.m_x < m_stepHeight) && (point.m_x > m_contactPatch)) {
					dVector normal(contact.m_normal[0], contact.m_normal[1], contact.m_normal[2], dFloat32(0.0f));
					dFloat32 relSpeed = normal.DotProduct3(veloc);
					if (relSpeed < dFloat32(-1.0e-2f)) {
						maxHigh = dMax(point.m_x, maxHigh);
					}
				}
			}

			if (maxHigh > 0.0f) {
				dVector step(stepMatrix.RotateVector(m_localFrame.RotateVector(dVector(maxHigh, dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f)))));
				matrix.m_posit += step;
			}
		}
	}

	SetVelocity(saveVeloc);
	NewtonBodySetMatrix(m_newtonBody, &matrix[0][0]);
*/
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
	ResolveStep(timestep, contactSolver);

	//// advance player until it hit a collision point, until there is not more time left
	//for (int i = 0; (i < D_DESCRETE_MOTION_STEPS) && (timeLeft > timeEpsilon); i++) {
	//	if (timeLeft > timeEpsilon) {
	//		ResolveCollision(contactSolver, timestep);
	//	}
	//
	//	dFloat32 predicetdTime = PredictTimestep(timeLeft, contactSolver);
	//	NewtonBodyIntegrateVelocity(m_newtonBody, predicetdTime);
	//	timeLeft -= predicetdTime;
	//}
	//
	//UpdatePlayerStatus(contactSolver);
}

