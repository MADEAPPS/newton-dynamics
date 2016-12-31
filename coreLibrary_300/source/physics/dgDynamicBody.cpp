/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
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

#include "dgPhysicsStdafx.h"
#include "dgWorld.h"
#include "dgDynamicBody.h"
#include "dgCollisionInstance.h"
#include "dgCollisionLumpedMassParticles.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

dgVector dgDynamicBody::m_equilibriumError2 (DG_ERR_TOLERANCE2);
dgVector dgDynamicBody::m_eulerTaylorCorrection(dgFloat32(1.0f / 12.0f));

dgDynamicBody::dgDynamicBody()
	:dgBody()
	,m_accel(dgFloat32 (0.0f))
	,m_alpha(dgFloat32 (0.0f))
	,m_externalForce(dgFloat32 (0.0f))
	,m_externalTorque(dgFloat32 (0.0f))
	,m_savedExternalForce(dgFloat32 (0.0f))
	,m_savedExternalTorque(dgFloat32 (0.0f))
	,m_dampCoef(dgFloat32 (0.0f))
	,m_sleepingCounter(0)
	,m_isInDestructionArrayLRU(0)
	,m_skeleton(NULL)
	,m_applyExtForces(NULL)
	,m_linearDampOn(true)
	,m_angularDampOn(true)
{
#ifdef DG_USEFULL_INERTIA_MATRIX
	m_principalAxis = dgGetIdentityMatrix();
#endif
	m_type = m_dynamicBody;
	m_rtti |= m_dynamicBodyRTTI;
	dgAssert ( dgInt32 (sizeof (dgDynamicBody) & 0x0f) == 0);
}

dgDynamicBody::dgDynamicBody (dgWorld* const world, const dgTree<const dgCollision*, dgInt32>* const collisionCashe, dgDeserialize serializeCallback, void* const userData, dgInt32 revisionNumber)
	:dgBody(world, collisionCashe, serializeCallback, userData, revisionNumber)
	,m_accel(dgFloat32 (0.0f))
	,m_alpha(dgFloat32 (0.0f))
	,m_externalForce(dgFloat32 (0.0f))
	,m_externalTorque(dgFloat32 (0.0f))
	,m_savedExternalForce(dgFloat32 (0.0f))
	,m_savedExternalTorque(dgFloat32 (0.0f))
	,m_dampCoef(dgFloat32 (0.0f))
	,m_sleepingCounter(0)
	,m_isInDestructionArrayLRU(0)
	,m_skeleton(NULL)
	,m_applyExtForces(NULL)
	,m_linearDampOn(true)
	,m_angularDampOn(true)
{
	dgInt32 val;
	m_type = m_dynamicBody;
	m_rtti |= m_dynamicBodyRTTI;

	m_invWorldInertiaMatrix[3][3] = dgFloat32 (1.0f);
	serializeCallback (userData, &m_mass, sizeof (m_mass));
	serializeCallback (userData, &m_invMass, sizeof (m_invMass));
	serializeCallback (userData, &m_dampCoef, sizeof (m_dampCoef));
	serializeCallback(userData, &val, sizeof (dgInt32));
	m_linearDampOn = (val & 1) ? true : false;
	m_angularDampOn = (val & 2) ? true : false;

#ifdef DG_USEFULL_INERTIA_MATRIX
	serializeCallback(userData, &m_principalAxis, sizeof (m_principalAxis));
#endif
}

dgDynamicBody::~dgDynamicBody()
{
	if (m_skeleton) {
		dgSkeletonContainer* const skel = m_skeleton;
		m_skeleton = NULL;
		m_world->DestroySkeletonContainer(skel);
	}
}

void dgDynamicBody::Serialize (const dgTree<dgInt32, const dgCollision*>& collisionRemapId, dgSerialize serializeCallback, void* const userData)
{
	dgBody::Serialize (collisionRemapId, serializeCallback, userData);

	dgInt32 val = (m_linearDampOn ? 1 : 0) & (m_angularDampOn ? 2 : 0) ;
	serializeCallback (userData, &m_mass, sizeof (m_mass));
	serializeCallback (userData, &m_invMass, sizeof (m_invMass));
	serializeCallback (userData, &m_dampCoef, sizeof (m_dampCoef));
	serializeCallback (userData, &val, sizeof (dgInt32));

#ifdef DG_USEFULL_INERTIA_MATRIX
	serializeCallback(userData, &m_principalAxis, sizeof (m_principalAxis));
#endif
}

#ifdef DG_USEFULL_INERTIA_MATRIX

void dgDynamicBody::SetMassMatrix(dgFloat32 mass, const dgMatrix& inertia)
{
	dgVector II;
	m_principalAxis = inertia;
	m_principalAxis.EigenVectors(II);
	dgMatrix massMatrix(dgGetIdentityMatrix());
	massMatrix[0][0] = II[0];
	massMatrix[1][1] = II[1];
	massMatrix[2][2] = II[2];
	dgBody::SetMassMatrix(mass, massMatrix);
}

dgMatrix dgDynamicBody::CalculateLocalInertiaMatrix() const
{
	dgMatrix matrix(m_principalAxis);
	matrix.m_posit = dgVector::m_wOne;
	dgMatrix diagonal(dgGetIdentityMatrix());
	diagonal[0][0] = m_mass[0];
	diagonal[1][1] = m_mass[1];
	diagonal[2][2] = m_mass[2];
	return matrix * diagonal * matrix.Inverse();
}

dgMatrix dgDynamicBody::CalculateInertiaMatrix() const
{
	dgMatrix matrix(m_principalAxis * m_matrix);
	matrix.m_posit = dgVector::m_wOne;
	dgMatrix diagonal(dgGetIdentityMatrix());
	diagonal[0][0] = m_mass[0];
	diagonal[1][1] = m_mass[1];
	diagonal[2][2] = m_mass[2];
	return matrix * diagonal * matrix.Inverse();
}

dgMatrix dgDynamicBody::CalculateInvInertiaMatrix() const
{
	dgMatrix matrix(m_principalAxis * m_matrix);
	matrix.m_posit = dgVector::m_wOne;
	dgMatrix diagonal(dgGetIdentityMatrix());
	diagonal[0][0] = m_invMass[0];
	diagonal[1][1] = m_invMass[1];
	diagonal[2][2] = m_invMass[2];
	return matrix * diagonal * matrix.Inverse();
}
#endif

void dgDynamicBody::SetMatrixResetSleep(const dgMatrix& matrix)
{
	dgBody::SetMatrixResetSleep(matrix);
	m_savedExternalForce = dgVector (dgFloat32 (0.0f));
	m_savedExternalTorque = dgVector (dgFloat32 (0.0f));
	CalcInvInertiaMatrix();
}

void dgDynamicBody::SetMatrixNoSleep(const dgMatrix& matrix)
{
	dgBody::SetMatrixNoSleep(matrix);
	CalcInvInertiaMatrix();
}


void dgDynamicBody::AttachCollision (dgCollisionInstance* const collision)
{
	dgBody::AttachCollision(collision);
	if (m_collision->IsType(dgCollision::dgCollisionMesh_RTTI) || m_collision->IsType(dgCollision::dgCollisionScene_RTTI)) {
		//SetMassMatrix (m_mass.m_w, m_mass.m_x, m_mass.m_y, m_mass.m_z);
		SetMassMatrix (m_mass.m_w, CalculateLocalInertiaMatrix());
	}
}


bool dgDynamicBody::IsInEquilibrium() const
{
	if (m_equilibrium) {
		dgVector deltaAccel((m_externalForce - m_savedExternalForce).Scale4(m_invMass.m_w));
		dgAssert(deltaAccel.m_w == 0.0f);
		dgFloat32 deltaAccel2 = deltaAccel.DotProduct4(deltaAccel).GetScalar();
		if (deltaAccel2 > DG_ERR_TOLERANCE2) {
			return false;
		}
		dgVector deltaAlpha(m_matrix.UnrotateVector(m_externalTorque - m_savedExternalTorque).CompProduct4(m_invMass));
		dgAssert(deltaAlpha.m_w == 0.0f);
		dgFloat32 deltaAlpha2 = deltaAlpha.DotProduct4(deltaAlpha).GetScalar();
		if (deltaAlpha2 > DG_ERR_TOLERANCE2) {
			return false;
		}
		return true;
	}
	return false;
}

void dgDynamicBody::ApplyExtenalForces (dgFloat32 timestep, dgInt32 threadIndex)
{
	m_externalForce = dgVector (dgFloat32 (0.0f));
	m_externalTorque = dgVector (dgFloat32 (0.0f));
	if (m_applyExtForces) {
		m_applyExtForces(*this, timestep, threadIndex);
	}
}

void dgDynamicBody::InvalidateCache ()
{
	m_sleepingCounter = 0;
	m_savedExternalForce = dgVector (dgFloat32 (0.0f));
	m_savedExternalTorque = dgVector (dgFloat32 (0.0f));
	dgBody::InvalidateCache ();
}

void dgDynamicBody::IntegrateOpenLoopExternalForce(dgFloat32 timestep)
{
	if (!m_equilibrium) {
		if (!m_collision->IsType(dgCollision::dgCollisionLumpedMass_RTTI)) {
			AddDampingAcceleration(timestep);
			CalcInvInertiaMatrix();

			dgVector accel(m_externalForce.Scale4(m_invMass.m_w));
			dgVector alpha(m_invWorldInertiaMatrix.RotateVector(m_externalTorque));

			m_accel = accel;
			m_alpha = alpha;

			dgVector timeStepVect(timestep);
			m_veloc += accel.CompProduct4(timeStepVect);
			dgVector correction(alpha.CrossProduct3(m_omega));
			m_omega += alpha.CompProduct4(timeStepVect.CompProduct4(dgVector::m_half)) + correction.CompProduct4(timeStepVect.CompProduct4(timeStepVect.CompProduct4(m_eulerTaylorCorrection)));
		} else {
			dgCollisionLumpedMassParticles* const lumpedMassShape = (dgCollisionLumpedMassParticles*)m_collision->m_childShape;
			lumpedMassShape->IntegrateForces(timestep);
		}
	} else {
		m_accel = dgVector::m_zero;
		m_alpha = dgVector::m_zero;
	}
}