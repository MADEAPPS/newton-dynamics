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

#include "dgPhysicsStdafx.h"
#include "dgWorld.h"
#include "dgDynamicBody.h"
#include "dgCollisionInstance.h"
#include "dgCollisionLumpedMassParticles.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

dgVector dgDynamicBody::m_equilibriumError2 (DG_ERR_TOLERANCE2);


dgDynamicBodyAsymetric::dgDynamicBodyAsymetric()
	:dgDynamicBody()
	, m_principalAxis(dgGetIdentityMatrix())
{
	m_type = m_dynamicBody;
	m_rtti |= m_dynamicBodyAsymentricRTTI;
	dgAssert(dgInt32(sizeof(dgDynamicBody) & 0x0f) == 0);

}

dgDynamicBodyAsymetric::dgDynamicBodyAsymetric(dgWorld* const world, const dgTree<const dgCollision*, dgInt32>* const collisionNode, dgDeserialize serializeCallback, void* const userData, dgInt32 revisionNumber)
	:dgDynamicBody(world, collisionNode, serializeCallback, userData, revisionNumber)
	, m_principalAxis(dgGetIdentityMatrix())
{
	m_type = m_dynamicBody;
	m_rtti |= m_dynamicBodyRTTI;
	serializeCallback(userData, &m_principalAxis, sizeof(m_principalAxis));
}

void dgDynamicBodyAsymetric::Serialize(const dgTree<dgInt32, const dgCollision*>& collisionRemapId, dgSerialize serializeCallback, void* const userData)
{
	dgDynamicBody::Serialize(collisionRemapId, serializeCallback, userData);
	serializeCallback(userData, &m_principalAxis, sizeof(m_principalAxis));
}


void dgDynamicBodyAsymetric::SetMassMatrix(dgFloat32 mass, const dgMatrix& inertia)
{
	//dgVector II;
	m_principalAxis = inertia;
	dgVector II (m_principalAxis.EigenVectors());
	dgMatrix massMatrix(dgGetIdentityMatrix());
	massMatrix[0][0] = II[0];
	massMatrix[1][1] = II[1];
	massMatrix[2][2] = II[2];
	dgBody::SetMassMatrix(mass, massMatrix);
}

dgMatrix dgDynamicBodyAsymetric::CalculateLocalInertiaMatrix() const
{
	dgMatrix matrix(m_principalAxis);
	matrix.m_posit = dgVector::m_wOne;
	dgMatrix diagonal(dgGetIdentityMatrix());
	diagonal[0][0] = m_mass[0];
	diagonal[1][1] = m_mass[1];
	diagonal[2][2] = m_mass[2];
	return matrix * diagonal * matrix.Inverse();
}

dgMatrix dgDynamicBodyAsymetric::CalculateInertiaMatrix() const
{
	dgMatrix matrix(m_principalAxis * m_matrix);
	matrix.m_posit = dgVector::m_wOne;
	dgMatrix diagonal(dgGetIdentityMatrix());
	diagonal[0][0] = m_mass[0];
	diagonal[1][1] = m_mass[1];
	diagonal[2][2] = m_mass[2];
	return matrix * diagonal * matrix.Inverse();
}

dgMatrix dgDynamicBodyAsymetric::CalculateInvInertiaMatrix() const
{
	dgMatrix matrix(m_principalAxis * m_matrix);
	matrix.m_posit = dgVector::m_wOne;
	dgMatrix diagonal(dgGetIdentityMatrix());
	diagonal[0][0] = m_invMass[0];
	diagonal[1][1] = m_invMass[1];
	diagonal[2][2] = m_invMass[2];
	return matrix * diagonal * matrix.Inverse();
}

void dgDynamicBodyAsymetric::IntegrateOpenLoopExternalForce(dgFloat32 timestep)
{
	dgDynamicBody::IntegrateOpenLoopExternalForce(timestep);
}


dgDynamicBody::dgDynamicBody()
	:dgBody()
	,m_externalForce(dgVector::m_zero)
	,m_externalTorque(dgVector::m_zero)
	,m_savedExternalForce(dgVector::m_zero)
	,m_savedExternalTorque(dgVector::m_zero)
	,m_dampCoef(dgVector::m_zero)
	,m_cachedDampCoef(dgVector::m_zero)
	,m_cachedTimeStep(dgFloat32(0.0f))
	,m_sleepingCounter(0)
	,m_isInDestructionArrayLRU(0)
	,m_skeleton(NULL)
	,m_applyExtForces(NULL)
{
	m_type = m_dynamicBody;
	m_rtti |= m_dynamicBodyRTTI;
	dgAssert ( dgInt32 (sizeof (dgDynamicBody) & 0x0f) == 0);
}

dgDynamicBody::dgDynamicBody (dgWorld* const world, const dgTree<const dgCollision*, dgInt32>* const collisionCashe, dgDeserialize serializeCallback, void* const userData, dgInt32 revisionNumber)
	:dgBody(world, collisionCashe, serializeCallback, userData, revisionNumber)
	,m_externalForce(dgVector::m_zero)
	,m_externalTorque(dgVector::m_zero)
	,m_savedExternalForce(dgVector::m_zero)
	,m_savedExternalTorque(dgVector::m_zero)
	,m_dampCoef(dgVector::m_zero)
	,m_cachedDampCoef(dgVector::m_zero)
	,m_cachedTimeStep(dgFloat32(0.0f))
	,m_sleepingCounter(0)
	,m_isInDestructionArrayLRU(0)
	,m_skeleton(NULL)
	,m_applyExtForces(NULL)
{
	dgInt32 val;
	m_type = m_dynamicBody;
	m_rtti |= m_dynamicBodyRTTI;

	m_invWorldInertiaMatrix[3][3] = dgFloat32 (1.0f);
	serializeCallback (userData, &m_mass, sizeof (m_mass));
	serializeCallback (userData, &m_invMass, sizeof (m_invMass));
	serializeCallback (userData, &m_dampCoef, sizeof (m_dampCoef));
	serializeCallback(userData, &val, sizeof (dgInt32));
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

	serializeCallback (userData, &m_mass, sizeof (m_mass));
	serializeCallback (userData, &m_invMass, sizeof (m_invMass));
	serializeCallback (userData, &m_dampCoef, sizeof (m_dampCoef));
}


void dgDynamicBody::SetMatrixResetSleep(const dgMatrix& matrix)
{
	dgBody::SetMatrixResetSleep(matrix);
	m_savedExternalForce = dgVector::m_zero;
	m_savedExternalTorque = dgVector::m_zero;
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


dgVector dgDynamicBody::GetAlpha() const
{
	return m_alpha;
}

dgVector dgDynamicBody::GetAccel() const
{
	return m_accel;
}

void dgDynamicBody::SetAlpha(const dgVector& alpha)
{
	m_alpha = alpha;
}

void dgDynamicBody::SetAccel(const dgVector& accel)
{
	m_accel = accel;
}


bool dgDynamicBody::IsInEquilibrium() const
{
	if (m_equilibrium) {
		dgVector deltaAccel((m_externalForce - m_savedExternalForce).Scale(m_invMass.m_w));
		dgAssert(deltaAccel.m_w == dgFloat32 (0.0f));
		dgFloat32 deltaAccel2 = deltaAccel.DotProduct(deltaAccel).GetScalar();
		if (deltaAccel2 > DG_ERR_TOLERANCE2) {
			return false;
		}
		dgVector deltaAlpha(m_matrix.UnrotateVector(m_externalTorque - m_savedExternalTorque) * m_invMass);
		dgAssert(deltaAlpha.m_w == dgFloat32 (0.0f));
		dgFloat32 deltaAlpha2 = deltaAlpha.DotProduct(deltaAlpha).GetScalar();
		if (deltaAlpha2 > DG_ERR_TOLERANCE2) {
			return false;
		}
		return true;
	}
	return false;
}

void dgDynamicBody::ApplyExtenalForces (dgFloat32 timestep, dgInt32 threadIndex)
{
	m_externalForce = dgVector::m_zero;
	m_externalTorque = dgVector::m_zero;
	if (m_applyExtForces) {
		m_applyExtForces(*this, timestep, threadIndex);
	}
	
	m_gyroRotation = m_rotation;
	m_gyroTorque = dgVector::m_zero;

	m_externalForce += m_impulseForce;
	m_externalTorque += m_impulseTorque;
	m_impulseForce = dgVector::m_zero;
	m_impulseTorque = dgVector::m_zero;
}

void dgDynamicBody::AddDampingAcceleration(dgFloat32 timestep)
{
	dgVector damp (GetDampCoeffcient (timestep));
	dgVector omegaDamp(damp & dgVector::m_triplexMask);
	dgVector omega(m_matrix.UnrotateVector(m_omega) * omegaDamp);

	m_veloc = m_veloc.Scale(damp.m_w);
	m_omega = m_matrix.RotateVector(omega);
}

void dgDynamicBody::InvalidateCache ()
{
	m_sleepingCounter = 0;
	m_savedExternalForce = dgVector::m_zero;
	m_savedExternalTorque = dgVector::m_zero;
	dgBody::InvalidateCache ();
}

void dgDynamicBody::IntegrateImplicit(dgFloat32 timestep)
{
	// using simple backward Euler or implicit integration, this is. 
	// f'(t + dt) = (f(t + dt) - f(t)) / dt  

	// therefore: 
	// f(t + dt) = f(t) + f'(t + dt) * dt

	// approximate f'(t + dt) by expanding the Taylor of f(w + dt)
	// f(w + dt) = f(w) + f'(w) * dt + f''(w) * dt^2 / 2! + ....

	// assume dt^2 is negligible, therefore we can truncate the expansion to
	// f(w + dt) ~= f(w) + f'(w) * dt

	// calculating dw as the  f'(w) = d(wx, wy, wz) | dt
	// dw/dt = a = (Tl - (wl x (wl * Il)) * Il^1

	// expanding f(w) 
	// f'(wx) = Ix * ax = Tx - (Iz - Iy) * wy * wz 
	// f'(wy) = Iy * ay = Ty - (Ix - Iz) * wz * wx
	// f'(wz) = Iz * az = Tz - (Iy - Ix) * wx * wy
	//
	// calculation the expansion 
	// Ix * ax = (Tx - (Iz - Iy) * wy * wz) - ((Iz - Iy) * wy * az + (Iz - Iy) * ay * wz) * dt
	// Iy * ay = (Ty - (Ix - Iz) * wz * wx) - ((Ix - Iz) * wz * ax + (Ix - Iz) * az * wx) * dt
	// Iz * az = (Tz - (Iy - Ix) * wx * wy) - ((Iy - Ix) * wx * ay + (Iy - Ix) * ax * wy) * dt   
	//
	// factorizing a we get
	// Ix * ax + (Iz - Iy) * dwy * az + (Iz - Iy) * dwz * ay = Tx - (Iz - Iy) * wy * wz
	// Iy * ay + (Ix - Iz) * dwz * ax + (Ix - Iz) * dwx * az = Ty - (Ix - Iz) * wz * wx
	// Iz * az + (Iy - Ix) * dwx * ay + (Iy - Ix) * dwy * ax = Tz - (Iy - Ix) * wx * wy

	dgVector localOmega(m_matrix.UnrotateVector(m_omega));
	dgVector localTorque(m_matrix.UnrotateVector(m_externalTorque - m_gyroTorque));

	// and solving for alpha we get the angular acceleration at t + dt
	// calculate gradient at a full time step
	dgVector gradientStep(localTorque.Scale(timestep));

	// derivative at half time step. (similar to midpoint Euler so that it does not loses too much energy)
	dgVector dw(localOmega.Scale(dgFloat32(0.5f) * timestep));
	//dgVector dw(localOmega.Scale(dgFloat32(1.0f) * timestep));

	// calculates Jacobian matrix (
	//		dWx / dwx, dWx / dwy, dWx / dwz
	//		dWy / dwx, dWy / dwy, dWy / dwz
	//		dWz / dwx, dWz / dwy, dWz / dwz
	//		
	//		dWx / dwx = Ix, dWx / dwy = (Iz - Iy) * wz * dt, dWx / dwz = (Iz - Iy) * wy * dt)
	//		dWy / dwx = (Ix - Iz) * wz * dt, dWy / dwy = Iy, dWy / dwz = (Ix - Iz) * wx * dt
	//		dWz / dwx = (Iy - Ix) * wy * dt, dWz / dwy = (Iy - Ix) * wx * dt, dWz / dwz = Iz
	dgMatrix jacobianMatrix(
		dgVector(m_mass[0], (m_mass[2] - m_mass[1]) * dw[2], (m_mass[2] - m_mass[1]) * dw[1], dgFloat32(0.0f)),
		dgVector((m_mass[0] - m_mass[2]) * dw[2], m_mass[1], (m_mass[0] - m_mass[2]) * dw[0], dgFloat32(0.0f)),
		dgVector((m_mass[1] - m_mass[0]) * dw[1], (m_mass[1] - m_mass[0]) * dw[0], m_mass[2], dgFloat32(0.0f)),
		dgVector::m_wOne);

	gradientStep = jacobianMatrix.SolveByGaussianElimination(gradientStep);

	localOmega += gradientStep;

	m_accel = m_externalForce.Scale(m_invMass.m_w);
	m_alpha = m_matrix.RotateVector(localTorque * m_invMass);

	m_veloc += m_accel.Scale(timestep);
	m_omega = m_matrix.RotateVector(localOmega);
}

dgJacobian dgDynamicBody::IntegrateForceAndToque(const dgVector& force, const dgVector& torque, const dgVector& timestep)
{
	dgJacobian velocStep;
	if (m_gyroTorqueOn) {
		dgVector dtHalf(timestep * dgVector::m_half);
		dgMatrix matrix(m_gyroRotation, dgVector::m_wOne);

		dgVector localOmega(matrix.UnrotateVector(m_omega));
		dgVector localTorque(matrix.UnrotateVector(torque - m_gyroTorque));

		// derivative at half time step. (similar to midpoint Euler so that it does not loses too much energy)
		dgVector dw(localOmega * dtHalf);
		dgMatrix jacobianMatrix(
			dgVector(m_mass[0], (m_mass[2] - m_mass[1]) * dw[2], (m_mass[2] - m_mass[1]) * dw[1], dgFloat32(0.0f)),
			dgVector((m_mass[0] - m_mass[2]) * dw[2], m_mass[1], (m_mass[0] - m_mass[2]) * dw[0], dgFloat32(1.0f)),
			dgVector((m_mass[1] - m_mass[0]) * dw[1], (m_mass[1] - m_mass[0]) * dw[0], m_mass[2], dgFloat32(1.0f)),
			dgVector::m_wOne);

		// and solving for alpha we get the angular acceleration at t + dt
		// calculate gradient at a full time step
		//dgVector gradientStep(localTorque * timestep);
		dgVector gradientStep(jacobianMatrix.SolveByGaussianElimination(localTorque * timestep));

		dgVector omega(matrix.RotateVector(localOmega + gradientStep));
		dgAssert(omega.m_w == dgFloat32(0.0f));

		// integrate rotation here
		dgFloat32 omegaMag2 = omega.DotProduct(omega).GetScalar() + dgFloat32(1.0e-12f);
		dgFloat32 invOmegaMag = dgRsqrt(omegaMag2);
		dgVector omegaAxis(omega.Scale(invOmegaMag));
		dgFloat32 omegaAngle = invOmegaMag * omegaMag2 * timestep.GetScalar();
		dgQuaternion deltaRotation(omegaAxis, omegaAngle);
		m_gyroRotation = m_gyroRotation * deltaRotation;
		dgAssert((m_gyroRotation.DotProduct(m_gyroRotation) - dgFloat32(1.0f)) < dgFloat32(1.0e-5f));

		matrix = dgMatrix(m_gyroRotation, dgVector::m_wOne);
		localOmega = matrix.UnrotateVector(omega);
		//dgVector angularMomentum(inertia * localOmega);
		//body->m_gyroTorque = matrix.RotateVector(localOmega.CrossProduct(angularMomentum));
		//body->m_gyroAlpha = body->m_invWorldInertiaMatrix.RotateVector(body->m_gyroTorque);
		dgVector localGyroTorque(localOmega.CrossProduct(m_mass * localOmega));
		m_gyroTorque = matrix.RotateVector(localGyroTorque);
		m_gyroAlpha = matrix.RotateVector(localGyroTorque * m_invMass);

		velocStep.m_angular = matrix.RotateVector(gradientStep);
	} else {
		velocStep.m_angular = m_invWorldInertiaMatrix.RotateVector(torque) * timestep;
		//velocStep.m_angular = velocStep.m_angular * dgVector::m_half;
	}

	velocStep.m_linear = force.Scale(m_invMass.m_w) * timestep;
	return velocStep;
}

void dgDynamicBody::IntegrateOpenLoopExternalForce(dgFloat32 timestep)
{
	if (!m_equilibrium) {
		if (!m_collision->IsType(dgCollision::dgCollisionLumpedMass_RTTI)) {
			IntegrateImplicit(timestep);
		} else {
			dgCollisionLumpedMassParticles* const lumpedMassShape = (dgCollisionLumpedMassParticles*)m_collision->m_childShape;
			lumpedMassShape->IntegrateForces(timestep);
		}
	} else {
		m_accel = dgVector::m_zero;
		m_alpha = dgVector::m_zero;
	}
}


