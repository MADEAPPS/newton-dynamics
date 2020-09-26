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
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndBodyDynamic.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


//dVector ntDynamicBody::m_equilibriumError2 (DG_ERR_TOLERANCE2);

ndBodyDynamic::ndBodyDynamic()
	:ndBodyKinematic()
	,m_accel(dVector::m_zero)
	,m_alpha(dVector::m_zero)
	,m_externalForce(dVector::m_zero)
	,m_externalTorque(dVector::m_zero)
	,m_impulseForce(dVector::m_zero)
	,m_impulseTorque(dVector::m_zero)
	,m_savedExternalForce(dVector::m_zero)
	,m_savedExternalTorque(dVector::m_zero)
	,m_jointArray()
	,m_sleepingCounter(0)
{
}

ndBodyDynamic::~ndBodyDynamic()
{
}

void ndBodyDynamic::SetForce(const dVector& force)
{
	m_externalForce = force & dVector::m_triplexMask;
	if (m_invMass.m_w == dFloat32(0.0f))
	{
		m_externalForce = dVector::m_zero;
	}

	if (m_equilibrium)
	{
		dVector deltaAccel((m_externalForce - m_savedExternalForce).Scale(m_invMass.m_w));
		dAssert(deltaAccel.m_w == dFloat32(0.0f));
		dFloat32 deltaAccel2 = deltaAccel.DotProduct(deltaAccel).GetScalar();
		m_equilibrium = (deltaAccel2 < D_ERR_TOLERANCE2);
	}
}

void ndBodyDynamic::SetTorque(const dVector& torque)
{
	m_externalTorque = torque & dVector::m_triplexMask;
	if (m_invMass.m_w == dFloat32(0.0f))
	{
		m_externalTorque = dVector::m_zero;
	}

	if (m_equilibrium)
	{
		dVector deltaAlpha(m_matrix.UnrotateVector(m_externalTorque - m_savedExternalTorque) * m_invMass);
		dAssert(deltaAlpha.m_w == dFloat32(0.0f));
		dFloat32 deltaAlpha2 = deltaAlpha.DotProduct(deltaAlpha).GetScalar();
		m_equilibrium = (deltaAlpha2 < D_ERR_TOLERANCE2);
	}
}

void ndBodyDynamic::ApplyExternalForces(dInt32 threadIndex, dFloat32 timestep)
{
	m_externalForce = dVector::m_zero;
	m_externalTorque = dVector::m_zero;
	if (m_notifyCallback)
	{
		m_notifyCallback->OnApplyExternalForce(threadIndex, timestep);
		dAssert(m_externalForce.m_w == dFloat32(0.0f));
		dAssert(m_externalTorque.m_w == dFloat32(0.0f));
	}

	m_gyroRotation = m_rotation;
	m_gyroTorque = dVector::m_zero;

	m_externalForce += m_impulseForce;
	m_externalTorque += m_impulseTorque;
	m_impulseForce = dVector::m_zero;
	m_impulseTorque = dVector::m_zero;
}

void ndBodyDynamic::AddDampingAcceleration(dFloat32 timestep)
{
	//dVector damp(GetDampCoeffcient(timestep));
	//dVector omegaDamp(damp & dVector::m_triplexMask);
	//dVector omega(m_matrix.UnrotateVector(m_omega) * omegaDamp);
	//
	//m_veloc = m_veloc.Scale(damp.m_w);
	//m_omega = m_matrix.RotateVector(omega);
}

void ndBodyDynamic::IntegrateVelocity(dFloat32 timestep)
{
	ndBodyKinematic::IntegrateVelocity(timestep);
	m_savedExternalForce = m_externalForce;
	m_savedExternalTorque = m_externalTorque;
}

ndJacobian ndBodyDynamic::IntegrateForceAndToque(const dVector& force, const dVector& torque, const dVector& timestep)
{
	ndJacobian velocStep;
	if (!m_gyroTorqueOn ||
		((dAbs(m_invMass.m_x - m_invMass.m_y) < dFloat32(1.0e-1f)) &&
		 (dAbs(m_invMass.m_x - m_invMass.m_z) < dFloat32(1.0e-1f))))
	{
		//velocStep.m_angular = m_invWorldInertiaMatrix.RotateVector(torque) * timestep;
		velocStep.m_angular = torque * m_invMass * timestep;
		dAssert(velocStep.m_angular.m_w == dFloat32(0.0f));
	}
	else
	{
		dVector dtHalf(timestep * dVector::m_half);
		dMatrix matrix(m_gyroRotation, dVector::m_wOne);

		dVector localOmega(matrix.UnrotateVector(m_omega));
		dVector localTorque(matrix.UnrotateVector(torque - m_gyroTorque));

		// derivative at half time step. (similar to midpoint Euler so that it does not loses too much energy)
		dVector dw(localOmega * dtHalf);
		dMatrix jacobianMatrix(
			dVector(m_mass[0], (m_mass[2] - m_mass[1]) * dw[2], (m_mass[2] - m_mass[1]) * dw[1], dFloat32(0.0f)),
			dVector((m_mass[0] - m_mass[2]) * dw[2], m_mass[1], (m_mass[0] - m_mass[2]) * dw[0], dFloat32(1.0f)),
			dVector((m_mass[1] - m_mass[0]) * dw[1], (m_mass[1] - m_mass[0]) * dw[0], m_mass[2], dFloat32(1.0f)),
			dVector::m_wOne);

		// and solving for alpha we get the angular acceleration at t + dt
		// calculate gradient at a full time step
		//dVector gradientStep(localTorque * timestep);
		dVector gradientStep(jacobianMatrix.SolveByGaussianElimination(localTorque * timestep));

		dVector omega(matrix.RotateVector(localOmega + gradientStep));
		dAssert(omega.m_w == dFloat32(0.0f));

		// integrate rotation here
		dFloat32 omegaMag2 = omega.DotProduct(omega).GetScalar() + dFloat32(1.0e-12f);
		dFloat32 invOmegaMag = dRsqrt(omegaMag2);
		dVector omegaAxis(omega.Scale(invOmegaMag));
		dFloat32 omegaAngle = invOmegaMag * omegaMag2 * timestep.GetScalar();
		dQuaternion deltaRotation(omegaAxis, omegaAngle);
		m_gyroRotation = m_gyroRotation * deltaRotation;
		dAssert((m_gyroRotation.DotProduct(m_gyroRotation) - dFloat32(1.0f)) < dFloat32(1.0e-5f));

		matrix = dMatrix(m_gyroRotation, dVector::m_wOne);
		localOmega = matrix.UnrotateVector(omega);
		//dVector angularMomentum(inertia * localOmega);
		//body->m_gyroTorque = matrix.RotateVector(localOmega.CrossProduct(angularMomentum));
		//body->m_gyroAlpha = body->m_invWorldInertiaMatrix.RotateVector(body->m_gyroTorque);
		dVector localGyroTorque(localOmega.CrossProduct(m_mass * localOmega));
		m_gyroTorque = matrix.RotateVector(localGyroTorque);
		m_gyroAlpha = matrix.RotateVector(localGyroTorque * m_invMass);

		velocStep.m_angular = matrix.RotateVector(gradientStep);
	}

	velocStep.m_linear = force.Scale(m_invMass.m_w) * timestep;
	return velocStep;
}
