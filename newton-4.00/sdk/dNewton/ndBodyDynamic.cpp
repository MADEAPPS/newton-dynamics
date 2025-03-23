/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndBodyDynamic.h"

ndVector ndBodyDynamic::m_sleepAccelTestScale2(ndFloat32 (0.0625f));

ndBodyDynamic::ndBodyDynamic()
	:ndBodyKinematic()
	,m_externalForce(ndVector::m_zero)
	,m_externalTorque(ndVector::m_zero)
	,m_impulseForce(ndVector::m_zero)
	,m_impulseTorque(ndVector::m_zero)
	,m_savedExternalForce(ndVector::m_zero)
	,m_savedExternalTorque(ndVector::m_zero)
	,m_dampCoef(ndVector::m_zero)
	,m_cachedDampCoef(ndVector::m_one)
	,m_sleepAccelTest2(D_SOLVER_MAX_ACCEL_ERROR * D_SOLVER_MAX_ACCEL_ERROR)
	,m_cachedTimeStep(ndFloat32(0.0f))
{
	m_isDynamics = 1;
}

ndBodyDynamic::ndBodyDynamic(const ndBodyDynamic& src)
	:ndBodyKinematic(src)
{
	m_isDynamics = 1;
}

ndBodyDynamic::~ndBodyDynamic()
{
}

void ndBodyDynamic::SetForce(const ndVector& force)
{
	m_externalForce = force & ndVector::m_triplexMask;
	if (m_invMass.m_w == ndFloat32(0.0f))
	{
		m_externalForce = ndVector::m_zero;
	}

	if (m_equilibrium)
	{
		ndVector deltaAccel((m_externalForce - m_savedExternalForce).Scale(m_invMass.m_w));
		ndAssert(deltaAccel.m_w == ndFloat32(0.0f));
		ndFloat32 deltaAccel2 = deltaAccel.DotProduct(deltaAccel).GetScalar();
		m_equilibrium = ndUnsigned8(deltaAccel2 < D_ERR_TOLERANCE2);
	}
}

void ndBodyDynamic::SetTorque(const ndVector& torque)
{
	m_externalTorque = torque & ndVector::m_triplexMask;
	if (m_invMass.m_w == ndFloat32(0.0f))
	{
		m_externalTorque = ndVector::m_zero;
	}

	if (m_equilibrium)
	{
		ndVector deltaAlpha(m_matrix.UnrotateVector(m_externalTorque - m_savedExternalTorque) * m_invMass);
		ndAssert(deltaAlpha.m_w == ndFloat32(0.0f));
		ndFloat32 deltaAlpha2 = deltaAlpha.DotProduct(deltaAlpha).GetScalar();
		m_equilibrium = ndUnsigned8(deltaAlpha2 < D_ERR_TOLERANCE2);
	}
}

void ndBodyDynamic::ApplyExternalForces(ndInt32 threadIndex, ndFloat32 timestep)
{
	m_externalForce = ndVector::m_zero;
	m_externalTorque = ndVector::m_zero;
	if (m_notifyCallback)
	{
		m_notifyCallback->OnApplyExternalForce(threadIndex, timestep);
		ndAssert(m_externalForce.m_w == ndFloat32(0.0f));
		ndAssert(m_externalTorque.m_w == ndFloat32(0.0f));
	}

	m_externalForce += m_impulseForce;
	m_externalTorque += m_impulseTorque;
	m_impulseForce = ndVector::m_zero;
	m_impulseTorque = ndVector::m_zero;
}

void ndBodyDynamic::AddImpulse(const ndVector& pointDeltaVeloc, const ndVector& pointPosit, ndFloat32 timestep)
{
	ndMatrix invInertia(CalculateInvInertiaMatrix());

	// get contact matrix
	ndMatrix tmp;
	ndVector globalContact(pointPosit - m_globalCentreOfMass);

	tmp[0][0] = ndFloat32(0.0f);
	tmp[0][1] = +globalContact[2];
	tmp[0][2] = -globalContact[1];
	tmp[0][3] = ndFloat32(0.0f);

	tmp[1][0] = -globalContact[2];
	tmp[1][1] = ndFloat32(0.0f);
	tmp[1][2] = +globalContact[0];
	tmp[1][3] = ndFloat32(0.0f);

	tmp[2][0] = +globalContact[1];
	tmp[2][1] = -globalContact[0];
	tmp[2][2] = ndFloat32(0.0f);
	tmp[2][3] = ndFloat32(0.0f);

	tmp[3][0] = ndFloat32(0.0f);
	tmp[3][1] = ndFloat32(0.0f);
	tmp[3][2] = ndFloat32(0.0f);
	tmp[3][3] = ndFloat32(1.0f);

	ndMatrix contactMatrix(tmp * invInertia * tmp);
	//for (ndInt32 i = 0; i < 3; ++i) 
	//{
	//	for (ndInt32 j = 0; j < 3; ++j) 
	//	{
	//		contactMatrix[i][j] *= -ndFloat32(1.0f);
	//	}
	//}
	contactMatrix[0] = contactMatrix[0] * ndVector::m_negOne;
	contactMatrix[1] = contactMatrix[1] * ndVector::m_negOne;
	contactMatrix[2] = contactMatrix[2] * ndVector::m_negOne;
	contactMatrix[0][0] += m_invMass.m_w;
	contactMatrix[1][1] += m_invMass.m_w;
	contactMatrix[2][2] += m_invMass.m_w;

	contactMatrix = contactMatrix.Inverse4x4();

	// change of momentum
	ndVector changeOfMomentum(contactMatrix.RotateVector(pointDeltaVeloc));

	if (changeOfMomentum.DotProduct(changeOfMomentum).GetScalar() > ndFloat32(1.0e-6f))
	{
		m_impulseForce += changeOfMomentum.Scale(1.0f / timestep);
		m_impulseTorque += globalContact.CrossProduct(m_impulseForce);

		m_equilibrium = false;
		//Unfreeze();
	}
}

void ndBodyDynamic::ApplyImpulsePair(const ndVector& linearImpulseIn, const ndVector& angularImpulseIn, ndFloat32 timestep)
{
	ndVector linearImpulse(linearImpulseIn & ndVector::m_triplexMask);
	ndVector angularImpulse(angularImpulseIn & ndVector::m_triplexMask);
	ndAssert(linearImpulse.m_w == ndFloat32(0.0f));
	ndAssert(angularImpulse.m_w == ndFloat32(0.0f));
	if ((linearImpulse.DotProduct(linearImpulse).GetScalar() > ndFloat32(1.0e-6f)) ||
		(angularImpulse.DotProduct(angularImpulse).GetScalar() > ndFloat32(1.0e-6f))) 
	{
		m_impulseForce += linearImpulse.Scale(1.0f / timestep);
		m_impulseTorque += angularImpulse.Scale(1.0f / timestep);

		m_equilibrium = false;
	}
}

void ndBodyDynamic::ApplyImpulsesAtPoint(ndInt32 count, const ndVector* const impulseArray, const ndVector* const pointArray, ndFloat32 timestep)
{
	ndVector impulse(ndVector::m_zero);
	ndVector angularImpulse(ndVector::m_zero);

	ndVector com(m_globalCentreOfMass);
	for (ndInt32 i = 0; i < count; ++i) 
	{
		ndVector r(pointArray[i]);
		ndVector L(impulseArray[i]);
		ndVector Q((r - com).CrossProduct(L));

		impulse += L;
		angularImpulse += Q;
	}

	impulse = impulse & ndVector::m_triplexMask;
	angularImpulse = angularImpulse & ndVector::m_triplexMask;

	if ((impulse.DotProduct(impulse).GetScalar() > ndFloat32(1.0e-6f)) ||
		(angularImpulse.DotProduct(angularImpulse).GetScalar() > ndFloat32(1.0e-6f))) 
	{
		m_impulseForce += impulse.Scale(1.0f / timestep);
		m_impulseTorque += angularImpulse.Scale(1.0f / timestep);

		m_equilibrium = false;
	}
}

void ndBodyDynamic::SetLinearDamping(ndFloat32 linearDamp)
{
	linearDamp = ndClamp(linearDamp, ndFloat32(0.0f), ndFloat32(1.0f));
	m_dampCoef.m_w = D_MAX_SPEED_ATT * linearDamp;
	m_cachedTimeStep = ndFloat32(0.0f);
}

ndFloat32 ndBodyDynamic::GetLinearDamping() const
{
	return m_dampCoef.m_w / D_MAX_SPEED_ATT;
}

ndVector ndBodyDynamic::GetAngularDamping() const
{
	return ndVector(m_dampCoef.m_x / D_MAX_SPEED_ATT,
				    m_dampCoef.m_y / D_MAX_SPEED_ATT,
				    m_dampCoef.m_z / D_MAX_SPEED_ATT, ndFloat32(0.0f));
}

void ndBodyDynamic::SetAngularDamping(const ndVector& angularDamp)
{
	ndFloat32 tmp = ndClamp(angularDamp.m_x, ndFloat32(0.0f), ndFloat32(1.0f));
	m_dampCoef.m_x = D_MAX_SPEED_ATT * tmp;

	tmp = ndClamp(angularDamp.m_y, ndFloat32(0.0f), ndFloat32(1.0f));
	m_dampCoef.m_y = D_MAX_SPEED_ATT * tmp;

	tmp = ndClamp(angularDamp.m_z, ndFloat32(0.0f), ndFloat32(1.0f));
	m_dampCoef.m_z = D_MAX_SPEED_ATT * tmp;

	m_cachedTimeStep = ndFloat32(0.0f);
}

void ndBodyDynamic::AddDampingAcceleration(ndFloat32 timestep)
{
	if (ndAbs(m_cachedTimeStep - timestep) > ndFloat32(1.0e-6f)) 
	{
		m_cachedTimeStep = timestep;
		// assume a nominal 60 frame seconds time step.
		ndFloat32 tau = ndFloat32(60.0f) * timestep;
		// recalculate damping to match the time independent drag
		m_cachedDampCoef.m_x = ndPow(ndFloat32(1.0f) - m_dampCoef.m_x, tau);
		m_cachedDampCoef.m_y = ndPow(ndFloat32(1.0f) - m_dampCoef.m_y, tau);
		m_cachedDampCoef.m_z = ndPow(ndFloat32(1.0f) - m_dampCoef.m_z, tau);
		m_cachedDampCoef.m_w = ndPow(ndFloat32(1.0f) - m_dampCoef.m_w, tau);
	}

	const ndVector omegaDamp(m_cachedDampCoef & ndVector::m_triplexMask);
	const ndVector omega(omegaDamp * m_inertiaPrincipalAxis.UnrotateVector(m_matrix.UnrotateVector(m_omega)));
	m_omega = m_matrix.RotateVector(m_inertiaPrincipalAxis.RotateVector(omega));
	m_veloc = m_veloc.Scale(m_cachedDampCoef.m_w);
}

ndFloat32 ndBodyDynamic::GetSleepAccel() const
{
	return m_sleepAccelTest2.m_x;
}

void ndBodyDynamic::SetSleepAccel(ndFloat32 accelMag2)
{
	m_sleepAccelTest2 = ndVector(accelMag2);
}

void ndBodyDynamic::IntegrateVelocity(ndFloat32 timestep)
{
	ndBodyKinematic::IntegrateVelocity(timestep);
	SaveExternalForces();
}

ndJacobian ndBodyDynamic::IntegrateForceAndToque(const ndVector& force, const ndVector& torque, const ndVector& timestep) const
{
	ndJacobian velocStep;

	const ndMatrix matrix(ndCalculateMatrix(m_gyroRotation, ndVector::m_wOne));
	const ndVector localOmega(m_inertiaPrincipalAxis.UnrotateVector(matrix.UnrotateVector(m_omega)));
	const ndVector localTorque(m_inertiaPrincipalAxis.UnrotateVector(matrix.UnrotateVector(torque)));

	// derivative at half time step. (similar to midpoint Euler so that it does not loses too much energy)
	const ndVector dw(localOmega * timestep);
	const ndMatrix jacobianMatrix(
		ndVector(m_mass.m_x, (m_mass.m_z - m_mass.m_y) * dw.m_z, (m_mass.m_z - m_mass.m_y) * dw.m_y, ndFloat32(0.0f)),
		ndVector((m_mass.m_x - m_mass.m_z) * dw.m_z, m_mass.m_y, (m_mass.m_x - m_mass.m_z) * dw.m_x, ndFloat32(0.0f)),
		ndVector((m_mass.m_y - m_mass.m_x) * dw.m_y, (m_mass.m_y - m_mass.m_x) * dw.m_x, m_mass.m_z, ndFloat32(0.0f)),
		ndVector::m_wOne);
	
	// and solving for alpha we get the angular acceleration at t + dt
	// calculate gradient at a full time step
	const ndVector gradientStep(jacobianMatrix.SolveByGaussianElimination(localTorque * timestep));

	velocStep.m_angular = matrix.RotateVector(m_inertiaPrincipalAxis.RotateVector(gradientStep));
	velocStep.m_linear = force.Scale(m_invMass.m_w) * timestep;

#ifdef _DEBUG
	const ndFloat32 maxLinear2 = m_maxLinearStep * m_maxLinearStep;
	const ndFloat32 maxAngular2 = m_maxAngleStep * m_maxAngleStep;

	const ndFloat32 linear2 = velocStep.m_linear.DotProduct(velocStep.m_linear).GetScalar() * timestep.m_x * timestep.m_x;
	const ndFloat32 angular2 = velocStep.m_angular.DotProduct(velocStep.m_angular).GetScalar() * timestep.m_x * timestep.m_x;
	if ((angular2 > maxAngular2) || (linear2 > maxLinear2))
	{
		ndTrace(("warning IntegrateForceAndToque %d w(%f %f %f) v(%f %f %f) with very high velocity or angular velocity, may be unstable\n", 
			m_uniqueId,
			velocStep.m_angular.m_x, velocStep.m_angular.m_y, velocStep.m_angular.m_z,
			velocStep.m_linear.m_x, velocStep.m_linear.m_y, velocStep.m_linear.m_z));
		//ndAssert(0);
	}
#endif

	return velocStep;
}

void ndBodyDynamic::IntegrateGyroSubstep(const ndVector& timestep)
{
	const ndFloat32 omegaMag2 = m_omega.DotProduct(m_omega).GetScalar();
	const ndFloat32 tol = (ndFloat32(0.0125f) * ndDegreeToRad);
	if (omegaMag2 > (tol * tol))
	{
		const ndFloat32 omegaAngle = ndSqrt(omegaMag2);
		const ndVector omegaAxis(m_omega.Scale(ndFloat32(1.0f) / omegaAngle));
		const ndQuaternion rotationStep(omegaAxis, omegaAngle * timestep.GetScalar());
		m_gyroRotation = m_gyroRotation * rotationStep;
		ndAssert((m_gyroRotation.DotProduct(m_gyroRotation).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-5f));
		
		// calculate new Gyro torque and Gyro acceleration
		const ndMatrix matrix(ndCalculateMatrix(m_gyroRotation, ndVector::m_wOne));

		const ndVector localOmega(m_inertiaPrincipalAxis.UnrotateVector(matrix.UnrotateVector(m_omega)));
		const ndVector localGyroTorque(localOmega.CrossProduct(m_mass * localOmega));
		m_gyroTorque = matrix.RotateVector(m_inertiaPrincipalAxis.RotateVector(localGyroTorque));
		m_gyroAlpha = matrix.RotateVector(m_inertiaPrincipalAxis.RotateVector(localGyroTorque * m_invMass));
	}
	else
	{
		m_gyroAlpha = ndVector::m_zero;
		m_gyroTorque = ndVector::m_zero;
	}
}

void ndBodyDynamic::EvaluateSleepState(ndFloat32 freezeSpeed2, ndFloat32 freezeAccel2)
{
	m_isJointFence0 = 1;
	if (m_isStatic)
	{
		m_equilibrium = 1;
	}
	else
	{
		ndAssert(m_accel.m_w == ndFloat32(0.0f));
		ndAssert(m_alpha.m_w == ndFloat32(0.0f));
		ndAssert(m_veloc.m_w == ndFloat32(0.0f));
		ndAssert(m_omega.m_w == ndFloat32(0.0f));
		
		ndInt32 count = ndInt32(m_weigh);
		ndAssert((!m_isConstrained && !m_weigh) || (m_isConstrained && m_weigh));

		#ifdef _DEBUG
		ndInt32 checkConnection = 0;
		for (ndJointList::ndNode* node = m_jointList.GetFirst(); node; node = node->GetNext())
		{
			checkConnection += node->GetInfo()->IsActive() ? 1 : 0;
		}

		ndContactMap::Iterator it(m_contactList);
		for (it.Begin(); it; it++)
		{
			ndContact* const contact = it.GetNode()->GetInfo();
			if (contact->IsActive() && !contact->IsTestOnly())
			{
				checkConnection++;
			}
		}
		ndAssert(count == checkConnection);
		#endif

		const ndVector maxAccNorm2 ((count > 1) ? m_sleepAccelTest2 : m_sleepAccelTest2 * m_sleepAccelTestScale2);

		const ndVector accelTest((m_accel.DotProduct(m_accel) > maxAccNorm2) | (m_alpha.DotProduct(m_alpha) > maxAccNorm2));
		m_accel = m_accel & accelTest;
		m_alpha = m_alpha & accelTest;

		ndUnsigned8 equilibrium = ndUnsigned8(m_isStatic | m_autoSleep);
		ndAssert(equilibrium == ((m_invMass.m_w == ndFloat32(0.0f)) ? 1 : m_autoSleep));
		const ndVector isMovingMask(m_veloc + m_omega + m_accel + m_alpha);
		const ndVector mask(isMovingMask.TestZero());
		const ndInt32 test = mask.GetSignMask() & 7;
		if (test != 7)
		{
			const ndFloat32 accelFreeze2 = freezeAccel2 * ((count <= 1) ? ndFloat32(0.01f) : ndFloat32(1.0f));
			const ndFloat32 accel2 = m_accel.DotProduct(m_accel).GetScalar();
			const ndFloat32 alpha2 = m_alpha.DotProduct(m_alpha).GetScalar();
			const ndFloat32 speed2 = m_veloc.DotProduct(m_veloc).GetScalar();
			const ndFloat32 omega2 = m_omega.DotProduct(m_omega).GetScalar();
			ndUnsigned32 equilibriumTest = ndUnsigned32((accel2 < accelFreeze2) && (alpha2 < accelFreeze2) && (speed2 < freezeSpeed2) && (omega2 < freezeSpeed2));

			if (equilibriumTest)
			{
				const ndFloat32 velocityDragCoeff = (count <= 1) ? D_FREEZZING_VELOCITY_DRAG : ndFloat32(0.9999f);
				const ndVector velocDragVect(velocityDragCoeff, velocityDragCoeff, velocityDragCoeff, ndFloat32(0.0f));
				const ndVector veloc(m_veloc * velocDragVect);
				const ndVector omega(m_omega * velocDragVect);
				const ndVector velocMask(veloc.DotProduct(veloc) > m_velocTol);
				const ndVector omegaMask(omega.DotProduct(omega) > m_velocTol);
				m_veloc = velocMask & veloc;
				m_omega = omegaMask & omega;
			}
			equilibrium &= equilibriumTest;
		}
		m_isJointFence0 = equilibrium;
		if (equilibrium & ~m_isConstrained)
		{
			m_equilibrium = equilibrium;
		}
	}
}

void ndBodyDynamic::InitSurrogateBody(ndBodyKinematic* const surrogate) const
{
	ndBodyKinematic::InitSurrogateBody(surrogate);
	ndBodyDynamic* const dst = ((ndBodyDynamic*)surrogate)->GetAsBodyDynamic();
	
	dst->m_externalForce = m_externalForce;
	dst->m_externalTorque = m_externalTorque;
	dst->m_impulseForce = m_impulseForce;
	dst->m_impulseTorque = m_impulseTorque;
	dst->m_savedExternalForce = m_savedExternalForce;
	dst->m_savedExternalTorque = m_savedExternalTorque;
}