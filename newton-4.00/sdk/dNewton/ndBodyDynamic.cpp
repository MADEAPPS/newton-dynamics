/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndBodyDynamic)

ndBodyDynamic::ndBodyDynamic()
	:ndBodyKinematic()
	,m_externalForce(dVector::m_zero)
	,m_externalTorque(dVector::m_zero)
	,m_impulseForce(dVector::m_zero)
	,m_impulseTorque(dVector::m_zero)
	,m_savedExternalForce(dVector::m_zero)
	,m_savedExternalTorque(dVector::m_zero)
	,m_dampCoef(dVector::m_zero)
	,m_cachedDampCoef(dVector::m_zero)
	,m_cachedTimeStep(dFloat32 (0.0f))
{
	m_isDynamics = 1;
}

ndBodyDynamic::ndBodyDynamic(const dLoadSaveBase::dLoadDescriptor& desc)
	:ndBodyKinematic(dLoadSaveBase::dLoadDescriptor(desc))
	,m_externalForce(dVector::m_zero)
	,m_externalTorque(dVector::m_zero)
	,m_impulseForce(dVector::m_zero)
	,m_impulseTorque(dVector::m_zero)
	,m_savedExternalForce(dVector::m_zero)
	,m_savedExternalTorque(dVector::m_zero)
	,m_dampCoef(dVector::m_zero)
	,m_cachedDampCoef(dVector::m_zero)
	,m_cachedTimeStep(dFloat32(0.0f))
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;

	m_isDynamics = 1;
	m_dampCoef = xmlGetVector3(xmlNode, "angularDampCoef");
	m_dampCoef.m_w = xmlGetFloat(xmlNode, "linearDampCoef");
}

ndBodyDynamic::~ndBodyDynamic()
{
}

void ndBodyDynamic::Save(const dLoadSaveBase::dSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndBodyKinematic::Save(dLoadSaveBase::dSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "linearDampCoef", m_dampCoef.m_w);
	xmlSaveParam(childNode, "angularDampCoef", m_dampCoef);
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

	m_externalForce += m_impulseForce;
	m_externalTorque += m_impulseTorque;
	m_impulseForce = dVector::m_zero;
	m_impulseTorque = dVector::m_zero;
}

void ndBodyDynamic::AddImpulse(const dVector& pointDeltaVeloc, const dVector& pointPosit, dFloat32 timestep)
{
	dMatrix invInertia(CalculateInvInertiaMatrix());

	// get contact matrix
	dMatrix tmp;
	dVector globalContact(pointPosit - m_globalCentreOfMass);

	tmp[0][0] = dFloat32(0.0f);
	tmp[0][1] = +globalContact[2];
	tmp[0][2] = -globalContact[1];
	tmp[0][3] = dFloat32(0.0f);

	tmp[1][0] = -globalContact[2];
	tmp[1][1] = dFloat32(0.0f);
	tmp[1][2] = +globalContact[0];
	tmp[1][3] = dFloat32(0.0f);

	tmp[2][0] = +globalContact[1];
	tmp[2][1] = -globalContact[0];
	tmp[2][2] = dFloat32(0.0f);
	tmp[2][3] = dFloat32(0.0f);

	tmp[3][0] = dFloat32(0.0f);
	tmp[3][1] = dFloat32(0.0f);
	tmp[3][2] = dFloat32(0.0f);
	tmp[3][3] = dFloat32(1.0f);

	dMatrix contactMatrix(tmp * invInertia * tmp);
	//for (dInt32 i = 0; i < 3; i++) 
	//{
	//	for (dInt32 j = 0; j < 3; j++) 
	//	{
	//		contactMatrix[i][j] *= -dFloat32(1.0f);
	//	}
	//}
	contactMatrix[0] = contactMatrix[0] * dVector::m_negOne;
	contactMatrix[1] = contactMatrix[1] * dVector::m_negOne;
	contactMatrix[2] = contactMatrix[2] * dVector::m_negOne;
	contactMatrix[0][0] += m_invMass.m_w;
	contactMatrix[1][1] += m_invMass.m_w;
	contactMatrix[2][2] += m_invMass.m_w;

	contactMatrix = contactMatrix.Inverse4x4();

	// change of momentum
	dVector changeOfMomentum(contactMatrix.RotateVector(pointDeltaVeloc));

	if (changeOfMomentum.DotProduct(changeOfMomentum).GetScalar() > dFloat32(1.0e-6f))
	{
		m_impulseForce += changeOfMomentum.Scale(1.0f / timestep);
		m_impulseTorque += globalContact.CrossProduct(m_impulseForce);

		m_equilibrium = false;
		//Unfreeze();
	}
}

void ndBodyDynamic::ApplyImpulsePair(const dVector& linearImpulse, const dVector& angularImpulse, dFloat32 timestep)
{
	dAssert(linearImpulse.m_w == dFloat32(0.0f));
	dAssert(angularImpulse.m_w == dFloat32(0.0f));
	if ((linearImpulse.DotProduct(linearImpulse).GetScalar() > dFloat32(1.0e-6f)) ||
		(angularImpulse.DotProduct(angularImpulse).GetScalar() > dFloat32(1.0e-6f))) 
	{
		m_impulseForce += linearImpulse.Scale(1.0f / timestep);
		m_impulseTorque += angularImpulse.Scale(1.0f / timestep);

		m_equilibrium = false;
	}
}

void ndBodyDynamic::ApplyImpulsesAtPoint(dInt32 count, const dVector* const impulseArray, const dVector* const pointArray, dFloat32 timestep)
{
	dVector impulse(dVector::m_zero);
	dVector angularImpulse(dVector::m_zero);

	dVector com(m_globalCentreOfMass);
	for (dInt32 i = 0; i < count; i++) 
	{
		dVector r(pointArray[i]);
		dVector L(impulseArray[i]);
		dVector Q((r - com).CrossProduct(L));

		impulse += L;
		angularImpulse += Q;
	}

	impulse = impulse & dVector::m_triplexMask;
	angularImpulse = angularImpulse & dVector::m_triplexMask;

	if ((impulse.DotProduct(impulse).GetScalar() > dFloat32(1.0e-6f)) ||
		(angularImpulse.DotProduct(angularImpulse).GetScalar() > dFloat32(1.0e-6f))) 
	{
		m_impulseForce += impulse.Scale(1.0f / timestep);
		m_impulseTorque += angularImpulse.Scale(1.0f / timestep);

		m_equilibrium = false;
	}
}

void ndBodyDynamic::SetLinearDamping(dFloat32 linearDamp)
{
	linearDamp = dClamp(linearDamp, dFloat32(0.0f), dFloat32(1.0f));
	m_dampCoef.m_w = D_MAX_SPEED_ATT * linearDamp;
	m_cachedTimeStep = dFloat32(0.0f);
}

dFloat32 ndBodyDynamic::GetLinearDamping() const
{
	return m_dampCoef.m_w / D_MAX_SPEED_ATT;
}

dVector ndBodyDynamic::GetAngularDamping() const
{
	return dVector(m_dampCoef.m_x / D_MAX_SPEED_ATT,
				   m_dampCoef.m_y / D_MAX_SPEED_ATT,
				   m_dampCoef.m_z / D_MAX_SPEED_ATT, dFloat32(0.0f));
}


void ndBodyDynamic::SetAngularDamping(const dVector& angularDamp)
{
	dFloat32 tmp = dClamp(angularDamp.m_x, dFloat32(0.0f), dFloat32(1.0f));
	m_dampCoef.m_x = D_MAX_SPEED_ATT * tmp;

	tmp = dClamp(angularDamp.m_y, dFloat32(0.0f), dFloat32(1.0f));
	m_dampCoef.m_y = D_MAX_SPEED_ATT * tmp;

	tmp = dClamp(angularDamp.m_z, dFloat32(0.0f), dFloat32(1.0f));
	m_dampCoef.m_z = D_MAX_SPEED_ATT * tmp;

	m_cachedTimeStep = dFloat32(0.0f);
}

void ndBodyDynamic::AddDampingAcceleration(dFloat32 timestep)
{
	if (dAbs(m_cachedTimeStep - timestep) > dFloat32(1.0e-6f)) 
	{
		m_cachedTimeStep = timestep;
		// assume a nominal 60 frame seconds time step.
		dFloat32 tau = dFloat32(60.0f) * timestep;
		// recalculate damping to match the time independent drag
		m_cachedDampCoef.m_x = dPow(dFloat32(1.0f) - m_dampCoef.m_x, tau);
		m_cachedDampCoef.m_y = dPow(dFloat32(1.0f) - m_dampCoef.m_y, tau);
		m_cachedDampCoef.m_z = dPow(dFloat32(1.0f) - m_dampCoef.m_z, tau);
		m_cachedDampCoef.m_w = dPow(dFloat32(1.0f) - m_dampCoef.m_w, tau);
	}

	const dVector omegaDamp(m_cachedDampCoef & dVector::m_triplexMask);
	const dVector omega(m_matrix.UnrotateVector(m_omega) * omegaDamp);
	m_omega = m_matrix.RotateVector(omega);
	m_veloc = m_veloc.Scale(m_cachedDampCoef.m_w);
}

void ndBodyDynamic::IntegrateVelocity(dFloat32 timestep)
{
	ndBodyKinematic::IntegrateVelocity(timestep);
	SaveExternalForces();
}

ndJacobian ndBodyDynamic::IntegrateForceAndToque(const dVector& force, const dVector& torque, const dVector& timestep) const
{
	ndJacobian velocStep;
	const dMatrix matrix(m_gyroRotation, dVector::m_wOne);
	const dVector localOmega(matrix.UnrotateVector(m_omega));
	const dVector localTorque(matrix.UnrotateVector(torque));
	
	// derivative at half time step. (similar to midpoint Euler so that it does not loses too much energy)
	const dVector dw(localOmega * timestep);
	const dMatrix jacobianMatrix(
		dVector(m_mass.m_x, (m_mass.m_z - m_mass.m_y) * dw.m_z, (m_mass.m_z - m_mass.m_y) * dw.m_y, dFloat32(0.0f)),
		dVector((m_mass.m_x - m_mass.m_z) * dw.m_z, m_mass.m_y, (m_mass.m_x - m_mass.m_z) * dw.m_x, dFloat32(0.0f)),
		dVector((m_mass.m_y - m_mass.m_x) * dw.m_y, (m_mass.m_y - m_mass.m_x) * dw.m_x, m_mass.m_z, dFloat32(0.0f)),
		dVector::m_wOne);
	
	// and solving for alpha we get the angular acceleration at t + dt
	// calculate gradient at a full time step
	const dVector gradientStep(jacobianMatrix.SolveByGaussianElimination(localTorque * timestep));

	velocStep.m_angular = matrix.RotateVector(gradientStep);
	velocStep.m_linear = force.Scale(m_invMass.m_w) * timestep;
	return velocStep;
}

void ndBodyDynamic::IntegrateGyroSubstep(const dVector& timestep)
{
	const dFloat32 omegaMag2 = m_omega.DotProduct(m_omega).GetScalar() + dFloat32(1.0e-12f);
	const dFloat32 tol = (dFloat32(0.0125f) * dDegreeToRad);
	if (omegaMag2 > (tol * tol))
	{
		// calculate new matrix
		const dFloat32 invOmegaMag = dRsqrt(omegaMag2);
		const dVector omegaAxis(m_omega.Scale(invOmegaMag));
		dFloat32 omegaAngle = invOmegaMag * omegaMag2 * timestep.GetScalar();
		const dQuaternion rotationStep(omegaAxis, omegaAngle);
		m_gyroRotation = m_gyroRotation * rotationStep;
		dAssert((m_gyroRotation.DotProduct(m_gyroRotation).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-5f));
		
		// calculate new Gyro torque and Gyro acceleration
		const dMatrix matrix(m_gyroRotation, dVector::m_wOne);

		const dVector localOmega(matrix.UnrotateVector(m_omega));
		const dVector localGyroTorque(localOmega.CrossProduct(m_mass * localOmega));
		m_gyroTorque = matrix.RotateVector(localGyroTorque);
		m_gyroAlpha = matrix.RotateVector(localGyroTorque * m_invMass);
	}
	else
	{
		m_gyroAlpha = dVector::m_zero;
		m_gyroTorque = dVector::m_zero;
	}
}
