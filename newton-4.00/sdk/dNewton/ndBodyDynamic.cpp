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

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndBodyDynamic.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndBodyDynamic)

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
	,m_dampCoef(dVector::m_zero)
	,m_cachedDampCoef(dVector::m_zero)
	,m_cachedTimeStep(dFloat32 (0.0f))
{
}

ndBodyDynamic::ndBodyDynamic(const dLoadSaveBase::dLoadDescriptor& desc)
	:ndBodyKinematic(dLoadSaveBase::dLoadDescriptor(desc))
	,m_accel(dVector::m_zero)
	,m_alpha(dVector::m_zero)
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

	m_dampCoef = xmlGetVector3(xmlNode, "angularDampCoef");
	m_dampCoef.m_w = xmlGetFloat(xmlNode, "linearDampCoef");
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

	m_externalForce += m_impulseForce;
	m_externalTorque += m_impulseTorque;
	m_impulseForce = dVector::m_zero;
	m_impulseTorque = dVector::m_zero;
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
		const dFloat32 tau = dFloat32(60.0f) * timestep;
		m_cachedDampCoef.m_x = dPow(dFloat32(1.0f) - m_dampCoef.m_x, tau);
		m_cachedDampCoef.m_y = dPow(dFloat32(1.0f) - m_dampCoef.m_y, tau);
		m_cachedDampCoef.m_z = dPow(dFloat32(1.0f) - m_dampCoef.m_z, tau);
		m_cachedDampCoef.m_w = dPow(dFloat32(1.0f) - m_dampCoef.m_w, tau);
	}
	dVector damp (m_cachedDampCoef);

	//dVector damp(GetDampCoeffcient(timestep));
	dVector omegaDamp(damp & dVector::m_triplexMask);
	dVector omega(m_matrix.UnrotateVector(m_omega) * omegaDamp);
	
	m_veloc = m_veloc.Scale(damp.m_w);
	m_omega = m_matrix.RotateVector(omega);
}

void ndBodyDynamic::IntegrateVelocity(dFloat32 timestep)
{
	ndBodyKinematic::IntegrateVelocity(timestep);
	m_savedExternalForce = m_externalForce;
	m_savedExternalTorque = m_externalTorque;
}

ndJacobian ndBodyDynamic::IntegrateForceAndToque(const dVector& force, const dVector& torque, const dVector& timestep) const
{
	ndJacobian velocStep;

	//dVector dtHalf(timestep * dVector::m_half);
	const dVector dtHalf(timestep);
	const dMatrix matrix(m_gyroRotation, dVector::m_wOne);
	
	const dVector localOmega(matrix.UnrotateVector(m_omega));
	const dVector localTorque(matrix.UnrotateVector(torque - m_gyroTorque));
	
	// derivative at half time step. (similar to midpoint Euler so that it does not loses too much energy)
	const dVector dw(localOmega * dtHalf);
	const dMatrix jacobianMatrix(
		dVector(m_mass[0], (m_mass[2] - m_mass[1]) * dw[2], (m_mass[2] - m_mass[1]) * dw[1], dFloat32(0.0f)),
		dVector((m_mass[0] - m_mass[2]) * dw[2], m_mass[1], (m_mass[0] - m_mass[2]) * dw[0], dFloat32(1.0f)),
		dVector((m_mass[1] - m_mass[0]) * dw[1], (m_mass[1] - m_mass[0]) * dw[0], m_mass[2], dFloat32(1.0f)),
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

void ndBodyDynamic::Save(const dLoadSaveBase::dSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndBodyKinematic::Save(dLoadSaveBase::dSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "linearDampCoef", m_dampCoef.m_w);
	xmlSaveParam(childNode, "angularDampCoef", m_dampCoef);
}