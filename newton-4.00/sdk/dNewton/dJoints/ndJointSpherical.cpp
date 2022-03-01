/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndJointSpherical.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointSpherical)

ndJointSpherical::ndJointSpherical(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(8, child, parent, pinAndPivotFrame)
	,m_rotation(dGetIdentityMatrix())
	,m_springK(ndFloat32(0.0f))
	,m_damperC(ndFloat32(0.0f))
	,m_maxConeAngle(ndFloat32(1.0e10f))
	,m_minTwistAngle(-ndFloat32(1.0e10f))
	,m_maxTwistAngle(ndFloat32(1.0e10f))
	,m_springDamperRegularizer(ndFloat32(0.0f))
{
	//m_rotation = dPitchMatrix(90.0f * ndDegreeToRad);
	//m_rotation = dYawMatrix(60.0f * ndDegreeToRad);
	//m_rotation = dRollMatrix(60.0f * ndDegreeToRad);
}

ndJointSpherical::ndJointSpherical(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointBilateralConstraint(ndLoadSaveBase::ndLoadDescriptor(desc))
	,m_rotation(dGetIdentityMatrix())
	,m_springK(ndFloat32(0.0f))
	,m_damperC(ndFloat32(0.0f))
	,m_maxConeAngle(ndFloat32(1.0e10f))
	,m_minTwistAngle(-ndFloat32(1.0e10f))
	,m_maxTwistAngle(ndFloat32(1.0e10f))
	,m_springDamperRegularizer(ndFloat32(0.0f))
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;

	m_rotation = xmlGetMatrix(xmlNode, "rotation");
	m_springK = xmlGetFloat(xmlNode, "springK");
	m_damperC = xmlGetFloat(xmlNode, "damperC");
	m_maxConeAngle = xmlGetFloat(xmlNode, "maxConeAngle");
	m_minTwistAngle = xmlGetFloat(xmlNode, "minTwistAngle");
	m_maxTwistAngle = xmlGetFloat(xmlNode, "maxTwistAngle");
	m_springDamperRegularizer = xmlGetFloat(xmlNode, "springDamperRegularizer");
}

ndJointSpherical::~ndJointSpherical()
{
}

void ndJointSpherical::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "rotation", m_rotation);
	xmlSaveParam(childNode, "springK", m_springK);
	xmlSaveParam(childNode, "damperC", m_damperC);
	xmlSaveParam(childNode, "maxConeAngle", m_maxConeAngle);
	xmlSaveParam(childNode, "minTwistAngle", m_minTwistAngle);
	xmlSaveParam(childNode, "maxTwistAngle", m_maxTwistAngle);
	xmlSaveParam(childNode, "springDamperRegularizer", m_springDamperRegularizer);
}

ndFloat32 ndJointSpherical::PenetrationOmega(ndFloat32 penetration) const
{
	ndFloat32 param = dClamp(penetration, ndFloat32(0.0f), D_MAX_SPHERICAL_PENETRATION) / D_MAX_SPHERICAL_PENETRATION;
	ndFloat32 omega = D_MAX_SPHERICAL_RECOVERY_SPEED * param;
	return omega;
}

ndMatrix ndJointSpherical::GetOffsetRotation() const
{
	return m_rotation;
}

void ndJointSpherical::SetOffsetRotation(const ndMatrix& rotation)
{
	m_rotation = rotation;
}

void ndJointSpherical::SetAsSpringDamper(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper)
{
	m_springK = dAbs(spring);
	m_damperC = dAbs(damper);
	m_springDamperRegularizer = dClamp(regularizer, ndFloat32(1.0e-2f), ndFloat32(0.99f));
}

void ndJointSpherical::GetSpringDamper(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const
{
	spring = m_springK;
	damper = m_damperC;
	regularizer = m_springDamperRegularizer;
}

void ndJointSpherical::SetTwistLimits(ndFloat32 minAngle, ndFloat32 maxAngle)
{
	m_minTwistAngle = dMin(minAngle, ndFloat32 (0.0f));
	m_maxTwistAngle = dMax(maxAngle, ndFloat32(0.0f));
}

void ndJointSpherical::GetTwistLimits(ndFloat32& minAngle, ndFloat32& maxAngle) const
{
	minAngle = m_minTwistAngle;
	maxAngle = m_maxTwistAngle;
}

ndFloat32 ndJointSpherical::GetConeLimit() const
{
	return m_maxConeAngle;
}

void ndJointSpherical::SetConeLimit(ndFloat32 maxConeAngle)
{
	//m_maxConeAngle = dClamp (maxConeAngle, ndFloat32 (0.0f), D_BALL_AND_SOCKED_MAX_ANGLE);
	m_maxConeAngle = dClamp(maxConeAngle, ndFloat32(0.0f), ndFloat32(1.0e10f));
}

void ndJointSpherical::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	debugCallback.DrawFrame(matrix0);
	debugCallback.DrawFrame(matrix1);

	const ndInt32 subdiv = 8;
	const ndVector& coneDir0 = matrix0.m_front;
	const ndVector& coneDir1 = matrix1.m_front;
	ndFloat32 cosAngleCos = coneDir0.DotProduct(coneDir1).GetScalar();
	ndMatrix coneRotation(dGetIdentityMatrix());
	if (cosAngleCos < ndFloat32(0.9999f))
	{
		ndVector lateralDir(coneDir1.CrossProduct(coneDir0));
		ndFloat32 mag2 = lateralDir.DotProduct(lateralDir).GetScalar();
		if (mag2 > ndFloat32 (1.0e-4f)) 
		{
			lateralDir = lateralDir.Scale(ndFloat32 (1.0f) / ndSqrt(mag2));
			coneRotation = ndMatrix(ndQuaternion(lateralDir, ndAcos(dClamp(cosAngleCos, ndFloat32(-1.0f), ndFloat32(1.0f)))), matrix1.m_posit);
		}
		else 
		{
			lateralDir = matrix0.m_up.Scale(-ndFloat32 (1.0f));
			coneRotation = ndMatrix(ndQuaternion(matrix0.m_up, ndFloat32 (180.0f) * ndDegreeToRad), matrix1.m_posit);
		}
	}
	else if (cosAngleCos < -ndFloat32 (0.9999f)) 
	{
		coneRotation[0][0] = ndFloat32(-1.0f);
		coneRotation[1][1] = ndFloat32(-1.0f);
	}
	
	const ndFloat32 radius = debugCallback.m_debugScale;
	ndVector arch[subdiv + 1];
	
	// show twist angle limits
	ndFloat32 deltaTwist = m_maxTwistAngle - m_minTwistAngle;
	if ((deltaTwist > ndFloat32(1.0e-3f)) && (deltaTwist < ndFloat32 (2.0f) * ndPi))
	{ 
		ndMatrix pitchMatrix(matrix1 * coneRotation);
		pitchMatrix.m_posit = matrix1.m_posit;
	
		ndVector point(ndFloat32(0.0f), ndFloat32(radius), ndFloat32(0.0f), ndFloat32(0.0f));
	
		ndFloat32 angleStep = dMin(m_maxTwistAngle - m_minTwistAngle, ndFloat32(2.0f * ndPi)) / subdiv;
		ndFloat32 angle0 = m_minTwistAngle;
	
		ndVector color(ndFloat32 (0.4f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
		for (ndInt32 i = 0; i <= subdiv; i++) 
		{
			arch[i] = pitchMatrix.TransformVector(dPitchMatrix(angle0).RotateVector(point));
			debugCallback.DrawLine(pitchMatrix.m_posit, arch[i], color);
			angle0 += angleStep;
		}
	
		for (ndInt32 i = 0; i < subdiv; i++) 
		{
			debugCallback.DrawLine(arch[i], arch[i + 1], color);
		}
	}
	
	// show cone angle limits
	if ((m_maxConeAngle > ndFloat32 (0.0f)) && (m_maxConeAngle < D_MAX_SPHERICAL_CONE_ANGLE))
	{
		ndVector color(ndFloat32(0.3f), ndFloat32(0.8f), ndFloat32(0.0f), ndFloat32(0.0f));
		ndVector point(radius * ndCos(m_maxConeAngle), radius * ndSin(m_maxConeAngle), ndFloat32 (0.0f), ndFloat32(0.0f));
		ndFloat32 angleStep = ndPi * ndFloat32(2.0f) / subdiv;
	
		ndFloat32 angle0 = ndFloat32 (0.0f);
		for (ndInt32 i = 0; i <= subdiv; i++) 
		{
			ndVector conePoint(dPitchMatrix(angle0).RotateVector(point));
			ndVector p(matrix1.TransformVector(conePoint));
			arch[i] = p;
			debugCallback.DrawLine(matrix1.m_posit, p, color);
			angle0 += angleStep;
		}
	
		for (ndInt32 i = 0; i < subdiv; i++) 
		{
			debugCallback.DrawLine(arch[i], arch[i + 1], color);
		}
	}
}

ndInt8 ndJointSpherical::SubmitTwistAngle(const ndVector& pin, ndFloat32 angle, ndConstraintDescritor& desc)
{
	ndInt8 ret = false;
	if ((m_maxTwistAngle - m_minTwistAngle) < (2.0f * ndDegreeToRad))
	{
		AddAngularRowJacobian(desc, pin, -angle);
		SetLowerFriction(desc, -D_LCP_MAX_VALUE * ndFloat32(0.1f));
		SetHighFriction(desc, D_LCP_MAX_VALUE * ndFloat32(0.1f));
	}
	else
	{
		if (angle < m_minTwistAngle)
		{
			AddAngularRowJacobian(desc, pin, ndFloat32(0.0f));
			const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
			const ndFloat32 penetration = angle - m_minTwistAngle;
			const ndFloat32 recoveringAccel = -desc.m_invTimestep * PenetrationOmega(-penetration);
			SetMotorAcceleration(desc, stopAccel - recoveringAccel);
			SetLowerFriction(desc, ndFloat32(0.0f));
			ret = dAbs(stopAccel) > ND_MAX_STOP_ACCEL;
		}
		else if (angle >= m_maxTwistAngle)
		{
			AddAngularRowJacobian(desc, pin, ndFloat32(0.0f));
			const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
			const ndFloat32 penetration = angle - m_maxTwistAngle;
			const ndFloat32 recoveringAccel = desc.m_invTimestep * PenetrationOmega(penetration);
			SetMotorAcceleration(desc, stopAccel - recoveringAccel);
			SetHighFriction(desc, ndFloat32(0.0f));
			ret = dAbs(stopAccel) > ND_MAX_STOP_ACCEL;
		}
	}
	return ret;
}

ndInt8 ndJointSpherical::SubmitAngularAxisCartesianApproximation(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	if (m_maxConeAngle < (ndFloat32 (1.0f) * ndDegreeToRad))
	{
		// two rows to restrict rotation around around the parent coordinate system
		ndFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
		AddAngularRowJacobian(desc, matrix1.m_up, angle0);
		SetLowerFriction(desc, -D_LCP_MAX_VALUE * ndFloat32(0.1f));
		SetHighFriction(desc, D_LCP_MAX_VALUE * ndFloat32(0.1f));

		ndFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
		AddAngularRowJacobian(desc, matrix1.m_right, angle1);
		SetLowerFriction(desc, -D_LCP_MAX_VALUE * ndFloat32(0.1f));
		SetHighFriction(desc, D_LCP_MAX_VALUE * ndFloat32(0.1f));
	}

	ndFloat32 pitchAngle = -CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);
	return SubmitTwistAngle(matrix0.m_front, pitchAngle, desc);
}

ndInt8 ndJointSpherical::SubmitAngularAxis(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	ndInt8 ret = false;
	if (m_maxConeAngle < D_MAX_SPHERICAL_CONE_ANGLE)
	{
		ndVector lateralDir(matrix1[0].CrossProduct(matrix0[0]));
		dAssert(lateralDir.DotProduct(lateralDir).GetScalar() > 1.0e-6f);
		lateralDir = lateralDir.Normalize();
		const ndFloat32 coneAngle = ndAcos(dClamp(matrix1.m_front.DotProduct(matrix0.m_front).GetScalar(), ndFloat32(-1.0f), ndFloat32(1.0f)));
		const ndMatrix coneRotation(ndQuaternion(lateralDir, coneAngle), matrix1.m_posit);
		if (coneAngle > m_maxConeAngle)
		{
			if (m_maxConeAngle > (ndFloat32(1.0f) * ndDegreeToRad))
			{
				AddAngularRowJacobian(desc, lateralDir, ndFloat32(0.0f));
				const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
				const ndFloat32 penetration = coneAngle - m_maxConeAngle;
				const ndFloat32 recoveringAccel = desc.m_invTimestep * PenetrationOmega(penetration);
				SetMotorAcceleration(desc, stopAccel - recoveringAccel);
				SetHighFriction(desc, ndFloat32(0.0f));
				ret = dAbs(stopAccel) > ND_MAX_STOP_ACCEL;
			}
			else
			{
				// two rows to restrict rotation around around the parent coordinate system
				ndFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
				AddAngularRowJacobian(desc, matrix1.m_up, angle0);
				SetLowerFriction(desc, -D_LCP_MAX_VALUE * ndFloat32(0.1f));
				SetHighFriction(desc, D_LCP_MAX_VALUE * ndFloat32(0.1f));

				ndFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
				AddAngularRowJacobian(desc, matrix1.m_right, angle1);
				SetLowerFriction(desc, -D_LCP_MAX_VALUE * ndFloat32(0.1f));
				SetHighFriction(desc, D_LCP_MAX_VALUE * ndFloat32(0.1f));
			}
		}

		const ndMatrix pitchMatrix(matrix1 * coneRotation * matrix0.Inverse());
		const ndFloat32 pitchAngle = -ndAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
		ndInt8 ret1 = SubmitTwistAngle(matrix0.m_front, pitchAngle, desc);
		ret = ret1 | ret;
	}
	return ret;
}

void ndJointSpherical::ApplyBaseRows(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[0]);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[1]);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[2]);
}

void ndJointSpherical::SubmitSpringDamper(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	if (m_springK == ndFloat32(0.0f))
	{
		dAssert(m_damperC > ndFloat32(0.0f));
		const ndBodyKinematic* const body0 = GetBody0();
		const ndBodyKinematic* const body1 = GetBody1();
		const ndVector omega0(body0->GetOmega());
		const ndVector omega1(body1->GetOmega());
		const ndVector relOmega(omega1 - omega0);
		ndFloat32 speed(relOmega.DotProduct(relOmega).GetScalar());
		if (speed > ndFloat32(1.0e-5f))
		{
			const ndVector pin (relOmega.Normalize());
			AddAngularRowJacobian(desc, pin, ndFloat32(0.0f));
			SetMassSpringDamperAcceleration(desc, m_springDamperRegularizer, ndFloat32(0.0f), m_damperC);
		}
	}
	else
	{
		const ndMatrix matrix11(m_rotation * matrix1);
		const ndQuaternion rotation(matrix0.Inverse() * matrix11);
		const ndVector pin(rotation & ndVector::m_triplexMask);
		const ndFloat32 dirMag2 = pin.DotProduct(pin).GetScalar();
		//if (dirMag2 > ndFloat32(ndFloat32(1.0e-7f)))
		const ndFloat32 tol = ndFloat32(1.0e-3f);
		if (dirMag2 > (tol * tol))
		{
			const ndMatrix basis(pin.Normalize());
			const ndFloat32 dirMag = ndSqrt(dirMag2);
			const ndFloat32 angle = ndFloat32(2.0f) * ndAtan2(dirMag, rotation.m_w);

			AddAngularRowJacobian(desc, basis[0], angle);
			SetMassSpringDamperAcceleration(desc, m_springDamperRegularizer, m_springK, m_damperC);
			AddAngularRowJacobian(desc, basis[1], ndFloat32(0.0f));
			AddAngularRowJacobian(desc, basis[2], ndFloat32(0.0f));
		}
		else
		{
			const ndFloat32 pitchAngle = CalculateAngle(matrix0[1], matrix11[1], matrix11[0]);
			AddAngularRowJacobian(desc, matrix11[0], pitchAngle);
			SetMassSpringDamperAcceleration(desc, m_springDamperRegularizer, m_springK, m_damperC);
			
			const ndFloat32 yawAngle = CalculateAngle(matrix0[0], matrix11[0], matrix11[1]);
			AddAngularRowJacobian(desc, matrix11[1], yawAngle);
			SetMassSpringDamperAcceleration(desc, m_springDamperRegularizer, m_springK, m_damperC);
			
			const ndFloat32 rollAngle = CalculateAngle(matrix0[0], matrix11[0], matrix11[2]);
			AddAngularRowJacobian(desc, matrix11[2], rollAngle);
			SetMassSpringDamperAcceleration(desc, m_springDamperRegularizer, m_springK, m_damperC);
		}
	}
}

ndInt8 ndJointSpherical::SubmitLimits(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	ndInt8 ret = false;
	ndFloat32 cosAngleCos = matrix1.m_front.DotProduct(matrix0.m_front).GetScalar();
	if (cosAngleCos >= ndFloat32(0.998f))
	{
		// special case where the front axis are almost aligned
		// solve by using Cartesian approximation
		ret = SubmitAngularAxisCartesianApproximation(matrix0, matrix1, desc);
	}
	else
	{
		ret = SubmitAngularAxis(matrix0, matrix1, desc);
	}
	return ret;
}

void ndJointSpherical::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);
	ApplyBaseRows(matrix0, matrix1, desc);
	ndInt8 hitLimit = SubmitLimits(matrix0, matrix1, desc);
	if (!hitLimit)
	{
		if ((m_springK > ndFloat32(0.0f)) || (m_damperC > ndFloat32(0.0f)))
		{
			SubmitSpringDamper(matrix0, matrix1, desc);
		}
	}
}