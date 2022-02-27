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
#include "ndJointDoubleHinge.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointDoubleHinge)

ndJointDoubleHinge::ndAxisParam::ndAxisParam()
	:m_angle(ndFloat32(0.0f))
	,m_omega(ndFloat32(0.0f))
	,m_springK(ndFloat32(0.0f))
	,m_damperC(ndFloat32(0.0f))
	,m_minLimit(ndFloat32(-1.0e10f))
	,m_maxLimit(ndFloat32(1.0e10f))
	,m_offsetAngle(ndFloat32(0.0f))
	,m_springDamperRegularizer(ndFloat32(0.1f))
{
}

void ndJointDoubleHinge::ndAxisParam::Load(const nd::TiXmlElement* const xmlNode)
{
	m_angle = xmlGetFloat(xmlNode, "angle");
	m_omega = xmlGetFloat(xmlNode, "omega");
	m_springK = xmlGetFloat(xmlNode, "springK");
	m_damperC = xmlGetFloat(xmlNode, "damperC");
	m_minLimit = xmlGetFloat(xmlNode, "minLimit");
	m_maxLimit = xmlGetFloat(xmlNode, "maxLimit");
	m_offsetAngle = xmlGetFloat(xmlNode, "offsetAngle");
	m_springDamperRegularizer = xmlGetFloat(xmlNode, "springDamperRegularizer");
}

void ndJointDoubleHinge::ndAxisParam::Save(nd::TiXmlElement* const xmlNode) const
{
	xmlSaveParam(xmlNode, "angle", m_angle);
	xmlSaveParam(xmlNode, "omega", m_omega);
	xmlSaveParam(xmlNode, "springK", m_springK);
	xmlSaveParam(xmlNode, "damperC", m_damperC);
	xmlSaveParam(xmlNode, "minLimit", m_minLimit);
	xmlSaveParam(xmlNode, "maxLimit", m_maxLimit);
	xmlSaveParam(xmlNode, "offsetAngle", m_offsetAngle);
	xmlSaveParam(xmlNode, "springDamperRegularizer", m_springDamperRegularizer);
}

ndJointDoubleHinge::ndJointDoubleHinge(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(8, child, parent, pinAndPivotFrame)
	,m_axis0()
	,m_axis1()
{
}

ndJointDoubleHinge::ndJointDoubleHinge(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointBilateralConstraint(ndLoadSaveBase::ndLoadDescriptor(desc))
	,m_axis0()
	,m_axis1()
{
	dAssert(m_maxDof == 8);
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;

	const nd::TiXmlNode* const axis0 = xmlFind(xmlNode, "axis0");
	m_axis0.Load((nd::TiXmlElement*)axis0);

	const nd::TiXmlNode* const axis1 = xmlFind(xmlNode, "axis1");
	m_axis1.Load((nd::TiXmlElement*)axis1);
}

ndJointDoubleHinge::~ndJointDoubleHinge()
{
}

void ndJointDoubleHinge::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	nd::TiXmlElement* const axis0 = new nd::TiXmlElement("axis0");
	childNode->LinkEndChild(axis0);
	m_axis0.Save(axis0);

	nd::TiXmlElement* const axis1 = new nd::TiXmlElement("axis1");
	childNode->LinkEndChild(axis1);
	m_axis0.Save(axis1);

}

ndFloat32 ndJointDoubleHinge::GetAngle0() const
{
	return m_axis0.m_angle;
}

ndFloat32 ndJointDoubleHinge::GetOmega0() const
{
	return m_axis0.m_omega;
}

void ndJointDoubleHinge::SetLimits0(ndFloat32 minLimit, ndFloat32 maxLimit)
{
	dAssert(minLimit <= 0.0f);
	dAssert(maxLimit >= 0.0f);
	m_axis0.m_minLimit = minLimit;
	m_axis0.m_maxLimit = maxLimit;
}

void ndJointDoubleHinge::GetLimits0(ndFloat32& minLimit, ndFloat32& maxLimit)
{
	minLimit = m_axis0.m_minLimit;
	maxLimit = m_axis0.m_maxLimit;
}

ndFloat32 ndJointDoubleHinge::GetOffsetAngle0() const
{
	return m_axis0.m_offsetAngle;
}

void ndJointDoubleHinge::SetOffsetAngle0(ndFloat32 angle)
{
	m_axis0.m_offsetAngle = angle;
}

void ndJointDoubleHinge::SetAsSpringDamper0(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper)
{
	m_axis0.m_springK = dAbs(spring);
	m_axis0.m_damperC = dAbs(damper);
	m_axis0.m_springDamperRegularizer = dClamp(regularizer, ndFloat32(1.0e-2f), ndFloat32(0.99f));
}

ndFloat32 ndJointDoubleHinge::GetAngle1() const
{
	return m_axis1.m_angle;
}

ndFloat32 ndJointDoubleHinge::GetOmega1() const
{
	return m_axis1.m_omega;
}

void ndJointDoubleHinge::SetLimits1(ndFloat32 minLimit, ndFloat32 maxLimit)
{
	dAssert(minLimit <= 0.0f);
	dAssert(maxLimit >= 0.0f);
	m_axis1.m_minLimit = minLimit;
	m_axis1.m_maxLimit = maxLimit;
}

void ndJointDoubleHinge::GetLimits1(ndFloat32& minLimit, ndFloat32& maxLimit)
{
	minLimit = m_axis1.m_minLimit;
	maxLimit = m_axis1.m_maxLimit;
}

ndFloat32 ndJointDoubleHinge::GetOffsetAngle1() const
{
	return m_axis1.m_offsetAngle;
}

void ndJointDoubleHinge::SetOffsetAngle1(ndFloat32 angle)
{
	m_axis1.m_offsetAngle = angle;
}

void ndJointDoubleHinge::SetAsSpringDamper1(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper)
{
	m_axis1.m_springK = dAbs(spring);
	m_axis1.m_damperC = dAbs(damper);
	m_axis1.m_springDamperRegularizer = dClamp(regularizer, ndFloat32(1.0e-2f), ndFloat32(0.99f));
}

void ndJointDoubleHinge::ApplyBaseRows(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1)
{
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[0]);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[1]);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[2]);

	const ndVector frontDir((matrix0.m_front - matrix1.m_up.Scale(matrix0.m_front.DotProduct(matrix1.m_up).GetScalar())).Normalize());
	const ndVector sideDir(frontDir.CrossProduct(matrix1.m_up));
	const ndFloat32 angle = CalculateAngle(matrix0.m_front, frontDir, sideDir);
	AddAngularRowJacobian(desc, sideDir, angle);
	
	// not happy with this method because it is a penalty system, 
	// but is hard for a first order integrator to prevent the side angle 
	// from drifting, even an implicit one expanding the Jacobian partial 
	// derivatives still has a hard time.
	// nullifying the gyro torque generated by the two angular velocities.
	const ndFloat32 alphaRollError = GetMotorZeroAcceleration(desc) + ndFloat32 (0.5f) * angle * desc.m_invTimestep * desc.m_invTimestep;
	SetMotorAcceleration(desc, alphaRollError);

	// save the current joint Omega
	const ndVector omega0(m_body0->GetOmega());
	const ndVector omega1(m_body1->GetOmega());
	
	// calculate joint parameters, angles and omega
	//const ndFloat32 deltaAngle0 = AnglesAdd(-CalculateAngle(matrix0.m_up, matrix1.m_up, frontDir), -m_angle0);
	const ndFloat32 deltaAngle0 = AnglesAdd(-CalculateAngle(matrix0.m_up, matrix1.m_up, frontDir), -m_axis0.m_angle);
	m_axis0.m_angle += deltaAngle0;
	m_axis0.m_omega = frontDir.DotProduct(omega0 - omega1).GetScalar();
	
	//const ndFloat32 deltaAngle1 = AnglesAdd(-CalculateAngle(frontDir, matrix1.m_front, matrix1.m_up), -m_angle1);
	const ndFloat32 deltaAngle1 = AnglesAdd(-CalculateAngle(frontDir, matrix1.m_front, matrix1.m_up), -m_axis1.m_angle);
	m_axis1.m_angle += deltaAngle1;
	m_axis1.m_omega = matrix1.m_up.DotProduct(omega0 - omega1).GetScalar();
}

ndFloat32 ndJointDoubleHinge::PenetrationOmega(ndFloat32 penetration) const
{
	ndFloat32 param = dClamp(penetration, ndFloat32(0.0f), D_MAX_DOUBLE_HINGE_PENETRATION) / D_MAX_DOUBLE_HINGE_PENETRATION;
	ndFloat32 omega = D_MAX_DOUBLE_HINGE_RECOVERY_SPEED * param;
	return omega;
}

//ndInt8 ndJointDoubleHinge::SubmitLimits(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1)
ndInt8 ndJointDoubleHinge::SubmitLimits(ndConstraintDescritor&, const ndMatrix&, const ndMatrix&)
{
	ndInt8 ret = 0;
	//if ((m_minLimit > (ndFloat32(-1.0f) * ndDegreeToRad)) && (m_maxLimit < (ndFloat32(1.0f) * ndDegreeToRad)))
	//{
	//	AddAngularRowJacobian(desc, &matrix1.m_front[0], -m_angle);
	//}
	//else
	//{
	//	const ndFloat32 angle = m_angle + m_omega * desc.m_timestep;
	//	if (angle < m_minLimit)
	//	{
	//		AddAngularRowJacobian(desc, &matrix0.m_front[0], ndFloat32(0.0f));
	//		const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
	//		const ndFloat32 penetration = angle - m_minLimit;
	//		const ndFloat32 recoveringAceel = -desc.m_invTimestep * PenetrationOmega(-penetration);
	//		SetMotorAcceleration(desc, stopAccel - recoveringAceel);
	//		SetLowerFriction(desc, ndFloat32(0.0f));
	//		ret = dAbs(stopAccel) > ND_MAX_STOP_ACCEL;
	//	}
	//	else if (angle > m_maxLimit)
	//	{
	//		AddAngularRowJacobian(desc, &matrix0.m_front[0], ndFloat32(0.0f));
	//		const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
	//		const ndFloat32 penetration = angle - m_maxLimit;
	//		const ndFloat32 recoveringAceel = desc.m_invTimestep * PenetrationOmega(penetration);
	//		SetMotorAcceleration(desc, stopAccel - recoveringAceel);
	//		SetHighFriction(desc, ndFloat32(0.0f));
	//		ret = dAbs(stopAccel) > ND_MAX_STOP_ACCEL;
	//	}
	//}
	return ret;
}

void ndJointDoubleHinge::SubmitSpringDamper0(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix&)
{
	AddAngularRowJacobian(desc, matrix0.m_front, m_axis0.m_offsetAngle - m_axis0.m_angle);
	SetMassSpringDamperAcceleration(desc, m_axis0.m_springDamperRegularizer, m_axis0.m_springK, m_axis0.m_damperC);
}

void ndJointDoubleHinge::SubmitSpringDamper1(ndConstraintDescritor& desc, const ndMatrix&, const ndMatrix& matrix1)
{
	AddAngularRowJacobian(desc, matrix1.m_up, m_axis1.m_offsetAngle - m_axis1.m_angle);
	SetMassSpringDamperAcceleration(desc, m_axis1.m_springDamperRegularizer, m_axis1.m_springK, m_axis1.m_damperC);
}

void ndJointDoubleHinge::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	ApplyBaseRows(desc, matrix0, matrix1);

	ndInt8 hitLimit = SubmitLimits(desc, matrix0, matrix1);
	if (!(hitLimit & 1))
	{
		if ((m_axis0.m_springK > ndFloat32(0.0f)) || (m_axis0.m_damperC > ndFloat32(0.0f)))
		{
			// spring damper with limits
			SubmitSpringDamper0(desc, matrix0, matrix1);
		}
	}

	if (!(hitLimit & 2))
	{
		if ((m_axis1.m_springK > ndFloat32(0.0f)) || (m_axis1.m_damperC > ndFloat32(0.0f)))
		{
			// spring damper with limits
			SubmitSpringDamper1(desc, matrix0, matrix1);
		}
	}
}

