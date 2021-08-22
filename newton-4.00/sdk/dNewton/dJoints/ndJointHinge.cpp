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

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndJointHinge.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointHinge)

ndJointHinge::ndJointHinge(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(7, child, parent, pinAndPivotFrame)
	,m_angle(dFloat32(0.0f))
	,m_omega(dFloat32(0.0f))
	,m_springK(dFloat32(0.0f))
	,m_damperC(dFloat32(0.0f))
	,m_minLimit(dFloat32(0.0f))
	,m_maxLimit(dFloat32(0.0f))
	,m_friction(dFloat32(0.0f))
	,m_springDamperRegularizer(dFloat32(0.1f))
	,m_hasLimits(false)
	,m_isSpringDamper(false)
{
}

ndJointHinge::ndJointHinge(const dMatrix& pinAndPivotInChild, const dMatrix& pinAndPivotInParent, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(7, child, parent, pinAndPivotInChild)
	,m_angle(dFloat32(0.0f))
	,m_omega(dFloat32(0.0f))
	,m_springK(dFloat32(0.0f))
	,m_damperC(dFloat32(0.0f))
	,m_minLimit(dFloat32(0.0f))
	,m_maxLimit(dFloat32(0.0f))
	,m_friction(dFloat32(0.0f))
	,m_springDamperRegularizer(dFloat32(0.1f))
	,m_hasLimits(false)
	,m_isSpringDamper(false)
{
	dMatrix tmp;
	CalculateLocalMatrix(pinAndPivotInChild, m_localMatrix0, tmp);
	CalculateLocalMatrix(pinAndPivotInParent, tmp, m_localMatrix1);
}

ndJointHinge::ndJointHinge(const dLoadSaveBase::dLoadDescriptor& desc)
	:ndJointBilateralConstraint(dLoadSaveBase::dLoadDescriptor(desc))
	,m_angle(dFloat32(0.0f))
	,m_omega(dFloat32(0.0f))
	,m_springK(dFloat32(0.0f))
	,m_damperC(dFloat32(0.0f))
	,m_minLimit(dFloat32(0.0f))
	,m_maxLimit(dFloat32(0.0f))
	,m_friction(dFloat32(0.0f))
	,m_springDamperRegularizer(dFloat32(0.1f))
	,m_hasLimits(false)
	,m_isSpringDamper(false)
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;

	m_springK = xmlGetFloat(xmlNode, "springK");
	m_damperC = xmlGetFloat(xmlNode, "damperC");
	m_minLimit = xmlGetFloat(xmlNode, "minLimit");
	m_maxLimit = xmlGetFloat(xmlNode, "maxLimit");
	m_friction = xmlGetFloat(xmlNode, "friction");
	m_springDamperRegularizer = xmlGetFloat(xmlNode, "springDamperRegularizer");
	m_hasLimits = xmlGetInt(xmlNode, "hasLimits") ? true : false;
	m_isSpringDamper = xmlGetInt(xmlNode, "isSpringDamper") ? true : false;
}

ndJointHinge::~ndJointHinge()
{
}

dFloat32 ndJointHinge::GetAngle() const
{
	return m_angle;
}

dFloat32 ndJointHinge::GetOmega() const
{
	return m_omega;
}

void ndJointHinge::EnableLimits(bool state, dFloat32 minLimit, dFloat32 maxLimit)
{
	m_hasLimits = state;
	dAssert(minLimit <= 0.0f);
	dAssert(maxLimit >= 0.0f);
	m_minLimit = minLimit;
	m_maxLimit = maxLimit;

	// adding one extra dof, this makes the mass matrix ill conditioned, 
	// but it could work with the direct solver
	m_maxDof = (m_isSpringDamper && m_hasLimits) ? 7 : 6;
}

void ndJointHinge::SetFriction(dFloat32 frictionTorque)
{
	m_friction = dAbs(frictionTorque);
}

dFloat32 ndJointHinge::GetFriction() const
{
	return m_friction;
}

void ndJointHinge::SetAsSpringDamper(bool state, dFloat32 regularizer, dFloat32 spring, dFloat32 damper)
{
	m_springK = dAbs(spring);
	m_damperC = dAbs(damper);
	m_springDamperRegularizer = dClamp(regularizer, dFloat32(1.0e-2f), dFloat32(0.99f));
	m_isSpringDamper = state;

	// adding one extra dof, this makes the mass matrix ill conditioned, 
	// but it could work with the direct solver
	m_maxDof = (m_isSpringDamper && m_hasLimits) ? 7 : 6;
}

void ndJointHinge::SubmitConstraintLimits(ndConstraintDescritor& desc, const dMatrix& matrix0, const dMatrix& matrix1)
{
	if ((m_minLimit > dFloat32 (-1.e-4f)) && (m_maxLimit < dFloat32(1.e-4f)))
	{
		AddAngularRowJacobian(desc, &matrix1.m_front[0], -m_angle);
	}
	else 
	{
		const dFloat32 angle = m_angle + m_omega * desc.m_timestep;
		if (angle < m_minLimit)
		{
			AddAngularRowJacobian(desc, &matrix0.m_front[0], dFloat32(0.0f));
			const dFloat32 stopAccel = GetMotorZeroAcceleration(desc);
			const dFloat32 penetration = angle - m_minLimit;
			const dFloat32 recoveringAceel = -desc.m_invTimestep * D_HINGE_PENETRATION_RECOVERY_SPEED * dMin(dAbs(penetration / D_HINGE_PENETRATION_LIMIT), dFloat32(1.0f));
			SetMotorAcceleration(desc, stopAccel - recoveringAceel);
			SetLowerFriction(desc, -m_friction);
		}
		else if (angle > m_maxLimit)
		{
			AddAngularRowJacobian(desc, &matrix0.m_front[0], dFloat32(0.0f));
			const dFloat32 stopAccel = GetMotorZeroAcceleration(desc);
			const dFloat32 penetration = angle - m_maxLimit;
			const dFloat32 recoveringAceel = desc.m_invTimestep * D_HINGE_PENETRATION_RECOVERY_SPEED * dMin(dAbs(penetration / D_HINGE_PENETRATION_LIMIT), dFloat32(1.0f));
			SetMotorAcceleration(desc, stopAccel - recoveringAceel);
			SetHighFriction(desc, m_friction);
		}
		else if (m_friction > dFloat32(0.0f))
		{
			AddAngularRowJacobian(desc, matrix0.m_front, dFloat32(0.0f));
			const dFloat32 stopAccel = GetMotorZeroAcceleration(desc);
			SetMotorAcceleration(desc, stopAccel);
			SetLowerFriction(desc, -m_friction);
			SetHighFriction(desc, m_friction);
		}
	}
}

void ndJointHinge::SubmitConstraintLimitSpringDamper(ndConstraintDescritor& desc, const dMatrix& matrix0, const dMatrix& )
{
	// add spring damper row
	AddAngularRowJacobian(desc, matrix0.m_front, -m_angle);
	SetMassSpringDamperAcceleration(desc, m_springDamperRegularizer, m_springK, m_damperC);

	const dFloat32 angle = m_angle + m_omega * desc.m_timestep;
	if (angle < m_minLimit)
	{
		AddAngularRowJacobian(desc, &matrix0.m_front[0], dFloat32(0.0f));
		const dFloat32 stopAccel = GetMotorZeroAcceleration(desc);
		const dFloat32 penetration = angle - m_minLimit;
		const dFloat32 recoveringAceel = -desc.m_invTimestep * D_HINGE_PENETRATION_RECOVERY_SPEED * dMin(dAbs(penetration / D_HINGE_PENETRATION_LIMIT), dFloat32(1.0f));
		SetMotorAcceleration(desc, stopAccel - recoveringAceel);
		SetLowerFriction(desc, -m_friction);
	}
	else if (angle > m_maxLimit)
	{
		AddAngularRowJacobian(desc, &matrix0.m_front[0], dFloat32(0.0f));
		const dFloat32 stopAccel = GetMotorZeroAcceleration(desc);
		const dFloat32 penetration = angle - m_maxLimit;
		const dFloat32 recoveringAceel = desc.m_invTimestep * D_HINGE_PENETRATION_RECOVERY_SPEED * dMin(dAbs(penetration / D_HINGE_PENETRATION_LIMIT), dFloat32(1.0f));
		SetMotorAcceleration(desc, stopAccel - recoveringAceel);
		SetHighFriction(desc, m_friction);
	}
}

void ndJointHinge::JacobianDerivative(ndConstraintDescritor& desc)
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[0]);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[1]);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[2]);

	// save the current joint Omega
	dVector omega0(m_body0->GetOmega());
	dVector omega1(m_body1->GetOmega());

	// the joint angle can be determined by getting the angle between any two non parallel vectors
	const dFloat32 deltaAngle = AnglesAdd(-CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front), -m_angle);
	m_angle += deltaAngle;
	m_omega = matrix1.m_front.DotProduct(omega0 - omega1).GetScalar();

	// two rows to restrict rotation around around the parent coordinate system
	const dFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
	AddAngularRowJacobian(desc, matrix1.m_up, angle0);

	const dFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
	AddAngularRowJacobian(desc, matrix1.m_right, angle1);

	if (m_hasLimits)
	{
		if (m_isSpringDamper)
		{
			// spring damper with limits
			SubmitConstraintLimitSpringDamper(desc, matrix0, matrix1);
		}
		else
		{
			// only hard limits
			SubmitConstraintLimits(desc, matrix0, matrix1);
		}
	}
	else if (m_isSpringDamper)
	{
		AddAngularRowJacobian(desc, matrix0.m_front, -m_angle);
		SetMassSpringDamperAcceleration(desc, m_springDamperRegularizer, m_springK, m_damperC);
	}
	else if (m_friction > dFloat32 (0.0f))
	{
		AddAngularRowJacobian(desc, matrix0.m_front, dFloat32(0.0f));
		const dFloat32 stopAccel = GetMotorZeroAcceleration(desc);
		SetMotorAcceleration(desc, stopAccel);
		SetLowerFriction(desc, -m_friction);
		SetHighFriction(desc, m_friction);
	}
}

void ndJointHinge::Save(const dLoadSaveBase::dSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(dLoadSaveBase::dSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "springK", m_springK);
	xmlSaveParam(childNode, "damperC", m_damperC);
	xmlSaveParam(childNode, "minLimit", m_minLimit);
	xmlSaveParam(childNode, "maxLimit", m_maxLimit);
	xmlSaveParam(childNode, "friction", m_friction);
	xmlSaveParam(childNode, "springDamperRegularizer", m_springDamperRegularizer);
	xmlSaveParam(childNode, "hasLimits", m_hasLimits ? 1 : 0);
	xmlSaveParam(childNode, "isSpringDamper", m_isSpringDamper ? 1 : 0);
}
