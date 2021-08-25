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
#include "ndJointHingeActuator.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointHingeActuator)

ndJointHingeActuator::ndJointHingeActuator(const dMatrix& pinAndPivotFrame, dFloat32 angularRate, dFloat32 minAngle, dFloat32 maxAngle, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointHinge(pinAndPivotFrame, child, parent)
	,m_targetAngle(dFloat32(0.0f))
	,m_motorSpeed(angularRate)
	,m_maxTorque(D_LCP_MAX_VALUE)
{
	m_friction = dFloat32 (0.0f);
	SetAngularRate(angularRate);
	EnableLimits(false, minAngle, maxAngle);
}

ndJointHingeActuator::ndJointHingeActuator(const dLoadSaveBase::dLoadDescriptor& desc)
	:ndJointHinge(dLoadSaveBase::dLoadDescriptor(desc))
	,m_targetAngle(dFloat32 (0.0f))
	,m_motorSpeed(dFloat32(0.0f))
	,m_maxTorque(D_LCP_MAX_VALUE)
{
	m_friction = dFloat32(0.0f);
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	
	m_targetAngle = xmlGetFloat(xmlNode, "targetAngle");
	m_motorSpeed = xmlGetFloat(xmlNode, "motorSpeed");
	m_maxTorque = xmlGetFloat(xmlNode, "maxTorque");
}

ndJointHingeActuator::~ndJointHingeActuator()
{
}

dFloat32 ndJointHingeActuator::GetMinAngularLimit() const
{
	return m_minLimit;
}

dFloat32 ndJointHingeActuator::GetMaxAngularLimit() const
{
	return m_maxLimit;
}

dFloat32 ndJointHingeActuator::GetAngularRate() const
{
	return m_motorSpeed;
}

void ndJointHingeActuator::SetMinAngularLimit(dFloat32 limit)
{
	m_minLimit = limit;
}

void ndJointHingeActuator::SetMaxAngularLimit(dFloat32 limit)
{
	m_maxLimit = limit;
}

void ndJointHingeActuator::SetAngularRate(dFloat32 rate)
{
	m_motorSpeed = rate;
}

dFloat32 ndJointHingeActuator::GetTargetAngle() const
{
	return m_targetAngle;
}

void ndJointHingeActuator::SetTargetAngle(dFloat32 angle)
{
	angle = dClamp (angle, m_minLimit, m_maxLimit);
	if (dAbs (angle - m_targetAngle) > dFloat32 (1.0e-3f))
	{
		//ndBodyKinematicSetSleepState(m_body0, 0);
		m_targetAngle = angle;
	}
}

dFloat32 ndJointHingeActuator::GetMaxTorque() const
{
    return m_maxTorque;
}

void ndJointHingeActuator::SetMaxTorque(dFloat32 torque)
{
    m_maxTorque = dAbs (torque);
}

void ndJointHingeActuator::JacobianDerivative(ndConstraintDescritor& desc)
{
	m_hasLimits = false;
	m_isSpringDamper = false;
	m_friction = dFloat32(0.0f);

	ndJointHinge::JacobianDerivative(desc);

	dAssert(m_motorSpeed >= 0.0f);
	dFloat32 step = m_motorSpeed * desc.m_timestep;

	dFloat32 currentSpeed = 0.0f;
	if (m_angle < (m_targetAngle - step))
	{
		currentSpeed = m_motorSpeed;
	}
	else if (m_angle > (m_targetAngle + step))
	{
		currentSpeed = -m_motorSpeed;
	}
	else if (m_angle < m_targetAngle)
	{
		currentSpeed = dFloat32 (0.3f) * (m_targetAngle - m_angle) * desc.m_invTimestep;
		dAssert(dAbs(currentSpeed) < m_motorSpeed);
	}
	else if (m_angle > m_targetAngle)
	{
		currentSpeed = dFloat32(0.3f) * (m_targetAngle - m_angle) * desc.m_invTimestep;
		dAssert(dAbs(currentSpeed) < m_motorSpeed);
	}

	const dVector pin(m_body0->GetMatrix().RotateVector(m_localMatrix0.m_front));
	
	AddAngularRowJacobian(desc, pin, dFloat32 (0.0f));
	dFloat32 accel = GetMotorZeroAcceleration(desc) + dFloat32(0.3f) * currentSpeed * desc.m_invTimestep;
	SetMotorAcceleration(desc, accel);
	if (m_angle > GetMaxAngularLimit())
	{
		SetHighFriction(desc, m_maxTorque);
	}
	else if (m_angle < GetMinAngularLimit())
	{
		SetLowerFriction(desc, -m_maxTorque);
	}
	else 
	{
		SetHighFriction(desc, m_maxTorque);
		SetLowerFriction(desc, -m_maxTorque);
	}
	dAssert(desc.m_rowsCount <= 6);
}

void ndJointHingeActuator::Save(const dLoadSaveBase::dSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointHinge::Save(dLoadSaveBase::dSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "targetAngle", m_targetAngle);
	xmlSaveParam(childNode, "motorSpeed", m_motorSpeed);
	xmlSaveParam(childNode, "maxTorque", m_maxTorque);
}
