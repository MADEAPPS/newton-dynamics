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
#include "ndJointIkHinge.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointIkHinge)

ndJointIkHinge::ndJointIkHinge(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointHinge(pinAndPivotFrame, child, parent)
{
	m_hasLimits = false;
	m_isSpringDamper = false;
	m_friction = ndFloat32(0.0f);
}

ndJointIkHinge::ndJointIkHinge(const ndMatrix& pinAndPivotInChild, const ndMatrix& pinAndPivotInParent, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointHinge(pinAndPivotInChild, pinAndPivotInParent, child, parent)
{
	m_hasLimits = false;
	m_isSpringDamper = false;
	m_friction = ndFloat32(0.0f);
}

ndJointIkHinge::ndJointIkHinge(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointHinge(ndLoadSaveBase::ndLoadDescriptor(desc))
{
	m_hasLimits = false;
	m_isSpringDamper = false;
	m_friction = ndFloat32(0.0f);

	//const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	//
	//m_springK = xmlGetFloat(xmlNode, "springK");
	//m_damperC = xmlGetFloat(xmlNode, "damperC");
	//m_minLimit = xmlGetFloat(xmlNode, "minLimit");
	//m_maxLimit = xmlGetFloat(xmlNode, "maxLimit");
	//m_friction = xmlGetFloat(xmlNode, "friction");
	//m_motorAccel = xmlGetFloat(xmlNode, "axisAccel");
	//m_springDamperRegularizer = xmlGetFloat(xmlNode, "springDamperRegularizer");
	//m_hasLimits = xmlGetInt(xmlNode, "isMotor") ? true : false;
	//m_hasLimits = xmlGetInt(xmlNode, "hasLimits") ? true : false;
	//m_isSpringDamper = xmlGetInt(xmlNode, "isSpringDamper") ? true : false;
}

ndJointIkHinge::~ndJointIkHinge()
{
}

//void ndJointIkHinge::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
void ndJointIkHinge::Save(const ndLoadSaveBase::ndSaveDescriptor&) const
{
	dAssert(0);
	//nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	//desc.m_rootNode->LinkEndChild(childNode);
	//childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	//ndJointBilateralConstraint::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));
	//
	//xmlSaveParam(childNode, "springK", m_springK);
	//xmlSaveParam(childNode, "damperC", m_damperC);
	//xmlSaveParam(childNode, "minLimit", m_minLimit);
	//xmlSaveParam(childNode, "maxLimit", m_maxLimit);
	//xmlSaveParam(childNode, "friction", m_friction);
	//xmlSaveParam(childNode, "axisAccel", m_motorAccel);
	//xmlSaveParam(childNode, "springDamperRegularizer", m_springDamperRegularizer);
	//xmlSaveParam(childNode, "isMotor", m_isMotor ? 1 : 0);
	//xmlSaveParam(childNode, "hasLimits", m_hasLimits ? 1 : 0);
	//xmlSaveParam(childNode, "isSpringDamper", m_isSpringDamper ? 1 : 0);
}


//void ndJointIkHinge::EnableMotorAccel(bool state, ndFloat32 motorAccel)
//{
//	m_isMotor = state;
//	m_motorAccel = motorAccel;
//}
//
//bool ndJointIkHinge::IsMotor() const
//{
//	return m_isMotor;
//}
//
//void ndJointIkHinge::EnableLimits(bool state, ndFloat32 minLimit, ndFloat32 maxLimit)
//{
//	m_hasLimits = state;
//	dAssert(minLimit <= 0.0f);
//	dAssert(maxLimit >= 0.0f);
//	m_minLimit = minLimit;
//	m_maxLimit = maxLimit;
//
//	// adding one extra dof, this makes the mass matrix ill conditioned, 
//	// but it could work with the direct solver
//	m_maxDof = (m_isSpringDamper && m_hasLimits) ? 7 : 6;
//}
//
//void ndJointIkHinge::GetLimits(ndFloat32& minLimit, ndFloat32& maxLimit)
//{
//	minLimit = m_minLimit;
//	maxLimit = m_maxLimit;
//}
//
//
//
//void ndJointIkHinge::SubmitConstraintLimits(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1)
//{
//	if ((m_minLimit > ndFloat32 (-1.e-4f)) && (m_maxLimit < ndFloat32(1.e-4f)))
//	{
//		AddAngularRowJacobian(desc, &matrix1.m_front[0], -m_angle);
//	}
//	else 
//	{
//		const ndFloat32 angle = m_angle + m_omega * desc.m_timestep;
//		if (angle < m_minLimit)
//		{
//			AddAngularRowJacobian(desc, &matrix0.m_front[0], ndFloat32(0.0f));
//			const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
//			const ndFloat32 penetration = angle - m_minLimit;
//			const ndFloat32 recoveringAceel = -desc.m_invTimestep * D_HINGE_PENETRATION_RECOVERY_SPEED * dMin(dAbs(penetration / D_HINGE_PENETRATION_LIMIT), ndFloat32(1.0f));
//			SetMotorAcceleration(desc, stopAccel - recoveringAceel);
//			SetLowerFriction(desc, -m_friction);
//		}
//		else if (angle > m_maxLimit)
//		{
//			AddAngularRowJacobian(desc, &matrix0.m_front[0], ndFloat32(0.0f));
//			const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
//			const ndFloat32 penetration = angle - m_maxLimit;
//			const ndFloat32 recoveringAceel = desc.m_invTimestep * D_HINGE_PENETRATION_RECOVERY_SPEED * dMin(dAbs(penetration / D_HINGE_PENETRATION_LIMIT), ndFloat32(1.0f));
//			SetMotorAcceleration(desc, stopAccel - recoveringAceel);
//			SetHighFriction(desc, m_friction);
//		}
//		else if (m_friction > ndFloat32(0.0f))
//		{
//			AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));
//			const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
//			SetMotorAcceleration(desc, stopAccel);
//			SetLowerFriction(desc, -m_friction);
//			SetHighFriction(desc, m_friction);
//		}
//	}
//}
//
//void ndJointIkHinge::SubmitConstraintLimitSpringDamper(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& )
//{
//	// add spring damper row
//	AddAngularRowJacobian(desc, matrix0.m_front, -m_angle);
//	SetMassSpringDamperAcceleration(desc, m_springDamperRegularizer, m_springK, m_damperC);
//
//	const ndFloat32 angle = m_angle + m_omega * desc.m_timestep;
//	if (angle < m_minLimit)
//	{
//		AddAngularRowJacobian(desc, &matrix0.m_front[0], ndFloat32(0.0f));
//		const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
//		const ndFloat32 penetration = angle - m_minLimit;
//		const ndFloat32 recoveringAceel = -desc.m_invTimestep * D_HINGE_PENETRATION_RECOVERY_SPEED * dMin(dAbs(penetration / D_HINGE_PENETRATION_LIMIT), ndFloat32(1.0f));
//		SetMotorAcceleration(desc, stopAccel - recoveringAceel);
//		SetLowerFriction(desc, -m_friction);
//	}
//	else if (angle > m_maxLimit)
//	{
//		AddAngularRowJacobian(desc, &matrix0.m_front[0], ndFloat32(0.0f));
//		const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
//		const ndFloat32 penetration = angle - m_maxLimit;
//		const ndFloat32 recoveringAceel = desc.m_invTimestep * D_HINGE_PENETRATION_RECOVERY_SPEED * dMin(dAbs(penetration / D_HINGE_PENETRATION_LIMIT), ndFloat32(1.0f));
//		SetMotorAcceleration(desc, stopAccel - recoveringAceel);
//		SetHighFriction(desc, m_friction);
//	}
//}
//
//ndJacobianPair ndJointIkHinge::GetPinJacobian() const
//{
//	ndMatrix matrix (GetLocalMatrix0() * GetBody0()->GetMatrix());
//	ndJacobianPair pair;
//	pair.m_jacobianM0.m_linear = ndVector::m_zero;
//	pair.m_jacobianM1.m_linear = ndVector::m_zero;
//	pair.m_jacobianM0.m_angular = matrix.m_front;
//	pair.m_jacobianM1.m_angular = matrix.m_front * ndVector::m_negOne;
//	return pair;
//}

void ndJointIkHinge::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndJointHinge::JacobianDerivative(desc);
	//ndMatrix matrix0;
	//ndMatrix matrix1;
	//CalculateGlobalMatrix(matrix0, matrix1);
	//
	//AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[0]);
	//AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[1]);
	//AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[2]);
	//
	//// save the current joint Omega
	//ndVector omega0(m_body0->GetOmega());
	//ndVector omega1(m_body1->GetOmega());
	//
	//// the joint angle can be determined by getting the angle between any two non parallel vectors
	//const ndFloat32 deltaAngle = AnglesAdd(-CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front), -m_angle);
	//m_angle += deltaAngle;
	//m_omega = matrix1.m_front.DotProduct(omega0 - omega1).GetScalar();
	//
	//// two rows to restrict rotation around around the parent coordinate system
	//const ndFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
	//AddAngularRowJacobian(desc, matrix1.m_up, angle0);
	//
	//const ndFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
	//AddAngularRowJacobian(desc, matrix1.m_right, angle1);
	//
	//if (m_isMotor)
	//{
	//	AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32 (0.0f));
	//	SetMotorAcceleration(desc, m_motorAccel);
	//}
	//else if (m_hasLimits)
	//{
	//	if (m_isSpringDamper)
	//	{
	//		// spring damper with limits
	//		SubmitConstraintLimitSpringDamper(desc, matrix0, matrix1);
	//	}
	//	else
	//	{
	//		// only hard limits
	//		SubmitConstraintLimits(desc, matrix0, matrix1);
	//	}
	//}
	//else if (m_isSpringDamper)
	//{
	//	AddAngularRowJacobian(desc, matrix0.m_front, -m_angle);
	//	SetMassSpringDamperAcceleration(desc, m_springDamperRegularizer, m_springK, m_damperC);
	//}
	//else if (m_friction > ndFloat32 (0.0f))
	//{
	//	AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));
	//	const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
	//	SetMotorAcceleration(desc, stopAccel);
	//	SetLowerFriction(desc, -m_friction);
	//	SetHighFriction(desc, m_friction);
	//}
}


