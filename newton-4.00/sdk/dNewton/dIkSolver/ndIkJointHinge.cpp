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
#include "ndIkJointHinge.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndIkJointHinge)

ndIkJointHinge::ndIkJointHinge(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointHinge(pinAndPivotFrame, child, parent)
	,m_axisAccel()
{
}

ndIkJointHinge::ndIkJointHinge(const ndMatrix& pinAndPivotInChild, const ndMatrix& pinAndPivotInParent, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointHinge(pinAndPivotInChild, pinAndPivotInParent, child, parent)
	,m_axisAccel()
{
}

ndIkJointHinge::ndIkJointHinge(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointHinge(ndLoadSaveBase::ndLoadDescriptor(desc))
	,m_axisAccel()
{
	dAssert(0);
	//const nd::TiXmlNode* const xmlNode = desc.m_rootNode;

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

ndIkJointHinge::~ndIkJointHinge()
{
}

void ndIkJointHinge::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointHinge::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	dAssert(0);
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

bool ndIkJointHinge::IsIk() const
{
	return true;
}

void ndIkJointHinge::SetIkSolver()
{
	m_axisAccel.Set();
}

void ndIkJointHinge::ResetIkSolver()
{
	m_axisAccel.Reset();
}

void ndIkJointHinge::StopIkMotor(ndFloat32 timestep)
{
	//m_motorAccel = -GetOmega() / timestep;
	m_axisAccel.m_motorAccel = -GetOmega() / timestep;
}

bool ndIkJointHinge::SetIkMotor(ndFloat32 timestep, const ndJacobian& forceBody0, const ndJacobian& forceBody1)
{
	const ndBodyKinematic* const body0 = GetBody0();
	const ndBodyKinematic* const body1 = GetBody1();

	const ndMatrix& invInertia0 = body0->GetInvInertiaMatrix();
	const ndMatrix& invInertia1 = body1->GetInvInertiaMatrix();

	const ndVector alpha0(invInertia0.RotateVector(forceBody0.m_angular));
	const ndVector alpha1(invInertia1.RotateVector(forceBody1.m_angular));

	ndFloat32 minLimit;
	ndFloat32 maxLimit;
	GetLimits(minLimit, maxLimit);
	ndJacobianPair jacobian(GetPinJacobian());
	ndFloat32 accel = (jacobian.m_jacobianM0.m_angular * alpha0 + jacobian.m_jacobianM1.m_angular * alpha1).AddHorizontal().GetScalar();
	ndFloat32 angle = GetAngle() + GetOmega() * timestep + accel * timestep * timestep;

	bool ret = true;
	if ((angle < minLimit) || (angle > maxLimit))
	{
		ret = false;
		accel = -GetOmega() / timestep;
		m_axisAccel.Reset();
	}
	m_axisAccel.m_motorAccel = accel;

	return ret;
}

void ndIkJointHinge::SetTorqueLimits(ndFloat32 minTorque, ndFloat32 maxTorque)
{
	m_axisAccel.SetForceLimit(minTorque, maxTorque);
}

void ndIkJointHinge::GetTorqueLimits(ndFloat32& minTorque, ndFloat32& maxTorque) const
{
	m_axisAccel.GetForceLimit(minTorque, maxTorque);
}

void ndIkJointHinge::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	ApplyBaseRows(desc, matrix0, matrix1);
	SubmitLimits(desc, matrix0, matrix1);

	const ndVector pin(matrix0.m_front);
	AddAngularRowJacobian(desc, pin, ndFloat32 (0.0f));
	SetMotorAcceleration(desc, m_axisAccel.m_motorAccel);
	SetLowerFriction(desc, m_axisAccel.m_minForce);
	SetHighFriction(desc, m_axisAccel.m_maxForce);
}


