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
	,m_minTorque(ndFloat32 (-1.0e10f))
	,m_maxTorque(ndFloat32(1.0e10f))
	,m_motorAccel(ndFloat32(0.0f))
	,m_savedMinToque(ndFloat32 (0.0f))
	,m_savedMaxTorque(ndFloat32(0.0f))
{
	m_hasLimits = false;
	m_isSpringDamper = false;
	m_friction = ndFloat32(0.0f);
}

ndJointIkHinge::ndJointIkHinge(const ndMatrix& pinAndPivotInChild, const ndMatrix& pinAndPivotInParent, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointHinge(pinAndPivotInChild, pinAndPivotInParent, child, parent)
	,m_minTorque(ndFloat32(-1.0e10f))
	,m_maxTorque(ndFloat32(1.0e10f))
	,m_motorAccel(ndFloat32(0.0f))
	,m_savedMinToque(ndFloat32(0.0f))
	,m_savedMaxTorque(ndFloat32(0.0f))
{
	m_hasLimits = false;
	m_isSpringDamper = false;
	m_friction = ndFloat32(0.0f);
}

ndJointIkHinge::ndJointIkHinge(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointHinge(ndLoadSaveBase::ndLoadDescriptor(desc))
	,m_minTorque(ndFloat32(-1.0e10f))
	,m_maxTorque(ndFloat32(1.0e10f))
	,m_motorAccel(ndFloat32(0.0f))
	,m_savedMinToque(ndFloat32(0.0f))
	,m_savedMaxTorque(ndFloat32(0.0f))
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

bool ndJointIkHinge::IsIk() const
{
	return true;
}

void ndJointIkHinge::SetIkSolver()
{
	m_savedMinToque = m_minTorque;
	m_savedMaxTorque = m_maxTorque;
	m_minTorque = ndFloat32(0.0f);
	m_maxTorque = ndFloat32(0.0f);
	m_motorAccel = ndFloat32(0.0f);
}

void ndJointIkHinge::ResetIkSolver()
{
	m_minTorque = m_savedMinToque;
	m_maxTorque = m_savedMaxTorque;
}

void ndJointIkHinge::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndJointHinge::JacobianDerivative(desc);
	const ndVector pin(m_body0->GetMatrix().RotateVector(GetLocalMatrix0().m_front));

	//ndMatrix matrix0;
	//ndMatrix matrix1;
	//CalculateGlobalMatrix(matrix0, matrix1);

	AddAngularRowJacobian(desc, pin, ndFloat32 (0.0f));
	SetMotorAcceleration(desc, m_motorAccel);
	SetLowerFriction(desc, m_minTorque);
	SetHighFriction(desc, m_maxTorque);
}


