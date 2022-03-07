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
#include "ndIkJointDoubleHinge.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndIkJointDoubleHinge)

ndIkJointDoubleHinge::ndIkJointDoubleHinge(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointDoubleHinge(pinAndPivotFrame, child, parent)
	,ndJointBilateralConstraint::ndIkInterface()
{
}

//ndIkJointDoubleHinge::ndIkJointDoubleHinge(const ndMatrix& pinAndPivotInChild, const ndMatrix& pinAndPivotInParent, ndBodyKinematic* const child, ndBodyKinematic* const parent)
//	:ndJointDoubleHinge(pinAndPivotInChild, pinAndPivotInParent, child, parent)
//	,ndIkInterface()
//{
//}

ndIkJointDoubleHinge::ndIkJointDoubleHinge(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointDoubleHinge(ndLoadSaveBase::ndLoadDescriptor(desc))
	,ndJointBilateralConstraint::ndIkInterface()
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

ndIkJointDoubleHinge::~ndIkJointDoubleHinge()
{
}

void ndIkJointDoubleHinge::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointDoubleHinge::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

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

void ndIkJointDoubleHinge::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	ApplyBaseRows(desc, matrix0, matrix1);
	SubmitLimits(desc, matrix0, matrix1);

	if (!m_ikMode)
	{
		const ndVector pin0(matrix0.m_front);
		ndFloat32 accel0 = (pin0 * m_accel0.m_angular - pin0 * m_accel1.m_angular).AddHorizontal().GetScalar();
		AddAngularRowJacobian(desc, pin0, 0.0f);
		SetMotorAcceleration(desc, accel0);
		//SetLowerFriction(desc, m_axisAccel.m_minForce);
		//SetHighFriction(desc, m_axisAccel.m_maxForce);

		const ndVector pin1(matrix1.m_up);
		ndFloat32 accel1 = (pin1 * m_accel0.m_angular - pin1 * m_accel1.m_angular).AddHorizontal().GetScalar();
		AddAngularRowJacobian(desc, pin1, 0.0f);
		SetMotorAcceleration(desc, accel1);
		//SetLowerFriction(desc, m_axisAccel.m_minForce);
		//SetHighFriction(desc, m_axisAccel.m_maxForce);
	}
}


