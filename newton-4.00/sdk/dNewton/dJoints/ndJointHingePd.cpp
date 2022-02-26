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
#include "ndJointHingePd.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointHingePd)

ndJointHingePd::ndJointHingePd(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointHinge(pinAndPivotFrame, child, parent)
	,m_targetAngle(ndFloat32(0.0f))
{
}

ndJointHingePd::ndJointHingePd(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointHinge(ndLoadSaveBase::ndLoadDescriptor(desc))
	,m_targetAngle(ndFloat32 (0.0f))
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	m_targetAngle = xmlGetFloat(xmlNode, "targetAngle");
}

ndJointHingePd::~ndJointHingePd()
{
}

void ndJointHingePd::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointHinge::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "targetAngle", m_targetAngle);
}


ndFloat32 ndJointHingePd::GetTarget() const
{
	return m_targetAngle;
}

void ndJointHingePd::SetTarget(ndFloat32 angle)
{
	m_targetAngle = angle;
}

void ndJointHingePd::SubmitSpringDamper(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix&)
{
	AddAngularRowJacobian(desc, matrix0.m_front, m_targetAngle - m_angle);
	SetMassSpringDamperAcceleration(desc, m_springDamperRegularizer, m_springK, m_damperC);
}

void ndJointHingePd::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	ApplyBaseRows(desc, matrix0, matrix1);
	bool hitLimit = SubmitConstraintLimits(desc, matrix0, matrix1);
	if (!hitLimit)
	{
		if ((m_springK > ndFloat32(0.0f)) || (m_damperC > ndFloat32(0.0f)))
		{
			// spring damper with limits
			SubmitSpringDamper(desc, matrix0, matrix1);
		}
	}
}


