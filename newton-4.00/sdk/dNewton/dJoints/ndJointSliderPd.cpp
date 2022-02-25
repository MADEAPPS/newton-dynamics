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
#include "ndJointSliderPd.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointSliderPd)

ndJointSliderPd::ndJointSliderPd(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointSlider(pinAndPivotFrame, child, parent)
	,m_targetPosit(ndFloat32 (0.0f))
{
}

ndJointSliderPd::ndJointSliderPd(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointSlider(ndLoadSaveBase::ndLoadDescriptor(desc))
	,m_targetPosit(ndFloat32(0.0f))
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	
	m_targetPosit = xmlGetFloat(xmlNode, "targetPosit");
}

ndJointSliderPd::~ndJointSliderPd()
{
}

void ndJointSliderPd::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointSlider::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "targetPosit", m_targetPosit);
}

ndFloat32 ndJointSliderPd::GetTarget() const
{
	return m_targetPosit;
}

void ndJointSliderPd::SetTarget(ndFloat32 target)
{
	m_targetPosit = target;
}

void ndJointSliderPd::SubmitSpringDamper(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1)
{
	const ndVector p1(matrix1.m_posit + matrix1.m_front.Scale(m_targetPosit));
	AddLinearRowJacobian(desc, matrix0.m_posit, p1, matrix1.m_front);
	SetMassSpringDamperAcceleration(desc, m_springDamperRegularizer, m_springK, m_damperC);
}

void ndJointSliderPd::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	ApplyBaseRows(desc, matrix0, matrix1);
	bool hitLimit = SubmitConstraintLimits(desc, matrix0, matrix1);
	if (!hitLimit)
	{
		if ((m_springK > ndFloat32(0.0f)) || (m_damperC > ndFloat32(0.0f)))
		{
			SubmitSpringDamper(desc, matrix0, matrix1);
		}
	}
}



