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
#include "ndJointPdSlider.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointPdSlider)

ndJointPdSlider::ndJointPdSlider(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointSlider(pinAndPivotFrame, child, parent)
	,n_targetPosit(ndFloat32 (0.0f))
{
}

ndJointPdSlider::ndJointPdSlider(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointSlider(ndLoadSaveBase::ndLoadDescriptor(desc))
	,n_targetPosit(ndFloat32(0.0f))
{
	dAssert(0);
	//const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	//
	//m_springK = xmlGetFloat(xmlNode, "springK");
	//m_damperC = xmlGetFloat(xmlNode, "damperC");
	//m_minLimit = xmlGetFloat(xmlNode, "minLimit");
	//m_maxLimit = xmlGetFloat(xmlNode, "maxLimit");
	//m_friction = xmlGetFloat(xmlNode, "friction");
	//m_springDamperRegularizer = xmlGetFloat(xmlNode, "springDamperRegularizer");
	//m_hasLimits = xmlGetInt(xmlNode, "hasLimits") ? true : false;
	//m_isSpringDamper = xmlGetInt(xmlNode, "isSpringDamper") ? true : false;
}

ndJointPdSlider::~ndJointPdSlider()
{
}

void ndJointPdSlider::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointSlider::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	dAssert(0);
	//xmlSaveParam(childNode, "springK", m_springK);
	//xmlSaveParam(childNode, "damperC", m_damperC);
	//xmlSaveParam(childNode, "minLimit", m_minLimit);
	//xmlSaveParam(childNode, "maxLimit", m_maxLimit);
	//xmlSaveParam(childNode, "friction", m_friction);
	//xmlSaveParam(childNode, "springDamperRegularizer", m_springDamperRegularizer);
	//xmlSaveParam(childNode, "hasLimits", m_hasLimits ? 1 : 0);
	//xmlSaveParam(childNode, "isSpringDamper", m_isSpringDamper ? 1 : 0);
}

ndFloat32 ndJointPdSlider::GetTarget() const
{
	return n_targetPosit;
}

void ndJointPdSlider::SetTarget(ndFloat32 target)
{
	n_targetPosit = dClamp(target, m_minLimit, m_maxLimit);
}

void ndJointPdSlider::JacobianDerivative(ndConstraintDescritor& desc)
{
	dAssert(0);
	//ndJointSlider::JacobianDerivative(desc);
	//
	//ndMatrix matrix0;
	//ndMatrix matrix1;
	//
	//// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	//CalculateGlobalMatrix(matrix0, matrix1);
	//
	//const ndVector posit(matrix0.m_posit - matrix1.m_front.Scale(n_targetPosit));
	//AddLinearRowJacobian(desc, posit, matrix1.m_posit, matrix1.m_front);
	//SetMassSpringDamperAcceleration(desc, m_springDamperRegularizer, m_springK, m_damperC);
	//SubmitConstraintLimits(desc, matrix0, matrix1);
}



