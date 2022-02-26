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
#include "ndJointDoubleHingePd.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointDoubleHingePd)

ndJointDoubleHingePd::ndJointDoubleHingePd(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointDoubleHinge(pinAndPivotFrame, child, parent)
{
}

ndJointDoubleHingePd::ndJointDoubleHingePd(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointDoubleHinge(ndLoadSaveBase::ndLoadDescriptor(desc))
{
	dAssert(0);
	//const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	//m_angle0 = xmlGetFloat(xmlNode, "angle0");
	//m_angle1 = xmlGetFloat(xmlNode, "angle1");
	//m_omega0 = xmlGetFloat(xmlNode, "omega0");
	//m_omega1 = xmlGetFloat(xmlNode, "omega1");
}

ndJointDoubleHingePd::~ndJointDoubleHingePd()
{
}

void ndJointDoubleHingePd::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointDoubleHinge::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	//xmlSaveParam(childNode, "angle0", m_angle0);
	//xmlSaveParam(childNode, "angle1", m_angle1);
	//xmlSaveParam(childNode, "omega0", m_omega0);
	//xmlSaveParam(childNode, "omega1", m_omega1);
}

void ndJointDoubleHingePd::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	ApplyBaseRows(desc, matrix0, matrix1);
}

