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
#include "ndJointInverseDynamicsBase.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointInverseDynamicsBase)

ndJointInverseDynamicsBase::ndJointInverseDynamicsBase(dInt32 dof, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(dof, child, parent, dGetIdentityMatrix())
{
	m_localMatrix0 = dGetIdentityMatrix();
	m_localMatrix1 = dGetIdentityMatrix();
	SetSolverModel(m_jointkinematicCloseLoop);
}

ndJointInverseDynamicsBase::ndJointInverseDynamicsBase(const dLoadSaveBase::dLoadDescriptor& desc)
	:ndJointBilateralConstraint(dLoadSaveBase::dLoadDescriptor(desc))
{
//	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
}

ndJointInverseDynamicsBase::~ndJointInverseDynamicsBase()
{
}

void ndJointInverseDynamicsBase::Save(const dLoadSaveBase::dSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(dLoadSaveBase::dSaveDescriptor(desc, childNode));
}


