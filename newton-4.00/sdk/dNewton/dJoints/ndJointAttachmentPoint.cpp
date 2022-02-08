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
#include "ndJointAttachmentPoint.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointAttachmentPoint)

ndJointAttachmentPoint::ndJointAttachmentPoint(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const body0, ndBodyKinematic* const body1)
	:ndJointBilateralConstraint(3, body0, body1, pinAndPivotFrame)
{
	m_lockedDimnetions.m_lockDof = 0x07f;
	SetSolverModel(m_jointkinematicCloseLoop);
}

ndJointAttachmentPoint::ndJointAttachmentPoint(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointBilateralConstraint(ndLoadSaveBase::ndLoadDescriptor(desc))
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	m_lockedDimnetions.m_lockDof = xmlGetInt(xmlNode, "lockDof");
}

ndJointAttachmentPoint::~ndJointAttachmentPoint()
{
}

void ndJointAttachmentPoint::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "lockDof", m_lockedDimnetions.m_lockDof);
}

void ndJointAttachmentPoint::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);
	if (m_lockedDimnetions.m_lock_x)
	{
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[0]);
	}
	if (m_lockedDimnetions.m_lock_y)
	{
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[1]);
	}
	if (m_lockedDimnetions.m_lock_z)
	{
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[2]);
	}
}
