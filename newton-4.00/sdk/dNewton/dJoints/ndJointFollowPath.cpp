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
#include "ndJointFollowPath.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointFollowPath)

ndJointFollowPath::ndJointFollowPath (const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(6, child, parent, pinAndPivotFrame)
{
	// calculate the two local matrix of the pivot point
	CalculateLocalMatrix (pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
}

ndJointFollowPath::ndJointFollowPath(const ndLoadSaveBase::dLoadDescriptor& desc)
	:ndJointBilateralConstraint(ndLoadSaveBase::dLoadDescriptor(desc))
{
	//const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
}

ndJointFollowPath::~ndJointFollowPath()
{
}

void ndJointFollowPath::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	ndVector pathPosit(ndVector::m_zero);
	ndVector pathTangent(ndVector::m_zero);
		
	// Get the global matrices of each rigid body.
	CalculateGlobalMatrix (matrix0, matrix1);
	
	GetPointAndTangentAtLocation(matrix0.m_posit, pathPosit, pathTangent);
	if (pathTangent.DotProduct(matrix0.m_front).GetScalar() < dFloat32 (0.0f)) 
	{
		pathTangent = pathTangent.Scale (-1.0f);
	}

	matrix1 = ndMatrix(pathTangent);
	matrix1.m_posit = pathPosit;
	matrix1.m_posit.m_w = 1.0f;
	
	// Restrict the movement on the pivot point along all tree the normal and bi normal of the path
	const ndVector& p0 = matrix0.m_posit;
	const ndVector& p1 = matrix1.m_posit;
	ndVector p00 (p0 + matrix0.m_front.Scale(50.0f));
	ndVector p11 (p1 + matrix1.m_front.Scale(50.0f));
	
	AddLinearRowJacobian(desc, p0, p1, matrix1[1]);
	AddLinearRowJacobian(desc, p0, p1, matrix1[2]);
	AddLinearRowJacobian(desc, p00, p11, matrix1[1]);
	AddLinearRowJacobian(desc, p00, p11, matrix1[2]);
}

void ndJointFollowPath::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));
}


