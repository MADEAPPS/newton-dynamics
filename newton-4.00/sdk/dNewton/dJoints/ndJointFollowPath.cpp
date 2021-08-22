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
#include "ndJointFollowPath.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointFollowPath)

ndJointFollowPath::ndJointFollowPath (const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(6, child, parent, pinAndPivotFrame)
{
	// calculate the two local matrix of the pivot point
	CalculateLocalMatrix (pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
}

ndJointFollowPath::ndJointFollowPath(const dLoadSaveBase::dLoadDescriptor& desc)
	:ndJointBilateralConstraint(dLoadSaveBase::dLoadDescriptor(desc))
{
	//const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
}

ndJointFollowPath::~ndJointFollowPath()
{
}

void ndJointFollowPath::JacobianDerivative(ndConstraintDescritor& desc)
{
	dMatrix matrix0;
	dMatrix matrix1;
	dVector pathPosit(dVector::m_zero);
	dVector pathTangent(dVector::m_zero);
		
	// Get the global matrices of each rigid body.
	CalculateGlobalMatrix (matrix0, matrix1);
	
	GetPointAndTangentAtLocation(matrix0.m_posit, pathPosit, pathTangent);
	if (pathTangent.DotProduct(matrix0.m_front).GetScalar() < dFloat32 (0.0f)) 
	{
		pathTangent = pathTangent.Scale (-1.0f);
	}

	matrix1 = dMatrix(pathTangent);
	matrix1.m_posit = pathPosit;
	matrix1.m_posit.m_w = 1.0f;
	
	// Restrict the movement on the pivot point along all tree the normal and bi normal of the path
	const dVector& p0 = matrix0.m_posit;
	const dVector& p1 = matrix1.m_posit;
	dVector p00 (p0 + matrix0.m_front.Scale(50.0f));
	dVector p11 (p1 + matrix1.m_front.Scale(50.0f));
	
	AddLinearRowJacobian(desc, p0, p1, matrix1[1]);
	AddLinearRowJacobian(desc, p0, p1, matrix1[2]);
	AddLinearRowJacobian(desc, p00, p11, matrix1[1]);
	AddLinearRowJacobian(desc, p00, p11, matrix1[2]);
}

void ndJointFollowPath::Save(const dLoadSaveBase::dSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(dLoadSaveBase::dSaveDescriptor(desc, childNode));
}


