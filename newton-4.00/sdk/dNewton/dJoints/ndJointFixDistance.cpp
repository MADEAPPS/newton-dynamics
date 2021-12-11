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
#include "ndJointFixDistance.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointFixDistance)

ndJointFixDistance::ndJointFixDistance(const ndVector& pivotInChildInGlobalSpace, const ndVector& pivotInParentInGlobalSpace, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(3, child, parent, dGetIdentityMatrix())
	,m_distance(dFloat32 (0.0f))
{
	ndVector dist(pivotInChildInGlobalSpace - pivotInParentInGlobalSpace);
	m_distance = dSqrt(dist.DotProduct(dist).GetScalar());

	ndMatrix childMatrix(dGetIdentityMatrix());
	ndMatrix parentMatrix(dGetIdentityMatrix());

	childMatrix.m_posit = pivotInChildInGlobalSpace;
	parentMatrix.m_posit = pivotInParentInGlobalSpace;
	childMatrix.m_posit.m_w = 1.0f;
	parentMatrix.m_posit.m_w = 1.0f;

	ndMatrix dummy;
	CalculateLocalMatrix(childMatrix, m_localMatrix0, dummy);
	CalculateLocalMatrix(parentMatrix, dummy, m_localMatrix1);
}

ndJointFixDistance::ndJointFixDistance(const ndLoadSaveBase::dLoadDescriptor& desc)
	:ndJointBilateralConstraint(ndLoadSaveBase::dLoadDescriptor(desc))
	,m_distance(dFloat32(0.0f))
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;

	m_distance = xmlGetFloat(xmlNode, "distance");
}

ndJointFixDistance::~ndJointFixDistance()
{
}

void ndJointFixDistance::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	ndVector p0(matrix0.m_posit);
	ndVector p1(matrix1.m_posit);

	ndVector dir(p1 - p0);
	dFloat32 mag2 = dir.DotProduct(dir).GetScalar();
	//if (mag2 < dFloat32 (1.0e-3f)) 
	//{
	//	AddLinearRowJacobian(desc, p0, p1, matrix0.m_front);
	//	AddLinearRowJacobian(desc, p0, p1, matrix0.m_up);
	//	AddLinearRowJacobian(desc, p0, p1, matrix0.m_right);
	//} 
	//else 
	if (mag2 > dFloat32(1.0e-3f))
	{
		dir = dir.Scale(1.0f / dSqrt(mag2));
		dFloat32 x = dSqrt (mag2) - m_distance;

		ndMatrix matrix(dir);
		ndVector com0(m_body0->GetCentreOfMass());
		ndMatrix body0Matrix(m_body0->GetMatrix());
		ndVector veloc0(m_body0->GetVelocityAtPoint(p0));

		ndVector com1(m_body1->GetCentreOfMass());
		ndMatrix body1Matrix(m_body1->GetMatrix());
		ndVector veloc1(m_body1->GetVelocityAtPoint(p1));

		dFloat32 v((veloc0 - veloc1).DotProduct(dir).GetScalar());
		dFloat32 a = (x - v * desc.m_timestep) * desc.m_invTimestep * desc.m_invTimestep;

		ndVector r0 ((p0 - body0Matrix.TransformVector(com0)).CrossProduct(matrix.m_front));
		ndVector r1 ((p1 - body1Matrix.TransformVector(com1)).CrossProduct(matrix.m_front));

		AddLinearRowJacobian(desc, p0, p0, matrix0.m_right);
		ndJacobian& jacobian0 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM0;
		ndJacobian& jacobian1 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM1;

		jacobian0.m_linear = matrix[0];
		jacobian0.m_angular = r0;

		jacobian1.m_linear = matrix[0].Scale(dFloat32 (-1.0f));
		jacobian1.m_angular = r1.Scale(dFloat32(-1.0f));

		SetMotorAcceleration(desc, a);
	}
}

void ndJointFixDistance::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "distance", m_distance);
}

