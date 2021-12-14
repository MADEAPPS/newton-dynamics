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
#include "ndJointPlane.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointPlane)

ndJointPlane::ndJointPlane (const ndVector& pivot, const ndVector& normal, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(5, child, parent, dGetIdentityMatrix())
	,m_enableControlRotation(true)
{
	ndMatrix pinAndPivotFrame(normal);
	pinAndPivotFrame.m_posit = pivot;
	pinAndPivotFrame.m_posit.m_w = 1.0f;
	// calculate the two local matrix of the pivot point
	CalculateLocalMatrix (pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
}

ndJointPlane::ndJointPlane(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointBilateralConstraint(ndLoadSaveBase::ndLoadDescriptor(desc))
	,m_enableControlRotation(true)
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;

	m_enableControlRotation = xmlGetInt(xmlNode, "enableControlRotation") ? true : false;
}

ndJointPlane::~ndJointPlane()
{
}

void ndJointPlane::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	// Restrict the movement on the pivot point along all two orthonormal axis direction perpendicular to the motion
	const ndVector& dir = matrix1[0];
	const ndVector& p0 = matrix0.m_posit;
	const ndVector& p1 = matrix1.m_posit;
	//NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &dir[0]);
	AddLinearRowJacobian(desc, p0, p1, dir);
	
	//const ndFloat32 invTimeStep = 1.0f / timestep;
	const ndFloat32 dist = 0.25f * dir.DotProduct((p1 - p0) & ndVector::m_triplexMask).GetScalar();
	//const ndFloat32 accel = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dist * invTimeStep * invTimeStep;
	const ndFloat32 accel = GetMotorZeroAcceleration(desc) + dist * desc.m_invTimestep * desc.m_invTimestep;

	//NewtonUserJointSetRowAcceleration(m_joint, accel);
	SetMotorAcceleration(desc, accel);
	//NewtonUserJointSetRowStiffness(m_joint, m_stiffness);

	// construct an orthogonal coordinate system with these two vectors
	if (m_enableControlRotation) 
	{
		//NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up), &matrix1.m_up[0]);
		AddAngularRowJacobian(desc, matrix1.m_up, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up));
		//NewtonUserJointSetRowStiffness(m_joint, m_stiffness);

		//NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right), &matrix1.m_right[0]);
		AddAngularRowJacobian(desc, matrix1.m_right, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right));
		//NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	}
}

void ndJointPlane::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "enableControlRotation", m_enableControlRotation ? 1 : 0);
}
