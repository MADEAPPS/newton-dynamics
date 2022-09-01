/* Copyright (c) <2003-2022> <Newton Game Dynamics>
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
#include "ndIkJointSpherical.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndIkJointSpherical)

ndIkJointSpherical::ndIkJointSpherical(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointSpherical(pinAndPivotFrame, child, parent)
	,ndJointBilateralConstraint::ndIkInterface()
{
}

ndIkJointSpherical::ndIkJointSpherical(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointSpherical(ndLoadSaveBase::ndLoadDescriptor(desc))
	,ndJointBilateralConstraint::ndIkInterface()
{
}

ndIkJointSpherical::~ndIkJointSpherical()
{
}

void ndIkJointSpherical::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointSpherical::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));
}

void ndIkJointSpherical::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	ndJointSpherical::DebugJoint(debugCallback);
}

void ndIkJointSpherical::SubmitAccel(const ndMatrix&, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	// if we have the alpha, there is not need to find a shortest path, any set of axis will do 
	for (ndInt32 i = 0; i < 3; ++i)
	{
		ndFloat32 accel = (matrix1[i] * m_accel0.m_angular - matrix1[i] * m_accel1.m_angular).AddHorizontal().GetScalar();
		AddAngularRowJacobian(desc, matrix1[i], ndFloat32(0.0f));
		SetMotorAcceleration(desc, accel);
		SetDiagonalRegularizer(desc, m_defualRegularizer);
	}
}

void ndIkJointSpherical::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);
	ApplyBaseRows(matrix0, matrix1, desc);
	if (!m_ikMode)
	{
		SubmitAccel(matrix0, matrix1, desc);
	}
	SubmitLimits(matrix0, matrix1, desc);
}