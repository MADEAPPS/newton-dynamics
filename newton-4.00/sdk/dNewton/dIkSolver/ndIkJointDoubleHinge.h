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

#ifndef __ND_IK_DOUBLE_JOINT_HINGE_H__
#define __ND_IK_DOUBLE_JOINT_HINGE_H__

#include "ndNewtonStdafx.h"
#include "ndJointDoubleHinge.h"

class ndIkJointDoubleHinge: public ndJointDoubleHinge, public ndJointBilateralConstraint::ndIkInterface
{
	public:
	D_CLASS_REFLECTION(ndIkJointDoubleHinge);
	D_NEWTON_API ndIkJointDoubleHinge(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API ndIkJointDoubleHinge(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	//D_NEWTON_API ndIkJointDoubleHinge(const ndMatrix& pinAndPivotInChild, const ndMatrix& pinAndPivotInParent, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndIkJointDoubleHinge();

	// inverse dynamics interface
	D_ADD_IK_INTERFACE();

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;
};

#endif 

