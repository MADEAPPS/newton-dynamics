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

#ifndef __ND_IK_JOINT_HINGE_H__
#define __ND_IK_JOINT_HINGE_H__

#include "ndNewtonStdafx.h"
#include "dJoints/ndJointHinge.h"

// setting to a value larger that D_IK_HINGE_MAX_TORQUE, disable torque limit
#define D_IK_HINGE_MAX_TORQUE ndFloat32 (1.0e10f)

D_MSV_NEWTON_CLASS_ALIGN_32
class ndIkJointHinge: public ndJointHinge, public ndJointBilateralConstraint::ndIkInterface
{
	public:
	D_CLASS_REFLECTION(ndIkJointHinge, ndJointHinge)

	D_NEWTON_API ndIkJointHinge();
	D_NEWTON_API ndIkJointHinge(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API ndIkJointHinge(const ndMatrix& pinAndPivotInChild, const ndMatrix& pinAndPivotInParent, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndIkJointHinge();

	// inverse dynamics interface
	D_ADD_IK_INTERFACE()

	D_NEWTON_API ndFloat32 GetMaxTorque() const;
	D_NEWTON_API void SetMaxTorque(ndFloat32 maxTorque);

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);

	ndFloat32 m_maxTorque;
} D_GCC_NEWTON_CLASS_ALIGN_32;

#endif 

