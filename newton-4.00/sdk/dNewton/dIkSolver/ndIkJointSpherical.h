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

#ifndef __ND_IK_JOINT_SPHERICAL_H__
#define __ND_IK_JOINT_SPHERICAL_H__

#include "ndNewtonStdafx.h"
#include "dJoints/ndJointSpherical.h"

D_MSV_NEWTON_CLASS_ALIGN_32
class ndIkJointSpherical: public ndJointSpherical, public ndJointBilateralConstraint::ndIkInterface
{
	public:
	D_CLASS_REFLECTION(ndIkJointSpherical, ndJointSpherical)

	D_NEWTON_API ndIkJointSpherical();
	D_NEWTON_API ndIkJointSpherical(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndIkJointSpherical();

	// inverse dynamics interface
	D_ADD_IK_INTERFACE()

	protected:
	D_NEWTON_API void SubmitAccel(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& debugCallback) const;

	D_NEWTON_API ndInt32 GetKinematicState(ndKinematicState* const state) const;
} D_GCC_NEWTON_CLASS_ALIGN_32;


#endif 

