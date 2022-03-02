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

#ifndef __ND_IK_JOINT_SPHERICAL_H__
#define __ND_IK_JOINT_SPHERICAL_H__

#include "ndNewtonStdafx.h"
#include "ndJointSpherical.h"

class ndIkJointSpherical: public ndJointSpherical, public ndJointBilateralConstraint::ndIkInterface
{
	public:
	D_CLASS_REFLECTION(ndIkJointSpherical);
	D_NEWTON_API ndIkJointSpherical(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API ndIkJointSpherical(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndIkJointSpherical();

	protected:
	// inverse dynamics interface
	D_ADD_IK_INTERFACE();
	D_COLLISION_API void SubmitAccel(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;
	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& debugCallback) const;
};

#endif 

