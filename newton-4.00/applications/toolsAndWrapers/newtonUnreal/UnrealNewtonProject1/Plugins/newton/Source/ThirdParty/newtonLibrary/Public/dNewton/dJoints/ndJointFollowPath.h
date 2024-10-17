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


#ifndef __ND_JOINT_FOLLOW_PATH_H__
#define __ND_JOINT_FOLLOW_PATH_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndJointFollowPath: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointFollowPath, ndJointBilateralConstraint)

	D_NEWTON_API ndJointFollowPath();
	D_NEWTON_API ndJointFollowPath (const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointFollowPath();

	virtual void GetPointAndTangentAtLocation(const ndVector& location, ndVector& positOut, ndVector& tangentOut) const 
	{ 
		positOut = location;
		tangentOut = ndVector(1.0f, 0.0f, 0.0f, 0.0f);
	}

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
};

#endif

