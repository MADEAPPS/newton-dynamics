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


#ifndef __D_JOINT_FOLLOW_PATH_H__
#define __D_JOINT_FOLLOW_PATH_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndJointFollowPath: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointFollowPath);
	D_NEWTON_API ndJointFollowPath(const dLoadSaveBase::dLoadDescriptor& desc);
	D_NEWTON_API ndJointFollowPath (const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointFollowPath();

	virtual void GetPointAndTangentAtLocation(const dVector& location, dVector& positOut, dVector& tangentOut) const 
	{ 
		positOut = location;
		tangentOut = dVector(1.0f, 0.0f, 0.0f, 0.0f);
	}

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const dLoadSaveBase::dSaveDescriptor& desc) const;
};

#endif

