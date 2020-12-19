/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __D_JOINT_FIX_DISTANCE_H_
#define __D_JOINT_FIX_DISTANCE_H_

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndJointFixDistance: public ndJointBilateralConstraint
{
	public:
	D_NEWTON_API ndJointFixDistance(const dVector& pivotInChildInGlobalSpace, const dVector& pivotFrame1, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointFixDistance();

	protected:
	void JacobianDerivative(ndConstraintDescritor& desc);

	dFloat32 m_distance;
};
#endif 

