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

#ifndef __ND_JOINT_FIX_DISTANCE_H_
#define __ND_JOINT_FIX_DISTANCE_H_

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

D_MSV_NEWTON_CLASS_ALIGN_32
class ndJointFixDistance: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointFixDistance, ndJointBilateralConstraint)

	D_NEWTON_API ndJointFixDistance();
	D_NEWTON_API ndJointFixDistance(const ndVector& childPivotInGlobalSpace, const ndVector& parentPivotInGlobalSpace, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointFixDistance();

	D_NEWTON_API ndFloat32 GetDistance() const;
	D_NEWTON_API void SetDistance(ndFloat32 dist);

	protected:
	void JacobianDerivative(ndConstraintDescritor& desc);

	ndFloat32 m_distance;
} D_GCC_NEWTON_CLASS_ALIGN_32;

#endif 

