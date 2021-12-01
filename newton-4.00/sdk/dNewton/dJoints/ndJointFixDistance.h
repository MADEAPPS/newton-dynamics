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

#ifndef __ND_JOINT_FIX_DISTANCE_H_
#define __ND_JOINT_FIX_DISTANCE_H_

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndJointFixDistance: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointFixDistance);
	D_NEWTON_API ndJointFixDistance(const dLoadSaveBase::dLoadDescriptor& desc);
	D_NEWTON_API ndJointFixDistance(const dVector& childPivotInGlobalSpace, const dVector& parentPivotInGlobalSpace, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointFixDistance();

	protected:
	void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const dLoadSaveBase::dSaveDescriptor& desc) const;

	dFloat32 m_distance;
};
#endif 

