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

#ifndef __D_JOINT_INVERSE_DYNAMICS_BASE_H__
#define __D_JOINT_INVERSE_DYNAMICS_BASE_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndJointInverseDynamicsBase: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointInverseDynamicsBase);
	D_NEWTON_API ndJointInverseDynamicsBase(const dLoadSaveBase::dLoadDescriptor& desc);
	D_NEWTON_API ndJointInverseDynamicsBase(dInt32 dof, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointInverseDynamicsBase();

	virtual void SetTargetLocalMatrix(const dMatrix& localMatrix);
	virtual void SetTargetGlobalMatrix(const dMatrix& globalMatrix);
	protected:
	D_NEWTON_API void Save(const dLoadSaveBase::dSaveDescriptor& desc) const;
};

inline void ndJointInverseDynamicsBase::SetTargetLocalMatrix(const dMatrix&)
{
	dAssert(0);
}

inline void ndJointInverseDynamicsBase::SetTargetGlobalMatrix(const dMatrix&)
{
	dAssert(0);
}

#endif 

