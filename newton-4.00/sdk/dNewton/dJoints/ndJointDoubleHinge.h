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

#ifndef __ND_JOINT_DOUBLE_HINGE_H__
#define __ND_JOINT_DOUBLE_HINGE_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndJointDoubleHinge: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointDoubleHinge);
	D_NEWTON_API ndJointDoubleHinge(const dLoadSaveBase::dLoadDescriptor& desc);
	D_NEWTON_API ndJointDoubleHinge(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointDoubleHinge();

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const dLoadSaveBase::dSaveDescriptor& desc) const;

	dFloat32 m_angle0;
	dFloat32 m_omega0;
	dFloat32 m_angle1;
	dFloat32 m_omega1;
};

#endif 

