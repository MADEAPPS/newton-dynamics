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

#ifndef __D_JOINT_DOUBLE_HINGE_H__
#define __D_JOINT_DOUBLE_HINGE_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndJointDoubleHinge: public ndJointBilateralConstraint
{
	public:
	ND_CLASS_RELECTION(ndJointDoubleHinge);
	D_NEWTON_API ndJointDoubleHinge(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointDoubleHinge();

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);

	dFloat32 m_jointAngle0;
	dFloat32 m_jointSpeed0;
	dFloat32 m_jointAngle1;
	dFloat32 m_jointSpeed1;
};

#endif 

