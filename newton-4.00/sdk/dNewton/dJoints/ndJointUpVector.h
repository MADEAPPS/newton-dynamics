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

#ifndef __D_JOINT_UPVECTOR_H__
#define __D_JOINT_UPVECTOR_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

// This joint is useful to for implementing character controllers, and also precise object picking
class ndJointUpVector: public ndJointBilateralConstraint
{
	public:
	D_CLASS_RELECTION(ndJointUpVector);

	D_NEWTON_API ndJointUpVector(const dVector& normal, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointUpVector();

	D_NEWTON_API void SetPinDir (const dVector& pin);

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
};

#endif

