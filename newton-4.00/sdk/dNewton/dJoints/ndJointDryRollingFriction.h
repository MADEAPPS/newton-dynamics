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


#ifndef __D_JOINT_DRY_ROLLING_FRICTION_H_
#define __D_JOINT_DRY_ROLLING_FRICTION_H_

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"


// this joint is usefully to simulate the rolling friction of a rolling ball over 
// a flat surface.
// normally this is not important for non spherical objects, but for games like 
// poll, pinball, bolling, golf or any other where the movement of balls is the main objective
// the rolling friction is a real big problem.
class ndJointDryRollingFriction: public ndJointBilateralConstraint
{
	public:
	ND_JOINT_RELECTION(ndJointDryRollingFriction);

	D_NEWTON_API ndJointDryRollingFriction(ndBodyKinematic* const body0, ndBodyKinematic* const body1, dFloat32 coefficient);
	D_NEWTON_API virtual ~ndJointDryRollingFriction();

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);

	dFloat32 m_coefficient;
	dFloat32 m_contactTrail;
};

#endif 

