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


#ifndef __ND_JOINT_DRY_ROLLING_FRICTION_H_
#define __ND_JOINT_DRY_ROLLING_FRICTION_H_

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

// this joint is usefully to simulate the rolling friction of a rolling ball over 
// a flat surface.
// normally this is not important for non spherical objects, but for games like 
// poll, pinball, bolling, golf or any other where the movement of balls is the main objective
// the rolling friction is a real big problem.
D_MSV_NEWTON_CLASS_ALIGN_32
class ndJointDryRollingFriction: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointDryRollingFriction, ndJointBilateralConstraint)

	D_NEWTON_API ndJointDryRollingFriction();
	D_NEWTON_API ndJointDryRollingFriction(ndBodyKinematic* const body0, ndBodyKinematic* const body1, ndFloat32 coefficient);
	D_NEWTON_API virtual ~ndJointDryRollingFriction();

	D_NEWTON_API void SetContactTrail(ndFloat32 trail);
	D_NEWTON_API void SetFrictionCoefficient(ndFloat32 friction);
	
	D_NEWTON_API ndFloat32 GetContactTrail() const;
	D_NEWTON_API ndFloat32 GetFrictionCoefficient() const;

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);

	ndFloat32 m_coefficient;
	ndFloat32 m_contactTrail;
} D_GCC_NEWTON_CLASS_ALIGN_32;

#endif 

