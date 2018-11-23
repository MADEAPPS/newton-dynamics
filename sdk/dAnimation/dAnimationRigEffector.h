/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __D_ANIMATION_RIG_EFFECTOR_H__
#define __D_ANIMATION_RIG_EFFECTOR_H__

#include "dAnimationRigJoint.h"
#include "dAnimationKinematicLoopJoint.h"

class dAnimationRigLimb;

class dAnimationRigEffector: public dAnimationKinematicLoopJoint
{
	public:
	dAnimationRigEffector(dAnimationRigLimb* const parent);
	virtual ~dAnimationRigEffector();

	virtual int GetMaxDof() const {return 3;}
	virtual void JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams) { dAssert(0); }
	virtual void UpdateSolverForces(const dComplementaritySolver::dJacobianPair* const jacobians) const { dAssert(0); }

};

#endif

