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

#ifndef __D_ANIMATION_END_EFECTOR_H__
#define __D_ANIMATION_END_EFECTOR_H__


#include "dAnimationLoopJoint.h"


class dAnimationRagDollEffector: public dAnimationLoopJoint
{
	public:
	dAnimationRagDollEffector(dAnimationJoint* const root);

	dMatrix GetMatrix() const;
	void SetTarget(const dMatrix& targetMatrix);

	virtual int GetMaxDof() const {return 6;}

	protected:
	dFloat ClipParam(dFloat value, dFloat maxValue) const;
	virtual void JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams);
	virtual void UpdateSolverForces(const dComplementaritySolver::dJacobianPair* const jacobians) const
	{
		dAssert(0);
	}

	dMatrix m_localMatrix;
	dMatrix m_targetMatrix;
	dFloat m_maxLinearSpeed;
	dFloat m_maxAngularSpeed;
	dFloat m_linearFriction;
	dFloat m_angularFriction;
};
#endif

