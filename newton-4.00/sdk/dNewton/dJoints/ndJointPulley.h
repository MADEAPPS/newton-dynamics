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

#ifndef __D_JOINT_PULLEY_H__
#define __D_JOINT_PULLEY_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"


class ndJointPulley: public ndJointBilateralConstraint
{
	public:
	ND_CLASS_RELECTION(ndJointPulley);
	D_NEWTON_API ndJointPulley(dFloat32 gearRatio,
		const dVector& body0Pin, ndBodyKinematic* const body0,
		const dVector& body1Pin, ndBodyKinematic* const body1);
	D_NEWTON_API virtual ~ndJointPulley();

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);

	dFloat32 m_gearRatio;
};

#endif 

