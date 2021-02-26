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

#ifndef __D_JOINT_DEBRIS_LINK_H_
#define __D_JOINT_DEBRIS_LINK_H_

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndJointFixDebrisLink: public ndJointBilateralConstraint
{
	public:
	D_NEWTON_API ndJointFixDebrisLink(ndBodyKinematic* const body0, ndBodyKinematic* const body1);
	D_NEWTON_API virtual ~ndJointFixDebrisLink();

	private:
	void JacobianDerivative(ndConstraintDescritor& desc);

	void SubmitAngularAxis(ndConstraintDescritor& desc, const dMatrix& matrix0, const dMatrix& matrix1);
	void SubmitAngularAxisCartisianApproximation(ndConstraintDescritor& desc, const dMatrix& matrix0, const dMatrix& matrix1);

	dFloat32 m_distance;
};
#endif 

