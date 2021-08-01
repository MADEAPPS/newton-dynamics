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

#ifndef __D_JOINT_FIX_6DOF_H_
#define __D_JOINT_FIX_6DOF_H_

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndJointFix6dof: public ndJointBilateralConstraint
{
	public:
	D_CLASS_RELECTION(ndJointFix6dof);
	D_NEWTON_API ndJointFix6dof(const dMatrix& frameInGlbalSpace, ndBodyKinematic* const body0, ndBodyKinematic* const body1);
	D_NEWTON_API virtual ~ndJointFix6dof();

	D_NEWTON_API void SetAsSoftJoint(bool mode);
	D_NEWTON_API void SetRegularizer(dFloat32 regularizer);

	private:
	void JacobianDerivative(ndConstraintDescritor& desc);

	void SubmitAngularAxis(ndConstraintDescritor& desc, const dMatrix& matrix0, const dMatrix& matrix1);
	void SubmitAngularAxisCartisianApproximation(ndConstraintDescritor& desc, const dMatrix& matrix0, const dMatrix& matrix1);

	dFloat32 m_softness;
	dFloat32 m_maxForce;
	dFloat32 m_maxTorque;
};
#endif 

