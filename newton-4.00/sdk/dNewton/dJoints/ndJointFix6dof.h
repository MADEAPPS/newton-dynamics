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

#ifndef __ND_JOINT_FIX_6DOF_H_
#define __ND_JOINT_FIX_6DOF_H_

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

D_MSV_NEWTON_CLASS_ALIGN_32
class ndJointFix6dof: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointFix6dof, ndJointBilateralConstraint)

	D_NEWTON_API ndJointFix6dof();
	D_NEWTON_API ndJointFix6dof(const ndMatrix& frameInGlobalSpace, ndBodyKinematic* const body0, ndBodyKinematic* const body1);
	D_NEWTON_API ndJointFix6dof(ndBodyKinematic* const body0, ndBodyKinematic* const body1, const ndMatrix& globalMatrixBody0, const ndMatrix& globalMatrixBody1 );
	D_NEWTON_API virtual ~ndJointFix6dof();

	D_NEWTON_API void SetAsSoftJoint(bool mode);

	D_NEWTON_API ndFloat32 GetRegularizer() const;
	D_NEWTON_API void SetRegularizer(ndFloat32 regularizer);

	D_NEWTON_API ndFloat32 GetMaxForce() const;
	D_NEWTON_API ndFloat32 GetMaxTorque() const;

	private:
	void JacobianDerivative(ndConstraintDescritor& desc);
	void SubmitAngularAxis(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);
	void SubmitAngularAxisCartisianApproximation(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);

	ndFloat32 m_softness;
	ndFloat32 m_maxForce;
	ndFloat32 m_maxTorque;
} D_GCC_NEWTON_CLASS_ALIGN_32;

#endif 

