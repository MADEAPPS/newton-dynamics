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

#ifndef __ND_JOINT_FIX_6DOF_H_
#define __ND_JOINT_FIX_6DOF_H_

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndJointFix6dof: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointFix6dof);
	D_NEWTON_API ndJointFix6dof(const ndLoadSaveBase::dLoadDescriptor& desc);
	D_NEWTON_API ndJointFix6dof(const ndMatrix& frameInGlbalSpace, ndBodyKinematic* const body0, ndBodyKinematic* const body1);
	D_NEWTON_API virtual ~ndJointFix6dof();

	D_NEWTON_API void SetAsSoftJoint(bool mode);
	D_NEWTON_API void SetRegularizer(ndFloat32 regularizer);

	private:
	void JacobianDerivative(ndConstraintDescritor& desc);
	void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	void SubmitAngularAxis(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);
	void SubmitAngularAxisCartisianApproximation(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);

	ndFloat32 m_softness;
	ndFloat32 m_maxForce;
	ndFloat32 m_maxTorque;
};
#endif 

