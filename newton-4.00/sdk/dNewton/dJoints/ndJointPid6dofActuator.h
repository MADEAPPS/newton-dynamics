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

#ifndef __D_JOINT_PID_6DOF_ACTUATOR_H__
#define __D_JOINT_PID_6DOF_ACTUATOR_H__

#include "ndNewtonStdafx.h"
#include "ndJointPid3dofActuator.h"

class ndJointPid6dofActuator : public ndJointPid3dofActuator
{
	public:
	D_CLASS_RELECTION(ndJointPid6dofActuator);

	D_NEWTON_API ndJointPid6dofActuator(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointPid6dofActuator();

	D_NEWTON_API void GetLinearSpringDamperRegularizer(dFloat32& spring, dFloat32& damper, dFloat32& regularizer) const;
	D_NEWTON_API void SetLinearSpringDamperRegularizer(dFloat32 spring, dFloat32 damper, dFloat32 regularizer = dFloat32 (5.0e-3f));

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	virtual void SubmitLinearLimits(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc);

	dFloat32 m_linearSpring;
	dFloat32 m_linearDamper;
	dFloat32 m_linearRegularizer;

	dFloat32 xxxx;
};

#endif 

