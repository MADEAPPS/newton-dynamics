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
	D_CLASS_REFLECTION(ndJointPid6dofActuator);
	D_NEWTON_API ndJointPid6dofActuator(const dLoadSaveBase::dLoadDescriptor& desc);
	D_NEWTON_API ndJointPid6dofActuator(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointPid6dofActuator();

	D_NEWTON_API void GetLinearSpringDamperRegularizer(dFloat32& spring, dFloat32& damper, dFloat32& regularizer) const;
	D_NEWTON_API void SetLinearSpringDamperRegularizer(dFloat32 spring, dFloat32 damper, dFloat32 regularizer = dFloat32 (5.0e-3f));

	dVector GetTargetPosition() const;
	void SetTargetPosition(const dVector& posit);

	dMatrix GetTargetMatrix() const;
	void SetTargetMatrix(const dMatrix& posit);

	protected:
	virtual void SubmitLinearLimits(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const dLoadSaveBase::dSaveDescriptor& desc) const;

	dFloat32 m_linearSpring;
	dFloat32 m_linearDamper;
	dFloat32 m_linearRegularizer;
};

inline dVector ndJointPid6dofActuator::GetTargetPosition() const
{
	return m_localMatrix1.m_posit;
}

inline void ndJointPid6dofActuator::SetTargetPosition(const dVector& posit)
{
	dAssert(posit.m_w == dFloat32(1.0f));
	m_localMatrix1.m_posit = posit;
}

inline dMatrix ndJointPid6dofActuator::GetTargetMatrix() const
{
	return m_localMatrix1;
}

inline void ndJointPid6dofActuator::SetTargetMatrix(const dMatrix& matrix)
{
	m_localMatrix1 = matrix;
}

#endif 

