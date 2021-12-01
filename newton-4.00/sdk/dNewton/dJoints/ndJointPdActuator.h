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

#ifndef __ND_JOINT_PID_3DOF_ACTUATOR_H__
#define __ND_JOINT_PID_3DOF_ACTUATOR_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndJointPdActuator : public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointPdActuator);
	D_NEWTON_API ndJointPdActuator(const dLoadSaveBase::dLoadDescriptor& desc);
	D_NEWTON_API ndJointPdActuator(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointPdActuator();

	D_NEWTON_API void SetTwistLimits(dFloat32 minAngle, dFloat32 maxAngle);
	D_NEWTON_API void GetTwistLimits(dFloat32& minAngle, dFloat32& maxAngle) const;
	D_NEWTON_API void GetTwistAngleSpringDamperRegularizer(dFloat32& spring, dFloat32& damper, dFloat32& regularizer) const;
	D_NEWTON_API void SetTwistAngleSpringDamperRegularizer(dFloat32 spring, dFloat32 damper, dFloat32 regularizer = dFloat32(5.0e-3f));

	D_NEWTON_API dFloat32 GetMaxConeAngle() const;
	D_NEWTON_API void SetConeLimit(dFloat32 maxConeAngle);
	D_NEWTON_API void GetConeAngleSpringDamperRegularizer(dFloat32& spring, dFloat32& damper, dFloat32& regularizer) const;
	D_NEWTON_API void SetConeAngleSpringDamperRegularizer(dFloat32 spring, dFloat32 damper, dFloat32 regularizer = dFloat32(5.0e-3f));

	D_NEWTON_API void GetLinearSpringDamperRegularizer(dFloat32& spring, dFloat32& damper, dFloat32& regularizer) const;
	D_NEWTON_API void SetLinearSpringDamperRegularizer(dFloat32 spring, dFloat32 damper, dFloat32 regularizer = dFloat32(5.0e-3f));

	D_NEWTON_API dVector GetTargetPosition() const;
	D_NEWTON_API void SetTargetPosition(const dVector& posit);

	D_NEWTON_API dMatrix GetTargetMatrix() const;
	D_NEWTON_API void SetTargetMatrix(const dMatrix& posit);

	const dMatrix& GetReferenceMatrix() const;

	dMatrix GetTargetRotation() const;
	void SetTargetRotation(const dMatrix& rotation);

	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& debugCallback) const;

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const dLoadSaveBase::dSaveDescriptor& desc) const;


	//void SubmitTwistLimits(const dVector& pin, dFloat32 angle, ndConstraintDescritor& desc);
	//void SubmitPdRotation(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc);

	void SubmitLinearLimits(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc);
	void SubmitAngularAxis(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc);
	void SubmitAngularAxisCartesianApproximation(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc);

	void SubmitConeAngleOnlyRows(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc);
	void SubmitTwistAngleOnlyRows(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc);

	dMatrix m_pivotFrame;
	
	dFloat32 m_minTwistAngle;
	dFloat32 m_maxTwistAngle;
	dFloat32 m_twistAngleSpring;
	dFloat32 m_twistAngleDamper;
	dFloat32 m_twistAngleRegularizer;

	dFloat32 m_maxConeAngle;
	dFloat32 m_coneAngleSpring;
	dFloat32 m_coneAngleDamper;
	dFloat32 m_coneAngleRegularizer;

	dFloat32 m_linearSpring;
	dFloat32 m_linearDamper;
	dFloat32 m_linearRegularizer;
};

inline dMatrix ndJointPdActuator::GetTargetRotation() const
{
	dMatrix tmp(m_localMatrix1);
	tmp.m_posit = m_pivotFrame.m_posit;
	return tmp;
}

inline void ndJointPdActuator::SetTargetRotation(const dMatrix& matrix)
{
	dMatrix tmp(matrix);
	tmp.m_posit = m_localMatrix1.m_posit;
	m_localMatrix1 = tmp;
}

inline const dMatrix& ndJointPdActuator::GetReferenceMatrix() const
{
	return m_pivotFrame;
}

#endif 

