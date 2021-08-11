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

#ifndef __D_JOINT_PID_3DOF_ACTUATOR_H__
#define __D_JOINT_PID_3DOF_ACTUATOR_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

#define D_PID_MAX_ANGLE	dFloat32 (120.0f * dDegreeToRad)
#define D_PID_PENETRATION_RECOVERY_ANGULAR_SPEED dFloat32 (0.1f) 
#define D_PID_PENETRATION_ANGULAR_LIMIT dFloat32 (10.0f * dDegreeToRad) 

class ndJointPid3dofActuator : public ndJointBilateralConstraint
{
	public:
	D_CLASS_RELECTION(ndJointPid3dofActuator);

	D_NEWTON_API ndJointPid3dofActuator(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointPid3dofActuator();

	D_NEWTON_API dFloat32 GetMaxConeAngle() const;
	D_NEWTON_API void SetConeLimit(dFloat32 maxConeAngle);
	
	D_NEWTON_API void SetTwistLimits(dFloat32 minAngle, dFloat32 maxAngle);
	D_NEWTON_API void GetTwistLimits(dFloat32& minAngle, dFloat32& maxAngle) const;
	
	D_NEWTON_API void GetAngularSpringDamperRegularizer(dFloat32& spring, dFloat32& damper, dFloat32& regularizer) const;
	D_NEWTON_API void SetAngularSpringDamperRegularizer(dFloat32 spring, dFloat32 damper, dFloat32 regularizer = dFloat32(5.0e-3f));

	void SetTargetRotation(dFloat32 pitch, dFloat32 yaw, dFloat32 roll);
	void GetTargetRotation(dFloat32& pitch, dFloat32& yaw, dFloat32& roll) const;

	D_NEWTON_API dMatrix CalculateGlobalTargetMatrix() const;

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& debugCallback) const;

	void SubmitTwistLimits(const dVector& pin, dFloat32 angle, ndConstraintDescritor& desc);
	void SubmitAngularAxis(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc);
	void SubmitPidRotation(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc);
	void SubmitAngularAxisCartesianApproximation(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc);

	virtual void SubmitLinearLimits(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc);

	dMatrix m_referenceFrameBody1;
	dVector m_targetPosition;
	dFloat32 m_targetPitch;
	dFloat32 m_targetYaw;
	dFloat32 m_targetRoll;
	dFloat32 m_maxConeAngle;
	dFloat32 m_minTwistAngle;
	dFloat32 m_maxTwistAngle;

	dFloat32 m_angularSpring;
	dFloat32 m_angularDamper;
	dFloat32 m_angularRegularizer;
};

inline void ndJointPid3dofActuator::SetTargetRotation(dFloat32 pitch, dFloat32 yaw, dFloat32 roll)
{
	m_targetPitch = pitch;
	m_targetYaw = yaw;
	m_targetRoll = roll;
}

inline void ndJointPid3dofActuator::GetTargetRotation(dFloat32& pitch, dFloat32& yaw, dFloat32& roll) const 
{
	pitch = m_targetPitch;
	yaw = m_targetYaw;
	roll = m_targetRoll;
}


#endif 

