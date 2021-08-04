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
	D_NEWTON_API void SetConeFriction(dFloat32 regularizer, dFloat32 viscousFriction);

	D_NEWTON_API void SetTwistLimits(dFloat32 minAngle, dFloat32 maxAngle);
	D_NEWTON_API void GetTwistLimits(dFloat32& minAngle, dFloat32& maxAngle) const;
	D_NEWTON_API void SetTwistFriction(dFloat32 regularizer, dFloat32 viscousFriction);

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& debugCallback) const;

	void SubmitTwistAngle(const dVector& pin, dFloat32 angle, ndConstraintDescritor& desc);
	void SubmitAngularAxis(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc);
	void SubmitPidRotation(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc);
	virtual void SubmitLinearLimits(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc);
	void SubmitAngularAxisCartesianApproximation(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc);

	dMatrix m_baseMatrix;
	dFloat32 m_targetPitch;
	dFloat32 m_targetYaw;
	dFloat32 m_targetRoll;
	dFloat32 m_maxConeAngle;
	dFloat32 m_minTwistAngle;
	dFloat32 m_maxTwistAngle;
	dFloat32 m_integralError;
};

#endif 

