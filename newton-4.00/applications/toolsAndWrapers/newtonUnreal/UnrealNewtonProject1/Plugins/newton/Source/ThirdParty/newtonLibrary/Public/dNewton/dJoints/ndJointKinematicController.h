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

#ifndef __ND_JOINT_KINEMATIC_CONTROLLER_H__
#define __ND_JOINT_KINEMATIC_CONTROLLER_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndJointKinematicController: public ndJointBilateralConstraint
{
	public:
	enum ndControlModes
	{	
		m_linear,
		m_full6dof,
		m_linearPlusAngularFriction, // for pick mode from screen
	};

	D_CLASS_REFLECTION(ndJointKinematicController, ndJointBilateralConstraint)

	D_NEWTON_API ndJointKinematicController();
	D_NEWTON_API ndJointKinematicController(ndBodyKinematic* const referenceBody, ndBodyKinematic* const body, const ndVector& attachmentPointInGlobalSpace);
	D_NEWTON_API ndJointKinematicController(ndBodyKinematic* const referenceBody, ndBodyKinematic* const body, const ndMatrix& attachmentMatrixInGlobalSpace);
	D_NEWTON_API virtual ~ndJointKinematicController();

	virtual bool IsBilateral() const;
	void SetControlMode(ndControlModes mode);
	void SetMaxSpeed(ndFloat32 speedInMetersPerSeconds);
	void SetMaxOmega(ndFloat32 speedInRadiansPerSeconds);
	void SetMaxLinearFriction(ndFloat32 force);
	void SetMaxAngularFriction(ndFloat32 torque);
	void SetAngularViscousFrictionCoefficient(ndFloat32 coefficient);

	ndFloat32 GetMaxSpeed() const;
	ndFloat32 GetMaxOmega() const;
	ndControlModes GetControlMode() const;
	ndFloat32 GetMaxLinearFriction() const;
	ndFloat32 GetMaxAngularFriction() const;
	ndFloat32 GetAngularViscousFrictionCoefficient() const;

	ndMatrix GetTargetMatrix() const;
	void SetTargetPosit(const ndVector& posit);
	void SetTargetRotation(const ndQuaternion& rotation);

	D_NEWTON_API void SetTargetMatrix(const ndMatrix& matrix);

	protected:
	void Init(const ndMatrix& matrix);

	D_NEWTON_API void CheckSleep() const;
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);

	private:
	ndFloat32 m_maxSpeed;
	ndFloat32 m_maxOmega;
	ndFloat32 m_maxLinearFriction;
	ndFloat32 m_maxAngularFriction;
	ndFloat32 m_angularFrictionCoefficient;
	ndControlModes m_controlMode;
	bool m_autoSleepState;
};

inline void ndJointKinematicController::SetControlMode(ndControlModes mode)
{
	m_controlMode = mode;
}

inline void ndJointKinematicController::SetMaxSpeed(ndFloat32 speedInMetersPerSeconds)
{
	m_maxSpeed = ndAbs(speedInMetersPerSeconds);
}

inline void ndJointKinematicController::SetMaxLinearFriction(ndFloat32 frictionForce)
{
	m_maxLinearFriction = ndAbs(frictionForce);
}

inline void ndJointKinematicController::SetMaxAngularFriction(ndFloat32 frictionTorque)
{
	m_maxAngularFriction = ndAbs(frictionTorque);
}

inline void ndJointKinematicController::SetMaxOmega(ndFloat32 speedInRadiansPerSeconds)
{
	m_maxOmega = ndAbs(speedInRadiansPerSeconds);
}

inline void ndJointKinematicController::SetAngularViscousFrictionCoefficient(ndFloat32 coefficient)
{
	ndVector mass (GetBody0()->GetMassMatrix());
	m_angularFrictionCoefficient = ndAbs(coefficient) * ndMax(mass.m_x, ndMax(mass.m_y, mass.m_z));
}

inline void ndJointKinematicController::SetTargetPosit(const ndVector& posit)
{
	ndMatrix matrix(m_localMatrix1);
	matrix.m_posit = posit;
	SetTargetMatrix(matrix);
}

inline void ndJointKinematicController::SetTargetRotation(const ndQuaternion& rotation)
{
	ndMatrix matrix(ndCalculateMatrix(rotation, m_localMatrix1.m_posit));
	SetTargetMatrix(matrix);
}

inline ndMatrix ndJointKinematicController::GetTargetMatrix() const
{
	ndAssert(0);
	return m_localMatrix0;
}

inline bool ndJointKinematicController::IsBilateral() const
{
	return true;
}

inline ndFloat32 ndJointKinematicController::GetMaxSpeed() const
{
	return m_maxSpeed;
}

inline ndFloat32 ndJointKinematicController::GetMaxOmega() const
{
	return m_maxOmega;
}

inline ndJointKinematicController::ndControlModes ndJointKinematicController::GetControlMode() const
{
	return m_controlMode;
}

inline ndFloat32 ndJointKinematicController::GetMaxLinearFriction() const
{
	return m_maxLinearFriction;
}

inline ndFloat32 ndJointKinematicController::GetMaxAngularFriction() const
{
	return m_maxAngularFriction;
}

inline ndFloat32 ndJointKinematicController::GetAngularViscousFrictionCoefficient() const
{
	return m_angularFrictionCoefficient;
}

#endif 

