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

D_MSV_NEWTON_CLASS_ALIGN_32
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

	D_NEWTON_API virtual bool IsBilateral() const;
	D_NEWTON_API void SetControlMode(ndControlModes mode);
	D_NEWTON_API void SetMaxSpeed(ndFloat32 speedInMetersPerSeconds);
	D_NEWTON_API void SetMaxOmega(ndFloat32 speedInRadiansPerSeconds);
	D_NEWTON_API void SetMaxLinearFriction(ndFloat32 force);
	D_NEWTON_API void SetMaxAngularFriction(ndFloat32 torque);
	D_NEWTON_API void SetAngularViscousFrictionCoefficient(ndFloat32 coefficient);

	D_NEWTON_API ndFloat32 GetMaxSpeed() const;
	D_NEWTON_API ndFloat32 GetMaxOmega() const;
	D_NEWTON_API ndControlModes GetControlMode() const;
	D_NEWTON_API ndFloat32 GetMaxLinearFriction() const;
	D_NEWTON_API ndFloat32 GetMaxAngularFriction() const;
	D_NEWTON_API ndFloat32 GetAngularViscousFrictionCoefficient() const;

	D_NEWTON_API void SetTargetPosit(const ndVector& posit);
	D_NEWTON_API void SetTargetRotation(const ndQuaternion& rotation);

	D_NEWTON_API ndMatrix GetTargetMatrix() const;
	D_NEWTON_API void SetTargetMatrix(const ndMatrix& matrix);

	protected:
	D_NEWTON_API void CheckSleep() const;
	D_NEWTON_API void Init(const ndMatrix& matrix);
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);

	private:
	ndFloat32 m_maxSpeed;
	ndFloat32 m_maxOmega;
	ndFloat32 m_maxLinearFriction;
	ndFloat32 m_maxAngularFriction;
	ndFloat32 m_angularFrictionCoefficient;
	ndControlModes m_controlMode;
	bool m_autoSleepState;
} D_GCC_NEWTON_CLASS_ALIGN_32;



#endif 

