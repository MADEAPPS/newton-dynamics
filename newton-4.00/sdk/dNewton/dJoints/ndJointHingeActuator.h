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

#ifndef __ND_JOINT_HINGE_ACTUATOR_H__
#define __ND_JOINT_HINGE_ACTUATOR_H__

#include "ndNewtonStdafx.h"
#include "ndJointHinge.h"

class ndJointHingeActuator: public ndJointHinge
{
	public:
	D_CLASS_REFLECTION(ndJointHingeActuator);
	D_NEWTON_API ndJointHingeActuator(const dLoadSaveBase::dLoadDescriptor& desc);
	//D_NEWTON_API ndJointHingeActuator(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API ndJointHingeActuator(const dMatrix& pinAndPivotFrame, dFloat32 angularRate, dFloat32 minAngle, dFloat32 maxAngle, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointHingeActuator();

	D_NEWTON_API dFloat32 GetTargetAngle() const;
	D_NEWTON_API void SetTargetAngle(dFloat32 angle);

	D_NEWTON_API dFloat32 GetMinAngularLimit() const;
	D_NEWTON_API dFloat32 GetMaxAngularLimit() const;

	D_NEWTON_API void SetMinAngularLimit(dFloat32 limit);
	D_NEWTON_API void SetMaxAngularLimit(dFloat32 limit);
	
	D_NEWTON_API dFloat32 GetAngularRate() const;
	D_NEWTON_API void SetAngularRate(dFloat32 rate);
	
    D_NEWTON_API dFloat32 GetMaxTorque() const;
    D_NEWTON_API void SetMaxTorque(dFloat32 torque);
	
	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const dLoadSaveBase::dSaveDescriptor& desc) const;
	
	dFloat32 m_targetAngle;
	dFloat32 m_motorSpeed;
	dFloat32 m_maxTorque;
};

#endif
