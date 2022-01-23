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
	D_NEWTON_API ndJointHingeActuator(const ndLoadSaveBase::ndLoadDescriptor& desc);
	//D_NEWTON_API ndJointHingeActuator(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API ndJointHingeActuator(const ndMatrix& pinAndPivotFrame, ndFloat32 angularRate, ndFloat32 minAngle, ndFloat32 maxAngle, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointHingeActuator();

	D_NEWTON_API virtual ndFloat32 GetTargetAngle() const;
	D_NEWTON_API virtual void SetTargetAngle(ndFloat32 angle);

	D_NEWTON_API virtual ndFloat32 GetMinAngularLimit() const;
	D_NEWTON_API virtual ndFloat32 GetMaxAngularLimit() const;

	D_NEWTON_API virtual void SetMinAngularLimit(ndFloat32 limit);
	D_NEWTON_API virtual void SetMaxAngularLimit(ndFloat32 limit);
	
	D_NEWTON_API virtual ndFloat32 GetAngularRate() const;
	D_NEWTON_API virtual void SetAngularRate(ndFloat32 rate);
	
    D_NEWTON_API virtual ndFloat32 GetMaxTorque() const;
    D_NEWTON_API virtual void SetMaxTorque(ndFloat32 torque);
	
	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;
	
	ndFloat32 m_targetAngle;
	ndFloat32 m_motorSpeed;
	ndFloat32 m_maxTorque;
};

#endif
