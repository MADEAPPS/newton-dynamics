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

#ifndef __D_JOINT_WHEEL_H__
#define __D_JOINT_WHEEL_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndWheelDescriptor
{
	public:
	ndWheelDescriptor()
		:m_springK(dFloat32(1.0f))
		,m_damperC(dFloat32(0.0f))
		,m_minLimit(dFloat32(-0.1f))
		,m_maxLimit(dFloat32(0.2f))
		,m_regularizer(dFloat32(0.1f))
		,m_brakeTorque(dFloat32(0.0f))
		,m_handBrakeTorque(dFloat32(0.0f))
		,m_steeringAngle(dFloat32(0.0f))
		,m_laterialStiffness (dFloat32(0.5f))
		,m_longitudinalStiffness (dFloat32(0.5f))
	{
	}
	
	dFloat32 m_springK;
	dFloat32 m_damperC;
	dFloat32 m_minLimit;
	dFloat32 m_maxLimit;
	dFloat32 m_regularizer;
	dFloat32 m_brakeTorque;
	dFloat32 m_handBrakeTorque;
	dFloat32 m_steeringAngle;
	dFloat32 m_laterialStiffness;
	dFloat32 m_longitudinalStiffness;
};

class ndJointWheel: public ndJointBilateralConstraint
{
	public:

	D_CLASS_RELECTION(ndJointWheel);
	D_NEWTON_API ndJointWheel(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent, const ndWheelDescriptor& desc);
	D_NEWTON_API virtual ~ndJointWheel();

	D_NEWTON_API void SetBrake(dFloat32 normalizedTorque);
	D_NEWTON_API void SetHandBrake(dFloat32 normalizedTorque);
	D_NEWTON_API void SetSteering(dFloat32 normalidedSteering);

	D_NEWTON_API dMatrix CalculateUpperBumperMatrix() const;

	const ndWheelDescriptor& GetInfo() const
	{
		return m_info;
	}

	void SetInfo(const ndWheelDescriptor& info)
	{
		m_info = info;
	}
	
	protected:
	void SetSteeringAngle();
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);

	dMatrix m_baseFrame;
	ndWheelDescriptor m_info;
	dFloat32 m_posit;
	dFloat32 m_speed;
	dFloat32 m_regularizer;
	dFloat32 m_normalizedBrake;
	dFloat32 m_normalidedSteering;
	dFloat32 m_normalizedHandBrake;
};

#endif 

