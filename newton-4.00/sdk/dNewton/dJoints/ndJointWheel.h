/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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

class ndJointWheel: public ndJointBilateralConstraint
{
	public:
	class ndWheelDescriptor
	{
		public:
		dFloat32 m_springK;
		dFloat32 m_damperC;
		dFloat32 m_minLimit;
		dFloat32 m_maxLimit;
		dFloat32 m_laterialStiffeness;
		dFloat32 m_longitudinalStiffeness;
	};

	ND_JOINT_RELECTION(ndJointWheel);
	D_NEWTON_API ndJointWheel(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent, const ndWheelDescriptor& desc);
	D_NEWTON_API virtual ~ndJointWheel();

	D_NEWTON_API void SetBrakeTorque(dFloat32 torque);
	D_NEWTON_API void SetSteeringAngle(dFloat32 steeringAngle);

	const ndWheelDescriptor& GetInfo() const
	{
		return m_info;
	}

	void SetInfo(const ndWheelDescriptor& info)
	{
		m_info = info;
	}
	
	private:
	void SubmitConstraintLimitSpringDamper(ndConstraintDescritor& desc, const dMatrix& matrix0, const dMatrix& matrix1);

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);

	dMatrix m_baseFrame;
	ndWheelDescriptor m_info;
	dFloat32 m_posit;
	dFloat32 m_speed;
	dFloat32 m_brakeTorque;
};

#endif 

