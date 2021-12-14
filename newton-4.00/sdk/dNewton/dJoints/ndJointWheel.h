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

#ifndef __ND_JOINT_WHEEL_H__
#define __ND_JOINT_WHEEL_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndWheelDescriptor
{
	public:
	ndWheelDescriptor()
		:m_springK(ndFloat32(1.0f))
		,m_damperC(ndFloat32(0.0f))
		,m_upperStop(ndFloat32(-0.1f))
		,m_lowerStop(ndFloat32(0.2f))
		,m_regularizer(ndFloat32(0.1f))
		,m_brakeTorque(ndFloat32(0.0f))
		,m_handBrakeTorque(ndFloat32(0.0f))
		,m_steeringAngle(ndFloat32(0.0f))
		,m_laterialStiffness (ndFloat32(0.5f))
		,m_longitudinalStiffness (ndFloat32(0.5f))
	{
	}
	
	D_NEWTON_API void Save(nd::TiXmlNode* const xmlNode) const;
	D_NEWTON_API void Load(const nd::TiXmlNode* const xmlNode);
	
	ndFloat32 m_springK;
	ndFloat32 m_damperC;
	ndFloat32 m_upperStop;
	ndFloat32 m_lowerStop;
	ndFloat32 m_regularizer;
	ndFloat32 m_brakeTorque;
	ndFloat32 m_handBrakeTorque;
	ndFloat32 m_steeringAngle;
	ndFloat32 m_laterialStiffness;
	ndFloat32 m_longitudinalStiffness;
};

class ndJointWheel: public ndJointBilateralConstraint
{
	public:

	D_CLASS_REFLECTION(ndJointWheel);
	D_NEWTON_API ndJointWheel(const ndLoadSaveBase::dLoadDescriptor& desc);
	D_NEWTON_API ndJointWheel(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent, const ndWheelDescriptor& desc);
	D_NEWTON_API virtual ~ndJointWheel();

	D_NEWTON_API void SetBrake(ndFloat32 normalizedTorque);
	D_NEWTON_API void SetHandBrake(ndFloat32 normalizedTorque);
	D_NEWTON_API void SetSteering(ndFloat32 normalidedSteering);
	
	D_NEWTON_API void UpdateTireSteeringAngleMatrix();
	D_NEWTON_API ndMatrix CalculateUpperBumperMatrix() const;

	const ndWheelDescriptor& GetInfo() const
	{
		return m_info;
	}

	void SetInfo(const ndWheelDescriptor& info)
	{
		m_info = info;
	}

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	ndMatrix m_baseFrame;
	ndWheelDescriptor m_info;
	ndFloat32 m_posit;
	ndFloat32 m_speed;
	ndFloat32 m_regularizer;
	ndFloat32 m_normalizedBrake;
	ndFloat32 m_normalidedSteering;
	ndFloat32 m_normalizedHandBrake;
	friend class ndMultiBodyVehicle;
};

#endif 

