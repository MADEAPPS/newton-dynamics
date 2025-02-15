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

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndMultiBodyVehicle.h"
#include "ndMultiBodyVehicleTireJoint.h"

ndMultiBodyVehicleTireJoint::ndMultiBodyVehicleTireJoint()
	:ndJointWheel()
	,m_vehicle(nullptr)
	,m_frictionModel()
	,m_lateralSlip(ndFloat32(0.0f))
	,m_longitudinalSlip(ndFloat32(0.0f))
	,m_normalizedAligningTorque(ndFloat32(0.0f))
{
	ndAssert(0);
}

ndMultiBodyVehicleTireJoint::ndMultiBodyVehicleTireJoint(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent, const ndMultiBodyVehicleTireJointInfo& info, ndMultiBodyVehicle* const vehicle)
	:ndJointWheel(pinAndPivotFrame, child, parent, info)
	,m_vehicle(vehicle)
	,m_frictionModel(info)
	,m_lateralSlip(ndFloat32 (0.0f))
	,m_longitudinalSlip(ndFloat32(0.0f))
	,m_normalizedAligningTorque(ndFloat32(0.0f))
{
	m_frictionModel.m_laterialStiffness = ndMax(ndAbs(m_frictionModel.m_laterialStiffness), ndFloat32(1.0f));
	m_frictionModel.m_longitudinalStiffness = ndMax(ndAbs(m_frictionModel.m_longitudinalStiffness), ndFloat32(1.0f));
}

ndMultiBodyVehicleTireJoint::~ndMultiBodyVehicleTireJoint()
{
}

void ndMultiBodyVehicleTireJoint::SetVehicleOwner(ndMultiBodyVehicle* const vehicle)
{
	m_vehicle = vehicle;
}

ndMultiBodyVehicleTireJointInfo ndMultiBodyVehicleTireJoint::GetInfo() const
{
	return ndMultiBodyVehicleTireJointInfo (ndJointWheel::GetInfo(), m_frictionModel);
}

ndFloat32 ndMultiBodyVehicleTireJoint::GetSideSlip() const
{
	return m_lateralSlip;
}

ndFloat32 ndMultiBodyVehicleTireJoint::GetLongitudinalSlip() const
{
	return m_longitudinalSlip;
}

const ndTireFrictionModel& ndMultiBodyVehicleTireJoint::GetFrictionModel() const
{
	return m_frictionModel;
}

void ndMultiBodyVehicleTireJoint::JacobianDerivative(ndConstraintDescritor& desc)
{
	m_regularizer = m_info.m_regularizer * m_vehicle->m_downForce.m_suspensionStiffnessModifier;
	ndJointWheel::JacobianDerivative(desc);
}

