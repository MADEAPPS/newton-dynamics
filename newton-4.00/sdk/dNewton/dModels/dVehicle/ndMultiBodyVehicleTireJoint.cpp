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

ndTireFrictionModel::ndBrushTireModel::ndBrushTireModel()
	:m_laterialStiffness(ndFloat32(10.0f))
	,m_longitudinalStiffness(ndFloat32(10.0f))
{
}

ndTireFrictionModel::ndBrushTireModel::ndBrushTireModel(ndFloat32 laterialStiffness, ndFloat32 longitudinalStiffness)
	:m_laterialStiffness(laterialStiffness)
	,m_longitudinalStiffness(longitudinalStiffness)
{
}

ndTireFrictionModel::ndPacejkaTireModel::ndPacejkaTireModel()
	:m_b(ndFloat32(0.5f))
	,m_c(ndFloat32(1.65f))
	,m_d(ndFloat32(1.0f))
	,m_e(ndFloat32(0.8f))
	,m_sv(ndFloat32(0.0f))
	,m_sh(ndFloat32(0.0f))
	,m_normalizingPhi(ndFloat32(1.0f))
{
	// set some defualt values, longitudinal force for a classic tire form Giancalr Genta book.
	CalculateMaxPhi();
}

ndTireFrictionModel::ndPacejkaTireModel::ndPacejkaTireModel(ndFloat32 B, ndFloat32 C, ndFloat32 D, ndFloat32 E, ndFloat32 Sv, ndFloat32 Sh)
	:m_b(B)
	,m_c(C)
	,m_d(D)
	,m_e(E)
	,m_sv(Sv)
	,m_sh(Sh)
	,m_normalizingPhi(ndFloat32(1.0f))
{
	CalculateMaxPhi();
}

void ndTireFrictionModel::ndPacejkaTireModel::CalculateMaxPhi()
{
	// claculate Max sizeSlipParam
	ndFloat32 maxPhi = ndFloat32(0.0f);
	ndFloat32 maxForce = ndFloat32(0.0f);
	for (ndFloat32 phi = ndFloat32(0.0f); phi <= ndFloat32(20.0f); phi += ndFloat32(0.001f))
	{
		ndFloat32 force = Evaluate(phi, 1000.0f);
		if (force >= maxForce)
		{
			maxPhi = phi;
			maxForce = force;
		}
	}
	m_normalizingPhi = maxPhi;
}

ndFloat32 ndTireFrictionModel::ndPacejkaTireModel::Evaluate(ndFloat32 phi, ndFloat32 f) const
{
	ndFloat32 displacedPhi = phi + m_sh;
	ndFloat32 EaTang = m_e * ndAtan(m_b * displacedPhi);
	ndFloat32 BEarg = m_b * (ndFloat32(1.0f) - m_e) * displacedPhi;
	ndFloat32 Carg = m_c * ndAtan(BEarg + EaTang);
	return f * (m_d * ndSin(Carg) + m_sv);
}

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
	//m_frictionModel.m_laterialStiffness = ndMax(ndAbs(m_frictionModel.m_laterialStiffness), ndFloat32(1.0f));
	//m_frictionModel.m_longitudinalStiffness = ndMax(ndAbs(m_frictionModel.m_longitudinalStiffness), ndFloat32(1.0f));
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

