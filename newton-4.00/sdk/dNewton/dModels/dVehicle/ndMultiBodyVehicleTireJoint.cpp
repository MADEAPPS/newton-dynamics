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

static ndTireFrictionModel::ndPacejkaTireModel pacejkaSportLateral(ndFloat32(0.2f), ndFloat32(1.5f), ndFloat32(1000.0f), ndFloat32(-0.1f), ndFloat32(0.0f), ndFloat32(0.01f));
static ndTireFrictionModel::ndPacejkaTireModel pacejkaSportLongitudinal(ndFloat32(0.5f), ndFloat32(1.65f), ndFloat32(1000.0f), ndFloat32(0.8f), ndFloat32(0.0f), ndFloat32(0.0f));

static ndTireFrictionModel::ndPacejkaTireModel pacejkaUtilityLateral(ndFloat32(0.14f), ndFloat32(1.35f), ndFloat32(1000.0f), ndFloat32(-0.5f), ndFloat32(0.0f), ndFloat32(0.01f));
static ndTireFrictionModel::ndPacejkaTireModel pacejkaUtilityLongitudinal(ndFloat32(0.2f), ndFloat32(1.4), ndFloat32(1000.0f), ndFloat32(-9.8f), ndFloat32(0.0f), ndFloat32(0.0f));

static ndTireFrictionModel::ndPacejkaTireModel pacejkaTruckLateral(ndFloat32(0.01f), ndFloat32(2.85f), ndFloat32(1000.0f), ndFloat32(1.42f), ndFloat32(0.0f), ndFloat32(0.01f));
static ndTireFrictionModel::ndPacejkaTireModel pacejkaTruckLongitudinal(ndFloat32(0.5f), ndFloat32(1.65f), ndFloat32(1000.0f), ndFloat32(0.8f), ndFloat32(0.0f), ndFloat32(0.0f));

ndTireFrictionModel::ndTireFrictionModel()
	:m_frictionModel(m_pacejkaUtility)
{
	SetPacejkaCurves(m_pacejkaUtility);
}

void ndTireFrictionModel::SetPacejkaCurves(const ndPacejkaTireModel& longitudinal, const ndPacejkaTireModel& lateral)
{
	m_frictionModel = m_pacejkaCustom;
	m_lateralPacejka = lateral;
	m_longitudinalPacejka = longitudinal;
}

void ndTireFrictionModel::SetPacejkaCurves(ndFrictionModel pacejkaStockModel)
{
	switch (pacejkaStockModel)
	{
		case m_pacejkaSport:
			SetPacejkaCurves(pacejkaSportLongitudinal, pacejkaSportLateral);
			m_frictionModel = pacejkaStockModel;
			break;

		case ndTireFrictionModel::m_pacejkaTruck:
			SetPacejkaCurves(pacejkaTruckLongitudinal, pacejkaTruckLateral);
			m_frictionModel = pacejkaStockModel;
			break;

		case ndTireFrictionModel::m_pacejkaUtility:
		default:
			SetPacejkaCurves(pacejkaUtilityLongitudinal, pacejkaUtilityLateral);
			m_frictionModel = pacejkaStockModel;
			break;
	}
}

void ndTireFrictionModel::GetPacejkaCurves(ndFrictionModel pacejkaStockModel, ndPacejkaTireModel& longitudinal, ndPacejkaTireModel& lateral) const
{
	switch (pacejkaStockModel)
	{
		case m_pacejkaSport:
			lateral = pacejkaSportLateral;
			longitudinal = pacejkaSportLongitudinal;
			break;

		case ndTireFrictionModel::m_pacejkaTruck:
			lateral = pacejkaTruckLateral;
			longitudinal = pacejkaTruckLongitudinal;
			break;

		case ndTireFrictionModel::m_pacejkaUtility:
		default:
			lateral = pacejkaUtilityLateral;
			longitudinal = pacejkaUtilityLongitudinal;
			break;
	}
}

void ndTireFrictionModel::PlotPacejkaCurves(const char* const name) const
{
	FILE* outFile;
	char nameExt[256];

	// write as execcl format
	snprintf(nameExt, size_t(nameExt) - 1, "%s.csv", name);

	outFile = fopen(nameExt, "wb");
	fprintf(outFile, "fx; fz; phi\n");
	for (ndFloat32 x = -20.0f; x < 20.0f; x += 0.01f)
	{
		ndFloat32 fx = m_longitudinalPacejka.Evaluate(x, ndFloat32(1.0f));
		ndFloat32 fz = m_lateralPacejka.Evaluate(x, ndFloat32(1.0f));
		fprintf(outFile, "%g; %g; %g\n", fx, fz, x);
	}
	fclose(outFile);
}

ndTireFrictionModel::ndPacejkaTireModel::ndPacejkaTireModel()
	:m_b(ndFloat32(0.5f))
	,m_c(ndFloat32(1.65f))
	,m_d(ndFloat32(1.0f))
	,m_e(ndFloat32(0.8f))
	,m_sv(ndFloat32(0.0f))
	,m_sh(ndFloat32(0.0f))
	,m_normalizingPhi(ndFloat32(1.0f))
	,m_norminalNormalForce(ndFloat32(1000.0f))
{
	// set some defualt values, longitudinal force for a classic tire form Giancalr Genta book.
	CalculateMaxPhi();
}

ndTireFrictionModel::ndPacejkaTireModel::ndPacejkaTireModel(ndFloat32 B, ndFloat32 C, ndFloat32 D, ndFloat32 E, ndFloat32 Sv, ndFloat32 Sh)
	:m_b(B)
	,m_c(C)
	,m_d(1.0f)
	,m_e(E)
	,m_sv(Sv)
	,m_sh(Sh)
	,m_normalizingPhi(ndFloat32(1.0f))
	,m_norminalNormalForce(D)
{
	CalculateMaxPhi();
}

void ndTireFrictionModel::ndPacejkaTireModel::CalculateMaxPhi()
{
	// claculate Max sizeSlipParam
	ndFloat32 maxPhi = ndFloat32(0.0f);
	ndFloat32 maxForce = ndFloat32(0.0f);
	for (ndFloat32 phi = ndFloat32(0.0f); phi < ndFloat32(20.0f); phi += ndFloat32(0.001f))
	{
		ndFloat32 force = Evaluate(phi, ndFloat32 (1.0f));
		if (force >= maxForce)
		{
			maxPhi = phi;
			maxForce = force;
		}
	}
	m_normalizingPhi = maxPhi;
}

ndFloat32 ndTireFrictionModel::ndPacejkaTireModel::Evaluate(ndFloat32 phi, ndFloat32 frictionCoefficient) const
{
	ndFloat32 displacedPhi = phi + m_sh;
	ndFloat32 EaTang = m_e * ndAtan(m_b * displacedPhi);
	ndFloat32 BEarg = m_b * (ndFloat32(1.0f) - m_e) * displacedPhi;
	ndFloat32 Carg = m_c * ndAtan(BEarg + EaTang);
	ndFloat32 f = m_d * ndSin(Carg) + m_sv;
	return frictionCoefficient * m_norminalNormalForce * f;
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

