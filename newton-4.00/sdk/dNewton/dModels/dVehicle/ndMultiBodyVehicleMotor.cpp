/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndJointWheel.h"
#include "ndBodyDynamic.h"
#include "ndMultiBodyVehicle.h"
#include "ndMultiBodyVehicleMotor.h"
#include "ndMultiBodyVehicleGearBox.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndMultiBodyVehicleMotor)

ndMultiBodyVehicleMotor::ndMultiBodyVehicleMotor(ndBodyKinematic* const motor, ndMultiBodyVehicle* const vehicelModel)
	:ndJointBilateralConstraint(3, motor, vehicelModel->m_chassis, motor->GetMatrix())
	,m_omega(dFloat32(0.0f))
	,m_idleOmega(dFloat32(0.0f))
	,m_throttle(dFloat32(0.0f))
	,m_engineTorque(dFloat32(0.0f))
	,m_fuelValveRate(dFloat32(10.0f))
	,m_vehicelModel(vehicelModel)
	,m_startEngine(false)
{
}

ndMultiBodyVehicleMotor::ndMultiBodyVehicleMotor(const dLoadSaveBase::dLoadDescriptor& desc)
	:ndJointBilateralConstraint(dLoadSaveBase::dLoadDescriptor(desc))
	,m_omega(dFloat32(0.0f))
	,m_idleOmega(dFloat32(0.0f))
	,m_throttle(dFloat32(0.0f))
	,m_engineTorque(dFloat32(0.0f))
	,m_fuelValveRate(dFloat32(10.0f))
	,m_vehicelModel(nullptr)
	,m_startEngine(false)
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	m_maxOmega = xmlGetFloat(xmlNode, "maxOmega");
	m_idleOmega = xmlGetFloat(xmlNode, "idleOmega");
	m_fuelValveRate = xmlGetFloat(xmlNode, "fuelValveRate");
}


void ndMultiBodyVehicleMotor::AlignMatrix()
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	//matrix1.m_posit += matrix1.m_up.Scale(1.0f);

	m_body0->SetMatrix(matrix1);
	m_body0->SetVelocity(m_body1->GetVelocity());

	const dVector omega0(m_body0->GetOmega());
	const dVector omega1(m_body1->GetOmega());

	const dVector wx(matrix1.m_front.Scale(matrix1.m_front.DotProduct(omega0).GetScalar()));
	const dVector wy(matrix1.m_up.Scale(matrix1.m_up.DotProduct(omega1).GetScalar()));
	const dVector wz(matrix1.m_right.Scale (matrix1.m_right.DotProduct(omega1).GetScalar()));
	const dVector omega(wx + wy + wz);

	//dVector error(omega1 - omega);
	//dTrace(("(%f %f %f)\n", error.m_x, error.m_y, error.m_z));
	m_body0->SetOmega(omega);
}

void ndMultiBodyVehicleMotor::SetFuelRate(dFloat32 radPerSecondsStep)
{
	m_fuelValveRate = dAbs(radPerSecondsStep);
}

void ndMultiBodyVehicleMotor::SetStart(bool startkey)
{
	m_startEngine = startkey;
}

void ndMultiBodyVehicleMotor::SetRpmLimits(dFloat32 idleRpm, dFloat32 redLineRpm)
{
	m_idleOmega = dAbs(idleRpm / dRadPerSecToRpm);
	m_maxOmega = dMax(redLineRpm / dRadPerSecToRpm, m_idleOmega);
}

void ndMultiBodyVehicleMotor::SetTorque(dFloat32 torqueInNewtonMeters)
{
	m_engineTorque = torqueInNewtonMeters;
}

void ndMultiBodyVehicleMotor::SetThrottle(dFloat32 param)
{
	m_throttle = dClamp(param, dFloat32(0.0f), dFloat32(1.0f));
}

dFloat32 ndMultiBodyVehicleMotor::CalculateAcceleration(ndConstraintDescritor& desc)
{
	const dVector& omega0 = m_body0->GetOmega();
	const ndJacobian& jacobian0 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM0;
	const dVector relOmega(omega0 * jacobian0.m_angular);

	m_omega = -relOmega.AddHorizontal().GetScalar();
	const dFloat32 throttleOmega = dClamp(m_throttle * m_maxOmega, m_idleOmega, m_maxOmega);
	const dFloat32 deltaOmega = throttleOmega - m_omega;
	dFloat32 omegaError = dClamp(deltaOmega, -m_fuelValveRate, m_fuelValveRate);
	//dTrace(("%f %f\n", throttleOmega, m_omega));
	return -omegaError * desc.m_invTimestep;
}

void ndMultiBodyVehicleMotor::JacobianDerivative(ndConstraintDescritor& desc)
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	// two rows to restrict rotation around around the parent coordinate system
	const dFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
	const dFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);

	AddAngularRowJacobian(desc, matrix1.m_up, angle0);
	AddAngularRowJacobian(desc, matrix1.m_right, angle1);

	// add rotor joint
	AddAngularRowJacobian(desc, matrix0.m_front, dFloat32(0.0f));
	const dFloat32 accel = CalculateAcceleration(desc);
	//dTrace(("%f\n", accel));
	if (m_startEngine)
	{
		const ndMultiBodyVehicleGearBox* const gearBox = m_vehicelModel->m_gearBox;
		dAssert(gearBox);
		if (gearBox && dAbs(gearBox->GetRatio()) > dFloat32(0.0f))
		{
			ndJacobian& jacobian = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM1;
			jacobian.m_angular = dVector::m_zero;
		}

		if (m_omega <= dFloat32(0.0f))
		{
			// engine rpm can not be negative
			dFloat32 stopAccel = dMin ((m_omega - 0.5f) * desc.m_invTimestep, accel);
			SetMotorAcceleration(desc, stopAccel);
			SetHighFriction(desc, m_engineTorque);
		}
		else if (m_omega >= m_maxOmega)
		{
			// engine rpm can not pass maximum allowed
			dFloat32 stopAccel = dMax ((m_omega - m_maxOmega) * desc.m_invTimestep, accel);
			SetMotorAcceleration(desc, stopAccel);
			SetLowerFriction(desc, -m_engineTorque);
		}
		else
		{
			// set engine gas and save the current joint omega
			SetMotorAcceleration(desc, accel);
			SetHighFriction(desc, m_engineTorque);
			SetLowerFriction(desc, -m_engineTorque);
		}
	}
	else
	{
		SetHighFriction(desc, m_engineTorque * dFloat32(4.0f));
		SetLowerFriction(desc, -m_engineTorque * dFloat32(4.0f));
		SetMotorAcceleration(desc, m_omega * desc.m_invTimestep);
	}
}

void ndMultiBodyVehicleMotor::Save(const dLoadSaveBase::dSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(dLoadSaveBase::dSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "maxOmega", m_maxOmega);
	xmlSaveParam(childNode, "idleOmega", m_idleOmega);
	xmlSaveParam(childNode, "fuelValveRate", m_fuelValveRate);
}
