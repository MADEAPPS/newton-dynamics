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
	,m_omega(ndFloat32(0.0f))
	,m_maxOmega(ndFloat32(100.0f))
	,m_omegaStep(ndFloat32(8.0f))
	,m_targetOmega(ndFloat32(0.0f))
	,m_engineTorque(ndFloat32(0.0f))
	,m_internalFriction(ndFloat32(100.0f))
	,m_vehicelModel(vehicelModel)
{
}

ndMultiBodyVehicleMotor::ndMultiBodyVehicleMotor(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointBilateralConstraint(ndLoadSaveBase::ndLoadDescriptor(desc))
	,m_omega(ndFloat32(0.0f))
	,m_maxOmega(ndFloat32(100.0f))
	,m_engineTorque(ndFloat32(0.0f))
	,m_internalFriction(ndFloat32(100.0f))
	,m_vehicelModel(nullptr)
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	m_maxOmega = xmlGetFloat(xmlNode, "maxOmega");
	m_omegaStep = xmlGetFloat(xmlNode, "omegaStep");
	m_internalFriction = xmlGetFloat(xmlNode, "internalFriction");
}

void ndMultiBodyVehicleMotor::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "maxOmega", m_maxOmega);
	xmlSaveParam(childNode, "omegaStep", m_omegaStep);
	xmlSaveParam(childNode, "internalFriction", m_internalFriction);
}

void ndMultiBodyVehicleMotor::AlignMatrix()
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	//matrix1.m_posit += matrix1.m_up.Scale(1.0f);

	m_body0->SetMatrix(matrix1);
	m_body0->SetVelocity(m_body1->GetVelocity());

	const ndVector omega0(m_body0->GetOmega());
	const ndVector omega1(m_body1->GetOmega());

	const ndVector wx(matrix1.m_front.Scale(matrix1.m_front.DotProduct(omega0).GetScalar()));
	const ndVector wy(matrix1.m_up.Scale(matrix1.m_up.DotProduct(omega1).GetScalar()));
	const ndVector wz(matrix1.m_right.Scale (matrix1.m_right.DotProduct(omega1).GetScalar()));
	const ndVector omega(wx + wy + wz);

	//ndVector error(omega1 - omega);
	//dTrace(("(%f %f %f)\n", error.m_x, error.m_y, error.m_z));
	m_body0->SetOmega(omega);
}

void ndMultiBodyVehicleMotor::SetFrictionLose(ndFloat32 newtonMeters)
{
	m_internalFriction = dAbs(newtonMeters);
}

void ndMultiBodyVehicleMotor::SetMaxRpm(ndFloat32 redLineRpm)
{
	m_maxOmega = dMax(redLineRpm / dRadPerSecToRpm, ndFloat32 (0.0f));
}

void ndMultiBodyVehicleMotor::SetOmegaAccel(ndFloat32 rpmStep)
{
	m_omegaStep = dAbs(rpmStep / dRadPerSecToRpm);
}

void ndMultiBodyVehicleMotor::SetTorqueAndRpm(ndFloat32 newtonMeters, ndFloat32 rpm)
{
	m_engineTorque = dMax(newtonMeters, ndFloat32(0.0f));
	m_targetOmega = dClamp(rpm / dRadPerSecToRpm, ndFloat32(0.0f), m_maxOmega);
}

ndFloat32 ndMultiBodyVehicleMotor::GetRpm() const
{
	return m_omega * dRadPerSecToRpm;
}

ndFloat32 ndMultiBodyVehicleMotor::CalculateAcceleration(ndConstraintDescritor& desc)
{
	const ndVector& motorOmega = m_body0->GetOmega();
	const ndJacobian& motorJacobian = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM0;
	const ndVector relOmega(motorOmega * motorJacobian.m_angular);

	ndFloat32 currentOmega = relOmega.AddHorizontal().GetScalar();
	if (currentOmega < ndFloat32(0.0f))
	{
		const ndVector clippedOmega(motorOmega - motorJacobian.m_angular * relOmega);
		m_body0->SetOmega(clippedOmega);
		currentOmega = 0;
	}

	m_omega = currentOmega;
	ndFloat32 omegaStep = dClamp(m_targetOmega - m_omega, -m_omegaStep, m_omegaStep);
	ndFloat32 accel = omegaStep * desc.m_invTimestep;
	return accel;
}

void ndMultiBodyVehicleMotor::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	// two rows to restrict rotation around around the parent coordinate system
	const ndFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
	const ndFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);

	AddAngularRowJacobian(desc, matrix1.m_up, angle0);
	AddAngularRowJacobian(desc, matrix1.m_right, angle1);

	// add rotor joint acceleration
	AddAngularRowJacobian(desc, matrix0.m_front * ndVector::m_negOne, ndFloat32(0.0f));

	const ndFloat32 accel = CalculateAcceleration(desc);
	const ndFloat32 torque = dMax(m_engineTorque, m_internalFriction);
	SetMotorAcceleration(desc, accel);
	SetHighFriction(desc, torque);
	SetLowerFriction(desc, -torque);
	SetDiagonalRegularizer(desc, ndFloat32(0.1f));

	// add torque coupling to chassis.
	const ndMultiBodyVehicleGearBox* const gearBox = m_vehicelModel->m_gearBox;
	dAssert(gearBox);
	if (gearBox && dAbs(gearBox->GetRatio()) > ndFloat32(0.0f))
	{
		ndJacobian& chassisJacobian = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM1;
		chassisJacobian.m_angular = ndVector::m_zero;
	}
}


