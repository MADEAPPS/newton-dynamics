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

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndJointWheel.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointWheel)

void ndWheelDescriptor::Load(const nd::TiXmlNode* const xmlNode)
{
	const nd::TiXmlNode* childNode = nullptr;
	for (const nd::TiXmlNode* node = xmlNode->FirstChild(); node; node = node->NextSibling())
	{
		const char* const name = node->Value();
		if (strcmp(name, "ndTireInfo") == 0)
		{
			childNode = node;
			break;
		}
	}
	
	dAssert(childNode);
	m_springK = xmlGetFloat(childNode, "springK");
	m_damperC = xmlGetFloat(childNode, "damperC");
	m_upperStop = xmlGetFloat(childNode, "upperStop");
	m_lowerStop = xmlGetFloat(childNode, "lowerStop");
	m_regularizer = xmlGetFloat(childNode, "regularizer");
	m_brakeTorque = xmlGetFloat(childNode, "brakeTorque");
	m_handBrakeTorque = xmlGetFloat(childNode, "handBrakeTorque");
	m_steeringAngle = xmlGetFloat(childNode, "steeringAngle");
	m_laterialStiffness = xmlGetFloat(childNode, "laterialStiffness");
	m_longitudinalStiffness = xmlGetFloat(childNode, "longitudinalStiffness");
}

void ndWheelDescriptor::Save(nd::TiXmlNode* const xmlNode) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement("ndTireInfo");
	xmlNode->LinkEndChild(childNode);

	xmlSaveParam(childNode, "springK", m_springK);
	xmlSaveParam(childNode, "damperC", m_damperC);
	xmlSaveParam(childNode, "upperStop", m_upperStop);
	xmlSaveParam(childNode, "lowerStop", m_lowerStop);
	xmlSaveParam(childNode, "regularizer", m_regularizer);
	xmlSaveParam(childNode, "brakeTorque", m_brakeTorque);
	xmlSaveParam(childNode, "handBrakeTorque", m_handBrakeTorque);
	xmlSaveParam(childNode, "steeringAngle", m_steeringAngle);
	xmlSaveParam(childNode, "laterialStiffness", m_laterialStiffness);
	xmlSaveParam(childNode, "longitudinalStiffness", m_longitudinalStiffness);
}

ndJointWheel::ndJointWheel(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent, const ndWheelDescriptor& info)
	:ndJointBilateralConstraint(7, child, parent, pinAndPivotFrame)
	,m_baseFrame(m_localMatrix1)
	,m_info(info)
	,m_posit(dFloat32 (0.0f))
	,m_speed(dFloat32(0.0f))
	,m_regularizer(info.m_regularizer)
	,m_normalizedBrake(dFloat32(0.0f))
	,m_normalidedSteering(dFloat32(0.0f))
	,m_normalizedHandBrake(dFloat32(0.0f))
{
}

ndJointWheel::ndJointWheel(const dLoadSaveBase::dLoadDescriptor& desc)
	:ndJointBilateralConstraint(dLoadSaveBase::dLoadDescriptor(desc))
	,m_baseFrame(dGetIdentityMatrix())
	,m_info()
	,m_posit(dFloat32(0.0f))
	,m_speed(dFloat32(0.0f))
	,m_regularizer(0.0f)
	,m_normalizedBrake(dFloat32(0.0f))
	,m_normalidedSteering(dFloat32(0.0f))
	,m_normalizedHandBrake(dFloat32(0.0f))
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;

	m_info.Load(desc.m_rootNode);
	m_baseFrame = xmlGetMatrix(xmlNode, "baseFrame");
	m_regularizer = xmlGetFloat(xmlNode, "regularizer");
	m_normalizedBrake = xmlGetFloat(xmlNode, "normalizedBrake");
	m_normalidedSteering = xmlGetFloat(xmlNode, "normalidedSteering");
	m_normalizedHandBrake = xmlGetFloat(xmlNode, "normalizedHandBrake");
}

ndJointWheel::~ndJointWheel()
{
}

void ndJointWheel::SetBrake(dFloat32 normalizedBrake)
{
	m_normalizedBrake = dClamp (normalizedBrake, dFloat32 (0.0f), dFloat32 (1.0f));
}

void ndJointWheel::SetHandBrake(dFloat32 normalizedBrake)
{
	m_normalizedHandBrake = dClamp(normalizedBrake, dFloat32(0.0f), dFloat32(1.0f));
}

void ndJointWheel::SetSteering(dFloat32 normalidedSteering)
{
	m_normalidedSteering = dClamp(normalidedSteering, dFloat32(-1.0f), dFloat32(1.0f));
}

void ndJointWheel::UpdateTireSteeringAngleMatrix()
{
	dMatrix tireMatrix;
	dMatrix chassisMatrix;
	m_localMatrix1 = dYawMatrix(m_normalidedSteering * m_info.m_steeringAngle) * m_baseFrame;

	CalculateGlobalMatrix(tireMatrix, chassisMatrix);
	const dVector localRelPosit(chassisMatrix.UntransformVector(tireMatrix.m_posit));
	const dFloat32 distance = dClamp(localRelPosit.m_y, m_info.m_upperStop, m_info.m_lowerStop);

	const dFloat32 spinAngle = -CalculateAngle(tireMatrix.m_up, chassisMatrix.m_up, chassisMatrix.m_front);
	dMatrix newTireMatrix(dPitchMatrix(spinAngle) * chassisMatrix);
	newTireMatrix.m_posit = chassisMatrix.m_posit + chassisMatrix.m_up.Scale(distance);

	const dMatrix tireBodyMatrix(m_localMatrix0.Inverse() * newTireMatrix);
	m_body0->SetMatrix(tireBodyMatrix);
}

dMatrix ndJointWheel::CalculateUpperBumperMatrix() const
{
	dMatrix matrix(m_localMatrix1 * m_body1->GetMatrix());
	matrix.m_posit += matrix.m_up.Scale(m_info.m_lowerStop);
	return matrix;
}

void ndJointWheel::JacobianDerivative(ndConstraintDescritor& desc)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	// calculate position and speed	
	const dVector veloc0(m_body0->GetVelocityAtPoint(matrix0.m_posit));
	const dVector veloc1(m_body1->GetVelocityAtPoint(matrix1.m_posit));

	const dVector& pin = matrix1[0];
	const dVector& p0 = matrix0.m_posit;
	const dVector& p1 = matrix1.m_posit;
	const dVector prel(p0 - p1);
	const dVector vrel(veloc0 - veloc1);

	m_speed = vrel.DotProduct(matrix1.m_up).GetScalar();
	m_posit = prel.DotProduct(matrix1.m_up).GetScalar();
	const dVector projectedPoint = p1 + pin.Scale(pin.DotProduct(prel).GetScalar());

	const dFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
	const dFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);

	AddLinearRowJacobian(desc, p0, projectedPoint, matrix1[0]);
	AddLinearRowJacobian(desc, p0, projectedPoint, matrix1[2]);
	AddAngularRowJacobian(desc, matrix1.m_up, angle0);
	AddAngularRowJacobian(desc, matrix1.m_right, angle1);
	
	// add suspension spring damper row
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1.m_up);
	SetMassSpringDamperAcceleration(desc, m_regularizer, m_info.m_springK, m_info.m_damperC);

	const dFloat32 brakeFrictionTorque = dMax(m_normalizedBrake * m_info.m_brakeTorque, m_normalizedHandBrake * m_info.m_handBrakeTorque);
	if (brakeFrictionTorque > dFloat32(0.0f))
	{
		const dFloat32 brakesToChassisInfluence = dFloat32 (0.125f);

		AddAngularRowJacobian(desc, matrix1.m_front, dFloat32(0.0f));
		const dVector tireOmega(m_body0->GetOmega());
		const dVector chassisOmega(m_body1->GetOmega());

		ndJacobian& jacobian0 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM0;
		ndJacobian& jacobian1 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM1;
		jacobian1.m_angular = jacobian1.m_angular.Scale(brakesToChassisInfluence);

		dFloat32 w0 = tireOmega.DotProduct(jacobian0.m_angular).GetScalar();
		dFloat32 w1 = chassisOmega.DotProduct(jacobian1.m_angular).GetScalar();
		dFloat32 wRel = (w0 + w1) * dFloat32 (0.35f);

		SetMotorAcceleration(desc, -wRel * desc.m_invTimestep);
		SetHighFriction(desc, brakeFrictionTorque);
		SetLowerFriction(desc, -brakeFrictionTorque);
	}

	// add suspension limits alone the vertical axis 
	const dFloat32 x = m_posit + m_speed * desc.m_timestep;
	if (x < m_info.m_upperStop)
	{
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1.m_up);
		const dFloat32 stopAccel = GetMotorZeroAcceleration(desc);
		SetMotorAcceleration(desc, stopAccel);
		SetLowerFriction(desc, dFloat32(0.0f));
	}
	else if (x > m_info.m_lowerStop)
	{
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1.m_up);
		const dFloat32 stopAccel = GetMotorZeroAcceleration(desc);
		SetMotorAcceleration(desc, stopAccel);
		SetHighFriction(desc, dFloat32(0.0f));
	}
}

void ndJointWheel::Save(const dLoadSaveBase::dSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(dLoadSaveBase::dSaveDescriptor(desc, childNode));

	m_info.Save(childNode);
	xmlSaveParam(childNode, "baseFrame", m_baseFrame);
	xmlSaveParam(childNode, "regularizer", m_regularizer);
	xmlSaveParam(childNode, "normalizedBrake", m_normalizedBrake);
	xmlSaveParam(childNode, "normalidedSteering", m_normalidedSteering);
	xmlSaveParam(childNode, "normalizedHandBrake", m_normalizedHandBrake);
}
