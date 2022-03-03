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

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndIk6DofEffector.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndIk6DofEffector)

ndIk6DofEffector::ndIk6DofEffector(const ndMatrix& pinAndPivotChild, const ndMatrix& pinAndPivotParent, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(6, child, parent, pinAndPivotChild, pinAndPivotParent)
	,m_targetFrame(pinAndPivotChild * pinAndPivotParent.Inverse())
	,m_angularSpring(ndFloat32(1000.0f))
	,m_angularDamper(ndFloat32(50.0f))
	,m_angularRegularizer(ndFloat32(5.0e-3f))
	,m_linearSpring(ndFloat32(1000.0f))
	,m_linearDamper(ndFloat32(50.0f))
	,m_linearRegularizer(ndFloat32(5.0e-3f))
	,m_swivelAngleValue(ndFloat32(0.0f))
	,m_rotationType(m_disabled)
	,m_controlDofOptions(0xff)
{
	SetSolverModel(m_jointkinematicCloseLoop);
}

ndIk6DofEffector::ndIk6DofEffector(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointBilateralConstraint(ndLoadSaveBase::ndLoadDescriptor(desc))
	,m_targetFrame(dGetIdentityMatrix())
	,m_angularSpring(ndFloat32(1000.0f))
	,m_angularDamper(ndFloat32(50.0f))
	,m_angularRegularizer(ndFloat32(5.0e-3f))
	,m_linearSpring(ndFloat32(1000.0f))
	,m_linearDamper(ndFloat32(50.0f))
	,m_linearRegularizer(ndFloat32(5.0e-3f))
	,m_swivelAngleValue(ndFloat32(0.0f))
	,m_rotationType(m_disabled)
	,m_controlDofOptions(0xff)
{
	dAssert(0);
	//const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	//
	//m_basePose = xmlGetMatrix(xmlNode, "pivotFrame");
	//m_targetPosit = xmlGetVector3(xmlNode, "targetPosit");
	//
	//m_angle = xmlGetFloat (xmlNode, "m_angle");
	//m_minAngle = xmlGetFloat (xmlNode, "minAngle");
	//m_maxAngle = xmlGetFloat (xmlNode, "maxAngle");
	//m_angularSpring = xmlGetFloat (xmlNode, "angularSpring");
	//m_angularDamper = xmlGetFloat (xmlNode, "angularDamper");
	//m_angularRegularizer = xmlGetFloat (xmlNode, "angularRegularizer");
	//
	//m_maxDist = xmlGetFloat(xmlNode, "maxDist");
	//m_linearSpring = xmlGetFloat(xmlNode, "linearSpring");
	//m_linearDamper = xmlGetFloat(xmlNode, "linearDamper");
	//m_linearRegularizer = xmlGetFloat(xmlNode, "linearRegularizer");
	//
	//m_targetPosit.m_w = ndFloat32(1.0f);
	//m_targetPosit.m_w = ndFloat32(1.0f);
	//SetTargetOffset(m_targetPosit);
}

ndIk6DofEffector::~ndIk6DofEffector()
{
}

void ndIk6DofEffector::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	dAssert(0);
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));
	
	//xmlSaveParam(childNode, "pivotFrame", m_basePose);
	//xmlSaveParam(childNode, "targetPosit", m_targetPosit);
	//
	//xmlSaveParam(childNode, "angle", m_angle);
	//xmlSaveParam(childNode, "minAngle", m_minAngle);
	//xmlSaveParam(childNode, "maxAngle", m_maxAngle);
	//xmlSaveParam(childNode, "angularSpring", m_angularSpring);
	//xmlSaveParam(childNode, "angularDamper", m_angularDamper);
	//xmlSaveParam(childNode, "angularRegularizer", m_angularRegularizer);
	//
	//xmlSaveParam(childNode, "maxDist", m_maxDist);
	//xmlSaveParam(childNode, "linearSpring", m_linearSpring);
	//xmlSaveParam(childNode, "linearDamper", m_linearDamper);
	//xmlSaveParam(childNode, "linearRegularizer", m_linearRegularizer);
}

void ndIk6DofEffector::EnableAxisX(bool state)
{
	m_axisX = state ? 1 : 0;
}

void ndIk6DofEffector::EnableAxisY(bool state)
{
	m_axisY = state ? 1 : 0;
}

void ndIk6DofEffector::EnableAxisZ(bool state)
{
	m_axisZ = state ? 1 : 0;
}

void ndIk6DofEffector::EnableRotationAxis(ndRotationType type)
{
	m_rotationType = type;
}

ndMatrix ndIk6DofEffector::GetOffsetMatrix() const
{
	return m_targetFrame;
}

ndFloat32 ndIk6DofEffector::GetSwivelAngle() const
{
	return m_swivelAngleValue;
}

void ndIk6DofEffector::SetSwivelAngle(const ndFloat32& angle)
{
	m_swivelAngleValue = angle;
}

void ndIk6DofEffector::SetOffsetMatrix(const ndMatrix& matrix)
{
	m_targetFrame = matrix;
}

void ndIk6DofEffector::SetLinearSpringDamper(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper)
{
	m_linearSpring = dAbs(spring);
	m_linearDamper = dAbs(damper);
	m_linearRegularizer = dClamp(regularizer, ndFloat32(1.0e-4f), ndFloat32(0.99f));
}

void ndIk6DofEffector::GetLinearSpringDamper(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const
{
	spring = m_linearSpring;
	damper = m_linearDamper;
	regularizer = m_linearRegularizer;
}

void ndIk6DofEffector::SetAngularSpringDamper(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper)
{
	m_angularSpring = dAbs(spring);
	m_angularDamper = dAbs(damper);
	m_angularRegularizer = dClamp(regularizer, ndFloat32(1.0e-4f), ndFloat32(0.99f));
}

void ndIk6DofEffector::GetAngularSpringDamper(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const
{
	spring = m_angularSpring;
	damper = m_angularDamper;
	regularizer = m_angularRegularizer;
}

void ndIk6DofEffector::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	const ndMatrix matrix0(m_localMatrix0 * m_body0->GetMatrix());
	const ndMatrix matrix1(m_localMatrix1 * m_body1->GetMatrix());
	const ndMatrix targetFrame(m_targetFrame * matrix1);
	
	debugCallback.DrawFrame(matrix0);
	debugCallback.DrawFrame(matrix1);
	debugCallback.DrawFrame(targetFrame);
}

void ndIk6DofEffector::SubmitShortestPathAxis(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	const ndQuaternion rotation(matrix0.Inverse() * matrix1);
	const ndVector pin(rotation & ndVector::m_triplexMask);
	const ndFloat32 dirMag2 = pin.DotProduct(pin).GetScalar();
	const ndFloat32 tol = ndFloat32(1.0e-3f);
	if (dirMag2 > (tol * tol))
	{
		const ndMatrix basis(pin);
		const ndFloat32 dirMag = ndSqrt(dirMag2);
		const ndFloat32 angle = ndFloat32(2.0f) * ndAtan2(dirMag, rotation.m_w);

		AddAngularRowJacobian(desc, basis[0], angle);
		SetMassSpringDamperAcceleration(desc, m_angularRegularizer, m_angularSpring, m_angularDamper);
		AddAngularRowJacobian(desc, basis[1], ndFloat32(0.0f));
		AddAngularRowJacobian(desc, basis[2], ndFloat32(0.0f));
	}
	else
	{
		const ndFloat32 pitchAngle = CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);
		AddAngularRowJacobian(desc, matrix1[0], pitchAngle);
		SetMassSpringDamperAcceleration(desc, m_angularRegularizer, m_angularSpring, m_angularDamper);

		const ndFloat32 yawAngle = CalculateAngle(matrix0[0], matrix1[0], matrix1[1]);
		AddAngularRowJacobian(desc, matrix1[1], yawAngle);
		SetMassSpringDamperAcceleration(desc, m_angularRegularizer, m_angularSpring, m_angularDamper);

		const ndFloat32 rollAngle = CalculateAngle(matrix0[0], matrix1[0], matrix1[2]);
		AddAngularRowJacobian(desc, matrix1[2], rollAngle);
		SetMassSpringDamperAcceleration(desc, m_angularRegularizer, m_angularSpring, m_angularDamper);
	}
}

void ndIk6DofEffector::SubmitAngularAxis(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	switch (m_rotationType)
	{
		case m_fixAxis:
		{
			const ndMatrix matrix11(m_targetFrame * matrix1);
			const ndVector& pin = matrix11.m_front;

			const ndFloat32 angle = CalculateAngle(matrix0[1], matrix11[1], matrix11[0]);
			AddAngularRowJacobian(desc, pin, angle);
			SetMassSpringDamperAcceleration(desc, m_angularRegularizer, m_angularSpring, m_angularDamper);
			break;
		}
		case m_swivelAngle:
		{
			const ndMatrix matrix11(m_targetFrame * matrix1);
			SubmitShortestPathAxis(matrix0, matrix11, desc);
			break;
		}
		case m_shortestPath:
		{
			const ndMatrix matrix11(m_targetFrame * matrix1);
			SubmitShortestPathAxis(matrix0, matrix11, desc);
			break;
		}
	}
}

void ndIk6DofEffector::SubmitLinearAxis(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	ndVector posit1(matrix1.TransformVector(m_targetFrame.m_posit));
dTrace(("%f %f\n", matrix0.m_posit.m_y, posit1.m_y));
	for (ndInt32 i = 0; i < 3; i++)
	{
		if (m_controlDofOptions & (1 << i))
		{
			const ndVector pin = matrix1[i];
			AddLinearRowJacobian(desc, matrix0.m_posit, posit1, pin);
			SetMassSpringDamperAcceleration(desc, m_linearRegularizer, m_linearSpring, m_linearDamper);
		}
	}
}

void ndIk6DofEffector::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);
	SubmitLinearAxis(matrix0, matrix1, desc);
	SubmitAngularAxis(matrix0, matrix1, desc);
}