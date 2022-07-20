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
#include "ndIkSwivelPositionEffector.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndIkSwivelPositionEffector)

ndIkSwivelPositionEffector::ndIkSwivelPositionEffector(const ndMatrix& pinAndPivotChild, const ndMatrix& pinAndPivotParent, const ndMatrix& swivelFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(6, child, parent, pinAndPivotChild, pinAndPivotParent)
	,m_targetFrame(pinAndPivotChild * pinAndPivotParent.Inverse())
	,m_swivelAngle(ndFloat32 (0.0f))
	,m_angularSpring(ndFloat32(1000.0f))
	,m_angularDamper(ndFloat32(50.0f))
	,m_angularMaxTorque(D_LCP_MAX_VALUE)
	,m_angularRegularizer(ndFloat32(5.0e-3f))
	,m_linearSpring(ndFloat32(1000.0f))
	,m_linearDamper(ndFloat32(50.0f))
	,m_linearMaxForce(D_LCP_MAX_VALUE)
	,m_linearRegularizer(ndFloat32(5.0e-3f))
	,m_minWorkSpaceRadio(ndFloat32(0.0f))
	,m_maxWorkSpaceRadio(ndFloat32(0.0f))
{
	CalculateLocalMatrix(swivelFrame, m_localSwivelMatrix0, m_localSwivelMatrix1);

	const ndVector dist2(ndVector::m_triplexMask & m_targetFrame.m_posit);
	m_maxWorkSpaceRadio = ndFloat32(0.9f) * ndSqrt(dist2.DotProduct(dist2).GetScalar());

	SetSolverModel(m_jointkinematicCloseLoop);
}

ndIkSwivelPositionEffector::ndIkSwivelPositionEffector(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointBilateralConstraint(ndLoadSaveBase::ndLoadDescriptor(desc))
	,m_targetFrame(dGetIdentityMatrix())
	,m_swivelAngle(ndFloat32(0.0f))
	,m_angularSpring(ndFloat32(1000.0f))
	,m_angularDamper(ndFloat32(50.0f))
	,m_angularMaxTorque(D_LCP_MAX_VALUE)
	,m_angularRegularizer(ndFloat32(5.0e-3f))
	,m_linearSpring(ndFloat32(1000.0f))
	,m_linearDamper(ndFloat32(50.0f))
	,m_linearMaxForce(D_LCP_MAX_VALUE)
	,m_linearRegularizer(ndFloat32(5.0e-3f))
	,m_minWorkSpaceRadio(ndFloat32(0.0f))
	,m_maxWorkSpaceRadio(ndFloat32(0.0f))
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	
	m_targetFrame = xmlGetMatrix(xmlNode, "targetFrame");
	m_localSwivelMatrix0 = xmlGetMatrix(xmlNode, "localSwivelMatrix0");
	m_localSwivelMatrix1 = xmlGetMatrix(xmlNode, "localSwivelMatrix1");

	m_swivelAngle = xmlGetFloat(xmlNode, "swivelAngle");

	m_angularSpring = xmlGetFloat(xmlNode, "angularSpring");
	m_angularDamper = xmlGetFloat(xmlNode, "angularDamper");
	m_angularMaxTorque = xmlGetFloat(xmlNode, "angularMaxTorque");
	m_angularRegularizer = xmlGetFloat(xmlNode, "angularRegularizer");
	
	m_linearSpring = xmlGetFloat(xmlNode, "linearSpring");
	m_linearDamper = xmlGetFloat(xmlNode, "linearDamper");
	m_linearMaxForce = xmlGetFloat(xmlNode, "linearMaxForce");
	m_linearRegularizer = xmlGetFloat(xmlNode, "linearRegularizer");

	m_minWorkSpaceRadio = xmlGetFloat(xmlNode, "minWorkSpaceRadior");
	m_maxWorkSpaceRadio = xmlGetFloat(xmlNode, "maxWorkSpaceRadior");
}

ndIkSwivelPositionEffector::~ndIkSwivelPositionEffector()
{
}

void ndIkSwivelPositionEffector::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "targetFrame", m_targetFrame);
	xmlSaveParam(childNode, "localSwivelMatrix0", m_localSwivelMatrix0);
	xmlSaveParam(childNode, "localSwivelMatrix1", m_localSwivelMatrix1);
	xmlSaveParam(childNode, "swivelAngle", m_swivelAngle);
	xmlSaveParam(childNode, "angularSpring", m_angularSpring);
	xmlSaveParam(childNode, "angularDamper", m_angularDamper);
	xmlSaveParam(childNode, "angularMaxTorque", m_angularMaxTorque);
	xmlSaveParam(childNode, "angularRegularizer", m_angularRegularizer);
	xmlSaveParam(childNode, "linearSpring", m_linearSpring);
	xmlSaveParam(childNode, "linearDamper", m_linearDamper);
	xmlSaveParam(childNode, "linearMaxForce", m_linearMaxForce);
	xmlSaveParam(childNode, "linearRegularizer", m_linearRegularizer);

	xmlSaveParam(childNode, "minWorkSpaceRadio", m_minWorkSpaceRadio);
	xmlSaveParam(childNode, "maxWorkSpaceRadio", m_maxWorkSpaceRadio);
}

ndVector ndIkSwivelPositionEffector::GetPosition() const
{
	return m_targetFrame.m_posit;
}

void ndIkSwivelPositionEffector::SetPosition(const ndVector& posit)
{
	ndVector target = posit & ndVector::m_triplexMask;
	ndFloat32 dist2 = target.DotProduct(target).GetScalar();
	if (dist2 > m_maxWorkSpaceRadio * m_maxWorkSpaceRadio)
	{
		target = target.Normalize().Scale(m_maxWorkSpaceRadio);
	}
	else if (dist2 < m_minWorkSpaceRadio * m_minWorkSpaceRadio)
	{
		target = target.Normalize().Scale(m_minWorkSpaceRadio);
	}
	m_targetFrame.m_posit = target | ndVector::m_wOne;
}

void ndIkSwivelPositionEffector::SetWorkSpaceConstraints(ndFloat32 minRadio, ndFloat32 maxRadio)
{
	m_minWorkSpaceRadio = minRadio;
	m_maxWorkSpaceRadio = maxRadio;
}

void ndIkSwivelPositionEffector::GetWorkSpaceConstraints(ndFloat32& minRadio, ndFloat32& maxRadio) const
{
	minRadio = m_minWorkSpaceRadio;
	maxRadio = m_maxWorkSpaceRadio;
}


ndFloat32 ndIkSwivelPositionEffector::GetMaxForce() const
{
	return m_linearMaxForce;
}

ndFloat32 ndIkSwivelPositionEffector::GetSwivelAngle() const
{
	return m_swivelAngle;
}

void ndIkSwivelPositionEffector::SetSwivelAngle(const ndFloat32 angle)
{
	m_swivelAngle = angle;
}

void ndIkSwivelPositionEffector::SetMaxForce(ndFloat32 force)
{
	m_linearMaxForce = ndAbs(force);
}

ndFloat32 ndIkSwivelPositionEffector::GetMaxTorque() const
{
	return m_angularMaxTorque;
}

void ndIkSwivelPositionEffector::SetMaxTorque(ndFloat32 torque)
{
	m_angularMaxTorque = ndAbs(torque);
}

void ndIkSwivelPositionEffector::SetLinearSpringDamper(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper)
{
	m_linearSpring = ndAbs(spring);
	m_linearDamper = ndAbs(damper);
	m_linearRegularizer = ndClamp(regularizer, ndFloat32(1.0e-4f), ndFloat32(0.99f));
}

void ndIkSwivelPositionEffector::GetLinearSpringDamper(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const
{
	spring = m_linearSpring;
	damper = m_linearDamper;
	regularizer = m_linearRegularizer;
}

void ndIkSwivelPositionEffector::SetAngularSpringDamper(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper)
{
	m_angularSpring = ndAbs(spring);
	m_angularDamper = ndAbs(damper);
	m_angularRegularizer = ndClamp(regularizer, ndFloat32(1.0e-4f), ndFloat32(0.99f));
}

void ndIkSwivelPositionEffector::GetAngularSpringDamper(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const
{
	spring = m_angularSpring;
	damper = m_angularDamper;
	regularizer = m_angularRegularizer;
}

void ndIkSwivelPositionEffector::CalculateSwivelMatrices(ndMatrix& swivelMatrix0, ndMatrix& swivelMatrix1) const
{
	const ndVector posit0(m_body0->GetMatrix().TransformVector(m_localMatrix0.m_posit));
	const ndVector posit1(m_body1->GetMatrix().TransformVector(m_localMatrix1.m_posit));

	swivelMatrix0 = m_localSwivelMatrix0 * m_body0->GetMatrix();
	swivelMatrix1 = m_localSwivelMatrix1 * m_body1->GetMatrix();

	const ndVector midPoint(ndVector::m_half * (posit0 + posit1));
	const ndVector pin(swivelMatrix1.UnrotateVector(posit0 - posit1).Normalize());
	ndMatrix localSwivel1(dGetIdentityMatrix());

	localSwivel1.m_front = pin;
	localSwivel1.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
	localSwivel1.m_right = (localSwivel1.m_front.CrossProduct(localSwivel1.m_up)).Normalize();
	localSwivel1.m_up = localSwivel1.m_right.CrossProduct(localSwivel1.m_front);

	swivelMatrix1 = localSwivel1 * swivelMatrix1;
	swivelMatrix0.m_posit = midPoint;
	swivelMatrix1.m_posit = midPoint;
}

void ndIkSwivelPositionEffector::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	//const ndMatrix matrix0(m_localMatrix0 * m_body0->GetMatrix());
	//const ndMatrix matrix1(m_localMatrix1 * m_body1->GetMatrix());
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);
	const ndMatrix targetFrame(m_targetFrame * matrix1);

	ndMatrix swivelMatrix0;
	ndMatrix swivelMatrix1;
	CalculateSwivelMatrices(swivelMatrix0, swivelMatrix1);
	swivelMatrix1 = dPitchMatrix(m_swivelAngle) * swivelMatrix1;

	//debugCallback.DrawFrame(swivelMatrix0);
	//debugCallback.DrawFrame(swivelMatrix1, 0.5f);
	//debugCallback.DrawLine(matrix0.m_posit, matrix1.m_posit, ndVector(ndFloat32(1.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(1.0f)));
	
	debugCallback.DrawFrame(matrix0);
	//debugCallback.DrawFrame(matrix1, 0.5f);
	//debugCallback.DrawFrame(targetFrame, 0.5f);
	//debugCallback.DrawPoint(targetFrame.m_posit, ndVector(1.0f, 1.0f, 0.0f, 0.0f), 8.0f);
}

void ndIkSwivelPositionEffector::SubmitAngularAxis(ndConstraintDescritor& desc)
{
	ndMatrix swivelMatrix0;
	ndMatrix swivelMatrix1;
	CalculateSwivelMatrices(swivelMatrix0, swivelMatrix1);
	swivelMatrix1 = dPitchMatrix(m_swivelAngle) * swivelMatrix1;

	const ndVector& pin = swivelMatrix1.m_front;
	const ndFloat32 angle = CalculateAngle(swivelMatrix0[1], swivelMatrix1[1], swivelMatrix1[0]);
	ndFloat32 xxxx = pin.DotProduct(swivelMatrix0[1]).GetScalar();
	if (xxxx > 0.9f)
	{
		xxxx *= 1;
	}


	AddAngularRowJacobian(desc, pin, angle);
	SetMassSpringDamperAcceleration(desc, m_angularRegularizer, m_angularSpring, m_angularDamper);
}

void ndIkSwivelPositionEffector::SubmitLinearAxis(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	const ndMatrix& axisDir = matrix1;
	const ndVector posit0(matrix0.m_posit);
	const ndVector posit1(matrix1.TransformVector(m_targetFrame.m_posit));
	
	for (ndInt32 i = 0; i < 3; ++i)
	{
		const ndVector pin = axisDir[i];
		AddLinearRowJacobian(desc, posit0, posit1, pin);
		SetMassSpringDamperAcceleration(desc, m_linearRegularizer, m_linearSpring, m_linearDamper*2.0f);
		SetLowerFriction(desc, -m_linearMaxForce);
		SetHighFriction(desc, m_linearMaxForce);
	}
}

void ndIkSwivelPositionEffector::JacobianDerivative(ndConstraintDescritor& desc)
{
	SubmitLinearAxis(desc);
	SubmitAngularAxis(desc);
}