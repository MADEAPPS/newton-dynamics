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
{
	CalculateLocalMatrix(swivelFrame, m_localSwivelMatrix0, m_localSwivelMatrix1);

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
}

ndVector ndIkSwivelPositionEffector::GetPosition() const
{
	return m_targetFrame.m_posit;
}

void ndIkSwivelPositionEffector::SetPosition(const ndVector& posit)
{
	m_targetFrame.m_posit = posit;
	m_targetFrame.m_posit.m_w = ndFloat32(1.0f);
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

void ndIkSwivelPositionEffector::CalculeteSwivelMatrices(const ndMatrix& matrix0, const ndMatrix& matrix1, ndMatrix& swivelMatrix0, ndMatrix& swivelMatrix1) const
{
	swivelMatrix0 = m_localSwivelMatrix0 * m_body0->GetMatrix();
	swivelMatrix1 = m_localSwivelMatrix1 * m_body1->GetMatrix();

	const ndVector midPoint(ndVector::m_half * (matrix0.m_posit + matrix1.m_posit));
	const ndVector pin(swivelMatrix1.UnrotateVector(matrix0.m_posit - matrix1.m_posit).Normalize());
	ndMatrix localSwivel1(dGetIdentityMatrix());

	localSwivel1.m_front = pin;
	localSwivel1.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
	localSwivel1.m_right = (localSwivel1.m_front.CrossProduct(localSwivel1.m_up)).Normalize();
	localSwivel1.m_up = localSwivel1.m_right.CrossProduct(localSwivel1.m_front);
	swivelMatrix1 = dPitchMatrix(m_swivelAngle) * localSwivel1 * swivelMatrix1;

	swivelMatrix0.m_posit = midPoint;
	swivelMatrix1.m_posit = midPoint;
}

void ndIkSwivelPositionEffector::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	const ndMatrix matrix0(m_localMatrix0 * m_body0->GetMatrix());
	const ndMatrix matrix1(m_localMatrix1 * m_body1->GetMatrix());
	const ndMatrix targetFrame(m_targetFrame * matrix1);

	ndMatrix swivelMatrix0;
	ndMatrix swivelMatrix1;
	CalculeteSwivelMatrices(matrix0, matrix1, swivelMatrix0, swivelMatrix1);
	
	debugCallback.DrawFrame(swivelMatrix0);
	debugCallback.DrawFrame(swivelMatrix1);
	debugCallback.DrawLine(matrix0.m_posit, matrix1.m_posit, ndVector(ndFloat32(1.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(1.0f)));
	
	debugCallback.DrawFrame(matrix0);
	//debugCallback.DrawFrame(matrix1);
	debugCallback.DrawFrame(targetFrame);
	debugCallback.DrawPoint(targetFrame.m_posit, ndVector(1.0f, 1.0f, 0.0f, 0.0f), 8.0f);
}

void ndIkSwivelPositionEffector::SubmitAngularAxis(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	ndMatrix swivelMatrix0;
	ndMatrix swivelMatrix1;
	CalculeteSwivelMatrices(matrix0, matrix1, swivelMatrix0, swivelMatrix1);

	const ndVector& pin = swivelMatrix1.m_front;
	const ndFloat32 angle = CalculateAngle(swivelMatrix0[1], swivelMatrix1[1], swivelMatrix1[0]);

ndMatrix xxxx0(swivelMatrix0 * swivelMatrix1.Inverse());
ndMatrix xxxx1(dGetIdentityMatrix());
ndFloat32 angle1 = CalculateAngle(xxxx0[1], xxxx1[1], xxxx1[0]);
dTrace(("%f %f\n", angle * ndRadToDegree, angle1 * ndRadToDegree));


	AddAngularRowJacobian(desc, pin, angle);
	SetMassSpringDamperAcceleration(desc, m_angularRegularizer, m_angularSpring, m_angularDamper);
}

void ndIkSwivelPositionEffector::SubmitLinearAxis(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	const ndMatrix& axisDir = matrix1;
	const ndVector posit0(matrix0.m_posit);
	const ndVector posit1(matrix1.TransformVector(m_targetFrame.m_posit));
	
#if 1
	for (ndInt32 i = 0; i < 3; ++i)
	{
		const ndVector pin = axisDir[i];
		AddLinearRowJacobian(desc, posit0, posit1, pin);
		SetMassSpringDamperAcceleration(desc, m_linearRegularizer, m_linearSpring, m_linearDamper*2.0f);
		SetLowerFriction(desc, -m_linearMaxForce);
		SetHighFriction(desc, m_linearMaxForce);
	}
#else

	const ndBodyKinematic* const body0 = GetBody0();
	const ndBodyKinematic* const body1 = GetBody1();

	const ndVector omega0(body0->GetOmega());
	const ndVector omega1(body1->GetOmega());
	const ndVector veloc0 (body0->GetVelocity());
	const ndVector veloc1 (body1->GetVelocity());

	for (ndInt32 i = 0; i < 3; ++i)
	{
		if (m_controlDofOptions & (1 << i))
		{
			const ndVector pin = axisDir[i];
			AddLinearRowJacobian(desc, posit0, posit1, pin);
			const ndInt32 index = desc.m_rowsCount - 1;
			const ndJacobian& jacobian0 = desc.m_jacobian[index].m_jacobianM0;
			const ndJacobian& jacobian1 = desc.m_jacobian[index].m_jacobianM1;
			const ndFloat32 relPosit = (jacobian0.m_linear * posit0 + jacobian1.m_linear * posit1).AddHorizontal().GetScalar();
			const ndFloat32 relVeloc = (jacobian0.m_linear * veloc0 + jacobian0.m_angular * omega0 + jacobian1.m_linear * veloc1 + jacobian1.m_angular * omega1).AddHorizontal().GetScalar();
			const ndFloat32 accel = CalculateSpringDamperAcceleration(desc.m_timestep, m_linearSpring, relPosit, m_linearDamper, relVeloc);
			SetMotorAcceleration(desc, accel);
			SetLowerFriction(desc, -m_linearMaxForce);
			SetHighFriction(desc, m_linearMaxForce);
		}
	}
#endif
}

void ndIkSwivelPositionEffector::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);
	SubmitLinearAxis(matrix0, matrix1, desc);
	SubmitAngularAxis(matrix0, matrix1, desc);
}