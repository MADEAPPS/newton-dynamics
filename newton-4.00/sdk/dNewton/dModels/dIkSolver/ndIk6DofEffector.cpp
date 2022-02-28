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

ndIk6DofEffector::ndIk6DofEffector(const ndMatrix& globalPinAndPivot, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(6, child, parent, globalPinAndPivot)
	,m_targetFrame(dGetIdentityMatrix())
	//,m_baseFrame(m_localMatrix1)
	//,m_angle(ndFloat32(0.0f))
	//,m_minAngle(-ndFloat32(45.0f) * ndDegreeToRad)
	//,m_maxAngle(ndFloat32(45.0f) * ndDegreeToRad)
	//,m_angularSpring(ndFloat32(1000.0f))
	//,m_angularDamper(ndFloat32(50.0f))
	//,m_angularRegularizer(ndFloat32(5.0e-3f))
	,m_linearSpring(ndFloat32(1000.0f))
	,m_linearDamper(ndFloat32(50.0f))
	,m_linearRegularizer(ndFloat32(5.0e-3f))
	//,m_linearMode(true)
	//,m_angularMode(true)
	,m_linearAxis (0x0f)
{
	SetSolverModel(m_jointkinematicCloseLoop);
}

ndIk6DofEffector::ndIk6DofEffector(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointBilateralConstraint(ndLoadSaveBase::ndLoadDescriptor(desc))
	,m_targetFrame(dGetIdentityMatrix())
	//,m_angle(ndFloat32(0.0f))
	//,m_minAngle(-ndFloat32(45.0f) * ndDegreeToRad)
	//,m_maxAngle(ndFloat32(45.0f) * ndDegreeToRad)
	//,m_angularSpring(ndFloat32(1000.0f))
	//,m_angularDamper(ndFloat32(50.0f))
	//,m_angularRegularizer(ndFloat32(5.0e-3f))
	,m_linearSpring(ndFloat32(1000.0f))
	,m_linearDamper(ndFloat32(50.0f))
	,m_linearRegularizer(ndFloat32(5.0e-3f))
	//,m_linearMode(true)
	//,m_angularMode(true)
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
	ndInt8 linearAxis = m_linearAxis & ~(1 << 0);
	m_linearAxis = linearAxis | ((state ? 1 : 0) << 0);
}

void ndIk6DofEffector::EnableAxisY(bool state)
{
	ndInt8 linearAxis = m_linearAxis & ~(1 << 1);
	m_linearAxis = linearAxis | ((state ? 1 : 0) << 1);
}

void ndIk6DofEffector::EnableAxisZ(bool state)
{
	ndInt8 linearAxis = m_linearAxis & ~(1 << 2);
	m_linearAxis = linearAxis | ((state ? 1 : 0) << 2);
}

ndMatrix ndIk6DofEffector::GetOffsetMatrix() const
{
	return m_targetFrame;
}

void ndIk6DofEffector::SetOffsetMatrix(const ndMatrix& matrix)
{
	m_targetFrame = matrix;
}

//bool ndIk6DofEffector::IsLinearMode() const
//{
//	return m_linearMode;
//}
//
//bool ndIk6DofEffector::IsAngularMode() const
//{
//	return m_angularMode;
//}
//
//void ndIk6DofEffector::SetMode(bool linear, bool angular)
//{
//	m_linearMode = linear;
//	m_angularMode = angular;
//}
//
//void ndIk6DofEffector::SetLinearSpringDamper(ndFloat32 regularizer, ndFloat32 springConst, ndFloat32 damperConst)
//{
//	m_linearSpring = springConst;
//	m_linearDamper = damperConst;
//	m_linearRegularizer = regularizer;
//}
//
//void ndIk6DofEffector::GetLinearSpringDamper(ndFloat32& regularizer, ndFloat32& springConst, ndFloat32& damperConst) const
//{
//	springConst = m_linearSpring;
//	damperConst = m_linearDamper;
//	regularizer = m_linearRegularizer;
//}
//
//void ndIk6DofEffector::SetAngularSpringDamper(ndFloat32 regularizer, ndFloat32 springConst, ndFloat32 damperConst)
//{
//	m_angularSpring = springConst;
//	m_angularDamper = damperConst;
//	m_angularRegularizer = regularizer;
//}
//
//void ndIk6DofEffector::GetAngularSpringDamper(ndFloat32& regularizer, ndFloat32& springConst, ndFloat32& damperConst) const
//{
//	springConst = m_angularSpring;
//	damperConst = m_angularDamper;
//	regularizer = m_angularRegularizer;
//}

void ndIk6DofEffector::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	//ndMatrix localRotationBody1(m_localRotationBody1);
	//localRotationBody1.m_posit = m_targetPosit + m_pivot;
	//localRotationBody1.m_posit.m_w = ndFloat32(1.0f);

	//ndMatrix localMatrix1(m_coronalFrame);
	//localMatrix1.m_posit += (m_targetPosit & ndVector::m_triplexMask);
	//const ndMatrix matrix1 (localMatrix1 * m_body1->GetMatrix());

	//const ndMatrix matrix0(m_localMatrix0 * m_body0->GetMatrix());
	const ndMatrix matrix0(m_localMatrix0 * m_body0->GetMatrix());
	const ndMatrix matrix1(m_localMatrix1 * m_body1->GetMatrix());
	
	//const ndMatrix twistMatrix(dPitchMatrix(0.0f));
	//const ndMatrix matrix0(twistMatrix * m_localRotationBody0 * m_body0->GetMatrix());
	//const ndMatrix matrix1(m_coneRotation * localRotationBody1 * m_body1->GetMatrix());
	
	debugCallback.DrawFrame(matrix0);
	debugCallback.DrawFrame(matrix1);

	//debugCallback.DrawLine(matrix1.m_posit, matrix1.m_posit - matrix1.m_front.Scale(m_maxDist), ndVector(1.0f, 1.0f, 0.0f, 1.0f));
	//debugCallback.DrawLine(matrix0.m_posit, matrix0.m_posit - matrix0.m_front.Scale(m_maxDist), ndVector(1.0f, 1.0f, 0.0f, 1.0f));
	
	//const dInt32 subdiv = 8;
	//const ndFloat32 radius = debugCallback.m_debugScale;
	//ndVector arch[subdiv + 1];
	//
	//// show twist angle limits
	//ndFloat32 deltaTwist = m_maxAngle - m_minAngle;
	//if ((deltaTwist > ndFloat32(1.0e-3f)) && (deltaTwist < ndFloat32(2.0f) * dPi))
	//{
	//	ndVector point(ndFloat32(0.0f), ndFloat32(radius), ndFloat32(0.0f), ndFloat32(1.0f));
	//	ndFloat32 angleStep = dMin(m_maxAngle - m_minAngle, ndFloat32(2.0f * dPi)) / subdiv;
	//	ndFloat32 angle0 = m_minAngle;
	//
	//	ndVector color(ndFloat32(0.4f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	//	for (dInt32 i = 0; i <= subdiv; i++)
	//	{
	//		arch[i] = matrix1.TransformVector(dPitchMatrix(angle0).RotateVector(point));
	//		debugCallback.DrawLine(matrix1.m_posit, arch[i], color);
	//		angle0 += angleStep;
	//	}
	//	
	//	for (dInt32 i = 0; i < subdiv; i++)
	//	{
	//		debugCallback.DrawLine(arch[i], arch[i + 1], color);
	//	}
	//}
}

//ndMatrix ndIk6DofEffector::GetReferenceMatrix() const
//{
//	return m_baseFrame;
//}
//
//void ndIk6DofEffector::SetTargetMatrix(const ndMatrix& localMatrix)
//{
//	m_localMatrix1 = localMatrix;
//}
//
//void ndIk6DofEffector::SubmitAngularAxisCartesianApproximation(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
//{
//	ndFloat32 m_twistAngleSpring = m_angularSpring;
//	ndFloat32 m_twistAngleDamper = m_angularDamper;
//	ndFloat32 m_twistAngleRegularizer = m_angularRegularizer;
//	ndFloat32 m_coneAngleSpring = m_angularSpring;
//	ndFloat32 m_coneAngleDamper = m_angularDamper;
//	ndFloat32 m_coneAngleRegularizer = m_angularRegularizer;
//
//	ndFloat32 twistAngle = CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);
//	AddAngularRowJacobian(desc, matrix0.m_front, twistAngle);
//	SetMassSpringDamperAcceleration(desc, m_twistAngleRegularizer, m_twistAngleSpring, m_twistAngleDamper);
//
//	ndFloat32 yawAngle = CalculateAngle(matrix0[0], matrix1[0], matrix1[1]);
//	AddAngularRowJacobian(desc, matrix1[1], yawAngle);
//	SetMassSpringDamperAcceleration(desc, m_coneAngleRegularizer, m_coneAngleSpring, m_coneAngleDamper);
//
//	ndFloat32 rollAngle = CalculateAngle(matrix0[0], matrix1[0], matrix1[2]);
//	AddAngularRowJacobian(desc, matrix1[2], rollAngle);
//	SetMassSpringDamperAcceleration(desc, m_coneAngleRegularizer, m_coneAngleSpring, m_coneAngleDamper);
//}
//
//void ndIk6DofEffector::SubmitAngularAxis(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
//{
//	ndFloat32 m_twistAngleSpring = m_angularSpring;
//	ndFloat32 m_twistAngleDamper = m_angularDamper;
//	ndFloat32 m_twistAngleRegularizer = m_angularRegularizer;
//	ndFloat32 m_coneAngleSpring = m_angularSpring;
//	ndFloat32 m_coneAngleDamper = m_angularDamper;
//	ndFloat32 m_coneAngleRegularizer = m_angularRegularizer;
//
//	const ndVector dir0(matrix0.m_front);
//	const ndVector dir1(matrix1.m_front);
//	const ndVector lateralDir(dir1.CrossProduct(dir0).Normalize());
//	dAssert(lateralDir.DotProduct(lateralDir).GetScalar() > 1.0e-6f);
//	ndFloat32 cosAngleCos = dir1.DotProduct(dir0).GetScalar();
//	ndFloat32 coneAngle = ndAcos(dClamp(cosAngleCos, ndFloat32(-1.0f), ndFloat32(1.0f)));
//	const ndQuaternion rotation(lateralDir, coneAngle);
//	const ndMatrix coneMatrix(rotation, ndVector::m_zero);
//	const ndMatrix pitchMatrix = matrix1 * coneMatrix * matrix0.Inverse();
//	ndFloat32 twistAngle = ndAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
//
//	AddAngularRowJacobian(desc, dir0, twistAngle);
//	SetMassSpringDamperAcceleration(desc, m_twistAngleRegularizer, m_twistAngleSpring, m_twistAngleDamper);
//
//	ndVector sideDir(lateralDir.CrossProduct(dir0));
//	AddAngularRowJacobian(desc, sideDir, ndFloat32(0.0f));
//	SetMassSpringDamperAcceleration(desc, m_coneAngleRegularizer, m_coneAngleSpring, m_coneAngleDamper);
//
//	AddAngularRowJacobian(desc, lateralDir, -coneAngle);
//	SetMassSpringDamperAcceleration(desc, m_coneAngleRegularizer, m_coneAngleSpring, m_coneAngleDamper);
//}

void ndIk6DofEffector::SubmitLinearAxis(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	//const ndMatrix pins(m_baseFrame * m_body1->GetMatrix());
	//AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, pins[0]);
	//SetMassSpringDamperAcceleration(desc, m_linearRegularizer, m_linearSpring, m_linearDamper);
	//
	//AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, pins[1]);
	//SetMassSpringDamperAcceleration(desc, m_linearRegularizer, m_linearSpring, m_linearDamper);
	//
	//AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, pins[2]);
	//SetMassSpringDamperAcceleration(desc, m_linearRegularizer, m_linearSpring, m_linearDamper);

	ndVector posit1(matrix1.TransformVector(m_targetFrame.m_posit));
	for (ndInt32 i = 0; i < 3; i++)
	{
		if (m_linearAxis & (1 << i))
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
	//if (m_linearMode)
	//{
	//	SubmitLinearAxis(matrix0, matrix1, desc);
	//}
	//
	//if (m_angularMode)
	//{
	//	ndFloat32 cosAngleCos = matrix1.m_front.DotProduct(matrix0.m_front).GetScalar();
	//	if (cosAngleCos >= ndFloat32(0.998f))
	//	{
	//		// special case where the front axis are almost aligned
	//		// solve by using Cartesian approximation
	//		SubmitAngularAxisCartesianApproximation(matrix0, matrix1, desc);
	//	}
	//	else
	//	{
	//		SubmitAngularAxis(matrix0, matrix1, desc);
	//	}
	//}

	SubmitLinearAxis(matrix0, matrix1, desc);
}