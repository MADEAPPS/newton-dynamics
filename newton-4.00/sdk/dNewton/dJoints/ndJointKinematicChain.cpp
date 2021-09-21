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
#include "ndJointKinematicChain.h"

#define D_TBIK_MAX_ANGLE	dFloat32 (120.0f * dDegreeToRad)
#define D_TBIK_PENETRATION_RECOVERY_ANGULAR_SPEED dFloat32 (0.1f) 
#define D_TBIK_PENETRATION_ANGULAR_LIMIT dFloat32 (10.0f * dDegreeToRad) 
#define D_TBIK_SMALL_DISTANCE_ERROR dFloat32 (1.0e-2f)
#define D_TBIK_SMALL_DISTANCE_ERROR2 (D_TBIK_SMALL_DISTANCE_ERROR * D_TBIK_SMALL_DISTANCE_ERROR)

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointKinematicChain)

//ndJointKinematicChain::ndJointKinematicChain(const dMatrix& basisInGlobalSpace, const dVector& pivotInGlobalSpace, ndBodyKinematic* const child, ndBodyKinematic* const parent)
//	:ndJointInverseDynamicsBase(6, child, parent)
//	,m_coneRotation(dGetIdentityMatrix())
//	,m_targetPosit(dVector::m_wOne)
//	,m_angle(dFloat32(0.0f))
//	,m_minAngle(-dFloat32(45.0f) * dDegreeToRad)
//	,m_maxAngle( dFloat32(45.0f) * dDegreeToRad)
//	,m_angularSpring(dFloat32(1000.0f))
//	,m_angularDamper(dFloat32(50.0f))
//	,m_angularRegularizer(dFloat32(5.0e-3f))
//	,m_linearSpring(dFloat32(1000.0f))
//	,m_linearDamper(dFloat32(50.0f))
//	,m_linearRegularizer(dFloat32(5.0e-3f))
//{
//	//CalculateLocalMatrix(basisInGlobalSpace, m_localMatrix0, m_localMatrix1);
//	//m_refPosit = m_localMatrix1.m_posit;
//	//m_pivot = m_body1->GetMatrix().UntransformVector(pivotInGlobalSpace);
//	//
//	//dVector dist(m_refPosit - m_pivot);
//	//m_maxDist = dSqrt(dist.DotProduct(dist & dVector::m_triplexMask).GetScalar());
//	//dAssert(m_maxDist > dFloat32(0.0f));
//	//
//	//m_localRotationBody1.m_front = dist.Normalize();
//	//m_localRotationBody1.m_right = (m_localRotationBody1.m_front.CrossProduct(m_localMatrix1.m_front)).Normalize();
//	//m_localRotationBody1.m_up = m_localRotationBody1.m_right.CrossProduct(m_localRotationBody1.m_front);
//	//m_localRotationBody1.m_posit = m_refPosit;
//	//m_localRotationBody0 = m_localRotationBody1 * m_body1->GetMatrix() * m_body0->GetMatrix().Inverse();
//	//
//	//SetTargetGlobalMatrix(basisInGlobalSpace.m_posit, dFloat32 (0.0f));
//	//dVector xxxx(basisInGlobalSpace.m_posit);
//	//xxxx.m_y += 0.1f;
//	//xxxx.m_x += 0.2f;
//	//SetTargetGlobalMatrix(xxxx, dFloat32 (0.0f));
//}

ndJointKinematicChain::ndJointKinematicChain(const dMatrix& hipReference, const dMatrix& pinAndpivot, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(6, child, parent, pinAndpivot)
	,m_coneRotation(dGetIdentityMatrix())
	,m_targetPosit(dVector::m_wOne)
	,m_angle(dFloat32(0.0f))
	,m_minAngle(-dFloat32(45.0f) * dDegreeToRad)
	,m_maxAngle(dFloat32(45.0f) * dDegreeToRad)
	,m_angularSpring(dFloat32(1000.0f))
	,m_angularDamper(dFloat32(50.0f))
	,m_angularRegularizer(dFloat32(5.0e-3f))
	,m_linearSpring(dFloat32(1000.0f))
	,m_linearDamper(dFloat32(50.0f))
	,m_linearRegularizer(dFloat32(5.0e-3f))
{
	//dAssert(0);
	//const dMatrix effectorMatrix = m_effectorBody->GetMatrix();
	//CalculateLocalMatrix(effectorMatrix, m_localMatrix0, m_localMatrix1);

	const dMatrix& parentMatrix = m_body1->GetMatrix();
	m_coronalFrame = hipReference * parentMatrix.Inverse();
	
	dVector dist(m_localMatrix1.m_posit - m_coronalFrame.m_posit);
	m_maxDist = dSqrt(dist.DotProduct(dist & dVector::m_triplexMask).GetScalar());
	dAssert(m_maxDist > dFloat32(0.0f));
	
	//dMatrix matrix(dGetIdentityMatrix());
	//matrix.m_front = dist.Normalize();
	//matrix.m_right = (matrix.m_front.CrossProduct(m_coronalFrame.m_front)).Normalize();
	//matrix.m_up = matrix.m_right.CrossProduct(matrix.m_front);
	//matrix = matrix * m_body1->GetMatrix();
	//m_localRotation = matrix * effectorMatrix.Inverse();
	//m_localRotation.m_posit = dVector::m_wOne;
	//
	////matrix.m_posit = pinAndPivot.m_posit;
	//////CalculateLocalMatrix(pinAndPivot, m_localMatrix0, m_localMatrix1);
	////CalculateLocalMatrix(matrix, m_localMatrix0, m_localMatrix1);
	//
	//SetTargetGlobalMatrix(effectorMatrix);
}

ndJointKinematicChain::ndJointKinematicChain(const dLoadSaveBase::dLoadDescriptor& desc)
	:ndJointBilateralConstraint(dLoadSaveBase::dLoadDescriptor(desc))
	,m_coneRotation(dGetIdentityMatrix())
	,m_targetPosit(dVector::m_wOne)
	,m_angle(dFloat32(0.0f))
	,m_minAngle(-dFloat32(45.0f) * dDegreeToRad)
	,m_maxAngle(dFloat32(45.0f) * dDegreeToRad)
	,m_angularSpring(dFloat32(1000.0f))
	,m_angularDamper(dFloat32(50.0f))
	,m_angularRegularizer(dFloat32(5.0e-3f))
	,m_linearSpring(dFloat32(1000.0f))
	,m_linearDamper(dFloat32(50.0f))
	,m_linearRegularizer(dFloat32(5.0e-3f))
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
	//m_targetPosit.m_w = dFloat32(1.0f);
	//m_targetPosit.m_w = dFloat32(1.0f);
	//SetTargetOffset(m_targetPosit);
}

ndJointKinematicChain::~ndJointKinematicChain()
{
}

void ndJointKinematicChain::Save(const dLoadSaveBase::dSaveDescriptor& desc) const
{
	dAssert(0);
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(dLoadSaveBase::dSaveDescriptor(desc, childNode));
	
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

void ndJointKinematicChain::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	//dMatrix localRotationBody1(m_localRotationBody1);
	//localRotationBody1.m_posit = m_targetPosit + m_pivot;
	//localRotationBody1.m_posit.m_w = dFloat32(1.0f);

	//dMatrix localMatrix1(m_coronalFrame);
	//localMatrix1.m_posit += (m_targetPosit & dVector::m_triplexMask);
	//const dMatrix matrix1 (localMatrix1 * m_body1->GetMatrix());

	//const dMatrix matrix0(m_localMatrix0 * m_body0->GetMatrix());
	const dMatrix matrix0(m_localMatrix0 * m_body0->GetMatrix());
	const dMatrix matrix1(m_localMatrix1 * m_body1->GetMatrix());
	
	//const dMatrix twistMatrix(dPitchMatrix(0.0f));
	//const dMatrix matrix0(twistMatrix * m_localRotationBody0 * m_body0->GetMatrix());
	//const dMatrix matrix1(m_coneRotation * localRotationBody1 * m_body1->GetMatrix());
	
	debugCallback.DrawFrame(matrix0);
	debugCallback.DrawFrame(matrix1);

	//debugCallback.DrawLine(matrix1.m_posit, matrix1.m_posit - matrix1.m_front.Scale(m_maxDist), dVector(1.0f, 1.0f, 0.0f, 1.0f));
	debugCallback.DrawLine(matrix0.m_posit, matrix0.m_posit - matrix0.m_front.Scale(m_maxDist), dVector(1.0f, 1.0f, 0.0f, 1.0f));
	
	//const dInt32 subdiv = 8;
	//const dFloat32 radius = debugCallback.m_debugScale;
	//dVector arch[subdiv + 1];
	//
	//// show twist angle limits
	//dFloat32 deltaTwist = m_maxAngle - m_minAngle;
	//if ((deltaTwist > dFloat32(1.0e-3f)) && (deltaTwist < dFloat32(2.0f) * dPi))
	//{
	//	dVector point(dFloat32(0.0f), dFloat32(radius), dFloat32(0.0f), dFloat32(1.0f));
	//	dFloat32 angleStep = dMin(m_maxAngle - m_minAngle, dFloat32(2.0f * dPi)) / subdiv;
	//	dFloat32 angle0 = m_minAngle;
	//
	//	dVector color(dFloat32(0.4f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
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

void ndJointKinematicChain::SetTargetLocalMatrix(const dMatrix& matrix)
{
	dAssert(0);
	//const dVector& offset, dFloat32 swivelAngle
	//m_angle = swivelAngle;
	//
	//dVector posit((offset - m_coronalFrame.m_posit) & dVector::m_triplexMask);
	//dFloat32 mag2 = posit.DotProduct(posit).GetScalar();
	//if (mag2 > (m_maxDist * m_maxDist))
	//{
	//	posit = posit.Normalize().Scale(m_maxDist);
	//}
	//
	//dAssert(posit.DotProduct(posit).GetScalar() > dFloat32(0.1f));
	//m_targetPosit = posit | dVector::m_wOne;
	//
	////m_localMatrix1.m_posit = m_targetPosit;
	////const dMatrix baseMatrix(m_localRotationBody1 * m_body1->GetMatrix());
	////const dVector dir0(baseMatrix.RotateVector(posit.Normalize()));
	////const dVector dir1(baseMatrix.m_front);
	////
	////dFloat32 cosAngleCos = dir0.DotProduct(dir1).GetScalar();
	////if (cosAngleCos >= dFloat32(0.998f))
	////{
	////	dFloat32 yawAngle = -CalculateAngle(dir0, baseMatrix.m_front, baseMatrix.m_up);
	////	dFloat32 rollAngle = -CalculateAngle(dir0, baseMatrix.m_front, baseMatrix.m_right);
	////	m_coneRotation = dYawMatrix(yawAngle) * dRollMatrix(rollAngle);
	////}
	////else
	////{
	////	const dVector lateralDir(dir1.CrossProduct(dir0).Normalize());
	////	const dFloat32 angle = dAcos(dClamp(cosAngleCos, dFloat32(-1.0f), dFloat32(1.0f)));
	////	const dQuaternion rotation(lateralDir, angle);
	////	const dMatrix coneMatrix (rotation, dVector::m_zero);
	////	m_coneRotation = baseMatrix * coneMatrix * baseMatrix.Inverse();
	////	m_coneRotation.m_posit = dVector::m_wOne;
	////}
}

void ndJointKinematicChain::SetTargetGlobalMatrix(const dMatrix& matrix)
{
	dAssert(0);
	//const dVector& offset, dFloat32 swivelAngle
	//dVector localOffset(m_body1->GetMatrix().UntransformVector(offset));
	//SetTargetLocalMatrix(localOffset, swivelAngle);
}

void ndJointKinematicChain::SubmitLinearLimits(ndConstraintDescritor& desc)
{
	dMatrix localMatrix1(m_coronalFrame);
	//localMatrix1.m_posit += (m_targetPosit & dVector::m_triplexMask);
	localMatrix1.m_posit = m_localMatrix1.m_posit;
	const dMatrix matrix1 (localMatrix1 * m_body1->GetMatrix());
	const dVector posit0 (m_body0->GetMatrix().TransformVector(m_localMatrix0.m_posit));

	// Cartesian motion
	const dVector step(posit0 - matrix1.m_posit);
	AddLinearRowJacobian(desc, posit0, matrix1.m_posit, matrix1[0]);
	SetMassSpringDamperAcceleration(desc, m_linearRegularizer, m_linearSpring, m_linearDamper);
	
	AddLinearRowJacobian(desc, posit0, matrix1.m_posit, matrix1[1]);
	SetMassSpringDamperAcceleration(desc, m_linearRegularizer, m_linearSpring, m_linearDamper);
	
	AddLinearRowJacobian(desc, posit0, matrix1.m_posit, matrix1[2]);
	SetMassSpringDamperAcceleration(desc, m_linearRegularizer, m_linearSpring, m_linearDamper);
}

void ndJointKinematicChain::SubmitAngularLimits(ndConstraintDescritor& desc)
{
	//const dMatrix twistMatrix(dPitchMatrix(m_angle));
	//const dMatrix matrix0(twistMatrix * m_localRotationBody0 * m_body0->GetMatrix());
	//const dMatrix matrix1(m_coneRotation * m_localRotationBody1 * m_body1->GetMatrix());
	//
	//const dMatrix pitchMatrix(matrix1 * matrix0.Inverse());
	//dFloat32 angle = -dAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
	//
	//AddAngularRowJacobian(desc, matrix0.m_front, -angle);
	//SetMassSpringDamperAcceleration(desc, m_angularRegularizer, m_angularSpring, m_angularDamper);
	//if ((m_maxAngle - m_minAngle) < (2.0f * dDegreeToRad))
	//{
	//	AddAngularRowJacobian(desc, matrix0.m_front, -angle);
	//	SetLowerFriction(desc, D_LCP_MAX_VALUE * dFloat32(0.1f));
	//}
	//else if (angle < m_minAngle)
	//{
	//	AddAngularRowJacobian(desc, matrix0.m_front, dFloat32(0.0f));
	//	const dFloat32 stopAccel = GetMotorZeroAcceleration(desc);
	//	const dFloat32 penetration = angle - m_minAngle;
	//	const dFloat32 recoveringAceel = -desc.m_invTimestep * D_TBIK_PENETRATION_RECOVERY_ANGULAR_SPEED * dMin(dAbs(penetration / D_TBIK_PENETRATION_ANGULAR_LIMIT), dFloat32(1.0f));
	//	SetMotorAcceleration(desc, stopAccel - recoveringAceel);
	//	SetLowerFriction(desc, dFloat32(0.0f));
	//}
	//else if (angle >= m_maxAngle)
	//{
	//	AddAngularRowJacobian(desc, matrix0.m_front, dFloat32(0.0f));
	//	const dFloat32 stopAccel = GetMotorZeroAcceleration(desc);
	//	const dFloat32 penetration = angle - m_maxAngle;
	//	const dFloat32 recoveringAceel = desc.m_invTimestep * D_TBIK_PENETRATION_RECOVERY_ANGULAR_SPEED * dMin(dAbs(penetration / D_TBIK_PENETRATION_ANGULAR_LIMIT), dFloat32(1.0f));
	//	SetMotorAcceleration(desc, stopAccel - recoveringAceel);
	//	SetHighFriction(desc, dFloat32(0.0f));
	//}
}

void ndJointKinematicChain::JacobianDerivative(ndConstraintDescritor& desc)
{
	SubmitLinearLimits(desc);
	//SubmitAngularLimits(desc);
}
