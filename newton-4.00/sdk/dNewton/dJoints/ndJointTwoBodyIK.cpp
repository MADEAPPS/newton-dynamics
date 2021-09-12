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
#include "ndJointTwoBodyIK.h"

#define D_TBIK_MAX_ANGLE	dFloat32 (120.0f * dDegreeToRad)
#define D_TBIK_PENETRATION_RECOVERY_ANGULAR_SPEED dFloat32 (0.1f) 
#define D_TBIK_PENETRATION_ANGULAR_LIMIT dFloat32 (10.0f * dDegreeToRad) 
#define D_TBIK_SMALL_DISTANCE_ERROR dFloat32 (1.0e-2f)
#define D_TBIK_SMALL_DISTANCE_ERROR2 (D_TBIK_SMALL_DISTANCE_ERROR * D_TBIK_SMALL_DISTANCE_ERROR)

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointTwoBodyIK)

ndJointTwoBodyIK::ndJointTwoBodyIK(const ndJointBilateralConstraint* const rootJoint, const dVector& locationInGlocalSpace, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointInverseDynamicsBase(6, child, parent)
	,m_pivotFrame(rootJoint->GetLocalMatrix1())
	,m_coneRotation(dGetIdentityMatrix())
	,m_targetPosit(dVector::m_zero)
	,m_referencePosit(dVector::m_zero)
	,m_angle(dFloat32(0.0f))
	,m_minAngle(-dFloat32(30.0f) * dDegreeToRad)
	,m_maxAngle(dFloat32(30.0f) * dDegreeToRad)
	,m_angularSpring(dFloat32(1000.0f))
	,m_angularDamper(dFloat32(50.0f))
	,m_angularRegularizer(dFloat32(5.0e-3f))
	,m_linearSpring(dFloat32(1000.0f))
	,m_linearDamper(dFloat32(50.0f))
	,m_linearRegularizer(dFloat32(5.0e-3f))
{
	dAssert(rootJoint->GetBody1() == m_body1);
	dMatrix matrix(m_pivotFrame * m_body1->GetMatrix());
	m_targetPosit = matrix.UntransformVector(locationInGlocalSpace);
	matrix.m_posit = locationInGlocalSpace;
	CalculateLocalMatrix(matrix, m_localMatrix0, m_localMatrix1);
	m_maxDist = dSqrt(m_targetPosit.DotProduct(m_targetPosit & dVector::m_triplexMask).GetScalar());
	m_referencePosit = m_localMatrix1.m_front.Scale (m_maxDist) | dVector::m_wOne;

//SetTargetOffset(dVector(0.0f, 0.2f, 0.4f, 0.0f));
//SetTargetOffset(dVector(0.0f, 0.0f, -0.4f, 0.0f));
//SetTargetOffset(m_referencePosit + dVector(0.0f, 0.0f, 0.0f, 0.0f));
//SetTargetOffset(m_referencePosit + dVector(-0.1f, 0.0f, 0.4f, 0.0f));
SetTargetOffset(m_referencePosit + dVector(-0.1f, -0.2f, 0.0f, 0.0f));
	
}

ndJointTwoBodyIK::ndJointTwoBodyIK(const dLoadSaveBase::dLoadDescriptor& desc)
	:ndJointInverseDynamicsBase(dLoadSaveBase::dLoadDescriptor(desc))
	,m_pivotFrame(dGetIdentityMatrix())
	,m_coneRotation(dGetIdentityMatrix())
	,m_targetPosit(dVector::m_zero)
	,m_referencePosit(dVector::m_zero)
	,m_angle(dFloat32(0.0f))
	,m_minAngle(-dFloat32(30.0f) * dDegreeToRad)
	,m_maxAngle(dFloat32(30.0f) * dDegreeToRad)
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
	//m_pivotFrame = xmlGetMatrix(xmlNode, "referenceFrameBody1");
	//m_maxConeAngle = xmlGetFloat (xmlNode, "maxConeAngle");
	//m_minTwistAngle = xmlGetFloat (xmlNode, "minTwistAngle");
	//m_maxTwistAngle = xmlGetFloat (xmlNode, "maxTwistAngle");
	//m_angularSpring = xmlGetFloat (xmlNode, "angularSpring");
	//m_angularDamper = xmlGetFloat (xmlNode, "angularDamper");
	//m_angularRegularizer = xmlGetFloat (xmlNode, "angularRegularizer");
	//
	//m_linearSpring = xmlGetFloat(xmlNode, "linearSpring");
	//m_linearDamper = xmlGetFloat(xmlNode, "linearDamper");
	//m_linearRegularizer = xmlGetFloat(xmlNode, "linearRegularizer");
}

ndJointTwoBodyIK::~ndJointTwoBodyIK()
{
}

void ndJointTwoBodyIK::Save(const dLoadSaveBase::dSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointInverseDynamicsBase::Save(dLoadSaveBase::dSaveDescriptor(desc, childNode));

	dAssert(0);
	//xmlSaveParam(childNode, "referenceFrameBody1", m_pivotFrame);
	//xmlSaveParam(childNode, "maxConeAngle", m_maxConeAngle);
	//xmlSaveParam(childNode, "minTwistAngle", m_minTwistAngle);
	//xmlSaveParam(childNode, "maxTwistAngle", m_maxTwistAngle);
	//xmlSaveParam(childNode, "angularSpring", m_angularSpring);
	//xmlSaveParam(childNode, "angularDamper", m_angularDamper);
	//xmlSaveParam(childNode, "angularRegularizer", m_angularRegularizer);
	//
	//xmlSaveParam(childNode, "linearSpring", m_linearSpring);
	//xmlSaveParam(childNode, "linearDamper", m_linearDamper);
	//xmlSaveParam(childNode, "linearRegularizer", m_linearRegularizer);
}

void ndJointTwoBodyIK::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	dMatrix matrix0;
	dMatrix matrix1;
	dMatrix matrix2(m_pivotFrame * m_body1->GetMatrix());
	CalculateGlobalMatrix(matrix0, matrix1);
	
	debugCallback.DrawFrame(matrix1);
	debugCallback.DrawFrame(matrix2);
	debugCallback.DrawLine(matrix1.m_posit, matrix1.m_posit - matrix1.m_front.Scale(m_maxDist), dVector(1.0f, 1.0f, 0.0f, 1.0f));

	dFloat32 scale = debugCallback.GetScale();
	debugCallback.SetScale(scale * dFloat32 (0.5f));
	debugCallback.DrawFrame(matrix0);
	debugCallback.SetScale(scale);

	const dInt32 subdiv = 8;
	const dVector& coneDir0 = matrix0.m_front;
	const dVector& coneDir1 = matrix1.m_front;
	dFloat32 cosAngleCos = coneDir0.DotProduct(coneDir1).GetScalar();
	dMatrix coneRotation(dGetIdentityMatrix());
	if (cosAngleCos < dFloat32(0.9999f))
	{
		dVector lateralDir(coneDir1.CrossProduct(coneDir0));
		dFloat32 mag2 = lateralDir.DotProduct(lateralDir).GetScalar();
		if (mag2 > dFloat32(1.0e-4f))
		{
			lateralDir = lateralDir.Scale(dFloat32(1.0f) / dSqrt(mag2));
			coneRotation = dMatrix(dQuaternion(lateralDir, dAcos(dClamp(cosAngleCos, dFloat32(-1.0f), dFloat32(1.0f)))), matrix1.m_posit);
		}
		else
		{
			lateralDir = matrix0.m_up.Scale(-dFloat32(1.0f));
			coneRotation = dMatrix(dQuaternion(matrix0.m_up, dFloat32(180.0f) * dDegreeToRad), matrix1.m_posit);
		}
	}
	else if (cosAngleCos < -dFloat32(0.9999f))
	{
		coneRotation[0][0] = dFloat32(-1.0f);
		coneRotation[1][1] = dFloat32(-1.0f);
	}
	
	const dFloat32 radius = debugCallback.m_debugScale;
	dVector arch[subdiv + 1];
	
	// show twist angle limits
	dFloat32 deltaTwist = m_maxAngle - m_minAngle;
	if ((deltaTwist > dFloat32(1.0e-3f)) && (deltaTwist < dFloat32(2.0f) * dPi))
	{
		dMatrix pitchMatrix(matrix1 * coneRotation);
		pitchMatrix.m_posit = matrix1.m_posit;
	
		dVector point(dFloat32(0.0f), dFloat32(radius), dFloat32(0.0f), dFloat32(0.0f));
	
		dFloat32 angleStep = dMin(m_maxAngle - m_minAngle, dFloat32(2.0f * dPi)) / subdiv;
		dFloat32 angle0 = m_minAngle;
	
		dVector color(dFloat32(0.4f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
		for (dInt32 i = 0; i <= subdiv; i++)
		{
			arch[i] = pitchMatrix.TransformVector(dPitchMatrix(angle0).RotateVector(point));
			debugCallback.DrawLine(pitchMatrix.m_posit, arch[i], color);
			angle0 += angleStep;
		}
	
		for (dInt32 i = 0; i < subdiv; i++)
		{
			debugCallback.DrawLine(arch[i], arch[i + 1], color);
		}
	}
}

//dMatrix ndJointTwoBodyIK::GetTargetRotation() const
//{
//	dMatrix tmp(m_localMatrix1);
//	tmp.m_posit = m_pivotFrame.m_posit;
//	return tmp;
//}

//void ndJointTwoBodyIK::SetTargetRotation(const dMatrix& matrix)
//{
//	dMatrix tmp(matrix);
//	tmp.m_posit = m_localMatrix1.m_posit;
//	m_localMatrix1 = tmp;
//}

//void ndJointTwoBodyIK::GetAngularSpringDamperRegularizer(dFloat32& spring, dFloat32& damper, dFloat32& regularizer) const
//{
//	spring = m_angularSpring;
//	damper = m_angularDamper;
//	regularizer = m_angularRegularizer;
//}

//void ndJointTwoBodyIK::SetAngularSpringDamperRegularizer(dFloat32 spring, dFloat32 damper, dFloat32 regularizer)
//{
//	m_angularSpring = dMax(spring, dFloat32(0.0f));
//	m_angularDamper = dMax(damper, dFloat32(0.0f));
//	m_angularRegularizer = dMax(regularizer, dFloat32(1.0e-4f));
//}

//void ndJointTwoBodyIK::SetTwistLimits(dFloat32 minAngle, dFloat32 maxAngle)
//{
//	m_minTwistAngle = -dAbs(minAngle);
//	m_maxTwistAngle = dAbs(maxAngle);
//}

//void ndJointTwoBodyIK::GetTwistLimits(dFloat32& minAngle, dFloat32& maxAngle) const
//{
//	minAngle = m_minTwistAngle;
//	maxAngle = m_maxTwistAngle;
//}

//dFloat32 ndJointTwoBodyIK::GetMaxConeAngle() const
//{
//	return m_maxConeAngle;
//}

//void ndJointTwoBodyIK::SetConeLimit(dFloat32 maxConeAngle)
//{
//	m_maxConeAngle = dMin (dAbs(maxConeAngle), D_TBIK_MAX_ANGLE * dFloat32 (0.999f));
//}

//dVector ndJointTwoBodyIK::GetTargetPosition() const
//{
//	return m_localMatrix1.m_posit;
//}

//void ndJointTwoBodyIK::SetTargetPosition(const dVector& posit)
//{
//	dAssert(posit.m_w == dFloat32(1.0f));
//	m_localMatrix1.m_posit = posit;
//}

//void ndJointTwoBodyIK::GetLinearSpringDamperRegularizer(dFloat32& spring, dFloat32& damper, dFloat32& regularizer) const
//{
//	spring = m_linearSpring;
//	damper = m_linearDamper;
//	regularizer = m_linearRegularizer;
//}

//void ndJointTwoBodyIK::SetLinearSpringDamperRegularizer(dFloat32 spring, dFloat32 damper, dFloat32 regularizer)
//{
//	m_linearSpring = dMax(spring, dFloat32(0.0f));
//	m_linearDamper = dMax(damper, dFloat32(0.0f));
//	m_linearRegularizer = dMax(regularizer, dFloat32(1.0e-4f));
//}

//dMatrix ndJointTwoBodyIK::GetTargetMatrix() const
//{
//	return m_localMatrix1;
//}

//void ndJointTwoBodyIK::SetTargetMatrix(const dMatrix& matrix)
//{
//	m_localMatrix1 = matrix;
//}

//void ndJointTwoBodyIK::SubmitAngularAxisCartesianApproximation(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc)
//{
//	dFloat32 pitchAngle = -CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);
//	dFloat32 coneAngle = dAcos(dClamp(matrix1.m_front.DotProduct(matrix0.m_front).GetScalar(), dFloat32(-1.0f), dFloat32(1.0f)));
//	if (coneAngle >= m_maxConeAngle)
//	{
//		dAssert(m_maxConeAngle == dFloat32(0.0f));
//		//this is a hinge joint
//		AddAngularRowJacobian(desc, matrix0.m_front, -pitchAngle);
//		SetMassSpringDamperAcceleration(desc, m_angularRegularizer, m_angularSpring, m_angularDamper);
//
//		// apply cone limits
//		// two rows to restrict rotation around the parent coordinate system
//		dFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
//		AddAngularRowJacobian(desc, matrix1.m_up, angle0);
//
//		dFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
//		AddAngularRowJacobian(desc, matrix1.m_right, angle1);
//	}
//	else
//	{
//		SubmitPidRotation(matrix0, matrix1, desc);
//	}
//
//	SubmitTwistLimits(matrix0.m_front, pitchAngle, desc);
//}
//
//void ndJointTwoBodyIK::SubmitTwistLimits(const dVector& pin, dFloat32 angle, ndConstraintDescritor& desc)
//{
//	if ((m_maxTwistAngle - m_minTwistAngle) < (2.0f * dDegreeToRad))
//	{
//		AddAngularRowJacobian(desc, pin, -angle);
//		// force this limit to be bound
//		SetLowerFriction(desc, D_LCP_MAX_VALUE * dFloat32 (0.1f));
//	}
//	else
//	{
//		if (angle < m_minTwistAngle)
//		{
//			AddAngularRowJacobian(desc, pin, dFloat32(0.0f));
//			const dFloat32 stopAccel = GetMotorZeroAcceleration(desc);
//			const dFloat32 penetration = angle - m_minTwistAngle;
//			const dFloat32 recoveringAceel = -desc.m_invTimestep * D_TBIK_PENETRATION_RECOVERY_ANGULAR_SPEED * dMin(dAbs(penetration / D_TBIK_PENETRATION_ANGULAR_LIMIT), dFloat32(1.0f));
//			SetMotorAcceleration(desc, stopAccel - recoveringAceel);
//			SetLowerFriction(desc, dFloat32(0.0f));
//		}
//		else if (angle >= m_maxTwistAngle)
//		{
//			AddAngularRowJacobian(desc, pin, dFloat32(0.0f));
//			const dFloat32 stopAccel = GetMotorZeroAcceleration(desc);
//			const dFloat32 penetration = angle - m_maxTwistAngle;
//			const dFloat32 recoveringAceel = desc.m_invTimestep * D_TBIK_PENETRATION_RECOVERY_ANGULAR_SPEED * dMin(dAbs(penetration / D_TBIK_PENETRATION_ANGULAR_LIMIT), dFloat32(1.0f));
//			SetMotorAcceleration(desc, stopAccel - recoveringAceel);
//			SetHighFriction(desc, dFloat32(0.0f));
//		}
//	}
//}
//
//void ndJointTwoBodyIK::SubmitAngularAxis(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc)
//{
//	SubmitPidRotation(matrix0, matrix1, desc);
//
//	dVector lateralDir(matrix1[0].CrossProduct(matrix0[0]));
//	dAssert(lateralDir.DotProduct(lateralDir).GetScalar() > 1.0e-6f);
//	lateralDir = lateralDir.Normalize();
//	dFloat32 coneAngle = dAcos(dClamp(matrix1.m_front.DotProduct(matrix0.m_front).GetScalar(), dFloat32(-1.0f), dFloat32(1.0f)));
//	dMatrix coneRotation(dQuaternion(lateralDir, coneAngle), matrix1.m_posit);
//
//	if (coneAngle > m_maxConeAngle)
//	{
//		AddAngularRowJacobian(desc, lateralDir, 0.0f);
//		const dFloat32 stopAccel = GetMotorZeroAcceleration(desc);
//		const dFloat32 penetration = coneAngle - m_maxConeAngle;
//		const dFloat32 recoveringAceel = desc.m_invTimestep * D_TBIK_PENETRATION_RECOVERY_ANGULAR_SPEED * dMin(dAbs(penetration / D_TBIK_PENETRATION_ANGULAR_LIMIT), dFloat32(1.0f));
//		SetMotorAcceleration(desc, stopAccel - recoveringAceel);
//		SetHighFriction(desc, dFloat32(0.0f));
//	}
//
//	dMatrix pitchMatrix(matrix1 * coneRotation * matrix0.Inverse());
//	dFloat32 pitchAngle = -dAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
//	SubmitTwistLimits(matrix0.m_front, pitchAngle, desc);
//}
//
//void ndJointTwoBodyIK::SubmitPidRotation(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc)
//{
//	dQuaternion q0(matrix0);
//	dQuaternion q1(matrix1);
//	if (q1.DotProduct(q0).GetScalar() < dFloat32(0.0f))
//	{
//		q1 = q1.Scale (dFloat32 (-1.0f));
//	}
//
//	dQuaternion dq(q1.Inverse() * q0);
//	dVector pin(dq.m_x, dq.m_y, dq.m_z, dFloat32(0.0f));
//
//	dFloat32 dirMag2 = pin.DotProduct(pin).GetScalar();
//	if (dirMag2 > dFloat32(dFloat32(1.0e-7f)))
//	{
//		dFloat32 dirMag = dSqrt(dirMag2);
//		pin = pin.Scale(dFloat32(1.0f) / dirMag);
//		dFloat32 angle = dFloat32(2.0f) * dAtan2(dirMag, dq.m_w);
//
//		dMatrix basis(pin);
//		AddAngularRowJacobian(desc, basis[0], -angle);
//		SetMassSpringDamperAcceleration(desc, m_angularRegularizer, m_angularSpring, m_angularDamper);
//
//		AddAngularRowJacobian(desc, basis[1], dFloat32 (0.0f));
//		SetMassSpringDamperAcceleration(desc, m_angularRegularizer, m_angularSpring, m_angularDamper);
//
//		AddAngularRowJacobian(desc, basis[2], dFloat32(0.0f));
//		SetMassSpringDamperAcceleration(desc, m_angularRegularizer, m_angularSpring, m_angularDamper);
//	}
//	else
//	{
//		dFloat32 pitchAngle = CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);
//		AddAngularRowJacobian(desc, matrix1[0], pitchAngle);
//		SetMassSpringDamperAcceleration(desc, m_angularRegularizer, m_angularSpring, m_angularDamper);
//		
//		dFloat32 yawAngle = CalculateAngle(matrix0[0], matrix1[0], matrix1[1]);
//		AddAngularRowJacobian(desc, matrix1[1], yawAngle);
//		SetMassSpringDamperAcceleration(desc, m_angularRegularizer, m_angularSpring, m_angularDamper);
//		
//		dFloat32 rollAngle = CalculateAngle(matrix0[0], matrix1[0], matrix1[2]);
//		AddAngularRowJacobian(desc, matrix1[2], rollAngle);
//		SetMassSpringDamperAcceleration(desc, m_angularRegularizer, m_angularSpring, m_angularDamper);
//	}
//}

void ndJointTwoBodyIK::SetTargetOffset(const dVector& offset)
{
	dVector posit(offset & dVector::m_triplexMask);
	dFloat32 mag2 = posit.DotProduct(posit).GetScalar();
	if (mag2 > (m_maxDist * m_maxDist))
	{
		posit = posit.Normalize().Scale(m_maxDist);
	}

	dAssert(posit.DotProduct(posit).GetScalar() > dFloat32(0.1f));
	m_targetPosit = posit | dVector::m_wOne;

	const dMatrix baseMatrix(m_pivotFrame * m_body1->GetMatrix());
	const dVector dir0(baseMatrix.RotateVector(posit.Normalize()));
	const dVector dir1(baseMatrix.m_front);

	dFloat32 cosAngleCos = dir0.DotProduct(dir1).GetScalar();
	if (cosAngleCos >= dFloat32(0.998f))
	{
		dFloat32 yawAngle = -CalculateAngle(dir0, baseMatrix.m_front, baseMatrix.m_up);
		dFloat32 rollAngle = -CalculateAngle(dir0, baseMatrix.m_front, baseMatrix.m_right);
		m_coneRotation = dYawMatrix(yawAngle) * dRollMatrix(rollAngle);
	}
	else
	{
		const dVector lateralDir(dir1.CrossProduct(dir0).Normalize());
		const dFloat32 angle = dAcos(dClamp(cosAngleCos, dFloat32(-1.0f), dFloat32(1.0f)));
		const dQuaternion rotation(lateralDir, angle);
		const dMatrix coneMatrix (rotation, dVector::m_zero);
		m_coneRotation = m_pivotFrame * coneMatrix * m_pivotFrame.Inverse();
		m_coneRotation.m_posit = dVector::m_wOne;
	}
}

dMatrix ndJointTwoBodyIK::CalculateBaseMatrix() const
{
	dMatrix matrix (m_pivotFrame);
	matrix.m_posit += matrix.RotateVector(m_targetPosit);
	return matrix;
}

void ndJointTwoBodyIK::JacobianDerivative(ndConstraintDescritor& desc)
{
	dMatrix matrix0;
	dMatrix matrix1;

	//m_localMatrix1 = CalculateBaseMatrix();
	//m_localMatrix0 = m_coneRotation * m_localMatrix1 * m_body1->GetMatrix() * m_body0->GetMatrix().Inverse();
	m_localMatrix1 = m_coneRotation * CalculateBaseMatrix();
	CalculateGlobalMatrix(matrix0, matrix1);
	//SubmitAngularLimits(matrix0, matrix1, desc);
	SubmitLinearLimits(matrix0, matrix1, desc);
}

void ndJointTwoBodyIK::SubmitLinearLimits(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc)
{
	const dVector step(matrix0.m_posit - matrix1.m_posit);
	if (step.DotProduct(step).GetScalar() <= D_TBIK_SMALL_DISTANCE_ERROR2)
	{
		// Cartesian motion
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[0]);
		SetMassSpringDamperAcceleration(desc, m_linearRegularizer, m_linearSpring, m_linearDamper);

		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[1]);
		SetMassSpringDamperAcceleration(desc, m_linearRegularizer, m_linearSpring, m_linearDamper);

		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[2]);
		SetMassSpringDamperAcceleration(desc, m_linearRegularizer, m_linearSpring, m_linearDamper);
	}
	else
	{
		const dMatrix basis(step.Normalize());

		// move alone the diagonal;
		AddLinearRowJacobian(desc, matrix1.m_posit, matrix1.m_posit, basis[2]);
		AddLinearRowJacobian(desc, matrix1.m_posit, matrix1.m_posit, basis[1]);
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, basis[0]);
		SetMassSpringDamperAcceleration(desc, m_linearRegularizer, m_linearSpring, m_linearDamper);
	}
}

void ndJointTwoBodyIK::SubmitAngularLimits(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc)
{
return;
	//dFloat32 cosAngleCos = matrix1.m_front.DotProduct(matrix0.m_front).GetScalar();
	//if (cosAngleCos >= dFloat32(0.998f))
	//{
	//	// special case where the front axis are almost aligned
	//	// solve by using Cartesian approximation
	//	SubmitAngularAxisCartesianApproximation(matrix0, matrix1, desc);
	//}
	//else
	//{
	//	SubmitAngularAxis(matrix0, matrix1, desc);
	//}
}
