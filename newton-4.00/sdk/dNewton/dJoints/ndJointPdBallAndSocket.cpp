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
#include "ndJointPdBallAndSocket.h"

#define D_PD_MAX_ANGLE	ndFloat32 (120.0f * ndDegreeToRad)
#define D_PD_PENETRATION_RECOVERY_ANGULAR_SPEED ndFloat32 (0.1f) 
#define D_PD_PENETRATION_ANGULAR_LIMIT ndFloat32 (10.0f * ndDegreeToRad) 

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointPdBallAndSocket)

ndJointPdBallAndSocket::ndJointPdBallAndSocket(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(8, child, parent, pinAndPivotFrame)
	,m_pivotFrame(m_localMatrix1)
	,m_minTwistAngle(-ndFloat32(1.0e10f))
	,m_maxTwistAngle(ndFloat32(1.0e10f))
	,m_twistAngleSpring(ndFloat32(1000.0f))
	,m_twistAngleDamper(ndFloat32(50.0f))
	,m_twistAngleRegularizer(ndFloat32(5.0e-3f))
	,m_maxConeAngle(ndFloat32(1.0e10f))
	,m_coneAngleSpring(ndFloat32(1000.0f))
	,m_coneAngleDamper(ndFloat32(50.0f))
	,m_coneAngleRegularizer(ndFloat32(5.0e-3f))
	,m_linearSpring(ndFloat32(0.0f))
	,m_linearDamper(ndFloat32(0.0f))
	,m_linearRegularizer(ndFloat32(0.0f))
{
}

ndJointPdBallAndSocket::ndJointPdBallAndSocket(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointBilateralConstraint(ndLoadSaveBase::ndLoadDescriptor(desc))
	,m_pivotFrame(dGetIdentityMatrix())
	,m_minTwistAngle(-ndFloat32(1.0e10f))
	,m_maxTwistAngle(ndFloat32(1.0e10f))
	,m_twistAngleSpring(ndFloat32(1000.0f))
	,m_twistAngleDamper(ndFloat32(50.0f))
	,m_twistAngleRegularizer(ndFloat32(5.0e-3f))
	,m_maxConeAngle(ndFloat32(1.0e10f))
	,m_coneAngleSpring(ndFloat32(1000.0f))
	,m_coneAngleDamper(ndFloat32(50.0f))
	,m_coneAngleRegularizer(ndFloat32(5.0e-3f))
	,m_linearSpring(ndFloat32(1000.0f))
	,m_linearDamper(ndFloat32(50.0f))
	,m_linearRegularizer(ndFloat32(5.0e-3f))
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;

	m_pivotFrame = xmlGetMatrix(xmlNode, "pivotFrame");
	m_minTwistAngle = xmlGetFloat (xmlNode, "minTwistAngle");
	m_maxTwistAngle = xmlGetFloat (xmlNode, "maxTwistAngle");
	m_twistAngleSpring = xmlGetFloat(xmlNode, "twistAngleSpring");
	m_twistAngleDamper = xmlGetFloat(xmlNode, "twistAngleDamper");
	m_twistAngleRegularizer = xmlGetFloat(xmlNode, "twistAngleRegularizer");

	m_maxConeAngle = xmlGetFloat(xmlNode, "maxConeAngle");
	m_coneAngleSpring = xmlGetFloat (xmlNode, "coneAngleSpring");
	m_coneAngleDamper = xmlGetFloat (xmlNode, "coneAngleDamper");
	m_coneAngleRegularizer = xmlGetFloat (xmlNode, "coneAngleRegularizer");

	m_linearSpring = xmlGetFloat(xmlNode, "linearSpring");
	m_linearDamper = xmlGetFloat(xmlNode, "linearDamper");
	m_linearRegularizer = xmlGetFloat(xmlNode, "linearRegularizer");
}

ndJointPdBallAndSocket::~ndJointPdBallAndSocket()
{
}

void ndJointPdBallAndSocket::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "pivotFrame", m_pivotFrame);
	xmlSaveParam(childNode, "minTwistAngle", m_minTwistAngle);
	xmlSaveParam(childNode, "maxTwistAngle", m_maxTwistAngle);
	xmlSaveParam(childNode, "twistAngleSpring", m_twistAngleSpring);
	xmlSaveParam(childNode, "twistAngleDamper", m_twistAngleDamper);
	xmlSaveParam(childNode, "twistAngleRegularizer", m_twistAngleRegularizer);

	xmlSaveParam(childNode, "maxConeAngle", m_maxConeAngle);
	xmlSaveParam(childNode, "coneAngleSpring", m_coneAngleSpring);
	xmlSaveParam(childNode, "coneAngleDamper", m_coneAngleDamper);
	xmlSaveParam(childNode, "coneAngleRegularizer", m_coneAngleRegularizer);

	xmlSaveParam(childNode, "linearSpring", m_linearSpring);
	xmlSaveParam(childNode, "linearDamper", m_linearDamper);
	xmlSaveParam(childNode, "linearRegularizer", m_linearRegularizer);
}

void ndJointPdBallAndSocket::GetConeAngleSpringDamperRegularizer(ndFloat32& spring, ndFloat32& damper, ndFloat32& regularizer) const
{
	spring = m_coneAngleSpring;
	damper = m_coneAngleDamper;
	regularizer = m_coneAngleRegularizer;
}

void ndJointPdBallAndSocket::SetConeAngleSpringDamperRegularizer(ndFloat32 spring, ndFloat32 damper, ndFloat32 regularizer)
{
	m_coneAngleSpring = dMax(spring, ndFloat32(0.0f));
	m_coneAngleDamper = dMax(damper, ndFloat32(0.0f));
	m_coneAngleRegularizer = dMax(regularizer, ndFloat32(0.0f));
}

void ndJointPdBallAndSocket::GetTwistAngleSpringDamperRegularizer(ndFloat32& spring, ndFloat32& damper, ndFloat32& regularizer) const
{
	spring = m_twistAngleSpring;
	damper = m_twistAngleDamper;
	regularizer = m_twistAngleRegularizer;
}

void ndJointPdBallAndSocket::SetTwistAngleSpringDamperRegularizer(ndFloat32 spring, ndFloat32 damper, ndFloat32 regularizer)
{
	m_twistAngleSpring = dMax(spring, ndFloat32(0.0f));
	m_twistAngleDamper = dMax(damper, ndFloat32(0.0f));
	m_twistAngleRegularizer = dMax(regularizer, ndFloat32(0.0f));
}

void ndJointPdBallAndSocket::SetTwistLimits(ndFloat32 minAngle, ndFloat32 maxAngle)
{
	m_minTwistAngle = -dAbs(minAngle);
	m_maxTwistAngle = dAbs(maxAngle);
}

void ndJointPdBallAndSocket::GetTwistLimits(ndFloat32& minAngle, ndFloat32& maxAngle) const
{
	minAngle = m_minTwistAngle;
	maxAngle = m_maxTwistAngle;
}

ndFloat32 ndJointPdBallAndSocket::GetMaxConeAngle() const
{
	return m_maxConeAngle;
}

void ndJointPdBallAndSocket::SetConeLimit(ndFloat32 maxConeAngle)
{
	m_maxConeAngle = dMin (dAbs(maxConeAngle), D_PD_MAX_ANGLE * ndFloat32 (0.999f));
}

ndVector ndJointPdBallAndSocket::GetTargetPosition() const
{
	return m_localMatrix1.m_posit;
}

void ndJointPdBallAndSocket::SetTargetPosition(const ndVector& posit)
{
	dAssert(posit.m_w == ndFloat32(1.0f));
	m_localMatrix1.m_posit = posit;
}

void ndJointPdBallAndSocket::GetLinearSpringDamperRegularizer(ndFloat32& spring, ndFloat32& damper, ndFloat32& regularizer) const
{
	spring = m_linearSpring;
	damper = m_linearDamper;
	regularizer = m_linearRegularizer;
}

void ndJointPdBallAndSocket::SetLinearSpringDamperRegularizer(ndFloat32 spring, ndFloat32 damper, ndFloat32 regularizer)
{
	m_linearSpring = dMax(spring, ndFloat32(0.0f));
	m_linearDamper = dMax(damper, ndFloat32(0.0f));
	m_linearRegularizer = dMax(regularizer, ndFloat32(0.0f));
}

ndMatrix ndJointPdBallAndSocket::GetTargetMatrix() const
{
	return m_localMatrix1;
}

void ndJointPdBallAndSocket::SetTargetMatrix(const ndMatrix& matrix)
{
	m_localMatrix1 = matrix;
}

void ndJointPdBallAndSocket::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);
	matrix1 = m_pivotFrame * m_body1->GetMatrix();

	debugCallback.DrawFrame(matrix0);
	debugCallback.DrawFrame(matrix1);

	const ndInt32 subdiv = 8;
	const ndVector& coneDir0 = matrix0.m_front;
	const ndVector& coneDir1 = matrix1.m_front;
	ndFloat32 cosAngleCos = coneDir0.DotProduct(coneDir1).GetScalar();
	ndMatrix coneRotation(dGetIdentityMatrix());
	if (cosAngleCos < ndFloat32(0.9999f))
	{
		ndVector lateralDir(coneDir1.CrossProduct(coneDir0));
		ndFloat32 mag2 = lateralDir.DotProduct(lateralDir).GetScalar();
		if (mag2 > ndFloat32(1.0e-4f))
		{
			lateralDir = lateralDir.Scale(ndFloat32(1.0f) / ndSqrt(mag2));
			coneRotation = ndMatrix(ndQuaternion(lateralDir, ndAcos(dClamp(cosAngleCos, ndFloat32(-1.0f), ndFloat32(1.0f)))), matrix1.m_posit);
		}
		else
		{
			lateralDir = matrix0.m_up.Scale(-ndFloat32(1.0f));
			coneRotation = ndMatrix(ndQuaternion(matrix0.m_up, ndFloat32(180.0f) * ndDegreeToRad), matrix1.m_posit);
		}
	}
	else if (cosAngleCos < -ndFloat32(0.9999f))
	{
		coneRotation[0][0] = ndFloat32(-1.0f);
		coneRotation[1][1] = ndFloat32(-1.0f);
	}

	const ndFloat32 radius = debugCallback.m_debugScale;
	ndVector arch[subdiv + 1];

	// show twist angle limits
	ndFloat32 deltaTwist = m_maxTwistAngle - m_minTwistAngle;
	if ((deltaTwist > ndFloat32(1.0e-3f)) && (deltaTwist < ndFloat32(2.0f) * ndPi))
	{
		ndMatrix pitchMatrix(matrix1 * coneRotation);
		pitchMatrix.m_posit = matrix1.m_posit;

		ndVector point(ndFloat32(0.0f), ndFloat32(radius), ndFloat32(0.0f), ndFloat32(0.0f));

		ndFloat32 angleStep = dMin(m_maxTwistAngle - m_minTwistAngle, ndFloat32(2.0f * ndPi)) / subdiv;
		ndFloat32 angle0 = m_minTwistAngle;

		ndVector color(ndFloat32(0.4f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
		for (ndInt32 i = 0; i <= subdiv; i++)
		{
			arch[i] = pitchMatrix.TransformVector(dPitchMatrix(angle0).RotateVector(point));
			debugCallback.DrawLine(pitchMatrix.m_posit, arch[i], color);
			angle0 += angleStep;
		}

		for (ndInt32 i = 0; i < subdiv; i++)
		{
			debugCallback.DrawLine(arch[i], arch[i + 1], color);
		}
	}

	// show cone angle limits
	if ((m_maxConeAngle > ndFloat32(0.0f)) && (m_maxConeAngle < D_PD_MAX_ANGLE))
	{
		ndVector color(ndFloat32(0.3f), ndFloat32(0.8f), ndFloat32(0.0f), ndFloat32(0.0f));
		ndVector point(radius * ndCos(m_maxConeAngle), radius * ndSin(m_maxConeAngle), ndFloat32 (0.0f), ndFloat32(0.0f));
		ndFloat32 angleStep = ndPi * ndFloat32(2.0f) / subdiv;

		ndFloat32 angle0 = ndFloat32(0.0f);
		for (ndInt32 i = 0; i <= subdiv; i++)
		{
			ndVector conePoint(dPitchMatrix(angle0).RotateVector(point));
			ndVector p(matrix1.TransformVector(conePoint));
			arch[i] = p;
			debugCallback.DrawLine(matrix1.m_posit, p, color);
			angle0 += angleStep;
		}

		for (ndInt32 i = 0; i < subdiv; i++)
		{
			debugCallback.DrawLine(arch[i], arch[i + 1], color);
		}
	}
}

//void ndJointPdBallAndSocket::SubmitTwistLimits(const ndVector& pin, ndFloat32 angle, ndConstraintDescritor& desc)
//{
//	if((m_maxTwistAngle - m_minTwistAngle) < (2.0f * ndDegreeToRad))
//	{ 
//		AddAngularRowJacobian(desc, pin, -angle);
//		// force this limit to be bound
//		SetLowerFriction(desc, D_LCP_MAX_VALUE * ndFloat32 (0.1f));
//	}
//	else
//	{
//		if (angle < m_minTwistAngle)
//		{
//			AddAngularRowJacobian(desc, pin, ndFloat32(0.0f));
//			const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
//			const ndFloat32 penetration = angle - m_minTwistAngle;
//			const ndFloat32 recoveringAceel = -desc.m_invTimestep * D_PD_PENETRATION_RECOVERY_ANGULAR_SPEED * dMin(dAbs(penetration / D_PD_PENETRATION_ANGULAR_LIMIT), ndFloat32(1.0f));
//			SetMotorAcceleration(desc, stopAccel - recoveringAceel);
//			SetLowerFriction(desc, ndFloat32(0.0f));
//		}
//		else if (angle >= m_maxTwistAngle)
//		{
//			AddAngularRowJacobian(desc, pin, ndFloat32(0.0f));
//			const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
//			const ndFloat32 penetration = angle - m_maxTwistAngle;
//			const ndFloat32 recoveringAceel = desc.m_invTimestep * D_PD_PENETRATION_RECOVERY_ANGULAR_SPEED * dMin(dAbs(penetration / D_PD_PENETRATION_ANGULAR_LIMIT), ndFloat32(1.0f));
//			SetMotorAcceleration(desc, stopAccel - recoveringAceel);
//			SetHighFriction(desc, ndFloat32(0.0f));
//		}
//	}
//}

//void ndJointPdBallAndSocket::SubmitPdRotation(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
//{
//	ndQuaternion q0(matrix0);
//	ndQuaternion q1(matrix1);
//	if (q1.DotProduct(q0).GetScalar() < ndFloat32(0.0f))
//	{
//		q1 = q1.Scale (ndFloat32 (-1.0f));
//	}
//
//	ndQuaternion dq(q1.Inverse() * q0);
//	ndVector pin(dq.m_x, dq.m_y, dq.m_z, ndFloat32(0.0f));
//
//	ndFloat32 dirMag2 = pin.DotProduct(pin).GetScalar();
//	if (dirMag2 > ndFloat32(ndFloat32(1.0e-7f)))
//	{
//		ndFloat32 dirMag = ndSqrt(dirMag2);
//		pin = pin.Scale(ndFloat32(1.0f) / dirMag);
//		ndFloat32 angle = ndFloat32(2.0f) * ndAtan2(dirMag, dq.m_w);
//
//		ndMatrix basis(pin);
//		AddAngularRowJacobian(desc, basis[0], -angle);
//		SetMassSpringDamperAcceleration(desc, m_coneAngleRegularizer, m_coneAngleSpring, m_coneAngleDamper);
//
//		AddAngularRowJacobian(desc, basis[1], ndFloat32 (0.0f));
//		SetMassSpringDamperAcceleration(desc, m_coneAngleRegularizer, m_coneAngleSpring, m_coneAngleDamper);
//
//		AddAngularRowJacobian(desc, basis[2], ndFloat32(0.0f));
//		SetMassSpringDamperAcceleration(desc, m_coneAngleRegularizer, m_coneAngleSpring, m_coneAngleDamper);
//	}
//	else
//	{
//		ndFloat32 pitchAngle = CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);
//		AddAngularRowJacobian(desc, matrix1[0], pitchAngle);
//		SetMassSpringDamperAcceleration(desc, m_coneAngleRegularizer, m_coneAngleSpring, m_coneAngleDamper);
//		
//		ndFloat32 yawAngle = CalculateAngle(matrix0[0], matrix1[0], matrix1[1]);
//		AddAngularRowJacobian(desc, matrix1[1], yawAngle);
//		SetMassSpringDamperAcceleration(desc, m_coneAngleRegularizer, m_coneAngleSpring, m_coneAngleDamper);
//		
//		ndFloat32 rollAngle = CalculateAngle(matrix0[0], matrix1[0], matrix1[2]);
//		AddAngularRowJacobian(desc, matrix1[2], rollAngle);
//		SetMassSpringDamperAcceleration(desc, m_coneAngleRegularizer, m_coneAngleSpring, m_coneAngleDamper);
//	}
//}

void ndJointPdBallAndSocket::SubmitLinearLimits(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	if (m_linearRegularizer == ndFloat32(0.0f))
	{
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[0]);
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[1]);
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[2]);
	}
	else
	{
		// Cartesian motion
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[0]);
		SetMassSpringDamperAcceleration(desc, m_linearRegularizer, m_linearSpring, m_linearDamper);

		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[1]);
		SetMassSpringDamperAcceleration(desc, m_linearRegularizer, m_linearSpring, m_linearDamper);

		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[2]);
		SetMassSpringDamperAcceleration(desc, m_linearRegularizer, m_linearSpring, m_linearDamper);
	}
}

//void ndJointPdBallAndSocket::SubmitTwistAngleOnlyRows(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
void ndJointPdBallAndSocket::SubmitTwistAngleOnlyRows(const ndMatrix&, const ndMatrix&, ndConstraintDescritor&)
{
	dAssert(0);
}

void ndJointPdBallAndSocket::SubmitConeAngleOnlyRows(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	ndFloat32 cosAngleCos = matrix1.m_front.DotProduct(matrix0.m_front).GetScalar();
	if (cosAngleCos >= ndFloat32(0.998f))
	{
		ndFloat32 pitchAngle = CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);
		AddAngularRowJacobian(desc, matrix0.m_front, pitchAngle);

		ndFloat32 yawAngle = CalculateAngle(matrix0[0], matrix1[0], matrix1[1]);
		AddAngularRowJacobian(desc, matrix1[1], yawAngle);
		SetMassSpringDamperAcceleration(desc, m_coneAngleRegularizer, m_coneAngleSpring, m_coneAngleDamper);
		
		ndFloat32 rollAngle = CalculateAngle(matrix0[0], matrix1[0], matrix1[2]);
		AddAngularRowJacobian(desc, matrix1[2], rollAngle);
		SetMassSpringDamperAcceleration(desc, m_coneAngleRegularizer, m_coneAngleSpring, m_coneAngleDamper);
	}
	else
	{
		const ndVector dir0(matrix0.m_front);
		const ndVector dir1(matrix1.m_front);
		const ndVector lateralDir(dir1.CrossProduct(dir0).Normalize());
		ndFloat32 coneAngle = ndAcos(dClamp(cosAngleCos, ndFloat32(-1.0f), ndFloat32(1.0f)));
		const ndQuaternion rotation(lateralDir, coneAngle);
		const ndMatrix coneMatrix(rotation, ndVector::m_zero);
		const ndMatrix pitchMatrix = matrix1 * coneMatrix * matrix0.Inverse();
		ndFloat32 twistAngle = ndAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);

		AddAngularRowJacobian(desc, dir0, twistAngle);
		if (dAbs(twistAngle) > 3.0f * ndDegreeToRad)
		{
			const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
			const ndFloat32 penetration = twistAngle * ndFloat32 (0.125f);
			const ndFloat32 recoveringAceel = desc.m_invTimestep * desc.m_invTimestep * penetration;
			SetMotorAcceleration(desc, stopAccel + recoveringAceel);
		}

		ndVector sideDir(lateralDir.CrossProduct(dir0));
		AddAngularRowJacobian(desc, sideDir, ndFloat32(0.0f));
		SetMassSpringDamperAcceleration(desc, m_coneAngleRegularizer, m_coneAngleSpring, m_coneAngleDamper);

		AddAngularRowJacobian(desc, lateralDir, -coneAngle);
		SetMassSpringDamperAcceleration(desc, m_coneAngleRegularizer, m_coneAngleSpring, m_coneAngleDamper);
		if (coneAngle > m_maxConeAngle)
		{
			dAssert(0);
		}
	}
}

void ndJointPdBallAndSocket::SubmitAngularAxis(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	const ndVector dir0(matrix0.m_front);
	const ndVector dir1(matrix1.m_front);
	const ndVector lateralDir(dir1.CrossProduct(dir0).Normalize());
	dAssert(lateralDir.DotProduct(lateralDir).GetScalar() > 1.0e-6f);
	ndFloat32 cosAngleCos = dir1.DotProduct(dir0).GetScalar();
	ndFloat32 coneAngle = ndAcos(dClamp(cosAngleCos, ndFloat32(-1.0f), ndFloat32(1.0f)));
	const ndQuaternion rotation(lateralDir, coneAngle);
	const ndMatrix coneMatrix(rotation, ndVector::m_zero);
	const ndMatrix pitchMatrix = matrix1 * coneMatrix * matrix0.Inverse();
	ndFloat32 twistAngle = ndAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);

	AddAngularRowJacobian(desc, dir0, twistAngle);
	SetMassSpringDamperAcceleration(desc, m_twistAngleRegularizer, m_twistAngleSpring, m_twistAngleDamper);

	ndVector sideDir(lateralDir.CrossProduct(dir0));
	AddAngularRowJacobian(desc, sideDir, ndFloat32(0.0f));
	SetMassSpringDamperAcceleration(desc, m_coneAngleRegularizer, m_coneAngleSpring, m_coneAngleDamper);

	AddAngularRowJacobian(desc, lateralDir, -coneAngle);
	SetMassSpringDamperAcceleration(desc, m_coneAngleRegularizer, m_coneAngleSpring, m_coneAngleDamper);

	if (coneAngle > m_maxConeAngle)
	{
		dAssert(0);
	//	AddAngularRowJacobian(desc, lateralDir, 0.0f);
	//	const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
	//	const ndFloat32 penetration = coneAngle - m_maxConeAngle;
	//	const ndFloat32 recoveringAceel = desc.m_invTimestep * D_PD_PENETRATION_RECOVERY_ANGULAR_SPEED * dMin(dAbs(penetration / D_PD_PENETRATION_ANGULAR_LIMIT), ndFloat32(1.0f));
	//	SetMotorAcceleration(desc, stopAccel - recoveringAceel);
	//	SetHighFriction(desc, ndFloat32(0.0f));
	}

	if (twistAngle < m_minTwistAngle)
	{
		dAssert(0);
		//AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));
		//const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
		//const ndFloat32 penetration = twistAngle - m_minTwistAngle;
		//const ndFloat32 recoveringAceel = -desc.m_invTimestep * D_PD_PENETRATION_RECOVERY_ANGULAR_SPEED * dMin(dAbs(penetration / D_PD_PENETRATION_ANGULAR_LIMIT), ndFloat32(1.0f));
		//SetMotorAcceleration(desc, stopAccel - recoveringAceel);
		//SetLowerFriction(desc, ndFloat32(0.0f));
	}
	else if (twistAngle >= m_maxTwistAngle)
	{
		dAssert(0);
		//AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));
		//const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
		//const ndFloat32 penetration = twistAngle - m_maxTwistAngle;
		//const ndFloat32 recoveringAceel = desc.m_invTimestep * D_PD_PENETRATION_RECOVERY_ANGULAR_SPEED * dMin(dAbs(penetration / D_PD_PENETRATION_ANGULAR_LIMIT), ndFloat32(1.0f));
		//SetMotorAcceleration(desc, stopAccel - recoveringAceel);
		//SetHighFriction(desc, ndFloat32(0.0f));
	}
}

void ndJointPdBallAndSocket::SubmitAngularAxisCartesianApproximation(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	ndFloat32 twistAngle = CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);
	AddAngularRowJacobian(desc, matrix0.m_front, twistAngle);
	SetMassSpringDamperAcceleration(desc, m_twistAngleRegularizer, m_twistAngleSpring, m_twistAngleDamper);

	ndFloat32 yawAngle = CalculateAngle(matrix0[0], matrix1[0], matrix1[1]);
	AddAngularRowJacobian(desc, matrix1[1], yawAngle);
	SetMassSpringDamperAcceleration(desc, m_coneAngleRegularizer, m_coneAngleSpring, m_coneAngleDamper);

	ndFloat32 rollAngle = CalculateAngle(matrix0[0], matrix1[0], matrix1[2]);
	AddAngularRowJacobian(desc, matrix1[2], rollAngle);
	SetMassSpringDamperAcceleration(desc, m_coneAngleRegularizer, m_coneAngleSpring, m_coneAngleDamper);

	if (twistAngle < m_minTwistAngle)
	{
		dAssert(0);
		AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));
		const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
		const ndFloat32 penetration = twistAngle - m_minTwistAngle;
		const ndFloat32 recoveringAceel = -desc.m_invTimestep * D_PD_PENETRATION_RECOVERY_ANGULAR_SPEED * dMin(dAbs(penetration / D_PD_PENETRATION_ANGULAR_LIMIT), ndFloat32(1.0f));
		SetMotorAcceleration(desc, stopAccel - recoveringAceel);
		SetLowerFriction(desc, ndFloat32(0.0f));
	}
	else if (twistAngle >= m_maxTwistAngle)
	{
		dAssert(0);
		AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));
		const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
		const ndFloat32 penetration = twistAngle - m_maxTwistAngle;
		const ndFloat32 recoveringAceel = desc.m_invTimestep * D_PD_PENETRATION_RECOVERY_ANGULAR_SPEED * dMin(dAbs(penetration / D_PD_PENETRATION_ANGULAR_LIMIT), ndFloat32(1.0f));
		SetMotorAcceleration(desc, stopAccel - recoveringAceel);
		SetHighFriction(desc, ndFloat32(0.0f));
	}
}

void ndJointPdBallAndSocket::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;

	CalculateGlobalMatrix(matrix0, matrix1);
	SubmitLinearLimits(matrix0, matrix1, desc);

	if (m_twistAngleRegularizer == ndFloat32(0.0f))
	{
		// handle special case that this is a cone angle only joint (2dof)
		SubmitConeAngleOnlyRows(matrix0, matrix1, desc);
	}
	else if (m_coneAngleRegularizer == ndFloat32(0.0f))
	{
		// handle special case that this is a twist angle only joint (a hinge)
		SubmitTwistAngleOnlyRows(matrix0, matrix1, desc);
	}
	else
	{
		ndFloat32 cosAngleCos = matrix1.m_front.DotProduct(matrix0.m_front).GetScalar();
		if (cosAngleCos >= ndFloat32(0.998f))
		{
			// special case where the front axis are almost aligned
			// solve by using Cartesian approximation
			SubmitAngularAxisCartesianApproximation(matrix0, matrix1, desc);
		}
		else
		{
			SubmitAngularAxis(matrix0, matrix1, desc);
		}
	}
}