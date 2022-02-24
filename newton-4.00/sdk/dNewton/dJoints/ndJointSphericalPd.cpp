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
#include "ndJointSphericalPd.h"

#define D_PD_MAX_ANGLE	ndFloat32 (120.0f * ndDegreeToRad)
#define D_PD_PENETRATION_RECOVERY_ANGULAR_SPEED ndFloat32 (0.1f) 
#define D_PD_PENETRATION_ANGULAR_LIMIT ndFloat32 (10.0f * ndDegreeToRad) 

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointSphericalPd)

ndJointSphericalPd::ndJointSphericalPd(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointSpherical(pinAndPivotFrame, child, parent)
	,m_pivotFrame(m_localMatrix1)
	,m_springConst(ndFloat32(0.0))
{
	m_maxDof = 8;
}

ndJointSphericalPd::ndJointSphericalPd(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointSpherical(ndLoadSaveBase::ndLoadDescriptor(desc))
	,m_pivotFrame(m_localMatrix1)
	,m_springConst(ndFloat32(0.0))
{
	dAssert(0);
	m_maxDof = 8;
	//const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	//
	//m_pivotFrame = xmlGetMatrix(xmlNode, "pivotFrame");
	//m_minTwistAngle = xmlGetFloat (xmlNode, "minTwistAngle");
	//m_maxTwistAngle = xmlGetFloat (xmlNode, "maxTwistAngle");
	//m_twistAngleSpring = xmlGetFloat(xmlNode, "twistAngleSpring");
	//m_twistAngleDamper = xmlGetFloat(xmlNode, "twistAngleDamper");
	//m_twistAngleRegularizer = xmlGetFloat(xmlNode, "twistAngleRegularizer");
	//
	//m_maxConeAngle = xmlGetFloat(xmlNode, "maxConeAngle");
	//m_coneAngleSpring = xmlGetFloat (xmlNode, "coneAngleSpring");
	//m_coneAngleDamper = xmlGetFloat (xmlNode, "coneAngleDamper");
	//m_coneAngleRegularizer = xmlGetFloat (xmlNode, "coneAngleRegularizer");
}

ndJointSphericalPd::~ndJointSphericalPd()
{
}

void ndJointSphericalPd::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	//xmlSaveParam(childNode, "pivotFrame", m_pivotFrame);
	//xmlSaveParam(childNode, "minTwistAngle", m_minTwistAngle);
	//xmlSaveParam(childNode, "maxTwistAngle", m_maxTwistAngle);
	//xmlSaveParam(childNode, "twistAngleSpring", m_twistAngleSpring);
	//xmlSaveParam(childNode, "twistAngleDamper", m_twistAngleDamper);
	//xmlSaveParam(childNode, "twistAngleRegularizer", m_twistAngleRegularizer);
	//
	//xmlSaveParam(childNode, "maxConeAngle", m_maxConeAngle);
	//xmlSaveParam(childNode, "coneAngleSpring", m_coneAngleSpring);
	//xmlSaveParam(childNode, "coneAngleDamper", m_coneAngleDamper);
	//xmlSaveParam(childNode, "coneAngleRegularizer", m_coneAngleRegularizer);
}

ndMatrix ndJointSphericalPd::GetTargetMatrix() const
{
	return m_pivotFrame;
}

void ndJointSphericalPd::SetTargetMatrix(const ndMatrix& matrix)
{
	m_pivotFrame = matrix;
}

void ndJointSphericalPd::SetSpringDamper(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper)
{
	m_springConst = spring;
	SetViscousFriction(regularizer, damper);
}

void ndJointSphericalPd::GetSpringDamper(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const
{
	spring = m_springConst;
	damper = m_viscousFriction;
	regularizer = m_viscousFrictionRegularizer;
}

void ndJointSphericalPd::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	ndJointSpherical::DebugJoint(debugCallback);
}

//void ndJointSphericalPd::SubmitTwistLimits(const ndVector& pin, ndFloat32 angle, ndConstraintDescritor& desc)
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

//void ndJointSphericalPd::SubmitPdRotation(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
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

//
////void ndJointSphericalPd::SubmitTwistAngleOnlyRows(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
//void ndJointSphericalPd::SubmitTwistAngleOnlyRows(const ndMatrix&, const ndMatrix&, ndConstraintDescritor&)
//{
//	dAssert(0);
//}
//
//void ndJointSphericalPd::SubmitConeAngleOnlyRows(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
//{
//	ndFloat32 cosAngleCos = matrix1.m_front.DotProduct(matrix0.m_front).GetScalar();
//	if (cosAngleCos >= ndFloat32(0.998f))
//	{
//		ndFloat32 pitchAngle = CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);
//		AddAngularRowJacobian(desc, matrix0.m_front, pitchAngle);
//
//		ndFloat32 yawAngle = CalculateAngle(matrix0[0], matrix1[0], matrix1[1]);
//		AddAngularRowJacobian(desc, matrix1[1], yawAngle);
//		SetMassSpringDamperAcceleration(desc, m_coneAngleRegularizer, m_coneAngleSpring, m_coneAngleDamper);
//		
//		ndFloat32 rollAngle = CalculateAngle(matrix0[0], matrix1[0], matrix1[2]);
//		AddAngularRowJacobian(desc, matrix1[2], rollAngle);
//		SetMassSpringDamperAcceleration(desc, m_coneAngleRegularizer, m_coneAngleSpring, m_coneAngleDamper);
//	}
//	else
//	{
//		const ndVector dir0(matrix0.m_front);
//		const ndVector dir1(matrix1.m_front);
//		const ndVector lateralDir(dir1.CrossProduct(dir0).Normalize());
//		ndFloat32 coneAngle = ndAcos(dClamp(cosAngleCos, ndFloat32(-1.0f), ndFloat32(1.0f)));
//		const ndQuaternion rotation(lateralDir, coneAngle);
//		const ndMatrix coneMatrix(rotation, ndVector::m_zero);
//		const ndMatrix pitchMatrix = matrix1 * coneMatrix * matrix0.Inverse();
//		ndFloat32 twistAngle = ndAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
//
//		AddAngularRowJacobian(desc, dir0, twistAngle);
//		if (dAbs(twistAngle) > 3.0f * ndDegreeToRad)
//		{
//			const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
//			const ndFloat32 penetration = twistAngle * ndFloat32 (0.125f);
//			const ndFloat32 recoveringAceel = desc.m_invTimestep * desc.m_invTimestep * penetration;
//			SetMotorAcceleration(desc, stopAccel + recoveringAceel);
//		}
//
//		ndVector sideDir(lateralDir.CrossProduct(dir0));
//		AddAngularRowJacobian(desc, sideDir, ndFloat32(0.0f));
//		SetMassSpringDamperAcceleration(desc, m_coneAngleRegularizer, m_coneAngleSpring, m_coneAngleDamper);
//
//		AddAngularRowJacobian(desc, lateralDir, -coneAngle);
//		SetMassSpringDamperAcceleration(desc, m_coneAngleRegularizer, m_coneAngleSpring, m_coneAngleDamper);
//		if (coneAngle > m_maxConeAngle)
//		{
//			dAssert(0);
//		}
//	}
//}

//
//void ndJointSphericalPd::SubmitAngularAxisCartesianApproximation(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
//{
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
//
//	if (twistAngle < m_minTwistAngle)
//	{
//		dAssert(0);
//		AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));
//		const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
//		const ndFloat32 penetration = twistAngle - m_minTwistAngle;
//		const ndFloat32 recoveringAceel = -desc.m_invTimestep * D_PD_PENETRATION_RECOVERY_ANGULAR_SPEED * dMin(dAbs(penetration / D_PD_PENETRATION_ANGULAR_LIMIT), ndFloat32(1.0f));
//		SetMotorAcceleration(desc, stopAccel - recoveringAceel);
//		SetLowerFriction(desc, ndFloat32(0.0f));
//	}
//	else if (twistAngle >= m_maxTwistAngle)
//	{
//		dAssert(0);
//		AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));
//		const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
//		const ndFloat32 penetration = twistAngle - m_maxTwistAngle;
//		const ndFloat32 recoveringAceel = desc.m_invTimestep * D_PD_PENETRATION_RECOVERY_ANGULAR_SPEED * dMin(dAbs(penetration / D_PD_PENETRATION_ANGULAR_LIMIT), ndFloat32(1.0f));
//		SetMotorAcceleration(desc, stopAccel - recoveringAceel);
//		SetHighFriction(desc, ndFloat32(0.0f));
//	}
//}


void ndJointSphericalPd::SubmitTwistAngle(const ndVector& pin, ndFloat32 angle, ndConstraintDescritor& desc)
{
	if ((m_maxTwistAngle - m_minTwistAngle) < (2.0f * ndDegreeToRad))
	{
		dAssert(desc.m_rowsCount < 8);
		AddAngularRowJacobian(desc, pin, -angle);
		SetLowerFriction(desc, -D_LCP_MAX_VALUE * ndFloat32(0.1f));
		SetHighFriction(desc, D_LCP_MAX_VALUE * ndFloat32(0.1f));
	}
	else
	{
		if (angle < m_minTwistAngle)
		{
			AddAngularRowJacobian(desc, pin, ndFloat32(0.0f));
			const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
			const ndFloat32 penetration = angle - m_minTwistAngle;
			const ndFloat32 recoveringAceel = -desc.m_invTimestep * D_BALL_AND_SOCKED_PENETRATION_RECOVERY_SPEED * dMin(dAbs(penetration / D_BALL_AND_SOCKED_PENETRATION_LIMIT), ndFloat32(1.0f));
			SetMotorAcceleration(desc, stopAccel - recoveringAceel);
			SetLowerFriction(desc, ndFloat32(0.0f));
		}
		else if (angle >= m_maxTwistAngle)
		{
			AddAngularRowJacobian(desc, pin, ndFloat32(0.0f));
			const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
			const ndFloat32 penetration = angle - m_maxTwistAngle;
			const ndFloat32 recoveringAceel = desc.m_invTimestep * D_BALL_AND_SOCKED_PENETRATION_RECOVERY_SPEED * dMin(dAbs(penetration / D_BALL_AND_SOCKED_PENETRATION_LIMIT), ndFloat32(1.0f));
			SetMotorAcceleration(desc, stopAccel - recoveringAceel);
			SetHighFriction(desc, ndFloat32(0.0f));
		}
	}
}

void ndJointSphericalPd::SubmitAngularAxis(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	ndVector lateralDir(matrix1[0].CrossProduct(matrix0[0]));
	dAssert(lateralDir.DotProduct(lateralDir).GetScalar() > 1.0e-6f);
	lateralDir = lateralDir.Normalize();
	const ndFloat32 coneAngle = ndAcos(dClamp(matrix1.m_front.DotProduct(matrix0.m_front).GetScalar(), ndFloat32(-1.0f), ndFloat32(1.0f)));
	const ndMatrix coneRotation(ndQuaternion(lateralDir, coneAngle), matrix1.m_posit);
	const ndVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
	if (coneAngle > m_maxConeAngle)
	{
		AddAngularRowJacobian(desc, lateralDir, ndFloat32(0.0f));
		const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
		const ndFloat32 penetration = coneAngle - m_maxConeAngle;
		const ndFloat32 recoveringAceel = desc.m_invTimestep * D_BALL_AND_SOCKED_PENETRATION_RECOVERY_SPEED * dMin(dAbs(penetration / D_BALL_AND_SOCKED_PENETRATION_LIMIT), ndFloat32(1.0f));
		SetMotorAcceleration(desc, stopAccel - recoveringAceel);
		SetHighFriction(desc, ndFloat32(0.0f));

		AddAngularRowJacobian(desc, sideDir, ndFloat32(0.0f));
		SetLowerFriction(desc, -D_LCP_MAX_VALUE * ndFloat32(0.1f));
		SetHighFriction(desc, D_LCP_MAX_VALUE * ndFloat32(0.1f));
	}

	const ndMatrix pitchMatrix(matrix1 * coneRotation * matrix0.Inverse());
	const ndFloat32 pitchAngle = -ndAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
	SubmitTwistAngle(matrix0.m_front, pitchAngle, desc);
}

void ndJointSphericalPd::SubmitAngularAxisCartesianApproximation(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	//ndFloat32 coneAngle = ndAcos(dClamp(matrix1.m_front.DotProduct(matrix0.m_front).GetScalar(), ndFloat32(-1.0f), ndFloat32(1.0f)));
	//if (coneAngle > m_maxConeAngle)
	if (m_maxConeAngle < (ndFloat32(1.0f) * ndDegreeToRad))
	{
		// two rows to restrict rotation around around the parent coordinate system
		ndFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
		AddAngularRowJacobian(desc, matrix1.m_up, angle0);
		SetLowerFriction(desc, -D_LCP_MAX_VALUE * ndFloat32(0.1f));
		SetHighFriction(desc, D_LCP_MAX_VALUE * ndFloat32(0.1f));
		
		ndFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
		AddAngularRowJacobian(desc, matrix1.m_right, angle1);
		SetLowerFriction(desc, -D_LCP_MAX_VALUE * ndFloat32 (0.1f));
		SetHighFriction(desc, D_LCP_MAX_VALUE * ndFloat32(0.1f));
	}

	ndFloat32 pitchAngle = -CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);
	SubmitTwistAngle(matrix0.m_front, pitchAngle, desc);
}

void ndJointSphericalPd::JacobianDerivative(ndConstraintDescritor& desc)
{
ndJointSpherical::JacobianDerivative(desc);
return;

	ndMatrix matrix0;
	ndMatrix matrix1;
	
	CalculateGlobalMatrix(matrix0, matrix1);
	ApplyBaseRows(matrix0, matrix1, desc);

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