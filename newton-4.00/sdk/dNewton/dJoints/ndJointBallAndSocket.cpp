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
#include "ndJointBallAndSocket.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointBallAndSocket)

ndJointBallAndSocket::ndJointBallAndSocket(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(6, child, parent, pinAndPivotFrame)
	,m_maxConeAngle(dFloat32 (1.0e10f))
	,m_coneFriction(dFloat32(0.0))
	,m_minTwistAngle(-dFloat32(1.0e10f))
	,m_maxTwistAngle( dFloat32(1.0e10f))
	,m_twistFriction(dFloat32 (0.0f))
	,m_coneFrictionRegularizer(dFloat32(0.0f))
	,m_twistFrictionRegularizer(dFloat32(0.0f))
{
}

ndJointBallAndSocket::ndJointBallAndSocket(const dLoadSaveBase::dLoadDescriptor& desc)
	:ndJointBilateralConstraint(dLoadSaveBase::dLoadDescriptor(desc))
	,m_maxConeAngle(dFloat32(1.0e10f))
	,m_coneFriction(dFloat32(0.0))
	,m_minTwistAngle(-dFloat32(1.0e10f))
	,m_maxTwistAngle(dFloat32(1.0e10f))
	,m_twistFriction(dFloat32(0.0f))
	,m_coneFrictionRegularizer(dFloat32(0.0f))
	,m_twistFrictionRegularizer(dFloat32(0.0f))
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;

	m_maxConeAngle = xmlGetFloat(xmlNode, "maxConeAngle");
	m_coneFriction = xmlGetFloat(xmlNode, "coneFriction");
	m_minTwistAngle = xmlGetFloat(xmlNode, "minTwistAngle");
	m_maxTwistAngle = xmlGetFloat(xmlNode, "maxTwistAngle");
	m_twistFriction = xmlGetFloat(xmlNode, "twistFriction");
	m_coneFrictionRegularizer = xmlGetFloat(xmlNode, "coneFrictionRegularizer");
	m_twistFrictionRegularizer = xmlGetFloat(xmlNode, "twistFrictionRegularizer");
}

ndJointBallAndSocket::~ndJointBallAndSocket()
{
}

void ndJointBallAndSocket::Save(const dLoadSaveBase::dSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(dLoadSaveBase::dSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "maxConeAngle", m_maxConeAngle);
	xmlSaveParam(childNode, "coneFriction", m_coneFriction);
	xmlSaveParam(childNode, "minTwistAngle", m_minTwistAngle);
	xmlSaveParam(childNode, "maxTwistAngle", m_maxTwistAngle);
	xmlSaveParam(childNode, "twistFriction", m_twistFriction);
	xmlSaveParam(childNode, "coneFrictionRegularizer", m_coneFrictionRegularizer);
	xmlSaveParam(childNode, "twistFrictionRegularizer", m_twistFrictionRegularizer);
}

void ndJointBallAndSocket::SetConeFriction(dFloat32 regularizer, dFloat32 viscousFriction)
{
	m_coneFriction = dAbs(viscousFriction);
	m_coneFrictionRegularizer = dMax(dAbs(regularizer), dFloat32(0.0f));
}

void ndJointBallAndSocket::SetTwistLimits(dFloat32 minAngle, dFloat32 maxAngle)
{
	m_minTwistAngle = dMin(minAngle, dFloat32 (0.0f));
	m_maxTwistAngle = dMax(maxAngle, dFloat32(0.0f));
}

void ndJointBallAndSocket::SetTwistFriction(dFloat32 regularizer, dFloat32 viscousFriction)
{
	m_twistFriction = dAbs(viscousFriction);
	m_twistFrictionRegularizer = dMax(dAbs(regularizer), dFloat32(0.0f));
}

void ndJointBallAndSocket::GetTwistLimits(dFloat32& minAngle, dFloat32& maxAngle) const
{
	minAngle = m_minTwistAngle;
	maxAngle = m_maxTwistAngle;
}

dFloat32 ndJointBallAndSocket::GetMaxConeAngle() const
{
	return m_maxConeAngle;
}

void ndJointBallAndSocket::SetConeLimit(dFloat32 maxConeAngle)
{
	//m_maxConeAngle = dMin (dAbs (maxConeAngle), D_BALL_AND_SOCKED_MAX_ANGLE * dFloat32 (0.999f));
	m_maxConeAngle = maxConeAngle;
}

void ndJointBallAndSocket::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	debugCallback.DrawFrame(matrix0);
	debugCallback.DrawFrame(matrix1);

	const dInt32 subdiv = 8;
	const dVector& coneDir0 = matrix0.m_front;
	const dVector& coneDir1 = matrix1.m_front;
	dFloat32 cosAngleCos = coneDir0.DotProduct(coneDir1).GetScalar();
	dMatrix coneRotation(dGetIdentityMatrix());
	if (cosAngleCos < dFloat32(0.9999f))
	{
		dVector lateralDir(coneDir1.CrossProduct(coneDir0));
		dFloat32 mag2 = lateralDir.DotProduct(lateralDir).GetScalar();
		if (mag2 > dFloat32 (1.0e-4f)) 
		{
			lateralDir = lateralDir.Scale(dFloat32 (1.0f) / dSqrt(mag2));
			coneRotation = dMatrix(dQuaternion(lateralDir, dAcos(dClamp(cosAngleCos, dFloat32(-1.0f), dFloat32(1.0f)))), matrix1.m_posit);
		}
		else 
		{
			lateralDir = matrix0.m_up.Scale(-dFloat32 (1.0f));
			coneRotation = dMatrix(dQuaternion(matrix0.m_up, dFloat32 (180.0f) * dDegreeToRad), matrix1.m_posit);
		}
	}
	else if (cosAngleCos < -dFloat32 (0.9999f)) 
	{
		coneRotation[0][0] = dFloat32(-1.0f);
		coneRotation[1][1] = dFloat32(-1.0f);
	}
	
	const dFloat32 radius = debugCallback.m_debugScale;
	dVector arch[subdiv + 1];
	
	// show twist angle limits
	dFloat32 deltaTwist = m_maxTwistAngle - m_minTwistAngle;
	if ((deltaTwist > dFloat32(1.0e-3f)) && (deltaTwist < dFloat32 (2.0f) * dPi))
	{ 
		dMatrix pitchMatrix(matrix1 * coneRotation);
		pitchMatrix.m_posit = matrix1.m_posit;
	
		dVector point(dFloat32(0.0f), dFloat32(radius), dFloat32(0.0f), dFloat32(0.0f));
	
		dFloat32 angleStep = dMin(m_maxTwistAngle - m_minTwistAngle, dFloat32(2.0f * dPi)) / subdiv;
		dFloat32 angle0 = m_minTwistAngle;
	
		dVector color(dFloat32 (0.4f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
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
	
	// show cone angle limits
	if ((m_maxConeAngle > dFloat32 (0.0f)) && (m_maxConeAngle < D_BALL_AND_SOCKED_MAX_ANGLE)) 
	{
		dVector color(dFloat32(0.3f), dFloat32(0.8f), dFloat32(0.0f), dFloat32(0.0f));
		dVector point(radius * dCos(m_maxConeAngle), radius * dSin(m_maxConeAngle), dFloat32 (0.0f), dFloat32(0.0f));
		dFloat32 angleStep = dPi * dFloat32(2.0f) / subdiv;
	
		dFloat32 angle0 = dFloat32 (0.0f);
		for (dInt32 i = 0; i <= subdiv; i++) 
		{
			dVector conePoint(dPitchMatrix(angle0).RotateVector(point));
			dVector p(matrix1.TransformVector(conePoint));
			arch[i] = p;
			debugCallback.DrawLine(matrix1.m_posit, p, color);
			angle0 += angleStep;
		}
	
		for (dInt32 i = 0; i < subdiv; i++) 
		{
			debugCallback.DrawLine(arch[i], arch[i + 1], color);
		}
	}
}

void ndJointBallAndSocket::SubmitAngularAxisCartesianApproximation(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc)
{
	dFloat32 coneAngle = dAcos(dClamp(matrix1.m_front.DotProduct(matrix0.m_front).GetScalar(), dFloat32(-1.0f), dFloat32(1.0f)));
	if (coneAngle > m_maxConeAngle)
	{
		// two rows to restrict rotation around around the parent coordinate system
		dFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
		AddAngularRowJacobian(desc, matrix1.m_up, angle0);

		dFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
		AddAngularRowJacobian(desc, matrix1.m_right, angle1);
	}

	dFloat32 pitchAngle = -CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);
	SubmitTwistAngle(matrix0.m_front, pitchAngle, desc);
}

void ndJointBallAndSocket::SubmitTwistAngle(const dVector& pin, dFloat32 angle, ndConstraintDescritor& desc)
{
	if ((m_maxTwistAngle - m_minTwistAngle) < (2.0f * dDegreeToRad)) 
	{
		dAssert(desc.m_rowsCount < 6);
		AddAngularRowJacobian(desc, pin, -angle);
	}
	else 
	{
		if (angle < m_minTwistAngle) 
		{
			AddAngularRowJacobian(desc, pin, dFloat32(0.0f));
			const dFloat32 stopAccel = GetMotorZeroAcceleration(desc);
			const dFloat32 penetration = angle - m_minTwistAngle;
			const dFloat32 recoveringAceel = -desc.m_invTimestep * D_BALL_AND_SOCKED_PENETRATION_RECOVERY_SPEED * dMin(dAbs(penetration / D_BALL_AND_SOCKED_PENETRATION_LIMIT), dFloat32(1.0f));
			SetMotorAcceleration(desc, stopAccel - recoveringAceel);
			SetLowerFriction(desc, dFloat32 (0.0f));
		}
		else if (angle >= m_maxTwistAngle) 
		{
			AddAngularRowJacobian(desc, pin, dFloat32(0.0f));
			const dFloat32 stopAccel = GetMotorZeroAcceleration(desc);
			const dFloat32 penetration = angle - m_maxTwistAngle;
			const dFloat32 recoveringAceel = desc.m_invTimestep * D_BALL_AND_SOCKED_PENETRATION_RECOVERY_SPEED * dMin(dAbs(penetration / D_BALL_AND_SOCKED_PENETRATION_LIMIT), dFloat32(1.0f));
			SetMotorAcceleration(desc, stopAccel - recoveringAceel);
			SetHighFriction(desc, dFloat32 (0.0f));
		}
		else if (m_twistFriction > dFloat32(0.0f))
		{
			AddAngularRowJacobian(desc, pin, dFloat32 (0.0f));
			SetMassSpringDamperAcceleration(desc, m_twistFrictionRegularizer, dFloat32 (0.0f), m_twistFriction);
		}
	}
}

void ndJointBallAndSocket::SubmitAngularAxis(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc)
{
	dVector lateralDir(matrix1[0].CrossProduct(matrix0[0]));
	dAssert(lateralDir.DotProduct(lateralDir).GetScalar() > 1.0e-6f);
	lateralDir = lateralDir.Normalize();
	const dFloat32 coneAngle = dAcos(dClamp(matrix1.m_front.DotProduct(matrix0.m_front).GetScalar(), dFloat32(-1.0f), dFloat32(1.0f)));
	const dMatrix coneRotation(dQuaternion(lateralDir, coneAngle), matrix1.m_posit);
	const dVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
	if (coneAngle > m_maxConeAngle)
	{ 
		AddAngularRowJacobian(desc, lateralDir, dFloat32 (0.0f));
		const dFloat32 stopAccel = GetMotorZeroAcceleration(desc);
		const dFloat32 penetration = coneAngle - m_maxConeAngle;
		const dFloat32 recoveringAceel = desc.m_invTimestep * D_BALL_AND_SOCKED_PENETRATION_RECOVERY_SPEED * dMin(dAbs(penetration / D_BALL_AND_SOCKED_PENETRATION_LIMIT), dFloat32(1.0f));
		SetMotorAcceleration(desc, stopAccel - recoveringAceel);
		SetHighFriction(desc, dFloat32(0.0f));
		
		AddAngularRowJacobian(desc, sideDir, dFloat32 (0.0f));
	}
	else if (m_coneFriction > dFloat32 (0.0f))
	{
		AddAngularRowJacobian(desc, lateralDir, dFloat32 (0.0f));
		SetMassSpringDamperAcceleration(desc, m_coneFrictionRegularizer, dFloat32(0.0f), m_coneFriction);

		AddAngularRowJacobian(desc, sideDir, dFloat32 (0.0f));
		SetMassSpringDamperAcceleration(desc, m_coneFrictionRegularizer, dFloat32(0.0f), m_coneFriction);
	}

	const dMatrix pitchMatrix(matrix1 * coneRotation * matrix0.Inverse());
	const dFloat32 pitchAngle = -dAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
	SubmitTwistAngle(matrix0.m_front, pitchAngle, desc);
}

void ndJointBallAndSocket::JacobianDerivative(ndConstraintDescritor& desc)
{
//if (m_body0->GetId() != 5)
//return ;

	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[0]);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[1]);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[2]);

	dFloat32 deltaTwist = m_maxTwistAngle - m_minTwistAngle;
	if (deltaTwist < (2.0f * dDegreeToRad))
	{
		SubmitConeAngleOnlyRows(matrix0, matrix1, desc);
	}
	else
	{
		//dAssert(0);
	}

	//bool hasAngleRows = deltaTwist > dFloat32(1.0e-3f);
	//hasAngleRows = hasAngleRows && (deltaTwist < dFloat32(2.0f) * dPi);
	//hasAngleRows = hasAngleRows || (m_maxConeAngle < D_BALL_AND_SOCKED_MAX_ANGLE);
	//if (hasAngleRows)
	//{
	//	dFloat32 cosAngleCos = matrix1.m_front.DotProduct(matrix0.m_front).GetScalar();
	//	if (cosAngleCos >= dFloat32(0.998f))
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
}

void ndJointBallAndSocket::SubmitConeAngleOnlyRows(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc)
{
	const dFloat32 cosAngleCos = matrix1.m_front.DotProduct(matrix0.m_front).GetScalar();
	if (cosAngleCos >= dFloat32(0.998f))
	{
		dAssert(dAcos(dClamp(cosAngleCos, dFloat32(-1.0f), dFloat32(1.0f))) < m_maxConeAngle);
		dFloat32 pitchAngle = CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);
		AddAngularRowJacobian(desc, matrix0.m_front, pitchAngle);
		//dTrace(("%f\n", pitchAngle * dRadToDegree));
	}
	else
	{
		dVector lateralDir(matrix1[0].CrossProduct(matrix0[0]));
		dAssert(lateralDir.DotProduct(lateralDir).GetScalar() > 1.0e-6f);
		lateralDir = lateralDir.Normalize();
		const dFloat32 coneAngle = dAcos(dClamp(cosAngleCos, dFloat32(-1.0f), dFloat32(1.0f)));

		const dMatrix coneRotation(dQuaternion(lateralDir, coneAngle), matrix1.m_posit);
		const dVector sideDir(lateralDir.CrossProduct(matrix0.m_front));

		const dMatrix pitchMatrix(matrix1 * coneRotation * matrix0.Inverse());
		const dFloat32 pitchAngle = dAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
		dTrace(("%f %f\n", pitchAngle * dRadToDegree, coneAngle * dRadToDegree));
		dMatrix xxxxx0(dPitchMatrix(-pitchAngle) * matrix1 * coneRotation);

		dAssert(desc.m_rowsCount < 6);
		AddAngularRowJacobian(desc, matrix0.m_front, pitchAngle);
		//const dFloat32 stopAccel = GetMotorZeroAcceleration(desc) - dFloat32 (0.0f) * pitchAngle * desc.m_invTimestep * desc.m_invTimestep;
		//SetMotorAcceleration(desc, stopAccel);

		if (coneAngle > m_maxConeAngle)
		{
			AddAngularRowJacobian(desc, lateralDir, dFloat32(0.0f));
			const dFloat32 stopAccel = GetMotorZeroAcceleration(desc);
			const dFloat32 penetration = coneAngle - m_maxConeAngle;
			const dFloat32 recoveringAceel = desc.m_invTimestep * D_BALL_AND_SOCKED_PENETRATION_RECOVERY_SPEED * dMin(dAbs(penetration / D_BALL_AND_SOCKED_PENETRATION_LIMIT), dFloat32(1.0f));
			SetMotorAcceleration(desc, stopAccel - recoveringAceel);
			SetHighFriction(desc, dFloat32(0.0f));

			AddAngularRowJacobian(desc, sideDir, dFloat32(0.0f));
		}
		else if (m_coneFriction > dFloat32(0.0f))
		{
			AddAngularRowJacobian(desc, lateralDir, dFloat32(0.0f));
			SetMassSpringDamperAcceleration(desc, m_coneFrictionRegularizer, dFloat32(0.0f), m_coneFriction);

			AddAngularRowJacobian(desc, sideDir, dFloat32(0.0f));
			SetMassSpringDamperAcceleration(desc, m_coneFrictionRegularizer, dFloat32(0.0f), m_coneFriction);
		}
	}
}