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
#include "ndJointBallAndSocket.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointBallAndSocket)

ndJointBallAndSocket::ndJointBallAndSocket(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(6, child, parent, pinAndPivotFrame)
	,m_maxConeAngle(ndFloat32 (1.0e10f))
	,m_coneFriction(ndFloat32(0.0))
	,m_minTwistAngle(-ndFloat32(1.0e10f))
	,m_maxTwistAngle( ndFloat32(1.0e10f))
	,m_twistFriction(ndFloat32 (0.0f))
	,m_coneFrictionRegularizer(ndFloat32(0.0f))
	,m_twistFrictionRegularizer(ndFloat32(0.0f))
{
}

ndJointBallAndSocket::ndJointBallAndSocket(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointBilateralConstraint(ndLoadSaveBase::ndLoadDescriptor(desc))
	,m_maxConeAngle(ndFloat32(1.0e10f))
	,m_coneFriction(ndFloat32(0.0))
	,m_minTwistAngle(-ndFloat32(1.0e10f))
	,m_maxTwistAngle(ndFloat32(1.0e10f))
	,m_twistFriction(ndFloat32(0.0f))
	,m_coneFrictionRegularizer(ndFloat32(0.0f))
	,m_twistFrictionRegularizer(ndFloat32(0.0f))
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

void ndJointBallAndSocket::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "maxConeAngle", m_maxConeAngle);
	xmlSaveParam(childNode, "coneFriction", m_coneFriction);
	xmlSaveParam(childNode, "minTwistAngle", m_minTwistAngle);
	xmlSaveParam(childNode, "maxTwistAngle", m_maxTwistAngle);
	xmlSaveParam(childNode, "twistFriction", m_twistFriction);
	xmlSaveParam(childNode, "coneFrictionRegularizer", m_coneFrictionRegularizer);
	xmlSaveParam(childNode, "twistFrictionRegularizer", m_twistFrictionRegularizer);
}

void ndJointBallAndSocket::SetConeFriction(ndFloat32 regularizer, ndFloat32 viscousFriction)
{
	m_coneFriction = dAbs(viscousFriction);
	m_coneFrictionRegularizer = dMax(dAbs(regularizer), ndFloat32(0.0f));
}

void ndJointBallAndSocket::SetTwistLimits(ndFloat32 minAngle, ndFloat32 maxAngle)
{
	m_minTwistAngle = dMin(minAngle, ndFloat32 (0.0f));
	m_maxTwistAngle = dMax(maxAngle, ndFloat32(0.0f));
}

void ndJointBallAndSocket::SetTwistFriction(ndFloat32 regularizer, ndFloat32 viscousFriction)
{
	m_twistFriction = dAbs(viscousFriction);
	m_twistFrictionRegularizer = dMax(dAbs(regularizer), ndFloat32(0.0f));
}

void ndJointBallAndSocket::GetTwistLimits(ndFloat32& minAngle, ndFloat32& maxAngle) const
{
	minAngle = m_minTwistAngle;
	maxAngle = m_maxTwistAngle;
}

ndFloat32 ndJointBallAndSocket::GetMaxConeAngle() const
{
	return m_maxConeAngle;
}

void ndJointBallAndSocket::SetConeLimit(ndFloat32 maxConeAngle)
{
	//m_maxConeAngle = dMin (dAbs (maxConeAngle), D_BALL_AND_SOCKED_MAX_ANGLE * ndFloat32 (0.999f));
	m_maxConeAngle = maxConeAngle;
}

void ndJointBallAndSocket::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

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
		if (mag2 > ndFloat32 (1.0e-4f)) 
		{
			lateralDir = lateralDir.Scale(ndFloat32 (1.0f) / ndSqrt(mag2));
			coneRotation = ndMatrix(ndQuaternion(lateralDir, ndAcos(dClamp(cosAngleCos, ndFloat32(-1.0f), ndFloat32(1.0f)))), matrix1.m_posit);
		}
		else 
		{
			lateralDir = matrix0.m_up.Scale(-ndFloat32 (1.0f));
			coneRotation = ndMatrix(ndQuaternion(matrix0.m_up, ndFloat32 (180.0f) * ndDegreeToRad), matrix1.m_posit);
		}
	}
	else if (cosAngleCos < -ndFloat32 (0.9999f)) 
	{
		coneRotation[0][0] = ndFloat32(-1.0f);
		coneRotation[1][1] = ndFloat32(-1.0f);
	}
	
	const ndFloat32 radius = debugCallback.m_debugScale;
	ndVector arch[subdiv + 1];
	
	// show twist angle limits
	ndFloat32 deltaTwist = m_maxTwistAngle - m_minTwistAngle;
	if ((deltaTwist > ndFloat32(1.0e-3f)) && (deltaTwist < ndFloat32 (2.0f) * ndPi))
	{ 
		ndMatrix pitchMatrix(matrix1 * coneRotation);
		pitchMatrix.m_posit = matrix1.m_posit;
	
		ndVector point(ndFloat32(0.0f), ndFloat32(radius), ndFloat32(0.0f), ndFloat32(0.0f));
	
		ndFloat32 angleStep = dMin(m_maxTwistAngle - m_minTwistAngle, ndFloat32(2.0f * ndPi)) / subdiv;
		ndFloat32 angle0 = m_minTwistAngle;
	
		ndVector color(ndFloat32 (0.4f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
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
	if ((m_maxConeAngle > ndFloat32 (0.0f)) && (m_maxConeAngle < D_BALL_AND_SOCKED_MAX_ANGLE)) 
	{
		ndVector color(ndFloat32(0.3f), ndFloat32(0.8f), ndFloat32(0.0f), ndFloat32(0.0f));
		ndVector point(radius * ndCos(m_maxConeAngle), radius * ndSin(m_maxConeAngle), ndFloat32 (0.0f), ndFloat32(0.0f));
		ndFloat32 angleStep = ndPi * ndFloat32(2.0f) / subdiv;
	
		ndFloat32 angle0 = ndFloat32 (0.0f);
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

void ndJointBallAndSocket::SubmitAngularAxisCartesianApproximation(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	ndFloat32 coneAngle = ndAcos(dClamp(matrix1.m_front.DotProduct(matrix0.m_front).GetScalar(), ndFloat32(-1.0f), ndFloat32(1.0f)));
	if (coneAngle > m_maxConeAngle)
	{
		// two rows to restrict rotation around around the parent coordinate system
		ndFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
		AddAngularRowJacobian(desc, matrix1.m_up, angle0);

		ndFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
		AddAngularRowJacobian(desc, matrix1.m_right, angle1);
	}

	ndFloat32 pitchAngle = -CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);
	SubmitTwistAngle(matrix0.m_front, pitchAngle, desc);
}

void ndJointBallAndSocket::SubmitTwistAngle(const ndVector& pin, ndFloat32 angle, ndConstraintDescritor& desc)
{
	if ((m_maxTwistAngle - m_minTwistAngle) < (2.0f * ndDegreeToRad)) 
	{
		dAssert(desc.m_rowsCount < 6);
		AddAngularRowJacobian(desc, pin, -angle);
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
			SetLowerFriction(desc, ndFloat32 (0.0f));
		}
		else if (angle >= m_maxTwistAngle) 
		{
			AddAngularRowJacobian(desc, pin, ndFloat32(0.0f));
			const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
			const ndFloat32 penetration = angle - m_maxTwistAngle;
			const ndFloat32 recoveringAceel = desc.m_invTimestep * D_BALL_AND_SOCKED_PENETRATION_RECOVERY_SPEED * dMin(dAbs(penetration / D_BALL_AND_SOCKED_PENETRATION_LIMIT), ndFloat32(1.0f));
			SetMotorAcceleration(desc, stopAccel - recoveringAceel);
			SetHighFriction(desc, ndFloat32 (0.0f));
		}
		else if (m_twistFriction > ndFloat32(0.0f))
		{
			AddAngularRowJacobian(desc, pin, ndFloat32 (0.0f));
			SetMassSpringDamperAcceleration(desc, m_twistFrictionRegularizer, ndFloat32 (0.0f), m_twistFriction);
		}
	}
}

void ndJointBallAndSocket::SubmitAngularAxis(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	ndVector lateralDir(matrix1[0].CrossProduct(matrix0[0]));
	dAssert(lateralDir.DotProduct(lateralDir).GetScalar() > 1.0e-6f);
	lateralDir = lateralDir.Normalize();
	const ndFloat32 coneAngle = ndAcos(dClamp(matrix1.m_front.DotProduct(matrix0.m_front).GetScalar(), ndFloat32(-1.0f), ndFloat32(1.0f)));
	const ndMatrix coneRotation(ndQuaternion(lateralDir, coneAngle), matrix1.m_posit);
	const ndVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
	if (coneAngle > m_maxConeAngle)
	{ 
		AddAngularRowJacobian(desc, lateralDir, ndFloat32 (0.0f));
		const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
		const ndFloat32 penetration = coneAngle - m_maxConeAngle;
		const ndFloat32 recoveringAceel = desc.m_invTimestep * D_BALL_AND_SOCKED_PENETRATION_RECOVERY_SPEED * dMin(dAbs(penetration / D_BALL_AND_SOCKED_PENETRATION_LIMIT), ndFloat32(1.0f));
		SetMotorAcceleration(desc, stopAccel - recoveringAceel);
		SetHighFriction(desc, ndFloat32(0.0f));
		
		AddAngularRowJacobian(desc, sideDir, ndFloat32 (0.0f));
	}
	else if (m_coneFriction > ndFloat32 (0.0f))
	{
		AddAngularRowJacobian(desc, lateralDir, ndFloat32 (0.0f));
		SetMassSpringDamperAcceleration(desc, m_coneFrictionRegularizer, ndFloat32(0.0f), m_coneFriction);

		AddAngularRowJacobian(desc, sideDir, ndFloat32 (0.0f));
		SetMassSpringDamperAcceleration(desc, m_coneFrictionRegularizer, ndFloat32(0.0f), m_coneFriction);
	}

	const ndMatrix pitchMatrix(matrix1 * coneRotation * matrix0.Inverse());
	const ndFloat32 pitchAngle = -ndAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
	SubmitTwistAngle(matrix0.m_front, pitchAngle, desc);
}

void ndJointBallAndSocket::SubmitConeAngleOnlyRows(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	const ndFloat32 cosAngleCos = matrix1.m_front.DotProduct(matrix0.m_front).GetScalar();
	if (cosAngleCos >= ndFloat32(0.998f))
	{
		dAssert(ndAcos(dClamp(cosAngleCos, ndFloat32(-1.0f), ndFloat32(1.0f))) < m_maxConeAngle);
		ndFloat32 pitchAngle = CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);
		AddAngularRowJacobian(desc, matrix0.m_front, pitchAngle);
		//dTrace(("%f\n", pitchAngle * ndRadToDegree));
	}
	else
	{
		ndVector lateralDir(matrix1[0].CrossProduct(matrix0[0]));
		dAssert(lateralDir.DotProduct(lateralDir).GetScalar() > 1.0e-6f);
		lateralDir = lateralDir.Normalize();
		const ndFloat32 coneAngle = ndAcos(dClamp(cosAngleCos, ndFloat32(-1.0f), ndFloat32(1.0f)));

		const ndMatrix coneRotation(ndQuaternion(lateralDir, coneAngle), matrix1.m_posit);
		const ndVector sideDir(lateralDir.CrossProduct(matrix0.m_front));

		const ndMatrix pitchMatrix(matrix1 * coneRotation * matrix0.Inverse());
		const ndFloat32 pitchAngle = ndAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
		dTrace(("%f %f\n", pitchAngle * ndRadToDegree, coneAngle * ndRadToDegree));
		ndMatrix xxxxx0(dPitchMatrix(-pitchAngle) * matrix1 * coneRotation);

		dAssert(desc.m_rowsCount < 6);
		AddAngularRowJacobian(desc, matrix0.m_front, pitchAngle);
		//const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc) - ndFloat32 (0.0f) * pitchAngle * desc.m_invTimestep * desc.m_invTimestep;
		//SetMotorAcceleration(desc, stopAccel);

		if (coneAngle > m_maxConeAngle)
		{
			AddAngularRowJacobian(desc, lateralDir, ndFloat32(0.0f));
			const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
			const ndFloat32 penetration = coneAngle - m_maxConeAngle;
			const ndFloat32 recoveringAceel = desc.m_invTimestep * D_BALL_AND_SOCKED_PENETRATION_RECOVERY_SPEED * dMin(dAbs(penetration / D_BALL_AND_SOCKED_PENETRATION_LIMIT), ndFloat32(1.0f));
			SetMotorAcceleration(desc, stopAccel - recoveringAceel);
			SetHighFriction(desc, ndFloat32(0.0f));

			AddAngularRowJacobian(desc, sideDir, ndFloat32(0.0f));
		}
		else if (m_coneFriction > ndFloat32(0.0f))
		{
			AddAngularRowJacobian(desc, lateralDir, ndFloat32(0.0f));
			SetMassSpringDamperAcceleration(desc, m_coneFrictionRegularizer, ndFloat32(0.0f), m_coneFriction);

			AddAngularRowJacobian(desc, sideDir, ndFloat32(0.0f));
			SetMassSpringDamperAcceleration(desc, m_coneFrictionRegularizer, ndFloat32(0.0f), m_coneFriction);
		}
	}
}

void ndJointBallAndSocket::JacobianDerivative(ndConstraintDescritor& desc)
{
	//if (m_body0->GetId() != 5)
	//return ;

	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[0]);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[1]);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[2]);

	ndFloat32 deltaTwist = m_maxTwistAngle - m_minTwistAngle;
	if (deltaTwist < (2.0f * ndDegreeToRad))
	{
		SubmitConeAngleOnlyRows(matrix0, matrix1, desc);
	}
	else
	{
		//dAssert(0);
	}

	//bool hasAngleRows = deltaTwist > ndFloat32(1.0e-3f);
	//hasAngleRows = hasAngleRows && (deltaTwist < ndFloat32(2.0f) * ndPi);
	//hasAngleRows = hasAngleRows || (m_maxConeAngle < D_BALL_AND_SOCKED_MAX_ANGLE);
	//if (hasAngleRows)
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
}