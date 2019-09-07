/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// dCustomBallAndSocket.cpp: implementation of the dCustomBallAndSocket class.
//
//////////////////////////////////////////////////////////////////////
#include "dCustomJointLibraryStdAfx.h"
#include "dCustomBallAndSocket.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
IMPLEMENT_CUSTOM_JOINT(dCustomBallAndSocket);

#define D_BALL_AND_SOCKED_MAX_CONE_ANGLE dFloat (120.0f * dDegreeToRad)

dCustomBallAndSocket::dCustomBallAndSocket(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(6, child, parent)
	,m_twistAngle(0.0f)
	,m_minTwistAngle(-180.0f * dDegreeToRad)
	,m_maxTwistAngle(180.0f * dDegreeToRad)
	,m_maxConeAngle(D_BALL_AND_SOCKED_MAX_CONE_ANGLE)
	,m_coneFriction(0.0f)
	,m_twistFriction(0.0f)
{
	CalculateLocalMatrix(pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
}

dCustomBallAndSocket::dCustomBallAndSocket(const dMatrix& pinAndPivotFrame0, const dMatrix& pinAndPivotFrame1, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(6, child, parent)
	,m_twistAngle(0.0f)
	,m_minTwistAngle(-180.0f * dDegreeToRad)
	,m_maxTwistAngle(180.0f * dDegreeToRad)
	,m_maxConeAngle(D_BALL_AND_SOCKED_MAX_CONE_ANGLE)
	,m_coneFriction(0.0f)
	,m_twistFriction(0.0f)
{
	dMatrix	dummy;
	CalculateLocalMatrix(pinAndPivotFrame0, m_localMatrix0, dummy);
	CalculateLocalMatrix(pinAndPivotFrame1, dummy, m_localMatrix1);
}

dCustomBallAndSocket::~dCustomBallAndSocket()
{
}

void dCustomBallAndSocket::Deserialize(NewtonDeserializeCallback callback, void* const userData)
{
	callback(userData, &m_twistAngle, sizeof(dAngularIntegration));
	callback(userData, &m_minTwistAngle, sizeof(dFloat));
	callback(userData, &m_maxTwistAngle, sizeof(dFloat));
	callback(userData, &m_maxConeAngle, sizeof(dFloat));
	callback(userData, &m_coneFriction, sizeof(dFloat));
	callback(userData, &m_twistFriction, sizeof(dFloat));
}

void dCustomBallAndSocket::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize(callback, userData);

	callback(userData, &m_twistAngle, sizeof(dAngularIntegration));
	callback(userData, &m_minTwistAngle, sizeof(dFloat));
	callback(userData, &m_maxTwistAngle, sizeof(dFloat));
	callback(userData, &m_maxConeAngle, sizeof(dFloat));
	callback(userData, &m_coneFriction, sizeof(dFloat));
	callback(userData, &m_twistFriction, sizeof(dFloat));
}

void dCustomBallAndSocket::EnableTwist(bool state)
{
	m_options.m_option0 = state;
}

void dCustomBallAndSocket::EnableCone(bool state)
{
	m_options.m_option2 = state;
}

void dCustomBallAndSocket::SetTwistLimits(dFloat minAngle, dFloat maxAngle)
{
	m_minTwistAngle = -dAbs (minAngle);
	m_maxTwistAngle = dAbs(maxAngle);
}

void dCustomBallAndSocket::GetTwistLimits(dFloat& minAngle, dFloat& maxAngle) const
{
	minAngle = m_minTwistAngle;
	maxAngle = m_maxTwistAngle;
}

void dCustomBallAndSocket::SetTwistFriction (dFloat frictionTorque)
{
	m_twistFriction = dAbs (frictionTorque);
}

dFloat dCustomBallAndSocket::GetTwistFriction (dFloat frictionTorque) const
{
	return m_twistFriction;
}

dFloat dCustomBallAndSocket::GetConeLimits() const
{
	return m_maxConeAngle;
}

void dCustomBallAndSocket::SetConeLimits(dFloat maxAngle)
{
	m_maxConeAngle = dMin (dAbs (maxAngle), D_BALL_AND_SOCKED_MAX_CONE_ANGLE);
}

void dCustomBallAndSocket::SetConeFriction(dFloat frictionTorque)
{
	m_coneFriction = dAbs (frictionTorque);
}

dFloat dCustomBallAndSocket::GetConeFriction(dFloat frictionTorque) const
{
	return m_coneFriction;
}


void dCustomBallAndSocket::SetTwistSpringDamper(bool state, dFloat springDamperRelaxation, dFloat spring, dFloat damper)
{
//	m_spring = spring;
//	m_damper = damper;
	m_options.m_option1 = state;
//	m_springDamperRelaxation = dClamp(springDamperRelaxation, dFloat(0.0f), dFloat(0.999f));
}

void dCustomBallAndSocket::Debug(dDebugDisplay* const debugDisplay) const
{
	dMatrix matrix0;
	dMatrix matrix1;

	dCustomJoint::Debug(debugDisplay);

	CalculateGlobalMatrix(matrix0, matrix1);

	debugDisplay->DrawFrame(matrix0);
	debugDisplay->DrawFrame(matrix1);

	const dVector& coneDir0 = matrix0.m_front;
	const dVector& coneDir1 = matrix1.m_front;
	dFloat cosAngleCos = coneDir0.DotProduct3(coneDir1);
	dMatrix coneRotation(dGetIdentityMatrix());
	if (cosAngleCos < 0.9999f) {
		dVector lateralDir(coneDir1.CrossProduct(coneDir0));
		dFloat mag2 = lateralDir.DotProduct3(lateralDir);
		//dAssert(mag2 > 1.0e-4f);
		if (mag2 > 1.0e-4f) {
			lateralDir = lateralDir.Scale(1.0f / dSqrt(mag2));
			coneRotation = dMatrix(dQuaternion(lateralDir, dAcos(dClamp(cosAngleCos, dFloat(-1.0f), dFloat(1.0f)))), matrix1.m_posit);
		} else {
			lateralDir = matrix0.m_up.Scale(-1.0f);
			coneRotation = dMatrix(dQuaternion(matrix0.m_up, 180 * dDegreeToRad), matrix1.m_posit);
		}
	} else if (cosAngleCos < -0.9999f) {
		coneRotation[0][0] = -1.0f;
		coneRotation[1][1] = -1.0f;
	}

	const int subdiv = 18;
	const dFloat radius = debugDisplay->m_debugScale;
	dVector arch[subdiv + 1];

	// show twist angle limits
	if (m_options.m_option0 && ((m_maxTwistAngle - m_minTwistAngle) > dFloat(1.0e-3f))) {
		dMatrix pitchMatrix(matrix1 * coneRotation);
		pitchMatrix.m_posit = matrix1.m_posit;

		dVector point(dFloat(0.0f), dFloat(radius), dFloat(0.0f), dFloat(0.0f));

		dFloat angleStep = dMin (m_maxTwistAngle - m_minTwistAngle, dFloat (2.0f * dPi)) / subdiv;
		dFloat angle0 = m_minTwistAngle;

		debugDisplay->SetColor(dVector(0.6f, 0.2f, 0.0f, 0.0f));
		for (int i = 0; i <= subdiv; i++) {
			arch[i] = pitchMatrix.TransformVector(dPitchMatrix(angle0).RotateVector(point));
			debugDisplay->DrawLine(pitchMatrix.m_posit, arch[i]);
			angle0 += angleStep;
		}

		for (int i = 0; i < subdiv; i++) {
			debugDisplay->DrawLine(arch[i], arch[i + 1]);
		}
	}

	// show cone angle limits
	if (m_options.m_option2) {
		dVector point(radius * dCos(m_maxConeAngle), radius * dSin(m_maxConeAngle), 0.0f, 0.0f);
		dFloat angleStep = dPi * 2.0f / subdiv;
		dFloat angle0 = 0.0f;
		debugDisplay->SetColor(dVector(0.3f, 0.8f, 0.0f, 0.0f));

		for (int i = 0; i <= subdiv; i++) {
			dVector conePoint(dPitchMatrix(angle0).RotateVector(point));
			dVector p(matrix1.TransformVector(conePoint));
			arch[i] = p;
			debugDisplay->DrawLine(matrix1.m_posit, p);
			angle0 += angleStep;
		}

		for (int i = 0; i < subdiv; i++) {
			debugDisplay->DrawLine(arch[i], arch[i + 1]);
		}
	}
}

void dCustomBallAndSocket::SubmitConstraintTwistLimits(const dMatrix& matrix0, const dMatrix& matrix1, const dVector& relOmega, dFloat timestep)
{
	dFloat jointOmega = relOmega.DotProduct3(matrix0.m_front);
	dFloat twistAngle = m_twistAngle.GetAngle() + jointOmega * timestep;
	if (twistAngle < m_minTwistAngle) {
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_front[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_twistFriction);

		const dFloat invtimestep = 1.0f / timestep;
		const dFloat speed = 0.5f * (m_minTwistAngle - m_twistAngle.GetAngle()) * invtimestep;
		const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + speed * invtimestep;
		NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
	} else if (twistAngle > m_maxTwistAngle) {
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_front[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_twistFriction);

		const dFloat invtimestep = 1.0f / timestep;
		const dFloat speed = 0.5f * (m_maxTwistAngle - m_twistAngle.GetAngle()) * invtimestep;
		const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + speed * invtimestep;
		NewtonUserJointSetRowAcceleration(m_joint, stopAccel);

	} else if (m_twistFriction > 0.0f) {
		NewtonUserJointAddAngularRow(m_joint, 0, &matrix0.m_front[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);

		dFloat accel = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
		NewtonUserJointSetRowAcceleration(m_joint, accel);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_twistFriction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_twistFriction);
	}
}


void dCustomBallAndSocket::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);
	SubmitLinearRows(0x07, matrix0, matrix1);

	const dVector& coneDir0 = matrix0.m_front;
	const dVector& coneDir1 = matrix1.m_front;

	dFloat cosAngleCos = coneDir1.DotProduct3(coneDir0);
	if (cosAngleCos < dFloat(0.99995f)) {
		SubmitFullAngularAxis(matrix0, matrix1, timestep);
	} else {
		// front axis are aligned solve by 
		SubmitAngularFrontAxisAligned (matrix0, matrix1, timestep);
	}
}

void dCustomBallAndSocket::SubmitAngularFrontAxisAligned(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
{
	dTrace(("%s\n", __FUNCTION__));
}

void dCustomBallAndSocket::SubmitFullAngularAxis (const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
{
	const dVector& coneDir0 = matrix0.m_front;
	const dVector& coneDir1 = matrix1.m_front;
	const dVector lateralDir ((coneDir1.CrossProduct(coneDir0)).Normalize());
	const dQuaternion coneRotation (lateralDir, dAcos(dClamp(coneDir1.DotProduct3(coneDir0), dFloat(-1.0f), dFloat(1.0f))));

	//	dVector omega0(0.0f);
	//	dVector omega1(0.0f);
	//	NewtonBodyGetOmega(m_body0, &omega0[0]);
	//	if (m_body1) {
	//		NewtonBodyGetOmega(m_body1, &omega1[0]);
	//	}
	//	dVector relOmega(omega0 - omega1);

	// do twist angle calculations
	dMatrix coneMatrix(coneRotation, dVector(0.0f, 0.0f, 0.0f, 1.0f));
	dMatrix twistMatrix(matrix1 * coneMatrix * matrix0.Inverse());
	dAssert(dAbs(twistMatrix[0][1]) < 1.0e-5f);
	dAssert(dAbs(twistMatrix[0][2]) < 1.0e-5f);
	dAssert(dAbs(twistMatrix[0][0] - 1.0f) < 1.0e-5f);
	dFloat twistAngle = m_twistAngle.Update(dAtan2(twistMatrix[1][2], twistMatrix[1][1]));

	ApplyTwistAngleRow(matrix0.m_front, twistAngle, timestep);
	ApplyConeAngleRow(matrix0, matrix1, lateralDir, timestep);
}

void dCustomBallAndSocket::ApplyTwistAngleRow(const dVector& pin, dFloat twistAngle, dFloat timestep)
{
	const dFloat angleError = GetMaxAngleError();
	if (m_options.m_option0) {
		if ((m_minTwistAngle == 0.0f) && (m_minTwistAngle == 0.0f)) {
			NewtonUserJointAddAngularRow(m_joint, twistAngle, &pin[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			if (dAbs(twistAngle) > angleError) {
				const dFloat alpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dFloat(0.25f) * twistAngle / (timestep * timestep);
				NewtonUserJointSetRowAcceleration(m_joint, alpha);
			}
		} else {
			if (m_options.m_option1) {
				// TODO spring option
				dAssert(0);
			}
			else {
				dAssert(0);
				//				SubmitConstraintTwistLimits(matrix0, matrix1, relOmega, timestep);
			}
		}
	} else if (m_options.m_option1) {
		// TODO spring option
		dAssert(0);
	} else if (m_twistFriction > 0.0f) {
		dAssert(0);
		NewtonUserJointAddAngularRow(m_joint, 0, &pin[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);

		NewtonUserJointSetRowAcceleration(m_joint, NewtonUserJointCalculateRowZeroAcceleration(m_joint));
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_twistFriction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_twistFriction);
	}
}

void dCustomBallAndSocket::ApplyConeAngleRow(const dMatrix& matrix0, const dMatrix& matrix1, const dVector& sidePin, dFloat timestep)
{
	// cone spring unsuported at this time
	dAssert(!m_options.m_option3);

	if (m_options.m_option2) {
		// cone limit are enabled
		if (m_maxConeAngle == 0.0f) {
			dAssert(0);
			dMatrix localMatrix(matrix0 * matrix1.Inverse());
			dVector euler0;
			dVector euler1;
			localMatrix.GetEulerAngles(euler0, euler1, m_pitchRollYaw);
			NewtonUserJointAddAngularRow(m_joint, -euler0[1], &matrix1[1][0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointAddAngularRow(m_joint, -euler0[2], &matrix1[2][0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		} else {
			const dVector& coneDir0 = matrix0.m_front;
			const dVector& coneDir1 = matrix1.m_front;
			dFloat coneAngleCos = coneDir1.DotProduct3(coneDir0);
			//dFloat jointOmega = relOmega.DotProduct3(lateralDir);
			dFloat coneAngle = dAcos(dClamp(coneAngleCos, dFloat(-1.0f), dFloat(1.0f)));
			//dFloat coneAngle = currentAngle + jointOmega * timestep;
			if (coneAngle >= m_maxConeAngle) {
			//	//dQuaternion rot(lateralDir, coneAngle);
			//	//dVector frontDir(rot.RotateVector(coneDir1));
			//	//dVector upDir(lateralDir.CrossProduct(frontDir));
			//
			//	dVector upDir(lateralDir.CrossProduct(coneDir0));
			//	NewtonUserJointAddAngularRow(m_joint, 0.0f, &upDir[0]);
			//	NewtonUserJointSetRowAcceleration(m_joint, NewtonUserJointCalculateRowZeroAcceleration(m_joint));
			//	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				dFloat errorAngle = coneAngle - m_maxConeAngle;
				NewtonUserJointAddAngularRow(m_joint, errorAngle, &sidePin[0]);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);
			//	const dFloat invtimestep = 1.0f / timestep;
			//	const dFloat speed = 0.5f * (m_maxConeAngle - currentAngle) * invtimestep;
			//	const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + speed * invtimestep;
			//	NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			//
			} else if (m_coneFriction != 0) {
				dAssert(0);
			//	NewtonUserJointAddAngularRow(m_joint, 0.0f, &lateralDir[0]);
			//	NewtonUserJointSetRowAcceleration(m_joint, NewtonUserJointCalculateRowZeroAcceleration(m_joint));
			//	NewtonUserJointSetRowMinimumFriction(m_joint, -m_coneFriction);
			//	NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);
			//
			//	dVector upDir(lateralDir.CrossProduct(coneDir0));
			//	NewtonUserJointAddAngularRow(m_joint, 0.0f, &upDir[0]);
			//	NewtonUserJointSetRowAcceleration(m_joint, NewtonUserJointCalculateRowZeroAcceleration(m_joint));
			//	NewtonUserJointSetRowMinimumFriction(m_joint, -m_coneFriction);
			//	NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);
			}
		}
	} else if (m_coneFriction > 0.0f) {
		dAssert(0);
/*
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &lateralDir[0]);
		NewtonUserJointSetRowAcceleration(m_joint, NewtonUserJointCalculateRowZeroAcceleration(m_joint));
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_coneFriction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);

		dVector upDir(lateralDir.CrossProduct(coneDir0));
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &upDir[0]);
		NewtonUserJointSetRowAcceleration(m_joint, NewtonUserJointCalculateRowZeroAcceleration(m_joint));
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_coneFriction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);
*/
	}
}