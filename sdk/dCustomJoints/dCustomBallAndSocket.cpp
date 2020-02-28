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
	,m_coneStiffness(1.0f)
	,m_twistSpring(0.0f)
	,m_twistDamper(0.0f)
	,m_coneSpring(0.0f)
	,m_coneDamper(0.0f)
	,m_mask(0x07)
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
	,m_coneStiffness(1.0f)
	,m_twistSpring(0.0f)
	,m_twistDamper(0.0f)
	,m_coneSpring(0.0f)
	,m_coneDamper(0.0f)
	,m_mask(0x07)
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
	callback(userData, &m_coneStiffness, sizeof(dFloat));
	callback(userData, &m_twistSpring, sizeof(dFloat));
	callback(userData, &m_twistDamper, sizeof(dFloat));
	callback(userData, &m_coneSpring, sizeof(dFloat));
	callback(userData, &m_coneDamper, sizeof(dFloat));
	callback(userData, &m_mask, sizeof(int));
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
	callback(userData, &m_coneStiffness, sizeof(dFloat));
	callback(userData, &m_twistSpring, sizeof(dFloat));
	callback(userData, &m_twistDamper, sizeof(dFloat));
	callback(userData, &m_coneSpring, sizeof(dFloat));
	callback(userData, &m_coneDamper, sizeof(dFloat));
	callback(userData, &m_mask, sizeof(int));
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

void dCustomBallAndSocket::SetConeStiffness(dFloat coneStiffness)
{
	m_coneStiffness = coneStiffness;
}

dFloat dCustomBallAndSocket::GetConeStiffness(dFloat frictionTorque) const
{
	return m_coneStiffness;
}

void dCustomBallAndSocket::SetConeSpringDamper(bool state, dFloat spring, dFloat damper)
{
	m_coneSpring = dAbs(spring);
	m_coneDamper = dAbs(damper);
	m_options.m_option4 = state;
}

void dCustomBallAndSocket::SetTwistSpringDamper(bool state, dFloat spring, dFloat damper)
{
	m_twistSpring = dAbs(spring);
	m_twistDamper = dAbs(damper);
	m_options.m_option3 = state;
}

void dCustomBallAndSocket::Debug(dDebugDisplay* const debugDisplay) const
{
	dMatrix matrix0;
	dMatrix matrix1;

	dCustomJoint::Debug(debugDisplay);

	CalculateGlobalMatrix(matrix0, matrix1);

	debugDisplay->DrawFrame(matrix0);
	debugDisplay->DrawFrame(matrix1);

	const int subdiv = 8;
	const dVector& coneDir0 = matrix0.m_front;
	const dVector& coneDir1 = matrix1.m_front;
	dFloat cosAngleCos = coneDir0.DotProduct3(coneDir1);
	dMatrix coneRotation(dGetIdentityMatrix());
	if (cosAngleCos < 0.9999f) {
		dVector lateralDir(coneDir1.CrossProduct(coneDir0));
		dFloat mag2 = lateralDir.DotProduct3(lateralDir);
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

	
	const dFloat radius = debugDisplay->m_debugScale;
	dVector arch[subdiv + 1];

	// show twist angle limits
	if (m_options.m_option0 && ((m_maxTwistAngle - m_minTwistAngle) > dFloat(1.0e-3f))) {
		dMatrix pitchMatrix(matrix1 * coneRotation);
		pitchMatrix.m_posit = matrix1.m_posit;

		dVector point(dFloat(0.0f), dFloat(radius), dFloat(0.0f), dFloat(0.0f));

		dFloat angleStep = dMin (m_maxTwistAngle - m_minTwistAngle, dFloat (2.0f * dPi)) / subdiv;
		dFloat angle0 = m_minTwistAngle;

		debugDisplay->SetColor(dVector(0.4f, 0.0f, 0.0f, 0.0f));
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

void dCustomBallAndSocket::SubmitTwistAngle(const dVector& pin, dFloat angle, dFloat timestep)
{
	if ((m_maxTwistAngle - m_minTwistAngle) < (2.0f * dDegreeToRad)) {
		NewtonUserJointAddAngularRow(m_joint, -angle, &pin[0]);
	} else {
		dFloat restoringOmega = 0.25f;
		const dFloat step = dMax(dAbs(angle * timestep), dFloat(5.0f * dDegreeToRad));
		if (angle < m_minTwistAngle) {
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &pin[0]);
			const dFloat invtimestep = 1.0f / timestep;
			const dFloat error0 = angle - m_minTwistAngle;
			const dFloat error1 = error0 + restoringOmega * timestep;
			if (error1 > 0.0f) {
				restoringOmega = -error0 * invtimestep;
			}
			const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + restoringOmega * invtimestep;
			NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_twistFriction);
		} else if (angle >= m_maxTwistAngle) {
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &pin[0]);
			const dFloat invtimestep = 1.0f / timestep;
			const dFloat error0 = angle - m_maxTwistAngle;
			const dFloat error1 = error0 - restoringOmega * timestep;
			if (error1 < 0.0f) {
				restoringOmega = error0 * invtimestep;
			}
			const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint) - restoringOmega / timestep;
			NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_twistFriction);
		} else if ((angle - step) <= m_minTwistAngle) {
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &pin[0]);
			const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_twistFriction);
		} else if ((angle + step) >= m_maxTwistAngle) {
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &pin[0]);
			const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_twistFriction);
		} else if (m_twistFriction > 0.0f) {
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &pin[0]);
			const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_twistFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_twistFriction);
		}
	}
}

void dCustomBallAndSocket::SubmitAngularAxisCartisianApproximation(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
{
	if (m_options.m_option2) {
		// two rows to restrict rotation around around the parent coordinate system
		//dFloat coneAngle = dAcos(dClamp(matrix1.m_front.DotProduct3(matrix0.m_front), dFloat(-1.0f), dFloat(1.0f)));
		const dFloat angleError = GetMaxAngleError();
		dFloat angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
		NewtonUserJointAddAngularRow(m_joint, angle0, &matrix1.m_up[0]);
		//NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowStiffness(m_joint, m_coneStiffness);
		if (dAbs(angle0) > angleError) {
			const dFloat alpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dFloat(0.25f) * angle0 / (timestep * timestep);
			NewtonUserJointSetRowAcceleration(m_joint, alpha);
		}

		dFloat angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
		NewtonUserJointAddAngularRow(m_joint, angle1, &matrix1.m_right[0]);
		//NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowStiffness(m_joint, m_coneStiffness);
		if (dAbs(angle1) > angleError) {
			const dFloat alpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dFloat(0.25f) * angle1 / (timestep * timestep);
			NewtonUserJointSetRowAcceleration(m_joint, alpha);
		}
	}

	if (m_options.m_option0) {
		dFloat pitchAngle = -CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);
		SubmitTwistAngle(matrix0.m_front, pitchAngle, timestep);
	}
}

void dCustomBallAndSocket::SubmitAngularAxis(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
{
	dVector lateralDir(matrix1[0].CrossProduct(matrix0[0]));
	dAssert(lateralDir.DotProduct3(lateralDir) > 1.0e-6f);
	lateralDir = lateralDir.Normalize();
	dFloat coneAngle = dAcos(dClamp(matrix1.m_front.DotProduct3(matrix0.m_front), dFloat(-1.0f), dFloat(1.0f)));
	dMatrix coneRotation(dQuaternion(lateralDir, coneAngle), matrix1.m_posit);

	if (m_options.m_option2) {
		const dFloat step = dMax(dAbs(coneAngle * timestep), dFloat(5.0f * dDegreeToRad));
		if (coneAngle > m_maxConeAngle) {
			dFloat restoringOmega = 0.25f;
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &lateralDir[0]);
			const dFloat invtimestep = 1.0f / timestep;
			const dFloat error0 = coneAngle - m_maxConeAngle;
			const dFloat error1 = error0 - restoringOmega * timestep;
			if (error1 < 0.0f) {
				restoringOmega = error0 * invtimestep;
			}
			const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint) - restoringOmega / timestep;
			NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			//NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowStiffness(m_joint, m_coneStiffness);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);

			dVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);

		} else if ((coneAngle + step) >= m_maxConeAngle) {
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &lateralDir[0]);
			const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			//NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowStiffness(m_joint, m_coneStiffness);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);

			dVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);
		} else if (m_coneFriction > 0.0f) {
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &lateralDir[0]);
			const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			//NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowStiffness(m_joint, m_coneStiffness);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_coneFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);

			dVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_coneFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);
		}
	}

	if (m_options.m_option0) {
		dMatrix pitchMatrix(matrix1 * coneRotation * matrix0.Inverse());
		dFloat pitchAngle = -dAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
		SubmitTwistAngle(matrix0.m_front, pitchAngle, timestep);
	}
}

void dCustomBallAndSocket::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);
	SubmitLinearRows(m_mask, matrix0, matrix1);

	dFloat cosAngleCos = matrix1.m_front.DotProduct3(matrix0.m_front);
	if (cosAngleCos < dFloat(0.998f)) {
		SubmitAngularAxis(matrix0, matrix1, timestep);
	} else {
		// front axis are aligned solve by Cartesian approximation
		SubmitAngularAxisCartisianApproximation(matrix0, matrix1, timestep);
	}
}
