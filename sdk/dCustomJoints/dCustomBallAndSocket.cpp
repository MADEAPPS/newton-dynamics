/* Copyright (c) <2003-2016> <Newton Game Dynamics>
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
IMPLEMENT_CUSTOM_JOINT(dCustomLimitBallAndSocket);

dCustomBallAndSocket::dCustomBallAndSocket(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(6, child, parent)
{
	CalculateLocalMatrix(pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
}

dCustomBallAndSocket::dCustomBallAndSocket(const dMatrix& pinAndPivotFrame0, const dMatrix& pinAndPivotFrame1, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(6, child, parent)
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
}

void dCustomBallAndSocket::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize(callback, userData);
}

void dCustomBallAndSocket::SubmitLinearRows (const dMatrix& matrix0, const dMatrix& matrix1) const
{
	const dVector& p0 = matrix0.m_posit;
	const dVector& p1 = matrix1.m_posit;

	for (int i = 0; i < 3; i++) {
		NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1[i][0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	}
}

void dCustomBallAndSocket::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	SubmitLinearRows (matrix0, matrix1);
}


//***********************
dCustomLimitBallAndSocket::dCustomLimitBallAndSocket(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomBallAndSocket(pinAndPivotFrame, child, parent)
	,m_friction(0.0f)
	,m_minTwistAngle(0.0f)
	,m_maxTwistAngle(0.0f)
{
	SetConeAngle (45.0f * dDegreeToRad);
	SetTwistAngle (-45.0f * dDegreeToRad, 45.0f * dDegreeToRad);
}

dCustomLimitBallAndSocket::dCustomLimitBallAndSocket(const dMatrix& pinAndPivotFrameChild, const dMatrix& pinAndPivotFrameParent, NewtonBody* const child, NewtonBody* const parent)
	:dCustomBallAndSocket(pinAndPivotFrameChild, pinAndPivotFrameParent, child, parent)
	,m_friction(0.0f)
	,m_minTwistAngle(0.0f)
	,m_maxTwistAngle(0.0f)
{
	SetConeAngle(45.0f * dDegreeToRad);
	SetTwistAngle(-45.0f * dDegreeToRad, 45.0f * dDegreeToRad);
}

dCustomLimitBallAndSocket::~dCustomLimitBallAndSocket()
{
}

void dCustomLimitBallAndSocket::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	callback(userData, &m_coneAngle, sizeof(dFloat));
	callback(userData, &m_coneAngleCos, sizeof(dFloat));
}

void dCustomLimitBallAndSocket::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	dCustomBallAndSocket::Serialize (callback, userData);

	callback (userData, &m_coneAngle, sizeof (dFloat));
	callback (userData, &m_coneAngleCos, sizeof (dFloat));
}

void dCustomLimitBallAndSocket::SetConeAngle (dFloat angle)
{
	m_coneAngle = dClamp (dAbs (angle), dFloat (0.0f), dFloat (160.0f) * dDegreeToRad);
	m_coneAngleCos = dCos (m_coneAngle);
}

void dCustomLimitBallAndSocket::SetTwistAngle (dFloat minAngle, dFloat maxAngle)
{
	m_minTwistAngle = dClamp(minAngle, -dFloat (180.0f) * dDegreeToRad, dFloat (0.0f));
	m_maxTwistAngle = dClamp(maxAngle, dFloat (0.0f), dFloat (180.0f) * dDegreeToRad);
}

void dCustomLimitBallAndSocket::GetTwistAngle(dFloat& minAngle, dFloat& maxAngle) const
{
	minAngle = m_minTwistAngle;
	maxAngle = m_maxTwistAngle;
}

dFloat dCustomLimitBallAndSocket::GetConeAngle () const
{
	return m_coneAngle;
}

void dCustomLimitBallAndSocket::SetFriction (dFloat friction)
{
	m_friction = dAbs (friction);
}

dFloat dCustomLimitBallAndSocket::GetFriction() const
{
	return m_friction;
}


/*
void dCustomLimitBallAndSocket::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);
	matrix1 = m_rotationOffset * matrix1;

	dCustomBallAndSocket::SubmitConstraints(timestep, threadIndex);

	// handle special case of the joint being a hinge
	if (m_coneAngleCos > 0.9999f) {
		NewtonUserJointAddAngularRow(m_joint, CalculateAngle (matrix0.m_front, matrix1.m_front, matrix1.m_up), &matrix1.m_up[0]);
		NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right), &matrix1.m_right[0]);

		// the joint angle can be determined by getting the angle between any two non parallel vectors
		dFloat pitchAngle = CalculateAngle (matrix0.m_up, matrix1.m_up, matrix1.m_front);
		if ((m_maxTwistAngle - m_minTwistAngle) < 1.0e-4f) {
			NewtonUserJointAddAngularRow(m_joint, pitchAngle, &matrix1.m_front[0]);
		} else {
			if (pitchAngle > m_maxTwistAngle) {
				pitchAngle -= m_maxTwistAngle;
				NewtonUserJointAddAngularRow(m_joint, pitchAngle, &matrix0.m_front[0]);
				NewtonUserJointSetRowMinimumFriction(m_joint, -0.0f);
			} else if (pitchAngle < m_minTwistAngle) {
				pitchAngle -= m_minTwistAngle;
				NewtonUserJointAddAngularRow(m_joint, pitchAngle, &matrix0.m_front[0]);
				NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
			}
		}

	} else {

		const dVector& coneDir0 = matrix0.m_front;
		const dVector& coneDir1 = matrix1.m_front;
		dFloat cosAngle = coneDir0.DotProduct3(coneDir1);
		if (cosAngle <= m_coneAngleCos) {
			dVector lateralDir(coneDir0.CrossProduct(coneDir1));
			dFloat mag2 = lateralDir.DotProduct3(lateralDir);
			dAssert(mag2 > 1.0e-4f);
			lateralDir = lateralDir.Scale(1.0f / dSqrt(mag2));

			dQuaternion rot(m_coneAngleHalfCos, lateralDir.m_x * m_coneAngleHalfSin, lateralDir.m_y * m_coneAngleHalfSin, lateralDir.m_z * m_coneAngleHalfSin);
			dVector frontDir(rot.UnrotateVector(coneDir1));
			dVector upDir(lateralDir.CrossProduct(frontDir));
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &upDir[0]);
			NewtonUserJointAddAngularRow(m_joint, CalculateAngle(coneDir0, frontDir, lateralDir), &lateralDir[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
		}

		//handle twist angle
		dFloat pitchAngle = CalculateAngle (matrix0.m_up, matrix1.m_up, matrix1.m_front);
		if ((m_maxTwistAngle - m_minTwistAngle) < 1.0e-4f) {
			NewtonUserJointAddAngularRow(m_joint, pitchAngle, &matrix1.m_front[0]);
		} else {
			if (pitchAngle > m_maxTwistAngle) {
				pitchAngle -= m_maxTwistAngle;
				NewtonUserJointAddAngularRow(m_joint, pitchAngle, &matrix0.m_front[0]);
				NewtonUserJointSetRowMinimumFriction(m_joint, -0.0f);
			} else if (pitchAngle < m_minTwistAngle) {
				pitchAngle -= m_minTwistAngle;
				NewtonUserJointAddAngularRow(m_joint, pitchAngle, &matrix0.m_front[0]);
				NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
			}
		}
	}
}
*/

void dCustomLimitBallAndSocket::Debug(dDebugDisplay* const debugDisplay) const
{
	dCustomBallAndSocket::Debug(debugDisplay);

	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	const dVector& coneDir0 = matrix0.m_front;
	const dVector& coneDir1 = matrix1.m_front;
	dFloat cosAngleCos = coneDir0.DotProduct3(coneDir1);
	dMatrix coneRotation (dGetIdentityMatrix());
	if (cosAngleCos < 0.9999f) {
		dVector lateralDir(coneDir1.CrossProduct(coneDir0));
		dFloat mag2 = lateralDir.DotProduct3(lateralDir);
		dAssert (mag2 > 1.0e-4f);
		lateralDir = lateralDir.Scale(1.0f / dSqrt(mag2));
		coneRotation = dMatrix (dQuaternion (lateralDir, dAcos (dClamp (cosAngleCos, dFloat(-1.0f), dFloat(1.0f)))), matrix1.m_posit);
	} else if (cosAngleCos < -0.9999f) {
		coneRotation[0][0] = -1.0f;
		coneRotation[1][1] = -1.0f;
	}

	const int subdiv = 18;
	const float radius = debugDisplay->m_debugScale;
	dVector arch[subdiv + 1];

	// show pitch angle limits
	if (m_maxTwistAngle - m_minTwistAngle > dFloat (1.0e-3f)) {
		dMatrix pitchMatrix (matrix1 * coneRotation);
		pitchMatrix.m_posit = matrix1.m_posit;

		dVector point(dFloat(0.0f), dFloat(radius), dFloat(0.0f), dFloat(0.0f));
		dFloat angleStep = (m_maxTwistAngle - m_minTwistAngle) / subdiv;
		dFloat angle0 = m_minTwistAngle;

		debugDisplay->SetColor(dVector(0.5f, 0.0f, 0.0f, 0.0f));
		for (int i = 0; i <= subdiv; i++) {
			arch[i] = pitchMatrix.TransformVector(dPitchMatrix(angle0).RotateVector(point));
			debugDisplay->DrawLine(pitchMatrix.m_posit, arch[i]);
			angle0 += angleStep;
		}

		for (int i = 0; i < subdiv; i++) {
			debugDisplay->DrawLine(arch[i], arch[i + 1]);
		}
	}

	{
		dVector point(radius * dCos(m_coneAngle), radius * dSin(m_coneAngle), 0.0f, 0.0f);
		dFloat angleStep = dPi * 2.0f / subdiv;
		dFloat angle0 = 0.0f;

		dVector arch[subdiv + 1];
		debugDisplay->SetColor(dVector(1.0f, 1.0f, 0.0f, 0.0f));

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

void dCustomLimitBallAndSocket::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);
	SubmitLinearRows (matrix0, matrix1);

	const dVector& coneDir0 = matrix0.m_front;
	const dVector& coneDir1 = matrix1.m_front;

	dVector omega0(0.0f);
	dVector omega1(0.0f);

	dAssert (m_body1);

	// get the omega vector
	NewtonBodyGetOmega(m_body0, &omega0[0]);
	NewtonBodyGetOmega(m_body1, &omega1[0]);
	dVector relOmega(omega0 - omega1);

	dFloat cosAngleCos = coneDir1.DotProduct3(coneDir0);
	dMatrix coneRotation(dGetIdentityMatrix());
	dVector lateralDir(matrix0.m_up);

	if (cosAngleCos < 0.9999f) {
		lateralDir = coneDir1.CrossProduct(coneDir0);
		dFloat mag2 = lateralDir.DotProduct3(lateralDir);
		dAssert(mag2 > 1.0e-4f);
		lateralDir = lateralDir.Scale(1.0f / dSqrt(mag2));
		coneRotation = dMatrix(dQuaternion(lateralDir, dAcos(dClamp(cosAngleCos, dFloat(-1.0f), dFloat(1.0f)))), matrix1.m_posit);
	}

	dMatrix twistMatrix (matrix0 * (matrix1 * coneRotation).Inverse());
	dFloat twistAngle = dAtan2 (twistMatrix[1][2], twistMatrix[1][1]);
	if ((m_minTwistAngle == 0.0f) && (m_minTwistAngle == 0.0f)) {
		NewtonUserJointAddAngularRow(m_joint, -twistAngle, &matrix0.m_front[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	} else if (twistAngle > m_maxTwistAngle) {
		NewtonUserJointAddAngularRow(m_joint, m_maxTwistAngle - twistAngle, &matrix0.m_front[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_friction);
	} else if (twistAngle < m_minTwistAngle) {
		NewtonUserJointAddAngularRow(m_joint, m_minTwistAngle - twistAngle, &matrix0.m_front[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_friction);
	} else if (m_friction > 1.0e-3f) {
		dFloat accel = relOmega.DotProduct3(matrix0.m_front) / timestep;
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_front[0]);
		NewtonUserJointSetRowAcceleration(m_joint, -accel);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_friction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_friction);
	}

	if (cosAngleCos <= m_coneAngleCos) {
		dQuaternion rot (lateralDir, m_coneAngle);
		dVector frontDir (rot.RotateVector(coneDir1));
		dFloat errorAngle = dAcos (dClamp (coneDir0.DotProduct3(frontDir), dFloat(-1.0f), dFloat(1.0f)));
		NewtonUserJointAddAngularRow(m_joint, -errorAngle, &lateralDir[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);

		dVector upDir(lateralDir.CrossProduct(frontDir));
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &upDir[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);

	} else if (m_friction > 1.0e-3f) {

		dFloat accel = relOmega.DotProduct3(lateralDir) / timestep;
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &lateralDir[0]);
		NewtonUserJointSetRowAcceleration(m_joint, -accel);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_friction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_friction);

		dVector upDir(lateralDir.CrossProduct(coneDir0));
		accel = relOmega.DotProduct3(upDir) / timestep;
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &upDir[0]);
		NewtonUserJointSetRowAcceleration(m_joint, -accel);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_friction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_friction);
	}
}

