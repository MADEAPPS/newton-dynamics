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

#if 0
IMPLEMENT_CUSTOM_JOINT(dCustomLimitBallAndSocket);
IMPLEMENT_CUSTOM_JOINT(dCustomControlledBallAndSocket);

dCustomBallAndSocket::dCustomBallAndSocket(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(6, child, parent)
{
	CalculateLocalMatrix (pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
}

dCustomBallAndSocket::dCustomBallAndSocket(const dMatrix& pinAndPivotFrame0, const dMatrix& pinAndPivotFrame1, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(6, child, parent)
{
	dMatrix	dummy;
	CalculateLocalMatrix (pinAndPivotFrame0, m_localMatrix0, dummy);
	CalculateLocalMatrix (pinAndPivotFrame1, dummy, m_localMatrix1);
}

dCustomBallAndSocket::~dCustomBallAndSocket()
{
}

void dCustomBallAndSocket::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
}

void dCustomBallAndSocket::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize (callback, userData);
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

dCustomBallAndSocketWithFriction::dCustomBallAndSocketWithFriction(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent, dFloat dryFriction)
	:dCustomBallAndSocket(pinAndPivotFrame, child, parent)
	,m_dryFriction(dryFriction)
{
}

void dCustomBallAndSocketWithFriction::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dVector omega0(0.0f);
	dVector omega1(0.0f);

	// get the omega vector
	NewtonBodyGetOmega(m_body0, &omega0[0]);
	if (m_body1) {
		NewtonBodyGetOmega(m_body1, &omega1[0]);
	}

	dCustomBallAndSocket::SubmitConstraints(timestep, threadIndex);

	dVector relOmega(omega0 - omega1);
	dFloat omegaMag = dSqrt(relOmega.DotProduct3(relOmega));
	if (omegaMag > 0.1f) {
		// tell newton to used this the friction of the omega vector to apply the rolling friction
		dMatrix basis(dGrammSchmidt(relOmega));

		NewtonUserJointAddAngularRow(m_joint, 0.0f, &basis[2][0]);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_dryFriction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_dryFriction);

		NewtonUserJointAddAngularRow(m_joint, 0.0f, &basis[1][0]);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_dryFriction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_dryFriction);

		// calculate the acceleration to stop the ball in one time step
		dFloat invTimestep = (timestep > 0.0f) ? 1.0f / timestep : 1.0f;
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &basis[0][0]);
		NewtonUserJointSetRowAcceleration(m_joint, -omegaMag * invTimestep);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_dryFriction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_dryFriction);
	} else {
		// when omega is too low this is correct but the small angle approximation theorem.
		dMatrix basis(dGetIdentityMatrix());
		for (int i = 0; i < 3; i++) {
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &basis[i][0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_dryFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_dryFriction);
		}
	}
}


dCustomLimitBallAndSocket::dCustomLimitBallAndSocket(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomBallAndSocket(pinAndPivotFrame, child, parent)
	,m_rotationOffset(dGetIdentityMatrix())
{
	SetConeAngle (0.0f);
	SetTwistAngle (0.0f, 0.0f);
}


dCustomLimitBallAndSocket::dCustomLimitBallAndSocket(const dMatrix& childPinAndPivotFrame, NewtonBody* const child, const dMatrix& parentPinAndPivotFrame, NewtonBody* const parent)
	:dCustomBallAndSocket(childPinAndPivotFrame, child, parent)
	,m_rotationOffset(childPinAndPivotFrame * parentPinAndPivotFrame.Inverse())
{
	SetConeAngle (0.0f);
	SetTwistAngle (0.0f, 0.0f);
	dMatrix matrix;
	CalculateLocalMatrix (parentPinAndPivotFrame, matrix, m_localMatrix1);
}


dCustomLimitBallAndSocket::~dCustomLimitBallAndSocket()
{
}

void dCustomLimitBallAndSocket::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	callback (userData, &m_rotationOffset, sizeof (dMatrix));
	callback (userData, &m_minTwistAngle, sizeof (dFloat));
	callback (userData, &m_maxTwistAngle, sizeof (dFloat));
	callback (userData, &m_coneAngleCos, sizeof (dFloat));
	callback (userData, &m_coneAngleSin, sizeof (dFloat));
	callback (userData, &m_coneAngleHalfCos, sizeof (dFloat));
	callback (userData, &m_coneAngleHalfSin, sizeof (dFloat));
}

void dCustomLimitBallAndSocket::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	dCustomBallAndSocket::Serialize (callback, userData);

	callback (userData, &m_rotationOffset, sizeof (dMatrix));
	callback (userData, &m_minTwistAngle, sizeof (dFloat));
	callback (userData, &m_maxTwistAngle, sizeof (dFloat));
	callback (userData, &m_coneAngleCos, sizeof (dFloat));
	callback (userData, &m_coneAngleSin, sizeof (dFloat));
	callback (userData, &m_coneAngleHalfCos, sizeof (dFloat));
	callback (userData, &m_coneAngleHalfSin, sizeof (dFloat));
}


void dCustomLimitBallAndSocket::SetConeAngle (dFloat angle)
{
	m_coneAngle = angle;
	m_coneAngleCos = dCos (angle);
	m_coneAngleSin = dSin (angle);
	m_coneAngleHalfCos = dCos (angle * 0.5f);
	m_coneAngleHalfSin = dSin (angle * 0.5f);
}


void dCustomLimitBallAndSocket::SetTwistAngle (dFloat minAngle, dFloat maxAngle)
{
	m_minTwistAngle = minAngle;
	m_maxTwistAngle = maxAngle;
}

dFloat dCustomLimitBallAndSocket::GetConeAngle () const
{
	return m_coneAngle;
}

void dCustomLimitBallAndSocket::GetTwistAngle (dFloat& minAngle, dFloat& maxAngle) const
{
	minAngle = m_minTwistAngle;
	maxAngle = m_maxTwistAngle;
}



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


dCustomControlledBallAndSocket::dCustomControlledBallAndSocket(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomBallAndSocket(pinAndPivotFrame, child, parent)
	,m_targetAngles (0.0f)
	,m_pitch(0.0f)
	,m_yaw(0.0f)
	,m_roll(0.0f)
	,m_angulaSpeed (1.0f)
{
}

void dCustomControlledBallAndSocket::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	dAssert (0);
}

dCustomControlledBallAndSocket::~dCustomControlledBallAndSocket()
{
}

void dCustomControlledBallAndSocket::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	dCustomBallAndSocket::Serialize (callback, userData);
	dAssert (0);
}


void dCustomControlledBallAndSocket::SetAngularVelocity (dFloat omegaMag)
{
	m_angulaSpeed = dAbs (omegaMag);
}

dFloat dCustomControlledBallAndSocket::GetAngularVelocity () const
{
	return m_angulaSpeed;
}

void dCustomControlledBallAndSocket::SetPitchAngle (dFloat angle)
{
	m_targetAngles[0] = angle;
	UpdateTargetMatrix ();
}

dFloat dCustomControlledBallAndSocket::SetPitchAngle () const
{
	return m_targetAngles[0];
}

void dCustomControlledBallAndSocket::SetYawAngle (dFloat angle)
{
	m_targetAngles[1] = angle;
	UpdateTargetMatrix ();
}

dFloat dCustomControlledBallAndSocket::SetYawAngle () const
{
	return m_targetAngles[1];
}

void dCustomControlledBallAndSocket::SetRollAngle (dFloat angle)
{
	m_targetAngles[2] = angle;
	UpdateTargetMatrix ();
}

dFloat dCustomControlledBallAndSocket::SetRollAngle () const
{
	return m_targetAngles[2];
}

void dCustomControlledBallAndSocket::UpdateTargetMatrix ()
{
	m_targetRotation = dPitchMatrix(m_targetAngles[0]) * dYawMatrix(m_targetAngles[1]) * dRollMatrix(m_targetAngles[2]);
}

void dCustomControlledBallAndSocket::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix (matrix0, matrix1);

	dCustomBallAndSocket::SubmitConstraints(timestep, threadIndex);

#if 0
	dVector euler0(0.0f);
	dVector euler1(0.0f);
	dMatrix localMatrix (matrix0 * matrix1.Inverse());
	localMatrix.GetEulerAngles(euler0, euler1);

	AngularIntegration pitchStep0 (AngularIntegration (euler0.m_x) - m_pitch);
	AngularIntegration pitchStep1 (AngularIntegration (euler1.m_x) - m_pitch);
	if (dAbs (pitchStep0.GetAngle()) > dAbs (pitchStep1.GetAngle())) {
		euler0 = euler1;
	}
	dVector euler (m_pitch.Update (euler0.m_x), m_yaw.Update (euler0.m_y), m_roll.Update (euler0.m_z), 0.0f);
	for (int i = 0; i < 3; i ++) {
		dFloat error = m_targetAngles[i] - euler[i];
		if (dAbs (error) > (0.125f * dDegreeToRad) ) {
			dFloat angularStep = dSign(error) * m_angulaSpeed * timestep;
			if (angularStep > 0.0f) {
				if (angularStep > error) {
					angularStep = error * 0.5f;
				}
			} else {
				if (angularStep < error) {
					angularStep = error * 0.5f;
				}
			}
			euler[i] = euler[i] + angularStep;
		}
	}
	
	dMatrix p0y0r0 (dPitchMatrix(euler[0]) * dYawMatrix(euler[1]) * dRollMatrix(euler[2]));
	dMatrix baseMatrix (p0y0r0 * matrix1);
	dMatrix rotation (matrix0.Inverse() * baseMatrix);

	dQuaternion quat (rotation);
	if (quat.m_q0 > dFloat (0.99995f)) {
		dVector euler0;
		dVector euler1;
		rotation.GetEulerAngles(euler0, euler1);
		NewtonUserJointAddAngularRow(m_joint, euler0[0], &rotation[0][0]);
		NewtonUserJointAddAngularRow(m_joint, euler0[1], &rotation[1][0]);
		NewtonUserJointAddAngularRow(m_joint, euler0[2], &rotation[2][0]);
	} else {
		dMatrix basis (dGrammSchmidt (dVector (quat.m_q1, quat.m_q2, quat.m_q3, 0.0f)));
		NewtonUserJointAddAngularRow (m_joint, 2.0f * dAcos (quat.m_q0), &basis[0][0]);
		NewtonUserJointAddAngularRow (m_joint, 0.0f, &basis[1][0]); 
		NewtonUserJointAddAngularRow (m_joint, 0.0f, &basis[2][0]); 
	}
#else

	matrix1 = m_targetRotation * matrix1;

	dQuaternion localRotation(matrix1 * matrix0.Inverse());
	if (localRotation.DotProduct(m_targetRotation) < 0.0f) {
		localRotation.Scale(-1.0f);
	}

	dFloat angle = 2.0f * dAcos(localRotation.m_q0);
	dFloat angleStep = m_angulaSpeed * timestep;
	if (angleStep < angle) {
		dVector axis(dVector(localRotation.m_q1, localRotation.m_q2, localRotation.m_q3, 0.0f));
		axis = axis.Scale(1.0f / dSqrt(axis.DotProduct3(axis)));
//		localRotation = dQuaternion(axis, angleStep);
	}

	dVector axis (matrix1.m_front.CrossProduct(matrix1.m_front));
	dVector axis1 (matrix1.m_front.CrossProduct(matrix1.m_front));
//dFloat sinAngle;
//dFloat cosAngle;
//CalculatePitchAngle (matrix0, matrix1, sinAngle, cosAngle);
//dFloat xxxx = dAtan2(sinAngle, cosAngle);
//dFloat xxxx1 = dAtan2(sinAngle, cosAngle);

	dQuaternion quat(localRotation);
	if (quat.m_q0 > dFloat(0.99995f)) {
//		dAssert (0);
/*
		dVector euler0(0.0f);
		dVector euler1(0.0f);
		rotation.GetEulerAngles(euler0, euler1);
		NewtonUserJointAddAngularRow(m_joint, euler0[0], &rotation[0][0]);
		NewtonUserJointAddAngularRow(m_joint, euler0[1], &rotation[1][0]);
		NewtonUserJointAddAngularRow(m_joint, euler0[2], &rotation[2][0]);
*/
	} else {
		dMatrix basis(dGrammSchmidt(dVector(quat.m_q1, quat.m_q2, quat.m_q3, 0.0f)));
		NewtonUserJointAddAngularRow(m_joint, -2.0f * dAcos(quat.m_q0), &basis[0][0]);
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &basis[1][0]);
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &basis[2][0]);
	}

#endif
}

#endif

#if 0



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


#endif




dCustomBallAndSocket::dCustomBallAndSocket(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(6, child, parent)
	,m_twistAngle(0.0f)
	,m_minTwistAngle(-180.0f * dDegreeToRad)
	,m_maxTwistAngle(180.0f * dDegreeToRad)
	,m_maxConeAngle(60.0f * dDegreeToRad)
	,m_coneFriction(0.0f)
	,m_twistFriction(0.0f)
	,m_options()
{
	CalculateLocalMatrix(pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
}

dCustomBallAndSocket::dCustomBallAndSocket(const dMatrix& pinAndPivotFrame0, const dMatrix& pinAndPivotFrame1, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(6, child, parent)
	,m_twistAngle(0.0f)
	,m_minTwistAngle(-180.0f * dDegreeToRad)
	,m_maxTwistAngle(180.0f * dDegreeToRad)
	,m_maxConeAngle(60.0f * dDegreeToRad)
	,m_coneFriction(0.0f)
	,m_twistFriction(0.0f)
	,m_options()
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
	callback(userData, &m_options, sizeof(m_options));
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
	callback(userData, &m_options, sizeof(m_options));
}

void dCustomBallAndSocket::EnableTwist(bool state)
{
	m_options.m_option0 = state;
}

void dCustomBallAndSocket::EnableCone(bool state)
{
	m_options.m_option1 = state;
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
	m_maxConeAngle = dMin (dAbs (maxAngle), dFloat (160.0f * dDegreeToRad));
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

	const dVector& coneDir0 = matrix0.m_front;
	const dVector& coneDir1 = matrix1.m_front;
	dFloat cosAngleCos = coneDir0.DotProduct3(coneDir1);
	dMatrix coneRotation(dGetIdentityMatrix());
	if (cosAngleCos < 0.9999f) {
		dVector lateralDir(coneDir1.CrossProduct(coneDir0));
		dFloat mag2 = lateralDir.DotProduct3(lateralDir);
		dAssert(mag2 > 1.0e-4f);
		//lateralDir = lateralDir.Scale(1.0f / dSqrt(mag2));
		//coneRotation = dMatrix(dQuaternion(lateralDir, dAcos(dClamp(cosAngleCos, dFloat(-1.0f), dFloat(1.0f)))), matrix1.m_posit);
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
	const float radius = debugDisplay->m_debugScale;
	dVector arch[subdiv + 1];

	// show pitch angle limits
	if (m_options.m_option0 && ((m_maxTwistAngle - m_minTwistAngle) > dFloat(1.0e-3f))) {
		dMatrix pitchMatrix(matrix1 * coneRotation);
		pitchMatrix.m_posit = matrix1.m_posit;

		dVector point(dFloat(0.0f), dFloat(radius), dFloat(0.0f), dFloat(0.0f));

		dFloat angleStep = dMin (m_maxTwistAngle - m_minTwistAngle, dFloat (2.0f * dPi)) / subdiv;
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

	if (m_options.m_option2) {
		dVector point(radius * dCos(m_maxConeAngle), radius * dSin(m_maxConeAngle), 0.0f, 0.0f);
		dFloat angleStep = dPi * 2.0f / subdiv;
		dFloat angle0 = 0.0f;
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
		const dFloat stopAccel = NewtonUserJointCalculateRowZeroAccelaration(m_joint) + speed * invtimestep;
		NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
	} else if (twistAngle > m_maxTwistAngle) {
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_front[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_twistFriction);

		const dFloat invtimestep = 1.0f / timestep;
		const dFloat speed = 0.5f * (m_maxTwistAngle - m_twistAngle.GetAngle()) * invtimestep;
		const dFloat stopAccel = NewtonUserJointCalculateRowZeroAccelaration(m_joint) + speed * invtimestep;
		NewtonUserJointSetRowAcceleration(m_joint, stopAccel);

	} else if (m_twistFriction > 0.0f) {
		NewtonUserJointAddAngularRow(m_joint, 0, &matrix0.m_front[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);

		dFloat accel = NewtonUserJointCalculateRowZeroAccelaration(m_joint);
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

	const dVector& p0 = matrix0.m_posit;
	const dVector& p1 = matrix1.m_posit;
	for (int i = 0; i < 3; i++) {
		NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1[i][0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	}

	const dVector& coneDir0 = matrix0.m_front;
	const dVector& coneDir1 = matrix1.m_front;

	dFloat cosAngleCos = coneDir1.DotProduct3(coneDir0);
	dMatrix coneRotation(dGetIdentityMatrix());
	dVector lateralDir(matrix0.m_up);

	if (cosAngleCos < 0.9999f) {
		lateralDir = coneDir1.CrossProduct(coneDir0);
		dFloat mag2 = lateralDir.DotProduct3(lateralDir);
		if (mag2 > 1.0e-4f) {
			lateralDir = lateralDir.Scale(1.0f / dSqrt(mag2));
			coneRotation = dMatrix(dQuaternion(lateralDir, dAcos(dClamp(cosAngleCos, dFloat(-1.0f), dFloat(1.0f)))), matrix1.m_posit);
		} else {
			lateralDir = matrix0.m_up.Scale (-1.0f);
			coneRotation = dMatrix(dQuaternion(matrix0.m_up, 180 * dDegreeToRad), matrix1.m_posit);
		}
	}

	dVector omega0(0.0f);
	dVector omega1(0.0f);
	NewtonBodyGetOmega(m_body0, &omega0[0]);
	if (m_body1) {
		NewtonBodyGetOmega(m_body1, &omega1[0]);
	}
	dVector relOmega(omega0 - omega1);

	// do twist angle calculations
	dMatrix twistMatrix(matrix0 * (matrix1 * coneRotation).Inverse());
	dFloat twistAngle = m_twistAngle.Update(dAtan2(twistMatrix[1][2], twistMatrix[1][1]));
	if (m_options.m_option0) {
		if ((m_minTwistAngle == 0.0f) && (m_minTwistAngle == 0.0f)) {
			NewtonUserJointAddAngularRow(m_joint, -twistAngle, &matrix0.m_front[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		} else {
			if (m_options.m_option1) {
				dAssert (0);
			} else {
				SubmitConstraintTwistLimits(matrix0, matrix1, relOmega, timestep);
			}
		}
	} else if (m_options.m_option1) {
		dAssert (0);
	} else if (m_twistFriction > 0.0f) {
		NewtonUserJointAddAngularRow(m_joint, 0, &matrix0.m_front[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);

		NewtonUserJointSetRowAcceleration(m_joint, NewtonUserJointCalculateRowZeroAccelaration(m_joint));
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_twistFriction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_twistFriction);
	}

	// do twist cone angle calculations
	if (m_options.m_option2) {
/*
		if ((m_minTwistAngle == 0.0f) && (m_minTwistAngle == 0.0f)) {
			NewtonUserJointAddAngularRow(m_joint, -twistAngle, &matrix0.m_front[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		} else {
			if (m_options.m_option1) {
				dAssert(0);
			}
			else {
				SubmitConstraintTwistLimits(matrix0, matrix1, relOmega, timestep);
			}
		}
	} else if (m_options.m_option1) {
		dAssert(0);
*/
	} else if (m_coneFriction > 0.0f) {
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &lateralDir[0]);
		NewtonUserJointSetRowAcceleration(m_joint, NewtonUserJointCalculateRowZeroAccelaration(m_joint));
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_coneFriction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);

		dVector upDir(lateralDir.CrossProduct(coneDir0));
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &upDir[0]);
		NewtonUserJointSetRowAcceleration(m_joint, NewtonUserJointCalculateRowZeroAccelaration(m_joint));
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_coneFriction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);
	}
}
