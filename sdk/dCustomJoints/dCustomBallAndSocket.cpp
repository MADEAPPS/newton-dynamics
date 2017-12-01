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


IMPLEMENT_CUSTOM_JOINT(dCustomPointToPoint);
IMPLEMENT_CUSTOM_JOINT(dCustomBallAndSocket);
IMPLEMENT_CUSTOM_JOINT(dCustomLimitBallAndSocket);
IMPLEMENT_CUSTOM_JOINT(dCustomControlledBallAndSocket);


dCustomPointToPoint::dCustomPointToPoint(const dVector& pivotInChildInGlobalSpace, const dVector& pivotInParentInGlobalSpace, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(6, child, parent)
{
	dVector dist(pivotInChildInGlobalSpace - pivotInParentInGlobalSpace);
	m_distance = dSqrt(dist.DotProduct3(dist));

	dMatrix childMatrix(dGetIdentityMatrix());
	dMatrix parentMatrix(dGetIdentityMatrix());

	childMatrix.m_posit = pivotInChildInGlobalSpace;
	parentMatrix.m_posit = pivotInParentInGlobalSpace;
	childMatrix.m_posit.m_w = 1.0f;
	parentMatrix.m_posit.m_w = 1.0f;

	dMatrix dummy;
	CalculateLocalMatrix(childMatrix, m_localMatrix0, dummy);
	CalculateLocalMatrix(parentMatrix, dummy, m_localMatrix1);
}

dCustomPointToPoint::~dCustomPointToPoint()
{
}


void dCustomPointToPoint::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
}

void dCustomPointToPoint::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize (callback, userData);
}

void dCustomPointToPoint::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	dVector p0(matrix0.m_posit);
	dVector p1(matrix1.m_posit);

	dVector dir(p1 - p0);
	dFloat mag2 = dir.DotProduct3(dir);
	if (mag2 < 1.0e-3f) {
		NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_up[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_right[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	} else {
		dir = dir.Scale(1.0f / dSqrt(mag2));
		dMatrix matrix (dGrammSchmidt (dir));
		dFloat x = dSqrt (mag2) - m_distance;

		dVector com0(0.0f);
		dVector com1(0.0f);
		dVector veloc0(0.0f);
		dVector veloc1(0.0f);
		dMatrix body0Matrix;
		dMatrix body1Matrix;

		NewtonBody* const body0 = GetBody0();
		NewtonBody* const body1 = GetBody1();

		NewtonBodyGetCentreOfMass(body0, &com0[0]);
		NewtonBodyGetMatrix(body0, &body0Matrix[0][0]);
		NewtonBodyGetPointVelocity (body0, &p0[0], &veloc0[0]);

		NewtonBodyGetCentreOfMass(body1, &com1[0]);
		NewtonBodyGetMatrix(body1, &body1Matrix[0][0]);
		NewtonBodyGetPointVelocity (body1, &p1[0], &veloc1[0]);

		dFloat v((veloc0 - veloc1).DotProduct3(dir));
		dFloat a = (x - v * timestep) / (timestep * timestep);

		dVector r0 ((p0 - body0Matrix.TransformVector(com0)).CrossProduct(matrix.m_front));
		dVector r1 ((p1 - body1Matrix.TransformVector(com1)).CrossProduct(matrix.m_front));
		dFloat jacobian0[6];
		dFloat jacobian1[6];

		jacobian0[0] = matrix[0][0];
		jacobian0[1] = matrix[0][1];
		jacobian0[2] = matrix[0][2];
		jacobian0[3] = r0[0];
		jacobian0[4] = r0[1];
		jacobian0[5] = r0[2];

		jacobian1[0] = -matrix[0][0];
		jacobian1[1] = -matrix[0][1];
		jacobian1[2] = -matrix[0][2];
		jacobian1[3] = -r1[0];
		jacobian1[4] = -r1[1];
		jacobian1[5] = -r1[2];

//		NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix[1][0]);
//		NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix[2][0]);
		NewtonUserJointAddGeneralRow(m_joint, jacobian0, jacobian1);
		NewtonUserJointSetRowAcceleration(m_joint, a);
	}
}



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


void dCustomBallAndSocket::Debug(dDebugDisplay* const debugDisplay) const
{
	dCustomJoint::Debug(debugDisplay);
}

void dCustomBallAndSocket::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	// Restrict the movement on the pivot point along all three orthonormal directions
	NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_front[0]);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_up[0]);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_right[0]);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
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
		if (dAbs (error) > (0.125f * 3.14159213f / 180.0f) ) {
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



