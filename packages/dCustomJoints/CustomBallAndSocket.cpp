/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// CustomBallAndSocket.cpp: implementation of the CustomBallAndSocket class.
//
//////////////////////////////////////////////////////////////////////
#include "CustomJointLibraryStdAfx.h"
#include "CustomBallAndSocket.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define MIN_JOINT_PIN_LENGTH	50.0f

IMPLEMENT_CUSTON_JOINT(CustomBallAndSocket);
IMPLEMENT_CUSTON_JOINT(CustomLimitBallAndSocket);

CustomBallAndSocket::CustomBallAndSocket(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:CustomJoint(6, child, parent)
{
	CalculateLocalMatrix (pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
}

CustomBallAndSocket::CustomBallAndSocket(const dMatrix& pinAndPivotFrame0, const dMatrix& pinAndPivotFrame1, NewtonBody* const child, NewtonBody* const parent)
	:CustomJoint(6, child, parent)
{
	dMatrix	dummy;
	CalculateLocalMatrix (pinAndPivotFrame0, m_localMatrix0, dummy);
	CalculateLocalMatrix (pinAndPivotFrame1, dummy, m_localMatrix1);
}


CustomBallAndSocket::~CustomBallAndSocket()
{
}

CustomBallAndSocket::CustomBallAndSocket (NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData)
	:CustomJoint(child, parent, callback, userData)
{

}

void CustomBallAndSocket::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	CustomJoint::Serialize (callback, userData);
}


void CustomBallAndSocket::GetInfo (NewtonJointRecord* const info) const
{
	strcpy (info->m_descriptionType, "ballsocket");

	info->m_attachBody_0 = m_body0;
	info->m_attachBody_1 = m_body1;

	info->m_minLinearDof[0] = 0.0f;
	info->m_maxLinearDof[0] = 0.0f;

	info->m_minLinearDof[1] = 0.0f;
	info->m_maxLinearDof[1] = 0.0f;;

	info->m_minLinearDof[2] = 0.0f;
	info->m_maxLinearDof[2] = 0.0f;

	info->m_minAngularDof[0] = -FLT_MAX ;
	info->m_maxAngularDof[0] =  FLT_MAX ;
	info->m_minAngularDof[1] = -FLT_MAX ;
	info->m_maxAngularDof[1] =  FLT_MAX ;
	info->m_minAngularDof[2] = -FLT_MAX ;
	info->m_maxAngularDof[2] =  FLT_MAX ;
	memcpy (info->m_attachmenMatrix_0, &m_localMatrix0, sizeof (dMatrix));
	memcpy (info->m_attachmenMatrix_1, &m_localMatrix1, sizeof (dMatrix));
}


void CustomBallAndSocket::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	// Restrict the movement on the pivot point along all three orthonormal directions
	NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_front[0]);
	NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_up[0]);
	NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_right[0]);
}



CustomLimitBallAndSocket::CustomLimitBallAndSocket(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:CustomBallAndSocket(pinAndPivotFrame, child, parent)
	,m_rotationOffset(dGetIdentityMatrix())
{
	SetConeAngle (0.0f);
	SetTwistAngle (0.0f, 0.0f);
}


CustomLimitBallAndSocket::CustomLimitBallAndSocket(const dMatrix& childPinAndPivotFrame, NewtonBody* const child, const dMatrix& parentPinAndPivotFrame, NewtonBody* const parent)
	:CustomBallAndSocket(childPinAndPivotFrame, child, parent)
	,m_rotationOffset(childPinAndPivotFrame * parentPinAndPivotFrame.Inverse())
{
	SetConeAngle (0.0f);
	SetTwistAngle (0.0f, 0.0f);
	dMatrix matrix;
	CalculateLocalMatrix (parentPinAndPivotFrame, matrix, m_localMatrix1);
}


CustomLimitBallAndSocket::~CustomLimitBallAndSocket()
{
}

CustomLimitBallAndSocket::CustomLimitBallAndSocket (NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData)
	:CustomBallAndSocket (child, parent, callback, userData)
{
	callback (userData, &m_rotationOffset, sizeof (dMatrix));
	callback (userData, &m_minTwistAngle, sizeof (dFloat));
	callback (userData, &m_maxTwistAngle, sizeof (dFloat));
	callback (userData, &m_coneAngleCos, sizeof (dFloat));
	callback (userData, &m_coneAngleSin, sizeof (dFloat));
	callback (userData, &m_coneAngleHalfCos, sizeof (dFloat));
	callback (userData, &m_coneAngleHalfSin, sizeof (dFloat));
}

void CustomLimitBallAndSocket::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	CustomBallAndSocket::Serialize (callback, userData);

	callback (userData, &m_rotationOffset, sizeof (dMatrix));
	callback (userData, &m_minTwistAngle, sizeof (dFloat));
	callback (userData, &m_maxTwistAngle, sizeof (dFloat));
	callback (userData, &m_coneAngleCos, sizeof (dFloat));
	callback (userData, &m_coneAngleSin, sizeof (dFloat));
	callback (userData, &m_coneAngleHalfCos, sizeof (dFloat));
	callback (userData, &m_coneAngleHalfSin, sizeof (dFloat));
}


void CustomLimitBallAndSocket::SetConeAngle (dFloat angle)
{
	m_coneAngle = angle;
	m_coneAngleCos = dCos (angle);
	m_coneAngleSin = dSin (angle);
	m_coneAngleHalfCos = dCos (angle * 0.5f);
	m_coneAngleHalfSin = dSin (angle * 0.5f);
}


void CustomLimitBallAndSocket::SetTwistAngle (dFloat minAngle, dFloat maxAngle)
{
	m_minTwistAngle = minAngle;
	m_maxTwistAngle = maxAngle;
}

dFloat CustomLimitBallAndSocket::GetConeAngle () const
{
	return m_coneAngle;
}

void CustomLimitBallAndSocket::GetTwistAngle (dFloat& minAngle, dFloat& maxAngle) const
{
	minAngle = m_minTwistAngle;
	maxAngle = m_maxTwistAngle;
}


void CustomLimitBallAndSocket::GetInfo (NewtonJointRecord* const info) const
{
	CustomBallAndSocket::GetInfo (info);

	info->m_minAngularDof[0] = m_minTwistAngle;
	info->m_maxAngularDof[0] = m_maxTwistAngle;
//	info->m_minAngularDof[1] = -dAcos (m_coneAngleCos);
//	info->m_maxAngularDof[1] =  dAcos (m_coneAngleCos);
//	info->m_minAngularDof[2] = -dAcos (m_coneAngleCos); 
//	info->m_maxAngularDof[2] =  dAcos (m_coneAngleCos);

	info->m_minAngularDof[1] = -m_coneAngle;
	info->m_maxAngularDof[1] =  m_coneAngle;
	info->m_minAngularDof[2] = -m_coneAngle; 
	info->m_maxAngularDof[2] =  m_coneAngle;

	strcpy (info->m_descriptionType, "limitballsocket");
}

void CustomLimitBallAndSocket::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	const dVector& p0 = matrix0.m_posit;
	const dVector& p1 = matrix1.m_posit;

	// Restrict the movement on the pivot point along all tree orthonormal direction
	NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix1.m_front[0]);
	NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix1.m_up[0]);
	NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix1.m_right[0]);

	matrix1 = m_rotationOffset * matrix1;

	// handle special case of the joint being a hinge
	if (m_coneAngleCos > 0.9999f) {
		dFloat cosAngle;
		dFloat sinAngle;

		CalculateYawAngle(matrix0, matrix1, sinAngle, cosAngle);
		NewtonUserJointAddAngularRow(m_joint, -dAtan2(sinAngle, cosAngle), &matrix1.m_up[0]);

		CalculateRollAngle(matrix0, matrix1, sinAngle, cosAngle);
		NewtonUserJointAddAngularRow(m_joint, -dAtan2(sinAngle, cosAngle), &matrix1.m_right[0]);

		// the joint angle can be determined by getting the angle between any two non parallel vectors
		CalculatePitchAngle(matrix0, matrix1, sinAngle, cosAngle);
		dFloat pitchAngle = -dAtan2(sinAngle, cosAngle);
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
		dFloat cosAngle = coneDir0 % coneDir1;
		if (cosAngle <= m_coneAngleCos) {
			dVector lateralDir (coneDir0 * coneDir1);
			dFloat mag2 = lateralDir % lateralDir;
			dAssert (mag2 > 1.0e-4f);
			lateralDir = lateralDir.Scale (1.0f / dSqrt (mag2));

			dQuaternion rot(m_coneAngleHalfCos, lateralDir.m_x * m_coneAngleHalfSin, lateralDir.m_y * m_coneAngleHalfSin, lateralDir.m_z * m_coneAngleHalfSin);
			dVector frontDir (rot.UnrotateVector(coneDir1));
			dVector upDir (lateralDir * frontDir);
			NewtonUserJointAddAngularRow (m_joint, 0.0f, &upDir[0]);

			dFloat cosAngle = frontDir % coneDir0;
			dFloat sinAngle = ((frontDir * coneDir0) % lateralDir);
			NewtonUserJointAddAngularRow (m_joint, -dAtan2(sinAngle, cosAngle), &lateralDir[0]);
				NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
		}

		//handle twist angle
		dFloat sinAngle;
		CalculatePitchAngle (matrix0, matrix1, sinAngle, cosAngle);
		dFloat pitchAngle = -dAtan2 (sinAngle, cosAngle);
	if ((m_maxTwistAngle - m_minTwistAngle) < 1.0e-4f) {
			NewtonUserJointAddAngularRow(m_joint, pitchAngle, &matrix1.m_front[0]);
	} else {
		if (pitchAngle > m_maxTwistAngle) {
			pitchAngle -= m_maxTwistAngle;
				NewtonUserJointAddAngularRow(m_joint, pitchAngle, &matrix0.m_front[0]);
			NewtonUserJointSetRowMinimumFriction (m_joint, -0.0f);
		} else if (pitchAngle < m_minTwistAngle) {
			pitchAngle -= m_minTwistAngle;
				NewtonUserJointAddAngularRow(m_joint, pitchAngle, &matrix0.m_front[0]);
			NewtonUserJointSetRowMaximumFriction (m_joint,  0.0f);
		}
	}
	}
}


CustomControlledBallAndSocket::CustomControlledBallAndSocket(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:CustomBallAndSocket(pinAndPivotFrame, child, parent)
	,m_targetAngles (0.0f, 0.0f, 0.0f, 0.0f)
	,m_pitch(0.0f)
	,m_yaw(0.0f)
	,m_roll(0.0f)
	,m_angulaSpeed (1.0f)
{
}

CustomControlledBallAndSocket::~CustomControlledBallAndSocket()
{
}


void CustomControlledBallAndSocket::GetInfo (NewtonJointRecord* const info) const
{
	CustomBallAndSocket::GetInfo (info);
	dAssert (0);
//	info->m_minAngularDof[0] = m_minTwistAngle;
//	info->m_maxAngularDof[0] = m_maxTwistAngle;
//	info->m_minAngularDof[1] = -m_coneAngle;
//	info->m_maxAngularDof[1] =  m_coneAngle;
//	info->m_minAngularDof[2] = -m_coneAngle; 
//	info->m_maxAngularDof[2] =  m_coneAngle;

	strcpy (info->m_descriptionType, "controlledballsocket");
}


void CustomControlledBallAndSocket::SetAngularVelocity (dFloat omegaMag)
{
	m_angulaSpeed = dAbs (omegaMag);
}

dFloat CustomControlledBallAndSocket::GetAngularVelocity () const
{
	return m_angulaSpeed;
}

void CustomControlledBallAndSocket::SetPitchAngle (dFloat angle)
{
	m_targetAngles[0] = angle;
}

dFloat CustomControlledBallAndSocket::SetPitchAngle () const
{
	return m_targetAngles[0];
}

void CustomControlledBallAndSocket::SetYawAngle (dFloat angle)
{
	m_targetAngles[1] = angle;
}

dFloat CustomControlledBallAndSocket::SetYawAngle () const
{
	return m_targetAngles[1];
}

void CustomControlledBallAndSocket::SetRollAngle (dFloat angle)
{
	m_targetAngles[2] = angle;
}

dFloat CustomControlledBallAndSocket::SetRollAngle () const
{
	return m_targetAngles[2];
}


void CustomControlledBallAndSocket::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	const dVector& p0 = matrix0.m_posit;
	const dVector& p1 = matrix1.m_posit;

	// Restrict the movement on the pivot point along all tree orthonormal direction
	NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix1.m_front[0]);
	NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix1.m_up[0]);
	NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix1.m_right[0]);

	dVector euler0;
	dVector euler1;
	dMatrix localMatrix (matrix0 * matrix1.Inverse());
	localMatrix.GetEulerAngles(euler0, euler1);

	AngularIntegration pitchStep0 (AngularIntegration (euler0.m_x) - m_pitch);
	AngularIntegration pitchStep1 (AngularIntegration (euler1.m_x) - m_pitch);
	if (dAbs (pitchStep0.GetAngle()) > dAbs (pitchStep1.GetAngle())) {
		euler0 = euler1;
	}

	dVector euler (m_pitch.Update (euler0.m_x), m_yaw.Update (euler0.m_y), m_roll.Update (euler0.m_z), 0.0f);
	//dTrace (("(%f %f %f) (%f %f %f)\n", m_pitch.m_angle * 180.0f / 3.141592f, m_yaw.m_angle * 180.0f / 3.141592f, m_roll.m_angle * 180.0f / 3.141592f,  euler0.m_x * 180.0f / 3.141592f, euler0.m_y * 180.0f / 3.141592f, euler0.m_z * 180.0f / 3.141592f));

	//bool limitViolation = false;
	for (int i = 0; i < 3; i ++) {
		dFloat error = m_targetAngles[i] - euler[i];
		if (dAbs (error) > (0.125f * 3.14159213f / 180.0f) ) {
			//limitViolation = true;
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


	//dMatrix pyr (dPitchMatrix(m_pitch.m_angle) * dYawMatrix(m_yaw.m_angle) * dRollMatrix(m_roll.m_angle));
	dMatrix p0y0r0 (dPitchMatrix(euler[0]) * dYawMatrix(euler[1]) * dRollMatrix(euler[2]));
	dMatrix baseMatrix (p0y0r0 * matrix1);
	dMatrix rotation (matrix0.Inverse() * baseMatrix);

	dQuaternion quat (rotation);
	if (quat.m_q0 > dFloat (0.99995f)) {
		dVector p0 (matrix0[3] + matrix0[0].Scale (MIN_JOINT_PIN_LENGTH));
		dVector p1 (matrix1[3] + baseMatrix[0].Scale (MIN_JOINT_PIN_LENGTH));
		NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &baseMatrix[1][0]);
		NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &baseMatrix[2][0]);

		dVector q0 (matrix0[3] + matrix0[1].Scale (MIN_JOINT_PIN_LENGTH));
		dVector q1 (matrix0[3] + baseMatrix[1].Scale (MIN_JOINT_PIN_LENGTH));
		NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &baseMatrix[2][0]);

	} else {
		dMatrix basis (dGrammSchmidt (dVector (quat.m_q1, quat.m_q2, quat.m_q3, 0.0f)));

		dVector p0 (matrix1[3] + basis[1].Scale (MIN_JOINT_PIN_LENGTH));
		dVector p1 (matrix1[3] + rotation.RotateVector(basis[1].Scale (MIN_JOINT_PIN_LENGTH)));
		NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &basis[2][0]);
		NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);

		dVector q0 (matrix1[3] + basis[0].Scale (MIN_JOINT_PIN_LENGTH));
		NewtonUserJointAddLinearRow (m_joint, &q0[0], &q0[0], &basis[1][0]);
		NewtonUserJointAddLinearRow (m_joint, &q0[0], &q0[0], &basis[2][0]);
	}
}
