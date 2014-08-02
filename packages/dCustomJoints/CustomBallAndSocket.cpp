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

//dInitRtti(CustomBallAndSocket);
//dInitRtti(CustomLimitBallAndSocket);

CustomBallAndSocket::CustomBallAndSocket(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:CustomJoint(6, child, parent)
{
	CalculateLocalMatrix (pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
}



CustomBallAndSocket::~CustomBallAndSocket()
{
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

	// calculate the position of the pivot point and the jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (m_localMatrix0, m_localMatrix1, matrix0, matrix1);

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
//	,m_pitch()
//	,m_yaw()
//	,m_roll()
{
	SetConeAngle (0.0f);
	SetTwistAngle (0.0f, 0.0f);
	dMatrix matrix;
	CalculateLocalMatrix (parentPinAndPivotFrame, matrix, m_localMatrix1);
}


CustomLimitBallAndSocket::~CustomLimitBallAndSocket()
{
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
	CalculateGlobalMatrix (m_localMatrix0, m_localMatrix1, matrix0, matrix1);

	const dVector& p0 = matrix0.m_posit;
	const dVector& p1 = matrix1.m_posit;

	// Restrict the movement on the pivot point along all tree orthonormal direction
	NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix1.m_front[0]);
	NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix1.m_up[0]);
	NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix1.m_right[0]);

	matrix1 = m_rotationOffset * matrix1;

/*
	dVector euler0;
	dVector euler1;
	dMatrix localMatrix (matrix0 * matrix1.Inverse());
	localMatrix.GetEulerAngles(euler0, euler1);

	AngularIntegration pitchStep0 (AngularIntegration (euler0.m_x) - m_pitch);
	AngularIntegration pitchStep1 (AngularIntegration (euler1.m_x) - m_pitch);
	if (dAbs (pitchStep0.m_angle) > dAbs (pitchStep1.m_angle)) {
		euler0 = euler1;
	}

	dVector euler (m_pitch.Update (euler0.m_x), m_yaw.Update (euler0.m_y), m_roll.Update (euler0.m_z), 0.0f);
//dTrace (("(%f %f %f) (%f %f %f)\n", m_pitch.m_angle * 180.0f / 3.141592f, m_yaw.m_angle * 180.0f / 3.141592f, m_roll.m_angle * 180.0f / 3.141592f,  euler0.m_x * 180.0f / 3.141592f, euler0.m_y * 180.0f / 3.141592f, euler0.m_z * 180.0f / 3.141592f));

	// handle special case of cone angle being zero
	if (m_coneAngle == 0.0f) {
		dVector p0 (matrix0[3] + matrix0[0].Scale (MIN_JOINT_PIN_LENGTH));
		dVector p1 (matrix0[3] + matrix1[0].Scale (MIN_JOINT_PIN_LENGTH));
		NewtonUserJointAddLinearRow (m_joint, &p0[0], &p0[0], &matrix1[1][0]);
		NewtonUserJointAddLinearRow (m_joint, &p0[0], &p0[0], &matrix1[2][0]);

		if ((m_maxTwistAngle - m_minTwistAngle) < 1.0e-4f) {
			// handle the cone angle zero as special case of twist angle being zero
			dAssert (0);
			dVector q0 (matrix0[3] + matrix0[1].Scale (MIN_JOINT_PIN_LENGTH));
			dVector q1 (matrix0[3] + matrix1[1].Scale (MIN_JOINT_PIN_LENGTH));
			NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &matrix1[2][0]);
		} else {
			if (euler[0] < m_minTwistAngle) {
				euler[0] = m_minTwistAngle;
				dMatrix pyr (dPitchMatrix(m_pitch.m_angle) * dYawMatrix(m_yaw.m_angle) * dRollMatrix(m_roll.m_angle));
				dMatrix p0y0r0 (dPitchMatrix(euler[0]) * dYawMatrix(euler[1]) * dRollMatrix(euler[2]));
				dMatrix baseMatrix0 (p0y0r0.Inverse() * matrix0);
				dVector q0 (matrix0[3] + baseMatrix0[1].Scale (MIN_JOINT_PIN_LENGTH));
				dVector q1 (matrix0[3] + matrix1[1].Scale (MIN_JOINT_PIN_LENGTH));
				NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &matrix1[2][0]);
				NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);

			} else if (euler[0] > m_maxTwistAngle) {
				euler[0] = m_maxTwistAngle;
				dMatrix pyr (dPitchMatrix(m_pitch.m_angle) * dYawMatrix(m_yaw.m_angle) * dRollMatrix(m_roll.m_angle));
				dMatrix p0y0r0 (dPitchMatrix(euler[0]) * dYawMatrix(euler[1]) * dRollMatrix(euler[2]));
				dMatrix baseMatrix0 (p0y0r0.Inverse() * matrix0);
				dVector q0 (matrix0[3] + baseMatrix0[1].Scale (MIN_JOINT_PIN_LENGTH));
				dVector q1 (matrix0[3] + matrix1[1].Scale (MIN_JOINT_PIN_LENGTH));
				NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &matrix1[2][0]);
				NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
			}
		}
	} else {
		bool limitViolation = false;
		if (euler[0] < m_minTwistAngle) {
			limitViolation = true;
			euler[0] = m_minTwistAngle;
		} else if (euler[0] > m_maxTwistAngle) {
			limitViolation = true;
			euler[0] = m_maxTwistAngle;
		}

		dMatrix coneMatrix (dYawMatrix(euler[1]) * dRollMatrix(euler[2]));
		dFloat coneAngle = dAcos (coneMatrix[0][0]);
		if (coneAngle > m_coneAngle) {
			limitViolation = true;
			dFloat angle = dAtan2 (coneMatrix[0][2], coneMatrix[0][1]);
			dMatrix alignMatrix (dPitchMatrix(angle));
			coneMatrix = coneMatrix * alignMatrix.Inverse();
			dAssert (dAbs (coneMatrix[0][2]) < 1.0e-4f);
			dAssert (dAbs(dAtan2 (coneMatrix[0][1], coneMatrix[0][0]) - coneAngle) < 1.0e-4f);
			dFloat deltaAngle = coneAngle - m_coneAngle;
			dMatrix clipMatrix (dRollMatrix(-deltaAngle));
			coneMatrix = coneMatrix * clipMatrix;
			coneMatrix = dPitchMatrix (euler[0]) * coneMatrix * alignMatrix;
		}

		if (limitViolation) {
			//dMatrix p0y0r0 (dPitchMatrix(euler[0]) * dYawMatrix(euler[1]) * dRollMatrix(euler[2]));
			//dMatrix rotation (pyr * p0y0r0.Inverse());
			dMatrix baseMatrix (coneMatrix * matrix1);
			dMatrix rotation (matrix0.Inverse() * baseMatrix);

			dQuaternion quat (rotation);
			if (quat.m_q0 > dFloat (0.99995f)) {
				//dVector p0 (matrix0[3] + baseMatrix[1].Scale (MIN_JOINT_PIN_LENGTH));
				//dVector p1 (matrix0[3] + baseMatrix[1].Scale (MIN_JOINT_PIN_LENGTH));
				//NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &baseMatrix[2][0]);
				//NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);

				//dVector q0 (matrix0[3] + baseMatrix[0].Scale (MIN_JOINT_PIN_LENGTH));
				//NewtonUserJointAddLinearRow (m_joint, &q0[0], &q0[0], &baseMatrix[1][0]);
				//NewtonUserJointAddLinearRow (m_joint, &q0[0], &q0[0], &baseMatrix[2][0]);

			} else {
				dMatrix basis (dGrammSchmidt (dVector (quat.m_q1, quat.m_q2, quat.m_q3, 0.0f)));

				dVector p0 (matrix0[3] + basis[1].Scale (MIN_JOINT_PIN_LENGTH));
				dVector p1 (matrix0[3] + rotation.RotateVector(basis[1].Scale (MIN_JOINT_PIN_LENGTH)));
				NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &basis[2][0]);
				NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);

				dVector q0 (matrix0[3] + basis[0].Scale (MIN_JOINT_PIN_LENGTH));
				NewtonUserJointAddLinearRow (m_joint, &q0[0], &q0[0], &basis[1][0]);
				NewtonUserJointAddLinearRow (m_joint, &q0[0], &q0[0], &basis[2][0]);
			}
		}
	}
*/

	dMatrix localMatrix (matrix0 * matrix1.Inverse());
	dFloat pitchAngle = -dAtan2(localMatrix[1][2], localMatrix[2][2]);

	if ((m_maxTwistAngle - m_minTwistAngle) < 1.0e-4f) {

		dMatrix base (dPitchMatrix(pitchAngle) * matrix0);
		dVector q0 (p1 + matrix0.m_up.Scale(MIN_JOINT_PIN_LENGTH));
		dVector q1 (p1 + base.m_up.Scale(MIN_JOINT_PIN_LENGTH));

		NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &base.m_right[0]);
	} else {
		if (pitchAngle > m_maxTwistAngle) {
			pitchAngle -= m_maxTwistAngle;
			dMatrix base (dPitchMatrix(pitchAngle) * matrix0);
			dVector q0 (p1 + matrix0.m_up.Scale(MIN_JOINT_PIN_LENGTH));
			dVector q1 (p1 + base.m_up.Scale(MIN_JOINT_PIN_LENGTH));
			NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &base.m_right[0]);
			NewtonUserJointSetRowMinimumFriction (m_joint, -0.0f);
		} else if (pitchAngle < m_minTwistAngle) {
			pitchAngle -= m_minTwistAngle;
			dMatrix base (dPitchMatrix(pitchAngle) * matrix0);
			dVector q0 (p1 + matrix0.m_up.Scale(MIN_JOINT_PIN_LENGTH));
			dVector q1 (p1 + base.m_up.Scale(MIN_JOINT_PIN_LENGTH));
			NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &base.m_right[0]);
			NewtonUserJointSetRowMaximumFriction (m_joint,  0.0f);
		}
	}


	const dVector& coneDir0 = matrix0.m_front;
	const dVector& coneDir1 = matrix1.m_front;
	dVector r0 (p0 + coneDir0.Scale(MIN_JOINT_PIN_LENGTH));
	dVector r1 (p1 + coneDir1.Scale(MIN_JOINT_PIN_LENGTH));

	// construct an orthogonal coordinate system with these two vectors
	dVector lateralDir (coneDir0 * coneDir1);
	dFloat mag2;
	mag2 = lateralDir % lateralDir;
	if (dAbs (mag2) <  1.0e-4f) {
		if (m_coneAngleSin < 1.0e-4f) {
			NewtonUserJointAddLinearRow (m_joint, &r0[0], &r1[0], &matrix0.m_up[0]);
			NewtonUserJointAddLinearRow (m_joint, &r0[0], &r1[0], &matrix0.m_right[0]);
		}
	} else {
		dFloat cosAngle;
		cosAngle = coneDir0 % coneDir1;
		if (cosAngle < m_coneAngleCos) {
			lateralDir = lateralDir.Scale (1.0f / dSqrt (mag2));
			dQuaternion rot (m_coneAngleHalfCos, lateralDir.m_x * m_coneAngleHalfSin, lateralDir.m_y * m_coneAngleHalfSin, lateralDir.m_z * m_coneAngleHalfSin);
			r1 = p1 + rot.UnrotateVector (r1 - p1);

			NewtonUserJointAddLinearRow (m_joint, &r0[0], &r1[0], &lateralDir[0]);

			dVector longitudinalDir (lateralDir * matrix0.m_front);
			NewtonUserJointAddLinearRow (m_joint, &r0[0], &r1[0], &longitudinalDir[0]);
			NewtonUserJointSetRowMinimumFriction (m_joint, -0.0f);
		}
	}
}

