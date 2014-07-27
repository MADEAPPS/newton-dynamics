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
	,m_rotationOffset(GetIdentityMatrix())
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


void CustomLimitBallAndSocket::SetConeAngle (dFloat angle)
{
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


void CustomLimitBallAndSocket::GetInfo (NewtonJointRecord* const info) const
{
	CustomBallAndSocket::GetInfo (info);

	info->m_minAngularDof[0] = m_minTwistAngle;
	info->m_maxAngularDof[0] = m_maxTwistAngle;
	info->m_minAngularDof[1] = -dAcos (m_coneAngleCos);
	info->m_maxAngularDof[1] =  dAcos (m_coneAngleCos);
	info->m_minAngularDof[2] = -dAcos (m_coneAngleCos); 
	info->m_maxAngularDof[2] =  dAcos (m_coneAngleCos);

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

	dMatrix localMatrix (matrix0 * (m_rotationOffset * matrix1).Inverse());
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

