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


// dCustom6DOF.cpp: implementation of the dCustom6DOF class.
//
//////////////////////////////////////////////////////////////////////
#include "dCustomJointLibraryStdAfx.h"
#include "dCustom6DOF.h"


IMPLEMENT_CUSTOM_JOINT(dCustom6DOF);

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//#define MIN_JOINT_PIN_LENGTH					50.0f
#define D_6DOF_ANGULAR_MAX_LINEAR_CORRECTION	0.5f
//#define D_6DOF_ANGULAR_MAX_ANGULAR_CORRECTION	10.0f

dCustom6DOF::dCustom6DOF (const dMatrix& pinsAndPivotChildFrame, const dMatrix& pinsAndPivotParentFrame___, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(6, child, parent)
	,m_minLinearLimits(0.0f)
	,m_maxLinearLimits(0.0f)
	,m_yaw()
	,m_roll()
	,m_pitch()
{
	CalculateLocalMatrix (pinsAndPivotChildFrame, m_localMatrix0, m_localMatrix1);
}

dCustom6DOF::~dCustom6DOF()
{
}

void dCustom6DOF::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	callback (userData, &m_minLinearLimits, sizeof (dVector));
	callback (userData, &m_maxLinearLimits, sizeof (dVector));
	callback (userData, &m_yaw, sizeof (dAngleData));
	callback (userData, &m_roll, sizeof (dAngleData));
	callback (userData, &m_pitch, sizeof (dAngleData));
}

void dCustom6DOF::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize (callback, userData);
	callback(userData, &m_minLinearLimits, sizeof (dVector));
	callback(userData, &m_maxLinearLimits, sizeof (dVector));
	callback(userData, &m_yaw, sizeof(dAngleData));
	callback(userData, &m_roll, sizeof(dAngleData));
	callback(userData, &m_pitch, sizeof(dAngleData));
}

void dCustom6DOF::SetLinearLimits (const dVector& minLinearLimits, const dVector& maxLinearLimits)
{
	for (int i = 0; i < 3; i ++) {
		m_minLinearLimits[i] =  (dAbs (minLinearLimits[i]) < dFloat (1.0e-5f)) ? 0.0f : minLinearLimits[i];
		m_maxLinearLimits[i] =  (dAbs (maxLinearLimits[i]) < dFloat (1.0e-5f)) ? 0.0f : maxLinearLimits[i];
	}
}


void dCustom6DOF::GetLinearLimits (dVector& minLinearLimits, dVector& maxLinearLimits) const
{
	minLinearLimits = m_minLinearLimits;
	maxLinearLimits = m_maxLinearLimits;
}

void dCustom6DOF::SetYawLimits(dFloat minAngle, dFloat maxAngle)
{
	m_yaw.m_maxAngle = dAbs(maxAngle);
	m_yaw.m_minAngle = -dAbs(minAngle);
}

void dCustom6DOF::SetRollLimits(dFloat minAngle, dFloat maxAngle)
{
	m_roll.m_maxAngle = dAbs(maxAngle);
	m_roll.m_minAngle = -dAbs(minAngle);
}

void dCustom6DOF::SetPitchLimits(dFloat minAngle, dFloat maxAngle)
{
	m_pitch.m_maxAngle = dAbs(maxAngle);
	m_pitch.m_minAngle = -dAbs(minAngle);
}

void dCustom6DOF::GetYawLimits(dFloat& minAngle, dFloat& maxAngle) const
{
	maxAngle = m_yaw.m_maxAngle;
	minAngle = m_yaw.m_minAngle;
}

void dCustom6DOF::GetRollLimits(dFloat& minAngle, dFloat& maxAngle) const
{
	maxAngle = m_roll.m_maxAngle;
	minAngle = m_roll.m_minAngle;
}

void dCustom6DOF::GetPitchLimits(dFloat& minAngle, dFloat& maxAngle) const
{
	maxAngle = m_pitch.m_maxAngle;
	minAngle = m_pitch.m_minAngle;
}



void dCustom6DOF::Debug(dDebugDisplay* const debugDisplay) const
{
	dMatrix matrix0;
	dMatrix matrix1;

	dCustomJoint::Debug(debugDisplay);

	const int subdiv = 12;
	const float radius = 0.5f;
	dVector arch[subdiv + 1];

	CalculateGlobalMatrix(matrix0, matrix1);
	{
		// show pitch angle limits
		dVector point(dFloat(0.0f), dFloat(radius), dFloat(0.0f), dFloat(0.0f));

		dFloat minAngle = dClamp(m_pitch.m_minAngle, -180.0f * 3.141592f / 180.0f, 0.0f * 3.141592f / 180.0f) + 0.0f * 3.141592f;
		dFloat maxAngle = dClamp(m_pitch.m_maxAngle, 0.0f * 3.141592f / 180.0f, 180.0f * 3.141592f / 180.0f) + 0.0f * 3.141592f;

		dFloat angleStep = (maxAngle - minAngle) / subdiv;
		dFloat angle0 = minAngle;

		debugDisplay->SetColor(dVector(0.5f, 0.0f, 0.0f, 0.0f));
		for (int i = 0; i <= subdiv; i++) {
			arch[i] = matrix0.TransformVector(dPitchMatrix(angle0).RotateVector(point));
			debugDisplay->DrawLine(matrix0.m_posit, arch[i]);
			angle0 += angleStep;
		}

		for (int i = 0; i < subdiv; i++) {
			debugDisplay->DrawLine(arch[i], arch[i + 1]);
		}
	}

return;
	{
		// show yaw angle limits
		dVector point(dFloat(radius), dFloat(0.0f), dFloat(0.0f), dFloat(0.0f));

		dFloat minAngle = dClamp(m_yaw.m_minAngle, -180.0f * 3.141592f / 180.0f, 0.0f * 3.141592f / 180.0f);
		dFloat maxAngle = dClamp(m_yaw.m_maxAngle, 0.0f * 3.141592f / 180.0f, 180.0f * 3.141592f / 180.0f);

		dFloat angleStep = (maxAngle - minAngle) / subdiv;
		dFloat angle0 = minAngle;

		debugDisplay->SetColor(dVector(0.0f, 0.5f, 0.0f, 0.0f));
		for (int i = 0; i <= subdiv; i++) {
			arch[i] = matrix1.TransformVector(dYawMatrix(angle0).RotateVector(point));
			debugDisplay->DrawLine(matrix1.m_posit, arch[i]);
			angle0 += angleStep;
		}

		for (int i = 0; i < subdiv; i++) {
			debugDisplay->DrawLine(arch[i], arch[i + 1]);
		}
	}

	{
		dFloat pitch;
		dFloat yaw;
		dFloat roll;
		GetEulers(pitch, yaw, roll, matrix0, matrix1);

		// show roll angle limits
		dVector point(dFloat(radius), dFloat(0.0f), dFloat(0.0f), dFloat(0.0f));

		dFloat minAngle = dClamp(m_roll.m_minAngle, -180.0f * 3.141592f / 180.0f, 0.0f * 3.141592f / 180.0f);
		dFloat maxAngle = dClamp(m_roll.m_maxAngle, 0.0f * 3.141592f / 180.0f, 180.0f * 3.141592f / 180.0f);

		dFloat angleStep = (maxAngle - minAngle) / subdiv;
		dFloat angle0 = minAngle;


		matrix1 = dYawMatrix(yaw) * matrix1;
		debugDisplay->SetColor(dVector(0.0f, 0.0f, 0.5f, 0.0f));
		for (int i = 0; i <= subdiv; i++) {
			arch[i] = matrix1.TransformVector(dRollMatrix(angle0).RotateVector(point));
			debugDisplay->DrawLine(matrix1.m_posit, arch[i]);
			angle0 += angleStep;
		}

		for (int i = 0; i < subdiv; i++) {
			debugDisplay->DrawLine(arch[i], arch[i + 1]);
		}
	}
}



void dCustom6DOF::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	// add the linear limits
	const dVector& p0 = matrix0.m_posit;
	const dVector& p1 = matrix1.m_posit;
	dVector dp (p0 - p1);

	for (int i = 0; i < 3; i ++) {
		if ((m_minLinearLimits[i] == 0.0f) && (m_maxLinearLimits[i] == 0.0f)) {
			NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix0[i][0]);
			NewtonUserJointSetRowStiffness (m_joint, 1.0f);
		} else {
			// it is a limited linear dof, check if it pass the limits
			dFloat dist = dp.DotProduct3(matrix1[i]);
			if (dist > m_maxLinearLimits[i]) {
				dVector q1 (p1 + matrix1[i].Scale (m_maxLinearLimits[i]));

				// clamp the error, so that not too much energy is added when constraint violation occurs
				dFloat maxDist = (p0 - q1).DotProduct3(matrix1[i]);
				if (maxDist > D_6DOF_ANGULAR_MAX_LINEAR_CORRECTION) {
					q1 = p0 - matrix1[i].Scale(D_6DOF_ANGULAR_MAX_LINEAR_CORRECTION);
				}

				NewtonUserJointAddLinearRow (m_joint, &p0[0], &q1[0], &matrix0[i][0]);
				NewtonUserJointSetRowStiffness (m_joint, 1.0f);
				// allow the object to return but not to kick going forward
				NewtonUserJointSetRowMaximumFriction (m_joint, 0.0f);

			} else if (dist < m_minLinearLimits[i]) {
				dVector q1 (p1 + matrix1[i].Scale (m_minLinearLimits[i]));

				// clamp the error, so the not too much energy is added when constraint violation occurs
				dFloat maxDist = (p0 - q1).DotProduct3(matrix1[i]);
				if (maxDist < -D_6DOF_ANGULAR_MAX_LINEAR_CORRECTION) {
					q1 = p0 - matrix1[i].Scale(-D_6DOF_ANGULAR_MAX_LINEAR_CORRECTION);
				}

				NewtonUserJointAddLinearRow (m_joint, &p0[0], &q1[0], &matrix0[i][0]);
				NewtonUserJointSetRowStiffness (m_joint, 1.0f);
				// allow the object to return but not to kick going forward
				NewtonUserJointSetRowMinimumFriction (m_joint, 0.0f);
			}
		}
	}

/*
	dVector euler0(0.0f);
	dVector euler1(0.0f);
	dMatrix localMatrix (matrix0 * matrix1.Inverse());
	localMatrix.GetEulerAngles(euler0, euler1);

	dAngularIntegration pitchStep0 (dAngularIntegration (euler0.m_x) - m_pitch);
	dAngularIntegration pitchStep1 (dAngularIntegration (euler1.m_x) - m_pitch);
	if (dAbs (pitchStep0.GetAngle()) > dAbs (pitchStep1.GetAngle())) {
		euler0 = euler1;
	}

	dVector euler (m_pitch.Update (euler0.m_x), m_yaw.Update (euler0.m_y), m_roll.Update (euler0.m_z), 0.0f);

//dTrace (("(%f %f %f) (%f %f %f)\n", m_pitch.m_angle * 180.0f / 3.141592f, m_yaw.m_angle * 180.0f / 3.141592f, m_roll.m_angle * 180.0f / 3.141592f,  euler0.m_x * 180.0f / 3.141592f, euler0.m_y * 180.0f / 3.141592f, euler0.m_z * 180.0f / 3.141592f));

	bool limitViolation = false;
	for (int i = 0; i < 3; i ++) {
		if (euler[i] < m_minAngularLimits[i]) {
			limitViolation = true;
			euler[i] = m_minAngularLimits[i];
		} else if (euler[i] > m_maxAngularLimits[i]) {
			limitViolation = true;
			euler[i] = m_maxAngularLimits[i];
		}
	}

	if (limitViolation) {
		//dMatrix pyr (dPitchMatrix(m_pitch.m_angle) * dYawMatrix(m_yaw.m_angle) * dRollMatrix(m_roll.m_angle));
		dMatrix p0y0r0 (dPitchMatrix(euler[0]) * dYawMatrix(euler[1]) * dRollMatrix(euler[2]));
		dMatrix baseMatrix (p0y0r0 * matrix1);
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

			dVector q0 (matrix0[3] + basis[1].Scale (MIN_JOINT_PIN_LENGTH));
			dVector q1 (matrix0[3] + rotation.RotateVector(basis[1].Scale (MIN_JOINT_PIN_LENGTH)));
			NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &basis[2][0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);

			//dVector q0 (matrix0[3] + basis[0].Scale (MIN_JOINT_PIN_LENGTH));
			//NewtonUserJointAddLinearRow (m_joint, &q0[0], &q0[0], &basis[1][0]);
			//NewtonUserJointAddLinearRow (m_joint, &q0[0], &q0[0], &basis[2][0]);
        }
	}
*/
}

