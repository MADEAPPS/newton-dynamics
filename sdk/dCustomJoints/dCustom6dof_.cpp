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


// dCustom6dof.cpp: implementation of the dCustom6dof class.
//
//////////////////////////////////////////////////////////////////////
#include "dCustomJointLibraryStdAfx.h"
#include "dCustom6dof.h"


IMPLEMENT_CUSTOM_JOINT(dCustom6dof);

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//#define MIN_JOINT_PIN_LENGTH					50.0f
//#define D_6DOF_ANGULAR_MAX_LINEAR_CORRECTION	0.5f
//#define D_6DOF_ANGULAR_MAX_ANGULAR_CORRECTION	10.0f

dCustom6dof::dCustom6dof (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(6, child, parent)
	,m_minLinearLimits(0.0f)
	,m_maxLinearLimits(0.0f)
	,m_yaw()
	,m_roll()
	,m_pitch()
	,m_mask(0x3f)
{
	CalculateLocalMatrix (pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
}

dCustom6dof::dCustom6dof (const dMatrix& pinAndPivotChildFrame, const dMatrix& pinAndPivotParentFrame,  NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(6, child, parent)
	,m_minLinearLimits(0.0f)
	,m_maxLinearLimits(0.0f)
	,m_yaw()
	,m_roll()
	,m_pitch()
	,m_mask(0x3f)
{
	dMatrix dummy;
	CalculateLocalMatrix(pinAndPivotChildFrame, m_localMatrix0, dummy);
	CalculateLocalMatrix(pinAndPivotParentFrame, dummy, m_localMatrix1);
}

dCustom6dof::~dCustom6dof()
{
}

void dCustom6dof::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	callback (userData, &m_minLinearLimits, sizeof (dVector));
	callback (userData, &m_maxLinearLimits, sizeof (dVector));
	callback (userData, &m_yaw, sizeof (dAngleData));
	callback (userData, &m_roll, sizeof (dAngleData));
	callback (userData, &m_pitch, sizeof (dAngleData));
}

void dCustom6dof::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize (callback, userData);
	callback(userData, &m_minLinearLimits, sizeof (dVector));
	callback(userData, &m_maxLinearLimits, sizeof (dVector));
	callback(userData, &m_yaw, sizeof(dAngleData));
	callback(userData, &m_roll, sizeof(dAngleData));
	callback(userData, &m_pitch, sizeof(dAngleData));
}

void dCustom6dof::SetLinearLimits (const dVector& minLinearLimits, const dVector& maxLinearLimits)
{
	for (int i = 0; i < 3; i ++) {
		m_minLinearLimits[i] =  (dAbs (minLinearLimits[i]) < dFloat (1.0e-5f)) ? 0.0f : minLinearLimits[i];
		m_maxLinearLimits[i] =  (dAbs (maxLinearLimits[i]) < dFloat (1.0e-5f)) ? 0.0f : maxLinearLimits[i];
	}
}


void dCustom6dof::GetLinearLimits (dVector& minLinearLimits, dVector& maxLinearLimits) const
{
	minLinearLimits = m_minLinearLimits;
	maxLinearLimits = m_maxLinearLimits;
}

void dCustom6dof::SetYawLimits(dFloat minAngle, dFloat maxAngle)
{
	m_yaw.m_maxAngle = dAbs(maxAngle);
	m_yaw.m_minAngle = -dAbs(minAngle);
}

void dCustom6dof::SetRollLimits(dFloat minAngle, dFloat maxAngle)
{
	m_roll.m_maxAngle = dAbs(maxAngle);
	m_roll.m_minAngle = -dAbs(minAngle);
}

void dCustom6dof::SetPitchLimits(dFloat minAngle, dFloat maxAngle)
{
	m_pitch.m_maxAngle = dAbs(maxAngle);
	m_pitch.m_minAngle = -dAbs(minAngle);
}

void dCustom6dof::GetYawLimits(dFloat& minAngle, dFloat& maxAngle) const
{
	maxAngle = m_yaw.m_maxAngle;
	minAngle = m_yaw.m_minAngle;
}

void dCustom6dof::GetRollLimits(dFloat& minAngle, dFloat& maxAngle) const
{
	maxAngle = m_roll.m_maxAngle;
	minAngle = m_roll.m_minAngle;
}

void dCustom6dof::GetPitchLimits(dFloat& minAngle, dFloat& maxAngle) const
{
	maxAngle = m_pitch.m_maxAngle;
	minAngle = m_pitch.m_minAngle;
}


void dCustom6dof::CalculateJointAngles(const dMatrix& matrix0, const dMatrix& matrix1)
{
	dMatrix localMatrix(matrix0 * matrix1.Inverse());
	dVector euler0;
	dVector euler1;
	localMatrix.GetEulerAngles(euler0, euler1, m_pitchRollYaw);

#if 0
	dMatrix matrix1_ (dGetIdentityMatrix());
	dMatrix matrix0_ (dPitchMatrix(30.0f * 3.141592f / 180.0f) * dRollMatrix(100.0f * 3.141592f / 180.0f) * dYawMatrix(50.0f * 3.141592f / 180.0f) * matrix1_);
	dMatrix localMatrix_(matrix0_ * matrix1_.Inverse());
	localMatrix_.GetEulerAngles(euler0, euler1, m_pitchRollYaw);
#endif

	dAngularIntegration deltaYaw(dAngularIntegration(euler0.m_y) - m_yaw.m_currentAngle);
	dAngularIntegration deltaPitch(dAngularIntegration(euler0.m_x) - m_pitch.m_currentAngle);
	if ((dAbs(deltaYaw.GetAngle()) > (0.5f * 3.141592f)) && (dAbs(deltaPitch.GetAngle()) > (0.5f * 3.141592f))) {
		euler0 = euler1;
	}

	m_yaw.m_currentAngle.Update(euler0.m_y);
	m_roll.m_currentAngle.Update(euler0.m_z);
	m_pitch.m_currentAngle.Update(euler0.m_x);
dTrace(("%f %f %f\n", GetPitch() * 180.0f / 3.141592f, GetYaw() * 180.0f / 3.141592f, GetRoll() * 180.0f / 3.141592f));
}

void dCustom6dof::Debug(dDebugDisplay* const debugDisplay) const
{
	dMatrix matrix0;
	dMatrix matrix1;

	dCustomJoint::Debug(debugDisplay);

	const int subdiv = 12;
	const float radius = 0.5f;
	dVector arch[subdiv + 1];

	CalculateGlobalMatrix(matrix0, matrix1);

	if (m_pitchAxis) {
		if ((m_pitch.m_maxAngle > 1.0e-3f) || (m_pitch.m_minAngle < -1.0e-3f)) {
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
	}

	if (m_yawAxis) {
		// show yaw angle limits
		if ((m_yaw.m_maxAngle > 1.0e-3f) || (m_yaw.m_minAngle < -1.0e-3f)) {
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
	}

	if (m_rollAxis) {
		// show roll angle limits
		if ((m_roll.m_maxAngle > 1.0e-3f) || (m_roll.m_minAngle < -1.0e-3f)) {
			dVector point(dFloat(radius), dFloat(0.0f), dFloat(0.0f), dFloat(0.0f));

			dFloat minAngle = dClamp(m_roll.m_minAngle, -180.0f * 3.141592f / 180.0f, 0.0f * 3.141592f / 180.0f);
			dFloat maxAngle = dClamp(m_roll.m_maxAngle, 0.0f * 3.141592f / 180.0f, 180.0f * 3.141592f / 180.0f);

			dFloat angleStep = (maxAngle - minAngle) / subdiv;
			dFloat angle0 = minAngle;


			matrix1 = dYawMatrix(GetYaw()) * matrix1;
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
}


void dCustom6dof::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	// add the linear limits
	const dVector& p0 = matrix0.m_posit;
	const dVector& p1 = matrix1.m_posit;

	for (int i = 0; i < 3; i ++) {
		if (m_mask & (1<<i)) {
			if ((m_minLinearLimits[i] == 0.0f) && (m_maxLinearLimits[i] == 0.0f)) {
				NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix1[i][0]);
				NewtonUserJointSetRowStiffness (m_joint, m_stiffness);
			} else {
				dAssert(0);
				/*
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
				*/
			}
		}
	}

	CalculateJointAngles(matrix0, matrix1);
	if (m_pitchAxis) {
		dFloat pitchAngle = GetPitch();
		if ((m_pitch.m_minAngle == 0.0f) && (m_pitch.m_maxAngle == 0.0f)) {
			NewtonUserJointAddAngularRow(m_joint, -pitchAngle, &matrix0.m_front[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		} else {
			if (pitchAngle > m_pitch.m_maxAngle) {
				NewtonUserJointAddAngularRow(m_joint, m_pitch.m_maxAngle - pitchAngle, &matrix0.m_front[0]);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
			} else if (pitchAngle < m_pitch.m_minAngle) {
				NewtonUserJointAddAngularRow(m_joint, m_pitch.m_minAngle - pitchAngle, &matrix0.m_front[0]);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
			}
		}
	}

	if (m_yawAxis) {
		dFloat yawAngle = GetYaw();
		if ((m_yaw.m_minAngle == 0.0f) && (m_yaw.m_maxAngle == 0.0f)) {
			NewtonUserJointAddAngularRow(m_joint, -yawAngle, &matrix1.m_up[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		} else {
			if (yawAngle > m_yaw.m_maxAngle) {
				NewtonUserJointAddAngularRow(m_joint, m_yaw.m_maxAngle - yawAngle, &matrix1.m_up[0]);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
			} else if (yawAngle < m_yaw.m_minAngle) {
				NewtonUserJointAddAngularRow(m_joint, m_yaw.m_minAngle - yawAngle, &matrix1.m_up[0]);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
			}
		}
	}

	if (m_rollAxis) {
		matrix1 = dYawMatrix(GetYaw()) * matrix1;
		dFloat rollAngle = GetRoll();
		if ((m_roll.m_minAngle == 0.0f) && (m_roll.m_maxAngle == 0.0f)) {
			NewtonUserJointAddAngularRow(m_joint, -rollAngle, &matrix1.m_right[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		} else {
			if (rollAngle > m_roll.m_maxAngle) {
				NewtonUserJointAddAngularRow(m_joint, m_roll.m_maxAngle - rollAngle, &matrix1.m_right[0]);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
			} else if (rollAngle < m_roll.m_minAngle) {
				NewtonUserJointAddAngularRow(m_joint, m_roll.m_minAngle - rollAngle, &matrix1.m_right[0]);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
			}
		}
	}

	int freedof = (m_mask & 0x55) + ((m_mask >> 1) & 0x55);
	freedof = (freedof & 0x33) + ((freedof >> 2) & 0x33);
	freedof = (freedof & 0x0f) + ((freedof >> 4) & 0xff);
	if (freedof != 6) {
		SubmitConstraintsFreeDof(6 - freedof, matrix0, matrix1, timestep, threadIndex);
	}
}

