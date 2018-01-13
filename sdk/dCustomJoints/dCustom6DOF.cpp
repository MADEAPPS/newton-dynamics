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
//#define D_6DOF_ANGULAR_MAX_LINEAR_CORRECTION	0.5f
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

void dCustom6DOF::GetEulers(dFloat& pitch, dFloat& yaw, dFloat& roll, const dMatrix& matrix0, const dMatrix& matrix1) const
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

	yaw = euler0.m_y;
	roll = euler0.m_z;
	pitch = euler0.m_x;

dTrace(("%f %f %f\n", pitch * 180.0f / 3.141592f, yaw * 180.0f / 3.141592f, roll * 180.0f / 3.141592f));
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

//return;
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
	//dVector dp (p0 - p1);

	for (int i = 0; i < 3; i ++) {
		if ((m_minLinearLimits[i] == 0.0f) && (m_maxLinearLimits[i] == 0.0f)) {
			NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix1[i][0]);
			NewtonUserJointSetRowStiffness (m_joint, m_stiffness);
		} else {
			dAssert (0);
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


static int xxx;
xxx++;

	dFloat yaw;
	dFloat roll;
	dFloat pitch;
	GetEulers(pitch, yaw, roll, matrix0, matrix1);

	if ((m_pitch.m_minAngle == 0.0f) && (m_pitch.m_maxAngle == 0.0f)) {
		NewtonUserJointAddAngularRow(m_joint, -pitch, &matrix0.m_front[0]);
		NewtonUserJointSetRowStiffness (m_joint, m_stiffness);
	} else {

	}

	if ((m_yaw.m_minAngle == 0.0f) && (m_yaw.m_maxAngle == 0.0f)) {
		NewtonUserJointAddAngularRow(m_joint, -yaw, &matrix1.m_up[0]);
		NewtonUserJointSetRowStiffness (m_joint, m_stiffness);
	} else {

	}

	matrix1 = dYawMatrix(yaw) * matrix1;
	if ((m_roll.m_minAngle == 0.0f) && (m_roll.m_maxAngle == 0.0f)) {
		NewtonUserJointAddAngularRow(m_joint, -roll, &matrix1.m_right[0]);
		NewtonUserJointSetRowStiffness (m_joint, m_stiffness);
	} else {

		dVector omega0;
		dVector omega1;

		NewtonBodyGetOmega(m_body0, &omega0[0]);
		NewtonBodyGetOmega(m_body1, &omega1[0]);
		dVector relOmega(omega0 - omega1);
		const dFloat damp = 0.4f;
		const dFloat invTimestep = 1.0f / timestep;

		// calculate twisting axis acceleration
		dFloat rollAlpha = (1.0f - relOmega.DotProduct3(matrix1.m_right)) * invTimestep;
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_right[0]);
		NewtonUserJointSetRowAcceleration(m_joint, rollAlpha);
	}

	m_yaw.m_currentAngle.Update(yaw);
	m_roll.m_currentAngle.Update(roll);
	m_pitch.m_currentAngle.Update(pitch);
}

