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
	,m_debugScale(1.0f)
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
	,m_debugScale(1.0f)
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
	callback(userData, &m_minLinearLimits, sizeof(m_minLinearLimits));
	callback(userData, &m_maxLinearLimits, sizeof(m_maxLinearLimits));
	callback(userData, &m_yaw, sizeof(m_yaw));
	callback(userData, &m_roll, sizeof(m_roll));
	callback(userData, &m_pitch, sizeof(m_pitch));
	callback(userData, &m_debugScale, sizeof(m_debugScale));
	callback(userData, &m_mask, sizeof(m_mask));
}

void dCustom6dof::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize (callback, userData);
	callback(userData, &m_minLinearLimits, sizeof (m_minLinearLimits));
	callback(userData, &m_maxLinearLimits, sizeof (m_maxLinearLimits));
	callback(userData, &m_yaw, sizeof(m_yaw));
	callback(userData, &m_roll, sizeof(m_roll));
	callback(userData, &m_pitch, sizeof(m_pitch));
	callback(userData, &m_debugScale, sizeof(m_debugScale));
	callback(userData, &m_mask, sizeof(m_mask));
}

void dCustom6dof::DisableAxisX()
{
	m_xAxis = 0;
}

void dCustom6dof::DisableAxisY()
{
	m_yAxis = 0;
}

void dCustom6dof::DisableAxisZ()
{
	m_zAxis = 0;
}

void dCustom6dof::DisableRotationX()
{
	m_pitchAxis = 0;
}

void dCustom6dof::DisableRotationY()
{
	m_yawAxis = 0;
}

void dCustom6dof::DisableRotationZ()
{
	m_rollAxis = 0;
}



dFloat dCustom6dof::GetDebugScale() const
{
	return m_debugScale;
}

void dCustom6dof::SetDebugScale(dFloat scale)
{
	m_debugScale = scale;
}


void dCustom6dof::SetLinearLimits (const dVector& minLinearLimits, const dVector& maxLinearLimits)
{
	for (int i = 0; i < 3; i ++) {
		m_minLinearLimits[i] =  (dAbs (minLinearLimits[i]) < dFloat (1.0e-3f)) ? dFloat(0.0f) : -dAbs (minLinearLimits[i]);
		m_maxLinearLimits[i] =  (dAbs (maxLinearLimits[i]) < dFloat (1.0e-3f)) ? dFloat(0.0f) :  dAbs (maxLinearLimits[i]);
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

	// deal with gimbals lock
	if (euler0.m_z > 89.5f * 3.141592f / 180.0f) {
//		dAssert(0);
	} else if (euler0.m_z < -89.5f * 3.141592f / 180.0f) {
//		dAssert(0);
	}

	// deal with roll angle flip
	dAngularIntegration deltaYaw(dAngularIntegration(euler0.m_y) - m_yaw.m_currentAngle);
	dAngularIntegration deltaPitch(dAngularIntegration(euler0.m_x) - m_pitch.m_currentAngle);
	if ((dAbs(deltaYaw.GetAngle()) > (0.5f * 3.141592f)) && (dAbs(deltaPitch.GetAngle()) > (0.5f * 3.141592f))) {
		euler0 = euler1;
	}

#if 0
dFloat x = GetPitch();
dFloat y = GetYaw();
dFloat z = GetRoll();
dAngularIntegration x0(m_pitch.m_currentAngle);
dAngularIntegration y0(m_yaw.m_currentAngle);
dAngularIntegration z0(m_roll.m_currentAngle);
x0.Update(euler0.m_x);
y0.Update(euler0.m_y);
z0.Update(euler0.m_z);
x -= x0.GetAngle();
y -= y0.GetAngle();
z -= z0.GetAngle();
dAssert (dAbs (x) < 10.0f * 3.141592f / 180.0f );
dAssert (dAbs (y) < 10.0f * 3.141592f / 180.0f );
dAssert (dAbs (z) < 10.0f * 3.141592f / 180.0f );
dTrace(("%f %f %f\n", x * 180.0f / 3.141592f, y * 180.0f / 3.141592f, z * 180.0f / 3.141592f));
#endif

	m_yaw.m_currentAngle.Update(euler0.m_y);
	m_roll.m_currentAngle.Update(euler0.m_z);
	m_pitch.m_currentAngle.Update(euler0.m_x);
}

void dCustom6dof::Debug(dDebugDisplay* const debugDisplay) const
{
	dMatrix matrix0;
	dMatrix matrix1;

	dCustomJoint::Debug(debugDisplay);

	const int subdiv = 12;
	dVector arch[subdiv + 1];
	const float radius = m_debugScale;

	CalculateGlobalMatrix(matrix0, matrix1);

	{
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

	matrix1 = dYawMatrix(GetYaw()) * matrix1;
	{
		// show roll angle limits
		if ((m_roll.m_maxAngle > 1.0e-3f) || (m_roll.m_minAngle < -1.0e-3f)) {
			dVector point(dFloat(radius), dFloat(0.0f), dFloat(0.0f), dFloat(0.0f));

			dFloat minAngle = dClamp(m_roll.m_minAngle, -180.0f * 3.141592f / 180.0f, 0.0f * 3.141592f / 180.0f);
			dFloat maxAngle = dClamp(m_roll.m_maxAngle, 0.0f * 3.141592f / 180.0f, 180.0f * 3.141592f / 180.0f);

			dFloat angleStep = (maxAngle - minAngle) / subdiv;
			dFloat angle0 = minAngle;
			
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

	matrix1 = dRollMatrix(GetRoll()) * matrix1;
	{
		if ((m_pitch.m_maxAngle > 1.0e-3f) || (m_pitch.m_minAngle < -1.0e-3f)) {
			// show pitch angle limits
			dVector point(dFloat(0.0f), dFloat(radius), dFloat(0.0f), dFloat(0.0f));

			dFloat minAngle = dClamp(m_pitch.m_minAngle, -180.0f * 3.141592f / 180.0f, 0.0f * 3.141592f / 180.0f) + 0.0f * 3.141592f;
			dFloat maxAngle = dClamp(m_pitch.m_maxAngle, 0.0f * 3.141592f / 180.0f, 180.0f * 3.141592f / 180.0f) + 0.0f * 3.141592f;

			dFloat angleStep = (maxAngle - minAngle) / subdiv;
			dFloat angle0 = minAngle;

			matrix1.m_posit = matrix0.m_posit;
			debugDisplay->SetColor(dVector(0.5f, 0.0f, 0.0f, 0.0f));
			for (int i = 0; i <= subdiv; i++) {
				arch[i] = matrix1.TransformVector(dPitchMatrix(angle0).RotateVector(point));
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

	// update joint angle
	CalculateJointAngles(matrix0, matrix1);

	// add the linear limits
	const dVector& p0 = matrix0.m_posit;
	const dVector& p1 = matrix1.m_posit;
	dVector step(p0 - p1);

	for (int i = 0; i < 3; i ++) {
		if (m_mask & (1<<i)) {
			if ((m_minLinearLimits[i] == 0.0f) && (m_maxLinearLimits[i] == 0.0f)) {
				NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix1[i][0]);
				NewtonUserJointSetRowStiffness (m_joint, m_stiffness);
			} else {
				dFloat posit = step.DotProduct3(matrix1[i]);
				if (posit < m_minLinearLimits.m_x) {
					NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1[i][0]);
					NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				} else if (posit > m_maxLinearLimits.m_x) {
					NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1[i][0]);
					NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				}
			}
		}
	}

#if 0
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
#else

	dFloat deltaYaw = GetYaw() - dClamp (GetYaw(), m_yaw.m_minAngle, m_yaw.m_maxAngle);
	dFloat deltaRoll = GetRoll() - dClamp (GetRoll(), m_roll.m_minAngle, m_roll.m_maxAngle);
	dFloat deltaPitch = GetPitch() - dClamp (GetPitch(), m_pitch.m_minAngle, m_pitch.m_maxAngle);
	
	dMatrix dL(dPitchMatrix(deltaPitch) * dRollMatrix(deltaRoll) * dYawMatrix(deltaRoll));
//	dMatrix L(matrix0 * matrix1.Inverse());
//  dMatrix matrix1_ = L * matrix1;
//	matrix0 = dL * dL.Inverse() * L * matrix1;
	dMatrix clipMatrix1 (dL.Inverse() * matrix0);

	if (m_pitchAxis) {
		if ((m_pitch.m_minAngle == 0.0f) && (m_pitch.m_maxAngle == 0.0f)) {
			NewtonUserJointAddAngularRow(m_joint, -deltaPitch, &matrix0.m_front[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		} else if (deltaPitch > 0.0f) {
			NewtonUserJointAddAngularRow(m_joint, -deltaPitch, &matrix0.m_front[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
		} else if (deltaPitch < 0.0f) {
			NewtonUserJointAddAngularRow(m_joint, -deltaPitch, &matrix0.m_front[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
		}
	}

	if (m_yawAxis) {
		if ((m_yaw.m_minAngle == 0.0f) && (m_yaw.m_maxAngle == 0.0f)) {
			NewtonUserJointAddAngularRow(m_joint, -deltaYaw, &clipMatrix1.m_up[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		} else if (deltaYaw > 0.0f) {
			NewtonUserJointAddAngularRow(m_joint, -deltaYaw, &clipMatrix1.m_up[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
		} else if (deltaYaw < 0.0f) {
			NewtonUserJointAddAngularRow(m_joint, -deltaYaw, &clipMatrix1.m_up[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
		}
	}

	if (m_rollAxis) {
		clipMatrix1 = dYawMatrix(deltaYaw) * clipMatrix1;
		if ((m_roll.m_minAngle == 0.0f) && (m_roll.m_maxAngle == 0.0f)) {
			NewtonUserJointAddAngularRow(m_joint, -deltaRoll, &clipMatrix1.m_right[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		} else if (deltaRoll > 0.0f) {
			NewtonUserJointAddAngularRow(m_joint, -deltaRoll, &clipMatrix1.m_right[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
		} else if (deltaRoll < 0.0f) {
			NewtonUserJointAddAngularRow(m_joint, -deltaRoll, &clipMatrix1.m_right[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
		}
	}

	dTrace (("%f %f %f\n", deltaPitch, deltaRoll, deltaYaw));

#endif
}

