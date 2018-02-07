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


//////////////////////////////////////////////////////////////////////
#include "dCustomJointLibraryStdAfx.h"
#include "dCustom6dof.h"


IMPLEMENT_CUSTOM_JOINT(dCustom6dof);

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
dCustom6dof::dCustom6dof (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(6, child, parent)
	,m_minLinearLimits(0.0f)
	,m_maxLinearLimits(0.0f)
	,m_pitch()
	,m_yaw()
	,m_roll()
	,m_options()
{
	m_options.m_value = 0x3f;
//static int xxxxx;
//xxxx = xxxxx++;
	CalculateLocalMatrix (pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
}

dCustom6dof::dCustom6dof (const dMatrix& pinAndPivotChildFrame, const dMatrix& pinAndPivotParentFrame,  NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(6, child, parent)
	,m_minLinearLimits(0.0f)
	,m_maxLinearLimits(0.0f)
	,m_pitch()
	,m_yaw()
	,m_roll()
	,m_options()
{
	dMatrix dummy;

	m_options.m_value = 0x3f;
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
	callback(userData, &m_options, sizeof(m_options));
}

void dCustom6dof::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize (callback, userData);
	callback(userData, &m_minLinearLimits, sizeof (m_minLinearLimits));
	callback(userData, &m_maxLinearLimits, sizeof (m_maxLinearLimits));
	callback(userData, &m_yaw, sizeof(m_yaw));
	callback(userData, &m_roll, sizeof(m_roll));
	callback(userData, &m_pitch, sizeof(m_pitch));
	callback(userData, &m_options, sizeof(m_options));
}

void dCustom6dof::DisableAxisX()
{
	dAssert(0);
//	m_xAxis = 0;
}

void dCustom6dof::DisableAxisY()
{
	dAssert(0);
//	m_yAxis = 0;
}

void dCustom6dof::DisableAxisZ()
{
	dAssert(0);
//	m_zAxis = 0;
}

void dCustom6dof::DisableRotationX()
{
	dAssert(0);
//	m_pitchAxis = 0;
}

void dCustom6dof::DisableRotationY()
{
	dAssert(0);
//	m_yawAxis = 0;
}

void dCustom6dof::DisableRotationZ()
{
	dAssert(0);
//	m_rollAxis = 0;
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
	dMatrix matrix0_ (dPitchMatrix(30.0f * dDegreeToRad) * dRollMatrix(100.0f * dDegreeToRad) * dYawMatrix(50.0f * dDegreeToRad) * matrix1_);
	dMatrix localMatrix_(matrix0_ * matrix1_.Inverse());
	localMatrix_.GetEulerAngles(euler0, euler1, m_pitchRollYaw);
#endif

	// deal with gimbals lock
	if (euler0.m_z > 89.5f * dDegreeToRad) {
//		dAssert(0);
	} else if (euler0.m_z < -89.5f * dDegreeToRad) {
//		dAssert(0);
	}

	// deal with roll angle flip
	dAngularIntegration deltaYaw(dAngularIntegration(euler0.m_y) - m_yaw.m_currentAngle);
	dAngularIntegration deltaPitch(dAngularIntegration(euler0.m_x) - m_pitch.m_currentAngle);
	if ((dAbs(deltaYaw.GetAngle()) > (0.5f * dPi)) && (dAbs(deltaPitch.GetAngle()) > (0.5f * dPi))) {
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
dAssert (dAbs (x) < 10.0f * dDegreeToRad );
dAssert (dAbs (y) < 10.0f * dDegreeToRad );
dAssert (dAbs (z) < 10.0f * dDegreeToRad );
dTrace(("%f %f %f\n", x * dRadToDegree, y * dRadToDegree, z * dRadToDegree));
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
	const float radius = debugDisplay->m_debugScale;

	CalculateGlobalMatrix(matrix0, matrix1);

	{
		// show yaw angle limits
		if ((m_yaw.m_maxAngle > 1.0e-3f) || (m_yaw.m_minAngle < -1.0e-3f)) {
			dVector point(dFloat(radius), dFloat(0.0f), dFloat(0.0f), dFloat(0.0f));

			dFloat minAngle = dClamp(m_yaw.m_minAngle, -180.0f * dDegreeToRad, 0.0f * dDegreeToRad);
			dFloat maxAngle = dClamp(m_yaw.m_maxAngle, 0.0f * dDegreeToRad, 180.0f * dDegreeToRad);

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

			dFloat minAngle = dClamp(m_roll.m_minAngle, -180.0f * dDegreeToRad, 0.0f * dDegreeToRad);
			dFloat maxAngle = dClamp(m_roll.m_maxAngle, 0.0f * dDegreeToRad, 180.0f * dDegreeToRad);

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

			dFloat minAngle = dClamp(m_pitch.m_minAngle, -180.0f * dDegreeToRad, 0.0f * dDegreeToRad) + 0.0f * dPi;
			dFloat maxAngle = dClamp(m_pitch.m_maxAngle, 0.0f * dDegreeToRad, 180.0f * dDegreeToRad) + 0.0f * dPi;

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
	dAssert(0);
#if 0
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

static int xxxxxxxxx;
xxxxxxxxx++;

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

#else

	dVector errorAngles(GetPitch() - dClamp(GetPitch(), m_pitch.m_minAngle, m_pitch.m_maxAngle),
						GetYaw() - dClamp(GetYaw(), m_yaw.m_minAngle, m_yaw.m_maxAngle),
						GetRoll() - dClamp(GetRoll(), m_roll.m_minAngle, m_roll.m_maxAngle),
						0.0f);
	dMatrix dL(dPitchMatrix(errorAngles.m_x) * dRollMatrix(errorAngles.m_z) * dYawMatrix(errorAngles.m_y));
//	dMatrix L(matrix0 * matrix1.Inverse());
//  dMatrix matrix1_ = L * matrix1;
//	matrix0 = dL * dL.Inverse() * L * matrix1;
	dMatrix clipMatrix (dL.Inverse() * matrix0);

	dAngleData* angle[] = {&m_pitch, &m_yaw, &m_roll};

	for (int i = 0; i < 3; i++) {
		if (m_mask & (1 << (i + 3))) {
			const dAngleData& angleLimits = *angle[i];
			if ((angleLimits.m_minAngle == 0.0f) && (angleLimits.m_maxAngle == 0.0f)) {
				NewtonUserJointAddAngularRow(m_joint, -errorAngles[i], &clipMatrix[i][0]);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);

				dTrace(("J: %f (%f %f %f\n", errorAngles[i] * dRadToDegree, clipMatrix[i][0], clipMatrix[i][1], clipMatrix[i][2]));
			} else if (errorAngles[i] > 0.0f) {
				NewtonUserJointAddAngularRow(m_joint, -errorAngles[i], &clipMatrix[i][0]);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
			} else if (errorAngles[i] < 0.0f) {
				NewtonUserJointAddAngularRow(m_joint, -errorAngles[i], &clipMatrix[i][0]);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
			}
		}
	}

/*
	const dVector& coneDir0 = matrix0.m_front;
	const dVector& coneDir1 = matrix1.m_front;
	dVector lateralDir(coneDir1.CrossProduct(coneDir0));
	dFloat mag2 = lateralDir.DotProduct3(lateralDir);
	if (mag2 > 1.0e-4f) {
		dAssert(mag2 > 1.0e-4f);
		lateralDir = lateralDir.Scale(1.0f / dSqrt(mag2));
		dQuaternion rot(lateralDir, dAcos(coneDir0.DotProduct3(coneDir1)));
		dMatrix xxxx (matrix1 * dMatrix(rot, matrix0.m_front));
		dMatrix xxxx1(matrix0 * xxxx.Inverse());

		dVector upDir(coneDir0.CrossProduct(lateralDir));
	}
*/



//if (dAbs(errorAngles.m_x * dRadToDegree) > 10.0f)
//{
//dTrace(("xxx-> "));
//}
//	dTrace (("f:%d j:%d (%f %f %f)\n", xxxxxxxxx/2, xxxx, errorAngles.m_x * dRadToDegree, errorAngles.m_y * dRadToDegree, errorAngles.m_z * dRadToDegree));

#endif

	int freedof = (m_mask & 0x55) + ((m_mask >> 1) & 0x55);
	freedof = (freedof & 0x33) + ((freedof >> 2) & 0x33);
	freedof = (freedof & 0x0f) + ((freedof >> 4) & 0xff);
	if (freedof != 6) {
		SubmitConstraintsFreeDof(6 - freedof, matrix0, matrix1, timestep, threadIndex);
	}

#endif
}

