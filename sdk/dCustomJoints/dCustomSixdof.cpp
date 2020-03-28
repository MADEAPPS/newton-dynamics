/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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
#include "dCustomSixdof.h"


IMPLEMENT_CUSTOM_JOINT(dCustomSixdof);

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
dCustomSixdof::dCustomSixdof (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(6, child, parent)
	,m_minLinearLimits(0.0f)
	,m_maxLinearLimits(0.0f)
	,m_pitch()
	,m_yaw()
	,m_roll()
{
	m_options.m_value = 0x3f;
	CalculateLocalMatrix (pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
}

dCustomSixdof::dCustomSixdof (const dMatrix& pinAndPivotChildFrame, const dMatrix& pinAndPivotParentFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(6, child, parent)
	,m_minLinearLimits(0.0f)
	,m_maxLinearLimits(0.0f)
	,m_pitch()
	,m_yaw()
	,m_roll()
{
	dMatrix dummy;
	m_options.m_value = 0x3f;
	CalculateLocalMatrix(pinAndPivotChildFrame, m_localMatrix0, dummy);
	CalculateLocalMatrix(pinAndPivotParentFrame, dummy, m_localMatrix1);
}

dCustomSixdof::~dCustomSixdof()
{
}

void dCustomSixdof::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	callback(userData, &m_minLinearLimits, sizeof(m_minLinearLimits));
	callback(userData, &m_maxLinearLimits, sizeof(m_maxLinearLimits));
	callback(userData, &m_yaw, sizeof(m_yaw));
	callback(userData, &m_roll, sizeof(m_roll));
	callback(userData, &m_pitch, sizeof(m_pitch));
}

void dCustomSixdof::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize (callback, userData);
	callback(userData, &m_minLinearLimits, sizeof (m_minLinearLimits));
	callback(userData, &m_maxLinearLimits, sizeof (m_maxLinearLimits));
	callback(userData, &m_yaw, sizeof(m_yaw));
	callback(userData, &m_roll, sizeof(m_roll));
	callback(userData, &m_pitch, sizeof(m_pitch));
}

void dCustomSixdof::ActiveAxisX(bool activeInactive)
{
	m_options.m_option0 = activeInactive;
}

void dCustomSixdof::ActiveAxisY(bool activeInactive)
{
	m_options.m_option1 = activeInactive;
}

void dCustomSixdof::ActiveAxisZ(bool activeInactive)
{
	m_options.m_option2 = activeInactive;
}

void dCustomSixdof::ActiveRotationX(bool activeInactive)
{
	m_options.m_option3 = activeInactive;
}

void dCustomSixdof::ActiveRotationY(bool activeInactive)
{
	m_options.m_option4 = activeInactive;
}

void dCustomSixdof::ActiveRotationZ(bool activeInactive)
{
	m_options.m_option5 = activeInactive;
}

void dCustomSixdof::SetLinearLimits (const dVector& minLinearLimits, const dVector& maxLinearLimits)
{
	for (int i = 0; i < 3; i ++) {
		m_minLinearLimits[i] =  (dAbs (minLinearLimits[i]) < dFloat (1.0e-3f)) ? dFloat(0.0f) : -dAbs (minLinearLimits[i]);
		m_maxLinearLimits[i] =  (dAbs (maxLinearLimits[i]) < dFloat (1.0e-3f)) ? dFloat(0.0f) :  dAbs (maxLinearLimits[i]);
	}
}

void dCustomSixdof::GetLinearLimits (dVector& minLinearLimits, dVector& maxLinearLimits) const
{
	minLinearLimits = m_minLinearLimits;
	maxLinearLimits = m_maxLinearLimits;
}

void dCustomSixdof::SetYawLimits(dFloat minAngle, dFloat maxAngle)
{
	m_yaw.m_maxAngle = dAbs(maxAngle);
	m_yaw.m_minAngle = -dAbs(minAngle);
}

void dCustomSixdof::SetRollLimits(dFloat minAngle, dFloat maxAngle)
{
	m_roll.m_maxAngle = dAbs(maxAngle);
	m_roll.m_minAngle = -dAbs(minAngle);
}

void dCustomSixdof::SetPitchLimits(dFloat minAngle, dFloat maxAngle)
{
	m_pitch.m_maxAngle = dAbs(maxAngle);
	m_pitch.m_minAngle = -dAbs(minAngle);
}

void dCustomSixdof::GetYawLimits(dFloat& minAngle, dFloat& maxAngle) const
{
	maxAngle = m_yaw.m_maxAngle;
	minAngle = m_yaw.m_minAngle;
}

void dCustomSixdof::GetRollLimits(dFloat& minAngle, dFloat& maxAngle) const
{
	maxAngle = m_roll.m_maxAngle;
	minAngle = m_roll.m_minAngle;
}

void dCustomSixdof::GetPitchLimits(dFloat& minAngle, dFloat& maxAngle) const
{
	maxAngle = m_pitch.m_maxAngle;
	minAngle = m_pitch.m_minAngle;
}

/*
void dCustomSixdof::CalculateJointAngles(const dMatrix& matrix0, const dMatrix& matrix1)
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

	m_yaw.m_currentAngle.Update(euler0.m_y);
	m_roll.m_currentAngle.Update(euler0.m_z);
	m_pitch.m_currentAngle.Update(euler0.m_x);
}
*/

void dCustomSixdof::Debug(dDebugDisplay* const debugDisplay) const
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	debugDisplay->DrawFrame(matrix0);
	debugDisplay->DrawFrame(matrix1);

	const int subdiv = 12;
	dVector arch[subdiv + 1];
	const dFloat radius = debugDisplay->m_debugScale;
	{
		// show yaw angle limits
		if ((m_yaw.m_maxAngle > 1.0e-3f) || (m_yaw.m_minAngle < -1.0e-3f)) {
			dVector point(dFloat(radius), dFloat(0.0f), dFloat(0.0f), dFloat(0.0f));

			dFloat minAngle = dClamp(m_yaw.m_minAngle, dFloat(-180.0f) * dDegreeToRad, dFloat(0.0f) * dDegreeToRad);
			dFloat maxAngle = dClamp(m_yaw.m_maxAngle, dFloat(0.0f) * dDegreeToRad, dFloat(180.0f) * dDegreeToRad);

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

			dFloat minAngle = dClamp(m_roll.m_minAngle, dFloat(-180.0f) * dDegreeToRad, dFloat(0.0f) * dDegreeToRad);
			dFloat maxAngle = dClamp(m_roll.m_maxAngle, dFloat(0.0f) * dDegreeToRad, dFloat(180.0f) * dDegreeToRad);

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

			dFloat minAngle = dClamp(m_pitch.m_minAngle, dFloat(-180.0f) * dDegreeToRad, dFloat(0.0f) * dDegreeToRad) + dFloat(0.0f) * dPi;
			dFloat maxAngle = dClamp(m_pitch.m_maxAngle, dFloat(0.0f) * dDegreeToRad, dFloat(180.0f) * dDegreeToRad) + dFloat(0.0f) * dPi;

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

void dCustomSixdof::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;
	dVector veloc0(0.0f);
	dVector veloc1(0.0f);

	const dFloat invtimestep = 1.0f / timestep;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	dAssert(m_body0);
	NewtonBodyGetPointVelocity(m_body0, &matrix0.m_posit[0], &veloc0[0]);
	if (m_body1) {
		NewtonBodyGetPointVelocity(m_body1, &matrix1.m_posit[0], &veloc1[0]);
	}

	// add the linear limits
	const dVector& p0 = matrix0.m_posit;
	const dVector& p1 = matrix1.m_posit;
	const dVector dp(p0 - p1);
	const dVector veloc(veloc0 - veloc1);
	for (int i = 0; i < 3; i ++) {
		if (m_options.m_value & (1 << i)) {
			if ((m_minLinearLimits[i] == 0.0f) && (m_maxLinearLimits[i] == 0.0f)) {
				NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1[i][0]);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			} else {
				dFloat posit = dp.DotProduct3(matrix1[i]);
				dFloat speed = veloc.DotProduct3(matrix1[i]);
				dFloat x = posit + speed * timestep;
				if (x < m_minLinearLimits[i]) {
					NewtonUserJointAddLinearRow(m_joint, &matrix1.m_posit[0], &matrix1.m_posit[0], &matrix1[i][0]);
					NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
					NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);

					speed = 0.5f * (m_minLinearLimits[i] - posit) * invtimestep;
					const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + speed * invtimestep;
					NewtonUserJointSetRowAcceleration(m_joint, stopAccel);

				} else if (x > m_maxLinearLimits[i]) {
					NewtonUserJointAddLinearRow(m_joint, &matrix1.m_posit[0], &matrix1.m_posit[0], &matrix1[i][0]);
					NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
					NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);

					speed = 0.5f * (m_maxLinearLimits[i] - posit) * invtimestep;
					const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + speed * invtimestep;
					NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
				}
			}
		}
	}

#if 0
	// update joint angle
	CalculateJointAngles(matrix0, matrix1);
	const dVector omega(omega0 - omega1);

	if (m_options.m_option3) {
		const dFloat pitchAngle = GetPitch();
		if ((m_pitch.m_minAngle == 0.0f) && (m_pitch.m_maxAngle == 0.0f)) {
			NewtonUserJointAddAngularRow(m_joint, -pitchAngle, &matrix0.m_front[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		} else {
			dFloat jointOmega = omega.DotProduct3(matrix0.m_front);
			dFloat projectAngle = pitchAngle + jointOmega * timestep;
			if (projectAngle < m_pitch.m_minAngle) {
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_front[0]);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);

				const dFloat speed = 0.5f * (m_pitch.m_minAngle - pitchAngle) * invtimestep;
				const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + speed * invtimestep;
				NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			} else if (projectAngle > m_pitch.m_maxAngle) {
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_front[0]);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);

				const dFloat speed = 0.5f * (m_pitch.m_maxAngle - pitchAngle) * invtimestep;
				const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + speed * invtimestep;
				NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			}
		}
	}

	if (m_options.m_option4) {
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

	if (m_options.m_option5) {
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
#endif

	dFloat cosAngleCos = matrix1.m_front.DotProduct3(matrix0.m_front);
	if (cosAngleCos >= dFloat(0.998f)) {
		// special case where the front axis are almost aligned
		// solve by using Cartesian approximation
		SubmitAngularAxisCartisianApproximation(matrix0, matrix1, timestep);
	} else {
		SubmitAngularAxis(matrix0, matrix1, timestep);
	}
}

void dCustomSixdof::SubmitTwistAngle(const dVector& pin, dFloat angle, dFloat timestep)
{
	if (m_options.m_option3) {
		const dFloat maxTwistAngle = m_pitch.m_maxAngle;
		const dFloat minTwistAngle = m_pitch.m_minAngle;
		const dFloat twistFriction___ = 0.0f;
		if ((maxTwistAngle - minTwistAngle) < (2.0f * dDegreeToRad)) {
			NewtonUserJointAddAngularRow(m_joint, -angle, &pin[0]);
		} else {
			dFloat restoringOmega = 0.25f;
			const dFloat step = dMax(dAbs(angle * timestep), dFloat(5.0f * dDegreeToRad));
			if (angle < minTwistAngle) {
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &pin[0]);
				const dFloat invtimestep = 1.0f / timestep;
				const dFloat error0 = angle - minTwistAngle;
				const dFloat error1 = error0 + restoringOmega * timestep;
				if (error1 > 0.0f) {
					restoringOmega = -error0 * invtimestep;
				}
				const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + restoringOmega * invtimestep;
				NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
			} else if (angle >= maxTwistAngle) {
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &pin[0]);
				const dFloat invtimestep = 1.0f / timestep;
				const dFloat error0 = angle - maxTwistAngle;
				const dFloat error1 = error0 - restoringOmega * timestep;
				if (error1 < 0.0f) {
					restoringOmega = error0 * invtimestep;
				}
				const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint) - restoringOmega / timestep;
				NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
			} else if ((angle - step) <= minTwistAngle) {
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &pin[0]);
				const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
				NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
			} else if ((angle + step) >= maxTwistAngle) {
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &pin[0]);
				const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
				NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
			} else if (twistFriction___ > 0.0f) {
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &pin[0]);
				const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
				NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				NewtonUserJointSetRowMinimumFriction(m_joint, -twistFriction___);
				NewtonUserJointSetRowMaximumFriction(m_joint, twistFriction___);
			}
		}
	}
}

void dCustomSixdof::SubmitAngularAxisCartisianApproximation(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
{
	dFloat pitchAngle = -CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);
	SubmitTwistAngle(matrix0.m_front, pitchAngle, timestep);

	// two rows to restrict rotation around around the parent coordinate system
	if (m_options.m_option4 || m_options.m_option5) {
		if ((m_yaw.m_minAngle == dgFloat32 (0.0f)) && (m_yaw.m_maxAngle == dgFloat32 (0.0f)) &&
			(m_roll.m_minAngle == dgFloat32(0.0f)) && (m_roll.m_maxAngle == dgFloat32(0.0f))) {
			const dFloat angleError = GetMaxAngleError();
			const dFloat angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
			NewtonUserJointAddAngularRow(m_joint, angle0, &matrix1.m_up[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			if (dAbs(angle0) > angleError) {
				const dFloat alpha0 = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dFloat(0.25f) * angle0 / (timestep * timestep);
				NewtonUserJointSetRowAcceleration(m_joint, alpha0);
			}

			const dFloat angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
			NewtonUserJointAddAngularRow(m_joint, angle1, &matrix1.m_right[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			if (dAbs(angle1) > angleError) {
				const dFloat alpha1 = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dFloat(0.25f) * angle1 / (timestep * timestep);
				NewtonUserJointSetRowAcceleration(m_joint, alpha1);
			}

		} else {
		
			const dFloat angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
			if (angle0 >= m_yaw.m_maxAngle) {
				NewtonUserJointAddAngularRow(m_joint, angle0, &matrix1.m_up[0]);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				const dFloat angleError = angle0 - m_yaw.m_maxAngle;
				const dFloat alpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dFloat(0.25f) * angleError / (timestep * timestep);
				NewtonUserJointSetRowAcceleration(m_joint, alpha);
				NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
			} else if (angle0 <= m_yaw.m_minAngle) {
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_up[0]);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				const dFloat angleError = angle0 - m_yaw.m_minAngle;
				const dFloat alpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dFloat(0.25f) * angleError / (timestep * timestep);
				NewtonUserJointSetRowAcceleration(m_joint, alpha);
				NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);			
			}

			const dFloat angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
			if (angle1 >= m_roll.m_maxAngle) {
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_right[0]);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				const dFloat angleError = angle1 - m_roll.m_maxAngle;
				const dFloat alpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dFloat(0.25f) * angleError / (timestep * timestep);
				NewtonUserJointSetRowAcceleration(m_joint, alpha);
				NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
			} else if (angle1 <= m_roll.m_minAngle) {
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_right[0]);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				const dFloat angleError = angle1 - m_roll.m_minAngle;
				const dFloat alpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dFloat(0.25f) * angleError / (timestep * timestep);
				NewtonUserJointSetRowAcceleration(m_joint, alpha);
				NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
			}
		}
	}
}

void dCustomSixdof::SubmitAngularAxis(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
{
	dVector lateralDir(matrix1[0].CrossProduct(matrix0[0]));
	dAssert(lateralDir.DotProduct3(lateralDir) > 1.0e-6f);
	lateralDir = lateralDir.Normalize();
	dFloat coneAngle = dAcos(dClamp(matrix1.m_front.DotProduct3(matrix0.m_front), dFloat(-1.0f), dFloat(1.0f)));
	const dMatrix coneRotation(dQuaternion(lateralDir, coneAngle), matrix1.m_posit);
	const dMatrix pitchMatrix(matrix1 * coneRotation * matrix0.Inverse());
	const dFloat pitchAngle = -dAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
	SubmitTwistAngle(matrix0.m_front, pitchAngle, timestep);

	if (m_options.m_option4 || m_options.m_option5) {
		// calculate yaw and roll angle matrix;
		//dMatrix yawRoll(matrix1 * coneRotation * matrix1.Inverse());
		dMatrix yawRoll(pitchMatrix * matrix0 * matrix1.Inverse());
	//	dAssert(dAbs(yawRoll[1][2]) < dFloat(1.0e-1f));
	if (dAbs(yawRoll[1][2]) > 0.6f)
	dTrace(("error %f\n", yawRoll[1][2]));

		dTrace(("error %f\n", yawRoll[1][2]));
	/*
		if (m_options.m_option2) {
			const dFloat step = dMax(dAbs(coneAngle * timestep), dFloat(5.0f * dDegreeToRad));
			if (coneAngle > m_maxConeAngle) {
				dFloat restoringOmega = 0.25f;
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &lateralDir[0]);
				const dFloat invtimestep = 1.0f / timestep;
				const dFloat error0 = coneAngle - m_maxConeAngle;
				const dFloat error1 = error0 - restoringOmega * timestep;
				if (error1 < 0.0f) {
					restoringOmega = error0 * invtimestep;
				}
				const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint) - restoringOmega / timestep;
				NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
				//NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				NewtonUserJointSetRowStiffness(m_joint, m_coneStiffness);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);

				dVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);

			} else if ((coneAngle + step) >= m_maxConeAngle) {
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &lateralDir[0]);
				const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
				NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
				//NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				NewtonUserJointSetRowStiffness(m_joint, m_coneStiffness);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);

				dVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);
			} else if (m_coneFriction > 0.0f) {
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &lateralDir[0]);
				const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
				NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
				//NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				NewtonUserJointSetRowStiffness(m_joint, m_coneStiffness);
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_coneFriction);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);

				dVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_coneFriction);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);
			}
		}
*/
	}
}
