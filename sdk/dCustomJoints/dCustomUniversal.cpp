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


// dCustomUniversal.cpp: implementation of the dCustomUniversal class.
//
//////////////////////////////////////////////////////////////////////
#include "dCustomJointLibraryStdAfx.h"
#include "dCustomUniversal.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//dInitRtti(dCustomUniversal);
IMPLEMENT_CUSTOM_JOINT(dCustomUniversal);

dCustomUniversal::dCustomUniversal(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustom6dof(pinAndPivotFrame, child, parent)
	,m_options(0)
{
	m_yawAxis = 0;
//	m_rollAxis = 0;
	m_pitchAxis = 0;

	m_actuator_0 = false;
	m_actuator_1 = false;

	m_limit_0_On = true;
	m_angularMotor_0_On = false;
	m_angularDamp_0 = 0.5f;
	m_angularAccel_0 = -4.0f;
	m_jointOmega_0 = 0.0f;
	SetPitchLimits(-45.0f * dDegreeToRad, 45.0f * dDegreeToRad);

	m_limit_1_On = true;
	m_angularMotor_1_On = false; 
	m_angularDamp_1 = 0.3f;
	m_angularAccel_1 = -4.0f;
	m_jointOmega_1 = 0.0f;
	SetYawLimits(-45.0f * dDegreeToRad, 45.0f * dDegreeToRad);
}

dCustomUniversal::dCustomUniversal(const dMatrix& pinAndPivotFrameChild, const dMatrix& pinAndPivotFrameParent, NewtonBody* const child, NewtonBody* const parent)
	:dCustom6dof(pinAndPivotFrameChild, pinAndPivotFrameParent, child, parent)
	,m_options(0)
{
	m_yawAxis = 0;
//	m_rollAxis = 0;
	m_pitchAxis = 0;

	m_actuator_0 = false;
	m_actuator_1 = false;

	m_limit_0_On = true;
	m_angularMotor_0_On = false;
	m_angularDamp_0 = 0.5f;
	m_angularAccel_0 = -4.0f;
	m_jointOmega_0 = 0.0f;
	SetPitchLimits(-45.0f * dDegreeToRad, 45.0f * dDegreeToRad);

	m_limit_1_On = true;
	m_angularMotor_1_On = false;
	m_angularDamp_1 = 0.3f;
	m_angularAccel_1 = -4.0f;
	m_jointOmega_1 = 0.0f;
	SetYawLimits(-45.0f * dDegreeToRad, 45.0f * dDegreeToRad);
}


void dCustomUniversal::Deserialize (NewtonDeserializeCallback callback, void* const userData) 
{
	callback(userData, &m_jointOmega_0, sizeof(m_jointOmega_0));
	callback(userData, &m_angularDamp_0, sizeof(m_angularDamp_0));
	callback(userData, &m_angularAccel_0, sizeof(m_angularAccel_0));

	callback(userData, &m_jointOmega_1, sizeof(m_jointOmega_1));
	callback(userData, &m_angularDamp_1, sizeof(m_angularDamp_1));
	callback(userData, &m_angularAccel_1, sizeof(m_angularAccel_1));

	callback(userData, &m_options, sizeof(m_options));
}

void dCustomUniversal::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize(callback, userData);

	callback(userData, &m_jointOmega_0, sizeof(m_jointOmega_0));
	callback(userData, &m_angularDamp_0, sizeof(m_angularDamp_0));
	callback(userData, &m_angularAccel_0, sizeof(m_angularAccel_0));

	callback(userData, &m_jointOmega_1, sizeof(m_jointOmega_1));
	callback(userData, &m_angularDamp_1, sizeof(m_angularDamp_1));
	callback(userData, &m_angularAccel_1, sizeof(m_angularAccel_1));

	callback(userData, &m_options, sizeof(m_options));
}

dCustomUniversal::~dCustomUniversal()
{
}


void dCustomUniversal::EnableLimit_0(bool state)
{
	m_limit_0_On = state;
}

void dCustomUniversal::EnableLimit_1(bool state)
{
	m_limit_1_On = state;
}

void dCustomUniversal::EnableMotor_0(bool state)
{
	m_angularMotor_0_On = state;
}

void dCustomUniversal::EnableMotor_1(bool state)
{
	m_angularMotor_1_On = state;
}

void dCustomUniversal::SetAccel_0(dFloat accel)
{
	m_angularAccel_0 = accel;
}

void dCustomUniversal::SetDamp_0(dFloat damp)
{
	m_angularDamp_0 = damp;
}


void dCustomUniversal::SetAccel_1(dFloat accel)
{
	m_angularAccel_1 = accel;
}

void dCustomUniversal::SetDamp_1(dFloat damp)
{
	m_angularDamp_1 = damp;
}

void dCustomUniversal::SetLimits_0(dFloat minAngle, dFloat maxAngle)
{
	dAssert (minAngle < 0.0f);
	dAssert (maxAngle > 0.0f);
	SetPitchLimits(minAngle, maxAngle);
}

dFloat dCustomUniversal::GetMinAngularLimit_0() const
{
	dFloat minAngle;
	dFloat maxAngle;
	GetPitchLimits(minAngle, maxAngle);
	return minAngle;
}

dFloat dCustomUniversal::GetMaxAngularLimit_0() const
{
	dFloat minAngle;
	dFloat maxAngle;
	GetPitchLimits(minAngle, maxAngle);
	return maxAngle;
}

void dCustomUniversal::SetLimits_1(dFloat minAngle, dFloat maxAngle)
{
	dAssert (minAngle < 0.0f);
	dAssert (maxAngle > 0.0f);
	SetYawLimits(minAngle, maxAngle);
}

dFloat dCustomUniversal::GetMinAngularLimit_1() const
{
	dFloat minAngle;
	dFloat maxAngle;
	GetYawLimits(minAngle, maxAngle);
	return minAngle;
}

dFloat dCustomUniversal::GetMaxAngularLimit_1() const
{
	dFloat minAngle;
	dFloat maxAngle;
	GetYawLimits(minAngle, maxAngle);
	return maxAngle;
}


dFloat dCustomUniversal::GetJointAngle_0 () const
{
	dAssert(0);
	return 0;
//	return -m_curJointAngle_0.GetAngle();
}

dFloat dCustomUniversal::GetJointAngle_1 () const
{
	dAssert(0);
	return 0;
//	return -m_curJointAngle_1.GetAngle();
}

dFloat dCustomUniversal::GetJointOmega_0 () const
{
	return m_jointOmega_0;
}

dFloat dCustomUniversal::GetJointOmega_1 () const
{
	return m_jointOmega_1;
}

dVector dCustomUniversal::GetPinAxis_0 () const
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix (matrix0, matrix1);
	return matrix0.m_front;
}

dVector dCustomUniversal::GetPinAxis_1 () const
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix (matrix0, matrix1);
	return matrix1.m_up;
}

#if 0
void dCustomUniversal::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	// Restrict the movement on the pivot point along all tree orthonormal direction
	NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_front[0]);
	NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_up[0]);
	NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_right[0]);

	// construct an orthogonal coordinate system with these two vectors
	dMatrix matrix1_1;
	matrix1_1.m_up = matrix1.m_up;
	matrix1_1.m_right = matrix0.m_front.CrossProduct(matrix1.m_up);
	dAssert (matrix1_1.m_right.DotProduct3(matrix1_1.m_right) > 0.0f);
	matrix1_1.m_right = matrix1_1.m_right.Scale (1.0f / dSqrt (matrix1_1.m_right.DotProduct3(matrix1_1.m_right)));
	matrix1_1.m_front = matrix1_1.m_up.CrossProduct(matrix1_1.m_right);
	

	// override the normal right side  because the joint is too week due to centripetal accelerations
	dVector omega0(0.0f);
	dVector omega1(0.0f);
	NewtonBodyGetOmega(m_body0, &omega0[0]);
	if (m_body1) {
		NewtonBodyGetOmega(m_body1, &omega1[0]);
	}
	dVector relOmega(omega0 - omega1);

	dFloat angle = -CalculateAngle(matrix0.m_front, matrix1_1.m_front, matrix1_1.m_right);
	dFloat omega = (relOmega.DotProduct3(matrix1_1.m_right));
	dFloat alphaError = -(angle + omega * timestep) / (timestep * timestep);
	
	NewtonUserJointAddAngularRow (m_joint, -angle, &matrix1_1.m_right[0]);
	NewtonUserJointSetRowAcceleration (m_joint, alphaError);
	NewtonUserJointSetRowStiffness (m_joint, 1.0f);

	dFloat sinAngle_0;
	dFloat cosAngle_0;
	CalculateAngle (matrix1_1.m_up, matrix0.m_up, matrix1_1.m_front, sinAngle_0, cosAngle_0);
	dFloat angle0 = -m_curJointAngle_0.Update (cosAngle_0, sinAngle_0);

	dFloat sinAngle_1;
	dFloat cosAngle_1;
	CalculateAngle(matrix1.m_front, matrix1_1.m_front, matrix1_1.m_up, sinAngle_1, cosAngle_1);
	dFloat angle1 = -m_curJointAngle_1.Update (cosAngle_1, sinAngle_1);

	// calculate the desired acceleration
	m_jointOmega_0 = relOmega.DotProduct3(matrix0.m_front);
	m_jointOmega_1 = relOmega.DotProduct3(matrix1.m_up);
	
	// check is the joint limit are enable
	if (m_limit_0_On) {
		if (angle0 < m_minAngle_0) {
			dFloat relAngle = angle0 - m_minAngle_0;

			// tell joint error will minimize the exceeded angle error
			NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix0.m_front[0]);

			// need high stiffeners here
			NewtonUserJointSetRowStiffness (m_joint, 1.0f);

			// allow the joint to move back freely
			NewtonUserJointSetRowMaximumFriction (m_joint, 0.0f);

		} else if (angle0 > m_maxAngle_0) {
			dFloat relAngle = angle0 - m_maxAngle_0;

			// tell joint error will minimize the exceeded angle error
			NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix0.m_front[0]);

			// need high stiffness here
			NewtonUserJointSetRowStiffness (m_joint, 1.0f);

			// allow the joint to move back freely 
			NewtonUserJointSetRowMinimumFriction (m_joint, 0.0f);
		}

		// check is the joint limit motor is enable
	} else if (m_angularMotor_0_On) {
		// calculate the desired acceleration
//		dFloat relOmega = (omega0 - omega1) % matrix0.m_front;
		dFloat relAccel = m_angularAccel_0 - m_angularDamp_0 * m_jointOmega_0;

		// add and angular constraint row to that will set the relative acceleration to zero
		NewtonUserJointAddAngularRow (m_joint, 0.0f, &matrix0.m_front[0]);

		// override the joint acceleration.
		NewtonUserJointSetRowAcceleration (m_joint, relAccel);
	}

	// if limit are enable ...
	if (m_limit_1_On) {
		if (angle1 < m_minAngle_1) {
			dFloat relAngle = angle1 - m_minAngle_1;

			// tell joint error will minimize the exceeded angle error
			NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix1.m_up[0]);

			// need high stiffeners here
			NewtonUserJointSetRowStiffness (m_joint, 1.0f);

			// allow the joint to move back freely 
			NewtonUserJointSetRowMaximumFriction (m_joint, 0.0f);

		} else if (angle1 > m_maxAngle_1) {
			dFloat relAngle = angle1 - m_maxAngle_1;
			
			// tell joint error will minimize the exceeded angle error
			NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix1.m_up[0]);

			// need high stiffness here
			NewtonUserJointSetRowStiffness (m_joint, 1.0f);

			// allow the joint to move back freely
			NewtonUserJointSetRowMinimumFriction (m_joint, 0.0f);
  		}
	} else if (m_angularMotor_1_On) {
		// calculate the desired acceleration
		dFloat relAccel = m_angularAccel_1 - m_angularDamp_1 * m_jointOmega_1;

		// add and angular constraint row to that will set the relative acceleration to zero
		NewtonUserJointAddAngularRow (m_joint, 0.0f, &matrix1.m_up[0]);
		
		// override the joint acceleration.
		NewtonUserJointSetRowAcceleration (m_joint, relAccel);
	}
}

#endif


void dCustomUniversal::SubmitConstraintsFreeDof(int freeDof, const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep, int threadIndex)
{
	dAssert(freeDof == 2);

	dMatrix rollMatrix (dYawMatrix(GetYaw()) * matrix1);
	dVector omega0;
	dVector omega1;
	NewtonBodyGetOmega(m_body0, &omega0[0]);
	if (m_body1) {
		NewtonBodyGetOmega(m_body1, &omega1[0]);
	}
	dVector relOmega(omega0 - omega1);
/*
	dFloat rollAngle = GetRoll();
	dFloat rollOmega = (relOmega.DotProduct3(rollMatrix.m_right));
	dFloat alphaRollError = - (rollAngle + rollOmega * timestep) / (timestep * timestep);
	NewtonUserJointAddAngularRow(m_joint, -rollAngle, &rollMatrix.m_right[0]);
	NewtonUserJointSetRowAcceleration(m_joint, alphaRollError);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
*/
	// check is the joint limit are enable
	if (m_limit_0_On) {
		dFloat angle = GetPitch();
		if (angle < m_pitch.m_minAngle) {
			dFloat relAngle = m_pitch.m_minAngle - angle;
			NewtonUserJointAddAngularRow(m_joint, relAngle, &matrix0.m_front[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
		} else if (angle > m_pitch.m_maxAngle) {
			dFloat relAngle = m_pitch.m_maxAngle - angle;
			NewtonUserJointAddAngularRow(m_joint, relAngle, &matrix0.m_front[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
		} else if (m_angularMotor_0_On) {
			dAssert (0);
		}
	
	} else if (m_angularMotor_0_On) {
		dAssert (0);
/*
		// calculate the desired acceleration
		// dFloat relOmega = (omega0 - omega1) % matrix0.m_front;
		dFloat relAccel = m_angularAccel_0 - m_angularDamp_0 * m_jointOmega_0;

		// add and angular constraint row to that will set the relative acceleration to zero
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_front[0]);

		// override the joint acceleration.
		NewtonUserJointSetRowAcceleration(m_joint, relAccel);
*/
	}


	// if limit are enable ...
	if (m_limit_1_On) {
		dFloat angle = GetYaw();
		if (angle < m_yaw.m_minAngle) {
			dFloat relAngle = m_yaw.m_minAngle - angle;
			NewtonUserJointAddAngularRow(m_joint, relAngle, &matrix1.m_up[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
		} else if (angle > m_yaw.m_maxAngle) {
			dFloat relAngle = m_yaw.m_maxAngle - angle;
			NewtonUserJointAddAngularRow(m_joint, relAngle, &matrix1.m_up[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
		} else if (m_angularMotor_1_On) {
			dAssert (0);
		}
	} else if (m_angularMotor_1_On) {
		dAssert (0);
/*
		// calculate the desired acceleration
		dFloat relAccel = m_angularAccel_1 - m_angularDamp_1 * m_jointOmega_1;

		// add and angular constraint row to that will set the relative acceleration to zero
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_up[0]);

		// override the joint acceleration.
		NewtonUserJointSetRowAcceleration(m_joint, relAccel);
*/
	}
}