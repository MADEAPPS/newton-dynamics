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



// dCustomSlider.cpp: implementation of the dCustomSlider class.
//
//////////////////////////////////////////////////////////////////////
#include "dCustomJointLibraryStdAfx.h"
#include "dCustomSlider.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

IMPLEMENT_CUSTOM_JOINT(dCustomSlider);

dCustomSlider::dCustomSlider (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(6, child, parent)
	,m_speed(0.0f)
	,m_posit(0.0f)
	,m_minDist(-1.0f)
	,m_maxDist(1.0f)
	,m_friction(0.0f)
	,m_spring(0.0f)
	,m_damper(0.0f)
	,m_springDamperRelaxation(0.6f)
	,m_limitsOn(false)
	,m_setAsSpringDamper(false)
{
	// calculate the two local matrix of the pivot point
	CalculateLocalMatrix (pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
}

dCustomSlider::dCustomSlider (const dMatrix& pinAndPivotFrameChild, const dMatrix& pinAndPivotFrameParent, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(6, child, parent)
	,m_speed(0.0f)
	,m_posit(0.0f)
	,m_minDist(-1.0f)
	,m_maxDist(1.0f)
	,m_spring(0.0f)
	,m_damper(0.0f)
	,m_springDamperRelaxation(0.6f)
	,m_limitsOn(false)
	,m_setAsSpringDamper(false)
{
	dMatrix	dummy;
	CalculateLocalMatrix(pinAndPivotFrameChild, m_localMatrix0, dummy);
	CalculateLocalMatrix(pinAndPivotFrameParent, dummy, m_localMatrix1);
}

void dCustomSlider::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	callback (userData, &m_speed, sizeof (m_speed));
	callback (userData, &m_posit, sizeof (m_posit));
	callback (userData, &m_minDist, sizeof (m_minDist));
	callback (userData, &m_maxDist, sizeof (m_maxDist));
	callback (userData, &m_friction, sizeof (m_friction));
	callback (userData, &m_spring, sizeof (m_spring));
	callback (userData, &m_damper, sizeof (m_damper));
	callback (userData, &m_springDamperRelaxation, sizeof (m_springDamperRelaxation));
	callback (userData, &m_options, sizeof (m_options));
}

dCustomSlider::~dCustomSlider()
{
}

void dCustomSlider::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize (callback, userData);

	callback(userData, &m_speed, sizeof(m_speed));
	callback(userData, &m_posit, sizeof(m_posit));
	callback (userData, &m_minDist, sizeof (m_minDist));
	callback (userData, &m_maxDist, sizeof (m_maxDist));
	callback (userData, &m_friction, sizeof (m_friction));
	callback(userData, &m_spring, sizeof(m_spring));
	callback(userData, &m_damper, sizeof(m_damper));
	callback(userData, &m_springDamperRelaxation, sizeof(m_springDamperRelaxation));
	callback(userData, &m_options, sizeof(m_options));
}


void dCustomSlider::EnableLimits(bool state)
{
	m_limitsOn = state;
}

void dCustomSlider::SetLimits(dFloat minDist, dFloat maxDist)
{
	m_minDist = minDist;
	m_maxDist = maxDist;
}

void dCustomSlider::SetAsSpringDamper(bool state, dFloat springDamperRelaxation, dFloat spring, dFloat damper)
{
	m_spring = spring;
	m_damper = damper;
	m_setAsSpringDamper = state;
	m_springDamperRelaxation = dClamp(springDamperRelaxation, dFloat(0.0f), dFloat(0.99f));
}

dFloat dCustomSlider::GetJointPosit () const
{
	return m_posit;
}

dFloat dCustomSlider::GetJointSpeed () const
{
	return m_speed;
}

void dCustomSlider::SetFriction (dFloat friction)
{
	m_friction = dAbs (friction);
}

dFloat dCustomSlider::GetFriction () const
{
	return m_friction;
}

/*
void dCustomSlider::SubmitConstraints(dFloat timestep, int threadIndex)
{

	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	// Restrict the movement on the pivot point along all tree orthonormal direction
	NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_up[0]);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_right[0]);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);

	// two rows to restrict rotation around around the parent coordinate system
	dMatrix localMatrix(matrix0 * matrix1.Inverse());
	dVector euler0;
	dVector euler1;
	localMatrix.GetEulerAngles(euler0, euler1, m_pitchRollYaw);

	NewtonUserJointAddAngularRow(m_joint, -euler0.m_x, &matrix1.m_front[0]);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	NewtonUserJointAddAngularRow(m_joint, -euler0.m_y, &matrix1.m_up[0]);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	NewtonUserJointAddAngularRow(m_joint, -euler0.m_z, &matrix1.m_right[0]);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);

	dVector veloc0;
	dVector veloc1;
	dAssert(m_body0);
	NewtonBodyGetPointVelocity(m_body0, &matrix0.m_posit[0], &veloc0[0]);
	if (m_body1) {
		NewtonBodyGetPointVelocity(m_body1, &matrix1.m_posit[0], &veloc1[0]);
	}
	m_posit = (matrix0.m_posit - matrix1.m_posit).DotProduct3(matrix1.m_front);
	m_speed = (veloc0 - veloc1).DotProduct3(matrix1.m_front);

	// if limit are enable ...
	if (m_limitsOn && m_setAsSpringDamper) {
		if (m_posit < m_minDist) {
			const dVector& p0 = matrix0.m_posit;
			dVector p1(p0 + matrix0.m_front.Scale(m_minDist - m_posit));
			NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1.m_front[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);

			float cutOffSpeed = 0.5f;
			const dFloat stopAccel = NewtonUserJointCalculateRowZeroAccelaration(m_joint);
			const dFloat speedStep = dAbs(stopAccel * timestep);
			if (speedStep > cutOffSpeed) {
				NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
				NewtonUserJointSetRowMinimumFriction(m_joint, dFloat(0.0f));
			}
		}
		else if (m_posit > m_maxDist) {
			const dVector& p0 = matrix0.m_posit;
			dVector p1(p0 + matrix0.m_front.Scale(m_maxDist - m_posit));
			NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1.m_front[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);

			float cutOffSpeed = 0.5f;
			const dFloat stopAccel = NewtonUserJointCalculateRowZeroAccelaration(m_joint);
			const dFloat speedStep = dAbs(stopAccel * timestep);
			if (speedStep > cutOffSpeed) {
				NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
				NewtonUserJointSetRowMaximumFriction(m_joint, dFloat(0.0f));
			}
		}
		else {
			const dVector& p0 = matrix0.m_posit;
			const dVector& p1 = matrix1.m_posit;
			NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1.m_front[0]);
			NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_springDamperRelaxation, m_spring, m_damper);
		}

	}
	else if (m_limitsOn) {

		if (m_posit < m_minDist) {
			dFloat positError = m_minDist - m_posit;
			NewtonUserJointAddLinearRow(m_joint, &matrix1.m_posit[0], &matrix1.m_posit[0], &matrix1.m_front[0]);
			NewtonUserJointSetRowAcceleration(m_joint, (positError * 5.0f - m_speed) / timestep);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_friction);
		}
		else if (m_posit > m_maxDist) {
			dFloat positError = m_maxDist - m_posit;
			NewtonUserJointAddLinearRow(m_joint, &matrix1.m_posit[0], &matrix1.m_posit[0], &matrix1.m_front[0]);
			NewtonUserJointSetRowAcceleration(m_joint, (positError * 5.0f - m_speed) / timestep);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_friction);
		}
		else if (m_friction != 0.0f) {
			NewtonUserJointAddLinearRow(m_joint, &matrix1.m_posit[0], &matrix1.m_posit[0], &matrix1.m_front[0]);
			NewtonUserJointSetRowAcceleration(m_joint, -m_speed / timestep);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_friction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_friction);
		}
	}
	else if (m_setAsSpringDamper) {
		const dVector& p0 = matrix0.m_posit;
		const dVector& p1 = matrix1.m_posit;
		NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
		NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_springDamperRelaxation, m_spring, m_damper);
	}
}
*/


void dCustomSlider::SubmitConstraintLimits(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
{
	dFloat x = m_posit + m_speed * timestep;
	if (x < m_minDist) {
		NewtonUserJointAddLinearRow(m_joint, &matrix1.m_posit[0], &matrix1.m_posit[0], &matrix1.m_front[0]);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_friction);

		const dFloat invtimestep = 1.0f / timestep;
		const dFloat speed = 0.5f * (m_minDist - m_posit) * invtimestep;
		const dFloat stopAccel = NewtonUserJointCalculateRowZeroAccelaration(m_joint) + speed * invtimestep;
		NewtonUserJointSetRowAcceleration(m_joint, stopAccel);

	} else if (x > m_maxDist) {
		NewtonUserJointAddLinearRow(m_joint, &matrix1.m_posit[0], &matrix1.m_posit[0], &matrix1.m_front[0]);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_friction);

		const dFloat invtimestep = 1.0f / timestep;
		const dFloat speed = 0.5f * (m_maxDist - m_posit) * invtimestep;
		const dFloat stopAccel = NewtonUserJointCalculateRowZeroAccelaration(m_joint) + speed * invtimestep; 
		NewtonUserJointSetRowAcceleration (m_joint, stopAccel);

	} else if (m_friction != 0.0f) {
		NewtonUserJointAddLinearRow(m_joint, &matrix1.m_posit[0], &matrix1.m_posit[0], &matrix1.m_front[0]);
		NewtonUserJointSetRowAcceleration(m_joint, -m_speed / timestep);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_friction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_friction);
	}
}


void dCustomSlider::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	// Restrict the movement on the pivot point along all two orthonormal axis direction perpendicular to the motion
	NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_up[0]);
	NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_right[0]);

	dMatrix localMatrix(matrix0 * matrix1.Inverse());
	dVector euler0;
	dVector euler1;
	localMatrix.GetEulerAngles(euler0, euler1, m_pitchRollYaw);
	for (int i = 0; i < 3; i++) {
		NewtonUserJointAddAngularRow(m_joint, -euler0[i], &matrix1[i][0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	}

	// calculate position and speed	
	dVector veloc0(0.0f);
	dVector veloc1(0.0f);
	dAssert(m_body0);
	NewtonBodyGetPointVelocity(m_body0, &matrix0.m_posit[0], &veloc0[0]);
	if (m_body1) {
		NewtonBodyGetPointVelocity(m_body1, &matrix1.m_posit[0], &veloc1[0]);
	}
	m_posit = (matrix0.m_posit - matrix1.m_posit).DotProduct3(matrix1.m_front);
	m_speed = (veloc0 - veloc1).DotProduct3(matrix1.m_front);

	if (m_limitsOn) {
		if (m_setAsSpringDamper) {
			dAssert(0);
		} else {
			SubmitConstraintLimits(matrix0, matrix1, timestep);
		}
	} else if (m_setAsSpringDamper) {
		SubmitConstraintSpringDamper(matrix0, matrix1, timestep);
	} else if (m_friction != 0.0f) {
		NewtonUserJointAddLinearRow(m_joint, &matrix1.m_posit[0], &matrix1.m_posit[0], &matrix1.m_front[0]);
		NewtonUserJointSetRowAcceleration(m_joint, -m_speed / timestep);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_friction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_friction);
	}
}


void dCustomSlider::SubmitConstraintSpringDamper(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
{
	const dVector& p0 = matrix0.m_posit;
	const dVector& p1 = matrix1.m_posit;
	NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
	NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_springDamperRelaxation, m_spring, m_damper);
}