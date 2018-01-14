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



// dCustomSlidingContact.cpp: implementation of the dCustomSlidingContact class.
//
//////////////////////////////////////////////////////////////////////
#include "dCustomJointLibraryStdAfx.h"
#include "dCustomSlidingContact.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

IMPLEMENT_CUSTOM_JOINT(dCustomSlidingContact);

dCustomSlidingContact::dCustomSlidingContact (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustom6dof(pinAndPivotFrame, child, parent)
	,m_speed(0.0f)
	,m_posit(0.0f)
	,m_flags(0)
{
	m_yAxis = 0;
	m_pitchAxis = 0;

	EnableLinearLimits(false);
	EnableAngularLimits(false);
	SetLinearLimits(-1.0f, 1.0f);
	SetAngularLimits(-30.0f * 3.141592f / 180.0f, 30.0f * 3.141592f / 180.0f);
	SetAsSpringDamper(false, 1.0f, 0.0f, 0.0f);
}

dCustomSlidingContact::dCustomSlidingContact(const dMatrix& pinAndPivotFrameChild, const dMatrix& pinAndPivotFrameParent, NewtonBody* const child, NewtonBody* const parent)
	:dCustom6dof(pinAndPivotFrameChild, pinAndPivotFrameParent, child, parent)
	,m_speed(0.0f)
	,m_posit(0.0f)
	,m_flags(0)
{
	m_yAxis = 0;
	m_pitchAxis = 0;

	EnableLinearLimits(false);
	EnableAngularLimits(false);
	SetLinearLimits(-1.0f, 1.0f);
	SetAngularLimits(-30.0f * 3.141592f / 180.0f, 30.0f * 3.141592f / 180.0f);
	SetAsSpringDamper(false, 1.0f, 0.0f, 0.0f);
}


void dCustomSlidingContact::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	callback(userData, &m_speed, sizeof(dFloat));
	callback(userData, &m_posit, sizeof(dFloat));
	callback(userData, &m_spring, sizeof(dFloat));
	callback(userData, &m_damper, sizeof(dFloat));
	callback(userData, &m_springDamperRelaxation, sizeof(dFloat));
	callback(userData, &m_flags, sizeof(int));
}

void dCustomSlidingContact::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize(callback, userData);

	callback(userData, &m_speed, sizeof(dFloat));
	callback(userData, &m_posit, sizeof(dFloat));
	callback(userData, &m_spring, sizeof(dFloat));
	callback(userData, &m_damper, sizeof(dFloat));
	callback(userData, &m_springDamperRelaxation, sizeof(dFloat));
	callback(userData, &m_flags, sizeof(int));
}


dCustomSlidingContact::~dCustomSlidingContact()
{
}

void dCustomSlidingContact::EnableLinearLimits(bool state)
{
	m_limitsLinearOn = state;
}

void dCustomSlidingContact::EnableAngularLimits(bool state)
{
	m_limitsAngularOn = state;
}

void dCustomSlidingContact::SetLinearLimits(dFloat minDist, dFloat maxDist)
{
	dVector minLinearLimits;
	dVector maxLinearLimits;

	dCustom6dof::GetLinearLimits(minLinearLimits, maxLinearLimits);
	minLinearLimits.m_y = -dAbs (minDist);
	maxLinearLimits.m_y = dAbs(maxDist);
	dCustom6dof::SetLinearLimits(minLinearLimits, maxLinearLimits);
}

void dCustomSlidingContact::SetAngularLimits(dFloat minDist, dFloat maxDist)
{
	SetPitchLimits(minDist, maxDist);
}

void dCustomSlidingContact::SetAsSpringDamper(bool state, dFloat springDamperRelaxation, dFloat spring, dFloat damper)
{
	m_spring = spring;
	m_damper = damper;
	m_setAsSpringDamper = state;
	m_springDamperRelaxation = dClamp(springDamperRelaxation, dFloat(0.0f), dFloat(0.99f));
}


dFloat dCustomSlidingContact::GetPosition() const
{
	return m_posit;
}

dFloat dCustomSlidingContact::GetSpeed() const
{
	return m_speed;
}

#if 0
void dCustomSlidingContact::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dAssert (0);
	dMatrix matrix0;
	dMatrix matrix1;
	dFloat sinAngle;
	dFloat cosAngle;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	// Restrict the movement on the pivot point along all two orthonormal axis direction perpendicular to the motion
	dVector p0(matrix0.m_posit);
	dVector p1(matrix1.m_posit + matrix1.m_front.Scale((p0 - matrix1.m_posit).DotProduct3(matrix1.m_front)));
	NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1.m_up[0]);
	NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1.m_right[0]);

	// construct an orthogonal coordinate system with these two vectors
	dMatrix matrix1_1;
	matrix1_1.m_up = matrix1.m_up;
	matrix1_1.m_right = matrix0.m_front.CrossProduct(matrix1.m_up);
	matrix1_1.m_right = matrix1_1.m_right.Scale(1.0f / dSqrt(matrix1_1.m_right.DotProduct3(matrix1_1.m_right)));
	matrix1_1.m_front = matrix1_1.m_up.CrossProduct(matrix1_1.m_right);
	NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_up, matrix1_1.m_up, matrix1_1.m_front), &matrix1_1.m_front[0]);
	NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_up, matrix1_1.m_up, matrix1_1.m_right), &matrix1_1.m_right[0]);

	// the joint angle can be determined by getting the angle between any two non parallel vectors
	CalculateAngle(matrix1_1.m_front, matrix1.m_front, matrix1.m_up, sinAngle, cosAngle);
	m_curJointAngle.Update(cosAngle, sinAngle);

	dVector veloc0(0.0f);
	dVector veloc1(0.0f);
	dAssert(m_body0);
	NewtonBodyGetPointVelocity(m_body0, &matrix0.m_posit[0], &veloc0[0]);
	if (m_body1) {
		NewtonBodyGetPointVelocity(m_body1, &matrix1.m_posit[0], &veloc1[0]);
	}
	m_posit = (matrix0.m_posit - matrix1.m_posit).DotProduct3(matrix1.m_front);
	m_speed = (veloc0 - veloc1).DotProduct3(matrix1.m_front);
	
	// if limit are enable ...
	if (m_limitsAngularOn) {
		dFloat angle1 = m_curJointAngle.GetAngle();
		if (angle1 < m_minAngularDist) {
			dFloat relAngle = angle1 - m_minAngularDist;

			// tell joint error will minimize the exceeded angle error
			NewtonUserJointAddAngularRow(m_joint, relAngle, &matrix1.m_up[0]);

			// need high stiffeners here
			NewtonUserJointSetRowStiffness(m_joint, 1.0f);

			// allow the joint to move back freely 
			NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);

		} else if (angle1 > m_maxAngularDist) {
			dFloat relAngle = angle1 - m_maxAngularDist;

			// tell joint error will minimize the exceeded angle error
			NewtonUserJointAddAngularRow(m_joint, relAngle, &matrix1.m_up[0]);

			// need high stiffness here
			NewtonUserJointSetRowStiffness(m_joint, 1.0f);

			// allow the joint to move back freely
			NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
		}
	}

	if (m_setAsSpringDamper && m_limitsLinearOn) {
		if (m_posit < m_minLinearDist) {
//			const dVector& p0 = matrix0.m_posit;
//			dVector p1(p0 + matrix0.m_front.Scale(m_minLinearDist - m_posit));
//			NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
//			dFloat accel = NewtonUserJointGetRowAcceleration(m_joint) + NewtonCalculateSpringDamperAcceleration(timestep, m_spring, m_posit, m_damper, m_speed);
//			NewtonUserJointSetRowAcceleration(m_joint, accel);

			// this is not correct but it work for now
			NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_front[0]);
			NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_springDamperRelaxation, m_spring, m_damper);
//			NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
		} else if (m_posit > m_maxLinearDist) {
//			const dVector& p0 = matrix0.m_posit;
//			dVector p1(p0 + matrix0.m_front.Scale(m_maxLinearDist - m_posit));
//			NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
//			dFloat accel = NewtonUserJointGetRowAcceleration(m_joint) + NewtonCalculateSpringDamperAcceleration(timestep, m_spring, m_posit, m_damper, m_speed);
//			NewtonUserJointSetRowAcceleration(m_joint, accel);
//			NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
			// this is not correct but it work for now
			NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_front[0]);
			NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_springDamperRelaxation, m_spring, m_damper);
//			NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
		} else {
			NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_front[0]);
			NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_springDamperRelaxation, m_spring, m_damper);
		}
	} else if (m_limitsLinearOn) {
		if (m_posit < m_minLinearDist) {
			dVector p(matrix1.m_posit + matrix1.m_front.Scale(m_minLinearDist));
			NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &p[0], &matrix1.m_front[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
		} else if (m_posit > m_maxLinearDist) {
			dVector p(matrix1.m_posit + matrix1.m_front.Scale(m_maxLinearDist));
			NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &p[0], &matrix1.m_front[0]);
			NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
		}
	} else if (m_setAsSpringDamper) {
		NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_front[0]);
		NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_springDamperRelaxation, m_spring, m_damper);
	}
}
#endif


void dCustomSlidingContact::SubmitConstraintsFreeDof(int freeDof, const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep, int threadIndex)
{
	dAssert(freeDof == 2);

	// if limit are enable ...
	if (m_limitsAngularOn) {
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
		}
	}

	m_posit = (matrix0.m_posit - matrix1.m_posit).DotProduct3(matrix1.m_up);
	if (m_setAsSpringDamper && m_limitsLinearOn) {
		if (m_posit < m_minLinearLimits.m_y) {
			//const dVector& p0 = matrix0.m_posit;
			//dVector p1(p0 + matrix0.m_front.Scale(m_minLinearDist - m_posit));
			//NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_up[0]);
			//dFloat accel = NewtonUserJointGetRowAcceleration(m_joint) + NewtonCalculateSpringDamperAcceleration(timestep, m_spring, m_posit, m_damper, m_speed);
			//NewtonUserJointSetRowAcceleration(m_joint, accel);

			// this is not correct but it work for now
			NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_up[0]);
			NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_springDamperRelaxation, m_spring, m_damper);
			//NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
		} else if (m_posit > m_maxLinearLimits.m_y) {
			//const dVector& p0 = matrix0.m_posit;
			//dVector p1(p0 + matrix0.m_front.Scale(m_maxLinearDist - m_posit));
			//NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
			//dFloat accel = NewtonUserJointGetRowAcceleration(m_joint) + NewtonCalculateSpringDamperAcceleration(timestep, m_spring, m_posit, m_damper, m_speed);
			//NewtonUserJointSetRowAcceleration(m_joint, accel);
			//NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
			// this is not correct but it work for now
			NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_up[0]);
			NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_springDamperRelaxation, m_spring, m_damper);
			//NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
		} else {
			NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_up[0]);
			NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_springDamperRelaxation, m_spring, m_damper);
		}
	} else if (m_limitsLinearOn) {
		if (m_posit < m_minLinearLimits.m_y) {
			dVector p(matrix1.m_posit + matrix1.m_up.Scale(m_minLinearLimits.m_y));
			NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &p[0], &matrix1.m_up[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
		} else if (m_posit > m_maxLinearLimits.m_y) {
			dVector p(matrix1.m_posit + matrix1.m_up.Scale(m_maxLinearLimits.m_y));
			NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &p[0], &matrix1.m_up[0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
		}
	} else if (m_setAsSpringDamper) {
		NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_up[0]);
		NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_springDamperRelaxation, m_spring, m_damper);
	}
}
