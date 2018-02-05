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

//********************************************************************
// dCustomCorkScrew.cpp: implementation of the dCustomCorkScrew class.
//
//////////////////////////////////////////////////////////////////////
#include "dCustomJointLibraryStdAfx.h"
#include "dCustomCorkScrew.h"



IMPLEMENT_CUSTOM_JOINT(dCustomCorkScrew);

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define D_CORKSCREW_LIMIT_FLAG			8
#define D_CORKSCREW__SRIND_DAMPER_FLAG	9


dCustomCorkScrew::dCustomCorkScrew (const dMatrix& pinAndPivotFrame, NewtonBody* child, NewtonBody* parent)
	:dCustomSlider(pinAndPivotFrame, child, parent)
	,m_curJointAngle()
	,m_minAngle(-45.0f * dDegreeToRad)
	,m_maxAngle(45.0f * dDegreeToRad)
	,m_angularFriction(0.0f)
	,m_angularOmega(0.0f)
	,m_angularSpring(0.0f)
	,m_angularDamper(0.0f)
	,m_angularSpringDamperRelaxation(0.9f)
{
}

dCustomCorkScrew::dCustomCorkScrew(const dMatrix& pinAndPivotFrameChild, const dMatrix& pinAndPivotFrameParent, NewtonBody* const child, NewtonBody* const parent)
	:dCustomSlider(pinAndPivotFrameChild, pinAndPivotFrameParent, child, parent)
	,m_curJointAngle()
	,m_minAngle(-45.0f * dDegreeToRad)
	,m_maxAngle(45.0f * dDegreeToRad)
	,m_angularFriction(0.0f)
	,m_angularOmega(0.0f)
	,m_angularSpring(0.0f)
	,m_angularDamper(0.0f)
	,m_angularSpringDamperRelaxation(0.9f)
{
}


dCustomCorkScrew::~dCustomCorkScrew()
{
}

void dCustomCorkScrew::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	callback(userData, &m_curJointAngle, sizeof(dAngularIntegration));
	callback(userData, &m_minAngle, sizeof(dFloat));
	callback(userData, &m_maxAngle, sizeof(dFloat));
	callback(userData, &m_angularFriction, sizeof(dFloat));
	callback(userData, &m_angularOmega, sizeof(dFloat));
	callback(userData, &m_angularSpring, sizeof(dFloat));
	callback(userData, &m_angularDamper, sizeof(dFloat));
	callback(userData, &m_angularSpringDamperRelaxation, sizeof(dFloat));
}

void dCustomCorkScrew::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize (callback, userData);

	callback(userData, &m_curJointAngle, sizeof(dAngularIntegration));
	callback(userData, &m_minAngle, sizeof(dFloat));
	callback(userData, &m_maxAngle, sizeof(dFloat));
	callback(userData, &m_angularFriction, sizeof(dFloat));
	callback(userData, &m_angularOmega, sizeof(dFloat));
	callback(userData, &m_angularSpring, sizeof(dFloat));
	callback(userData, &m_angularDamper, sizeof(dFloat));
	callback(userData, &m_angularSpringDamperRelaxation, sizeof(dFloat));
}

void dCustomCorkScrew::EnableAngularLimits(bool state)
{
	m_options = (m_options & ~(1<<D_CORKSCREW_LIMIT_FLAG)) | (int(state) << D_CORKSCREW_LIMIT_FLAG);
}

void dCustomCorkScrew::SetAsSpringDamper(bool state, dFloat springDamperRelaxation, dFloat spring, dFloat damper)
{
	m_spring = spring;
	m_damper = damper;
	m_springDamperRelaxation = dClamp(springDamperRelaxation, dFloat(0.0f), dFloat(0.999f));
	m_options = (m_options & ~(1 << D_CORKSCREW__SRIND_DAMPER_FLAG)) | (int(state) << D_CORKSCREW__SRIND_DAMPER_FLAG);
}

void dCustomCorkScrew::SetAngularLimis(dFloat minDist, dFloat maxDist)
{
	m_minAngle = -dAbs(minDist);
	m_maxAngle = dAbs(maxDist);
}

void dCustomCorkScrew::SubmitConstraintSpringDamper(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
{
	NewtonUserJointAddAngularRow(m_joint, -m_curJointAngle.GetAngle(), &matrix1.m_front[0]);
	NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_angularSpringDamperRelaxation, m_angularSpring, m_angularDamper);
}

void dCustomCorkScrew::SubmitConstraintLimits(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
{
	dFloat angle = m_curJointAngle.GetAngle() + m_angularOmega * timestep;
	if (angle < m_minAngle) {
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_front[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);

		const dFloat invtimestep = 1.0f / timestep;
		const dFloat speed = 0.5f * (m_minAngle - m_curJointAngle.GetAngle()) * invtimestep;
		const dFloat stopAccel = NewtonUserJointCalculateRowZeroAccelaration(m_joint) + speed * invtimestep;
		NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
	} else if (angle > m_maxAngle) {
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_front[0]);
		NewtonUserJointSetRowStiffness(m_joint, 1.0f);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);

		const dFloat invtimestep = 1.0f / timestep;
		const dFloat speed = 0.5f * (m_maxAngle - m_curJointAngle.GetAngle()) * invtimestep;
		const dFloat stopAccel = NewtonUserJointCalculateRowZeroAccelaration(m_joint) + speed * invtimestep;
		NewtonUserJointSetRowAcceleration(m_joint, stopAccel);

	} else if (m_friction != 0.0f) {
		NewtonUserJointAddAngularRow(m_joint, 0, &matrix1.m_front[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowAcceleration(m_joint, -m_angularOmega / timestep);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);
	}
}

void dCustomCorkScrew::SubmitConstraintLimitSpringDamper(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
{
	dFloat angle = m_curJointAngle.GetAngle() + m_angularOmega * timestep;
	if (angle < m_minAngle) {
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_front[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);

		const dFloat invtimestep = 1.0f / timestep;
		const dFloat speed = 0.5f * (m_minAngle - m_curJointAngle.GetAngle()) * invtimestep;
		const dFloat springAccel = NewtonCalculateSpringDamperAcceleration(timestep, m_angularSpring, m_curJointAngle.GetAngle(), m_angularDamper, m_angularOmega);
		const dFloat stopAccel = NewtonUserJointCalculateRowZeroAccelaration(m_joint) + speed * invtimestep + springAccel;
		NewtonUserJointSetRowAcceleration(m_joint, stopAccel);

	} else if (angle > m_maxAngle) {
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_front[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);

		const dFloat invtimestep = 1.0f / timestep;
		const dFloat speed = 0.5f * (m_maxAngle - m_curJointAngle.GetAngle()) * invtimestep;
		const dFloat springAccel = NewtonCalculateSpringDamperAcceleration(timestep, m_angularSpring, m_curJointAngle.GetAngle(), m_angularDamper, m_angularOmega);
		const dFloat stopAccel = NewtonUserJointCalculateRowZeroAccelaration(m_joint) + speed * invtimestep + springAccel;
		NewtonUserJointSetRowAcceleration(m_joint, stopAccel);

	} else {
		dCustomCorkScrew::SubmitConstraintSpringDamper(matrix0, matrix1, timestep);
	}
}


void dCustomCorkScrew::SubmitAngularRow(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
{
	dMatrix localMatrix(matrix0 * matrix1.Inverse());
	dVector euler0;
	dVector euler1;
	localMatrix.GetEulerAngles(euler0, euler1, m_pitchRollYaw);

	NewtonUserJointAddAngularRow(m_joint, -euler0[1], &matrix1[1][0]);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	NewtonUserJointAddAngularRow(m_joint, -euler0[2], &matrix1[2][0]);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);

	// the joint angle can be determined by getting the angle between any two non parallel vectors
	m_curJointAngle.Update(euler0.m_x);

	// save the current joint Omega
	dVector omega0(0.0f);
	dVector omega1(0.0f);
	NewtonBodyGetOmega(m_body0, &omega0[0]);
	if (m_body1) {
		NewtonBodyGetOmega(m_body1, &omega1[0]);
	}
	m_angularOmega = (omega0 - omega1).DotProduct3(matrix1.m_front);

	int limitsOn = m_options & (1 << D_CORKSCREW_LIMIT_FLAG);
	int setAsSpringDamper = m_options & (1 << D_CORKSCREW__SRIND_DAMPER_FLAG);
	if (limitsOn) {
		if (setAsSpringDamper) {
			dCustomCorkScrew::SubmitConstraintLimitSpringDamper(matrix0, matrix1, timestep);
		} else {
			dCustomCorkScrew::SubmitConstraintLimits(matrix0, matrix1, timestep);
		}
	} else if (setAsSpringDamper) {
		dCustomCorkScrew::SubmitConstraintSpringDamper(matrix0, matrix1, timestep);
	} else if (m_friction != 0.0f) {
		NewtonUserJointAddAngularRow(m_joint, 0, &matrix1.m_front[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowAcceleration(m_joint, -m_angularOmega / timestep);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);
	}
}


