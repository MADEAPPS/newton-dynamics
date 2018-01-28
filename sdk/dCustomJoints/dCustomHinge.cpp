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



// dCustomHinge.cpp: implementation of the dCustomHinge class.
//
//////////////////////////////////////////////////////////////////////
#include "dCustomJointLibraryStdAfx.h"
#include "dCustomHinge.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

IMPLEMENT_CUSTOM_JOINT(dCustomHinge);

dCustomHinge::dCustomHinge (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(6, child, parent)
	,m_curJointAngle()
	,m_minAngle(-45.0f * dDegreeToRad)
	,m_maxAngle(45.0f * dDegreeToRad)
	,m_friction(0.0f)
	,m_jointOmega(0.0f)
	,m_spring(0.0f)
	,m_damper(0.0f)
	,m_springDamperRelaxation(0.97f)
	,m_options(0)
{
	m_limitsOn = false;
	m_actuatorFlag = false;
	m_setAsSpringDamper = false;
	SetLimits(-45.0f * dDegreeToRad, 45.0f * dDegreeToRad);
	CalculateLocalMatrix (pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
}

dCustomHinge::dCustomHinge (const dMatrix& pinAndPivotFrameChild, const dMatrix& pinAndPivotFrameParent, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(6, child, parent)
	,m_curJointAngle()
	,m_minAngle(-45.0f * dDegreeToRad)
	,m_maxAngle(45.0f * dDegreeToRad)
	,m_friction(0.0f)
	,m_jointOmega(0.0f)
	,m_spring(0.0f)
	,m_damper(0.0f)
	,m_springDamperRelaxation(0.97f)
	,m_options(0)
{
	dMatrix	dummy;
	m_limitsOn = false;
	m_actuatorFlag = false;
	m_setAsSpringDamper = false;
	SetLimits(-45.0f * dDegreeToRad, 45.0f * dDegreeToRad);
	CalculateLocalMatrix (pinAndPivotFrameChild, m_localMatrix0, dummy);
	CalculateLocalMatrix (pinAndPivotFrameParent, dummy, m_localMatrix1);
}

dCustomHinge::~dCustomHinge()
{
}

void dCustomHinge::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	callback(userData, &m_curJointAngle, sizeof(dAngularIntegration));
	callback (userData, &m_minAngle, sizeof (dFloat));
	callback (userData, &m_maxAngle, sizeof (dFloat));
	callback (userData, &m_friction, sizeof (m_friction));
	callback (userData, &m_jointOmega, sizeof (m_jointOmega));
	callback (userData, &m_spring, sizeof (m_spring));
	callback (userData, &m_damper, sizeof (m_damper));
	callback (userData, &m_springDamperRelaxation, sizeof (m_springDamperRelaxation));
	callback(userData, &m_options, sizeof(m_options));
}

void dCustomHinge::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize (callback, userData);
	callback(userData, &m_curJointAngle, sizeof(dAngularIntegration));
	callback (userData, &m_minAngle, sizeof (dFloat));
	callback (userData, &m_maxAngle, sizeof (dFloat));
	callback(userData, &m_friction, sizeof(m_friction));
	callback(userData, &m_jointOmega, sizeof(m_jointOmega));
	callback(userData, &m_spring, sizeof(m_spring));
	callback(userData, &m_damper, sizeof(m_damper));
	callback(userData, &m_springDamperRelaxation, sizeof(m_springDamperRelaxation));
	callback(userData, &m_options, sizeof(m_options));
}


void dCustomHinge::EnableLimits(bool state)
{
	m_limitsOn = state;
}

void dCustomHinge::SetLimits(dFloat minAngle, dFloat maxAngle)
{
	m_minAngle = minAngle;
	m_maxAngle = maxAngle;
}

void dCustomHinge::SetAsSpringDamper(bool state, dFloat springDamperRelaxation, dFloat spring, dFloat damper)
{
	m_setAsSpringDamper = state;
	m_spring = spring;
	m_damper = damper;
	m_springDamperRelaxation = dClamp(springDamperRelaxation, dFloat(0.0f), dFloat(0.999f));
}

dFloat dCustomHinge::GetJointAngle () const
{
	return m_curJointAngle.GetAngle();
}

dVector dCustomHinge::GetPinAxis () const
{
	dMatrix matrix;
	NewtonBodyGetMatrix (m_body0, &matrix[0][0]);
	return matrix.RotateVector (m_localMatrix0.m_front);
}

dFloat dCustomHinge::GetJointOmega () const
{
	return m_jointOmega;
}

void dCustomHinge::SetFriction (dFloat frictionTorque)
{
	m_friction = frictionTorque;
}

dFloat dCustomHinge::GetFriction () const
{
	return m_friction;
}

void dCustomHinge::SubmitConstraintsLimitsOnly(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
{
	dFloat angle = m_curJointAngle.GetAngle();
	if (angle < m_minAngle) {
		dFloat relAngle = m_minAngle - angle;
		NewtonUserJointAddAngularRow(m_joint, relAngle, &matrix0.m_front[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
	} else if (angle > m_maxAngle) {
		dFloat relAngle = m_maxAngle - angle;
		NewtonUserJointAddAngularRow(m_joint, relAngle, &matrix0.m_front[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
	}
}

void dCustomHinge::SubmitConstraintsFrictionOnly(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
{
	dFloat alpha = m_jointOmega / timestep;
	NewtonUserJointAddAngularRow(m_joint, 0, &matrix1.m_front[0]);
	NewtonUserJointSetRowAcceleration(m_joint, -alpha);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	NewtonUserJointSetRowMinimumFriction(m_joint, -m_friction);
	NewtonUserJointSetRowMaximumFriction(m_joint, m_friction);
}

void dCustomHinge::SubmitConstraintsFrictionAndLimit(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
{
	dFloat angle = m_curJointAngle.GetAngle();
	if (angle < m_minAngle) {
		dFloat relAngle = m_minAngle - angle;
		NewtonUserJointAddAngularRow(m_joint, relAngle, &matrix0.m_front[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
	} else if (angle > m_maxAngle) {
		dFloat relAngle = m_maxAngle - angle;
		NewtonUserJointAddAngularRow(m_joint, relAngle, &matrix0.m_front[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
	} else {
		// friction but not limits
		dFloat alpha = m_jointOmega / timestep;
		NewtonUserJointAddAngularRow(m_joint, 0, &matrix1.m_front[0]);
		NewtonUserJointSetRowAcceleration(m_joint, -alpha);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_friction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_friction);
	}
}

void dCustomHinge::Debug(dDebugDisplay* const debugDisplay) const
{
	dCustomJoint::Debug(debugDisplay);

	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	const int subdiv = 12;
	dVector arch[subdiv + 1];
	const float radius = debugDisplay->m_debugScale;

	if ((m_maxAngle > 1.0e-3f) || (m_minAngle < -1.0e-3f)) {
		// show pitch angle limits
		dVector point(dFloat(0.0f), dFloat(radius), dFloat(0.0f), dFloat(0.0f));

		dFloat minAngle = m_minAngle;
		dFloat maxAngle = m_maxAngle;
		if ((maxAngle - minAngle) >= dPi * 2.0f) {
			minAngle = 0.0f;
			maxAngle = dPi * 2.0f;
		}

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

void dCustomHinge::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	// Restrict the movement on the pivot point along all tree orthonormal direction
	NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_front[0]);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_up[0]);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_right[0]);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);

	// two rows to restrict rotation around around the parent coordinate system
	dMatrix localMatrix(matrix0 * matrix1.Inverse());
	dVector euler0;
	dVector euler1;
	localMatrix.GetEulerAngles(euler0, euler1, m_pitchRollYaw);

	NewtonUserJointAddAngularRow(m_joint, -euler0.m_y, &matrix1.m_up[0]);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	NewtonUserJointAddAngularRow(m_joint, -euler0.m_z, &matrix1.m_right[0]);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);


	dVector omega0(0.0f);
	dVector omega1(0.0f);
	NewtonBodyGetOmega(m_body0, &omega0[0]);
	if (m_body1) {
		NewtonBodyGetOmega(m_body1, &omega1[0]);
	}
	m_jointOmega = (omega0 - omega1).DotProduct3(matrix1.m_front);

	//m_friction = 0;
	//m_limitsOn = false;
	//m_setAsSpringDamper = 1;
	//m_spring = 1000;
	//m_damper = 10.0f;
	//m_springDamperRelaxation = 0.97f;

	m_curJointAngle.Update(euler0.m_x);
	if (m_setAsSpringDamper) {
		NewtonUserJointAddAngularRow(m_joint, -m_curJointAngle.GetAngle(), &matrix0.m_front[0]);
		NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_springDamperRelaxation, m_spring, m_damper);
	} else {
		if (m_limitsOn) {
			if (m_friction != 0.0f) {
				SubmitConstraintsFrictionAndLimit(matrix0, matrix1, timestep);
			} else {
				SubmitConstraintsLimitsOnly(matrix0, matrix1, timestep);
			}
		} else {
			if (m_friction != 0.0f) {
				SubmitConstraintsFrictionOnly(matrix0, matrix1, timestep);
			}
		}
	}
}