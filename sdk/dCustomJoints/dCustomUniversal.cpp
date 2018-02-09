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
	:dCustomHinge(pinAndPivotFrame, child, parent)
	,m_curJointAngle2()
	,m_minAngle2(-45.0f * dDegreeToRad)
	,m_maxAngle2(45.0f * dDegreeToRad)
	,m_friction2(0.0f)
	,m_jointOmega2(0.0f)
	,m_spring2(0.0f)
	,m_damper2(0.0f)
	,m_springDamperRelaxation2(0.9f)
{
	m_options.m_option4 = -1;
}


dCustomUniversal::dCustomUniversal(const dMatrix& pinAndPivotFrameChild, const dMatrix& pinAndPivotFrameParent, NewtonBody* const child, NewtonBody* const parent)
	:dCustomHinge(pinAndPivotFrameChild, pinAndPivotFrameParent, child, parent)
	,m_curJointAngle2()
	,m_minAngle2(-45.0f * dDegreeToRad)
	,m_maxAngle2(45.0f * dDegreeToRad)
	,m_friction2(0.0f)
	,m_jointOmega2(0.0f)
	,m_spring2(0.0f)
	,m_damper2(0.0f)
	,m_springDamperRelaxation2(0.9f)
{
	m_options.m_option4 = -1;
}

dCustomUniversal::~dCustomUniversal()
{
}

void dCustomUniversal::Deserialize(NewtonDeserializeCallback callback, void* const userData)
{
	callback(userData, &m_curJointAngle2, sizeof(dAngularIntegration));
	callback(userData, &m_minAngle2, sizeof(dFloat));
	callback(userData, &m_maxAngle2, sizeof(dFloat));
	callback(userData, &m_friction2, sizeof(dFloat));
	callback(userData, &m_jointOmega2, sizeof(dFloat));
	callback(userData, &m_spring2, sizeof(dFloat));
	callback(userData, &m_damper2, sizeof(dFloat));
	callback(userData, &m_springDamperRelaxation2, sizeof(dFloat));
}

void dCustomUniversal::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize(callback, userData);
	callback(userData, &m_curJointAngle2, sizeof(dAngularIntegration));
	callback(userData, &m_minAngle2, sizeof(dFloat));
	callback(userData, &m_maxAngle2, sizeof(dFloat));
	callback(userData, &m_friction2, sizeof(dFloat));
	callback(userData, &m_jointOmega2, sizeof(dFloat));
	callback(userData, &m_spring2, sizeof(dFloat));
	callback(userData, &m_damper2, sizeof(dFloat));
	callback(userData, &m_springDamperRelaxation2, sizeof(dFloat));
}


void dCustomUniversal::SetHardMiddleAxis(bool state)
{
	m_options.m_option4 = state;
}

void dCustomUniversal::EnableLimits2(bool state)
{
	m_options.m_option2 = state;
}

void dCustomUniversal::SetLimits2(dFloat minAngle, dFloat maxAngle)
{
	m_minAngle2 = -dAbs(minAngle);
	m_maxAngle2 = dAbs(maxAngle);
}

void dCustomUniversal::SetAsSpringDamper2(bool state, dFloat springDamperRelaxation, dFloat spring, dFloat damper)
{
	m_spring2 = spring;
	m_damper2 = damper;
	m_options.m_option3 = state;
	m_springDamperRelaxation2 = dClamp(springDamperRelaxation, dFloat(0.0f), dFloat(0.999f));
}

dFloat dCustomUniversal::GetJointAngle2() const
{
	return m_curJointAngle2.GetAngle();
}

dVector dCustomUniversal::GetPinAxis2() const
{
	dMatrix matrix;
	NewtonBodyGetMatrix(m_body0, &matrix[0][0]);
dAssert (0);
	return matrix.RotateVector(m_localMatrix0.m_front);
}

dFloat dCustomUniversal::GetJointOmega2() const
{
	return m_jointOmega2;
}

void dCustomUniversal::SetFriction2(dFloat frictionTorque)
{
	m_friction2 = frictionTorque;
}

dFloat dCustomUniversal::GetFriction2() const
{
	return m_friction2;
}

/*
void dCustomUniversal::Debug(dDebugDisplay* const debugDisplay) const
{
	dCustomHinge::Debug(debugDisplay);

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
*/

void dCustomUniversal::SubmitConstraintSpringDamper(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
{
	NewtonUserJointAddAngularRow(m_joint, -m_curJointAngle2.GetAngle(), &matrix1.m_up[0]);
	NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_springDamperRelaxation2, m_spring2, m_damper2);
}

void dCustomUniversal::SubmitConstraintLimits(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
{
	dFloat angle = m_curJointAngle2.GetAngle() + m_jointOmega2 * timestep;
	if (angle < m_minAngle2) {
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_up[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_friction2);

		const dFloat invtimestep = 1.0f / timestep;
		const dFloat speed = 0.5f * (m_minAngle2 - m_curJointAngle2.GetAngle()) * invtimestep;
		const dFloat stopAccel = NewtonUserJointCalculateRowZeroAccelaration(m_joint) + speed * invtimestep;
		NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
	} else if (angle > m_maxAngle2) {
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_up[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_friction);

		const dFloat invtimestep = 1.0f / timestep;
		const dFloat speed = 0.5f * (m_maxAngle2 - m_curJointAngle2.GetAngle()) * invtimestep;
		const dFloat stopAccel = NewtonUserJointCalculateRowZeroAccelaration(m_joint) + speed * invtimestep;
		NewtonUserJointSetRowAcceleration(m_joint, stopAccel);

	} else if (m_friction != 0.0f) {
		NewtonUserJointAddAngularRow(m_joint, 0, &matrix1.m_up[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowAcceleration(m_joint, -m_jointOmega2 / timestep);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_friction2);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_friction2);
	}
}

void dCustomUniversal::SubmitConstraintLimitSpringDamper(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
{
	dFloat angle = m_curJointAngle2.GetAngle() + m_jointOmega2 * timestep;
	if (angle < m_minAngle2) {
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_up[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_friction2);

		const dFloat invtimestep = 1.0f / timestep;
		const dFloat speed = 0.5f * (m_minAngle2 - m_curJointAngle2.GetAngle()) * invtimestep;
		const dFloat springAccel = NewtonCalculateSpringDamperAcceleration(timestep, m_spring2, m_curJointAngle2.GetAngle(), m_damper2, m_jointOmega2);
		const dFloat stopAccel = NewtonUserJointCalculateRowZeroAccelaration(m_joint) + speed * invtimestep + springAccel;
		NewtonUserJointSetRowAcceleration(m_joint, stopAccel);

	} else if (angle > m_maxAngle2) {
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_up[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_friction2);

		const dFloat invtimestep = 1.0f / timestep;
		const dFloat speed = 0.5f * (m_maxAngle2 - m_curJointAngle2.GetAngle()) * invtimestep;
		const dFloat springAccel = NewtonCalculateSpringDamperAcceleration(timestep, m_spring2, m_curJointAngle2.GetAngle(), m_damper2, m_jointOmega2);
		const dFloat stopAccel = NewtonUserJointCalculateRowZeroAccelaration(m_joint) + speed * invtimestep + springAccel;
		NewtonUserJointSetRowAcceleration(m_joint, stopAccel);

	} else {
		SubmitConstraintSpringDamper(matrix0, matrix1, timestep);
	}
}


void dCustomUniversal::SubmitAngularRow(const dMatrix& matrix0, const dMatrix& matrix1, const dVector& eulers, dFloat timestep)
{
	// the joint angle can be determined by getting the angle between any two non parallel vectors
	m_curJointAngle.Update(eulers.m_y);

	// save the current joint Omega
	dVector omega0(0.0f);
	dVector omega1(0.0f);
	NewtonBodyGetOmega(m_body0, &omega0[0]);
	if (m_body1) {
		NewtonBodyGetOmega(m_body1, &omega1[0]);
	}
	dVector relOmega(omega0 - omega1);
	m_jointOmega2 = relOmega.DotProduct3(matrix1.m_up);

	dMatrix rollMatrix(dYawMatrix(eulers[1]) * matrix1);
	NewtonUserJointAddAngularRow(m_joint, -eulers[2], &rollMatrix.m_right[0]);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	if (m_options.m_option4) {
		dFloat rollOmega = relOmega.DotProduct3(rollMatrix.m_right);
		dFloat alphaRollError = -(eulers[2] + rollOmega * timestep) / (timestep * timestep);
		NewtonUserJointSetRowAcceleration(m_joint, alphaRollError);
	}

	if (m_options.m_option2) {
		if (m_options.m_option3) {
			dCustomUniversal::SubmitConstraintLimitSpringDamper(matrix0, matrix1, timestep);
		} else {
			dCustomUniversal::SubmitConstraintLimits(matrix0, matrix1, timestep);
		}
	} else if (m_options.m_option3) {
		dCustomUniversal::SubmitConstraintSpringDamper(matrix0, matrix1, timestep);
	} else if (m_friction != 0.0f) {
		NewtonUserJointAddAngularRow(m_joint, 0, &matrix1.m_up[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointSetRowAcceleration(m_joint, -m_jointOmega2 / timestep);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_friction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_friction);
	}
}

void dCustomUniversal::Debug(dDebugDisplay* const debugDisplay) const
{
	dCustomHinge::Debug(debugDisplay);

	if (m_options.m_option2) {
		dMatrix matrix0;
		dMatrix matrix1;
		CalculateGlobalMatrix(matrix0, matrix1);

		const int subdiv = 12;
		dVector arch[subdiv + 1];
		const float radius = debugDisplay->m_debugScale;

		if ((m_maxAngle > 1.0e-3f) || (m_minAngle < -1.0e-3f)) {
			// show pitch angle limits
			dVector point(dFloat(radius), dFloat(0.0f), dFloat(0.0f), dFloat(0.0f));

			dFloat minAngle = m_minAngle;
			dFloat maxAngle = m_maxAngle;
			if ((maxAngle - minAngle) >= dPi * 2.0f) {
				minAngle = 0.0f;
				maxAngle = dPi * 2.0f;
			}

			dFloat angleStep = (maxAngle - minAngle) / subdiv;
			dFloat angle0 = minAngle;

			matrix1.m_posit = matrix0.m_posit;
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
}
