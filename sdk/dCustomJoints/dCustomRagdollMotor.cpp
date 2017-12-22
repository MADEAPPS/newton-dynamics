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


// dCustomBallAndSocket.cpp: implementation of the dCustomBallAndSocket class.
//
//////////////////////////////////////////////////////////////////////
#include "dCustomJointLibraryStdAfx.h"
#include "dCustomRagdollMotor.h"
#include "dCustomModelLoadSave.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

IMPLEMENT_CUSTOM_JOINT(dCustomRagdollMotor);
IMPLEMENT_CUSTOM_JOINT(dCustomRagdollMotor_1dof)
IMPLEMENT_CUSTOM_JOINT(dCustomRagdollMotor_2dof)
IMPLEMENT_CUSTOM_JOINT(dCustomRagdollMotor_3dof)
IMPLEMENT_CUSTOM_JOINT(dCustomRagdollMotor_EndEffector)


dCustomRagdollMotor::dCustomRagdollMotor(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomBallAndSocket(pinAndPivotFrame, child, parent)
	,m_torque(1.0f)
	,m_motorMode(0)
{
}

dCustomRagdollMotor::~dCustomRagdollMotor()
{
}

void dCustomRagdollMotor::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	callback(userData, &m_motorMode, sizeof(int));
	callback(userData, &m_torque, sizeof(dFloat));
}

void dCustomRagdollMotor::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomBallAndSocket::Serialize(callback, userData);

	callback(userData, &m_motorMode, sizeof(int));
	callback(userData, &m_torque, sizeof(dFloat));
}

void dCustomRagdollMotor::Load(dCustomJointSaveLoad* const fileLoader)
{
	const char* token = fileLoader->NextToken();
	dAssert(!strcmp(token, "frictionTorque:"));
	m_torque = fileLoader->LoadFloat();

	token = fileLoader->NextToken();
	dAssert(!strcmp(token, "motorMode:"));
	m_motorMode = fileLoader->LoadInt();
}

void dCustomRagdollMotor::Save(dCustomJointSaveLoad* const fileSaver) const
{
	dCustomBallAndSocket::Save(fileSaver);
	fileSaver->SaveFloat("\tfrictionTorque", m_torque);
	fileSaver->SaveInt("\tmotorMode", m_motorMode);
}

bool dCustomRagdollMotor::GetMode() const
{
	return m_motorMode ? 1 : 0;
}

void dCustomRagdollMotor::SetMode(bool ragDollOrMotor)
{
	m_motorMode = ragDollOrMotor ? 1 : 0;
}


void dCustomRagdollMotor::SetJointTorque(dFloat torque)
{
	m_torque = dAbs(torque);
}


dFloat dCustomRagdollMotor::GetJointTorque() const
{
	return m_torque;
}

void dCustomRagdollMotor::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dCustomBallAndSocket::SubmitConstraints(timestep, threadIndex);
}

dCustomRagdollMotor_1dof::dCustomRagdollMotor_1dof(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomRagdollMotor(pinAndPivotFrame, child, parent)
	,m_minTwistAngle(-30.0f * 3.141592f / 180.0f)
	,m_maxTwistAngle( 30.0f * 3.141592f / 180.0f)
{
}

void dCustomRagdollMotor_1dof::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	m_minTwistAngle = 0.0f;
	m_maxTwistAngle = 0.0f;

	callback(userData, &m_minTwistAngle, sizeof(m_minTwistAngle));
	callback(userData, &m_maxTwistAngle, sizeof(m_maxTwistAngle));
}

void dCustomRagdollMotor_1dof::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomRagdollMotor::Serialize(callback, userData);
	callback(userData, &m_minTwistAngle, sizeof(m_minTwistAngle));
	callback(userData, &m_maxTwistAngle, sizeof(m_maxTwistAngle));
}

void dCustomRagdollMotor_1dof::Load(dCustomJointSaveLoad* const fileLoader)
{
	const char* token = fileLoader->NextToken();
	dAssert(!strcmp(token, "minTwistAngle:"));
	m_minTwistAngle = fileLoader->LoadFloat() * 3.141592f / 180.0f;

	token = fileLoader->NextToken();
	dAssert(!strcmp(token, "maxTwistAngle:"));
	m_maxTwistAngle = fileLoader->LoadFloat() * 3.141592f / 180.0f;
}

void dCustomRagdollMotor_1dof::Save(dCustomJointSaveLoad* const fileSaver) const
{
	dCustomRagdollMotor::Save(fileSaver);
	fileSaver->SaveFloat("\tminTwistAngle", m_minTwistAngle * 180.0f / 3.141592f);
	fileSaver->SaveFloat("\tmaxTwistAngle", m_maxTwistAngle * 180.0f / 3.141592f);
}

void dCustomRagdollMotor_1dof::SetTwistAngle(dFloat minAngle, dFloat maxAngle)
{
	m_minTwistAngle = dMax(-dAbs(minAngle), dFloat(-170.0f * 3.141582f / 180.0f));
	m_maxTwistAngle = dMin( dAbs(maxAngle), dFloat( 170.0f * 3.141582f / 180.0f));
}

void dCustomRagdollMotor_1dof::GetTwistAngle(dFloat& minAngle, dFloat& maxAngle) const
{
	minAngle = m_minTwistAngle;
	maxAngle = m_maxTwistAngle;
}


void dCustomRagdollMotor_1dof::Debug(dDebugDisplay* const debugDisplay) const
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	dCustomRagdollMotor::Debug(debugDisplay);

	// vis limits
	const int subdiv = 16;
	const float radius = 0.25f;

	dVector point (0.0f, radius, 0.0f, 0.0f);
	dFloat angleStep = (m_maxTwistAngle - m_minTwistAngle) / subdiv;
	dFloat angle0 = m_minTwistAngle;

	dVector arch[subdiv + 1];
	debugDisplay->SetColor(dVector (1.0f, 1.0f, 0.0f, 0.0f));
	for (int i = 0; i <= subdiv; i++) {
		dVector p (matrix1.TransformVector(dPitchMatrix(angle0).RotateVector(point)));
		arch[i] = p;
		debugDisplay->DrawLine(matrix1.m_posit, p);
		angle0 += angleStep;
	}

	for (int i = 0; i < subdiv; i++) {
		debugDisplay->DrawLine(arch[i], arch[i + 1]);
	}
}


void dCustomRagdollMotor_1dof::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;
	dVector omega0(0.0f);
	dVector omega1(0.0f);

	CalculateGlobalMatrix(matrix0, matrix1);
	dCustomRagdollMotor::SubmitConstraints(timestep, threadIndex);

	// two rows to restrict rotation around around the parent coordinate system
	NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up), &matrix1.m_up[0]);
	NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right), &matrix1.m_right[0]);

	// the joint angle can be determined by getting the angle between any two non parallel vectors
	NewtonBodyGetOmega(m_body0, &omega0[0]);
	NewtonBodyGetOmega(m_body1, &omega1[0]);

	dVector relOmega(omega1 - omega0);

	dFloat accel = 0.0f;
	dFloat correctionFactor = 0.3f;
	dFloat invTimestep = 1.0f / timestep;

	dFloat angle = CalculateAngle(matrix1.m_up, matrix0.m_up, matrix1.m_front);
	if (angle < m_minTwistAngle) {
		angle = angle - m_minTwistAngle;

		// tell joint error will minimize the exceeded angle error
		NewtonUserJointAddAngularRow(m_joint, -angle, &matrix1.m_front[0]);

		accel = (-correctionFactor * angle * invTimestep + relOmega.DotProduct3(matrix1.m_front)) * invTimestep;
		NewtonUserJointSetRowAcceleration(m_joint, accel);

		// allow the joint to move back freely 
		NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);

	} else if (angle > m_maxTwistAngle) {
		angle = angle - m_maxTwistAngle;

		// tell joint error will minimize the exceeded angle error
		NewtonUserJointAddAngularRow(m_joint, -angle, &matrix1.m_front[0]);

		accel = (-correctionFactor * angle * invTimestep + relOmega.DotProduct3(matrix1.m_front)) * invTimestep;
		NewtonUserJointSetRowAcceleration(m_joint, accel);

		// allow the joint to move back freely
		NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);

	} else if (m_motorMode) {
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_front[0]);
		accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
		NewtonUserJointSetRowAcceleration(m_joint, accel);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);
	}
}


dCustomRagdollMotor_2dof::dCustomRagdollMotor_2dof(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomRagdollMotor(pinAndPivotFrame, child, parent)
	,m_coneAngle(30.0f * 3.141592f / 180.0f)
{
}

void dCustomRagdollMotor_2dof::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	m_coneAngle = 0.0f;
	callback(userData, &m_coneAngle, sizeof(m_coneAngle));
}

void dCustomRagdollMotor_2dof::Load(dCustomJointSaveLoad* const fileLoader)
{
	const char* token = fileLoader->NextToken();
	token;
	dAssert(!strcmp(token, "coneAngle:"));
	m_coneAngle = fileLoader->LoadFloat() * 3.141592f / 180.0f;
}

void dCustomRagdollMotor_2dof::Save(dCustomJointSaveLoad* const fileSaver) const
{
	dCustomRagdollMotor::Save(fileSaver);
	fileSaver->SaveFloat ("\tconeAngle", m_coneAngle * 180.0f / 3.141592f);
}

void dCustomRagdollMotor_2dof::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomRagdollMotor::Serialize(callback, userData);
	callback(userData, &m_coneAngle, sizeof(m_coneAngle));
}

void dCustomRagdollMotor_2dof::SetConeAngle(dFloat angle)
{
	m_coneAngle = dMin(dAbs(angle), dFloat(150.0f * 3.141582f / 180.0f));
}

dFloat dCustomRagdollMotor_2dof::GetConeAngle() const
{
	return m_coneAngle;
}


void dCustomRagdollMotor_2dof::Debug(dDebugDisplay* const debugDisplay) const
{
	dMatrix matrix0;
	dMatrix matrix1;

	CalculateGlobalMatrix(matrix0, matrix1);
	dCustomRagdollMotor::Debug(debugDisplay);

	const int subdiv = 24;
	const float radius = 0.25f;

	dVector point(radius * dCos(m_coneAngle), radius * dSin(m_coneAngle), 0.0f, 0.0f);
	dFloat angleStep = 3.141692f * 2.0f / subdiv;
	dFloat angle0 = 0.0f;

	dVector arch[subdiv + 1];
	debugDisplay->SetColor(dVector(1.0f, 1.0f, 0.0f, 0.0f));
	for (int i = 0; i <= subdiv; i++) {
		dVector p(matrix1.TransformVector(dPitchMatrix(angle0).RotateVector(point)));
		arch[i] = p;
		debugDisplay->DrawLine(matrix1.m_posit, p);
		angle0 += angleStep;
	}

	for (int i = 0; i < subdiv; i++) {
		debugDisplay->DrawLine(arch[i], arch[i + 1]);
	}
}


void dCustomRagdollMotor_2dof::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;
	dVector omega0(0.0f);
	dVector omega1(0.0f);

	CalculateGlobalMatrix(matrix0, matrix1);
	dCustomRagdollMotor::SubmitConstraints(timestep, threadIndex);

	const dVector& coneDir0 = matrix0.m_front;
	const dVector& coneDir1 = matrix1.m_front;
	dFloat dot = coneDir0.DotProduct3(coneDir1);

	// do the twist
	dQuaternion quat0(matrix0);
	dQuaternion quat1(matrix1);

	if (quat0.DotProduct(quat1) < 0.0f) {
		quat0.Scale(-1.0f);
	}

	// factor rotation about x axis between quat0 and quat1. 
	// Code is an optimization of this: qt = q0.Inversed() * q1; 
	// halfTwistAngle = atan (qt.x / qt.w);
	dFloat* const q0 = &quat0.m_q0;
	dFloat* const q1 = &quat1.m_q0;
	dFloat num = (q0[0] * q1[1]) + (-q0[1] * q1[0]) + (-q0[2] * q1[3]) - (-q0[3] * q1[2]);
	dFloat den = (q0[0] * q1[0]) - (-q0[1] * q1[1]) - (-q0[2] * q1[2]) - (-q0[3] * q1[3]);
	dFloat twistAngle = 2.0f * dAtan2(num, den);

	// select an axis for the twist. 
	// any on the unit arc from coneDir0 to coneDir1 would do - average seemed best after some tests
	if (dot > -0.999f) {
		dVector twistAxis = coneDir0 + coneDir1;
		twistAxis = twistAxis.Scale(1.0f / dSqrt(twistAxis.DotProduct3(twistAxis)));
		NewtonUserJointAddAngularRow(m_joint, twistAngle, &twistAxis[0]);
	} else {
		NewtonUserJointAddAngularRow(m_joint, twistAngle, &coneDir1[0]);
	}

	NewtonBodyGetOmega(m_body0, &omega0[0]);
	NewtonBodyGetOmega(m_body1, &omega1[0]);

	dVector relOmega(omega1 - omega0);

	dFloat accel = 0.0f;
	dFloat correctionFactor = 0.3f;
	dFloat invTimestep = 1.0f / timestep;
	dFloat coneAngle = dAcos(dClamp(dot, dFloat(-1.0f), dFloat(1.0f)));
	dFloat angle = coneAngle - m_coneAngle;
	if (angle > 0.0f) {
		dVector swingAxis(coneDir0.CrossProduct(coneDir1));
		dAssert(swingAxis.DotProduct3(swingAxis) > 0.0f);
		swingAxis = swingAxis.Scale(1.0f / dSqrt(swingAxis.DotProduct3(swingAxis)));
		NewtonUserJointAddAngularRow(m_joint, angle, &swingAxis[0]);
		accel = (correctionFactor * angle * invTimestep + relOmega.DotProduct3(swingAxis)) * invTimestep;
		NewtonUserJointSetRowAcceleration(m_joint, accel);
		NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
		if (m_motorMode) {
			dVector sideDir(swingAxis.CrossProduct(coneDir0));
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);
			accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, accel);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);
		}
	} else if (m_motorMode) {
		if (coneAngle > 1.0f * 3.141592f / 180.0f) {
			dVector swingAxis = (coneDir0.CrossProduct(coneDir1));
			dAssert(swingAxis.DotProduct3(swingAxis) > 0.0f);
			swingAxis = swingAxis.Scale(1.0f / dSqrt(swingAxis.DotProduct3(swingAxis)));

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &swingAxis[0]);
			accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, accel);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);

			dVector sideDir(swingAxis.CrossProduct(coneDir0));
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);
			accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, accel);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);
		} else {
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_right[0]);
			accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, accel);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_up[0]);
			accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, accel);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);
		}
	}
}


dCustomRagdollMotor_3dof::dCustomRagdollMotor_3dof(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomRagdollMotor(pinAndPivotFrame, child, parent)
	,m_coneAngle(30.0f * 3.141592f / 180.0f)
	,m_minTwistAngle(-30.0f * 3.141592f / 180.0f)
	,m_maxTwistAngle(30.0f * 3.141592f / 180.0f)
{
}

void dCustomRagdollMotor_3dof::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	callback(userData, &m_coneAngle, sizeof(m_coneAngle));
	callback(userData, &m_minTwistAngle, sizeof(m_minTwistAngle));
	callback(userData, &m_maxTwistAngle, sizeof(m_maxTwistAngle));
}


void dCustomRagdollMotor_3dof::Load(dCustomJointSaveLoad* const fileLoader)
{
	const char* token = fileLoader->NextToken();
	dAssert(!strcmp(token, "coneAngle:"));
	m_coneAngle = fileLoader->LoadFloat() * 3.141592f / 180.0f;

	token = fileLoader->NextToken();
	dAssert(!strcmp(token, "minTwistAngle:"));
	m_minTwistAngle = fileLoader->LoadFloat() * 3.141592f / 180.0f;

	token = fileLoader->NextToken();
	dAssert(!strcmp(token, "maxTwistAngle:"));
	m_maxTwistAngle = fileLoader->LoadFloat() * 3.141592f / 180.0f;
}

void dCustomRagdollMotor_3dof::Save(dCustomJointSaveLoad* const fileSaver) const
{
	dCustomRagdollMotor::Save(fileSaver);
	fileSaver->SaveFloat ("\tconeAngle", m_coneAngle * 180.0f / 3.141592f);
	fileSaver->SaveFloat ("\tminTwistAngle", m_minTwistAngle * 180.0f / 3.141592f);
	fileSaver->SaveFloat ("\tmaxTwistAngle", m_maxTwistAngle * 180.0f / 3.141592f);
}

void dCustomRagdollMotor_3dof::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomRagdollMotor::Serialize(callback, userData);
	callback(userData, &m_coneAngle, sizeof(m_coneAngle));
	callback(userData, &m_minTwistAngle, sizeof(m_minTwistAngle));
	callback(userData, &m_maxTwistAngle, sizeof(m_maxTwistAngle));
}

void dCustomRagdollMotor_3dof::SetTwistAngle(dFloat minAngle, dFloat maxAngle)
{
	m_minTwistAngle = dMax(-dAbs(minAngle), dFloat(-60.0f * 3.141582f / 180.0f));
	m_maxTwistAngle = dMin( dAbs(maxAngle), dFloat( 60.0f * 3.141582f / 180.0f));
}

void dCustomRagdollMotor_3dof::GetTwistAngle(dFloat& minAngle, dFloat& maxAngle) const
{
	minAngle = m_minTwistAngle;
	maxAngle = m_maxTwistAngle;
}

void dCustomRagdollMotor_3dof::SetConeAngle(dFloat angle)
{
	m_coneAngle = dMin(dAbs(angle), dFloat(150.0f * 3.141582f / 180.0f));
}

dFloat dCustomRagdollMotor_3dof::GetConeAngle() const
{
	return m_coneAngle;
}


void dCustomRagdollMotor_3dof::Debug(dDebugDisplay* const debugDisplay) const
{
	dMatrix matrix0;
	dMatrix matrix1;

	CalculateGlobalMatrix(matrix0, matrix1);
	dCustomRagdollMotor::Debug(debugDisplay);

	const int subdiv = 24;
	const float radius = 0.25f;

	dVector point(radius * dCos(m_coneAngle), radius * dSin(m_coneAngle), 0.0f, 0.0f);
	dFloat angleStep = 3.141692f * 2.0f / subdiv;
	dFloat angle0 = 0.0f;

	dVector arch[subdiv + 1];
	debugDisplay->SetColor(dVector(1.0f, 1.0f, 0.0f, 0.0f));
	for (int i = 0; i <= subdiv; i++) {
		dVector p(matrix1.TransformVector(dPitchMatrix(angle0).RotateVector(point)));
		arch[i] = p;
		debugDisplay->DrawLine(matrix1.m_posit, p);
		angle0 += angleStep;
	}

	for (int i = 0; i < subdiv; i++) {
		debugDisplay->DrawLine(arch[i], arch[i + 1]);
	}

	// select an axis for the twist. 
	// any on the unit arc from coneDir0 to coneDir1 would do - average seemed best after some tests
	dVector coneDir0 (matrix0.m_front);
	dVector coneDir1 (matrix1.m_front);
	dFloat dot = coneDir0.DotProduct3(coneDir1);
	if (dot < 0.999f) {
		dVector pin (coneDir1.CrossProduct(coneDir0));
		dVector axis = pin.Scale (1.0f / dSqrt (pin.DotProduct3(pin)));
		dFloat angle = dAcos(dClamp(dot, dFloat(-1.0f), dFloat(1.0f)));
		dQuaternion rot (axis, angle);
		dVector posit (matrix1.m_posit);
		matrix1 = matrix1 * dMatrix (rot, dVector(0.0f, 0.0f, 0.0f, 1.0f));
		matrix1.m_posit = posit;
	}
	
	debugDisplay->SetColor(dVector(1.0f, 0.0f, 1.0f, 0.0f));
	debugDisplay->DrawLine(matrix0.m_posit, matrix0.m_posit - matrix0.m_up.Scale (radius * 1.25f));

	matrix1 = dRollMatrix(-90.0f * 3.141592f / 180.0f) * matrix1;
	const int subdiv1 = 12;
	point = dVector (radius, 0.0f, 0.0f, 0.0f);
	angleStep = (m_maxTwistAngle - m_minTwistAngle) / subdiv1;
	angle0 = m_minTwistAngle;

	dVector arch1[subdiv + 1];
	debugDisplay->SetColor(dVector(0.0f, 1.0f, 1.0f, 0.0f));
	for (int i = 0; i <= subdiv1; i++) {
		dVector p(matrix1.TransformVector(dYawMatrix(angle0).RotateVector(point)));
		arch[i] = p;
		debugDisplay->DrawLine(matrix1.m_posit, p);
		angle0 += angleStep;
	}

	for (int i = 0; i < subdiv1; i++) {
		debugDisplay->DrawLine(arch[i], arch[i + 1]);
	}
}


void dCustomRagdollMotor_3dof::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;
	dVector omega0(0.0f);
	dVector omega1(0.0f);

	CalculateGlobalMatrix(matrix0, matrix1);
	dCustomRagdollMotor::SubmitConstraints(timestep, threadIndex);

	const dVector& coneDir0 = matrix0.m_front;
	const dVector& coneDir1 = matrix1.m_front;
	dFloat dot = coneDir0.DotProduct3(coneDir1);

	NewtonBodyGetOmega(m_body0, &omega0[0]);
	NewtonBodyGetOmega(m_body1, &omega1[0]);

	dVector relOmega (omega1 - omega0);

	dFloat accel = 0.0f;
	dFloat correctionFactor = 0.3f;
	dFloat invTimestep = 1.0f / timestep;
	dFloat coneAngle = dAcos(dClamp(dot, dFloat(-1.0f), dFloat(1.0f)));
	dFloat angle = coneAngle - m_coneAngle;
	if (angle > 0.0f) {
		dVector swingAxis(coneDir0.CrossProduct(coneDir1));
		dAssert(swingAxis.DotProduct3(swingAxis) > 0.0f);
		swingAxis = swingAxis.Scale(1.0f / dSqrt(swingAxis.DotProduct3(swingAxis)));
		NewtonUserJointAddAngularRow(m_joint, angle, &swingAxis[0]);
		accel = (correctionFactor * angle * invTimestep + relOmega.DotProduct3(swingAxis)) * invTimestep;
		NewtonUserJointSetRowAcceleration(m_joint, accel);
		NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
		if (m_motorMode) {
			dVector sideDir(swingAxis.CrossProduct(coneDir0));
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);
			accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, accel);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);
		}
	} else if (m_motorMode) {
		if (coneAngle > 1.0f * 3.141592f / 180.0f) {
			dVector swingAxis = (coneDir0.CrossProduct(coneDir1));
			dAssert(swingAxis.DotProduct3(swingAxis) > 0.0f);
			swingAxis = swingAxis.Scale(1.0f / dSqrt(swingAxis.DotProduct3(swingAxis)));

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &swingAxis[0]);
			accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, accel);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);

			dVector sideDir(swingAxis.CrossProduct(coneDir0));
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);
			accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, accel);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);
		} else {
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_right[0]);
			accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, accel);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_up[0]);
			accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, accel);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);
		}
	}

	if (dot < 0.999f) {
		dVector pin(coneDir1.CrossProduct(coneDir0));
		dVector axis = pin.Scale(1.0f / dSqrt(pin.DotProduct3(pin)));
		angle = dAcos(dClamp(dot, dFloat(-1.0f), dFloat(1.0f)));
		dQuaternion rot(axis, angle);
		matrix1 = matrix1 * dMatrix(rot, dVector(0.0f, 0.0f, 0.0f, 1.0f));
	}

	dFloat twistAngle = CalculateAngle (matrix0.m_up, matrix1.m_up, matrix1.m_front);
	if (twistAngle < m_minTwistAngle) {
		twistAngle = twistAngle - m_minTwistAngle;
		NewtonUserJointAddAngularRow(m_joint, twistAngle, &matrix1.m_front[0]);
		accel = (correctionFactor * twistAngle * invTimestep + relOmega.DotProduct3(matrix1.m_front)) * invTimestep;
		NewtonUserJointSetRowAcceleration(m_joint, accel);
		NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
	} else if (twistAngle > m_maxTwistAngle) {
		twistAngle = twistAngle - m_maxTwistAngle;
		NewtonUserJointAddAngularRow(m_joint, twistAngle, &matrix1.m_front[0]);
		accel = (correctionFactor * twistAngle * invTimestep + relOmega.DotProduct3(matrix1.m_front)) * invTimestep;
		NewtonUserJointSetRowAcceleration(m_joint, accel);
		NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
	} else if (m_motorMode) {
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_front[0]);
		accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
		NewtonUserJointSetRowAcceleration(m_joint, accel);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);
	}
}


dCustomRagdollMotor_EndEffector::dCustomRagdollMotor_EndEffector(NewtonInverseDynamics* const invDynSolver, void* const invDynNode, const dMatrix& attachmentPointInGlobalSpace)
	:dCustomJoint (invDynSolver, invDynNode)
	,m_targetMatrix(attachmentPointInGlobalSpace)
	,m_linearSpeed(0.4f)
	,m_angularSpeed(1.0f)
	,m_maxLinearFriction(10000.0f)
	,m_maxAngularFriction(1000.0f)
	,m_isSixdof(true)
{
	SetAsSixdof();
	CalculateLocalMatrix(attachmentPointInGlobalSpace, m_localMatrix0, m_localMatrix1);
	SetTargetMatrix(attachmentPointInGlobalSpace);
	SetSolverModel(2);
}

dCustomRagdollMotor_EndEffector::~dCustomRagdollMotor_EndEffector()
{
}


void dCustomRagdollMotor_EndEffector::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	dAssert(0);
}

void dCustomRagdollMotor_EndEffector::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dAssert(0);
}


void dCustomRagdollMotor_EndEffector::SetAsSixdof()
{
	m_isSixdof = true;
}

void dCustomRagdollMotor_EndEffector::SetAsThreedof()
{
	m_isSixdof = false;
}


void dCustomRagdollMotor_EndEffector::SetMaxLinearFriction(dFloat accel)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	NewtonBodyGetMass(m_body0, &mass, &Ixx, &Iyy, &Izz);
	m_maxLinearFriction = dAbs(accel) * mass;
}

void dCustomRagdollMotor_EndEffector::SetMaxAngularFriction(dFloat alpha)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	NewtonBodyGetMass(m_body0, &mass, &Ixx, &Iyy, &Izz);
	m_maxAngularFriction = dAbs(alpha) * mass;
}


void dCustomRagdollMotor_EndEffector::SetLinearSpeed(dFloat speed)
{
	m_linearSpeed = dAbs (speed);
}

void dCustomRagdollMotor_EndEffector::SetAngularSpeed(dFloat speed)
{
	m_angularSpeed = dAbs (speed);
}


void dCustomRagdollMotor_EndEffector::SetTargetRotation(const dQuaternion& rotation)
{
	NewtonBodySetSleepState(m_body0, 0);
}

void dCustomRagdollMotor_EndEffector::SetTargetPosit(const dVector& posit)
{
	NewtonBodySetSleepState(m_body0, 0);
	m_targetMatrix.m_posit = posit;
	m_targetMatrix.m_posit.m_w = 1.0f;
}

void dCustomRagdollMotor_EndEffector::SetTargetMatrix(const dMatrix& matrix)
{
	NewtonBodySetSleepState(m_body0, 0);
	m_targetMatrix = matrix;
	m_targetMatrix.m_posit.m_w = 1.0f;
}

dMatrix dCustomRagdollMotor_EndEffector::GetBodyMatrix() const
{
	dMatrix matrix0;
	NewtonBodyGetMatrix(m_body0, &matrix0[0][0]);
	return m_localMatrix0 * matrix0;
}


dMatrix dCustomRagdollMotor_EndEffector::GetTargetMatrix() const
{
	return m_targetMatrix;
}

void dCustomRagdollMotor_EndEffector::Debug(dDebugDisplay* const debugDisplay) const
{
	debugDisplay->DrawFrame(GetBodyMatrix());
	debugDisplay->DrawFrame(m_targetMatrix);
}

void dCustomRagdollMotor_EndEffector::Load(dCustomJointSaveLoad* const fileLoader)
{
	const char* token = fileLoader->NextToken();
	token;
	dAssert(!strcmp(token, "stiffness:"));
	m_stiffness = fileLoader->LoadFloat();
}


void dCustomRagdollMotor_EndEffector::Save(dCustomJointSaveLoad* const fileSaver) const
{
	dCustomJoint::Save(fileSaver);
	dAssert(0);
}


void dCustomRagdollMotor_EndEffector::SubmitConstraints(dFloat timestep, int threadIndex)
{
	// check if this is an impulsive time step
	dMatrix matrix0(GetBodyMatrix());
	dVector veloc(0.0f);
	dVector omega(0.0f);
	dVector com(0.0f);
	dVector pointVeloc(0.0f);
	dAssert(timestep > 0.0f);
	const dFloat damp = 0.3f;
	const dFloat invTimestep = 1.0f / timestep;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	dVector relPosit(m_targetMatrix.m_posit - matrix0.m_posit);
	NewtonBodyGetPointVelocity(m_body0, &m_targetMatrix.m_posit[0], &pointVeloc[0]);

	for (int i = 0; i < 3; i++) {
		// Restrict the movement on the pivot point along all tree orthonormal direction
		dFloat speed = pointVeloc.DotProduct3(m_targetMatrix[i]);
		dFloat dist = relPosit.DotProduct3(m_targetMatrix[i]) * damp;
		dFloat relSpeed = dClamp (dist * invTimestep - speed, -m_linearSpeed, m_linearSpeed);
		dFloat relAccel = relSpeed * invTimestep;
		NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix0.m_posit[0], &m_targetMatrix[i][0]);
		NewtonUserJointSetRowAcceleration(m_joint, relAccel);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxLinearFriction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_maxLinearFriction);
	}

	if (m_isSixdof) {

/*
		dQuaternion rotation(matrix0.Inverse() * m_targetMatrix);
		if (dAbs(rotation.m_q0) < 0.99998f) {
			dMatrix rot(dGrammSchmidt(dVector(rotation.m_q1, rotation.m_q2, rotation.m_q3)));
			dFloat angle = 2.0f * dAcos(dClamp(rotation.m_q0, dFloat(-1.0f), dFloat(1.0f)));

			NewtonUserJointAddAngularRow(m_joint, angle, &rot.m_front[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &rot.m_up[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &rot.m_right[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);

		} else {
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_front[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_up[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_right[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);
		}
*/
		
		dQuaternion rotation(matrix0.Inverse() * m_targetMatrix);
		if (dAbs(rotation.m_q0) < 0.99998f) {
			dMatrix rot(dGrammSchmidt(dVector(rotation.m_q1, rotation.m_q2, rotation.m_q3)));
			dFloat angle = 2.0f * dAcos(dClamp(rotation.m_q0, dFloat(-1.0f), dFloat(1.0f)));
			NewtonUserJointAddAngularRow(m_joint, angle, &rot.m_front[0]);
			dFloat alpha = NewtonUserJointGetRowAcceleration (m_joint);
			if (dAbs (alpha) > m_angularSpeed * invTimestep) {
				alpha = dClamp (alpha, -m_angularSpeed * invTimestep, m_angularSpeed * invTimestep);
				NewtonUserJointSetRowAcceleration(m_joint, alpha);
			}
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &rot.m_up[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &rot.m_right[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);
		} else {
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_front[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_up[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_right[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);
		}
	}
}
