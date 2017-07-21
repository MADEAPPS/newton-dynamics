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


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


IMPLEMENT_CUSTOM_JOINT(dCustomRagdollMotor);
IMPLEMENT_CUSTOM_JOINT(dCustomRagdollMotor_1dof)
IMPLEMENT_CUSTOM_JOINT(dCustomRagdollMotor_2dof)
IMPLEMENT_CUSTOM_JOINT(dCustomRagdollMotor_3dof)



void dCustomRagdollMotor::dSaveLoad::Save (const char* const fileName, NewtonBody* const rootbody)
{
return;
/*
	char* const oldloc = setlocale(LC_ALL, 0);
	setlocale(LC_ALL, "C");

	FILE* const file = fopen(fileName, "wt");
	dAssert(file);

	const char* const xxx = GetBodyUniqueName (rootbody);

	dTree<int, dCustomArticulatedTransformController::dSkeletonBone*> filter;

	NewtonBody* const body = controller->GetBoneBody(controller->GetBone(0));
	DemoEntity* entity = (DemoEntity*)NewtonBodyGetUserData(body);
	fprintf(file, "nodesCount: %d\n", controller->GetBoneCount());
	fprintf(file, "rootBone: %s\n\n", entity->GetName().GetStr());

	PrintRagdollBodies(file, controller, controller->GetBone(0), 0, filter);
	filter.RemoveAll();

	fprintf(file, "jointsCount: %d\n", controller->GetBoneCount() - 1);
	PrintRagdollJoints(file, controller, controller->GetBone(0), 0, filter);

	fclose(file);
	setlocale(LC_ALL, oldloc);
*/
}

NewtonBody* dCustomRagdollMotor::dSaveLoad::Load(const char* const name)
{
	dAssert (0);
	return NULL;
}

dCustomRagdollMotor::dCustomRagdollMotor(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomBallAndSocket(pinAndPivotFrame, child, parent)
	,m_torque(1000.0f)
	,m_motorMode(0)
{
m_motorMode = 1;
}


dCustomRagdollMotor::~dCustomRagdollMotor()
{
}

dCustomRagdollMotor::dCustomRagdollMotor(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData)
	:dCustomBallAndSocket(child, parent, callback, userData)
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

void dCustomRagdollMotor::SetJointTorque(dFloat torque)
{
	m_torque = dAbs(torque);
}


dFloat dCustomRagdollMotor::GetJointTorque() const
{
	return m_torque;
}

void dCustomRagdollMotor::CalcutaleEulers (const dMatrix& matrix0, const dMatrix& matrix1, dFloat& twist, dFloat& roll, dFloat& yaw) const
{
	dMatrix matrix(matrix0 * matrix1.Inverse());

	dAssert(matrix.m_front.m_y < 0.999f);

	roll  = dAsin(matrix.m_front.m_y);
	yaw   = dAtan2(-matrix.m_front.m_z, matrix.m_front.m_x);
	twist = dAtan2(-matrix.m_right.m_y, matrix.m_up.m_y);

	dTrace(("%f %f %f\n", twist * 180.0f / 3.1416f, roll * 180.0f / 3.1416f, yaw * 180.0f / 3.1416f));
}


void dCustomRagdollMotor::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dCustomBallAndSocket::SubmitConstraints(timestep, threadIndex);
}



dCustomRagdollMotor_1dof::dCustomRagdollMotor_1dof(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomRagdollMotor(pinAndPivotFrame, child, parent)
	,m_twistAngle()
{
}

dCustomRagdollMotor_1dof::dCustomRagdollMotor_1dof(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData)
	:dCustomRagdollMotor(child, parent, callback, userData)
{
	callback(userData, &m_twistAngle, sizeof(dAngleLimit));
}

void dCustomRagdollMotor_1dof::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomRagdollMotor::Serialize(callback, userData);
	callback(userData, &m_twistAngle, sizeof(dAngleLimit));
}


void dCustomRagdollMotor_1dof::SetTwistAngle(dFloat minAngle, dFloat maxAngle)
{
	m_twistAngle.m_minAngle = dClamp(-dAbs(minAngle), -160.0f * 3.141582f / 180.0f, 160.0f * 3.141582f / 180.0f);
	m_twistAngle.m_maxAngle = dClamp(dAbs(maxAngle), -160.0f * 3.141582f / 180.0f, 160.0f * 3.141582f / 180.0f);
}

void dCustomRagdollMotor_1dof::GetTwistAngle(dFloat& minAngle, dFloat& maxAngle) const
{
	minAngle = m_twistAngle.m_minAngle;
	maxAngle = m_twistAngle.m_maxAngle;
}




dCustomRagdollMotor_2dof::dCustomRagdollMotor_2dof(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomRagdollMotor(pinAndPivotFrame, child, parent)
	,m_yawAngle()
	,m_rollAngle()
{
}

dCustomRagdollMotor_2dof::dCustomRagdollMotor_2dof(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData)
	:dCustomRagdollMotor(child, parent, callback, userData)
{
	callback(userData, &m_yawAngle, sizeof(dAngleLimit));
	callback(userData, &m_rollAngle, sizeof(dAngleLimit));
}

void dCustomRagdollMotor_2dof::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomRagdollMotor::Serialize(callback, userData);
	callback(userData, &m_yawAngle, sizeof(dAngleLimit));
	callback(userData, &m_rollAngle, sizeof(dAngleLimit));
}


void dCustomRagdollMotor_2dof::SetYawAngles(dFloat minAngle, dFloat maxAngle)
{
	m_yawAngle.m_minAngle = dClamp(-dAbs(minAngle), -120.0f * 3.141582f / 180.0f, 120.0f * 3.141582f / 180.0f);
	m_yawAngle.m_maxAngle = dClamp(dAbs(maxAngle), -120.0f * 3.141582f / 180.0f, 120.0f * 3.141582f / 180.0f);
}

void dCustomRagdollMotor_2dof::SetRollAngles(dFloat minAngle, dFloat maxAngle)
{
	m_rollAngle.m_minAngle = dClamp(-dAbs(minAngle), -70.0f * 3.141582f / 180.0f, 70.0f * 3.141582f / 180.0f);
	m_rollAngle.m_maxAngle = dClamp(dAbs(maxAngle), -70.0f * 3.141582f / 180.0f, 70.0f * 3.141582f / 180.0f);
}


void dCustomRagdollMotor_2dof::GetYawAngles(dFloat& minAngle, dFloat& maxAngle) const
{
	minAngle = m_yawAngle.m_minAngle;
	maxAngle = m_yawAngle.m_maxAngle;
}

void dCustomRagdollMotor_2dof::GetRollAngles(dFloat& minAngle, dFloat& maxAngle) const
{
	minAngle = m_rollAngle.m_minAngle;
	maxAngle = m_rollAngle.m_maxAngle;
}


dCustomRagdollMotor_3dof::dCustomRagdollMotor_3dof(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomRagdollMotor(pinAndPivotFrame, child, parent)
	,m_twistAngle()
	,m_yawAngle()
	,m_rollAngle()
{
}

dCustomRagdollMotor_3dof::dCustomRagdollMotor_3dof(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData)
	:dCustomRagdollMotor(child, parent, callback, userData)
{
	callback(userData, &m_twistAngle, sizeof(dAngleLimit));
	callback(userData, &m_yawAngle, sizeof(dAngleLimit));
	callback(userData, &m_rollAngle, sizeof(dAngleLimit));
}

void dCustomRagdollMotor_3dof::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomRagdollMotor::Serialize(callback, userData);
	callback(userData, &m_twistAngle, sizeof(dAngleLimit));
	callback(userData, &m_yawAngle, sizeof(dAngleLimit));
	callback(userData, &m_rollAngle, sizeof(dAngleLimit));
}


void dCustomRagdollMotor_3dof::SetTwistAngle(dFloat minAngle, dFloat maxAngle)
{
	m_twistAngle.m_minAngle = dClamp(-dAbs(minAngle), -30.0f * 3.141582f / 180.0f, 30.0f * 3.141582f / 180.0f);
	m_twistAngle.m_maxAngle = dClamp(dAbs(maxAngle), -30.0f * 3.141582f / 180.0f, 30.0f * 3.141582f / 180.0f);
}

void dCustomRagdollMotor_3dof::SetRollAngles(dFloat minAngle, dFloat maxAngle)
{
	m_rollAngle.m_minAngle = dClamp(-dAbs(minAngle), -70.0f * 3.141582f / 180.0f, 70.0f * 3.141582f / 180.0f);
	m_rollAngle.m_maxAngle = dClamp(dAbs(maxAngle), -70.0f * 3.141582f / 180.0f, 70.0f * 3.141582f / 180.0f);
}

void dCustomRagdollMotor_3dof::SetYawAngles(dFloat minAngle, dFloat maxAngle)
{
	m_yawAngle.m_minAngle = dClamp(-dAbs(minAngle), -120.0f * 3.141582f / 180.0f, 120.0f * 3.141582f / 180.0f);
	m_yawAngle.m_maxAngle = dClamp(dAbs(maxAngle), -120.0f * 3.141582f / 180.0f, 120.0f * 3.141582f / 180.0f);
}



void dCustomRagdollMotor_3dof::GetTwistAngle(dFloat& minAngle, dFloat& maxAngle) const
{
	minAngle = m_twistAngle.m_minAngle;
	maxAngle = m_twistAngle.m_maxAngle;
}


void dCustomRagdollMotor_3dof::GetYawAngles(dFloat& minAngle, dFloat& maxAngle) const
{
	minAngle = m_yawAngle.m_minAngle;
	maxAngle = m_yawAngle.m_maxAngle;
}

void dCustomRagdollMotor_3dof::GetRollAngles(dFloat& minAngle, dFloat& maxAngle) const
{
	minAngle = m_rollAngle.m_minAngle;
	maxAngle = m_rollAngle.m_maxAngle;
}


void dCustomRagdollMotor_3dof::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	dFloat yawAngle;
	dFloat rollAngle;
	dFloat twistAngle;

	dCustomRagdollMotor::SubmitConstraints(timestep, threadIndex);
	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	CalcutaleEulers(matrix0, matrix1, twistAngle, rollAngle, yawAngle);

	NewtonUserJointAddAngularRow(m_joint, -twistAngle * 40.0f, &matrix0.m_front[0]);

	if (rollAngle < m_rollAngle.m_minAngle) {
		rollAngle -= m_rollAngle.m_minAngle;
		NewtonUserJointAddAngularRow(m_joint, rollAngle, &matrix0.m_right[0]);
		NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
	} else if (rollAngle > m_rollAngle.m_maxAngle) {
		rollAngle -= m_rollAngle.m_maxAngle;
		NewtonUserJointAddAngularRow(m_joint, rollAngle, &matrix0.m_right[0]);
		NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
	} else if (m_motorMode) {
		dFloat accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_right[0]);
		NewtonUserJointSetRowAcceleration(m_joint, accel);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);
	}

	if (yawAngle < m_yawAngle.m_minAngle) {
		yawAngle -= m_yawAngle.m_minAngle;
		NewtonUserJointAddAngularRow(m_joint, yawAngle, &matrix1.m_up[0]);
		NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);

	} else if (yawAngle > m_yawAngle.m_maxAngle) {
		yawAngle -= m_yawAngle.m_maxAngle;
		NewtonUserJointAddAngularRow(m_joint, yawAngle, &matrix1.m_up[0]);
		NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
	}
	else if (m_motorMode) {
		dFloat accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_up[0]);
		NewtonUserJointSetRowAcceleration(m_joint, accel);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);
	}
}

void dCustomRagdollMotor_2dof::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	dFloat yawAngle;
	dFloat rollAngle;
	dFloat twistAngle;

	dCustomRagdollMotor::SubmitConstraints(timestep, threadIndex);
	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	CalcutaleEulers(matrix0, matrix1, twistAngle, rollAngle, yawAngle);

	NewtonUserJointAddAngularRow(m_joint, -twistAngle * 40.0f, &matrix0.m_front[0]);

	if (rollAngle < m_rollAngle.m_minAngle) {
		rollAngle -= m_rollAngle.m_minAngle;
		NewtonUserJointAddAngularRow(m_joint, rollAngle, &matrix0.m_right[0]);
		NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
	} else if (rollAngle > m_rollAngle.m_maxAngle) {
		rollAngle -= m_rollAngle.m_maxAngle;
		NewtonUserJointAddAngularRow(m_joint, rollAngle, &matrix0.m_right[0]);
		NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
	} else if (m_motorMode) {
		dFloat accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_right[0]);
		NewtonUserJointSetRowAcceleration(m_joint, accel);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);
	}

	if (yawAngle < m_yawAngle.m_minAngle) {
		yawAngle -= m_yawAngle.m_minAngle;
		NewtonUserJointAddAngularRow(m_joint, yawAngle, &matrix1.m_up[0]);
		NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);

	} else if (yawAngle > m_yawAngle.m_maxAngle) {
		yawAngle -= m_yawAngle.m_maxAngle;
		NewtonUserJointAddAngularRow(m_joint, yawAngle, &matrix1.m_up[0]);
		NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
	} else if (m_motorMode) {
		dFloat accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_up[0]);
		NewtonUserJointSetRowAcceleration(m_joint, accel);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);
	}
}



void dCustomRagdollMotor_1dof::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	dFloat yawAngle;
	dFloat rollAngle;
	dFloat twistAngle;

	dCustomRagdollMotor::SubmitConstraints(timestep, threadIndex);

	CalculateGlobalMatrix(matrix0, matrix1);
	CalcutaleEulers(matrix0, matrix1, twistAngle, rollAngle, yawAngle);

	NewtonUserJointAddAngularRow(m_joint, -yawAngle, &matrix1.m_up[0]);
	NewtonUserJointAddAngularRow(m_joint, -rollAngle, &matrix1.m_right[0]);


	if (twistAngle < m_twistAngle.m_minAngle) {
		twistAngle -= m_twistAngle.m_minAngle;
		NewtonUserJointAddAngularRow(m_joint, yawAngle, &matrix1.m_front[0]);
		NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
	} else if (twistAngle > m_twistAngle.m_maxAngle) {
		twistAngle -= m_twistAngle.m_maxAngle;
		NewtonUserJointAddAngularRow(m_joint, twistAngle, &matrix1.m_front[0]);
		NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
//	} else if (m_motorMode) {
	} else if (0) {
		dFloat accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_front[0]);
		NewtonUserJointSetRowAcceleration(m_joint, accel);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);
	}
}
