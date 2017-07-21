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


void dCustomRagdollMotor_1dof::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dAssert (0);
	dCustomRagdollMotor::SubmitConstraints(timestep, threadIndex);


/*

	// handle special case of the joint being a hinge
	dAssert(0);
	const dFloat yawAngle = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
	const dFloat rollAngle = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
	NewtonUserJointAddAngularRow(m_joint, yawAngle, &matrix1.m_up[0]);
	NewtonUserJointAddAngularRow(m_joint, rollAngle, &matrix1.m_right[0]);

	// the joint angle can be determined by getting the angle between any two non parallel vectors
	dFloat pitchAngle = CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front);
	if (pitchAngle > m_maxTwistAngle) {
		pitchAngle -= m_maxTwistAngle;
		NewtonUserJointAddAngularRow(m_joint, pitchAngle, &matrix0.m_front[0]);
		NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
	}
	else if (pitchAngle < m_minTwistAngle) {
		pitchAngle -= m_minTwistAngle;
		NewtonUserJointAddAngularRow(m_joint, pitchAngle, &matrix0.m_front[0]);
		NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
	}
	else {
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_front[0]);
		dFloat accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
		NewtonUserJointSetRowAcceleration(m_joint, accel);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);
	}
*/



/*
	dMatrix matrix0;
	dMatrix matrix1;

	dAssert((m_maxYaw - m_minYaw) >= 0.0f);
	dAssert((m_maxRoll - m_minRoll) >= 0.0f);
	dAssert((m_maxTwistAngle - m_minTwistAngle) >= 0.0f);

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	dCustomBallAndSocket::SubmitConstraints(timestep, threadIndex);

	if (((m_maxYaw - m_minYaw) < 1.0e-4f) && ((m_maxRoll - m_minRoll) < 1.0e-4f)) {
		// this is a hinge
		Submit1DOFConstraints(matrix0, matrix1, timestep);
	}
	else if ((m_maxTwistAngle - m_minTwistAngle) < 1.0e-4f) {
		// this is a universal joint
		//		Submit2DOFConstraints(matrix0, matrix1, timestep);

		Submit2DOFConstraints(matrix0, matrix1, timestep);
	}
	else {
		// this is a general three dof joint
		//Submit3DOFConstraints(matrix0, matrix1, timestep);

		Submit2DOFConstraints(matrix0, matrix1, timestep);
	}
*/
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
	dAssert(0);
}

void dCustomRagdollMotor_2dof::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dCustomRagdollMotor::SubmitConstraints(timestep, threadIndex);

	dMatrix matrix0;
	dMatrix matrix1;

	dFloat yawAngle;
	dFloat rollAngle;
	dFloat twistAngle;

/*
	const dVector& coneDir0 = matrix0.m_front;
	const dVector& coneDir1 = matrix1.m_front;
	dFloat cosAngle = coneDir0.DotProduct3(coneDir1);
	if (cosAngle <= m_coneAngleCos) {
		dVector lateralDir(coneDir0.CrossProduct(coneDir1));
		dFloat mag2 = lateralDir.DotProduct3(lateralDir);
		dAssert(mag2 > 1.0e-4f);
		lateralDir = lateralDir.Scale(1.0f / dSqrt(mag2));

		dQuaternion rot(m_coneAngleHalfCos, lateralDir.m_x * m_coneAngleHalfSin, lateralDir.m_y * m_coneAngleHalfSin, lateralDir.m_z * m_coneAngleHalfSin);
		dVector frontDir(rot.UnrotateVector(coneDir1));
		dVector upDir(lateralDir.CrossProduct(frontDir));
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &upDir[0]);
		NewtonUserJointAddAngularRow(m_joint, CalculateAngle(coneDir0, frontDir, lateralDir), &lateralDir[0]);
		NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
	}

	//handle twist angle
	dFloat pitchAngle = CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front);
	if ((m_maxTwistAngle - m_minTwistAngle) < 1.0e-4f) {
		NewtonUserJointAddAngularRow(m_joint, pitchAngle, &matrix1.m_front[0]);
	}
	else {
		if (pitchAngle > m_maxTwistAngle) {
			pitchAngle -= m_maxTwistAngle;
			NewtonUserJointAddAngularRow(m_joint, pitchAngle, &matrix0.m_front[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -0.0f);
		}
		else if (pitchAngle < m_minTwistAngle) {
			pitchAngle -= m_minTwistAngle;
			NewtonUserJointAddAngularRow(m_joint, pitchAngle, &matrix0.m_front[0]);
			NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
		}
	}
*/

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
	}
}
