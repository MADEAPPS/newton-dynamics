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
	,m_minYaw(0.0f)
	,m_maxYaw(0.0f)
	,m_minRoll(0.0f)
	,m_maxRoll(0.0f)
	,m_minTwistAngle(0.0f)
	,m_maxTwistAngle(0.0f)
	,m_motorMode(false)
{
}

/*
dCustomRagdollMotor::dCustomRagdollMotor(const dMatrix& childPinAndPivotFrame, NewtonBody* const child, const dMatrix& parentPinAndPivotFrame, NewtonBody* const parent)
	:dCustomBallAndSocket(childPinAndPivotFrame, parentPinAndPivotFrame, child, parent)
	,m_torque(1000.0f)
	,m_minAngle0(0.0f)
	,m_maxAngle0(0.0f)
	,m_minAngle1(0.0f)
	,m_maxAngle1(0.0f)
	,m_minTwistAngle(0.0f)
	,m_maxTwistAngle(0.0f)
	,m_motorMode(false)
{
}
*/

dCustomRagdollMotor::~dCustomRagdollMotor()
{
}

dCustomRagdollMotor::dCustomRagdollMotor(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData)
	:dCustomBallAndSocket(child, parent, callback, userData)
{
	callback(userData, &m_torque, sizeof(dFloat));
	callback(userData, &m_minYaw, sizeof(dFloat));
	callback(userData, &m_maxYaw, sizeof(dFloat));
	callback(userData, &m_minRoll, sizeof(dFloat));
	callback(userData, &m_maxRoll, sizeof(dFloat));
	callback(userData, &m_minTwistAngle, sizeof(dFloat));
	callback(userData, &m_maxTwistAngle, sizeof(dFloat));
}

void dCustomRagdollMotor::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomBallAndSocket::Serialize(callback, userData);

	callback(userData, &m_torque, sizeof(dFloat));
	callback(userData, &m_minYaw, sizeof(dFloat));
	callback(userData, &m_maxYaw, sizeof(dFloat));
	callback(userData, &m_minRoll, sizeof(dFloat));
	callback(userData, &m_maxRoll, sizeof(dFloat));
	callback(userData, &m_minTwistAngle, sizeof(dFloat));
	callback(userData, &m_maxTwistAngle, sizeof(dFloat));
}

void dCustomRagdollMotor::SetJointTorque(dFloat torque)
{
	m_torque = dAbs(torque);
}

void dCustomRagdollMotor::SetYawAngles(dFloat minAngle, dFloat maxAngle)
{
	m_minYaw = dClamp (-dAbs (minAngle), -120.0f * 3.141582f/180.0f, 120.0f * 3.141582f/180.0f);
	m_maxYaw = dClamp ( dAbs (maxAngle), -120.0f * 3.141582f/180.0f, 120.0f * 3.141582f/180.0f);
}

void dCustomRagdollMotor::SetRollAngles(dFloat minAngle, dFloat maxAngle)
{
	m_minRoll = dClamp (-dAbs(minAngle), -65.0f * 3.141582f/180.0f, 65.0f * 3.141582f/180.0f);
	m_maxRoll = dClamp ( dAbs(maxAngle), -65.0f * 3.141582f/180.0f, 65.0f * 3.141582f/180.0f);
}

void dCustomRagdollMotor::SetTwistAngle(dFloat minAngle, dFloat maxAngle)
{
	m_minTwistAngle = dClamp (-dAbs(minAngle), -60.0f * 3.141582f/180.0f, 60.0f * 3.141582f/180.0f);
	m_maxTwistAngle = dClamp ( dAbs(maxAngle), -60.0f * 3.141582f/180.0f, 60.0f * 3.141582f/180.0f);
}

dFloat dCustomRagdollMotor::GetJointTorque() const
{
	return m_torque;
}

void dCustomRagdollMotor::GetYawAngles(dFloat& minAngle, dFloat& maxAngle) const
{
	minAngle = m_minYaw;
	maxAngle = m_maxYaw;
}

void dCustomRagdollMotor::GetRollAngles(dFloat& minAngle, dFloat& maxAngle) const
{
	minAngle = m_minRoll;
	maxAngle = m_maxRoll;
}

void dCustomRagdollMotor::GetTwistAngle(dFloat& minAngle, dFloat& maxAngle) const
{
	minAngle = m_minTwistAngle;
	maxAngle = m_maxTwistAngle;
}



void dCustomRagdollMotor::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	dAssert ((m_maxYaw - m_minYaw) >= 0.0f);
	dAssert ((m_maxRoll - m_minRoll) >= 0.0f);
	dAssert ((m_maxTwistAngle - m_minTwistAngle) >= 0.0f);

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	dCustomBallAndSocket::SubmitConstraints(timestep, threadIndex);

	if (((m_maxYaw - m_minYaw) < 1.0e-4f) && ((m_maxRoll - m_minRoll) < 1.0e-4f)) {
		// this is a hinge
		Submit1DOFConstraints(matrix0, matrix1, timestep);
	} else if ((m_maxTwistAngle - m_minTwistAngle) < 1.0e-4f) {
		// this is a universal joint
//		Submit2DOFConstraints(matrix0, matrix1, timestep);

Submit2DOFConstraints(matrix0, matrix1, timestep);
	} else {
		// this is a general three dof joint
		//Submit3DOFConstraints(matrix0, matrix1, timestep);

Submit2DOFConstraints(matrix0, matrix1, timestep);
	}
}

void dCustomRagdollMotor::Submit3DOFConstraints(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
{
	dAssert (0);
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
}


void dCustomRagdollMotor::Submit1DOFConstraints(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
{
	// handle special case of the joint being a hinge
dAssert (0);
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
	} else if (pitchAngle < m_minTwistAngle) {
		pitchAngle -= m_minTwistAngle;
		NewtonUserJointAddAngularRow(m_joint, pitchAngle, &matrix0.m_front[0]);
		NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
	} else {
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_front[0]);
		dFloat accel = NewtonUserJointGetRowInverseDynamicsAcceleration (m_joint);
		NewtonUserJointSetRowAcceleration (m_joint, accel);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);
	}
}

void dCustomRagdollMotor::Submit2DOFConstraints(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
{
//dMatrix matrix0(matrix0__);
//dMatrix matrix1(matrix1__);
//matrix0 = dGetIdentityMatrix();
//matrix1 = dGetIdentityMatrix();
//matrix0 = matrix1  * dPitchMatrix (10.0f * 3.141592f / 180.0f) * dRollMatrix (40.0f * 3.141592f / 180.0f) * dYawMatrix (30.0f * 3.141592f / 180.0f);

	 dMatrix matrix (matrix0 * matrix1.Inverse());

	 dAssert (matrix.m_front.m_y < 0.999f);

	 dFloat rollAngle = dAsin (matrix.m_front.m_y);
	 dFloat yawAngle = dAtan2 (-matrix.m_front.m_z, matrix.m_front.m_x);
	 dFloat twistAngle = dAtan2 (-matrix.m_right.m_y, matrix.m_up.m_y);

	 dMatrix matrix1_1 (matrix1 * dYawMatrix (-yawAngle));
	 dMatrix matrix1_2 (matrix1_1 * dRollMatrix (-rollAngle));

	 NewtonUserJointAddAngularRow(m_joint, -twistAngle, &matrix1_2.m_front[0]);
//dTrace (("%f (%f %f %f)"))

	 if (rollAngle < m_minRoll) {
		 dAssert (0);
	 } else if (rollAngle > m_maxRoll) {
//		 dAssert (0);
	 } else if (m_motorMode) {
	 }

	 if (yawAngle < m_minYaw) {
		 dAssert (0);
	 } else if (yawAngle > m_maxYaw) {
		 dAssert (0);
	 } else if (m_motorMode) {
	 }



#if 0
	dMatrix matrix1_1;
	matrix1_1.m_up = matrix1.m_up;
	matrix1_1.m_front = matrix1.m_up.CrossProduct(matrix0.m_right);
	dAssert (matrix1_1.m_front.DotProduct3(matrix1_1.m_front) > 0.0f);
	matrix1_1.m_front = matrix1_1.m_front.Scale(1.0f / dSqrt(matrix1_1.m_front.DotProduct3(matrix1_1.m_front)));
	matrix1_1.m_right = matrix1_1.m_front.CrossProduct(matrix1_1.m_up);

	dFloat yawAngle = -CalculateAngle(matrix1_1.m_right, matrix1.m_right, matrix1.m_up);



	dFloat twistAngle = -CalculateAngle(matrix0.m_right, matrix1_1.m_right, matrix1_1.m_front);
//	NewtonUserJointAddAngularRow(m_joint, -twistAngle, &matrix1_1.m_front[0]);


yawAngle = -CalculateAngle(matrix1_1.m_right, matrix1.m_right, matrix1.m_up);
	if (yawAngle < m_minYaw) {
//		dAssert (0);
	} else if (yawAngle > m_maxYaw) {
//		dAssert (0);
	} else {
//		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1_1.m_up[0]);
//		dFloat accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
//		NewtonUserJointSetRowAcceleration(m_joint, accel);
//		NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
//		NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);
	}

//	dFloat angle0 = -CalculateAngle(matrix0.m_front, matrix0.m_up, matrix1_1.m_front);
#endif

}


