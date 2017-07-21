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


void dCustomRagdollMotor::dSaveLoad::GetBodyList (dList<BodyJointPair>& list, NewtonBody* const rootBody)
{
	NewtonBody* stackMem[256];
	int stack = 1;
	stackMem[0] = rootBody;

	dTree<int, NewtonBody*> filter;

	filter.Insert(0, rootBody);
	list.Append (BodyJointPair (rootBody, NULL));
	while (stack) {
		stack--;
		NewtonBody* const root = stackMem[stack];

		for (NewtonJoint* joint = NewtonBodyGetFirstJoint(root); joint; joint = NewtonBodyGetNextJoint(root, joint)) {
			dCustomJoint* const cJoint = (dCustomJoint*)NewtonJointGetUserData(joint);
			NewtonBody* const childBody = cJoint->GetBody0();
			NewtonBody* const parentBody = cJoint->GetBody1();

			if ((root == parentBody) && cJoint->IsType(dCustomRagdollMotor::GetKeyType()) && !filter.Find(childBody) ) {
				filter.Insert(0, childBody);
				list.Append (BodyJointPair (childBody, (dCustomRagdollMotor*)cJoint));
				stackMem[stack] = childBody;
				stack ++;
			}
		}
	}
}

void dCustomRagdollMotor::dSaveLoad::Save (const char* const fileName, NewtonBody* const rootbody)
{
	char* const oldloc = setlocale(LC_ALL, 0);
	setlocale(LC_ALL, "C");

	dList<BodyJointPair> list;
	GetBodyList (list, rootbody);

	FILE* const file = fopen(fileName, "wt");
	dAssert(file);

	fprintf(file, "nodesCount: %d\n", list.GetCount());
	fprintf(file, "rootBone: %s\n\n", GetBodyUniqueName (rootbody));
	for (dList<BodyJointPair>::dListNode* ptr = list.GetFirst(); ptr; ptr = ptr->GetNext()) {
		dMatrix boneMatrix;
		dVector euler0;
		dVector euler1;
		NewtonCollisionInfoRecord collisionInfo;
		dFloat mass;
		dFloat ixx;

		BodyJointPair& pair = ptr->GetInfo(); 
		NewtonBody* const body = pair.m_body;
		
		NewtonBodyGetMatrix(body, &boneMatrix[0][0]);
		boneMatrix.GetEulerAngles(euler0, euler1);

		NewtonBodyGetMass(body, &mass, &ixx, &ixx, &ixx);
		NewtonCollision* const collision = NewtonBodyGetCollision(body);
		NewtonCollisionGetInfo(collision, &collisionInfo);

		fprintf(file, "node: %s\n", GetBodyUniqueName (body));
		fprintf(file, "  mass: %f\n", mass);
		fprintf(file, "  position: %f %f %f\n", boneMatrix.m_posit.m_x, boneMatrix.m_posit.m_y, boneMatrix.m_posit.m_z);
		fprintf(file, "  eulerAngles: %f %f %f\n", euler0.m_x * 180.0f / 3.141592f, euler0.m_y * 180.0f / 3.141592f, euler0.m_z * 180.0f / 3.141592f);

		switch (collisionInfo.m_collisionType) {
		case SERIALIZE_ID_SPHERE:
		{
			fprintf(file, "  shapeType: sphere\n");
			fprintf(file, "    radio: %f\n", collisionInfo.m_sphere.m_radio);
			break;
		}

		case SERIALIZE_ID_CAPSULE:
		{
			fprintf(file, "  shapeType: capsule\n");
			fprintf(file, "    radio0: %f\n", collisionInfo.m_capsule.m_radio0);
			fprintf(file, "    radio1: %f\n", collisionInfo.m_capsule.m_radio1);
			fprintf(file, "    height: %f\n", collisionInfo.m_capsule.m_height);
			break;
		}

		case SERIALIZE_ID_CONVEXHULL:
		{
			fprintf(file, "  shapeType: convexHull\n");
			fprintf(file, "    points: %d\n", collisionInfo.m_convexHull.m_vertexCount);
			const int stride = collisionInfo.m_convexHull.m_vertexStrideInBytes / sizeof(dFloat);
			const dFloat* points = collisionInfo.m_convexHull.m_vertex;
			for (int i = 0; i < collisionInfo.m_convexHull.m_vertexCount; i++) {
				dFloat x = points[i * stride + 0];
				dFloat y = points[i * stride + 1];
				dFloat z = points[i * stride + 2];
				fprintf(file, "    convexHullPoint: %f %f %f\n", x, y, z);
			}
			break;
		}

		default:
		{
			dAssert(0);
		}
		}

		dMatrix shapeMatrix(&collisionInfo.m_offsetMatrix[0][0]);
		shapeMatrix.GetEulerAngles(euler0, euler1);
		fprintf(file, "    shapeScale: %f %f %f\n", 1.0f, 1.0f, 1.0f);
		fprintf(file, "    shapePosition: %f %f %f\n", shapeMatrix.m_posit.m_x, shapeMatrix.m_posit.m_y, shapeMatrix.m_posit.m_z);
		fprintf(file, "    shapeEulerAngle: %f %f %f\n", euler0.m_x * 180.0f / 3.141592f, euler0.m_y * 180.0f / 3.141592f, euler0.m_z * 180.0f / 3.141592f);
		fprintf(file, "nodeEnd:\n\n");
	}


	fprintf(file, "jointsCount: %d\n", list.GetCount() - 1);
	for (dList<BodyJointPair>::dListNode* ptr = list.GetFirst()->GetNext(); ptr; ptr = ptr->GetNext()) {
		BodyJointPair& pair = ptr->GetInfo(); 
		NewtonBody* const body = pair.m_body;
		dCustomRagdollMotor* const joint = pair.m_joint;
		NewtonBody* const parentBody = joint->GetBody1();
			
		fprintf(file, "joint: %s\n", joint->GetTypeName());
		fprintf(file, "  childBody: %s\n", GetBodyUniqueName (body));
		fprintf(file, "  parentBody: %s\n", GetBodyUniqueName (parentBody));

		dMatrix childMatrix (joint->GetMatrix0());
		dMatrix parentMatrix (joint->GetMatrix1());

		dVector euler;
		dVector childEuler;
		dVector parentEuler;

		childMatrix.GetEulerAngles(childEuler, euler);
		parentMatrix.GetEulerAngles(parentEuler, euler);

		childEuler = childEuler.Scale (180.0f / 3.141592f);
		parentEuler = parentEuler.Scale (180.0f / 3.141592f);

		fprintf(file, "  childPivot: %f %f %f\n", childMatrix.m_posit.m_x, childMatrix.m_posit.m_y, childMatrix.m_posit.m_z);
		fprintf(file, "  childEulers: %f %f %f\n", childEuler.m_x, childEuler.m_y, childEuler.m_z);

		fprintf(file, "  parentPivot: %f %f %f\n", parentMatrix.m_posit.m_x, parentMatrix.m_posit.m_y, parentMatrix.m_posit.m_z);
		fprintf(file, "  parentEulers: %f %f %f\n", parentEuler.m_x, parentEuler.m_y, parentEuler.m_z);

		if (joint->IsType(dCustomRagdollMotor_1dof::GetKeyType())) {
//			dAssert (0);
/*
			dFloat minAngle;
			dFloat maxAngle;

			dCustomRagdollMotor* const ragdollJoint = (dCustomRagdollMotor*) joint;
			ragdollJoint->GetTwistAngle(minAngle, maxAngle);
			fprintf(file, "  minTwistAngle: %f\n", minAngle * 180.0f / 3.141592f);
			fprintf(file, "  maxTwistAngle: %f\n", maxAngle * 180.0f / 3.141592f);

			ragdollJoint->GetYawAngles(minAngle, maxAngle);
			fprintf(file, "  coneAngle: %f\n", maxAngle * 180.0f / 3.141592f);
*/
		} else if (joint->IsType(dCustomRagdollMotor_2dof::GetKeyType())) {
//			dAssert (0);
		} else if (joint->IsType(dCustomRagdollMotor_3dof::GetKeyType())) {
//			dAssert (0);
		}

		fprintf(file, "jointEnd:\n\n");
	}


	fclose(file);
	setlocale(LC_ALL, oldloc);
}

