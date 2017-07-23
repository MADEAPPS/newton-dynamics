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
//IMPLEMENT_CUSTOM_JOINT(dCustomRagdollMotor_3dof)

NewtonBody* dCustomRagdollMotor::dSaveLoad::Load(const char* const fileName)
{
	char* const oldloc = setlocale(LC_ALL, 0);
	setlocale(LC_ALL, "C");
	FILE* const file = fopen(fileName, "rt");
	dAssert(file);

	char token[256];
	char rootBodyName[256];

	dTree<NewtonBody*, const dString> bodyMap;

	int nodesCount;
	fscanf(file, "%s %d", token, &nodesCount);
	fscanf(file, "%s %s", token, rootBodyName);
	for (int i = 0; i < nodesCount; i++) {
		ParseRigidBody(file, bodyMap);
	}

	int jointsCount;
	fscanf(file, "%s %d", token, &jointsCount);
	for (int i = 0; i < jointsCount; i++) {
		ParseJoint(file, bodyMap);
	}

	fclose(file);
	setlocale(LC_ALL, oldloc);
	return bodyMap.Find(rootBodyName)->GetInfo();
}


void dCustomRagdollMotor::dSaveLoad::Save(const char* const fileName, NewtonBody* const rootbody)
{
	char* const oldloc = setlocale(LC_ALL, 0);
	setlocale(LC_ALL, "C");

	dList<BodyJointPair> list;
	GetBodyList(list, rootbody);

	FILE* const file = fopen(fileName, "wt");
	dAssert(file);

	fprintf(file, "nodesCount: %d\n", list.GetCount());
	fprintf(file, "rootBone: %s\n\n", GetBodyUniqueName(rootbody));
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

		fprintf(file, "node: %s\n", GetBodyUniqueName(body));
		fprintf(file, "  mass: %f\n", mass);
		fprintf(file, "  position: %f %f %f\n", boneMatrix.m_posit.m_x, boneMatrix.m_posit.m_y, boneMatrix.m_posit.m_z);
		fprintf(file, "  eulerAngles: %f %f %f\n", euler0.m_x * 180.0f / 3.141592f, euler0.m_y * 180.0f / 3.141592f, euler0.m_z * 180.0f / 3.141592f);

		switch (collisionInfo.m_collisionType) 
		{
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
		fprintf(file, "  childBody: %s\n", GetBodyUniqueName(body));
		fprintf(file, "  parentBody: %s\n", GetBodyUniqueName(parentBody));

		dMatrix childMatrix(joint->GetMatrix0());
		dMatrix parentMatrix(joint->GetMatrix1());

		dVector euler;
		dVector childEuler;
		dVector parentEuler;
		dFloat minAngle;
		dFloat maxAngle;

		childMatrix.GetEulerAngles(childEuler, euler);
		parentMatrix.GetEulerAngles(parentEuler, euler);

		childEuler = childEuler.Scale(180.0f / 3.141592f);
		parentEuler = parentEuler.Scale(180.0f / 3.141592f);

		fprintf(file, "  childPivot: %f %f %f\n", childMatrix.m_posit.m_x, childMatrix.m_posit.m_y, childMatrix.m_posit.m_z);
		fprintf(file, "  childEulers: %f %f %f\n", childEuler.m_x, childEuler.m_y, childEuler.m_z);

		fprintf(file, "  parentPivot: %f %f %f\n", parentMatrix.m_posit.m_x, parentMatrix.m_posit.m_y, parentMatrix.m_posit.m_z);
		fprintf(file, "  parentEulers: %f %f %f\n", parentEuler.m_x, parentEuler.m_y, parentEuler.m_z);

		if (joint->IsType(dCustomRagdollMotor_1dof::GetType())) {
			dCustomRagdollMotor_1dof* const ragdollJoint = (dCustomRagdollMotor_1dof*)joint;

			ragdollJoint->GetTwistAngle(minAngle, maxAngle);
			fprintf(file, "  minTwistAngle: %f\n", minAngle * 180.0f / 3.141592f);
			fprintf(file, "  maxTwistAngle: %f\n", maxAngle * 180.0f / 3.141592f);

		} else if (joint->IsType(dCustomRagdollMotor_2dof::GetType())) {
			dCustomRagdollMotor_2dof* const ragdollJoint = (dCustomRagdollMotor_2dof*)joint;
			fprintf(file, "  coneAngle: %f\n", ragdollJoint->GetConeAngle() * 180.0f / 3.141592f);
/*
		} else if (joint->IsType(dCustomRagdollMotor_3dof::GetType())) {
			dCustomRagdollMotor_3dof* const ragdollJoint = (dCustomRagdollMotor_3dof*)joint;

			ragdollJoint->GetTwistAngle(minAngle, maxAngle);
			fprintf(file, "  minTwistAngle: %f\n", minAngle * 180.0f / 3.141592f);
			fprintf(file, "  maxTwistAngle: %f\n", maxAngle * 180.0f / 3.141592f);

			ragdollJoint->GetRollAngles(minAngle, maxAngle);
			fprintf(file, "  minRollAngle: %f\n", minAngle * 180.0f / 3.141592f);
			fprintf(file, "  maxRollAngle: %f\n", maxAngle * 180.0f / 3.141592f);

			ragdollJoint->GetYawAngles(minAngle, maxAngle);
			fprintf(file, "  minYawAngle: %f\n", minAngle * 180.0f / 3.141592f);
			fprintf(file, "  maxYawAngle: %f\n", maxAngle * 180.0f / 3.141592f);
*/
		} else {
			dAssert (0);
		}

		fprintf(file, "jointEnd:\n\n");
	}

	fclose(file);
	setlocale(LC_ALL, oldloc);
}


NewtonCollision* dCustomRagdollMotor::dSaveLoad::ParseCollisonShape(FILE* const file)
{
	char token[256];
	NewtonCollision* collision = NULL;

	fscanf(file, "%s", token);

	if (!strcmp(token, "sphere")) {
		dFloat radio;
		fscanf(file, "%s %f", token, &radio);
		collision = NewtonCreateSphere(m_world, radio, 0, NULL);
	} else if (!strcmp(token, "capsule")) {
		dFloat radio0;
		dFloat radio1;
		dFloat height;
		fscanf(file, "%s %f %s %f %s %f", token, &radio0, token, &radio1, token, &height);
		collision = NewtonCreateCapsule(m_world, radio0, radio1, height, 0, NULL);
	} else if (!strcmp(token, "convexHull")) {
		dVector array[1024];
		int pointCount;
		fscanf(file, "%s %d", token, &pointCount);
		for (int i = 0; i < pointCount; i++) {
			fscanf(file, "%s %f %f %f", token, &array[i].m_x, &array[i].m_y, &array[i].m_z);
		}
		collision = NewtonCreateConvexHull(m_world, pointCount, &array[0][0], sizeof(dVector), 1.0e-3f, 0, NULL);
	} else {
		dAssert(0);
	}

	dVector scale;
	dVector posit;
	dVector euler;
	fscanf(file, "%s %f %f %f", token, &scale.m_x, &scale.m_y, &scale.m_z);
	fscanf(file, "%s %f %f %f", token, &posit.m_x, &posit.m_y, &posit.m_z);
	fscanf(file, "%s %f %f %f", token, &euler.m_x, &euler.m_y, &euler.m_z);

	euler = euler.Scale(3.141592f / 180.0f);
	dMatrix matrix(euler.m_x, euler.m_y, euler.m_z, posit);
	NewtonCollisionSetMatrix(collision, &matrix[0][0]);
	//NewtonCollisionSetMode(collision, 0);
	return collision;
}

void dCustomRagdollMotor::dSaveLoad::ParseRigidBody(FILE* const file, dTree<NewtonBody*, const dString>& bodyMap)
{
	dVector posit(0.0f);
	dVector euler(0.0f);
	NewtonBody* body = NULL;
	NewtonCollision* collision = NULL;
	dFloat mass = 0.0f;

	char token[256];
	char nodeName[256];

	while (!feof(file)) {
		fscanf(file, "%s", token);

		if (!strcmp(token, "node:")) {
			fscanf(file, "%s", &nodeName);
		} else if (!strcmp(token, "mass:")) {
			fscanf(file, "%f", &mass);
		} else if (!strcmp(token, "position:")) {
			fscanf(file, "%f %f %f", &posit.m_x, &posit.m_y, &posit.m_z);
		} else if (!strcmp(token, "eulerAngles:")) {
			fscanf(file, "%f %f %f", &euler.m_x, &euler.m_y, &euler.m_z);
			euler = euler.Scale(3.141592f / 180.0f);
		} else if (!strcmp(token, "shapeType:")) {
			collision = ParseCollisonShape(file);
		} else if (!strcmp(token, "nodeEnd:")) {
			dMatrix matrix(euler.m_x, euler.m_y, euler.m_z, posit);
			body = NewtonCreateDynamicBody(m_world, collision, &matrix[0][0]);
			NewtonBodySetMassProperties(body, mass, collision);

			InitRigiBody(body, nodeName);
			NewtonDestroyCollision(collision);
			bodyMap.Insert(body, nodeName);
			break;
		} else {
			dAssert(0);
		}
	}
}


void dCustomRagdollMotor::dSaveLoad::ParseJoint(FILE* const file, const dTree<NewtonBody*, const dString>& bodyMap)
{
	char token[256];
	char jointType[256];
	char childName[256];
	char parentName[256];

	dVector childPivot(0.0f);
	dVector parentPivot(0.0f);
	dVector childEuler(0.0f);
	dVector parentEuler(0.0f);
	dFloat coneAngle = 0.0f;
	dFloat minTwistAngle = 0.0f;
	dFloat maxTwistAngle = 0.0f;

	NewtonBody* child = NULL;
	NewtonBody* parent = NULL;

	while (!feof(file)) {
		fscanf(file, "%s", token);
		if (!strcmp(token, "joint:")) {
			fscanf(file, "%s", &jointType);
		} else if (!strcmp(token, "childBody:")) {
			fscanf(file, "%s", childName);
			child = bodyMap.Find(childName)->GetInfo();
		} else if (!strcmp(token, "parentBody:")) {
			fscanf(file, "%s", parentName);
			parent = bodyMap.Find(parentName)->GetInfo();
		} else if (!strcmp(token, "childPivot:")) {
			fscanf(file, "%f %f %f", &childPivot.m_x, &childPivot.m_y, &childPivot.m_z);
		} else if (!strcmp(token, "childEulers:")) {
			fscanf(file, "%f %f %f", &childEuler.m_x, &childEuler.m_y, &childEuler.m_z);
		} else if (!strcmp(token, "parentPivot:")) {
			fscanf(file, "%f %f %f", &parentPivot.m_x, &parentPivot.m_y, &parentPivot.m_z);
		} else if (!strcmp(token, "parentEulers:")) {
			fscanf(file, "%f %f %f", &parentEuler.m_x, &parentEuler.m_y, &parentEuler.m_z);
		} else if (!strcmp(token, "coneAngle:")) {
			fscanf(file, "%f", &coneAngle);
		} else if (!strcmp(token, "minTwistAngle:")) {
			fscanf(file, "%f", &minTwistAngle);
		} else if (!strcmp(token, "maxTwistAngle:")) {
			fscanf(file, "%f", &maxTwistAngle);
		} else if (!strcmp(token, "jointEnd:")) {
			break;
		} else {
			dAssert(0);
		}
	}

	dAssert (child);
	dAssert (parent);

	dMatrix childBodyMatrix;
	dMatrix parentBodyMatrix;

	childEuler = childEuler.Scale(3.141592f / 180.0f);
	parentEuler = parentEuler.Scale(3.141592f / 180.0f);

	NewtonBodyGetMatrix(child, &childBodyMatrix[0][0]);
	NewtonBodyGetMatrix(parent, &parentBodyMatrix[0][0]);
	dMatrix childPinAndPivotInGlobalSpace(childEuler.m_x, childEuler.m_y, childEuler.m_z, childPivot);
	dMatrix parentPinAndPivotInGlobalSpace(parentEuler.m_x, parentEuler.m_y, parentEuler.m_z, parentPivot);

	childPinAndPivotInGlobalSpace = childPinAndPivotInGlobalSpace * childBodyMatrix;
	parentPinAndPivotInGlobalSpace = parentPinAndPivotInGlobalSpace * parentBodyMatrix;

	if (!strcmp(jointType, "dCustomRagdollMotor_1dof")) {
		dCustomRagdollMotor_1dof* const ragdollJoint = new dCustomRagdollMotor_1dof (parentPinAndPivotInGlobalSpace, child, parent);
		ragdollJoint->SetTwistAngle(minTwistAngle * 3.141592f / 180.0f, maxTwistAngle * 3.141592f / 180.0f);
	} else if (!strcmp(jointType, "dCustomRagdollMotor_2dof")) {
		dCustomRagdollMotor_2dof* const ragdollJoint = new dCustomRagdollMotor_2dof (parentPinAndPivotInGlobalSpace, child, parent);
		ragdollJoint->SetConeAngle(coneAngle * 3.141592f / 180.0f);
/*
	} else if (!strcmp(jointType, "dCustomRagdollMotor_3dof")) {
		dCustomRagdollMotor_3dof* const ragdollJoint = new dCustomRagdollMotor_3dof (parentPinAndPivotInGlobalSpace, child, parent);
		ragdollJoint->SetYawAngles(minYawAngle * 3.141592f / 180.0f, maxYawAngle * 3.141592f / 180.0f);
		ragdollJoint->SetRollAngles(minRollAngle * 3.141592f / 180.0f, maxRollAngle * 3.141592f / 180.0f);
		ragdollJoint->SetTwistAngle(minTwistAngle * 3.141592f / 180.0f, maxTwistAngle * 3.141592f / 180.0f);
	} else {
*/
		dAssert (0);
		new dCustomRagdollMotor (parentPinAndPivotInGlobalSpace, child, parent);
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

			if ((root == parentBody) && cJoint->IsType(dCustomRagdollMotor::GetType()) && !filter.Find(childBody) ) {
				filter.Insert(0, childBody);
				list.Append (BodyJointPair (childBody, (dCustomRagdollMotor*)cJoint));
				stackMem[stack] = childBody;
				stack ++;
			}
		}
	}
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


/*
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
#if 0
	dMatrix matrix0;
	dMatrix matrix1;
	dFloat yawAngle;
	dFloat rollAngle;
	dFloat twistAngle;

	dCustomRagdollMotor::SubmitConstraints(timestep, threadIndex);
	CalcutaleMatrixAndEulers(matrix0, matrix1, twistAngle, rollAngle, yawAngle);

	NewtonUserJointAddAngularRow(m_joint, -twistAngle * 40.0f, &matrix0.m_front[0]);

	if (rollAngle < m_rollAngle.m_minAngle) {
		rollAngle -= m_rollAngle.m_minAngle;
		NewtonUserJointAddAngularRow(m_joint, -rollAngle, &matrix0.m_right[0]);
		NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
	} else if (rollAngle > m_rollAngle.m_maxAngle) {
		rollAngle -= m_rollAngle.m_maxAngle;
		NewtonUserJointAddAngularRow(m_joint, -rollAngle, &matrix0.m_right[0]);
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
		NewtonUserJointAddAngularRow(m_joint, -yawAngle, &matrix1.m_up[0]);
		NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);

	} else if (yawAngle > m_yawAngle.m_maxAngle) {
		yawAngle -= m_yawAngle.m_maxAngle;
		NewtonUserJointAddAngularRow(m_joint, -yawAngle, &matrix1.m_up[0]);
		NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
	}
	else if (m_motorMode) {
		dFloat accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_up[0]);
		NewtonUserJointSetRowAcceleration(m_joint, accel);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);
	}
#endif

	
	dMatrix matrix0;
	dMatrix matrix1;
	dFloat invTimestep = 1.0f / timestep;

	CalculateGlobalMatrix(matrix0, matrix1);
	dCustomRagdollMotor::SubmitConstraints(timestep, threadIndex);


bool m_isLimitJoint = true;
	if (m_isLimitJoint) {

		const dVector& coneDir0 = matrix0.m_front;
		const dVector& coneDir1 = matrix1.m_front;
		dFloat dot = coneDir0.DotProduct3(coneDir1);
		if (dot < -0.999f) {
			// near the singularity pole causing a flip, should never happen
			dAssert (0);
			
			return; 
		}

		// do the twist
		if (m_twistAngle.m_maxAngle >= m_twistAngle.m_minAngle) // twist restricted?
		{
			dQuaternion quat0(matrix0), quat1(matrix1);
			float *q0 = (float*)&quat0;
			float *q1 = (float*)&quat1;

			// factor rotation about x axis between quat0 and quat1. Code is an optimization of this: qt = q0.Inversed() * q1; halfTwistAngle = atan (qt.x / qt.w);
			float twistAngle = 2.0f * atan(
				((((q0[0] * q1[1]) + (-q0[1] * q1[0])) + (-q0[2] * q1[3])) - (-q0[3] * q1[2])) /
				((((q0[0] * q1[0]) - (-q0[1] * q1[1])) - (-q0[2] * q1[2])) - (-q0[3] * q1[3])));

			// select an axis for the twist - any on the unit arc from coneDir0 to coneDir1 would do - average seemed best after some tests
			dVector twistAxis = coneDir0 + coneDir1;
			twistAxis = twistAxis.Scale(1.0f / sqrt(twistAxis.DotProduct3(twistAxis)));

			if (m_maxTwistAngle == m_minTwistAngle) // no freedom for any twist
			{
				NewtonUserJointAddAngularRow(m_joint, twistAngle - m_maxTwistAngle, (float*)&twistAxis);
			}
			else if (twistAngle > m_maxTwistAngle) {
				NewtonUserJointAddAngularRow(m_joint, twistAngle - m_maxTwistAngle, (float*)&twistAxis);
				NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
			}
			else if (twistAngle < m_minTwistAngle) {
				NewtonUserJointAddAngularRow(m_joint, twistAngle - m_minTwistAngle, (float*)&twistAxis);
				NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
			}
		}

		// do the swing

		if (m_arcAngle <= 0.0f) // simple cone limit
		{
			float angle = acos(max(-1.0f, min(1.0f, dot))) - m_coneAngle;
			if (angle > 0.0f) {
				dVector swingAxis = (coneDir0.CrossProduct(coneDir1));
				swingAxis = swingAxis.Scale(1.0 / sqrt(swingAxis.DotProduct3(swingAxis)));
				dFloat sql = swingAxis.DotProduct3(swingAxis);
				if (sql > E) {
					NewtonUserJointAddAngularRow(m_joint, angle, (float*)&swingAxis);
					NewtonUserJointSetRowMinimumFriction(m_joint, 0.0);
				}
			}
		}
		else // cone on arc limit - think of an piece of pizza (arc) and an allowed max distance from it (cone):
		{
			// project current axis to the arc plane (y)
			dVector d = matrix1.UnrotateVector(matrix0.m_front);
			dVector cone = d; cone.m_y = 0;
			dFloat sql = cone.DotProduct3(cone);
			cone = (sql > E) ? cone.Scale(1.0f / sqrt(sql)) : dVector(1.0f, 0.0f, 0.0f, 0.0f);


			// clamp the result to be within the arc angle
			if (cone.m_x < m_arcAngleCos)
				cone = dVector(m_arcAngleCos, 0.0f, ((cone.m_z < 0.0f) ? -m_arcAngleSin : m_arcAngleSin));

			// do a regular cone constraint from that
			float angle = acos(max(-1.0f, min(1.0f, d.DotProduct3(cone)))) - m_coneAngle;
			if (angle >= 0.0f) {
				dVector swingAxis = matrix1.RotateVector(d.CrossProduct(cone));
				dFloat sql = swingAxis.DotProduct3(swingAxis);
				if (sql > E) {
					swingAxis = swingAxis.Scale(1.0f / sqrt(sql));
					NewtonUserJointAddAngularRow(m_joint, angle, (float*)&swingAxis);
					NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
				}
			}
		}

	}
	else // motor
	{
		dAssert (0);
		if (m_anim_speed != 0.0f) // some animation to illustrate purpose
		{
			m_anim_time += timestep * m_anim_speed;
			dFloat a0 = sin(m_anim_time);
			dFloat a1 = m_anim_offset * 3.14f;
			dVector axis(sin(a1), 0.0f, cos(a1));
			//dVector axis (1,0,0);
			m_target = dQuaternion(axis, a0 * 0.5f);
		}

		// measure error
		dQuaternion q0(matrix0);
		dQuaternion q1(matrix1);
		dQuaternion qt0 = m_target * q1;
		dQuaternion qErr = ((q0.DotProduct(qt0) < 0.0f) ? dQuaternion(-q0.m_q0, q0.m_q1, q0.m_q2, q0.m_q3) : dQuaternion(q0.m_q0, -q0.m_q1, -q0.m_q2, -q0.m_q3)) * qt0;
		qErr.Normalize();

		dFloat errorAngle = 2.0f * acos(dMax(dFloat(-1.0f), dMin(dFloat(1.0f), qErr.m_q0)));
		dVector errorAngVel(0, 0, 0);

		dMatrix basis;
		if (errorAngle > 1.0e-10f) {
			dVector errorAxis(qErr.m_q1, qErr.m_q2, qErr.m_q3, 0.0f);
			errorAxis = errorAxis.Scale(1.0f / dSqrt(errorAxis.DotProduct3(errorAxis)));
			errorAngVel = errorAxis.Scale(errorAngle * invTimestep);

			basis = dGrammSchmidt(errorAxis);
		}
		else {
			basis = dMatrix(qt0, dVector(0.0f, 0.0f, 0.0f, 1.0f));
		}

		dVector angVel0(0.0f);
		dVector angVel1(0.0f);
		NewtonBodyGetOmega(m_body0, (dFloat*)&angVel0);
		NewtonBodyGetOmega(m_body1, (dFloat*)&angVel1);

		dVector angAcc = (errorAngVel.Scale(m_reduceError) - (angVel0 - angVel1)).Scale(invTimestep);

		dCustomBallAndSocket::SubmitConstraints(timestep, threadIndex);
		// motors
		for (int n = 0; n < 3; n++) {
			// calculate the desired acceleration
			dVector &axis = basis[n];
			dFloat relAccel = angAcc.DotProduct3(axis);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &axis[0]);
			NewtonUserJointSetRowAcceleration(m_joint, relAccel);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		}
	}
}


*/



dCustomRagdollMotor_1dof::dCustomRagdollMotor_1dof(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomRagdollMotor(pinAndPivotFrame, child, parent)
	,m_minTwistAngle(-30.0f * 3.141592f / 180.0f)
	,m_maxTwistAngle( 30.0f * 3.141592f / 180.0f)
{
}

dCustomRagdollMotor_1dof::dCustomRagdollMotor_1dof(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData)
	:dCustomRagdollMotor(child, parent, callback, userData)
	,m_minTwistAngle(0.0f)
	,m_maxTwistAngle(0.0f)
{
	callback(userData, &m_minTwistAngle, sizeof(m_minTwistAngle));
	callback(userData, &m_maxTwistAngle, sizeof(m_maxTwistAngle));
}

void dCustomRagdollMotor_1dof::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomRagdollMotor::Serialize(callback, userData);
	callback(userData, &m_minTwistAngle, sizeof(m_minTwistAngle));
	callback(userData, &m_maxTwistAngle, sizeof(m_maxTwistAngle));
}


void dCustomRagdollMotor_1dof::SetTwistAngle(dFloat minAngle, dFloat maxAngle)
{
	m_minTwistAngle = dClamp(-dAbs(minAngle), -170.0f * 3.141582f / 180.0f, 170.0f * 3.141582f / 180.0f);
	m_maxTwistAngle = dClamp(dAbs(maxAngle), -170.0f * 3.141582f / 180.0f, 170.0f * 3.141582f / 180.0f);
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

	dVector point (radius, 0.0f, 0.0f, 0.0f);
	dFloat angleStep = (m_maxTwistAngle - m_minTwistAngle) / subdiv;
	dFloat angle0 = m_minTwistAngle;

	dVector arch[subdiv + 1];
	debugDisplay->SetColor(this, dVector (1.0f, 1.0f, 0.0f, 0.0f));
	for (int i = 0; i <= subdiv; i++) {
		dVector p (matrix1.TransformVector(dYawMatrix(-angle0).RotateVector(point)));
		arch[i] = p;
		debugDisplay->DrawLine(this, matrix1.m_posit, p);
		angle0 += angleStep;
	}

	for (int i = 0; i < subdiv; i++) {
		debugDisplay->DrawLine(this, arch[i], arch[i + 1]);
	}
}


void dCustomRagdollMotor_1dof::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	CalculateGlobalMatrix(matrix0, matrix1);
	dCustomRagdollMotor::SubmitConstraints(timestep, threadIndex);

	const dVector& coneDir0 = matrix0.m_front;
	const dVector& coneDir1 = matrix1.m_front;

	// do the twist
	dQuaternion quat0(matrix0);
	dQuaternion quat1(matrix1);

	if (quat0.DotProduct(quat1) < 0.0f) {
		quat0.Scale (-1.0f);
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
	dFloat dot = coneDir0.DotProduct3(coneDir1);
	if (dot > -0.999f) {
		dVector twistAxis = coneDir0 + coneDir1;
		twistAxis = twistAxis.Scale(1.0f / dSqrt(twistAxis.DotProduct3(twistAxis)));
		NewtonUserJointAddAngularRow(m_joint, twistAngle, &twistAxis[0]);
	} else {
		NewtonUserJointAddAngularRow(m_joint, twistAngle, &coneDir1[0]);
	}

	// do the swing
	// cone on arc limit - think of an piece of pizza (arc) and an allowed max distance from it (cone):
	// project current axis to the arc plane (y)
	dVector d (matrix1.UnrotateVector(matrix0.m_front));
	dVector cone (d); 
	cone.m_y = 0;
			
	dFloat sql = cone.DotProduct3(cone);
	dAssert (sql > 0.0f);
	cone = cone.Scale(1.0f / sqrt(sql));

	// do a regular cone constraint from that
	dVector planeDir (matrix1.RotateVector(cone));
	dVector swingAxis (planeDir.CrossProduct(matrix1.m_up));
	float swingAngle = CalculateAngle (matrix0.m_front, planeDir, swingAxis);
	NewtonUserJointAddAngularRow(m_joint, swingAngle, &swingAxis[0]);

	swingAngle = CalculateAngle (planeDir, matrix1.m_front, matrix1.m_up);
	if (swingAngle < m_minTwistAngle) {
		swingAngle = swingAngle - m_minTwistAngle;
		NewtonUserJointAddAngularRow(m_joint, swingAngle, &matrix1.m_up[0]);
		NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
	} else if (swingAngle > m_maxTwistAngle) {
		swingAngle = swingAngle - m_maxTwistAngle;
		NewtonUserJointAddAngularRow(m_joint, swingAngle, &matrix1.m_up[0]);
		NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
	} else if (m_motorMode) {
		dFloat accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_up[0]);
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

dCustomRagdollMotor_2dof::dCustomRagdollMotor_2dof(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData)
	:dCustomRagdollMotor(child, parent, callback, userData)
	,m_coneAngle(0.0f)
{
	callback(userData, &m_coneAngle, sizeof(m_coneAngle));
}

void dCustomRagdollMotor_2dof::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomRagdollMotor::Serialize(callback, userData);
	callback(userData, &m_coneAngle, sizeof(m_coneAngle));
}

void dCustomRagdollMotor_2dof::SetConeAngle(dFloat angle)
{
	m_coneAngle = dMin(-dAbs(angle), -150.0f * 3.141582f / 180.0f);
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
/*
	dVector p[4];
	dVector o[4];

	dFloat length = 0.125f;

	dMatrix minRollMatrix(dRollMatrix(m_rollAngle.m_minAngle));
	dMatrix maxRollMatrix(dRollMatrix(m_rollAngle.m_maxAngle));

	p[0] = dVector(0.25f, 0.0f, length, 0.0f);
	p[1] = dVector(0.25f, 0.0f, -length, 0.0f);
	p[2] = dVector(0.25f, 0.0f, -length, 0.0f);
	p[3] = dVector(0.25f, 0.0f, length, 0.0f);

	o[0] = dVector(0.0f, 0.0f, length, 0.0f);
	o[1] = dVector(0.0f, 0.0f, -length, 0.0f);
	o[2] = dVector(0.0f, 0.0f, -length, 0.0f);
	o[3] = dVector(0.0f, 0.0f, length, 0.0f);

	p[0] = minRollMatrix.RotateVector(p[0]);
	p[1] = minRollMatrix.RotateVector(p[1]);
	p[2] = maxRollMatrix.RotateVector(p[2]);
	p[3] = maxRollMatrix.RotateVector(p[3]);

	debugDisplay->SetColor(this, dVector(0.0f, 0.0f, 0.5f));
	debugDisplay->DrawLine(this, matrix1.TransformVector(o[0]), matrix1.TransformVector(o[1]));
	for (int i = 0, i0 = 3; i < 4; i++) {
		dVector p0 = matrix1.TransformVector(p[i0]);
		dVector p1 = matrix1.TransformVector(p[i]);
		debugDisplay->DrawLine(this, p0, p1);
		debugDisplay->DrawLine(this, matrix1.TransformVector(o[i]), p1);
		i0 = i;
	}


	dMatrix minYawMatrix(dYawMatrix(m_yawAngle.m_minAngle));
	dMatrix maxYawMatrix(dYawMatrix(m_yawAngle.m_maxAngle));
	p[0] = dVector(0.25f, length, 0.0f, 0.0f);
	p[1] = dVector(0.25f, -length, 0.0f, 0.0f);
	p[2] = dVector(0.25f, -length, 0.0f, 0.0f);
	p[3] = dVector(0.25f, length, 0.0f, 0.0f);

	o[0] = dVector(0.0f, length, 0.0f, 0.0f);
	o[1] = dVector(0.0f, -length, 0.0f, 0.0f);
	o[2] = dVector(0.0f, -length, 0.0f, 0.0f);
	o[3] = dVector(0.0f, length, 0.0f, 0.0f);

	p[0] = minYawMatrix.RotateVector(p[0]);
	p[1] = minYawMatrix.RotateVector(p[1]);
	p[2] = maxYawMatrix.RotateVector(p[2]);
	p[3] = maxYawMatrix.RotateVector(p[3]);

	debugDisplay->SetColor(this, dVector(0.0f, 0.5f, 0.0f));
	debugDisplay->DrawLine(this, matrix1.TransformVector(o[0]), matrix1.TransformVector(o[1]));
	for (int i = 0, i0 = 3; i < 4; i++) {
		dVector p0 = matrix1.TransformVector(p[i0]);
		dVector p1 = matrix1.TransformVector(p[i]);
		debugDisplay->DrawLine(this, p0, p1);
		debugDisplay->DrawLine(this, matrix1.TransformVector(o[i]), p1);
		i0 = i;
	}
*/
}


void dCustomRagdollMotor_2dof::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;
//	dFloat yawAngle;
//	dFloat rollAngle;
//	dFloat twistAngle;
	dCustomRagdollMotor::SubmitConstraints(timestep, threadIndex);
/*
	NewtonUserJointAddAngularRow(m_joint, -twistAngle * 40.0f, &matrix0.m_front[0]);

	if (rollAngle < m_rollAngle.m_minAngle) {
		rollAngle -= m_rollAngle.m_minAngle;
		NewtonUserJointAddAngularRow(m_joint, rollAngle, &matrix0.m_right[0]);
		NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
	}
	else if (rollAngle > m_rollAngle.m_maxAngle) {
		rollAngle -= m_rollAngle.m_maxAngle;
		NewtonUserJointAddAngularRow(m_joint, rollAngle, &matrix0.m_right[0]);
		NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
	}
	else if (m_motorMode) {
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
*/
}

