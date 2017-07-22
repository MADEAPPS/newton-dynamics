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

		if (joint->IsType(dCustomRagdollMotor_1dof::GetKeyType())) {
			dCustomRagdollMotor_1dof* const ragdollJoint = (dCustomRagdollMotor_1dof*)joint;

			ragdollJoint->GetTwistAngle(minAngle, maxAngle);
			fprintf(file, "  minTwistAngle: %f\n", minAngle * 180.0f / 3.141592f);
			fprintf(file, "  maxTwistAngle: %f\n", maxAngle * 180.0f / 3.141592f);

		} else if (joint->IsType(dCustomRagdollMotor_2dof::GetKeyType())) {
			dCustomRagdollMotor_2dof* const ragdollJoint = (dCustomRagdollMotor_2dof*)joint;

			ragdollJoint->GetRollAngles(minAngle, maxAngle);
			fprintf(file, "  minRollAngle: %f\n", minAngle * 180.0f / 3.141592f);
			fprintf(file, "  maxRollAngle: %f\n", maxAngle * 180.0f / 3.141592f);

			ragdollJoint->GetYawAngles(minAngle, maxAngle);
			fprintf(file, "  minYawAngle: %f\n", minAngle * 180.0f / 3.141592f);
			fprintf(file, "  maxYawAngle: %f\n", maxAngle * 180.0f / 3.141592f);

		} else if (joint->IsType(dCustomRagdollMotor_3dof::GetKeyType())) {
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
	dFloat minRollAngle = 0.0f;
	dFloat maxRollAngle = 0.0f;
	dFloat minYawAngle = 0.0f;
	dFloat maxYawAngle = 0.0f;
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
		} else if (!strcmp(token, "minRollAngle:")) {
			fscanf(file, "%f", &minRollAngle);
		} else if (!strcmp(token, "maxRollAngle:")) {
			fscanf(file, "%f", &maxRollAngle);
		} else if (!strcmp(token, "minYawAngle:")) {
			fscanf(file, "%f", &minYawAngle);
		} else if (!strcmp(token, "maxYawAngle:")) {
			fscanf(file, "%f", &maxYawAngle);
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
		ragdollJoint->SetYawAngles(minYawAngle * 3.141592f / 180.0f, maxYawAngle * 3.141592f / 180.0f);
		ragdollJoint->SetRollAngles(minRollAngle * 3.141592f / 180.0f, maxRollAngle * 3.141592f / 180.0f);
	} else if (!strcmp(jointType, "dCustomRagdollMotor_3dof")) {
		dCustomRagdollMotor_3dof* const ragdollJoint = new dCustomRagdollMotor_3dof (parentPinAndPivotInGlobalSpace, child, parent);
		ragdollJoint->SetYawAngles(minYawAngle * 3.141592f / 180.0f, maxYawAngle * 3.141592f / 180.0f);
		ragdollJoint->SetRollAngles(minRollAngle * 3.141592f / 180.0f, maxRollAngle * 3.141592f / 180.0f);
		ragdollJoint->SetTwistAngle(minTwistAngle * 3.141592f / 180.0f, maxTwistAngle * 3.141592f / 180.0f);
	} else {
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

			if ((root == parentBody) && cJoint->IsType(dCustomRagdollMotor::GetKeyType()) && !filter.Find(childBody) ) {
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

void dCustomRagdollMotor::SetJointTorque(dFloat torque)
{
	m_torque = dAbs(torque);
}


dFloat dCustomRagdollMotor::GetJointTorque() const
{
	return m_torque;
}


void dCustomRagdollMotor::CalcutaleMatrixAndEulers(dMatrix& matrix0, dMatrix& matrix1, dFloat& twistAngle, dFloat& rollAngle, dFloat& yawAngle) const
{
	CalculateGlobalMatrix(matrix0, matrix1);

//	matrix0 = dPitchMatrix (10.0f * 3.141592f / 180.0f) * dRollMatrix (40.0f * 3.141592f / 180.0f) * dYawMatrix (30.0f * 3.141592f / 180.0f) * matrix1;
	dMatrix matrix(matrix0 * matrix1.Inverse());

	dAssert(matrix.m_front.m_y < 0.999f);

	rollAngle = dAsin(matrix.m_front.m_y);
	yawAngle = dAtan2(-matrix.m_front.m_z, matrix.m_front.m_x);
	twistAngle = dAtan2(-matrix.m_right.m_y, matrix.m_up.m_y);

	dTrace(("%f %f %f\n", twistAngle * 180.0f / 3.1416f, rollAngle * 180.0f / 3.1416f, yawAngle * 180.0f / 3.1416f));
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


void dCustomRagdollMotor_1dof::Debug(dDebugDisplay* const debugDisplay) const
{
	dCustomRagdollMotor::Debug(debugDisplay);

	dMatrix matrix0;
	dMatrix matrix1;

	CalculateGlobalMatrix(matrix0, matrix1);


	dVector p[4];
	dVector o[4];

	dFloat length = 0.125f;

	dMatrix minPitchMatrix(dPitchMatrix(m_twistAngle.m_minAngle));
	dMatrix maxPitchMatrix(dPitchMatrix(m_twistAngle.m_maxAngle));

	p[0] = dVector( length, 0.0f, 0.25f, 0.0f);
	p[1] = dVector(-length, 0.0f, 0.25f, 0.0f);
	p[2] = dVector(-length, 0.0f, 0.25f, 0.0f);
	p[3] = dVector( length, 0.0f, 0.25f, 0.0f);

	o[0] = dVector( length, 0.0f, 0.0f, 0.0f);
	o[1] = dVector(-length, 0.0f, 0.0f, 0.0f);
	o[2] = dVector(-length, 0.0f, 0.0f, 0.0f);
	o[3] = dVector( length, 0.0f, 0.0f, 0.0f);

	p[0] = minPitchMatrix.RotateVector(p[0]);
	p[1] = minPitchMatrix.RotateVector(p[1]);
	p[2] = maxPitchMatrix.RotateVector(p[2]);
	p[3] = maxPitchMatrix.RotateVector(p[3]);

	debugDisplay->SetColor(this, dVector(0.5f, 0.0f, 0.0f));
	debugDisplay->DrawLine(this, matrix1.TransformVector(o[0]), matrix1.TransformVector(o[1]));
	for (int i = 0, i0 = 3; i < 4; i++) {
		dVector p0 = matrix1.TransformVector(p[i0]);
		dVector p1 = matrix1.TransformVector(p[i]);
		debugDisplay->DrawLine(this, p0, p1);
		debugDisplay->DrawLine(this, matrix1.TransformVector(o[i]), p1);
		i0 = i;
	}
}

void dCustomRagdollMotor_2dof::Debug(dDebugDisplay* const debugDisplay) const
{

	dCustomRagdollMotor::Debug(debugDisplay);

	dMatrix matrix0;
	dMatrix matrix1;

	CalculateGlobalMatrix(matrix0, matrix1);
	

	dVector p[4];
	dVector o[4];

	dFloat length = 0.125f; 

	dMatrix minRollMatrix(dRollMatrix(m_rollAngle.m_minAngle));
	dMatrix maxRollMatrix(dRollMatrix(m_rollAngle.m_maxAngle));

	p[0] = dVector (0.25f, 0.0f,  length, 0.0f);
	p[1] = dVector (0.25f, 0.0f, -length, 0.0f);
	p[2] = dVector (0.25f, 0.0f, -length, 0.0f);
	p[3] = dVector (0.25f, 0.0f,  length, 0.0f);

	o[0] = dVector(0.0f, 0.0f,  length, 0.0f);
	o[1] = dVector(0.0f, 0.0f, -length, 0.0f);
	o[2] = dVector(0.0f, 0.0f, -length, 0.0f);
	o[3] = dVector(0.0f, 0.0f,  length, 0.0f);

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
	p[0] = dVector(0.25f,  length, 0.0f, 0.0f);
	p[1] = dVector(0.25f, -length, 0.0f, 0.0f);
	p[2] = dVector(0.25f, -length, 0.0f, 0.0f);
	p[3] = dVector(0.25f,  length, 0.0f, 0.0f);

	o[0] = dVector(0.0f,  length, 0.0f, 0.0f);
	o[1] = dVector(0.0f, -length, 0.0f, 0.0f);
	o[2] = dVector(0.0f, -length, 0.0f, 0.0f);
	o[3] = dVector(0.0f,  length, 0.0f, 0.0f);

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
}




void dCustomRagdollMotor_1dof::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;
	dFloat yawAngle;
	dFloat rollAngle;
	dFloat twistAngle;

	dCustomRagdollMotor::SubmitConstraints(timestep, threadIndex);
	CalcutaleMatrixAndEulers(matrix0, matrix1, twistAngle, rollAngle, yawAngle);

	NewtonUserJointAddAngularRow(m_joint, -yawAngle, &matrix1.m_up[0]);
	NewtonUserJointAddAngularRow(m_joint, -rollAngle, &matrix1.m_right[0]);

	if (twistAngle < m_twistAngle.m_minAngle) {
		twistAngle -= m_twistAngle.m_minAngle;
		NewtonUserJointAddAngularRow(m_joint, -twistAngle, &matrix1.m_front[0]);
		NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
	} else if (twistAngle > m_twistAngle.m_maxAngle) {
		twistAngle -= m_twistAngle.m_maxAngle;
		NewtonUserJointAddAngularRow(m_joint, -twistAngle, &matrix1.m_front[0]);
		NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
	} else if (m_motorMode) {
		dFloat accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_front[0]);
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
	CalcutaleMatrixAndEulers(matrix0, matrix1, twistAngle, rollAngle, yawAngle);

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
