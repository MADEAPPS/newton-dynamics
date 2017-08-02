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
#include "dCustomJoint.h"
#include "dCustomModelLoadSave.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
/*
class dCustomJointSaveLoad::BodyJointPair
{
	public:
	BodyJointPair(NewtonBody* const body, dCustomJoint* const joint)
		:m_body(body)
		,m_joint(joint)
	{
	}

	NewtonBody* m_body;
	dCustomJoint* m_joint;
};
*/

/*

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
	} else if (!strcmp(jointType, "dCustomRagdollMotor_3dof")) {
		dCustomRagdollMotor_3dof* const ragdollJoint = new dCustomRagdollMotor_3dof (parentPinAndPivotInGlobalSpace, child, parent);
		ragdollJoint->SetConeAngle(coneAngle * 3.141592f / 180.0f);
		ragdollJoint->SetTwistAngle(minTwistAngle * 3.141592f / 180.0f, maxTwistAngle * 3.141592f / 180.0f);
	} else {

		dAssert (0);
		new dCustomRagdollMotor (parentPinAndPivotInGlobalSpace, child, parent);
	}
}


*/


void dCustomJointSaveLoad::Load()
{
	dAssert (0);
}


void dCustomJointSaveLoad::GetBodiesAndJointsList (dList<const NewtonBody*>& bodylist, dList<const dCustomJoint*>& jointlist, NewtonBody* const rootbody)
{
	dTree<int, const NewtonBody*> bodyFilter;
	dTree<int, const dCustomJoint*> jointFilter;
	const NewtonBody* stackMem[256];

	int stack = 1;
	stackMem[0] = rootbody;
	while (stack) {
		stack--;
		const NewtonBody* const root = stackMem[stack];

		if (bodyFilter.Insert(0, root)) {
			bodylist.Append(root);
		}

		for (NewtonJoint* joint = NewtonBodyGetFirstJoint(root); joint; joint = NewtonBodyGetNextJoint(root, joint)) {
			dCustomJoint* const customJoint = (dCustomJoint*)NewtonJointGetUserData(joint);
			const NewtonBody* const body0 = customJoint->GetBody0();
			const NewtonBody* const body1 = customJoint->GetBody1();
			dAssert (body0);
			dAssert (body1);

			if (jointFilter.Insert (0, customJoint)) {
				jointlist.Append(customJoint);
			}

			if (body0 == root) {
				if (!bodyFilter.Find(body1)) {
					stackMem[stack] = body1;
					stack ++;
				}
			} else {
				dAssert (body1 == root);
				if (!bodyFilter.Find(body0)) {
					stackMem[stack] = body0;
					stack++;
				}
			}
		}
	}
}


void dCustomJointSaveLoad::SaveInt (const char* const token, int val) const
{
	fprintf(m_file, "  %s: %d\n", token, val);
}

void dCustomJointSaveLoad::SaveFloat (const char* const token, dFloat val) const
{
	fprintf(m_file, "  %s: %f\n", token, val);
}

void dCustomJointSaveLoad::SaveVector (const char* const token, const dVector& v) const
{
	fprintf(m_file, "  %s: %f %f %f\n", token, v.m_x, v.m_y, v.m_z);
}


void dCustomJointSaveLoad::Save(NewtonBody* const rootbody)
{
	dList<const NewtonBody*> bodyList;
	dList<const dCustomJoint*> jointList;
	GetBodiesAndJointsList(bodyList, jointList, rootbody);

	fprintf(m_file, "rootBone: %s\n\n", GetBodyUniqueName(rootbody));
	fprintf(m_file, "nodesCount: %d\n", bodyList.GetCount());
	for (dList<const NewtonBody*>::dListNode* ptr = bodyList.GetFirst(); ptr; ptr = ptr->GetNext()) {
		dMatrix boneMatrix;
		dVector euler0;
		dVector euler1;
		NewtonCollisionInfoRecord collisionInfo;
		dFloat mass;
		dFloat ixx;

		const NewtonBody* const body = ptr->GetInfo();

		NewtonBodyGetMatrix(body, &boneMatrix[0][0]);
		boneMatrix.GetEulerAngles(euler0, euler1);

		NewtonBodyGetMass(body, &mass, &ixx, &ixx, &ixx);
		NewtonCollision* const collision = NewtonBodyGetCollision(body);
		NewtonCollisionGetInfo(collision, &collisionInfo);

		fprintf(m_file, "node: %s\n", GetBodyUniqueName(body));

		fprintf(m_file, "  mass: %f\n", mass);
		fprintf(m_file, "  position: %f %f %f\n", boneMatrix.m_posit.m_x, boneMatrix.m_posit.m_y, boneMatrix.m_posit.m_z);
		fprintf(m_file, "  eulerAngles: %f %f %f\n", euler0.m_x * 180.0f / 3.141592f, euler0.m_y * 180.0f / 3.141592f, euler0.m_z * 180.0f / 3.141592f);

		switch (collisionInfo.m_collisionType) 
		{
			case SERIALIZE_ID_SPHERE:
			{
				fprintf(m_file, "  shapeType: sphere\n");
				fprintf(m_file, "    radio: %f\n", collisionInfo.m_sphere.m_radio);
				break;
			}

			case SERIALIZE_ID_CAPSULE:
			{
				fprintf(m_file, "  shapeType: capsule\n");
				fprintf(m_file, "    radio0: %f\n", collisionInfo.m_capsule.m_radio0);
				fprintf(m_file, "    radio1: %f\n", collisionInfo.m_capsule.m_radio1);
				fprintf(m_file, "    height: %f\n", collisionInfo.m_capsule.m_height);
				break;
			}

			case SERIALIZE_ID_CONVEXHULL:
			{
				fprintf(m_file, "  shapeType: convexHull\n");
				fprintf(m_file, "    points: %d\n", collisionInfo.m_convexHull.m_vertexCount);
				const int stride = collisionInfo.m_convexHull.m_vertexStrideInBytes / sizeof(dFloat);
				const dFloat* points = collisionInfo.m_convexHull.m_vertex;
				for (int i = 0; i < collisionInfo.m_convexHull.m_vertexCount; i++) {
					dFloat x = points[i * stride + 0];
					dFloat y = points[i * stride + 1];
					dFloat z = points[i * stride + 2];
					fprintf(m_file, "    v%d: %f %f %f\n", i, x, y, z);
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
		fprintf(m_file, "    shapeScale: %f %f %f\n", 1.0f, 1.0f, 1.0f);
		fprintf(m_file, "    shapePosition: %f %f %f\n", shapeMatrix.m_posit.m_x, shapeMatrix.m_posit.m_y, shapeMatrix.m_posit.m_z);
		fprintf(m_file, "    shapeEulerAngle: %f %f %f\n", euler0.m_x * 180.0f / 3.141592f, euler0.m_y * 180.0f / 3.141592f, euler0.m_z * 180.0f / 3.141592f);

		fprintf(m_file, "nodeEnd:\n\n");
	}


	fprintf(m_file, "jointsCount: %d\n", jointList.GetCount());
	for (dList<const dCustomJoint*>::dListNode* ptr = jointList.GetFirst(); ptr; ptr = ptr->GetNext()) {
		const dCustomJoint* const joint = ptr->GetInfo();

		fprintf(m_file, "joint: %s\n", joint->GetTypeName());
		fprintf(m_file, "  childBody: %s\n", GetBodyUniqueName(joint->GetBody0()));
		fprintf(m_file, "  parentBody: %s\n", GetBodyUniqueName(joint->GetBody1()));

		joint->Save (this);
/*
		dFloat minAngle;
		dFloat maxAngle;

		if (joint->IsType(dCustomRagdollMotor_1dof::GetType())) {
			dCustomRagdollMotor_1dof* const ragdollJoint = (dCustomRagdollMotor_1dof*)joint;

			ragdollJoint->GetTwistAngle(minAngle, maxAngle);
			fprintf(file, "  minTwistAngle: %f\n", minAngle * 180.0f / 3.141592f);
			fprintf(file, "  maxTwistAngle: %f\n", maxAngle * 180.0f / 3.141592f);

		}
		else if (joint->IsType(dCustomRagdollMotor_2dof::GetType())) {
			dCustomRagdollMotor_2dof* const ragdollJoint = (dCustomRagdollMotor_2dof*)joint;
			fprintf(file, "  coneAngle: %f\n", ragdollJoint->GetConeAngle() * 180.0f / 3.141592f);

		}
		else if (joint->IsType(dCustomRagdollMotor_3dof::GetType())) {

		}
*/
		fprintf(m_file, "jointEnd:\n\n");
	}
}
