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

void dCustomJointSaveLoad::GetBodiesAndJointsList (dList<NewtonBody*>& bodylist, dList<dCustomJoint*>& jointlist, NewtonBody* const rootbody)
{
	dTree<int, const dCustomJoint*> jointFilter;
	NewtonBody* stackMem[256];
	m_bodyFilter.RemoveAll();

	int stack = 1;
	stackMem[0] = rootbody;
	int enumeration = 0;
	while (stack) {
		stack--;
		NewtonBody* const root = stackMem[stack];

		if (m_bodyFilter.Insert(enumeration, root)) {
			enumeration ++;
			bodylist.Append(root);
		}

		for (NewtonJoint* joint = NewtonBodyGetFirstJoint(root); joint; joint = NewtonBodyGetNextJoint(root, joint)) {
			dCustomJoint* const customJoint = (dCustomJoint*)NewtonJointGetUserData(joint);
			NewtonBody* const body0 = customJoint->GetBody0();
			NewtonBody* const body1 = customJoint->GetBody1();

			if (jointFilter.Insert (0, customJoint)) {
				jointlist.Append(customJoint);
			}

			if (body0 && body1) {
				if (body0 == root) {
					if (!m_bodyFilter.Find(body1)) {
						stackMem[stack] = body1;
						stack ++;
					}
				} else if (body1){
					if (!m_bodyFilter.Find(body0)) {
						stackMem[stack] = body0;
						stack++;
					}
				}
			}
		}
	}
}

const char* dCustomJointSaveLoad::NextToken() const
{
	fscanf(m_file, "%s:", &m_token[0]);
	return m_token;
}

int dCustomJointSaveLoad::LoadInt() const
{
	int val;
	fscanf(m_file, "%d", &val);
	return val;
}

dFloat dCustomJointSaveLoad::LoadFloat() const
{
	dFloat val;
	fscanf(m_file, "%f", &val);
	return val;
}

dVector dCustomJointSaveLoad::LoadVector() const
{
	dVector val(0.0f);
	fscanf(m_file, "%f %f %f", &val.m_x, &val.m_y, &val.m_z);
	return val;
}

dMatrix dCustomJointSaveLoad::LoadMatrix () const
{
	dVector euler;
	dVector posit;
	fscanf(m_file, "%f %f %f %f %f %f", &euler.m_x, &euler.m_y, &euler.m_z, &posit.m_x, &posit.m_y, &posit.m_z);

	posit.m_w = 1.0f;
	euler = euler.Scale(3.141592f / 180.0f);
	return dMatrix (euler.m_x, euler.m_y, euler.m_z, posit);
}

void dCustomJointSaveLoad::LoadName(char* const name) const
{
	fscanf(m_file, "%s", name);
}


void dCustomJointSaveLoad::SaveInt (const char* const token, int val) const
{
	fprintf(m_file, "%s: %d\n", token, val);
}

void dCustomJointSaveLoad::SaveFloat (const char* const token, dFloat val) const
{
	fprintf(m_file, "%s: %f\n", token, val);
}

void dCustomJointSaveLoad::SaveVector (const char* const token, const dVector& v) const
{
	fprintf(m_file, "%s: %f %f %f\n", token, v.m_x, v.m_y, v.m_z);
}

void dCustomJointSaveLoad::SaveMatrix (const char* const token, const dMatrix& matrix) const
{
	dVector euler0;
	dVector euler1;
	matrix.GetEulerAngles(euler0, euler1);
	euler0 = euler0.Scale (180.0f / 3.141592f);
	fprintf(m_file, "%s: %f %f %f %f %f %f\n", token, euler0.m_x, euler0.m_y, euler0.m_z, matrix.m_posit.m_x, matrix.m_posit.m_y, matrix.m_posit.m_z);
}

void dCustomJointSaveLoad::SaveName (const char* const token, const char* const name) const
{
	fprintf(m_file, "%s: %s\n", token, name);
}

void dCustomJointSaveLoad::Newline () const
{
	fprintf(m_file, "\n");
}

int dCustomJointSaveLoad::FindBodyId(NewtonBody* const body) const
{
	dTree<int, NewtonBody*>::dTreeNode* const node = m_bodyFilter.Find(body);
	return node ? node->GetInfo() : -1;
}

NewtonBody* dCustomJointSaveLoad::FindBodyId(int id) const
{
	dTree<int, NewtonBody*>::Iterator iter (m_bodyFilter);
	for (iter.Begin(); iter; iter ++) {
		if (*iter == id) {
			return iter.GetKey();
		}
	}
	return NULL;
}

int dCustomJointSaveLoad::FindJointId(dCustomJoint* const joint) const
{
	dTree<dCustomJoint*, int>::Iterator iter (m_jointFilter);
	for (iter.Begin(); iter; iter ++) {
		if (iter.GetNode()->GetInfo() == joint) {
			return iter.GetKey();
		}
	}
	return -1;
}



void dCustomJointSaveLoad::SaveBodyList(dList<NewtonBody*>& bodyList)
{
	class dEntry
	{
		public:
		dEntry(int id, NewtonCollision* const collision)
			:m_id (id)
			,m_collision (collision)
		{
		}

		int m_id;
		NewtonCollision* m_collision;
	};

	dTree<dEntry, const void*> collisionList;
	int id = 0;
	for (dList<NewtonBody*>::dListNode* ptr = bodyList.GetFirst(); ptr; ptr = ptr->GetNext()) {
		NewtonBody* const body = ptr->GetInfo();
		NewtonCollision* const collision = NewtonBodyGetCollision(body);
		if (collisionList.Insert(dEntry (id, collision), NewtonCollisionDataPointer(collision))) {
			id ++;
		}
	}

	SaveInt("shapeCount", collisionList.GetCount());
	dTree<dEntry, const void*>::Iterator iter(collisionList);
	for (iter.Begin(); iter; iter ++) {

		dEntry& entry = *iter;
		SaveInt("collision", entry.m_id);
		
		NewtonCollisionInfoRecord collisionInfo;
		NewtonCollisionGetInfo(entry.m_collision, &collisionInfo);

		switch (collisionInfo.m_collisionType) 
		{
			case SERIALIZE_ID_SPHERE:
			{
				SaveName("\tshapeType", "sphere");
				SaveFloat("\tradio", collisionInfo.m_sphere.m_radio);
				break;
			}

			case SERIALIZE_ID_CAPSULE:
			{
				SaveName("\tshapeType", "capsule");
				SaveFloat("\tradio0", collisionInfo.m_capsule.m_radio0);
				SaveFloat("\tradio1", collisionInfo.m_capsule.m_radio1);
				SaveFloat("\theight", collisionInfo.m_capsule.m_height);
				break;
			}

			case SERIALIZE_ID_CHAMFERCYLINDER:
			{
				SaveName("\tshapeType", "chamferCylinder");
				SaveFloat("\tradio", collisionInfo.m_chamferCylinder.m_radio);
				SaveFloat("\theight", collisionInfo.m_chamferCylinder.m_height);
				break;
			}

			case SERIALIZE_ID_CONVEXHULL:
			{
				SaveName("\tshapeType", "convexHull");
				SaveInt("\tpoints", collisionInfo.m_convexHull.m_vertexCount);
				const int stride = collisionInfo.m_convexHull.m_vertexStrideInBytes / sizeof(dFloat);
				const dFloat* points = collisionInfo.m_convexHull.m_vertex;
				for (int i = 0; i < collisionInfo.m_convexHull.m_vertexCount; i++) {
					dVector p(points[i * stride + 0], points[i * stride + 1], points[i * stride + 2], 0.0f);
					SaveVector("\tv", p);
				}
				break;
			}

			default:
			{
				dAssert(0);
			}
		}

		SaveName("collisionEnd", "\n");
	}

	SaveInt("nodesCount", bodyList.GetCount());
	for (dList<NewtonBody*>::dListNode* ptr = bodyList.GetFirst(); ptr; ptr = ptr->GetNext()) {
		dMatrix bodyMatrix;
		dMatrix shapeMatrix;
		dVector scale;
		dVector com;
		dFloat mass;
		dFloat ixx;

		NewtonBody* const body = ptr->GetInfo();
		NewtonBodyGetMatrix(body, &bodyMatrix[0][0]);
		NewtonBodyGetMass(body, &mass, &ixx, &ixx, &ixx);
		NewtonBodyGetCentreOfMass(body, &com.m_x);
		NewtonCollision* const collision = NewtonBodyGetCollision(body);
		NewtonCollisionGetMatrix(collision, &shapeMatrix[0][0]);
		NewtonCollisionGetScale(collision, &scale[0], &scale[1], &scale[2]);

		dEntry& entry = collisionList.Find(NewtonCollisionDataPointer(collision))->GetInfo();

		SaveInt("node", m_bodyFilter.Find(body)->GetInfo());
		SaveName("\tuserData", GetUserDataName(body));

		SaveFloat("\tmass", mass);
		SaveVector("\tcenterOfMass", com);
		SaveMatrix("\tmatrix", bodyMatrix);

		SaveInt("\tcollision", entry.m_id);

		SaveVector("\tshapeScale", scale);
		SaveMatrix("\tshapeMatrix", shapeMatrix);

		SaveName("nodeEnd", "\n");
	}
}


void dCustomJointSaveLoad::LoadBodyList(dTree<NewtonBody*, int>& bodyList)
{
	dTree<NewtonCollision*, int> collisionMap;

	const char* token = NextToken();
	dAssert(!strcmp(token, "shapeCount:"));
	int collisionCount = LoadInt();
	for (int i = 0; i < collisionCount; i++) {
		char shapeTypeName[128];
		token = NextToken();
		dAssert(!strcmp(token, "collision:"));
		int collisionIndex = LoadInt();

		token = NextToken();
		dAssert(!strcmp(token, "shapeType:"));
		LoadName(shapeTypeName);

		NewtonCollision* collision = NULL;
		if (!strcmp(shapeTypeName, "sphere")) {
			token = NextToken();
			dAssert(!strcmp(token, "radio:"));
			dFloat radio = LoadFloat();
			collision = NewtonCreateSphere(m_world, radio, 0, NULL);
		} else if (!strcmp(shapeTypeName, "capsule")) {
			token = NextToken();
			dAssert(!strcmp(token, "radio0:"));
			dFloat radio0 = LoadFloat();

			token = NextToken();
			dAssert(!strcmp(token, "radio1:"));
			dFloat radio1 = LoadFloat();

			token = NextToken();
			dAssert(!strcmp(token, "height:"));
			dFloat height = LoadFloat();
			collision = NewtonCreateCapsule(m_world, radio0, radio1, height, 0, NULL);

		} else if (!strcmp(shapeTypeName, "chamferCylinder")) {
			token = NextToken();
			dAssert(!strcmp(token, "radio:"));
			dFloat radio = LoadFloat();

			token = NextToken();
			dAssert(!strcmp(token, "height:"));
			dFloat height = LoadFloat();
			collision = NewtonCreateChamferCylinder(m_world, radio, height, 0, NULL);

		} else if (!strcmp(shapeTypeName, "convexHull")) {
			dVector array[1024 * 4];

			token = NextToken();
			dAssert(!strcmp(token, "points:"));

			int pointCount = LoadInt();
			for (int j = 0; j < pointCount; j++) {
				token = NextToken();
				array[j] = LoadVector();
			}
			collision = NewtonCreateConvexHull(m_world, pointCount, &array[0][0], sizeof(dVector), 1.0e-3f, 0, NULL);
		} else {
			dAssert(0);
		}

		token = NextToken();
		dAssert(!strcmp(token, "collisionEnd:"));

		collisionMap.Insert(collision, collisionIndex);
	}

	token = NextToken();
	dAssert(!strcmp(token, "nodesCount:"));
	int nodeCount = LoadInt();
	m_bodyFilter.RemoveAll();
	for (int i = 0; i < nodeCount; i ++) {

		dMatrix matrix(dGetIdentityMatrix());
		dMatrix shapeMatrix(dGetIdentityMatrix());
		dVector com(0.0f);
		dVector shapeScale(1.0f);
		dVector shapePosit(1.0f);
		
		NewtonBody* body = NULL;
		NewtonCollision* collision = NULL;
		dFloat mass = 0.0f;
		int bodyIndex = -1;
		char userDataName[128];

		for (const char* token1 = NextToken(); strcmp(token1, "nodeEnd:"); token1 = NextToken()) {

			if (!strcmp(token1, "node:")) {
				bodyIndex = LoadInt();
			} else if (!strcmp(token1, "userData:")) {
				LoadName(userDataName);
			} else if (!strcmp(token1, "mass:")) {
				mass = LoadFloat();
			} else if (!strcmp(token1, "centerOfMass:")) {
				com = LoadVector();
			} else if (!strcmp(token1, "matrix:")) {
				matrix = LoadMatrix();
			} else if (!strcmp(token1, "collision:")) {
				int coolIndex = LoadInt();
				collision = collisionMap.Find(coolIndex)->GetInfo();
			} else if (!strcmp(token1, "shapeScale:")) {
				shapeScale = LoadVector();
			} else if (!strcmp(token1, "shapeMatrix:")) {
				shapeMatrix = LoadMatrix();
			} else {
				dAssert(0);
			}
		}

		NewtonCollisionSetMatrix(collision, &shapeMatrix[0][0]);
		NewtonCollisionSetScale(collision, shapeScale.m_x, shapeScale.m_y, shapeScale.m_z);
//NewtonCollisionSetMode(collision, 0);

		body = NewtonCreateDynamicBody(m_world, collision, &matrix[0][0]);
		NewtonBodySetMassProperties(body, mass, collision);
		NewtonBodySetCentreOfMass(body, &com.m_x);
		InitRigiBody(body, userDataName);
		bodyList.Insert(body, bodyIndex);
		m_bodyFilter.Insert(bodyIndex, body);
	}


	dTree<NewtonCollision*, int>::Iterator iter(collisionMap);
	for (iter.Begin(); iter; iter++) {
		NewtonCollision* const collision = *iter;
		NewtonDestroyCollision(collision);
	}
}

void dCustomJointSaveLoad::SaveJointList(dList<dCustomJoint*>& jointList)
{
	int index = 0;
	m_jointFilter.RemoveAll();
	SaveInt("jointsCount", jointList.GetCount());
	for (dList<dCustomJoint*>::dListNode* ptr = jointList.GetFirst(); ptr; ptr = ptr->GetNext()) {
		dCustomJoint* const joint = ptr->GetInfo();

		SaveInt("joint", index);
		m_jointFilter.Insert(joint, index);
		SaveName("\tjointType", joint->GetTypeName());
		SaveInt("\tchildBody", FindBodyId(joint->GetBody0()));
		SaveInt("\tparentBody", FindBodyId(joint->GetBody1()));
		joint->Save(this);
		SaveName("jointEnd", "\n");
		index ++;
	}
}

void dCustomJointSaveLoad::LoadJointList(const dTree<NewtonBody*, int>& bodyList)
{
	const char* token = NextToken();
	dAssert(!strcmp(token, "jointsCount:"));
	int jointsCount = LoadInt();
	m_jointFilter.RemoveAll();
	for (int i = 0; i < jointsCount; i++) {
		char jointType[128];

		token = NextToken();
		dAssert(!strcmp(token, "joint:"));
		int jointIndex = LoadInt();

		token = NextToken();
		dAssert(!strcmp(token, "jointType:"));
		LoadName(jointType);

		token = NextToken();
		dAssert(!strcmp(token, "childBody:"));
		int childIndex = LoadInt();
		NewtonBody* const child = bodyList.Find(childIndex)->GetInfo();

		token = NextToken();
		dAssert(!strcmp(token, "parentBody:"));
		int parentIndex = LoadInt();
		NewtonBody* const parent = (parentIndex != -1) ? bodyList.Find(parentIndex)->GetInfo() : NULL;
		dCustomJoint* const joint = dCustomJoint::Load(this, jointType, child, parent);
		//jointMap.Insert (joint, jointIndex);
		m_jointFilter.Insert(joint, jointIndex);
		while (strcmp(NextToken(), "jointEnd:"));
	}
}


void dCustomJointSaveLoad::Save(NewtonBody* const rootbody)
{
	dList<NewtonBody*> bodyList;
	dList<dCustomJoint*> jointList;
	GetBodiesAndJointsList(bodyList, jointList, rootbody);

	SaveInt("rootNode", m_bodyFilter.Find(rootbody)->GetInfo());
	Newline();

	SaveBodyList(bodyList);
	SaveJointList(jointList);
}


NewtonBody* dCustomJointSaveLoad::Load()
{
	dTree<NewtonBody*, int> bodyMap;

#ifdef _DEBUG
	const char* token = NextToken();
	dAssert(!strcmp(token, "rootNode:"));
#else
	NextToken();
#endif
	int rootBodyIndex = LoadInt();

	LoadBodyList(bodyMap);
	LoadJointList(bodyMap);

	return bodyMap.Find(rootBodyIndex)->GetInfo();
}
