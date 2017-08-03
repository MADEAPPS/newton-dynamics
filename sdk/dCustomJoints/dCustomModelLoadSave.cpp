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

void dCustomJointSaveLoad::GetBodiesAndJointsList (dList<const NewtonBody*>& bodylist, dList<const dCustomJoint*>& jointlist, NewtonBody* const rootbody)
{
	dTree<int, const dCustomJoint*> jointFilter;
	const NewtonBody* stackMem[256];
	m_bodyFilter.RemoveAll();

	int stack = 1;
	stackMem[0] = rootbody;
	int enumeration = 0;
	while (stack) {
		stack--;
		const NewtonBody* const root = stackMem[stack];

		if (m_bodyFilter.Insert(enumeration, root)) {
			enumeration ++;
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
				if (!m_bodyFilter.Find(body1)) {
					stackMem[stack] = body1;
					stack ++;
				}
			} else {
				dAssert (body1 == root);
				if (!m_bodyFilter.Find(body0)) {
					stackMem[stack] = body0;
					stack++;
				}
			}
		}
	}
}


const char* dCustomJointSaveLoad::NextToken() const
{
	fscanf(m_file, "%s:", m_token);
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

void dCustomJointSaveLoad::SaveName (const char* const token, const char* const name) const
{
	fprintf(m_file, "%s: %s\n", token, name);
}

void dCustomJointSaveLoad::Newline () const
{
	fprintf(m_file, "\n");
}




NewtonCollision* dCustomJointSaveLoad::ParseCollisonShape()
{
	NewtonCollision* collision = NULL;

	const char* token = NextToken();

	if (!strcmp(token, "sphere")) {
		dAssert(0);
/*
		dFloat radio;
		fscanf(file, "%s %f", token, &radio);
		collision = NewtonCreateSphere(m_world, radio, 0, NULL);
*/
	} else if (!strcmp(token, "capsule")) {
		token = NextToken();
		dAssert (!strcmp(token, "radio0:"));
		dFloat radio0 = LoadFloat();

		token = NextToken();
		dAssert(!strcmp(token, "radio1:"));
		dFloat radio1 = LoadFloat();

		token = NextToken();
		dAssert(!strcmp(token, "height:"));
		dFloat height = LoadFloat();
		collision = NewtonCreateCapsule(m_world, radio0, radio1, height, 0, NULL);

	} else if (!strcmp(token, "convexHull")) {
		dVector array[1024* 4];
		
		token = NextToken();
		dAssert(!strcmp(token, "points:"));

		int pointCount = LoadInt();
		for (int i = 0; i < pointCount; i++) {
			token = NextToken();
			array[i] = LoadVector();
		}
		collision = NewtonCreateConvexHull(m_world, pointCount, &array[0][0], sizeof(dVector), 1.0e-3f, 0, NULL);
	} else {
		dAssert(0);
	}


	token = NextToken();
	dAssert(!strcmp(token, "shapeScale:"));
	dVector scale (LoadVector());

	token = NextToken();
	dAssert(!strcmp(token, "shapePosition:"));
	dVector posit(LoadVector());
	posit.m_w = 1.0f;

	token = NextToken();
	dAssert(!strcmp(token, "shapeEulerAngle:"));
	dVector euler(LoadVector());
	euler = euler.Scale(3.141592f / 180.0f);

	dMatrix matrix(euler.m_x, euler.m_y, euler.m_z, posit);
	NewtonCollisionSetMatrix(collision, &matrix[0][0]);
//NewtonCollisionSetMode(collision, 0);
	return collision;
}


void dCustomJointSaveLoad::ParseRigidBody(dTree<NewtonBody*, const dString>& bodyMap)
{
	dVector posit(0.0f);
	dVector euler(0.0f);
	NewtonBody* body = NULL;
	NewtonCollision* collision = NULL;
	dFloat mass = 0.0f;
	char nodeName[256];

	for (const char* token = NextToken (); strcmp(token, "nodeEnd:"); token = NextToken ()) { 

		if (!strcmp(token, "node:")) {
			LoadName (nodeName);
		} else if (!strcmp(token, "mass:")) {
			mass = LoadFloat ();
		} else if (!strcmp(token, "position:")) {
			posit = LoadVector();
			posit.m_w = 1.0f;
		} else if (!strcmp(token, "eulerAngles:")) {
			euler = LoadVector();
			euler = euler.Scale(3.141592f / 180.0f);
		} else if (!strcmp(token, "shapeType:")) {
			collision = ParseCollisonShape();
		} else {
			dAssert(0);
		}
	}

	dMatrix matrix(euler.m_x, euler.m_y, euler.m_z, posit);
	body = NewtonCreateDynamicBody(m_world, collision, &matrix[0][0]);
	NewtonBodySetMassProperties(body, mass, collision);

	InitRigiBody(body, nodeName);
	NewtonDestroyCollision(collision);
	bodyMap.Insert(body, nodeName);
}



void dCustomJointSaveLoad::ParseJoint(const dTree<NewtonBody*, const dString>& bodyMap)
{
	char jointType[256];
	char childName[256];
	char parentName[256];

	const char* token = NextToken();
	dAssert(!strcmp(token, "joint:"));
	LoadName (jointType);
	
	token = NextToken();
	dAssert(!strcmp(token, "childBody:"));
	LoadName (childName);
	NewtonBody* const child = bodyMap.Find(childName)->GetInfo();

	token = NextToken();
	dAssert(!strcmp(token, "parentBody:"));
	LoadName(parentName);
	NewtonBody* const parent = bodyMap.Find(parentName)->GetInfo();

	dCustomJoint* const joint = dCustomJoint::Load(this, jointType, child, parent);
	while (strcmp(NextToken(), "jointEnd:"));
}


NewtonBody* dCustomJointSaveLoad::Load()
{
	char rootBodyName[256];
//	m_bodyMap.RemoveAll();
	dTree<NewtonBody*, const dString> bodyMap;

	const char* token = NextToken();
	dAssert (!strcmp(token, "rootBone:"));
	LoadName (rootBodyName);

	token = NextToken();
	dAssert (!strcmp(token, "nodesCount:"));
	int nodesCount = LoadInt ();
	for (int i = 0; i < nodesCount; i++) {
		ParseRigidBody(bodyMap);
	}

	token = NextToken();
	dAssert (!strcmp(token, "jointsCount:"));
	int jointsCount = LoadInt ();
	for (int i = 0; i < jointsCount; i++) {
		ParseJoint(bodyMap);
	}

	return bodyMap.Find(rootBodyName)->GetInfo();
}


void dCustomJointSaveLoad::Save(NewtonBody* const rootbody)
{
	dList<const NewtonBody*> bodyList;
	dList<const dCustomJoint*> jointList;
	GetBodiesAndJointsList(bodyList, jointList, rootbody);

	SaveInt("rootNode", m_bodyFilter.Find(rootbody)->GetInfo());
	Newline();

	SaveInt("nodesCount", bodyList.GetCount());
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

		SaveInt("node", m_bodyFilter.Find(body)->GetInfo());
		SaveName("\tuserData", GetUserDataName(body));
		SaveFloat("\tmass", mass);
		SaveVector("\tposition", boneMatrix.m_posit);
		SaveVector("\teulerAngles", euler0.Scale(180.0f / 3.141592f));

		switch (collisionInfo.m_collisionType) 
		{
			case SERIALIZE_ID_SPHERE:
			{
				SaveName("\tshapeType", "sphere");
				SaveFloat("\t\tradio", collisionInfo.m_sphere.m_radio);
				break;
			}

			case SERIALIZE_ID_CAPSULE:
			{
				SaveName("\tshapeType", "capsule");
				SaveFloat("\t\tradio0", collisionInfo.m_capsule.m_radio0);
				SaveFloat("\t\tradio1", collisionInfo.m_capsule.m_radio1);
				SaveFloat("\t\theight", collisionInfo.m_capsule.m_height);
				break;
			}

			case SERIALIZE_ID_CHAMFERCYLINDER:
			{
				SaveName("\tshapeType", "chamferCylinder");
				SaveFloat("\t\tradio", collisionInfo.m_chamferCylinder.m_radio);
				SaveFloat("\t\theight", collisionInfo.m_chamferCylinder.m_height);
				break;
			}

			case SERIALIZE_ID_CONVEXHULL:
			{
				SaveName("\tshapeType", "convexHull");
				SaveInt("\t\tpoints", collisionInfo.m_convexHull.m_vertexCount);
				const int stride = collisionInfo.m_convexHull.m_vertexStrideInBytes / sizeof(dFloat);
				const dFloat* points = collisionInfo.m_convexHull.m_vertex;
				for (int i = 0; i < collisionInfo.m_convexHull.m_vertexCount; i++) {
					dVector p(points[i * stride + 0], points[i * stride + 1], points[i * stride + 2], 0.0f);
					SaveVector("\t\tv", p);
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

		SaveVector("\tshapeScale", dVector(1.0f, 1.0f, 1.0f, 1.0f));
		SaveVector("\tshapePosition", shapeMatrix.m_posit);
		SaveVector("\tshapeEulerAngle", euler0.Scale(180.0f / 3.141592f));
		SaveName("nodeEnd", "\n");
	}

	SaveInt("jointsCount", jointList.GetCount());
	for (dList<const dCustomJoint*>::dListNode* ptr = jointList.GetFirst(); ptr; ptr = ptr->GetNext()) {
		const dCustomJoint* const joint = ptr->GetInfo();

		SaveName("joint", joint->GetTypeName());
		SaveInt("\tchildBody", m_bodyFilter.Find(joint->GetBody0())->GetInfo());
		SaveInt("\tparentBody", m_bodyFilter.Find(joint->GetBody1())->GetInfo());
		joint->Save(this);
		SaveName("jointEnd", "\n");
	}
}
