/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "toolbox_stdafx.h"
#include "SkyBox.h"
#include "DemoMesh.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "TargaToOpenGl.h"
#include "DemoEntityManager.h"

#include "DebugDisplay.h"
#include "HeightFieldPrimitive.h"

#define ARTICULATED_VEHICLE_CAMERA_EYEPOINT			1.5f
#define ARTICULATED_VEHICLE_CAMERA_HIGH_ABOVE_HEAD	2.0f
#define ARTICULATED_VEHICLE_CAMERA_DISTANCE			13.0f


#define EXCAVATOR_TREAD_THICKNESS	0.13f

struct ARTICULATED_VEHICLE_DEFINITION
{
	enum SHAPES_ID
	{
		m_terrain			= 1<<0,
		m_bodyPart			= 1<<1,
		m_tirePart			= 1<<2,
		m_linkPart 			= 1<<3,
		m_propBody			= 1<<4,
	};

	char m_boneName[32];
	char m_shapeTypeName[32];
	dFloat m_mass;
	char m_articulationName[32];
};

class dExcavatorModel: public dModelRootNode
{
	public:
	class dThreadContacts
	{
		public:
		dVector m_point[2];
		dVector m_normal[2];
		NewtonBody* m_tire;
		const NewtonBody* m_link[2];
		dFloat m_penetration[2];
		int m_count;
	};

	dExcavatorModel(NewtonBody* const rootBody, int linkMaterilID)
		:dModelRootNode(rootBody, dGetIdentityMatrix())
		,m_bodyMap()
		,m_shereCast (NewtonCreateSphere (NewtonBodyGetWorld(rootBody), 0.05f, 0, NULL))
		,m_tireCount(0)
	{
		m_bodyMap.Insert(rootBody);
		memset (m_tireArray, 0, sizeof (m_tireArray));

		MakeLeftTrack();
		MakeRightTrack();
		MakeThread("leftThread", linkMaterilID);
		MakeThread("rightThread", linkMaterilID);
		MakeCabinAndUpperBody ();
	}

	~dExcavatorModel()
	{
		NewtonDestroyCollision(m_shereCast);
	}

/*
	void CalculateTireContacts()
	{
		for (int i = 0; i < m_tireCount; i ++) {
			CalculateContact(&m_tireArray[i]);
		}
	}

	static void RenderDebugTire(void* userData, int vertexCount, const dFloat* const faceVertec, int id)
	{
		dCustomJoint::dDebugDisplay* const debugContext = (dCustomJoint::dDebugDisplay*) userData;

		int index = vertexCount - 1;
		dVector p0(faceVertec[index * 3 + 0], faceVertec[index * 3 + 1], faceVertec[index * 3 + 2]);
		for (int i = 0; i < vertexCount; i++) {
			dVector p1(faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2]);
			debugContext->DrawLine(p0, p1);
			p0 = p1;
		}
	}

	virtual void OnDebug(dCustomJoint::dDebugDisplay* const debugContext) 
	{
		debugContext->SetColor(dVector(0.7f, 0.4f, 0.0f, 1.0f));

		for (int i = 0; i < m_tireCount; i ++) {
			dMatrix matrix;
			dVector scale;
			NewtonCollision* const collision = NewtonBodyGetCollision(m_tireArray[i].m_tire);
			NewtonBodyGetMatrix(m_tireArray[i].m_tire, &matrix[0][0]);
			NewtonCollisionGetScale(collision, &scale.m_x, &scale.m_y, &scale.m_z);

			NewtonCollisionInfoRecord collisionInfo;
			NewtonCollisionGetInfo(collision, &collisionInfo);

			const dVector threadThickness(EXCAVATOR_TREAD_THICKNESS, EXCAVATOR_TREAD_THICKNESS, EXCAVATOR_TREAD_THICKNESS, 0.0f);
			dFloat r0 = collisionInfo.m_chamferCylinder.m_radio + collisionInfo.m_chamferCylinder.m_height * 0.5f;
			dFloat r1 = r0 + threadThickness.m_x;
			dFloat newScale = r1 / r0;
			NewtonCollisionSetScale(collision, scale.m_x, newScale, newScale);
			NewtonCollisionForEachPolygonDo(collision, &matrix[0][0], RenderDebugTire, debugContext);
			NewtonCollisionSetScale(collision, scale.m_x, scale.m_y, scale.m_z);
		}

		debugContext->SetColor(dVector(1.0f, 0.0f, 0.0f, 1.0f));
		for (int i = 0; i < m_tireCount; i ++) {
			const dThreadContacts* const entry = &m_tireArray[i];
			for (int j = 0; j < entry->m_count; j++) {
				debugContext->DrawPoint(entry->m_point[j], 10.0f);
			}
		}
	}

	const dThreadContacts* FindTireEntry(const NewtonBody* const linkBody) const 
	{
		for (int i = 0; i < m_tireCount; i ++) {
			const dThreadContacts* const entry = &m_tireArray[i];
			for (int j = 0; j < entry->m_count; j ++) {
				if (entry->m_link[j] == linkBody) {
					return entry;
				}
			}
		}
		return NULL;
	}

	int OnBoneAABBOverlap(const NewtonJoint* const contactJoint) const
	{
		const NewtonBody* const linkBody = NewtonJointGetBody0(contactJoint);
		return FindTireEntry(linkBody) ? 1 : 0;
	}

	int SpecialGenerateLinkGroundContact (const NewtonMaterial* const material, const NewtonBody* const linkBody0, const NewtonBody* const staticBody, NewtonUserContactPoint* const contactBuffer)
	{
		const dThreadContacts* const linkBody = FindTireEntry(linkBody0);
		int ret = 0;
		if (linkBody && linkBody->m_count) {
			contactBuffer->m_point[0] = linkBody->m_point[0].m_x;
			contactBuffer->m_point[1] = linkBody->m_point[0].m_y;
			contactBuffer->m_point[2] = linkBody->m_point[0].m_z;
			contactBuffer->m_point[3] = 1.0f;
			contactBuffer->m_normal[0] = linkBody->m_normal[0].m_x;
			contactBuffer->m_normal[1] = linkBody->m_normal[0].m_y;
			contactBuffer->m_normal[2] = linkBody->m_normal[0].m_z;
			contactBuffer->m_normal[3] = 0.0f;
			contactBuffer->m_penetration = linkBody->m_penetration[0];
			contactBuffer->m_shapeId0 = 0;
			contactBuffer->m_shapeId1 = 0;
			ret = 1;
		}
		return ret;
	}
*/
	private:
/*
	class dCollidingBodies
	{
		public:
		dCollidingBodies (dExcavatorModel* const me)
			:m_excavator(me)
			,m_staticCount(0)
			,m_threaLinkCount(0)
		{
		}
		
		const NewtonBody* m_threaLinks[24];
		const NewtonBody* m_staticBodies[8];
		dExcavatorModel* m_excavator;
		int m_staticCount;
		int m_threaLinkCount;
	};

	static int CollectBodiesOfInterest (const NewtonBody* const body, void* const userData)
	{
		dFloat mass;
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;
		dCollidingBodies* const set = (dCollidingBodies*)userData;
		if (!set->m_excavator->m_bodyMap.Find(body)) {
			NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);
			if (mass > 0.0f) {
				const NewtonCollision* const collision = NewtonBodyGetCollision(body);
				dLong id = NewtonCollisionGetUserID(collision);
				if (id == ARTICULATED_VEHICLE_DEFINITION::m_linkPart) {
					set->m_threaLinks[set->m_threaLinkCount] = body;
					set->m_threaLinkCount ++;
					dAssert (set->m_threaLinkCount < sizeof (set->m_threaLinks) / sizeof (set->m_threaLinks[0]));
				}
			} else {
				set->m_staticBodies[set->m_staticCount] = body;
				set->m_staticCount ++;
				dAssert (set->m_staticCount < sizeof (set->m_staticBodies) / sizeof (set->m_staticBodies[0]));
			}
		}
		return 1;
	}

	void CalculateContact(dThreadContacts* const tire)
	{
		dMatrix matrix;
		dVector scale;
		dVector p0;
		dVector p1;

		// get the static bodies and thread link close to this scaled tire. 
		NewtonWorld* const world = NewtonBodyGetWorld(tire->m_tire);
		NewtonCollision* const collision = NewtonBodyGetCollision(tire->m_tire);

		NewtonBodyGetMatrix(tire->m_tire, &matrix[0][0]);
		NewtonCollisionGetScale(collision, &scale.m_x, &scale.m_y, &scale.m_z);

		NewtonCollisionInfoRecord collisionInfo;
		NewtonCollisionGetInfo (collision, &collisionInfo);
		dAssert (collisionInfo.m_collisionType == SERIALIZE_ID_CHAMFERCYLINDER);

		const dVector threadThickness (EXCAVATOR_TREAD_THICKNESS, EXCAVATOR_TREAD_THICKNESS, EXCAVATOR_TREAD_THICKNESS, 0.0f);
		dFloat r0 = collisionInfo.m_chamferCylinder.m_radio + collisionInfo.m_chamferCylinder.m_height * 0.5f;
		dFloat r1 = r0 + threadThickness.m_x;
		dFloat newScale = r1 / r0;
		NewtonCollisionSetScale(collision, scale.m_x, newScale, newScale);
		
		NewtonCollisionCalculateAABB (collision, &matrix[0][0], &p0[0], &p1[0]);
		p0 -= threadThickness;
		p1 += threadThickness;

		dCollidingBodies collidingBodies (this);
		NewtonWorldForEachBodyInAABBDo(world, &p0.m_x, &p1.m_x, CollectBodiesOfInterest, &collidingBodies);

		// for each static body calculate the contact with a thread link
		tire->m_count = 0;
		dMatrix vehMatrix;
		NewtonBodyGetMatrix(GetBody(), &vehMatrix[0][0]);
		for (int i = 0; i < collidingBodies.m_staticCount; i ++) {
			CalculateContact(tire, vehMatrix.m_up, matrix, collision, collidingBodies, collidingBodies.m_staticBodies[i]);
		}

		NewtonCollisionSetScale(collision, scale.m_x, scale.m_y, scale.m_z);
	}

	bool CalculateContact(
		const NewtonBody* const staticBody, const dMatrix& tireMatrix, NewtonCollision* const tireShape, const dVector& updir,
		dVector& targetPoint, dVector& targetNormal, dFloat& targetPenetration)
	{
		dMatrix staticMatrix;
		NewtonBodyGetMatrix(staticBody, &staticMatrix[0][0]);
		NewtonCollision* const staticShape = NewtonBodyGetCollision(staticBody);
		NewtonWorld* const world = NewtonBodyGetWorld(staticBody);
		dVector zero(0.0f);
		dVector veloc(updir.Scale(-0.7f));
		dFloat timeOfImpact;

		const int maxContactCount = 2;
		dFloat points[maxContactCount][3];
		dFloat normals[maxContactCount][3];
		dFloat penetrations[maxContactCount];
		dLong attributeA[maxContactCount];
		dLong attributeB[maxContactCount];

		int count = 0;
		if (0) {
			count = NewtonCollisionCollide(world, 1,
						tireShape, &tireMatrix[0][0],
						staticShape, &staticMatrix[0][0],
						&points[0][0], &normals[0][0], penetrations, attributeA, attributeB, 0);
		} else {
			count = NewtonCollisionCollideContinue(world, 1, 1.0f,
						m_shereCast, &tireMatrix[0][0], &veloc[0], &zero[0],
						staticShape, &staticMatrix[0][0], &zero[0], &zero[0],
						&timeOfImpact, &points[0][0], &normals[0][0], &penetrations[0],
						&attributeA[0], &attributeB[0], 0);
		}

		if (count) {
			targetPoint = dVector (points[0][0], points[0][1], points[0][2], dFloat(1.0f));
			targetNormal = dVector (normals[0][0], normals[0][1], normals[0][2], dFloat(0.0f));
			targetPenetration = penetrations[0];
		}

		return count ? true : false;
	}

	void CalculateContact(dThreadContacts* const tire, const dVector& updir, const dMatrix& tireMatrix, NewtonCollision* const tireShape, const dCollidingBodies& collidingBodies, const NewtonBody* const staticBody)
	{
		dVector targetPoint;
		dVector targetNormal;
		dFloat targetPenetration;
		bool hasContact = CalculateContact(staticBody, tireMatrix, tireShape, updir, targetPoint, targetNormal, targetPenetration);

		if (hasContact) {
			dMatrix scanMatrix (dGetIdentityMatrix());
			dVector zero (0.0f);

			const int maxContactCount = 2;
			dFloat points[maxContactCount][3];
			dFloat normals[maxContactCount][3];
			dFloat penetrations[maxContactCount];
			dLong attributeA[maxContactCount];
			dLong attributeB[maxContactCount];
			
			NewtonWorld* const world = NewtonBodyGetWorld(staticBody);
			dVector dir(tireMatrix.m_posit - targetPoint);
			dir = dir.Normalize();
			dVector veloc(dir.Scale(1.0f));
			scanMatrix.m_posit = targetPoint - dir.Scale (0.5f);

			for (int i = 0; i < collidingBodies.m_threaLinkCount && (tire->m_count < 2); i ++) {
				dMatrix linkMatrix;
				dFloat timeOfImpact;
				NewtonCollision* const linkCollision = NewtonBodyGetCollision(collidingBodies.m_threaLinks[i]);
				NewtonBodyGetMatrix (collidingBodies.m_threaLinks[i], &linkMatrix[0][0]);
			
				int collide = NewtonCollisionCollideContinue(
					world, 1, 1.0f,
					m_shereCast, &scanMatrix[0][0], &veloc[0], &zero[0],
					linkCollision, &linkMatrix[0][0], &zero[0], &zero[0],
					&timeOfImpact, &points[0][0], &normals[0][0], &penetrations[0],
					&attributeA[0], &attributeB[0], 0);
				if (collide) {
					dVector p (points[0][0], points[0][1], points[0][2], dFloat (1.0f));
					dFloat dist = dir.DotProduct3(p - targetPoint);
					if (dist <= 0.0f) {
						int index = tire->m_count;
						tire->m_point[index] = targetPoint;
						tire->m_normal[index] = targetNormal;
						tire->m_penetration[index] = -dist;
						tire->m_link[index] = collidingBodies.m_threaLinks[i];
						tire->m_count = index + 1;
					}
				}
			}
		}
	}
*/

	void GetTireDimensions(DemoEntity* const bodyPart, dFloat& radius, dFloat& width)
	{
		radius = 0.0f;
		dFloat maxWidth = 0.0f;
		dFloat minWidth = 0.0f;

		DemoMesh* const mesh = (DemoMesh*)bodyPart->GetMesh();
		dAssert(mesh->IsType(DemoMesh::GetRttiType()));
		const dMatrix& matrix = bodyPart->GetMeshMatrix();
		dFloat* const array = mesh->m_vertex;
		for (int i = 0; i < mesh->m_vertexCount; i++) {
			dVector p(matrix.TransformVector(dVector(array[i * 3 + 0], array[i * 3 + 1], array[i * 3 + 2], 1.0f)));
			maxWidth = dMax(p.m_z, maxWidth);
			minWidth = dMin(p.m_z, minWidth);
			radius = dMax(p.m_x, radius);
		}
		width = maxWidth - minWidth;
		radius -= width * 0.5f;
	}

	NewtonCollision* const MakeDoubleRingTireShape(DemoEntity* const tireModel)
	{
		dFloat width;
		dFloat radius;
		GetTireDimensions(tireModel, radius, width);
		dMatrix align(dYawMatrix(90.0f * dDegreeToRad));
		NewtonWorld* const world = NewtonBodyGetWorld(GetBody());
		NewtonCollision* const tireShape = NewtonCreateChamferCylinder(world, radius, width, ARTICULATED_VEHICLE_DEFINITION::m_tirePart, &align[0][0]);
		return tireShape;
	}

	dModelNode* MakeTireBody(const char* const entName, NewtonCollision* const tireCollision)
	{
		NewtonBody* const parentBody = GetBody();
		NewtonWorld* const world = NewtonBodyGetWorld(GetBody());
		DemoEntity* const parentModel = (DemoEntity*)NewtonBodyGetUserData(parentBody);
		DemoEntity* const tireModel = parentModel->Find(entName);

		// calculate the bone matrix
		dMatrix matrix(tireModel->CalculateGlobalMatrix());

		// create the rigid body that will make this bone
		NewtonBody* const tireBody = NewtonCreateDynamicBody(world, tireCollision, &matrix[0][0]);

		// assign the material ID
		NewtonBodySetMaterialGroupID(tireBody, NewtonMaterialGetDefaultGroupID(world));

		// get the collision from body
		NewtonCollision* const collision = NewtonBodyGetCollision(tireBody);

		// save the root node as the use data
		NewtonCollisionSetUserData(collision, this);

		// set the material properties for each link
		NewtonCollisionMaterial material;
		NewtonCollisionGetMaterial(collision, &material);
		material.m_userId = ARTICULATED_VEHICLE_DEFINITION::m_tirePart;
		material.m_userParam[0].m_int =
			ARTICULATED_VEHICLE_DEFINITION::m_terrain |
			ARTICULATED_VEHICLE_DEFINITION::m_bodyPart |
			ARTICULATED_VEHICLE_DEFINITION::m_linkPart |
			ARTICULATED_VEHICLE_DEFINITION::m_tirePart |
			ARTICULATED_VEHICLE_DEFINITION::m_propBody;
		NewtonCollisionSetMaterial(collision, &material);

		// calculate the moment of inertia and the relative center of mass of the solid
		NewtonBodySetMassProperties(tireBody, 30.0f, collision);

		// save the user data with the bone body (usually the visual geometry)
		NewtonBodySetUserData(tireBody, tireModel);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback(tireBody, PhysicsApplyGravityForce);

		dMatrix bindMatrix(tireModel->GetParent()->CalculateGlobalMatrix(parentModel).Inverse());
		dModelNode* const bone = new dModelNode(tireBody, bindMatrix, this);

		m_bodyMap.Insert(tireBody);

		return bone;
	}

	dModelNode* MakeRollerTire(const char* const entName)
	{
		NewtonBody* const parentBody = GetBody();
		DemoEntity* const parentModel = (DemoEntity*)NewtonBodyGetUserData(parentBody);
		DemoEntity* const tireModel = parentModel->Find(entName);

		NewtonCollision* const tireCollision = MakeDoubleRingTireShape(tireModel);
		dModelNode* const bone = MakeTireBody(entName, tireCollision);
		NewtonDestroyCollision(tireCollision);

		// connect the tire the body with a hinge
		dMatrix matrix;
		NewtonBodyGetMatrix(bone->GetBody(), &matrix[0][0]);
		dMatrix hingeFrame(dYawMatrix(90.0f * dDegreeToRad) * matrix);

		new dCustomHinge(hingeFrame, bone->GetBody(), parentBody);
		return bone;
	}

	dCustomJoint* LinkTires(dModelNode* const master, dModelNode* const slave)
	{
		NewtonCollisionInfoRecord slaveTire;
		NewtonCollisionInfoRecord masterTire;

		NewtonCollisionGetInfo(NewtonBodyGetCollision(slave->GetBody()), &slaveTire);
		NewtonCollisionGetInfo(NewtonBodyGetCollision(master->GetBody()), &masterTire);

		dAssert(masterTire.m_collisionType == SERIALIZE_ID_CHAMFERCYLINDER);
		dAssert(slaveTire.m_collisionType == SERIALIZE_ID_CHAMFERCYLINDER);
		dAssert(masterTire.m_collisionMaterial.m_userId == ARTICULATED_VEHICLE_DEFINITION::m_tirePart);
		dAssert(slaveTire.m_collisionMaterial.m_userId == ARTICULATED_VEHICLE_DEFINITION::m_tirePart);

		dFloat masterRadio = masterTire.m_chamferCylinder.m_height * 0.5f + masterTire.m_chamferCylinder.m_radio;
		dFloat slaveRadio = slaveTire.m_chamferCylinder.m_height * 0.5f + slaveTire.m_chamferCylinder.m_radio;

		dMatrix pinMatrix0;
		dMatrix pinMatrix1;
		const dCustomJoint* const joint = master->GetJoint();
		joint->CalculateGlobalMatrix(pinMatrix0, pinMatrix1);
		return new dCustomGear(slaveRadio / masterRadio, pinMatrix0[0], pinMatrix0[0].Scale(-1.0f), slave->GetBody(), master->GetBody());
	}

	void MakeLeftTrack()
	{
		dModelNode* const leftTire_0 = MakeRollerTire("leftGear");
		dModelNode* const leftTire_7 = MakeRollerTire("leftFrontRoller");
		m_tireArray[m_tireCount++].m_tire = leftTire_0->GetBody();
		m_tireArray[m_tireCount++].m_tire = leftTire_7->GetBody();
		LinkTires (leftTire_0, leftTire_7);
		MakeRollerTire("leftSupportRoller");

		for (int i = 0; i < 3; i++) {
			char name[64];
			sprintf(name, "leftRoller%d", i);
			dModelNode* const rollerTire = MakeRollerTire(name);
			m_tireArray[m_tireCount++].m_tire = rollerTire->GetBody();
			LinkTires (leftTire_0, rollerTire);
		}
	}

	void MakeRightTrack()
	{
		dModelNode* const rightTire_0 = MakeRollerTire("rightGear");
		dModelNode* const rightTire_7 = MakeRollerTire("rightFrontRoller");
		m_tireArray[m_tireCount++].m_tire = rightTire_0->GetBody();
		m_tireArray[m_tireCount++].m_tire = rightTire_7->GetBody();
		LinkTires(rightTire_0, rightTire_7);
		MakeRollerTire("rightSupportRoller");

		for (int i = 0; i < 3; i++) {
			char name[64];
			sprintf(name, "rightRoller%d", i);
			dModelNode* const rollerTire = MakeRollerTire(name);
			m_tireArray[m_tireCount++].m_tire = rollerTire->GetBody();
			LinkTires(rightTire_0, rollerTire);
		}
	}

	NewtonBody* MakeThreadLinkBody(DemoEntity* const linkNode, NewtonCollision* const linkCollision, int linkMaterilID)
	{
		NewtonWorld* const world = NewtonBodyGetWorld(GetBody());

		// calculate the bone matrix
		dMatrix matrix(linkNode->CalculateGlobalMatrix());

		// create the rigid body that will make this bone
		NewtonBody* const linkBody = NewtonCreateDynamicBody(world, linkCollision, &matrix[0][0]);

		// assign the material ID
		NewtonBodySetMaterialGroupID(linkBody, linkMaterilID);

		// get the collision from body
		NewtonCollision* const collision = NewtonBodyGetCollision(linkBody);

		// save the root node as the use data
		NewtonCollisionSetUserData(collision, this);

		// set the material properties for each link
		NewtonCollisionMaterial material;
		NewtonCollisionGetMaterial(collision, &material);
		material.m_userId = ARTICULATED_VEHICLE_DEFINITION::m_linkPart;
		material.m_userParam[0].m_int = 
									  ARTICULATED_VEHICLE_DEFINITION::m_terrain | 
									  //ARTICULATED_VEHICLE_DEFINITION::m_bodyPart |
									  //ARTICULATED_VEHICLE_DEFINITION::m_linkPart |
									  ARTICULATED_VEHICLE_DEFINITION::m_tirePart |
									  ARTICULATED_VEHICLE_DEFINITION::m_propBody;
		NewtonCollisionSetMaterial(collision, &material);

		// calculate the moment of inertia and the relative center of mass of the solid
		NewtonBodySetMassProperties(linkBody, 5.0f, collision);

		// save the user data with the bone body (usually the visual geometry)
		NewtonBodySetUserData(linkBody, linkNode);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback(linkBody, PhysicsApplyGravityForce);

		return linkBody;
	}

	void MakeThread(const char* const baseName, int linkMaterilID)
	{
		NewtonBody* const parentBody = GetBody();
		NewtonWorld* const world = NewtonBodyGetWorld(GetBody());
		DemoEntity* const parentModel = (DemoEntity*)NewtonBodyGetUserData(parentBody);

		DemoEntity* linkArray[256];
		DemoEntity* stackPool[256];
		char name[256];

		int stack = 1;
		int linksCount = 0;
		sprintf(name, "%s_00", baseName);
		stackPool[0] = parentModel->Find(name);

		while (stack) {
			stack--;
			DemoEntity* link = stackPool[stack];
			linkArray[linksCount] = link;
			linksCount++;

			for (DemoEntity* child = link->GetChild(); child; child = child->GetSibling()) {
				if (strstr(child->GetName().GetStr(), baseName)) {
					stackPool[stack] = child;
					stack++;
				}
			}
		}

		
		NewtonCollision* const linkCollision = linkArray[0]->CreateCollisionFromchildren(world);

		NewtonBody* const firstLinkBody = MakeThreadLinkBody(linkArray[0], linkCollision, linkMaterilID);

		//dMatrix bindMatrix(linkArray[count]->GetParent()->CalculateGlobalMatrix(parentModel).Inverse());
		dMatrix bindMatrix(dGetIdentityMatrix());
		dModelNode* const linkNode = new dModelNode(firstLinkBody, bindMatrix, this);

		dMatrix planeMatrix;
		NewtonBodyGetMatrix(firstLinkBody, &planeMatrix[0][0]);
		dVector planePivot(planeMatrix.m_posit);
		dVector planeNornal(planeMatrix.m_up);
		new dCustomPlane(planePivot, planeNornal, firstLinkBody, GetBody());

		dModelNode* linkNode0 = linkNode;
		for (int i = 1; i < linksCount; i++) {
			dMatrix hingeMatrix;
			NewtonBody* const linkBody = MakeThreadLinkBody(linkArray[i], linkCollision, linkMaterilID);
			NewtonBodyGetMatrix(linkBody, &hingeMatrix[0][0]);
			hingeMatrix = dRollMatrix(90.0f * dDegreeToRad) * hingeMatrix;
			dCustomHinge* const hinge = new dCustomHinge(hingeMatrix, linkBody, linkNode0->GetBody());
			hinge->SetAsSpringDamper(true, 0.9f, 0.0f, 5.0f);
			dModelNode* const linkNode1 = new dModelNode(linkBody, bindMatrix, linkNode0);
			linkNode0 = linkNode1;
		}

		dMatrix hingeMatrix;
		NewtonBodyGetMatrix(firstLinkBody, &hingeMatrix[0][0]);
		hingeMatrix = dRollMatrix(90.0f * dDegreeToRad) * hingeMatrix;
		dCustomHinge* const hinge = new dCustomHinge(hingeMatrix, firstLinkBody, linkNode0->GetBody());
		hinge->SetAsSpringDamper(true, 0.9f, 0.0f, 5.0f);
		NewtonDestroyCollision(linkCollision);
	}

	void MakeCabinAndUpperBody ()
	{
		NewtonBody* const parentBody = GetBody();
		NewtonWorld* const world = NewtonBodyGetWorld(GetBody());
		DemoEntity* const parentModel = (DemoEntity*)NewtonBodyGetUserData(parentBody);
		DemoEntity* const bodyPart = parentModel->Find("EngineBody");
		
		NewtonCollision* const shape = bodyPart->CreateCollisionFromchildren(world);
		dAssert(shape);

		// calculate the bone matrix
		dMatrix matrix(bodyPart->CalculateGlobalMatrix());

		// create the rigid body that will make this bone
		NewtonBody* const body = NewtonCreateDynamicBody(world, shape, &matrix[0][0]);

		// assign the material ID
		NewtonBodySetMaterialGroupID(body, NewtonMaterialGetDefaultGroupID(world));

		// destroy the collision helper shape 
		NewtonDestroyCollision(shape);

		// get the collision from body
		NewtonCollision* const collision = NewtonBodyGetCollision(body);

		// save the root node as the use data
		NewtonCollisionSetUserData(collision, this);

		// set the material properties for each link
		NewtonCollisionMaterial material;
		NewtonCollisionGetMaterial(collision, &material);
		material.m_userId = ARTICULATED_VEHICLE_DEFINITION::m_bodyPart;
		material.m_userParam[0].m_int =
			ARTICULATED_VEHICLE_DEFINITION::m_terrain |
			ARTICULATED_VEHICLE_DEFINITION::m_bodyPart |
			ARTICULATED_VEHICLE_DEFINITION::m_linkPart |
			ARTICULATED_VEHICLE_DEFINITION::m_tirePart |
			ARTICULATED_VEHICLE_DEFINITION::m_propBody;
		NewtonCollisionSetMaterial(collision, &material);

		// calculate the moment of inertia and the relative center of mass of the solid
		NewtonBodySetMassProperties(body, 400.0f, collision);

		// save the user data with the bone body (usually the visual geometry)
		NewtonBodySetUserData(body, bodyPart);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);

		// connect the part to the main body with a hinge
		dMatrix hingeFrame;
		NewtonBodyGetMatrix(body, &hingeFrame[0][0]);
		new dCustomHinge(hingeFrame, body, parentBody);

		dMatrix bindMatrix(bodyPart->GetParent()->CalculateGlobalMatrix(parentModel).Inverse());
		new dModelNode(body, bindMatrix, this);

		m_bodyMap.Insert(body);
		//return bone;
	}

	dMap<NewtonBody*, const NewtonBody*> m_bodyMap;
	dThreadContacts m_tireArray[16];
	NewtonCollision* m_shereCast;
	int m_tireCount;
};

class ArticulatedVehicleManagerManager: public dModelManager
{
	public:
	ArticulatedVehicleManagerManager (DemoEntityManager* const scene, int threadMaterialID)
		:dModelManager (scene->GetNewton())
		,m_threadMaterialID(threadMaterialID)
	{

		// create a material for early collision culling
		NewtonWorld* const world = scene->GetNewton();
		int material = NewtonMaterialGetDefaultGroupID (world);

		NewtonMaterialSetCallbackUserData (world, material, material, this);
		NewtonMaterialSetCollisionCallback (world, material, material, StandardAABBOverlapTest, NULL);

		NewtonMaterialSetCallbackUserData(world, material, threadMaterialID, this);
		NewtonMaterialSetCollisionCallback (world, material, threadMaterialID, StandardAABBOverlapTest, NULL);

		NewtonMaterialSetCallbackUserData(world, threadMaterialID, threadMaterialID, this);
		NewtonMaterialSetCollisionCallback (world, threadMaterialID, threadMaterialID, StandardAABBOverlapTest, NULL);
		NewtonMaterialSetContactGenerationCallback (world, threadMaterialID, threadMaterialID, ThreadStaticContactsGeneration);
	}

	NewtonBody* CreateBodyPart(DemoEntity* const bodyPart, dFloat mass)
	{
		NewtonWorld* const world = GetWorld();
		NewtonCollision* const shape = bodyPart->CreateCollisionFromchildren(world);
		dAssert(shape);

		// calculate the bone matrix
		dMatrix matrix(bodyPart->CalculateGlobalMatrix());

		// create the rigid body that will make this bone
		NewtonBody* const body = NewtonCreateDynamicBody(world, shape, &matrix[0][0]);

		// assign the material ID
		NewtonBodySetMaterialGroupID(body, NewtonMaterialGetDefaultGroupID(world));

		// destroy the collision helper shape 
		NewtonDestroyCollision(shape);

		// get the collision from body
		NewtonCollision* const collision = NewtonBodyGetCollision(body);

		// save the root node as the use data
		NewtonCollisionSetUserData(collision, this);

		// set collision filter
		// set the material properties for each link
		NewtonCollisionMaterial material;
		NewtonCollisionGetMaterial(collision, &material);
		material.m_userId = ARTICULATED_VEHICLE_DEFINITION::m_bodyPart;
		material.m_userParam[0].m_int =
			ARTICULATED_VEHICLE_DEFINITION::m_terrain |
			ARTICULATED_VEHICLE_DEFINITION::m_bodyPart |
			ARTICULATED_VEHICLE_DEFINITION::m_linkPart |
			ARTICULATED_VEHICLE_DEFINITION::m_tirePart |
			ARTICULATED_VEHICLE_DEFINITION::m_propBody;
		NewtonCollisionSetMaterial(collision, &material);


		// calculate the moment of inertia and the relative center of mass of the solid
		NewtonBodySetMassProperties(body, mass, collision);

		// save the user data with the bone body (usually the visual geometry)
		NewtonBodySetUserData(body, bodyPart);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);
		return body;
	}

	dModelRootNode* CreateExcavator (const char* const modelName, const dMatrix& location)
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		// make a clone of the mesh 
		DemoEntity* const vehicleModel = DemoEntity::LoadNGD_mesh (modelName, world, scene->GetShaderCache());		
		scene->Append(vehicleModel);

		// place the model at its location
		dMatrix matrix (vehicleModel->GetCurrentMatrix());
		matrix.m_posit = location.m_posit;
		vehicleModel->ResetMatrix(*scene, matrix);

		DemoEntity* const rootEntity = (DemoEntity*)vehicleModel->Find("base");
		NewtonBody* const rootBody = CreateBodyPart(rootEntity, 4000.0f);
		dExcavatorModel* const controller = new dExcavatorModel(rootBody, m_threadMaterialID);

		// the the model to calculate the local transformation
		controller->SetTranformMode(true);

		// add the model to the manager
		AddRoot(controller);

		// add engine
		//dCustomTransformController::dSkeletonBone* const engineBone = CreateEngineNode(controller, chassisBone);
		//vehicleModel->m_engineJoint = (dCustomDoubleHinge*) engineBone->FindJoint();
		//vehicleModel->m_engineMotor = CreateEngineMotor(controller, vehicleModel->m_engineJoint);
		//
		//// set power parameter for a simple DC engine
		//vehicleModel->m_maxTurmVelocity = 10.0f;
		//vehicleModel->m_maxEngineSpeed = 30.0f;
		//vehicleModel->m_engineMotor->SetTorque(2000.0f);
		//vehicleModel->m_engineMotor->SetTorque1(2500.0f);
		//
		//// set the steering torque 
		//vehicleModel->m_engineJoint->SetFriction(500.0f);
		//AddCraneBase (controller);

#if 0
	/*

		dCustomTransformController::dSkeletonBone* stackPool[32];
		stackPool[0] = chassisBone;
		int stack = 1;
		while (stack) {
			stack --;
			dCustomTransformController::dSkeletonBone* const bone = stackPool[stack];
			NewtonBody* const body = bone->m_body;
			NewtonCollision* const collision = NewtonBodyGetCollision(body);

			NewtonCollisionSetUserData (collision, controller);
			if (NewtonCollisionGetType(collision) == SERIALIZE_ID_COMPOUND) {
				for (void* node = NewtonCompoundCollisionGetFirstNode(collision); node; node = NewtonCompoundCollisionGetNextNode(collision, node)) {
					NewtonCollision* const subShape = NewtonCompoundCollisionGetCollisionFromNode (collision, node);
					NewtonCollisionSetUserData (subShape, controller);
				}
			}

			for (dCustomTransformController::dSkeletonBone::dListNode* node = bone->GetFirst(); node; node = node->GetNext()) {
				stackPool[stack] = &node->GetInfo(); 
				stack ++;
			}
		}
*/
#endif
		return controller;
	}

	virtual void OnDebug(dModelRootNode* const model, dCustomJoint::dDebugDisplay* const debugContext) 
	{
		//dExcavatorModel* const excavator = (dExcavatorModel*) model;
		//excavator->OnDebug(debugContext);
	}

	virtual void OnPreUpdate(dModelRootNode* const model, dFloat timestep) const 
	{
		//dExcavatorModel* const excavator = (dExcavatorModel*) model;
		//excavator->CalculateTireContacts();
	}

	virtual void OnUpdateTransform(const dModelNode* const bone, const dMatrix& localMatrix) const
	{
		NewtonBody* const body = bone->GetBody();
		DemoEntity* const ent = (DemoEntity*)NewtonBodyGetUserData(body);
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(NewtonBodyGetWorld(body));

		dQuaternion rot(localMatrix);
		ent->SetMatrix(*scene, rot, localMatrix.m_posit);
	}

	static int StandardAABBOverlapTest(const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
	{
		const NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
		const NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);
		const NewtonCollision* const collision0 = NewtonBodyGetCollision(body0);
		const NewtonCollision* const collision1 = NewtonBodyGetCollision(body1);

		if (NewtonCollisionGetUserData(collision0) != NewtonCollisionGetUserData(collision1)) {
			return 1;
		}

		NewtonCollisionMaterial material0;
		NewtonCollisionMaterial material1;
		NewtonCollisionGetMaterial(collision0, &material0);
		NewtonCollisionGetMaterial(collision1, &material1);

		//m_terrain	 = 1 << 0,
		//m_bodyPart = 1 << 1,
		//m_tirePart = 1 << 2,
		//m_linkPart = 1 << 3,
		//m_propBody = 1 << 4,
	
		const dLong mask0 = material0.m_userId & material1.m_userParam[0].m_int;
		const dLong mask1 = material1.m_userId & material0.m_userParam[0].m_int;
		return (mask0 && mask1) ? 1 : 0;
	}

	static int ThreadStaticContactsGeneration (const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonCollision* const collision0, const NewtonBody* const body1, const NewtonCollision* const collision1, NewtonUserContactPoint* const contactBuffer, int maxCount, int threadIndex)
	{
		//const NewtonCollision* const collision0 = NewtonBodyGetCollision(body0);
		//const NewtonCollision* const collision1 = NewtonBodyGetCollision(body1);
		dAssert(NewtonBodyGetMaterialGroupID(body0) == NewtonBodyGetMaterialGroupID(body1));
		dAssert(NewtonBodyGetMaterialGroupID(body0) != NewtonMaterialGetDefaultGroupID(NewtonBodyGetWorld(body0)));
		dAssert(NewtonBodyGetMaterialGroupID(body1) != NewtonMaterialGetDefaultGroupID(NewtonBodyGetWorld(body1)));

		dAssert(NewtonCollisionGetUserID(collision0) == ARTICULATED_VEHICLE_DEFINITION::m_linkPart);
		dAssert(NewtonCollisionGetUserID(collision1) == ARTICULATED_VEHICLE_DEFINITION::m_terrain);
		//dAssert (NewtonCollisionGetUserID(collision1) & (ARTICULATED_VEHICLE_DEFINITION::m_linkPart | ARTICULATED_VEHICLE_DEFINITION::m_terrain));

		//dExcavatorModel* const excavator = (dExcavatorModel*)NewtonCollisionGetUserData(collision0);
		//dAssert (excavator);
		//return excavator->OnBoneAABBOverlap(contactJoint);

		return 0;
	}

	int m_threadMaterialID;
};

void ArticulatedJoints (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	int specialThreadId = NewtonMaterialCreateGroupID(scene->GetNewton());

	NewtonBody* const floor = CreateLevelMesh (scene, "flatPlane.ngd", true);
	//NewtonBody* floor = CreateHeightFieldTerrain (scene, 9, 8.0f, 1.5f, 0.2f, 200.0f, -50.0f);
	NewtonCollision* const floorCollision = NewtonBodyGetCollision(floor);

	// set collision filter mask
	NewtonCollisionMaterial material;
	NewtonCollisionGetMaterial(floorCollision, &material);
	material.m_userId = ARTICULATED_VEHICLE_DEFINITION::m_terrain;
	material.m_userParam[0].m_int =
		//ARTICULATED_VEHICLE_DEFINITION::m_terrain |
		ARTICULATED_VEHICLE_DEFINITION::m_bodyPart |
		ARTICULATED_VEHICLE_DEFINITION::m_linkPart |
		ARTICULATED_VEHICLE_DEFINITION::m_tirePart |
		ARTICULATED_VEHICLE_DEFINITION::m_propBody;

	NewtonCollisionSetMaterial(floorCollision, &material);
	NewtonBodySetMaterialGroupID(floor, specialThreadId);

	// add an input Manage to manage the inputs and user interaction 
	//AriculatedJointInputManager* const inputManager = new AriculatedJointInputManager (scene);

	//  create a skeletal transform controller for controlling rag doll
	ArticulatedVehicleManagerManager* const vehicleManager = new ArticulatedVehicleManagerManager (scene, specialThreadId);

	NewtonWorld* const world = scene->GetNewton();
	dVector origin (FindFloor (world, dVector (-10.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));

	dMatrix matrix (dGetIdentityMatrix());
	matrix.m_posit = FindFloor (world, origin, 100.0f);
	matrix.m_posit.m_y += 1.5f;

	//DemoEntity* const model = DemoEntity::LoadNGD_mesh ("excavator.ngd", scene->GetNewton(), scene->GetShaderCache());
	//scene->Append(xxxx);
	//xxxx->ResetMatrix(*scene, matrix);

//	ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*)robotModel.CreateClone();
//	scene->Append(vehicleModel);
//	ArticulatedEntityModel robotModel(scene, "excavator.ngd");

	dModelRootNode* const excavator = vehicleManager->CreateExcavator ("excavator.ngd", matrix);
/*
	inputManager->AddPlayer (robot);

	matrix.m_posit.m_z += 4.0f;

	// add some object to play with
//	LoadLumberYardMesh(scene, dVector(6.0f, 0.0f, 0.0f, 0.0f), ARTICULATED_VEHICLE_DEFINITION::m_woodSlab);
//	LoadLumberYardMesh(scene, dVector(6.0f, 0.0f, 10.0f, 0.0f), ARTICULATED_VEHICLE_DEFINITION::m_woodSlab);
//	LoadLumberYardMesh (scene, dVector(10.0f, 0.0f, -5.0f, 0.0f), ARTICULATED_VEHICLE_DEFINITION::m_woodSlab);
//	LoadLumberYardMesh (scene, dVector(10.0f, 0.0f,  5.0f, 0.0f), ARTICULATED_VEHICLE_DEFINITION::m_woodSlab);
//	LoadLumberYardMesh(scene, dVector(14.0f, 0.0f, 0.0f, 0.0f), ARTICULATED_VEHICLE_DEFINITION::m_woodSlab);
//	LoadLumberYardMesh(scene, dVector(14.0f, 0.0f, 10.0f, 0.0f), ARTICULATED_VEHICLE_DEFINITION::m_woodSlab);
//	LoadLumberYardMesh(scene, dVector(18.0f, 0.0f, -5.0f, 0.0f), ARTICULATED_VEHICLE_DEFINITION::m_woodSlab);
//	LoadLumberYardMesh(scene, dVector(18.0f, 0.0f,  5.0f, 0.0f), ARTICULATED_VEHICLE_DEFINITION::m_woodSlab);
*/

	origin.m_x -= 15.0f;
	origin.m_y += 5.0f;
	dQuaternion rot (dVector (0.0f, 1.0f, 0.0f, 0.0f), -30.0f * dDegreeToRad);  
	scene->SetCameraMatrix(rot, origin);
}
