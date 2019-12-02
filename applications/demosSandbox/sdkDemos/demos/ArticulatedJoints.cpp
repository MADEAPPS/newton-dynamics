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
		,m_shereCast (NewtonCreateSphere (NewtonBodyGetWorld(rootBody), 0.01f, 0, NULL))
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
				debugContext->DrawPoint(entry->m_point[j], 20.0f);
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

	private:
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

			scanMatrix.m_posit = targetPoint;
			NewtonWorld* const world = NewtonBodyGetWorld(staticBody);
			for (int i = 0; i < collidingBodies.m_threaLinkCount && (tire->m_count < 2); i ++) {
				dMatrix linkMatrix;
				//dVector veloc (targetPoint - tireMatrix.m_posit);
				dVector dir (tireMatrix.m_posit - scanMatrix.m_posit);
				dir = dir.Normalize();
				dVector veloc (dir.Scale (0.7f));
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
		for (int i = 0; i < 3; i++) {
			char name[64];
			sprintf(name, "leftRoller%d", i);
			dModelNode* const rollerTire = MakeRollerTire(name);
			m_tireArray[m_tireCount++].m_tire = rollerTire->GetBody();
			LinkTires (leftTire_0, rollerTire);
		}
		MakeRollerTire("leftSupportRoller");
	}

	void MakeRightTrack()
	{
		dModelNode* const rightTire_0 = MakeRollerTire("rightGear");
		dModelNode* const rightTire_7 = MakeRollerTire("rightFrontRoller");
		m_tireArray[m_tireCount++].m_tire = rightTire_0->GetBody();
		m_tireArray[m_tireCount++].m_tire = rightTire_7->GetBody();

		LinkTires(rightTire_0, rightTire_7);
		for (int i = 0; i < 3; i++) {
			char name[64];
			sprintf(name, "rightRoller%d", i);
			dModelNode* const rollerTire = MakeRollerTire(name);
			m_tireArray[m_tireCount++].m_tire = rollerTire->GetBody();
			LinkTires(rightTire_0, rollerTire);
		}
		MakeRollerTire("rightSupportRoller");
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

		// set collision filter
		//NewtonCollisionMaterial material;
		//NewtonCollisionGetMaterial(shape, &material);
		//material.m_userId = ARTICULATED_VEHICLE_DEFINITION::m_bodyPart;
		//material.m_userParam[0].m_int = ARTICULATED_VEHICLE_DEFINITION::m_bodyPart + ARTICULATED_VEHICLE_DEFINITION::m_woodSlab + ARTICULATED_VEHICLE_DEFINITION::m_terrain;
		//NewtonCollisionSetMaterial(shape, &material);

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

#if 0
class ArticulatedEntityModel: public DemoEntity
{
	public:
	class InputRecord
	{
		public:
		InputRecord()
		{
			memset (this, 0, sizeof (InputRecord));
		}

		dFloat m_gripperValue;
		dFloat m_slideValue;
		dFloat m_turnValue;
		dFloat m_liftValue;
		dFloat m_gripperRollValue;
		dFloat m_gripperPitchValue;
		int m_steerValue;
		int m_throttleValue;
	};

	ArticulatedEntityModel (DemoEntityManager* const scene, const char* const name)
		:DemoEntity(dGetIdentityMatrix(), NULL)
		,m_tractionTiresCount(0)
		,m_liftActuatorsCount(0)
		,m_paletteActuatorsCount(0)
		,m_maxEngineTorque(0.0f)
		,m_engineMotor(NULL)
		,m_engineJoint(NULL)
		,m_angularActuator0(NULL)
		,m_angularActuator1(NULL)
		,m_gripperRotator(NULL)
	{
		// load the vehicle model
		LoadNGD_mesh (name, scene->GetNewton(), scene->GetShaderCache());
	}

	ArticulatedEntityModel (const ArticulatedEntityModel& copy)
		:DemoEntity(copy)
		,m_tractionTiresCount(0)
		,m_liftActuatorsCount(0)
		,m_paletteActuatorsCount(0)
		,m_maxEngineTorque(0.0f)
		,m_maxEngineSpeed(30.0f)
		,m_maxTurmVelocity(10.0f)
		,m_tiltAngle(0.0f)
		,m_liftPosit(0.0f)
		,m_engineMotor(NULL)
		,m_engineJoint(NULL)
		,m_angularActuator0(NULL)
		,m_angularActuator1(NULL)
		,m_gripperRotator(NULL)
	{
	}

	DemoEntity* CreateClone() const
	{
		return new ArticulatedEntityModel(*this);
	}

	void SetInput (const InputRecord& inputs)
	{
		m_inputs = inputs;
	}

	int m_tractionTiresCount;
	int m_liftActuatorsCount;
	int m_paletteActuatorsCount;
	dFloat m_maxEngineTorque;
	dFloat m_maxEngineSpeed;
	dFloat m_maxTurmVelocity;
	dFloat m_tiltAngle;
	dFloat m_liftPosit;

	dCustomMotor2* m_engineMotor;
	dCustomDoubleHinge* m_engineJoint;
	dCustomHingeActuator* m_angularActuator0;
	dCustomHingeActuator* m_angularActuator1;
	dCustomDoubleHingeActuator* m_gripperRotator;

	dCustomSlidingContact* m_tractionTiresJoints[4];
	
	dCustomSliderActuator* m_liftJoints[2];
	dCustomSliderActuator* m_paletteJoints[4];
	

	InputRecord m_inputs;
};


// we recommend using and input manage to control input for all games
class AriculatedJointInputManager: public dCustomListener
{
	public:
	AriculatedJointInputManager (DemoEntityManager* const scene)
		:dCustomListener(scene->GetNewton(), "D_LISTENER")
		,m_scene(scene)
		,m_cameraMode(true)
		,m_changeVehicle(true)
		,m_playersCount(0)
		,m_currentPlayer(0)
		,m_gripper(0.0f)
		,m_liftboom(0.0f) 
		,m_slideboom(0.0f) 
		,m_rotatebase(0.0f)
		,m_gripperRoll(0.0f)
		,m_gripperPitch(0.0f)
	{
		// plug a callback for 2d help display
		memset (m_player, 0, sizeof (m_player));
		scene->Set2DDisplayRenderFunction (RenderPlayerHelp, NULL, this);
		scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
	}

	void OnBeginUpdate (dFloat timestepInSecunds)
	{
		dAssert(0);
		/*
		ArticulatedEntityModel::InputRecord inputs;
		if (m_playersCount && m_player[m_currentPlayer % m_playersCount]) {
			dCustomTransformController* const player = m_player[m_currentPlayer % m_playersCount];
			ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*) player->GetUserData();

			inputs.m_steerValue = int (m_scene->GetKeyState ('D')) - int (m_scene->GetKeyState ('A'));
			inputs.m_throttleValue = int (m_scene->GetKeyState ('W')) - int (m_scene->GetKeyState ('S'));

			inputs.m_gripperValue = m_gripper;
			inputs.m_slideValue = m_slideboom;
			inputs.m_liftValue = m_liftboom * dDegreeToRad;
			inputs.m_turnValue = m_rotatebase * dDegreeToRad;
			inputs.m_gripperRollValue = m_gripperRoll * dDegreeToRad;
			inputs.m_gripperPitchValue = m_gripperPitch * dDegreeToRad;

			// check if we must activate the player
			if (m_needsWakeUp ||
				m_scene->GetKeyState ('A') || 
				m_scene->GetKeyState ('D') ||
				m_scene->GetKeyState ('W') ||
				m_scene->GetKeyState ('S'))
			{
				NewtonBody* const body = m_player[m_currentPlayer % m_playersCount]->GetRoot()->m_body;
				NewtonBodySetSleepState(body, false);
			}

#if 0
	#if 0
			static FILE* file = fopen ("log.bin", "wb");
			if (file) {
				fwrite (&inputs, sizeof (inputs), 1, file);
				fflush(file);
			}
	#else 
			static FILE* file = fopen ("log.bin", "rb");
			if (file) {
				fread (&inputs, sizeof (inputs), 1, file);
			}
	#endif
#endif
			vehicleModel->SetInput (inputs);
		}
*/
	}

	static void UpdateCameraCallback(DemoEntityManager* const manager, void* const context, dFloat timestep)
	{
		AriculatedJointInputManager* const me = (AriculatedJointInputManager*)context;
		me->UpdateCamera(timestep);
	}

	void UpdateCamera (dFloat timestepInSecunds)
	{
		dAssert(0);
		/*
		if (m_playersCount && m_player[m_currentPlayer % m_playersCount]) {
			DemoCamera* const camera = m_scene->GetCamera();
			ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*) m_player[m_currentPlayer % m_playersCount]->GetUserData();

			if (m_changeVehicle.UpdateTrigger(m_scene->GetKeyState ('P'))) {
				m_currentPlayer ++;
			}
		
			dMatrix camMatrix(camera->GetNextMatrix());
			dMatrix playerMatrix (vehicleModel->GetNextMatrix());

			dVector frontDir (camMatrix[0]);
			dVector camOrigin(0.0f); 
			m_cameraMode.UpdatePushButton(m_scene->GetKeyState('C'));
			if (m_cameraMode.GetPushButtonState()) {
				camOrigin = playerMatrix.TransformVector( dVector(0.0f, ARTICULATED_VEHICLE_CAMERA_HIGH_ABOVE_HEAD, 0.0f, 0.0f));
				camOrigin -= frontDir.Scale(ARTICULATED_VEHICLE_CAMERA_DISTANCE);
			} else {
				camMatrix = camMatrix * playerMatrix;
				camOrigin = playerMatrix.TransformVector(dVector(-0.8f, ARTICULATED_VEHICLE_CAMERA_EYEPOINT, 0.0f, 0.0f));
			}

			camera->SetNextMatrix (*m_scene, camMatrix, camOrigin);
		}
*/
	}

	void OnEndUpdate (dFloat timestepInSecunds)
	{
	}

	void AddPlayer(dCustomTransformController* const player)
	{
		m_player[m_playersCount] = player;
		m_playersCount ++;
	}

	void RenderPlayerHelp (DemoEntityManager* const scene)
	{
		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print (color, "Navigation Keys");
		scene->Print (color, "drive forward:      W");
		scene->Print (color, "drive backward:     S");
		scene->Print (color, "turn right:         D");
		scene->Print (color, "turn left:          A");

		m_needsWakeUp = false;
		m_needsWakeUp = ImGui::SliderFloat("Gripper", &m_gripper, -0.2f, 2.0f) || m_needsWakeUp;
		m_needsWakeUp = ImGui::SliderFloat("Roll", &m_gripperRoll, -180.0f, 180.0f) || m_needsWakeUp;
		m_needsWakeUp = ImGui::SliderFloat("Pitch", &m_gripperPitch, -180.0f, 180.0f) || m_needsWakeUp;
		m_needsWakeUp = ImGui::SliderFloat("BoomSlide", &m_slideboom, 0.0f, 3.0f) || m_needsWakeUp;
		m_needsWakeUp = ImGui::SliderFloat("BoomLift", &m_liftboom, -180.0f, 180.0f) || m_needsWakeUp;
		m_needsWakeUp = ImGui::SliderFloat("RotateBase", &m_rotatebase, -180.0f, 180.0f) || m_needsWakeUp;
	}				

	static void RenderPlayerHelp (DemoEntityManager* const scene, void* const context)
	{
		AriculatedJointInputManager* const me = (AriculatedJointInputManager*) context;
		me->RenderPlayerHelp (scene);
	}

	DemoEntityManager* m_scene;
	dCustomTransformController* m_player[2];
	DemoEntityManager::ButtonKey m_cameraMode;
	DemoEntityManager::ButtonKey m_changeVehicle;
	int m_playersCount;
	int m_currentPlayer;
	bool m_needsWakeUp;
	dFloat32 m_gripper; 
	dFloat32 m_liftboom; 
	dFloat32 m_slideboom; 
	dFloat32 m_rotatebase; 
	dFloat32 m_gripperRoll; 
	dFloat32 m_gripperPitch; 
};
#endif

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
		NewtonMaterialSetCollisionCallback (world, material, material, StandardAABBOverlapTest, StandardContactsProcess);

		NewtonMaterialSetCallbackUserData(world, material, threadMaterialID, this);
		NewtonMaterialSetCollisionCallback (world, material, threadMaterialID, StandardAABBOverlapTest, StandardContactsProcess);

		NewtonMaterialSetCallbackUserData(world, threadMaterialID, threadMaterialID, this);
		NewtonMaterialSetCollisionCallback (world, threadMaterialID, threadMaterialID, SpecialAABBOverlapTest, SpecialContactsProcess);
		NewtonMaterialSetContactGenerationCallback (world, threadMaterialID, threadMaterialID, SpecialGenerateLinkGroundContacts);
	}

#if 0
	virtual void OnPreUpdate (dCustomTransformController* const controller, dFloat timestep, int threadIndex) const
	{
		dAssert(0);
		/*
		ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*)controller->GetUserData();

		if (vehicleModel->m_engineJoint) {
			dFloat engineRPM = 0.0f;
			dFloat brakeTorque = 2000.0f;
			
			if (vehicleModel->m_inputs.m_throttleValue > 0) {
				brakeTorque = 0.0f;
				engineRPM = -vehicleModel->m_maxEngineSpeed;
			} else if (vehicleModel->m_inputs.m_throttleValue < 0) {
				brakeTorque = 0.0f;
				engineRPM = vehicleModel->m_maxEngineSpeed;
			}

			// apply DC engine torque
			vehicleModel->m_engineMotor->SetSpeed1(engineRPM);

			// apply DC rate turn Motor 
			if (vehicleModel->m_inputs.m_steerValue > 0) {
				brakeTorque = 0.0f;
				vehicleModel->m_engineMotor->SetSpeed(vehicleModel->m_maxTurmVelocity);
			} else if (vehicleModel->m_inputs.m_steerValue < 0){
				brakeTorque = 0.0f;
				vehicleModel->m_engineMotor->SetSpeed(-vehicleModel->m_maxTurmVelocity);
			} else {
				vehicleModel->m_engineMotor->SetSpeed(0.0f);
			}

			// apply breaks
			for (int i = 0; i < vehicleModel->m_tractionTiresCount; i ++) {
				vehicleModel->m_tractionTiresJoints[i]->SetFriction(brakeTorque);
			}
		}

		// apply crane controls
		if (vehicleModel->m_angularActuator0) {
			vehicleModel->m_angularActuator1->SetTargetAngle(vehicleModel->m_inputs.m_turnValue);
			vehicleModel->m_angularActuator0->SetTargetAngle(vehicleModel->m_inputs.m_liftValue);
			vehicleModel->m_liftJoints[0]->SetTargetPosit(vehicleModel->m_inputs.m_slideValue);
			vehicleModel->m_liftJoints[1]->SetTargetPosit(vehicleModel->m_inputs.m_slideValue);
			vehicleModel->m_gripperRotator->SetTargetAngle0(vehicleModel->m_inputs.m_gripperPitchValue);
			vehicleModel->m_gripperRotator->SetTargetAngle1(vehicleModel->m_inputs.m_gripperRollValue);

			// open Close palette position
			for (int i = 0; i < vehicleModel->m_paletteActuatorsCount; i++) {
				vehicleModel->m_paletteJoints[i]->SetTargetPosit(vehicleModel->m_inputs.m_gripperValue);
			}
		}
*/
	}

	static int CompoundSubCollisionAABBOverlap (const NewtonJoint* const contact, dFloat timestep, const NewtonBody* const body0, const void* const collisionNode0, const NewtonBody* const body1, const void* const collisionNode1, int threadIndex)
	{
		dAssert(collisionNode0);
		NewtonCollision* const collision0 = NewtonCompoundCollisionGetCollisionFromNode (NewtonBodyGetCollision(body0), collisionNode0);
		NewtonCollision* const collision1 = collisionNode1 ? NewtonCompoundCollisionGetCollisionFromNode(NewtonBodyGetCollision(body1), collisionNode1) : NewtonBodyGetCollision(body1);

		NewtonCollisionMaterial material0;
		NewtonCollisionMaterial material1;
		NewtonCollisionGetMaterial(collision0, &material0);
		NewtonCollisionGetMaterial(collision1, &material1);

		dAssert(0);
		return 0;
		//int mask = material0.m_userId | material1.m_userId;
		//int filter0 = material0.m_userId & material1.m_userFlags;
		//int filter1 = material1.m_userId & material0.m_userFlags;
		//return ((filter0 + filter1) == mask) ? 1 : 0;
	}


	dCustomTransformController::dSkeletonBone* CreateEngineNode(dCustomTransformController* const controller, dCustomTransformController::dSkeletonBone* const chassisBone)
	{
		dAssert(0);
		return NULL;
		/*

		NewtonWorld* const world = GetWorld();
		NewtonCollision* const shape = NewtonCreateCylinder (world, 0.125f, 0.125f, 0.75f, 0, NULL);

		NewtonBody* const chassis = chassisBone->m_body;

		// create the rigid body that will make this bone
		dMatrix engineMatrix;
		NewtonBodyGetMatrix(chassis, &engineMatrix[0][0]);
		engineMatrix = dRollMatrix(0.5f * dPi) * engineMatrix;
		engineMatrix.m_posit.m_y += 1.0f;

		NewtonBody* const engineBody = NewtonCreateDynamicBody(world, shape, &engineMatrix[0][0]);

		// destroy the collision helper shape 
		NewtonDestroyCollision(shape);

		// get the collision from body
		NewtonCollision* const collision = NewtonBodyGetCollision(engineBody);
		NewtonCollisionSetMode (collision, 0);

		// calculate the moment of inertia and the relative center of mass of the solid
		//NewtonBodySetMassProperties(engineBody, 50.0f, collision);
		dFloat mass = 50.0f;
		dFloat radius = 1.0f;
		dFloat Inertia = 2.0f * mass * radius * radius / 5.0f;
		NewtonBodySetMassMatrix (engineBody, mass, Inertia, Inertia, Inertia);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback(engineBody, PhysicsApplyGravityForce);

		// connect engine to chassis with a hinge
		dMatrix engineAxis;
		engineAxis.m_front = engineMatrix.m_front;
		engineAxis.m_up = engineMatrix.m_right;
		engineAxis.m_right = engineAxis.m_front.CrossProduct(engineAxis.m_up);
		engineAxis.m_posit = engineMatrix.m_posit;

		dCustomDoubleHinge* const engineJoint = new dCustomDoubleHinge(engineAxis, engineBody, chassis);
		engineJoint->EnableLimits(false);
		engineJoint->EnableLimits1(false);
		return controller->AddBone(engineBody, dGetIdentityMatrix(), chassisBone);
		*/
	}

	dCustomMotor2* CreateEngineMotor(dCustomTransformController* const controller, dCustomDoubleHinge* const engineJoint)
	{
		dMatrix engineMatrix;
		dMatrix chassisMatrix;
		NewtonBody* const engine = engineJoint->GetBody0();
		NewtonBody* const chassis = engineJoint->GetBody1();
		engineJoint->CalculateGlobalMatrix(engineMatrix, chassisMatrix);
		return new dCustomMotor2(engineMatrix.m_front, engineMatrix.m_up, engine, engine, chassis);
	}

	dCustomTransformController::dSkeletonBone* MakeTractionTire(const char* const entName, const char* const tireName, dCustomTransformController* const controller, dCustomTransformController::dSkeletonBone* const parentBone)
	{
		dAssert(0);
		return NULL;
		/*

		ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*)controller->GetUserData();

		NewtonBody* const parentBody = parentBone->m_body;
		DemoEntity* const parentModel = (DemoEntity*)NewtonBodyGetUserData(parentBody);
		DemoEntity* const tireModel = parentModel->Find(entName);

		NewtonCollision* const tireCollision = MakeDoubleRingTireShape(tireModel);
		dCustomTransformController::dSkeletonBone* const bone = MakeTireBody(entName, tireName, controller, parentBone, tireCollision);
		NewtonDestroyCollision(tireCollision);

		// connect the tire tp the body with a hinge
		dMatrix matrix;
		NewtonBodyGetMatrix(bone->m_body, &matrix[0][0]);
		dMatrix tireHingeMatrix(dRollMatrix(0.0f * dDegreeToRad) * matrix);

		vehicleModel->m_tractionTiresJoints[vehicleModel->m_tractionTiresCount] = new dCustomSlidingContact(tireHingeMatrix, bone->m_body, parentBone->m_body);
		vehicleModel->m_tractionTiresJoints[vehicleModel->m_tractionTiresCount]->EnableLimits(true);
		vehicleModel->m_tractionTiresJoints[vehicleModel->m_tractionTiresCount]->SetLimits(0.0f, 0.0f);
		vehicleModel->m_tractionTiresJoints[vehicleModel->m_tractionTiresCount]->SetAsSpringDamper(false, 1.0f, 0.0f, 0.0f);
		vehicleModel->m_tractionTiresCount++;

		// link traction tire to the engine using a differential gear
		dMatrix engineMatrix;
		dMatrix chassisMatrix;
		
		NewtonBody* const tire = bone->m_body;
		NewtonBody* const engine = vehicleModel->m_engineJoint->GetBody0();
		NewtonBody* const chassis = vehicleModel->m_engineJoint->GetBody1();
		vehicleModel->m_engineJoint->CalculateGlobalMatrix(engineMatrix, chassisMatrix);

		dFloat sign = dSign(engineMatrix.m_up.DotProduct3(tireHingeMatrix.m_posit - engineMatrix.m_posit));
		new dCustomDifferentialGear(5.0f, tireHingeMatrix.m_up, engineMatrix.m_front.Scale(sign), chassisMatrix.m_up, tire, engine, chassis);
		return bone;
*/
	}

		
	class ConstantSpeedKnotInterpolant
	{
		public:
		dFloat m_u;
		dFloat m_dist;
	};

	dFloat CalculateKnotParam(const ConstantSpeedKnotInterpolant* const interpolants, int interpolantsCount, dFloat t) const
	{
		int low = 0;
		int high = interpolantsCount - 1;

		while ((high - low) >= 4) {
			int mid = (low + high) >> 1;
			if (t > interpolants[mid].m_dist) {
				low = mid;
			} else {
				high = mid;
			}
		}

		dAssert(interpolants[low].m_dist <= t);
		for (int i = low; i < interpolantsCount; i++) {
			if (interpolants[i + 1].m_dist >= t) {
				low = i;
				break;
			}
		}

		dFloat u0 = interpolants[low].m_u;
		dFloat h0 = interpolants[low].m_dist;
		dFloat du = interpolants[low + 1].m_u - u0;
		dFloat dh = interpolants[low + 1].m_dist - h0;
		return dMod(u0 + du * (t - h0) / dh, 1.0f);
	}

	void CalculaterUniformSpaceSamples(DemoEntity* const chassis, dFloat offset, dCustomTransformController::dSkeletonBone* const rootNode, dCustomTransformController* const controller)
	{
		dAssert(0);
/*
		dFloat linkLength = 0.33f;

		DemoEntity* const threadPath = chassis->Find("trackPath");
		dAssert(threadPath);
		DemoBezierCurve* const bezierPath = (DemoBezierCurve*) threadPath->GetMesh();

		dFloat length = dFloat (bezierPath->m_curve.CalculateLength (0.01f));
		int linksCount = int(dFloor (length / linkLength)) + 1;
		linkLength = length / linksCount;

		// create the uniform speed knot interpolation table
		ConstantSpeedKnotInterpolant steps[2048];
		steps[0].m_u = 0.0f;
		steps[0].m_dist = 0.0f;

		int count = 0;
		steps[count].m_u = 0.0f;
		steps[count].m_dist = 0.0f;
		count++;
		int samplingRate = linksCount * 20;
		dFloat distAcc = 0.0f;

		dFloat stepAcc = linkLength;
		dVector p0(bezierPath->m_curve.CurvePoint(0.0f));
		for (int i = 1; i < samplingRate + 45; i++) {
			dFloat u = dFloat(i) / samplingRate;
			dVector p1(bezierPath->m_curve.CurvePoint(dMod(u, 1.0f)));
			dVector err(p1 - p0);
			dFloat errMag = dSqrt(err.DotProduct3(err));
			distAcc += errMag;
			if (distAcc >= stepAcc) {
				stepAcc += linkLength;
				steps[count].m_u = u;
				steps[count].m_dist = distAcc;
				count++;
				dAssert(count < int(sizeof (steps) / sizeof (steps[0])));
			}
			p0 = p1;
		}

		dMatrix rootMatrix (chassis->GetCurrentMatrix());

		dMatrix aligmentMatrix (dRollMatrix(180.0f * dDegreeToRad));

		DemoEntity* const threadLink = chassis->Find("link");
		dMatrix shapeMatrix (threadPath->GetMeshMatrix() * threadPath->GetCurrentMatrix());

		DemoEntity* const threadLinkChild = chassis->Find("Object001");
		dMatrix shapeChildMatrix (threadLinkChild->GetCurrentMatrix());

		dFloat s = 0.0f;
		dMatrix matrix(dGetIdentityMatrix());
		dFloat u0 = CalculateKnotParam(steps, linksCount, s);
		dVector r0(bezierPath->m_curve.CurvePoint(u0));

		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);
		dMatrix linkMatrix (dGetIdentityMatrix());
		linkMatrix.m_posit.m_y = linkLength * 0.5f;
		NewtonCollision* const collision = NewtonCreateBox (GetWorld(), 0.06f, linkLength, 0.5f, 0, &linkMatrix[0][0]);
		NewtonCollisionSetUserID(collision, ARTICULATED_VEHICLE_DEFINITION::m_linkPart );
		NewtonCollisionSetUserData (collision, controller);

		NewtonBody* linkArray[1024];

		int bodyCount = 0;
		// add extra link for wiggle room
		//linksCount += 2;
		linksCount += 1;

		void* const aggregate = NewtonCollisionAggregateCreate(world);
		NewtonCollisionAggregateSetSelfCollision (aggregate, 0);
		for (int i = 1; i < linksCount + 1; i++) {
			s += linkLength;
			dFloat u1 = CalculateKnotParam(steps, linksCount, dMod (s, length));
			dVector r1(bezierPath->m_curve.CurvePoint(u1));
			dVector dir(r1 - r0);

			dir = dir.Scale(1.0f / dSqrt(dir.DotProduct3(dir)));
			matrix.m_front = dVector(dir.m_z, -dir.m_y, 0.0f, 0.0f);
			matrix.m_up = dVector(dir.m_y, dir.m_z, 0.0f, 0.0f);
			matrix.m_right = dVector(0.0f, 0.0f, 1.0f, 0.0f);
			matrix = aligmentMatrix * matrix;
			matrix.m_posit = shapeMatrix.TransformVector(r0);
			matrix.m_posit.m_z = offset;

			matrix = matrix * rootMatrix;
			NewtonBody* const link = CreateSimpleSolid (scene, (DemoMesh*)threadLink->GetMesh(), 10.0f, matrix, collision, 0);
			NewtonCollisionAggregateAddBody (aggregate, link);
			linkArray[bodyCount] = link;
			bodyCount ++;
			DemoEntity* const threadPart = (DemoEntity*) NewtonBodyGetUserData(link);

			DemoEntity* const threadPartChild = new DemoEntity(shapeChildMatrix, threadPart);
			threadPartChild->SetMesh(threadLinkChild->GetMesh(), dGetIdentityMatrix());

			r0 = r1;
			u0 = u1;
		}
		NewtonDestroyCollision(collision);

		dMatrix aligment (dYawMatrix(90.0f * dDegreeToRad));
		NewtonBody* link0 = linkArray[0];

		NewtonJoint* hingeArray[1024];
		for (int i = 1; i < bodyCount; i++) {
			NewtonBody* const link1 = linkArray[i];

			dMatrix matrix;
			NewtonBodyGetMatrix(link1, &matrix[0][0]);
			dMatrix franmeMatrix (aligment * matrix);
			dCustomHinge* const hinge = new dCustomHinge (franmeMatrix, link1, link0);
			hinge->SetStiffness(0.99f);
			hinge->SetAsSpringDamper(true, 0.9f, 0.0f, 20.0f);

			hingeArray[i-1] = hinge->GetJoint();
			link0 = link1;
		}

		dMatrix matrix0;
		dMatrix matrix1;
		dMatrix matrix2;
		NewtonBodyGetMatrix(linkArray[0], &matrix0[0][0]);
		NewtonBodyGetMatrix(linkArray[1], &matrix2[0][0]);
		NewtonBodyGetMatrix(linkArray[bodyCount - 1], &matrix1[0][0]);
		
		dVector dist (matrix2.m_posit - matrix0.m_posit);
		matrix1.m_posit += dist;
		dCustomHinge* const hinge = new dCustomHinge (aligment * matrix0, aligment * matrix1, linkArray[0], linkArray[bodyCount - 1]);
		hinge->SetStiffness(0.99f);
		hinge->SetAsSpringDamper(true, 0.9f, 0.0f, 20.0f);

		dVector com (0.0f);
		for (int i = 0; i < bodyCount; i++) {
			dMatrix matrix;
			NewtonBody* const link = linkArray[i];
			NewtonBodyGetMatrix(link, &matrix[0][0]);
			com += matrix.m_posit;
		}
		com = com.Scale (1.0f / bodyCount);
		com.m_w = 1.0f;

		NewtonBody* const chassisBody = rootNode->m_body;
		NewtonBody* const chainBody = linkArray[0];
		dMatrix planeMatrix;
		NewtonBodyGetMatrix(chassisBody, &planeMatrix[0][0]);
		new dCustomPlane (com, planeMatrix.m_right, chainBody, chassisBody);
*/
	}

	dCustomTransformController::dSkeletonBone* AddCraneBoom(dCustomTransformController* const controller, dCustomTransformController::dSkeletonBone* const baseBone, const char* name)
	{
		dAssert(0);
		return 0;
/*
		dCustomTransformController::dSkeletonBone* const chassisBone = controller->GetRoot();
		ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*)NewtonBodyGetUserData(chassisBone->m_body);

		DemoEntity* const boom = vehicleModel->Find(name);

		ARTICULATED_VEHICLE_DEFINITION definition;
		strcpy(definition.m_boneName, name);
		strcpy(definition.m_shapeTypeName, "convexHull");
		definition.m_mass = 10.0f;
		definition.m_bodyPartID = ARTICULATED_VEHICLE_DEFINITION::m_bodyPart;
		strcpy(definition.m_articulationName, name);
		NewtonBody* const boomBody = CreateBodyPart(boom, definition);

		dMatrix matrix;
		NewtonBodyGetMatrix(boomBody, &matrix[0][0]);
		matrix = dRollMatrix(0.5f * dPi) * matrix;

		dFloat minLimit =  0.0f;
		dFloat maxLimit =  4.0f;
		dFloat linearRate = 2.0f;
		vehicleModel->m_liftJoints[vehicleModel->m_liftActuatorsCount] = new dCustomSliderActuator(&matrix[0][0], linearRate, minLimit, maxLimit, boomBody, baseBone->m_body);
		vehicleModel->m_liftActuatorsCount++;
		dCustomTransformController::dSkeletonBone* const boomBone = controller->AddBone(boomBody, dGetIdentityMatrix(), baseBone);
		return boomBone;
*/
	}

	void AddCranekPaletteActuator(dCustomTransformController* const controller, dCustomTransformController::dSkeletonBone* const baseBone, const char* const name)
	{
		dAssert(0);
/*
		dCustomTransformController::dSkeletonBone* const chassisBone = controller->GetRoot();
		ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*)NewtonBodyGetUserData(chassisBone->m_body);

		DemoEntity* const palette = vehicleModel->Find(name);

		ARTICULATED_VEHICLE_DEFINITION definition;
		strcpy(definition.m_boneName, name);
		strcpy(definition.m_shapeTypeName, "convexHull");
		definition.m_mass = 10.0f;
		definition.m_bodyPartID = ARTICULATED_VEHICLE_DEFINITION::m_bodyPart;
		strcpy(definition.m_articulationName, name);
		NewtonBody* const paletteBody = CreateBodyPart(palette, definition);

		dMatrix matrix;
		NewtonBodyGetMatrix(paletteBody, &matrix[0][0]);
		matrix = dRollMatrix(0.5f * dPi) * matrix;

		dFloat minLimit = -0.01f;
		dFloat maxLimit =  1.25f;
		dFloat rate = 2.0f;

		vehicleModel->m_paletteJoints[vehicleModel->m_paletteActuatorsCount] = new dCustomSliderActuator(&matrix[0][0], rate, minLimit, maxLimit, paletteBody, baseBone->m_body);
		vehicleModel->m_paletteJoints[vehicleModel->m_paletteActuatorsCount]->SetMaxForcePower(10000.0f);
		vehicleModel->m_paletteActuatorsCount++;
		controller->AddBone(paletteBody, dGetIdentityMatrix(), baseBone);
*/
	}

	void AddCraneGripper(dCustomTransformController* const controller, dCustomTransformController::dSkeletonBone* const baseBone)
	{
		dAssert(0);
/*
		dCustomTransformController::dSkeletonBone* const chassisBone = controller->GetRoot();
		ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*)NewtonBodyGetUserData(chassisBone->m_body);

		DemoEntity* const gripper = vehicleModel->Find("effector");

		ARTICULATED_VEHICLE_DEFINITION definition;
		strcpy(definition.m_boneName, "effector");
		strcpy(definition.m_shapeTypeName, "convexHull");
		definition.m_mass = 10.0f;
		definition.m_bodyPartID = ARTICULATED_VEHICLE_DEFINITION::m_bodyPart;
		strcpy(definition.m_articulationName, "effector");
		NewtonBody* const gripperBody = CreateBodyPart(gripper, definition);

		dMatrix matrix;
		NewtonBodyGetMatrix(gripperBody, &matrix[0][0]);
		matrix = dPitchMatrix(0.5f * dPi) * matrix;
		matrix = dYawMatrix(0.5f * dPi) * matrix;

		dFloat minAngleLimit = -120.0f * dDegreeToRad;
		dFloat maxAngleLimit =  120.0f * dDegreeToRad;
		dFloat angularRate = 30.0f * dDegreeToRad;
		vehicleModel->m_gripperRotator = new dCustomDoubleHingeActuator(&matrix[0][0], angularRate, minAngleLimit, maxAngleLimit, angularRate, minAngleLimit, maxAngleLimit, gripperBody, baseBone->m_body);
		dCustomTransformController::dSkeletonBone* const gripperBone = controller->AddBone(gripperBody, dGetIdentityMatrix(), baseBone);
		AddCranekPaletteActuator (controller, gripperBone, "leftHand");
		AddCranekPaletteActuator (controller, gripperBone, "rightHand");
*/
	}

	void AddCraneLift(dCustomTransformController* const controller, dCustomTransformController::dSkeletonBone* const baseBone)
	{
		dAssert(0);
/*
		dCustomTransformController::dSkeletonBone* const chassisBone = controller->GetRoot();
		ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*)NewtonBodyGetUserData(chassisBone->m_body);

		DemoEntity* const boom = vehicleModel->Find("Boom1");

		ARTICULATED_VEHICLE_DEFINITION definition;
		strcpy(definition.m_boneName, "Boom1");
		strcpy(definition.m_shapeTypeName, "convexHull");
		definition.m_mass = 10.0f;
		definition.m_bodyPartID = ARTICULATED_VEHICLE_DEFINITION::m_bodyPart;
		strcpy(definition.m_articulationName, "Boom1");
		NewtonBody* const boomBody = CreateBodyPart(boom, definition);

		dMatrix matrix;
		NewtonBodyGetMatrix(boomBody, &matrix[0][0]);
		matrix = dRollMatrix(0.5f * dPi) * matrix;

		dFloat minAngleLimit = -60.0f * dDegreeToRad;
		dFloat maxAngleLimit =  10.0f * dDegreeToRad;
		dFloat angularRate = 40.0f * dDegreeToRad;

		vehicleModel->m_angularActuator0 = new dCustomHingeActuator(&matrix[0][0], angularRate, minAngleLimit, maxAngleLimit, boomBody, baseBone->m_body);
		dCustomTransformController::dSkeletonBone* const boomBone1 = controller->AddBone(boomBody, dGetIdentityMatrix(), baseBone);
		dCustomTransformController::dSkeletonBone* const boomBone2 = AddCraneBoom (controller, boomBone1, "Boom2");
		dCustomTransformController::dSkeletonBone* const boomBone3 = AddCraneBoom (controller, boomBone2, "Boom3");
		AddCraneGripper(controller, boomBone3);
*/
	}

	void AddCraneBase(dCustomTransformController* const controller)
	{
		dAssert(0);
/*
		dCustomTransformController::dSkeletonBone* const chassisBone = controller->GetRoot();
		ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*)NewtonBodyGetUserData(chassisBone->m_body);

		DemoEntity* const base = vehicleModel->Find("base");

		ARTICULATED_VEHICLE_DEFINITION definition;
		strcpy(definition.m_boneName, "base");
		strcpy(definition.m_shapeTypeName, "convexHull");
		definition.m_mass = 30.0f;
		definition.m_bodyPartID = ARTICULATED_VEHICLE_DEFINITION::m_bodyPart;
		strcpy(definition.m_articulationName, "base");
		NewtonBody* const baseBody = CreateBodyPart(base, definition);

		dMatrix matrix;
		NewtonBodyGetMatrix (baseBody, &matrix[0][0]);
		matrix = dRollMatrix (0.5f * dPi) * matrix;

		dFloat minAngleLimit = -1.0e10f;
		dFloat maxAngleLimit = 1.0e10f;
		dFloat angularRate = 40.0f * dDegreeToRad;
		vehicleModel->m_angularActuator1 = new dCustomHingeActuator(&matrix[0][0], angularRate, minAngleLimit, maxAngleLimit, baseBody, chassisBone->m_body);
		dCustomTransformController::dSkeletonBone* const baseBone = controller->AddBone(baseBody, dGetIdentityMatrix(), chassisBone);

		AddCraneLift(controller, baseBone);
*/
	}
#endif

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
		dExcavatorModel* const excavator = (dExcavatorModel*) model;
		excavator->OnDebug(debugContext);
	}

	virtual void OnPreUpdate(dModelRootNode* const model, dFloat timestep) const 
	{
		dExcavatorModel* const excavator = (dExcavatorModel*) model;
		excavator->CalculateTireContacts();
	}

	virtual void OnUpdateTransform(const dModelNode* const bone, const dMatrix& localMatrix) const
	{
		NewtonBody* const body = bone->GetBody();
		DemoEntity* const ent = (DemoEntity*)NewtonBodyGetUserData(body);
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(NewtonBodyGetWorld(body));

		dQuaternion rot(localMatrix);
		ent->SetMatrix(*scene, rot, localMatrix.m_posit);
	}

	static void StandardContactsProcess (const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
	{
		//dAssert (0);
#if 0
		int countCount = 0;
		void* contactList[32];

		for (void* contact = NewtonContactJointGetFirstContact(contactJoint); contact; contact = NewtonContactJointGetNextContact(contactJoint, contact)) {
			contactList[countCount] = contact;
			countCount++;
		}

		NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
		NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);

		//void* userData0 = NewtonCollisionGetUserData(NewtonBodyGetCollision(body0));
		//void* userData1 = NewtonCollisionGetUserData(NewtonBodyGetCollision(body1));

		for (int i = 0; i < countCount; i ++) {
			NewtonMaterial* const material = NewtonContactGetMaterial (contactList[i]);
			NewtonCollision* const collision0 = NewtonMaterialGetBodyCollidingShape(material, body0);
			NewtonCollision* const collision1 = NewtonMaterialGetBodyCollidingShape(material, body1);

			ARTICULATED_VEHICLE_DEFINITION::SHAPES_ID id0 = ARTICULATED_VEHICLE_DEFINITION::SHAPES_ID(NewtonCollisionGetUserID(collision0));
			ARTICULATED_VEHICLE_DEFINITION::SHAPES_ID id1 = ARTICULATED_VEHICLE_DEFINITION::SHAPES_ID(NewtonCollisionGetUserID(collision1));

			switch (id0 | id1) 
			{
				case ARTICULATED_VEHICLE_DEFINITION::m___linkPart  | ARTICULATED_VEHICLE_DEFINITION::m___tireInnerRing:
				{
					dAssert(0);
/*
					dMatrix tireMatrix;
					NewtonBody* tireBody = body0;
					if (id1 == ARTICULATED_VEHICLE_DEFINITION::m_tireInnerRing) {
						tireBody = body1;
					}
					NewtonBodyGetMatrix(tireBody, &tireMatrix[0][0]);
					dVector tireAxis(tireMatrix.m_up);
					for (void* contact = NewtonContactJointGetFirstContact(contactJoint); contact; contact = NewtonContactJointGetNextContact(contactJoint, contact)) {
						dVector posit;
						dVector normal;
						NewtonMaterial* const material = NewtonContactGetMaterial(contact);
						NewtonMaterialGetContactPositionAndNormal (material, tireBody, &posit[0], &normal[0]);
						dVector dir (normal.CrossProduct(tireAxis));
						NewtonMaterialContactRotateTangentDirections(material, &dir[0]);
						NewtonMaterialSetContactFrictionState (material, 0, 1);
					}
*/
					break;
				}
			}
		}


//dAssert (0);
/*
		if (linkBody && !tireBody) {
			// find the root body from the articulated structure 
			NewtonCollision* const linkCollision = NewtonBodyGetCollision(linkBody);
			const dCustomArticulatedTransformController::dSkeletonBone* const rootbone = (dCustomArticulatedTransformController::dSkeletonBone*)NewtonCollisionGetUserData(linkCollision);
			dCustomArticulatedTransformController* const controller = rootbone->m_controller;
			NewtonBody* const chassiBody = controller->GetBoneBody(rootbone);

			int countCount = 0;
			void* contactList[32];
			for (void* contact = NewtonContactJointGetFirstContact(contactJoint); contact; contact = NewtonContactJointGetNextContact(contactJoint, contact)) {
				contactList[countCount] = contact;
				countCount ++;
			}

			for (int i = 1; i < countCount; i++) {
				NewtonContactJointRemoveContact(contactJoint, contactList[i]);
			}

			dMatrix tireMatrix;
			dMatrix chassisMatrix;
			NewtonBodyGetMatrix(linkBody, &tireMatrix[0][0]);
			NewtonBodyGetMatrix(chassiBody, &chassisMatrix[0][0]);
			dVector upDir(chassisMatrix.RotateVector(dVector(0.0f, 1.0f, 0.0f, 0.0f)));
			dVector tireAxis(tireMatrix.RotateVector(dVector(1.0f, 0.0f, 0.0f, 0.0f)));
			dVector contactDirection(upDir.CrossProduct(tireAxis));

			NewtonMaterial* const material = NewtonContactGetMaterial(contactList[0]);
			NewtonMaterialContactRotateTangentDirections(material, &contactDirection[0]);
			NewtonMaterialSetContactFrictionCoef(material, 0.5f, 0.5f, 0);
			NewtonMaterialSetContactFrictionCoef(material, 1.0f, 1.0f, 1);

		} else if (tireBody) {

			// find the root body from the articulated structure 
			NewtonCollision* const tireCollsion = NewtonBodyGetCollision(tireBody);
			const dCustomArticulatedTransformController::dSkeletonBone* const bone = (dCustomArticulatedTransformController::dSkeletonBone*)NewtonCollisionGetUserData (tireCollsion);

			dCustomArticulatedTransformController* const controller = bone->m_controller;
			const dCustomArticulatedTransformController::dSkeletonBone* const rootbone = controller->GetParent(bone);
			NewtonBody* const chassiBody = controller->GetBoneBody(rootbone);

			// Get the root and tire matrices
			dMatrix tireMatrix;
			dMatrix chassisMatrix;
			NewtonBodyGetMatrix(tireBody, &tireMatrix[0][0]);
			NewtonBodyGetMatrix(chassiBody, &chassisMatrix[0][0]);

			dVector upDir (chassisMatrix.RotateVector(dVector (0.0f, 1.0f, 0.0f, 0.0f)));
			dVector tireAxis (tireMatrix.RotateVector(dVector (1.0f, 0.0f, 0.0f, 0.0f)));

			dVector contactDirection (upDir.CrossProduct(tireAxis));
			for (void* contact = NewtonContactJointGetFirstContact (contactJoint); contact; contact = NewtonContactJointGetNextContact (contactJoint, contact)) {
				NewtonMaterial* const material = NewtonContactGetMaterial (contact);
				NewtonMaterialContactRotateTangentDirections (material, &contactDirection[0]);
				NewtonMaterialSetContactFrictionCoef (material, 1.0f, 1.0f, 0);
				NewtonMaterialSetContactFrictionCoef (material, 1.0f, 1.0f, 1);
			}
		}
*/
#endif
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

		//m_terrain = 1 << 0,
		//m_bodyPart = 1 << 1,
		//m_tirePart = 1 << 2,
		//m_linkPart = 1 << 3,
		//m_propBody = 1 << 4,
	
		const dLong mask0 = material0.m_userId & material1.m_userParam[0].m_int;
		const dLong mask1 = material1.m_userId & material0.m_userParam[0].m_int;
		return (mask0 && mask1) ? 1 : 0;
	}

	static int SpecialAABBOverlapTest(const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
	{
		const NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
		const NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);
		const NewtonCollision* const collision0 = NewtonBodyGetCollision(body0);
		const NewtonCollision* const collision1 = NewtonBodyGetCollision(body1);

		dAssert (NewtonBodyGetMaterialGroupID(body0) == NewtonBodyGetMaterialGroupID(body1));
		dAssert (NewtonBodyGetMaterialGroupID(body0) != NewtonMaterialGetDefaultGroupID(NewtonBodyGetWorld(body0)));
		dAssert (NewtonBodyGetMaterialGroupID(body1) != NewtonMaterialGetDefaultGroupID(NewtonBodyGetWorld(body1)));
		dAssert (NewtonCollisionGetUserID(collision0) & (ARTICULATED_VEHICLE_DEFINITION::m_linkPart | ARTICULATED_VEHICLE_DEFINITION::m_terrain));
		dAssert (NewtonCollisionGetUserID(collision1) & (ARTICULATED_VEHICLE_DEFINITION::m_linkPart | ARTICULATED_VEHICLE_DEFINITION::m_terrain));

		dExcavatorModel* const excavator = (dExcavatorModel*)NewtonCollisionGetUserData(collision0);
		dAssert (excavator);
		return excavator->OnBoneAABBOverlap(contactJoint);
	}

	static int SpecialGenerateLinkGroundContacts (const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonCollision* const collision0, const NewtonBody* const body1, const NewtonCollision* const collision1, NewtonUserContactPoint* const contactBuffer, int maxCount, int threadIndex)
	{
		dExcavatorModel* const excavator = (dExcavatorModel*)NewtonCollisionGetUserData(collision0);
		dAssert(excavator);
		return excavator->SpecialGenerateLinkGroundContact (material, body0, body1, contactBuffer);
	}

	static void SpecialContactsProcess(const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
	{
		//dAssert(0);
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
