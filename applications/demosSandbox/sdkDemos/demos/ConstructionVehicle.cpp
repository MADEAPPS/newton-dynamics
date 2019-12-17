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
#include "dWoodFracture.h"

#include "DebugDisplay.h"
#include "HeightFieldPrimitive.h"

#define ARTICULATED_VEHICLE_CAMERA_HIGH_ABOVE_HEAD		3.0f
#define ARTICULATED_VEHICLE_CAMERA_DISTANCE				11.0f
#define EXCAVATOR_TREAD_THICKNESS						0.13f

#define EXCAVATOR_GEAR_GAIN								5.0f
#define EXCAVATOR_ENGINE_RPM							40.0f
#define EXCAVATOR_STEERING_RPM							25.0f
#define EXCAVATOR_ENGINE_TORQUE							5000.0f
#define EXCAVATOR_STEERING_TORQUE						5000.0f

#define EXCAVATOR_SIMULATE_TRACKS

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

class dExcavatorControls
{
	public:
	dExcavatorControls()
	{
		memset (this, 0, sizeof (dExcavatorControls));
	}
	dFloat m_throttle;
	dFloat m_steeringValue;
	dFloat m_bucket_x;
	dFloat m_bucket_y;
	dFloat m_bucket_angle;
	int m_cabinSpeed;
};

class dExcavatorEngine: public dCustomDoubleHinge
{
	public:

	dExcavatorEngine(const dMatrix& pinAndPivotFrame, NewtonBody* const engine, NewtonBody* const chassis)
		:dCustomDoubleHinge(pinAndPivotFrame, engine, chassis)
		,m_control()
	{
		EnableLimits(false);
		EnableLimits1(false);
	}

	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		dMatrix matrix1;
		NewtonBodyGetMatrix(m_body1, &matrix1[0][0]);

		dMatrix matrix0 (GetMatrix0().Inverse() * GetMatrix1() * matrix1);
		NewtonBodySetMatrixNoSleep(m_body0, &matrix0[0][0]);
		dCustomDoubleHinge::SubmitConstraints(timestep, threadIndex);
	}

	void SubmitAngularRow(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
	{
		dCustomDoubleHinge::SubmitAngularRow(matrix0, matrix1, timestep);

		const dVector& tractionDir = matrix0.m_up;
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &tractionDir[0]);
		dFloat accel = NewtonUserJointCalculateRowZeroAcceleration(m_joint) - EXCAVATOR_ENGINE_RPM * m_control.m_throttle / timestep;
		NewtonUserJointSetRowAcceleration(m_joint, accel);
		NewtonUserJointSetRowMinimumFriction(m_joint, -EXCAVATOR_ENGINE_TORQUE);
		NewtonUserJointSetRowMaximumFriction(m_joint, EXCAVATOR_ENGINE_TORQUE);

		const dVector& steeringDir = matrix0.m_front;
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &steeringDir[0]);
		accel = NewtonUserJointCalculateRowZeroAcceleration(m_joint) - EXCAVATOR_STEERING_RPM * m_control.m_steeringValue / timestep;
		NewtonUserJointSetRowAcceleration(m_joint, accel);
		NewtonUserJointSetRowMinimumFriction(m_joint, -EXCAVATOR_STEERING_TORQUE);
		NewtonUserJointSetRowMaximumFriction(m_joint, EXCAVATOR_STEERING_TORQUE);
	}

	dExcavatorControls m_control;
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

	dExcavatorModel(NewtonWorld* const world, const char* const modelName, const dMatrix& location, int linkMaterilID)
		:dModelRootNode(NULL, dGetIdentityMatrix())
		,m_effectorMatrix(dGetIdentityMatrix())
		,m_shereCast (NewtonCreateSphere (world, 0.20f, 0, NULL))
		,m_engineJoint(NULL)
		,m_bucketJoint(NULL)
		,m_effector(NULL)
		,m_cabinAngle(0.0f)
		,m_bucketAngle(0.0f)
	{
		MakeChassis(world, modelName, location);
		AddLocomotion();
		MakeCabinAndUpperBody();

		MakeLeftTrack();
		MakeRightTrack();
#ifdef EXCAVATOR_SIMULATE_TRACKS
		MakeThread("leftThread", linkMaterilID);
		MakeThread("rightThread", linkMaterilID);
#endif		
	}

	~dExcavatorModel()
	{
		NewtonDestroyCollision(m_shereCast);
	}

	void ApplyControls(const dExcavatorControls& control, dFloat timestep)
	{
		if (m_engineJoint->m_control.m_throttle || 
			m_engineJoint->m_control.m_steeringValue ||
			(m_engineJoint->m_control.m_bucket_x != control.m_bucket_x) ||
			(m_engineJoint->m_control.m_bucket_y != control.m_bucket_y) ||
			(m_engineJoint->m_control.m_bucket_angle != control.m_bucket_angle) ||
			m_engineJoint->m_control.m_cabinSpeed) {

			NewtonBodySetSleepState(GetBody(), 0);

			m_bucketJoint->SetTargetAngle(control.m_bucket_angle * dDegreeToRad);

			dMatrix cabinMatrix (m_effectorMatrix);
			cabinMatrix.m_posit.m_x += m_engineJoint->m_control.m_bucket_x * 0.5f;
			cabinMatrix.m_posit.m_y += m_engineJoint->m_control.m_bucket_y * 0.5f;

			dFloat speed = -10.0f * m_engineJoint->m_control.m_cabinSpeed * dDegreeToRad;
			m_cabinAngle = dMod ( dFloat(m_cabinAngle + speed * timestep), dFloat(2.0f * dPi));
			dMatrix rotation = dYawMatrix(m_cabinAngle);
			
			cabinMatrix.m_posit = rotation.RotateVector(cabinMatrix.m_posit);
			cabinMatrix.m_posit.m_w = 1.0f;
			m_effector->SetTargetMatrix (cabinMatrix);
		}
		m_engineJoint->m_control = control;
	}

	virtual void OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
	{
		int stack = 1;
		dModelNode* pool[32];
		pool[0] = this;
		while (stack) {
			stack--;
			dModelNode* const node = pool[stack];
			NewtonBody* const body = node->GetBody();
			dAssert(body);
			NewtonCollision* const collision = NewtonBodyGetCollision(body);
			dLong id = NewtonCollisionGetUserID(collision);
			if (id == ARTICULATED_VEHICLE_DEFINITION::m_linkPart) {
				dMatrix matrix (BuildCollisioMatrix(body));
				debugContext->DrawFrame(matrix);
				debugContext->SetColor(dVector(0.7f, 0.4f, 0.0f, 1.0f));
				NewtonCollisionForEachPolygonDo(m_shereCast, &matrix[0][0], RenderDebugTire, debugContext);
			}

			for (dModelChildrenList::dListNode* ptrNode = node->GetChildren().GetFirst(); ptrNode; ptrNode = ptrNode->GetNext()) {
				pool[stack] = ptrNode->GetInfo();
				stack++;
			}
		}
	}

	int CollideLink(const NewtonMaterial* const material, const NewtonBody* const linkBody, const NewtonBody* const staticBody, NewtonUserContactPoint* const contactBuffer)
	{
		dMatrix staticMatrix;
		dMatrix linkMatrix(BuildCollisioMatrix(linkBody));
		const int maxContactCount = 2;
		dFloat points[maxContactCount][3];
		dFloat normals[maxContactCount][3];
		dFloat penetrations[maxContactCount];
		dLong attributeA[maxContactCount];
		dLong attributeB[maxContactCount];

		NewtonWorld* const world = NewtonBodyGetWorld(GetBody());
		NewtonBodyGetMatrix(staticBody, &staticMatrix[0][0]);
		NewtonCollision* const staticShape = NewtonBodyGetCollision(staticBody);

		int count = NewtonCollisionCollide(world, 1,
				m_shereCast, &linkMatrix[0][0],
				staticShape, &staticMatrix[0][0],
				&points[0][0], &normals[0][0], penetrations, attributeA, attributeB, 0);
		if (count) {
			contactBuffer->m_point[0] = points[0][0];
			contactBuffer->m_point[1] = points[0][1];
			contactBuffer->m_point[2] = points[0][2];
			contactBuffer->m_point[3] = 1.0f;
			contactBuffer->m_normal[0] = normals[0][0];
			contactBuffer->m_normal[1] = normals[0][1];
			contactBuffer->m_normal[2] = normals[0][2];
			contactBuffer->m_normal[3] = 0.0f;
			contactBuffer->m_penetration = penetrations[0];
			contactBuffer->m_shapeId0 = 0;
			contactBuffer->m_shapeId1 = 0;
		}
		return count;
	}

	private:
	dMatrix BuildCollisioMatrix(const NewtonBody* const threadLink) const
	{
		dMatrix matrix;
		dVector com;
		NewtonBodyGetCentreOfMass(threadLink, &com[0]);
		NewtonBodyGetMatrix(threadLink, &matrix[0][0]);
		matrix.m_posit = matrix.TransformVector(com);
		matrix.m_posit -= matrix.m_front.Scale(0.12f);
		return matrix;
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

	NewtonCollision* const MakeTireShape(DemoEntity* const tireModel)
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
		return bone;
	}

	dModelNode* MakeRollerTire(const char* const entName)
	{
		NewtonBody* const parentBody = GetBody();
		DemoEntity* const parentModel = (DemoEntity*)NewtonBodyGetUserData(parentBody);
		DemoEntity* const tireModel = parentModel->Find(entName);

		NewtonCollision* const tireCollision = MakeTireShape(tireModel);
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
		LinkTires (leftTire_0, leftTire_7);
		MakeRollerTire("leftSupportRoller");

		for (int i = 0; i < 3; i++) {
			char name[64];
			sprintf(name, "leftRoller%d", i);
			dModelNode* const rollerTire = MakeRollerTire(name);
			LinkTires (leftTire_0, rollerTire);
		}

		// link traction tire to the engine using a differential gear
		dMatrix engineMatrix;
		dMatrix chassisMatrix;

		NewtonBody* const tire = leftTire_0->GetBody();
		NewtonBody* const engine = m_engineJoint->GetBody0();
		m_engineJoint->CalculateGlobalMatrix(engineMatrix, chassisMatrix);

		dMatrix tireMatrix;
		NewtonBodyGetMatrix(tire, &tireMatrix[0][0]);
		new dCustomDifferentialGear___(EXCAVATOR_GEAR_GAIN, engineMatrix.m_front.Scale (-1.0f), engineMatrix.m_up, tireMatrix.m_right.Scale(1.0f), engine, tire);
	}

	void MakeRightTrack()
	{
		dModelNode* const rightTire_0 = MakeRollerTire("rightGear");
		dModelNode* const rightTire_7 = MakeRollerTire("rightFrontRoller");
		LinkTires(rightTire_0, rightTire_7);
		MakeRollerTire("rightSupportRoller");

		for (int i = 0; i < 3; i++) {
			char name[64];
			sprintf(name, "rightRoller%d", i);
			dModelNode* const rollerTire = MakeRollerTire(name);
			LinkTires(rightTire_0, rollerTire);
		}

		// link traction tire to the engine using a differential gear
		dMatrix engineMatrix;
		dMatrix chassisMatrix;

		NewtonBody* const tire = rightTire_0->GetBody();
		NewtonBody* const engine = m_engineJoint->GetBody0();
		m_engineJoint->CalculateGlobalMatrix(engineMatrix, chassisMatrix);

		dMatrix tireMatrix;
		NewtonBodyGetMatrix(tire, &tireMatrix[0][0]);
		new dCustomDifferentialGear___(EXCAVATOR_GEAR_GAIN, engineMatrix.m_front.Scale (1.0f), engineMatrix.m_up, tireMatrix.m_right.Scale(-1.0f), engine, tire);
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

	NewtonBody* MakeBodyPart(dModelNode* const parentNode, const char* const name, dFloat mass)
	{
		NewtonBody* const parentBody = parentNode->GetBody();
		NewtonWorld* const world = NewtonBodyGetWorld(GetBody());
		DemoEntity* const parentModel = (DemoEntity*)NewtonBodyGetUserData(parentBody);
		DemoEntity* const bodyPart = parentModel->Find(name);

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
		NewtonBodySetMassProperties(body, mass, collision);

		// save the user data with the bone body (usually the visual geometry)
		NewtonBodySetUserData(body, bodyPart);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);

		return body;
	}

	void MakeCabinAndUpperBody ()
	{
		// add cabin and engine upper body
		NewtonBody* const parentBody = GetBody();
		NewtonBody* const cabinBody = MakeBodyPart(this, "EngineBody", 400.0f);

		// connect the part to the main body with a hinge
		dMatrix hingeFrame;
		NewtonBodyGetMatrix(cabinBody, &hingeFrame[0][0]);
		new dCustomHinge(hingeFrame, cabinBody, parentBody);

		// set the center of mass of engine
		dVector com(hingeFrame.UnrotateVector(dVector(-2.0f, 0.0f, 0.0f, 0.0f)));
		NewtonBodySetCentreOfMass(cabinBody, &com[0]);

		//dMatrix bindMatrix(bodyPart->GetParent()->CalculateGlobalMatrix(parentModel).Inverse());
		dMatrix bindMatrix(dGetIdentityMatrix());
		dModelNode* const cabinNode = new dModelNode(cabinBody, bindMatrix, this);

		// add boom
		NewtonBody* const boomBody = MakeBodyPart(cabinNode, "Boom", 50.0f);
		NewtonBodyGetMatrix(boomBody, &hingeFrame[0][0]);
		hingeFrame = dYawMatrix(90.0f * dDegreeToRad) * hingeFrame;
		new dCustomHinge(hingeFrame, boomBody, cabinBody);
		dModelNode* const boomNode = new dModelNode(boomBody, bindMatrix, cabinNode);

		// add Arm
		NewtonBody* const armBody = MakeBodyPart(cabinNode, "arm02", 50.0f);
		NewtonBodyGetMatrix(armBody, &hingeFrame[0][0]);
		hingeFrame = dRollMatrix(90.0f * dDegreeToRad) * hingeFrame;
		new dCustomHinge(hingeFrame, armBody, boomBody);
		dModelNode* const armNode = new dModelNode(armBody, bindMatrix, boomNode);

		// add bucket
		NewtonBody* const bucketBody = MakeBodyPart(armNode, "bucket", 40.0f);
		NewtonBodyGetMatrix(bucketBody, &hingeFrame[0][0]);
		hingeFrame = dRollMatrix(90.0f * dDegreeToRad) * hingeFrame;
		m_bucketJoint = new dCustomHingeActuator(hingeFrame, bucketBody, armBody);
		m_bucketJoint->SetAngularRate(0.5f);
		new dModelNode(bucketBody, bindMatrix, armNode);

		// create effector to control bucket
		NewtonBody* const effectorReferenceBody = GetBody();
		m_effector = new dCustomKinematicController(armBody, hingeFrame, effectorReferenceBody);
		m_effector->SetSolverModel(1);
		m_effector->SetControlMode(dCustomKinematicController::m_linear);
		m_effector->SetMaxLinearFriction(2000.0f * 9.8f * 50.0f);
		m_effectorMatrix = m_effector->GetTargetMatrix ();
	}

	void AddLocomotion()
	{
		NewtonBody* const chassis = GetBody();
		NewtonWorld* const world = NewtonBodyGetWorld(GetBody());
		NewtonCollision* const shape = NewtonCreateCylinder (world, 0.125f, 0.125f, 0.75f, 0, NULL);

		// create the rigid body that will make this bone
		dMatrix engineMatrix;
		NewtonBodyGetMatrix(chassis, &engineMatrix[0][0]);
		engineMatrix = dRollMatrix(0.5f * dPi) * engineMatrix;
		engineMatrix.m_posit.m_y += 1.0f;

		// make a non collideble engine body
		NewtonBody* const engineBody = NewtonCreateDynamicBody(world, shape, &engineMatrix[0][0]);

		// destroy the collision helper shape
		NewtonDestroyCollision(shape);

		// get the collision from body
		NewtonCollision* const collision = NewtonBodyGetCollision(engineBody);
		NewtonCollisionSetMode (collision, 0);

		// calculate the moment of inertia and the relative center of mass of the solid
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

		// add the engine joint 
		m_engineJoint = new dExcavatorEngine(engineAxis, engineBody, chassis);

		// connect engine to chassis.
		dMatrix bindMatrix(dGetIdentityMatrix());
		new dModelNode(engineBody, bindMatrix, this);
	}

	NewtonBody* CreateChassisBody(NewtonWorld* const world, DemoEntity* const bodyPart, dFloat mass)
	{
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

	void MakeChassis(NewtonWorld* const world, const char* const modelName, const dMatrix& location)
	{
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		// make a clone of the mesh 
		DemoEntity* const vehicleModel = DemoEntity::LoadNGD_mesh(modelName, world, scene->GetShaderCache());
		scene->Append(vehicleModel);

		// place the model at its location
		dMatrix matrix(vehicleModel->GetCurrentMatrix());
		matrix.m_posit = location.m_posit;
		vehicleModel->ResetMatrix(*scene, matrix);

		DemoEntity* const rootEntity = (DemoEntity*)vehicleModel->Find("base");
		m_body = CreateChassisBody(world, rootEntity, 4000.0f);
	}

	dMatrix m_effectorMatrix;
	NewtonCollision* m_shereCast;
	dExcavatorEngine* m_engineJoint;
	dCustomHingeActuator* m_bucketJoint;
	dCustomKinematicController* m_effector;
	dFloat m_cabinAngle;
	dFloat m_bucketAngle;
};

class ArticulatedVehicleManagerManager: public dModelManager
{
	public:
	ArticulatedVehicleManagerManager (DemoEntityManager* const scene, int threadMaterialID)
		:dModelManager (scene->GetNewton())
		,m_player(NULL)
		,m_threadMaterialID(threadMaterialID)
		,m_cabinSpeed(0)
		,m_bucket_x(0.0f)
		,m_bucket_y(0.0f)
		,m_bucket_angle(0.0f)
	{
		scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
		scene->Set2DDisplayRenderFunction(RenderPlayerHelp, NULL, this);

		// create a material for early collision culling
		NewtonWorld* const world = scene->GetNewton();
		int material = NewtonMaterialGetDefaultGroupID (world);

		NewtonMaterialSetCallbackUserData (world, material, material, this);
		NewtonMaterialSetCollisionCallback (world, material, material, StandardAABBOverlapTest, NULL);
		NewtonMaterialSetDefaultElasticity(world, material, material, 0.1f);
		NewtonMaterialSetDefaultFriction(world, material, material, 0.9f, 0.9f);

		NewtonMaterialSetCallbackUserData(world, material, threadMaterialID, this);
		NewtonMaterialSetCollisionCallback (world, material, threadMaterialID, StandardAABBOverlapTest, NULL);
		NewtonMaterialSetDefaultElasticity(world, material, threadMaterialID, 0.1f);
		NewtonMaterialSetDefaultFriction(world, material, threadMaterialID, 0.9f, 0.9f);

		NewtonMaterialSetCallbackUserData(world, threadMaterialID, threadMaterialID, this);
		NewtonMaterialSetCollisionCallback (world, threadMaterialID, threadMaterialID, StandardAABBOverlapTest, NULL);
		NewtonMaterialSetContactGenerationCallback (world, threadMaterialID, threadMaterialID, ThreadStaticContactsGeneration);
		NewtonMaterialSetDefaultElasticity(world, threadMaterialID, threadMaterialID, 0.1f);
		NewtonMaterialSetDefaultFriction(world, threadMaterialID, threadMaterialID, 0.9f, 0.9f);
	}

	static void UpdateCameraCallback(DemoEntityManager* const manager, void* const context, dFloat timestep)
	{
		ArticulatedVehicleManagerManager* const me = (ArticulatedVehicleManagerManager*)context;
		me->UpdateCamera(timestep);
	}

	static void RenderPlayerHelp(DemoEntityManager* const scene, void* const context)
	{
		ArticulatedVehicleManagerManager* const me = (ArticulatedVehicleManagerManager*)context;
		me->RenderPlayerHelp(scene);
	}

	void RenderPlayerHelp(DemoEntityManager* const scene)
	{
		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print(color, "Navigation Keys");
		scene->Print(color, "drive forward:      w");
		scene->Print(color, "drive backward:     s");
		scene->Print(color, "turn right:         d");
		scene->Print(color, "turn left:          a");

		scene->Print(color, "bucket controls");
		ImGui::SliderInt("cabin rotation", &m_cabinSpeed, -3, 3);
		ImGui::SliderFloat("bucket x", &m_bucket_x, -2.0f, 4.0f);
		ImGui::SliderFloat("bucket y", &m_bucket_y, -6.0f, 6.0f);
		ImGui::SliderFloat("bucket angle", &m_bucket_angle, -60.0f, 130.0f);
	}

	void UpdateCamera(dFloat timestep)
	{
		//return;
		if (!m_player) {
			return;
		}

		DemoEntity* const player = (DemoEntity*)NewtonBodyGetUserData(m_player->GetBody());
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(GetWorld());
		DemoCamera* const camera = scene->GetCamera();
		dMatrix camMatrix(camera->GetNextMatrix());
		dMatrix playerMatrix(player->GetNextMatrix());

		dVector frontDir(camMatrix[0]);
		dVector camOrigin (playerMatrix.m_posit + dVector(0.0f, ARTICULATED_VEHICLE_CAMERA_HIGH_ABOVE_HEAD, 0.0f, 0.0f));
		camOrigin -= frontDir.Scale(ARTICULATED_VEHICLE_CAMERA_DISTANCE);
		camera->SetNextMatrix(*scene, camMatrix, camOrigin);
	}

	dModelRootNode* CreateExcavator (const char* const modelName, const dMatrix& location)
	{
		dExcavatorModel* const controller = new dExcavatorModel(GetWorld(), modelName, location, m_threadMaterialID);

		// the the model to calculate the local transformation
		controller->SetTransformMode(true);

		// add the model to the manager
		AddRoot(controller);

		m_player = controller;
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

		NewtonWorld* const world = NewtonBodyGetWorld(excavator->GetBody());
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		dExcavatorControls controls;
		controls.m_throttle = (dFloat(scene->GetKeyState('W')) - dFloat(scene->GetKeyState('S')));
		controls.m_steeringValue = (dFloat(scene->GetKeyState('A')) - dFloat(scene->GetKeyState('D')));
		controls.m_cabinSpeed = m_cabinSpeed;
		controls.m_bucket_x = m_bucket_x;
		controls.m_bucket_y = m_bucket_y;
		controls.m_bucket_angle = m_bucket_angle;

		excavator->ApplyControls(controls, timestep);
	}

	virtual void OnUpdateTransform(const dModelNode* const bone, const dMatrix& localMatrix) const
	{
		NewtonBody* const body = bone->GetBody();
		DemoEntity* const ent = (DemoEntity*)NewtonBodyGetUserData(body);
		if (ent) {
			DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(NewtonBodyGetWorld(body));

			dQuaternion rot(localMatrix);
			ent->SetMatrix(*scene, rot, localMatrix.m_posit);
		}
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
		dAssert(NewtonBodyGetMaterialGroupID(body0) == NewtonBodyGetMaterialGroupID(body1));
		dAssert(NewtonBodyGetMaterialGroupID(body0) != NewtonMaterialGetDefaultGroupID(NewtonBodyGetWorld(body0)));
		dAssert(NewtonBodyGetMaterialGroupID(body1) != NewtonMaterialGetDefaultGroupID(NewtonBodyGetWorld(body1)));

		dAssert(NewtonCollisionGetUserID(collision0) == ARTICULATED_VEHICLE_DEFINITION::m_linkPart);
		dAssert(NewtonCollisionGetUserID(collision1) == ARTICULATED_VEHICLE_DEFINITION::m_terrain);

		dExcavatorModel* const excavator = (dExcavatorModel*)NewtonCollisionGetUserData(collision0);
		dAssert (excavator);
		return excavator->CollideLink(material, body0, body1, contactBuffer);
	}

	dExcavatorModel* m_player;
	int m_threadMaterialID;
	int m_cabinSpeed;
	dFloat32 m_bucket_x;
	dFloat32 m_bucket_y;
	dFloat32 m_bucket_angle;
};

void ConstructionVehicle (DemoEntityManager* const scene)
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

	//  create a skeletal transform controller for controlling rag doll
	ArticulatedVehicleManagerManager* const vehicleManager = new ArticulatedVehicleManagerManager (scene, specialThreadId);

	NewtonWorld* const world = scene->GetNewton();
	dVector origin (FindFloor (world, dVector (-10.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));

	dMatrix matrix (dGetIdentityMatrix());
	matrix.m_posit = FindFloor (world, origin, 100.0f);
	matrix.m_posit.m_y += 1.5f;
	vehicleManager->CreateExcavator("excavator.ngd", matrix);

	matrix.m_posit.m_x += 10.0f;
	matrix.m_posit.m_y =   0.0f;
	matrix.m_posit.m_z +=  0.0f;
	// add some object to play with

#if 1
	LoadLumberYardMesh(scene, matrix.m_posit + dVector(6.0f, 0.0f, 0.0f, 0.0f), ARTICULATED_VEHICLE_DEFINITION::m_propBody);
	LoadLumberYardMesh(scene, dVector(6.0f, 0.0f, 10.0f, 0.0f), ARTICULATED_VEHICLE_DEFINITION::m_propBody);
	LoadLumberYardMesh(scene, dVector(10.0f, 0.0f, -5.0f, 0.0f), ARTICULATED_VEHICLE_DEFINITION::m_propBody);
	LoadLumberYardMesh(scene, dVector(10.0f, 0.0f,  5.0f, 0.0f), ARTICULATED_VEHICLE_DEFINITION::m_propBody);
	LoadLumberYardMesh(scene, dVector(14.0f, 0.0f, 0.0f, 0.0f), ARTICULATED_VEHICLE_DEFINITION::m_propBody);
	LoadLumberYardMesh(scene, dVector(14.0f, 0.0f, 10.0f, 0.0f), ARTICULATED_VEHICLE_DEFINITION::m_propBody);
	LoadLumberYardMesh(scene, dVector(18.0f, 0.0f, -5.0f, 0.0f), ARTICULATED_VEHICLE_DEFINITION::m_propBody);
	LoadLumberYardMesh(scene, dVector(18.0f, 0.0f,  5.0f, 0.0f), ARTICULATED_VEHICLE_DEFINITION::m_propBody);

	dMatrix shapeOffsetMatrix(dGetIdentityMatrix());
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID(scene->GetNewton());

	int woodX = 8;
	int woodZ = 8;
	matrix.m_posit.m_z += 10.0f;
	matrix.m_posit.m_x -= 5.0f;

	AddFracturedWoodPrimitive(scene, 1000.0f, matrix.m_posit, dVector(0.3f, 3.0f, 0.3f, 0.0f), 
		woodX, woodZ, 0.5f, ARTICULATED_VEHICLE_DEFINITION::m_propBody, defaultMaterialID, shapeOffsetMatrix);
#endif

	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		NewtonCollision* const collision = NewtonBodyGetCollision(body);
		if (NewtonCollisionGetUserID(collision) == ARTICULATED_VEHICLE_DEFINITION::m_propBody) {
			NewtonBodySetGyroscopicTorque(body, 1);
			NewtonCollisionMaterial material;
			NewtonCollisionGetMaterial(collision, &material);
			material.m_userParam[0].m_int = -1;
			NewtonCollisionSetMaterial(collision, &material);
		}
	}

	origin.m_x -= 15.0f;
	origin.m_y += 5.0f;
	dQuaternion rot (dVector (0.0f, 1.0f, 0.0f, 0.0f), -30.0f * dDegreeToRad);  
	scene->SetCameraMatrix(rot, origin);
}
