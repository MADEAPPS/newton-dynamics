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

#include "toolbox_stdafx.h"
#include "SkyBox.h"
#include "DemoMesh.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "TargaToOpenGl.h"
#include "DemoEntityManager.h"
#include "DebugDisplay.h"
#include "HeightFieldPrimitive.h"

struct dPasiveRagDollDefinition
{
	struct dPasiveRagDollJointLimitx
	{
		dFloat m_minTwistAngle;
		dFloat m_maxTwistAngle;
		dFloat m_coneAngle;
	};

	struct dPasiveRagDollFrameMatrix
	{
		dFloat m_pitch;
		dFloat m_yaw;
		dFloat m_roll;
	};

	char m_boneName[32];
	dFloat m_friction;
	dPasiveRagDollJointLimitx m_jointLimits;
	dPasiveRagDollFrameMatrix m_frameBasics;
};

static dPasiveRagDollDefinition skeletonRagDoll[] =
{
	{ "mixamorig:Hips"},
	{ "mixamorig:Spine", 100.0f, {-15.0f, 15.0f, 30.0f}, {0.0f, 0.0f, 180.0f}},
	{ "mixamorig:Spine1", 100.0f, {-15.0f, 15.0f, 30.0f}, {0.0f, 0.0f, 180.0f}},
	{ "mixamorig:Spine2", 100.0f, {-15.0f, 15.0f, 30.0f}, {0.0f, 0.0f, 180.0f}},

	{ "mixamorig:Neck", 100.0f, {-15.0f, 15.0f, 30.0f}, {0.0f, 0.0f, 180.0f}},

	{ "mixamorig:LeftArm", 100.0f, {-45.0f, 45.0f, 120.0f}, {0.0f, 0.0f, 180.0f}},
	{ "mixamorig:LeftForeArm", 50.0f, {-140.0f, 10.0f, 0.0f}, {0.0f, 0.0f, -90.0f}},
	
	{ "mixamorig:RightArm", 100.0f, {-45.0f, 45.0f, 120.0f}, {0.0f, 0.0f, 180.0f}},
	{ "mixamorig:RightForeArm", 50.0f, {-140.0f, 10.0f, 0.0f}, {0.0f, 00.0f, 90.0f}},

	{ "mixamorig:LeftUpLeg", 100.0f, {-45.0f, 45.0f, 120.0f}, {0.0f, 0.0f, 180.0f}},
	{ "mixamorig:LeftLeg", 50.0f, {-140.0f, 10.0f, 0.0f}, {0.0f, 90.0f, 90.0f}},

	{ "mixamorig:RightUpLeg", 100.0f, {-45.0f, 45.0f, 120.0f}, {0.0f, 0.0f, 180.0f}},
	{ "mixamorig:RightLeg", 50.0f, {-140.0f, 10.0f, 0.0f}, {0.0f, 90.0f, 90.0f}},
};


class PassiveRagdollManager: public dCustomTransformManager
{
	public: 
	PassiveRagdollManager (DemoEntityManager* const scene)
		:dCustomTransformManager (scene->GetNewton())
	{
		// create a material for early collision culling
		m_material = NewtonMaterialCreateGroupID(scene->GetNewton());
		NewtonMaterialSetCallbackUserData (scene->GetNewton(), m_material, m_material, this);
		NewtonMaterialSetCollisionCallback (scene->GetNewton(), m_material, m_material, OnBoneAABBOverlap, NULL);
	}

	virtual void OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
	{
	}

	virtual void OnPreUpdate (dCustomTransformController* const controller, dFloat timestep, int threadIndex) const
	{
	}

	static int OnBoneAABBOverlap(const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
	{
//		const NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
//		const NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);
//		const NewtonCollision* const collision0 = NewtonBodyGetCollision(body0);
//		const NewtonCollision* const collision1 = NewtonBodyGetCollision(body1);
//		const dCustomTransformController::dSkeletonBone* const bone0 = (dCustomTransformController::dSkeletonBone*)NewtonCollisionGetUserData (collision0);
//		const dCustomTransformController::dSkeletonBone* const bone1 = (dCustomTransformController::dSkeletonBone*)NewtonCollisionGetUserData (collision1);
//
//		dAssert (bone0);
//		dAssert (bone1);
////		if (bone0->m_controller && bone1->m_controller) {
////			return bone0->m_controller->SelfCollisionTest (bone0, bone1) ? 1 : 0;
////		}
		return 1;
	}

	void GetDimentions(DemoEntity* const bodyPart, dVector& origin, dVector& size) const
	{	
		DemoMesh* const mesh = (DemoMesh*)bodyPart->GetMesh();
		dAssert (mesh->IsType(DemoMesh::GetRttiType()));

		dFloat* const array = mesh->m_vertex;
		dVector pmin( 1.0e20f,  1.0e20f,  1.0e20f, 0.0f);
		dVector pmax(-1.0e20f, -1.0e20f, -1.0e20f, 0.0f);

		for (int i = 0; i < mesh->m_vertexCount; i ++) {
			dFloat x = array[i * 3 + 0];
			dFloat y = array[i * 3 + 1];
			dFloat z = array[i * 3 + 2];

			pmin.m_x = x < pmin.m_x ? x : pmin.m_x;
			pmin.m_y = y < pmin.m_y ? y : pmin.m_y;
			pmin.m_z = z < pmin.m_z ? z : pmin.m_z;
									  
			pmax.m_x = x > pmax.m_x ? x : pmax.m_x;
			pmax.m_y = y > pmax.m_y ? y : pmax.m_y;
			pmax.m_z = z > pmax.m_z ? z : pmax.m_z;
		}

		size = (pmax - pmin).Scale (0.5f);
		origin = (pmax + pmin).Scale (0.5f);
		origin.m_w = 1.0f;
	}

	//NewtonBody* CreateBodyPart (DemoEntity* const bodyPart, const dPasiveRagDollDefinition& definition) 
	NewtonBody* CreateBodyPart (DemoEntity* const bodyPart) 
	{
		NewtonWorld* const world = GetWorld();
		NewtonCollision* const shape = bodyPart->CreateCollisionFromchildren(world);
		dAssert(shape);

		// calculate the bone matrix
		dMatrix matrix (bodyPart->CalculateGlobalMatrix());

		// create the rigid body that will make this bone
		NewtonBody* const bone = NewtonCreateDynamicBody (world, shape, &matrix[0][0]);

		// calculate the moment of inertia and the relative center of mass of the solid
		//NewtonBodySetMassProperties (bone, definition.m_mass, shape);
		NewtonBodySetMassProperties (bone, 1.0f, shape);

		// save the user data with the bone body (usually the visual geometry)
		NewtonBodySetUserData(bone, bodyPart);

		// assign the material for early collision culling
		NewtonBodySetMaterialGroupID (bone, m_material);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback (bone, PhysicsApplyGravityForce);

		// destroy the collision helper shape 
		NewtonDestroyCollision (shape);
		return bone;
	}

	void ConnectBodyParts (NewtonBody* const bone, NewtonBody* const parent, const dPasiveRagDollDefinition& definition) const
	{
		dMatrix matrix;
		NewtonBodyGetMatrix(bone, &matrix[0][0]);


		dPasiveRagDollDefinition::dPasiveRagDollFrameMatrix frameAngle (definition.m_frameBasics);
		dMatrix pinAndPivotInGlobalSpace (dPitchMatrix(frameAngle.m_pitch * dDegreeToRad) * dYawMatrix(frameAngle.m_yaw * dDegreeToRad) * dRollMatrix(frameAngle.m_roll * dDegreeToRad));
		pinAndPivotInGlobalSpace = pinAndPivotInGlobalSpace * matrix;

		dMatrix parentRollMatrix (dGetIdentityMatrix () * pinAndPivotInGlobalSpace);

		dPasiveRagDollDefinition::dPasiveRagDollJointLimitx jointLimits (definition.m_jointLimits);
		dCustomBallAndSocket* const joint = new dCustomBallAndSocket(pinAndPivotInGlobalSpace, parentRollMatrix, bone, parent);

		dFloat friction = definition.m_friction * 0.25f;
		joint->EnableCone(true);
		joint->SetConeFriction(friction);
		joint->SetConeLimits(jointLimits.m_coneAngle * dDegreeToRad);

		joint->EnableTwist(true);
		joint->SetTwistFriction(friction);
		joint->SetTwistLimits(jointLimits.m_minTwistAngle * dDegreeToRad, jointLimits.m_maxTwistAngle * dDegreeToRad);
	}

	dCustomJoint* FindJoint(NewtonBody* const child, NewtonBody* const parent)
	{
		for (NewtonJoint* joint = NewtonBodyGetFirstJoint(child); joint; joint = NewtonBodyGetNextJoint(child, joint)) {
			dCustomJoint* const cJoint = (dCustomJoint*) NewtonJointGetUserData(joint);
			if (((child == cJoint->GetBody0()) && (parent == cJoint->GetBody1())) ||
				((child == cJoint->GetBody1()) && (parent == cJoint->GetBody0()))) {
				return cJoint;
			}
		}
		dAssert (0);
		return NULL;
	}

	virtual void OnUpdateTransform(const dCustomTransformController::dSkeletonBone* const bone, const dMatrix& localMatrix) const
	{
		NewtonBody* const body = bone->GetBody();
		DemoEntity* const ent = (DemoEntity*)NewtonBodyGetUserData(body);
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(NewtonBodyGetWorld(body));

		dQuaternion rot(localMatrix);
		ent->SetMatrix(*scene, rot, localMatrix.m_posit);
	}

	dCustomTransformController* CreateRagDoll(const dMatrix& location, const DemoEntity* const model, dPasiveRagDollDefinition* const definition, int defintionCount)
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		// make a clone of the mesh 
		DemoEntity* const modelEntity = (DemoEntity*)model->CreateClone();
		scene->Append(modelEntity);

		// add the root bone
		DemoEntity* const rootEntity = (DemoEntity*)modelEntity->Find(definition[0].m_boneName);
		NewtonBody* const rootBone = CreateBodyPart(rootEntity);

		// build the rag doll with rigid bodies connected by joints
		dCustomTransformController* const controller = CreateController(rootBone, dGetIdentityMatrix());

		controller->SetCalculateLocalTransforms(true);

		//dCustomTransformController::dSkeletonBone* const bone0 = controller->AddRoot(rootBone, dGetIdentityMatrix());
		// save the controller as the collision user data, for collision culling
		NewtonCollisionSetUserData(NewtonBodyGetCollision(rootBone), controller);

		int stackIndex = 0;
		DemoEntity* childEntities[32];
		dCustomTransformController::dSkeletonBone* parentBones[32];
		for (DemoEntity* child = rootEntity->GetChild(); child; child = child->GetSibling()) {
			parentBones[stackIndex] = controller;
			childEntities[stackIndex] = child;
			stackIndex++;
		}

		int bodyCount = 1;
		NewtonBody* bodyArray[1024];
		bodyArray[0] = rootBone;

		// walk model hierarchic adding all children designed as rigid body bones. 
		while (stackIndex) {
			stackIndex--;
			DemoEntity* const entity = childEntities[stackIndex];
			dCustomTransformController::dSkeletonBone* parentBone = parentBones[stackIndex];

			const char* const name = entity->GetName().GetStr();
			for (int i = 0; i < defintionCount; i++) {
				if (!strcmp(definition[i].m_boneName, name)) {
					NewtonBody* const bone = CreateBodyPart(entity);
					bodyArray[bodyCount] = bone;
					bodyCount ++;
					
					// connect this body part to its parent with a ragdoll joint
					NewtonBody* const parentBody = parentBone->GetBody();
					ConnectBodyParts(bone, parentBody, definition[i]);

					dMatrix bindMatrix(entity->GetParent()->CalculateGlobalMatrix((DemoEntity*)NewtonBodyGetUserData(parentBody)).Inverse());
					parentBone = controller->AddBone(bone, bindMatrix, parentBone);
					// save the controller as the collision user data, for collision culling
					NewtonCollisionSetUserData(NewtonBodyGetCollision(bone), parentBone);
					break;
				}
			}

			for (DemoEntity* child = entity->GetChild(); child; child = child->GetSibling()) {
				parentBones[stackIndex] = parentBone;
				childEntities[stackIndex] = child;
				stackIndex++;
			}
		}

		dFloat volume = 0.0f;
		for (int i = 0; i < bodyCount; i ++) {
			volume += NewtonConvexCollisionCalculateVolume (NewtonBodyGetCollision(bodyArray[i]));
		}
		dFloat density = 100.0f / volume;

		for (int i = 0; i < bodyCount; i ++) {
			dFloat Ixx;
			dFloat Iyy;
			dFloat Izz;
			dFloat mass;
			NewtonBody* const body = bodyArray[i];
			NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);
			dFloat scale = density * NewtonConvexCollisionCalculateVolume (NewtonBodyGetCollision(body));
			mass *= scale;
			Ixx *= scale;
			Iyy *= scale;
			Izz *= scale;
			NewtonBodySetMassMatrix(body, mass, Ixx, Iyy, Izz);
		}

		// set the collision mask
		// note this container work best with a material call back for setting bit field 
		//dAssert(0);
		//controller->SetDefaultSelfCollisionMask();

		// transform the entire contraction to its location
		dMatrix worldMatrix(rootEntity->GetCurrentMatrix() * location);
		NewtonBodySetMatrixRecursive(rootBone, &worldMatrix[0][0]);

		return controller;
	}

	int m_material;
};

void PassiveRagdoll (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();


	//CreateLevelMesh (scene, "flatPlane.ngd", true);
	CreateHeightFieldTerrain(scene, HEIGHTFIELD_DEFAULT_SIZE, HEIGHTFIELD_DEFAULT_CELLSIZE, 1.5f, 0.2f, 200.0f, -50.0f);

	// load a skeleton mesh 
	dPointer<DemoEntity> ragDollModel (DemoEntity::LoadNGD_mesh("whiteman.ngd", scene->GetNewton(), scene->GetShaderCache()));

	//  create a skeletal transform controller for controlling rag doll
	PassiveRagdollManager* const manager = new PassiveRagdollManager (scene);

	NewtonWorld* const world = scene->GetNewton();
	dMatrix matrix (dGetIdentityMatrix());

//	dVector origin (-10.0f, 1.0f, 0.0f, 1.0f);
	dVector origin (FindFloor (world, dVector (-10.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));

	dVector p(origin + dVector(0.0f, 0.0f, 0.0f, 0.0f));
	matrix.m_posit = FindFloor(world, p, 100.0f);
	matrix.m_posit.m_y += 2.0f;
	dCustomTransformController* const ragdoll = manager->CreateRagDoll(matrix, &(*ragDollModel), skeletonRagDoll, sizeof(skeletonRagDoll) / sizeof(skeletonRagDoll[0]));

	// attach this ragdoll to world with a fix joint
	dMatrix rootMatrix; 
	NewtonBodyGetMatrix(ragdoll->GetBody(), &rootMatrix[0][0]);
	//new dCustom6dof (rootMatrix, ragdoll->GetBody());

	int count = 6;
	for (int x = 0; x < count; x ++) {
		for (int z = 0; z < count; z ++) {
			dVector p (origin + dVector (10.0f + (x - count / 2) * 3.0f - count / 2, 0.0f, (z - count / 2) * 3.0f, 0.0f));
			matrix.m_posit = FindFloor (world, p, 100.0f);
			matrix.m_posit.m_y += 2.0f;
			manager->CreateRagDoll(matrix, &(*ragDollModel), skeletonRagDoll, sizeof(skeletonRagDoll) / sizeof(skeletonRagDoll[0]));
		}
	}

//	const int defaultMaterialID = NewtonMaterialGetDefaultGroupID(scene->GetNewton());
	const dVector location(origin);
	const dVector size(0.25f, 0.25f, 0.375f, 0.0f);
//	const int count1 = 5;
	const dMatrix shapeOffsetMatrix(dGetIdentityMatrix());
//	AddPrimitiveArray(scene, 10.0f, location, size, count1, count1, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);

//	origin.m_x -= 25.0f;
	origin.m_x -= 4.0f;
	origin.m_y += 3.0f;
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
}
