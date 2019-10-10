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


class PassiveRagdollManager: public dModelManager
{
	class dJointDefinition
	{
		public:
		enum dCollsionMask
		{
			m_type0,
			m_type1,
			m_type2,
			m_type3,
		};

		struct dJointLimit
		{
			dFloat m_minTwistAngle;
			dFloat m_maxTwistAngle;
			dFloat m_coneAngle;
		};

		struct dFrameMatrix
		{
			dFloat m_pitch;
			dFloat m_yaw;
			dFloat m_roll;
		};

		char m_boneName[32];
		int m_type;
		int m_typeMask;
		dFloat m_friction;
		dJointLimit m_jointLimits;
		dFrameMatrix m_frameBasics;
	};

	public: 
	PassiveRagdollManager (DemoEntityManager* const scene)
		:dModelManager (scene->GetNewton())
	{
		// create a material for early collision culling
		m_material = NewtonMaterialCreateGroupID(scene->GetNewton());
		NewtonMaterialSetCallbackUserData (scene->GetNewton(), m_material, m_material, this);
		NewtonMaterialSetCollisionCallback (scene->GetNewton(), m_material, m_material, OnBoneAABBOverlap, NULL);
	}

	virtual void OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
	{
	}

	virtual void OnPreUpdate(dModelRootNode* const root, dFloat timestep) const
	{
	}

	static int OnBoneAABBOverlap(const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
	{
		const NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
		const NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);
		const NewtonCollision* const collision0 = NewtonBodyGetCollision(body0);
		const NewtonCollision* const collision1 = NewtonBodyGetCollision(body1);

		NewtonCollisionMaterial collisionMaterial0;
		NewtonCollisionMaterial collisionMaterial1;
		NewtonCollisionGetMaterial(collision0, &collisionMaterial0);
		NewtonCollisionGetMaterial(collision1, &collisionMaterial1);
		if (collisionMaterial0.m_userData != collisionMaterial1.m_userData) {
			return 1;
		}

		int val0 = (collisionMaterial0.m_userFlags0 & collisionMaterial1.m_userFlags1);
		int val1 = (collisionMaterial1.m_userFlags0 & collisionMaterial0.m_userFlags1);
		return (val0 && val1) ? 1 : 0;
	}

	static void ClampAngularVelocity(const NewtonBody* body, dFloat timestep, int threadIndex)
	{
		dVector omega;
		NewtonBodyGetOmega(body, &omega[0]);
		omega.m_w = 0.0f;
		dFloat mag2 = omega.DotProduct3(omega);
		if (mag2 > (100.0f * 100.0f)) {
			omega = omega.Normalize().Scale (100.0f);
			NewtonBodySetOmega(body, &omega[0]);
		}

		PhysicsApplyGravityForce (body, timestep, threadIndex);
	}

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
		//NewtonBodySetForceAndTorqueCallback (bone, PhysicsApplyGravityForce);
		NewtonBodySetForceAndTorqueCallback (bone, ClampAngularVelocity);

		// destroy the collision helper shape 
		NewtonDestroyCollision (shape);
		return bone;
	}

	void ConnectBodyParts (NewtonBody* const bone, NewtonBody* const parent, const dJointDefinition& definition) const
	{
		dMatrix matrix;
		NewtonBodyGetMatrix(bone, &matrix[0][0]);
		
		dJointDefinition::dFrameMatrix frameAngle (definition.m_frameBasics);
		dMatrix pinAndPivotInGlobalSpace (dPitchMatrix(frameAngle.m_pitch * dDegreeToRad) * dYawMatrix(frameAngle.m_yaw * dDegreeToRad) * dRollMatrix(frameAngle.m_roll * dDegreeToRad));
		pinAndPivotInGlobalSpace = pinAndPivotInGlobalSpace * matrix;

		dMatrix parentRollMatrix (dGetIdentityMatrix () * pinAndPivotInGlobalSpace);

		dJointDefinition::dJointLimit jointLimits (definition.m_jointLimits);
		dCustomBallAndSocket* const joint = new dCustomBallAndSocket(pinAndPivotInGlobalSpace, parentRollMatrix, bone, parent);

		dFloat friction = definition.m_friction * 0.25f;
		joint->EnableCone(true);
		joint->SetConeFriction(friction);
		joint->SetConeLimits(jointLimits.m_coneAngle * dDegreeToRad);

		joint->EnableTwist(true);
		joint->SetTwistFriction(friction);
		joint->SetTwistLimits(jointLimits.m_minTwistAngle * dDegreeToRad, jointLimits.m_maxTwistAngle * dDegreeToRad);
	}

	virtual void OnUpdateTransform(const dModelNode* const bone, const dMatrix& localMatrix) const
	{
		NewtonBody* const body = bone->GetBody();
		DemoEntity* const ent = (DemoEntity*)NewtonBodyGetUserData(body);
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(NewtonBodyGetWorld(body));

		dQuaternion rot(localMatrix);
		ent->SetMatrix(*scene, rot, localMatrix.m_posit);
	}

	void SetModelMass (dFloat mass, int bodyCount, NewtonBody** const bodyArray) const
	{
		dFloat volume = 0.0f;
		for (int i = 0; i < bodyCount; i++) {
			volume += NewtonConvexCollisionCalculateVolume(NewtonBodyGetCollision(bodyArray[i]));
		}
		dFloat density = mass / volume;

		for (int i = 0; i < bodyCount; i++) {
			dFloat Ixx;
			dFloat Iyy;
			dFloat Izz;

			NewtonBody* const body = bodyArray[i];
			NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);
			dFloat scale = density * NewtonConvexCollisionCalculateVolume(NewtonBodyGetCollision(body));
			mass *= scale;
			Ixx *= scale;
			Iyy *= scale;
			Izz *= scale;
			NewtonBodySetMassMatrix(body, mass, Ixx, Iyy, Izz);
		}
	}

	dModelRootNode* CreateRagDoll(const dMatrix& location, const DemoEntity* const model)
	{
		static dJointDefinition jointsDefinition[] =
		{
			{ "mixamorig:Hips", 1, 16 },

			{ "mixamorig:Neck", 16, 31, 100.0f, { -15.0f, 15.0f, 30.0f }, { 0.0f, 0.0f, 180.0f } },
			{ "mixamorig:Spine", 2, 16, 100.0f, { -15.0f, 15.0f,  30.0f }, { 0.0f, 0.0f, 180.0f } },
			{ "mixamorig:Spine1", 4, 16, 100.0f, { -15.0f, 15.0f, 30.0f }, { 0.0f, 0.0f, 180.0f } },
			{ "mixamorig:Spine2", 8, 16, 100.0f, { -15.0f, 15.0f, 30.0f }, { 0.0f, 0.0f, 180.0f } },
			
			{ "mixamorig:LeftArm", 16, 27, 100.0f, { -45.0f, 45.0f, 80.0f }, { 0.0f, 0.0f, 180.0f } },
			{ "mixamorig:LeftForeArm", 16, 31, 50.0f, { -140.0f, 10.0f, 0.0f }, { 0.0f, 0.0f, -90.0f } },
			
			{ "mixamorig:RightArm", 16, 27, 100.0f, { -45.0f, 45.0f, 80.0f }, { 0.0f, 0.0f, 180.0f } },
			{ "mixamorig:RightForeArm", 16, 31, 50.0f, { -140.0f, 10.0f, 0.0f }, { 0.0f, 00.0f, 90.0f } },
			
			{ "mixamorig:LeftUpLeg", 16, 31, 100.0f, { -45.0f, 45.0f, 120.0f }, { 0.0f, 0.0f, 180.0f } },
			{ "mixamorig:LeftLeg", 16, 31, 50.0f, { -140.0f, 10.0f, 0.0f }, { 0.0f, 90.0f, 90.0f } },
			
			{ "mixamorig:RightUpLeg", 16, 31, 100.0f, { -45.0f, 45.0f, 120.0f }, { 0.0f, 0.0f, 180.0f } },
			{ "mixamorig:RightLeg", 16, 31, 50.0f, { -140.0f, 10.0f, 0.0f }, { 0.0f, 90.0f, 90.0f } },
		};

		const int definitionCount = sizeof (jointsDefinition)/sizeof (jointsDefinition[0]);

		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		// make a clone of the mesh 
		DemoEntity* const modelEntity = (DemoEntity*)model->CreateClone();
		scene->Append(modelEntity);

		// add the root bone
		DemoEntity* const rootEntity = (DemoEntity*)modelEntity->Find(jointsDefinition[0].m_boneName);
		NewtonBody* const rootBone = CreateBodyPart(rootEntity);

		// build the rag doll with rigid bodies connected by joints
		dModelRootNode* const controller = new dModelRootNode(rootBone, dGetIdentityMatrix());

		// add this root model 
		AddRoot(controller);

		//controller->SetCalculateLocalTransforms(true);
		controller->SetTranformMode(true);

		NewtonCollisionMaterial collisionMaterial;
		NewtonCollisionGetMaterial(NewtonBodyGetCollision(rootBone), &collisionMaterial);
		collisionMaterial.m_userData = rootBone;
		collisionMaterial.m_userFlags0 = jointsDefinition[0].m_type;
		collisionMaterial.m_userFlags1 = jointsDefinition[0].m_typeMask;
		NewtonCollisionSetMaterial(NewtonBodyGetCollision(rootBone), &collisionMaterial);

		int stackIndex = 0;
		dModelNode* parentBones[32];
		DemoEntity* childEntities[32];
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
			dModelNode* parentBone = parentBones[stackIndex];
			DemoEntity* const entity = childEntities[stackIndex];
			const char* const name = entity->GetName().GetStr();
			dTrace(("name: %s\n", name));
			for (int i = 0; i < definitionCount; i++) {
				if (!strcmp(jointsDefinition[i].m_boneName, name)) {
					NewtonBody* const childBody = CreateBodyPart(entity);
					bodyArray[bodyCount] = childBody;
					bodyCount ++;
					
					// connect this body part to its parent with a ragdoll joint
					NewtonBody* const parentBody = parentBone->GetBody();
					ConnectBodyParts(childBody, parentBody, jointsDefinition[i]);

					dMatrix bindMatrix(entity->GetParent()->CalculateGlobalMatrix((DemoEntity*)NewtonBodyGetUserData(parentBody)).Inverse());
					parentBone = new dModelNode(childBody, bindMatrix, parentBone);

					// save the controller as the collision user data, for collision culling
					NewtonCollisionMaterial collisionMaterial;
					NewtonCollisionGetMaterial(NewtonBodyGetCollision(childBody), &collisionMaterial);
					collisionMaterial.m_userData = rootBone;
					collisionMaterial.m_userFlags0 = jointsDefinition[i].m_type;
					collisionMaterial.m_userFlags1 = jointsDefinition[i].m_typeMask;
					NewtonCollisionSetMaterial(NewtonBodyGetCollision(childBody), &collisionMaterial);
					break;
				}
			}

			for (DemoEntity* child = entity->GetChild(); child; child = child->GetSibling()) {
				parentBones[stackIndex] = parentBone;
				childEntities[stackIndex] = child;
				stackIndex++;
			}
		}

		SetModelMass (100.0f, bodyCount, bodyArray);

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
	dModelRootNode* const ragdoll = manager->CreateRagDoll(matrix, &(*ragDollModel));

	// attach this ragdoll to world with a fix joint
	dMatrix rootMatrix; 
	NewtonBodyGetMatrix(ragdoll->GetBody(), &rootMatrix[0][0]);

#if 0
	dCustomHinge* const fixToWorld = new dCustomHinge(rootMatrix, ragdoll->GetBody(), NULL);
	fixToWorld->EnableLimits(true);
	fixToWorld->SetLimits(0.0f, 0.0f);
#endif

	int count = 5;
	for (int x = 0; x < count; x ++) {
		for (int z = 0; z < count; z ++) {
			dVector p (origin + dVector (10.0f + (x - count / 2) * 3.0f - count / 2, 0.0f, (z - count / 2) * 3.0f, 0.0f));
			matrix.m_posit = FindFloor (world, p, 100.0f);
			matrix.m_posit.m_y += 2.0f;
			manager->CreateRagDoll(matrix, &(*ragDollModel));
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
