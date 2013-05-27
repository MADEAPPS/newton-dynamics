/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include <toolbox_stdafx.h>
#include "SkyBox.h"
#include "DemoMesh.h"
#include "DemoCamera.h"
#include "NewtonDemos.h"
#include "PhysicsUtils.h"
#include "TargaToOpenGl.h"
#include "CustomRagDoll.h"
#include "DemoEntityManager.h"
#include "toolBox/DebugDisplay.h"
#include "CustomSkeletonTransformManager.h"



struct RAGDOLL_BONE_DEFINITION
{
	char m_boneName[32];
	char m_shapeType[32];

	dFloat m_shapePitch;
	dFloat m_shapeYaw;
	dFloat m_shapeRoll;

	dFloat m_shape_x;
	dFloat m_shape_y;
	dFloat m_shape_z;

	dFloat m_radio;
	dFloat m_height;

	dFloat m_mass;
	dFloat m_coneAngle;
	dFloat m_minTwistAngle;
	dFloat m_maxTwistAngle;

	dFloat m_pitchLimit;
	dFloat m_yawLimit;
	dFloat m_rollLimit;

	int m_collideWithNonImmidiateBodies;
};


static RAGDOLL_BONE_DEFINITION skeletonRagDoll[] =
{
	{"Bip01_Pelvis",	"capsule", 0.0f, 0.0f, -90.0f, 0.0f, 0.0f, 0.00f, 0.07f, 0.16f, 20.0f,  0.0f,  -0.0f,    0.0f, 0.0f,  0.0f,  0.0f, 1}, 
	{"Bip01_Spine",		"capsule", 0.0f, 0.0f, -90.0f, 0.0f, 0.0f, 0.05f, 0.07f, 0.15f, 20.0f,  0.0f,  -0.0f,    0.0f, 0.0f,  0.0f,  0.0f, 1}, 
	{"Bip01_Spine1",	"capsule", 0.0f, 0.0f, -90.0f, 0.0f, 0.0f, 0.05f, 0.07f, 0.14f, 20.0f,  0.0f,  -0.0f,    0.0f, 0.0f,  0.0f,  0.0f, 1}, 
	{"Bip01_Spine2",	"capsule", 0.0f, 0.0f, -90.0f, 0.0f, 0.0f, 0.05f, 0.07f, 0.12f, 20.0f,  0.0f,  -0.0f,    0.0f, 0.0f,  0.0f,  0.0f, 1}, 
	
};



class RagDollManager: public CustomSkeletonTransformManager
{
	public: 
	RagDollManager (DemoEntityManager* const scene)
		:CustomSkeletonTransformManager (scene->GetNewton())
	{
	}

	void GetDimentions(DemoEntity* const bodyPart, dVector& origin, dVector& size) const
	{	
		DemoMesh* const mesh = bodyPart->GetMesh();

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


	NewtonCollision* MakeSphere(DemoEntity* const bodyPart) const
	{
//		dVector size;
//		dVector origin;
//		dMatrix matrix (GetIdentityMatrix());
//		GetDimentions(bone, matrix.m_posit, size);
//		return NewtonCreateSphere (nWorld, size.m_x, size.m_x, size.m_x, 0, &matrix[0][0]);
		dAssert (0);
		return NULL;
	}

	NewtonCollision* MakeCapsule(DemoEntity* const bodyPart, const RAGDOLL_BONE_DEFINITION& definition) const
	{
		dVector size;
		dVector origin;
		dMatrix matrix (dPitchMatrix(definition.m_shapePitch * 3.141592f / 180.0f) * dYawMatrix(definition.m_shapeYaw * 3.141592f / 180.0f) * dRollMatrix(definition.m_shapeRoll * 3.141592f / 180.0f));

		matrix.m_posit.m_x = definition.m_shape_x;
		matrix.m_posit.m_y = definition.m_shape_y;
		matrix.m_posit.m_z = definition.m_shape_z;
		return NewtonCreateCapsule (GetWorld(), definition.m_radio, definition.m_height, 0, &matrix[0][0]);
	}

	NewtonCollision* MakeBox(DemoEntity* const bodyPart) const
	{
		dAssert (0);
//		dVector size;
//		dVector origin;
//		dMatrix matrix (GetIdentityMatrix());
//		GetDimentions(bone, matrix.m_posit, size);
//		return NewtonCreateBox (nWorld, 2.0f * size.m_x, 2.0f * size.m_y, 2.0f * size.m_z, 0, &matrix[0][0]);
		return NULL;
	}

/*
	void ApplyBoneMatrix (int boneIndex, void* userData, const dMatrix& matrix) const
	{
		if (boneIndex == 0) {
			dBone* boneNode = (dBone*) userData;
			dMatrix rootMatrix (matrix);
			while (boneNode->GetParent()) {
				rootMatrix = boneNode->GetMatrix().Inverse() * rootMatrix;
				boneNode = boneNode->GetParent();
			}
			m_model->SetMatrix(rootMatrix);
		} else {

			dBone* boneNode = (dBone*) userData;
			// check if the parent of this bone is also a bone, body1 is the parentBone
			dBone* parentBone = (dBone*) NewtonBodyGetUserData (GetParentBone (boneIndex));
			if (boneIndex && (boneNode->GetParent() != parentBone)) {
				// this is not and immediate bone calculate the offset matrix
				dBone* parent;
				parent = boneNode->GetParent();
				dMatrix offset (parent->GetMatrix());
				for (parent = parent->GetParent(); parent != parentBone; parent = parent->GetParent()) {
					offset = offset * parent->GetMatrix();
				}

				dMatrix localMatrix (matrix * offset.Inverse());
				boneNode->SetMatrix (localMatrix);

			} else {
				boneNode->SetMatrix (matrix);
			}
		}

	}
*/

	NewtonCollision* MakeConvexHull(DemoEntity* const bodyPart) const
	{
		dAssert (0);
/*
		dVector points[1024 * 16];

		int vertexCount = 0;
		dSkinModifier* skinModifier = (dSkinModifier*) skinMeshInstance->GetModifier();
		_ASSERTE (skinModifier);

		// go over the vertex array and find and collect all vertices's weighted by this bone.
		const dVector* const weights = skinModifier->m_vertexWeight;	
		const dBone** const skinnedBones = skinModifier->m_skinnedBones;
		const dSkinModifier::dBoneWeightIndex* const indices = skinModifier->m_boneWeightIndex;
		const dMesh* skin = skinMeshInstance->m_mesh;
		for(int i = 0; i < skin->m_vertexCount; i ++) {
			for (int j = 0 ;  (j < 4) && (weights[i][j] > 0.125f); j ++) {
				int boneIndex = indices[i].m_index[j];
				// if the vertex is weighted by this bone consider it part of the collision if the weight is the largest
				if (skinnedBones[boneIndex] == bone) {
					points[vertexCount].m_x = skin->m_vertex[i * 3 + 0];
					points[vertexCount].m_y = skin->m_vertex[i * 3 + 1];
					points[vertexCount].m_z = skin->m_vertex[i * 3 + 2];
					vertexCount ++;
					break;
				}
			}
		}

		// here we have the vertex array the are part of the collision shape
		_ASSERTE (vertexCount);

		int bondMatrixIndex;
		for (bondMatrixIndex = 0; bondMatrixIndex < skinModifier->m_bonesCount; bondMatrixIndex ++) {
			if (bone == skinModifier->m_skinnedBones[bondMatrixIndex]) {
				break;
			}
		}

		const dMatrix matrix (skinModifier->m_shapeBindMatrix * skinModifier->m_bindingMatrices[bondMatrixIndex]);
		matrix.TransformTriplex (&points[0].m_x, sizeof (dVector), &points[0].m_x, sizeof (dVector), vertexCount);
		return NewtonCreateConvexHull (nWorld, vertexCount, &points[0].m_x, sizeof (dVector), 1.0e-3f, 0, NULL);
*/
		return NULL;
	}



	//NewtonBody* CreateRagDollBodyPart (NewtonWorld* nWorld, const dBone* bone, const RAGDOLL_BONE_DEFINITION& definition, const dMeshInstance* skinMesh) 
	NewtonBody* CreateRagDollBodyPart (DemoEntity* const bodyPart, const RAGDOLL_BONE_DEFINITION& definition) 
	{
		NewtonCollision* shape = NULL;
		if (!strcmp (definition.m_shapeType, "sphere")) {
			shape = MakeSphere (bodyPart);
		} else if (!strcmp (definition.m_shapeType, "capsule")) {
			shape = MakeCapsule(bodyPart, definition);
		} else if (!strcmp (definition.m_shapeType, "box")) {
			shape = MakeBox (bodyPart);
		} else {
			shape = MakeConvexHull(bodyPart);
		}

		// calculate the bone matrix
		dMatrix matrix (bodyPart->CalculateGlobalMatrix());

		// find the index of the bone parent
//		int parentIndeIndex = -1;
//		for (dBone* parentNode = bone->GetParent(); parentNode && (parentIndeIndex == -1); parentNode = parentNode->GetParent()) {
//			for (int i = 0; i <  GetBoneCount(); i ++) {
//				if (parentNode == NewtonBodyGetUserData (GetBone (i))) {
//					parentIndeIndex = i;
//					break;
//				}
//			}
//		} 
//		dMatrix pinAndPivot (dPitchMatrix (definition.m_pitch * 3.141592f / 180.0f) * dYawMatrix (definition.m_yaw * 3.141592f / 180.0f) * dRollMatrix (definition.m_roll * 3.141592f / 180.0f));
//		pinAndPivot = pinAndPivot * rootMatrix;
//		int boneIndex = AddBone (nWorld, parentIndeIndex, (void*) bone, rootMatrix, definition.m_mass, pinAndPivot, shape);
//		SetCollisionState (boneIndex, definition.m_collideWithNonImmidiateBodies);
//		SetBoneConeLimits (boneIndex, definition.m_coneAngle * 3.141592f / 180.0f);
//		SetBoneTwistLimits (boneIndex, definition.m_minTwistAngle * 3.141592f / 180.0f, definition.m_maxTwistAngle * 3.141592f / 180.0f);

		NewtonWorld* const world = GetWorld();

		// create the rigid body that will make this bone
		NewtonBody* const bone = NewtonCreateDynamicBody (world, shape, &matrix[0][0]);

		// calculate the moment of inertia and the relative center of mass of the solid
		NewtonBodySetMassProperties (bone, definition.m_mass, shape);

		// save the user data with the bone body (usually the visual geometry)
		NewtonBodySetUserData(bone, bodyPart);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
//		NewtonBodySetForceAndTorqueCallback (bone, PhysicsApplyGravityForce);


//		CustomRagDollLimbJoint* joint = NULL;
//		if (m_bonesCount == 0) {
//			// this is the root bone, initialize ragdoll with the root Bone
//			Init (1, bone, NULL);
//			NewtonBodySetTransformCallback(bone, TransformCallback);
//			parentBoneIndex = 0;
//		} else {
//			// this is a child bone, need to be connected to its parent by a joint 
//			NewtonBody* const parent = m_bonesBodies[parentBoneIndex];
//			joint = new CustomRagDollLimbJoint (pivotInGlobalSpace, bone, parent);
//		}
//		m_joints[m_bonesCount] = joint;
//		m_bonesBodies[m_bonesCount] = bone;
//		m_bonesParents[m_bonesCount] = m_bonesBodies[parentBoneIndex];
//		m_bonesCount ++;
//		return m_bonesCount - 1;


		// destroy the collision helper shape 
		NewtonDestroyCollision (shape);
		return bone;
	}


	void CreateRagDoll (const dMatrix& location, const DemoEntity* const model, RAGDOLL_BONE_DEFINITION* const definition, int defintionCount)
	{
		NewtonWorld* const world = GetWorld(); 
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);

		// make a clone of the mesh 
		DemoEntity* const ragDollEntity = (DemoEntity*) model->CreateClone();
		scene->Append(ragDollEntity);

		// build the ragdoll with rigid bodies connected by joints
		// create a transform controller
		CustomSkeletonTransformController* const controller = CreateTransformController (ragDollEntity);

		// add the root bone
		DemoEntity* const rootEntity = (DemoEntity*) ragDollEntity->Find (definition[0].m_boneName);
		NewtonBody* const rootBone = CreateRagDollBodyPart (rootEntity, definition[0]);
		controller->AddBone (rootBone, NULL);

		int stackIndex;
		NewtonBody* parentBone[32];
		DemoEntity* childEntities[32];
		stackIndex = 0;
		for (DemoEntity* child = rootEntity->GetChild(); child; child = child->GetSibling()) {
			parentBone[stackIndex] = rootBone;
			childEntities[stackIndex] = child;
			stackIndex ++;
		}

		while (stackIndex) {
			stackIndex --;
			DemoEntity* const entity = childEntities[stackIndex];
			NewtonBody* parent = parentBone[stackIndex];

			const char* const name = entity->GetName().GetStr();
			for (int i = 0; i < defintionCount; i ++) {
				if (!strcmp (definition[i].m_boneName, name)) {
					NewtonBody* const bone = CreateRagDollBodyPart (entity, definition[i]);
					controller->AddBone (bone, parent);
					parent = bone;
					break;
				}
			}

			for (DemoEntity* child = entity->GetChild(); child; child = child->GetSibling()) {
				parentBone[stackIndex] = parent;
				childEntities[stackIndex] = child;
				stackIndex ++;
			}
		}

//		matrix.m_posit = FindFloor (scene->GetNewton(), point, 10.0f);

	}
};


void DescreteRagDoll (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();
	CreateLevelMesh (scene, "flatPlane.ngd", true);

//CreateLevelMesh (scene, "skeleton.ngd", true);

	// load a skeleton mesh for using as a ragdoll manager
	DemoEntity ragDollModel(GetIdentityMatrix(), NULL);
	ragDollModel.LoadNGD_mesh ("skeleton.ngd", scene->GetNewton());

	dVector origin (-10.0f, 2.0f, 0.0f, 0.0f);

	//  create a skeletal transform controller for controlling ragdoll
	RagDollManager* const manager = new RagDollManager (scene);

	dMatrix matrix (GetIdentityMatrix());

	for (int x = 0; x < 1; x ++) {
		for (int z = 0; z < 1; z ++) {
			matrix.m_posit = origin + dVector (x * 3.0f + 5.0f, 0.0f, z * 3.0f, 1.0f);
			manager->CreateRagDoll (matrix, &ragDollModel, skeletonRagDoll, sizeof (skeletonRagDoll) / sizeof (skeletonRagDoll[0]));
		}
	}
	
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
}



