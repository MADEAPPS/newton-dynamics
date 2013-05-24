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
#include "NewtonDemos.h"
#include "PhysicsUtils.h"
#include "TargaToOpenGl.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "toolBox/DebugDisplay.h"
#include "CustomPlayerControllerManager.h"


#if 0

struct RAGGOLL_BONE_DEFINTION
{
	char m_boneName[32];
	char m_shapeType[32];
	dFloat m_mass;
	dFloat m_coneAngle;
	dFloat m_minTwistAngle;
	dFloat m_maxTwistAngle;

	dFloat m_pitch;
	dFloat m_yaw;
	dFloat m_roll;

	int m_collideWithNonImmidiateBodies;
};


static RAGGOLL_BONE_DEFINTION snowManDefinition[] =
{
	{"pelvis",		"sphere", 20.0f,  0.0f,  -0.0f,    0.0f, 0.0f,  0.0f,  0.0f, 1}, 
	{"torso",		"sphere",  8.0f, 30.0f,  -30.0f,  30.0f, 0.0f,  0.0f,  0.0f, 1}, 
	{"head",		"sphere",  5.0f, 30.0f, -30.0f,   30.0f, 0.0f,  0.0f,  0.0f, 1}, 
	{"rightArm",	"capsule", 5.0f, 80.0f,  -15.0f,  15.0f, 0.0f,  0.0f,  0.0f, 1}, 
	{"rightForeArm","capsule", 5.0f,  0.0f, -160.0f,   0.0f, 0.0f,  0.0f, 90.0f, 1}, 
	{"rightHand",	"box",     5.0f,  0.0f,  -30.0f,  30.0f, 0.0f,  90.0f, 0.0f, 1}, 

	{"leftArm",		"capsule", 5.0f, 80.0f,  -15.0f,  15.0f, 0.0f,  0.0f,  0.0f, 1}, 
	{"leftForeArm", "capsule", 5.0f,  0.0f,    0.0f, 160.0f, 0.0f,  0.0f, 90.0f, 1}, 
	{"leftHand",    "box",     5.0f,  0.0f,  -30.0f,  30.0f, 0.0f,  90.0f, 0.0f, 1}, 

	{"rightLeg",	"capsule",  8.0f, 80.0f,  -30.0f, 30.0f, 0.0f,  0.0f,  0.0f, 1}, 
	{"rightCalf",	"capsule",  5.0f,  0.0f, -150.0f,  0.0f, 0.0f, 90.0f,  0.0f, 1}, 
	{"rightFoot",	"box",	    2.0f,  0.0f,  -30.0f, 30.0f, 0.0f, 90.0f,  0.0f, 1}, 

	{"leftLeg",		"capsule",  8.0f, 80.0f,  -30.0f, 30.0f, 0.0f,  0.0f,  0.0f, 1}, 
	{"leftCalf",	"capsule",  5.0f,  0.0f, -150.0f,  0.0f, 0.0f, 90.0f,  0.0f, 1}, 
	{"leftFoot",	"box",	    2.0f,  0.0f,  -30.0f, 30.0f, 0.0f, 90.0f,  0.0f, 1}, 
};


static RAGGOLL_BONE_DEFINTION gymnastDefinition[] =
{
	{"PELVIS",	    "convex", 10.0f,  0.0f,  -0.0f,    0.0f, 0.0f,  0.0f,  0.0f, 1}, 

//	{"STOMACH",	    "convex",  8.0f, 30.0f,  -30.0f,  30.0f, 0.0f,  0.0f, 90.0f, 1}, 
	{"CHEST",	    "convex",  8.0f, 30.0f,  -30.0f,  30.0f, 0.0f,  0.0f, 90.0f, 0}, 
//	{"NECK",	    "convex",  8.0f, 30.0f,  -30.0f,  30.0f, 0.0f,  0.0f, 90.0f, 1}, 
	{"HEAD",	    "convex",  5.0f, 30.0f,  -30.0f,  30.0f, 0.0f,  0.0f, 90.0f, 1}, 


//	{"TIBULA_L",        "convex",  8.0f,	0.0f,  -15.0f, 15.0f, 0.0f,  0.0f, 0.0f, 0}, 
	{"ARM_L",	    "convex",  8.0f,	30.0f, -30.0f, 30.0f, 0.0f,  0.0f, 90.0f, 1}, 
	{"FOREARM_L",	"convex",  8.0f,	0.0f, -130.0f, 0.0f, 0.0f,  0.0f, 0.0f, 1}, 

//	{"TIBULA_R",        "convex",  8.0f,	0.0f,  -15.0f, 15.0f, 0.0f,  0.0f, 0.0f, 0}, 
	{"ARM_R",	    "convex",  8.0f,	30.0f, -30.0f, 30.0f, 0.0f,  0.0f, 90.0f, 1}, 
	{"FOREARM_R",	"convex",  8.0f,	0.0f, -130.0f, 0.0f, 0.0f,  0.0f, 0.0f, 1}, 

	{"FEMUR_R",	    "convex",  8.0f,  80.0f,  -30.0f, 30.0f, 0.0f,  0.0f, 90.0f, 1}, 
	{"CALF_R",	    "convex",	5.0f,  0.0f, -150.0f,   0.0f,  0.0f, 90.0f,  0.0f, 1}, 
	{"ANKLE_R",	    "convex",	2.0f,  0.0f,  -30.0f,	30.0f, 0.0f,  0.0f,  0.0f, 1}, 

	{"FEMUR_L",	    "convex",  8.0f,  80.0f,  -30.0f,	30.0f, 0.0f,   0.0f, 90.0f, 1}, 
	{"CALF_L",	    "convex",	5.0f,  0.0f, -150.0f,    0.0f,  0.0f,-90.0f,  0.0f, 1}, 
	{"ANKLE_L",	    "convex",	2.0f,  0.0f,  -30.0f,	30.0f, 0.0f,   0.0f,  0.0f, 1}, 
};



class RagDoll: public CustomRagDoll
{
	public:

	static RagDoll* Create (const dModel& ragDollMesh, int bonesCount, const RAGGOLL_BONE_DEFINTION* definition, SceneManager* system, NewtonWorld* nWorld, const dMatrix& matrix) 
	{
		OGLModel* model = new OGLModel;
		model->InitFromModel (ragDollMesh);

		model->SetMatrix(matrix);

		system->AddModel___ (model);
		model->Release();

		// set all matrices from root bone to mesh root to identity
//		dBone* rootBone = model->FindBone (definition[0].m_boneName);
//		model->SetMatrix (rootBone->CalcGlobalMatrix() * matrix);
//		for (dBone* node = rootBone; node; node = node->GetParent()) {
//			node->SetMatrix(GetIdentityMatrix());
//		}

		// Create the Rag doll
		RagDoll *ragDoll = new RagDoll(model);


		// find the skin mesh, if this is a skinned model
		dMeshInstance* skinMeshInstance = NULL;
		for (dList<dMeshInstance>::dListNode* list = model->m_meshList.GetFirst(); list; list = list->GetNext()) {
			dMeshInstance& instance = list->GetInfo();
			if (instance.GetModifier()) {
				skinMeshInstance = &instance;
				break;
			}
		}

		int stackIndex;
		dBone* nodeStack[32];
		stackIndex = 1;
		nodeStack[0] = model->FindBone(0);
		while (stackIndex) {
			dBone* node;
			stackIndex --;
			node = nodeStack[stackIndex];

			const char* name = node->GetName();
			for (int i = 0; i < bonesCount; i ++) {
				if (!strcmp (definition[i].m_boneName, name)) {
					ragDoll->AddBody (nWorld, node, definition[i], skinMeshInstance); 
					break;
				}
			}

			for (node = node->GetChild(); node; node = node->GetSibling()) {
				nodeStack[stackIndex] = node;
				stackIndex ++;
			}
		}

		// set the force callback to all bodies
		for (int i = 0; i < ragDoll->GetBoneCount(); i ++) {
			const NewtonBody* bone;
			bone = ragDoll->GetBone(i);
			NewtonBodySetForceAndTorqueCallback (bone, PhysicsApplyGravityForce);
		}

		return ragDoll;
	}

protected:
	RagDoll(OGLModel* model)
		:CustomRagDoll ()
	{
		m_model = model;;
	}

	void GetDimentions(const dBone* bone, dVector& origin, dVector& size) const
	{	
		OGLMesh* mesh;
		mesh = (OGLMesh*) m_model->FindMesh (bone->GetName());

		dFloat* const array = mesh->m_vertex;
		dVector pmin( 1.0e20f,  1.0e20f,  1.0e20f, 0.0f);
		dVector pmax(-1.0e20f, -1.0e20f, -1.0e20f, 0.0f);
		for (int i = 0; i < mesh->m_vertexCount; i ++) {
			pmin.m_x = array[i * 3 + 0] < pmin.m_x ? array[i * 3 + 0] : pmin.m_x;
			pmin.m_y = array[i * 3 + 1] < pmin.m_y ? array[i * 3 + 1] : pmin.m_y;
			pmin.m_z = array[i * 3 + 2] < pmin.m_z ? array[i * 3 + 2] : pmin.m_z;
													 
			pmax.m_x = array[i * 3 + 0] > pmax.m_x ? array[i * 3 + 0] : pmax.m_x;
			pmax.m_y = array[i * 3 + 1] > pmax.m_y ? array[i * 3 + 1] : pmax.m_y;
			pmax.m_z = array[i * 3 + 2] > pmax.m_z ? array[i * 3 + 2] : pmax.m_z;
		}

		size = (pmax - pmin).Scale (0.5f);
		origin = (pmax + pmin).Scale (0.5f);
		origin.m_w = 1.0f;

	}

	NewtonCollision* MakeSphere(NewtonWorld* nWorld, const dBone* bone) const
	{
		dVector size;
		dVector origin;

		dMatrix matrix (GetIdentityMatrix());
		GetDimentions(bone, matrix.m_posit, size);

		return NewtonCreateSphere (nWorld, size.m_x, size.m_x, size.m_x, 0, &matrix[0][0]);
	}

	NewtonCollision* MakeCapsule(NewtonWorld* nWorld, const dBone* bone) const
	{
		dVector size;
		dVector origin;

		dMatrix matrix (GetIdentityMatrix());
		GetDimentions(bone, matrix.m_posit, size);

		return NewtonCreateCapsule (nWorld, size.m_y, 2.0f * size.m_x, 0, &matrix[0][0]);
	}

	NewtonCollision* MakeBox(NewtonWorld* nWorld, const dBone* bone) const
	{
		dVector size;
		dVector origin;

		dMatrix matrix (GetIdentityMatrix());
		GetDimentions(bone, matrix.m_posit, size);

		return NewtonCreateBox (nWorld, 2.0f * size.m_x, 2.0f * size.m_y, 2.0f * size.m_z, 0, &matrix[0][0]);
	}


	NewtonCollision* MakeConvexHull(NewtonWorld* nWorld, const dBone* bone, const dMeshInstance* skinMeshInstance) const
	{
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
	}

	void AddBody (NewtonWorld* nWorld, const dBone* bone, const RAGGOLL_BONE_DEFINTION& definition, const dMeshInstance* skinMesh) 
	{
		NewtonCollision* shape = NULL;
		if (!strcmp (definition.m_shapeType, "sphere")) {
			shape = MakeSphere (nWorld, bone);
		} else if (!strcmp (definition.m_shapeType, "capsule")) {
			shape = MakeCapsule(nWorld, bone);
		} else if (!strcmp (definition.m_shapeType, "box")) {
			shape = MakeBox (nWorld, bone);
		} else {
			shape = MakeConvexHull(nWorld, bone, skinMesh);
		}

		// calculate the bone matrix
		dMatrix rootMatrix (bone->CalcGlobalMatrix());

		// find the index of the bone parent
		int parentIndeIndex = -1;
		for (dBone* parentNode = bone->GetParent(); parentNode && (parentIndeIndex == -1); parentNode = parentNode->GetParent()) {
			for (int i = 0; i <  GetBoneCount(); i ++) {
				if (parentNode == NewtonBodyGetUserData (GetBone (i))) {
					parentIndeIndex = i;
					break;
				}
			}
		} 

		dMatrix pinAndPivot (dPitchMatrix (definition.m_pitch * 3.141592f / 180.0f) * dYawMatrix (definition.m_yaw * 3.141592f / 180.0f) * dRollMatrix (definition.m_roll * 3.141592f / 180.0f));
		pinAndPivot = pinAndPivot * rootMatrix;
		int boneIndex = AddBone (nWorld, parentIndeIndex, (void*) bone, rootMatrix, definition.m_mass, pinAndPivot, shape);

		SetCollisionState (boneIndex, definition.m_collideWithNonImmidiateBodies);
		SetBoneConeLimits (boneIndex, definition.m_coneAngle * 3.141592f / 180.0f);
		SetBoneTwistLimits (boneIndex, definition.m_minTwistAngle * 3.141592f / 180.0f, definition.m_maxTwistAngle * 3.141592f / 180.0f);

		// set the offset matrix 
		NewtonReleaseCollision (nWorld, shape);
	}


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


	void SubmitConstraints (dFloat timestep, int threadIndex)
	{
		CustomRagDoll::SubmitConstraints (timestep, threadIndex);

		for (int i = 1; i < GetBoneCount(); i ++) {
			const NewtonCustomJoint* joint;
			joint = GetJoint(i);
			ShowJointInfo(joint);
		}
	}

	OGLModel* m_model;
};


static void SetDemoCallbacks (SceneManager& system)
{
	system.m_control = Keyboard;
	system.m_autoSleep = AutoSleep;
	system.m_showIslands = SetShowIslands;
	system.m_showContacts = SetShowContacts; 
	system.m_setMeshCollision = SetShowMeshCollision;
}


static void BuildFloorAndSceneRoot (SceneManager& system)
{
	NewtonWorld* world;
	RenderPrimitive* floor;
	NewtonBody* floorBody;
	NewtonCollision* floorCollision;
	OGLMesh* meshInstance;

	world = system.m_world;
	// /////////////////////////////////////////////////////////////////////
	//
	// create the sky box,
	OGLModel* sky = new SkyBox ();
	system.AddModel___ (sky);
	sky->Release();


	// create the the floor graphic objects
	dVector floorSize (100.0f, 2.0f, 100.0f);
	dMatrix location (GetIdentityMatrix());
	location.m_posit.m_y = -5.0f; 

	// create a box for floor 
	floorCollision = NewtonCreateBox (world, floorSize.m_x, floorSize.m_y, floorSize.m_z, 0, NULL); 

//	meshInstance = OGLMesh::MakeBox (world, size.m_x, size.m_y, size.m_z, "GrassAndDirt.tga");
	meshInstance = new OGLMesh ("Floor", floorCollision, "GrassAndDirt.tga", "metal_30.tga", "metal_30.tga");
	floor = new RenderPrimitive (location, meshInstance);
	system.AddModel___ (floor);
	floor->Release();
	meshInstance->Release();

	// create the the floor collision, and body with default values
	floorBody = NewtonCreateBody (world, floorCollision);
	NewtonReleaseCollision (world, floorCollision);


	// set the transformation for this rigid body
	NewtonBodySetMatrix (floorBody, &location[0][0]);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (floorBody, floor);

	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (floorBody, PhysicsBodyDestructor);


	// get the default material ID
	int defaultID;
	defaultID = NewtonMaterialGetDefaultGroupID (world);

	// set default material properties
	NewtonMaterialSetDefaultSoftness (world, defaultID, defaultID, 0.05f);
	NewtonMaterialSetDefaultElasticity (world, defaultID, defaultID, 0.4f);
	NewtonMaterialSetDefaultCollidable (world, defaultID, defaultID, 1);
	NewtonMaterialSetDefaultFriction (world, defaultID, defaultID, 1.0f, 0.5f);
	NewtonMaterialSetCollisionCallback (world, defaultID, defaultID, NULL, NULL, GenericContactProcess); 

//	NewtonMaterialSetSurfaceThickness(world, materialID, materialID, 0.1f);
	NewtonMaterialSetSurfaceThickness(world, defaultID, defaultID, 0.0f);

	// set the island update callback
	NewtonSetIslandUpdateEvent (world, PhysicsIslandUpdate);

	// save the callback
	SetDemoCallbacks (system);

	InitEyePoint (dVector (1.0f, 0.0f, 0.0f), dVector (-40.0f, 10.0f, 0.0f));
}
#endif


void SkinRagDoll (DemoEntityManager* const scene)
{
_ASSERTE (0);
/*

	NewtonWorld* world;

	world = system.m_world;

	// create the sky box and the floor,
	BuildFloorAndSceneRoot (system);

	dVector posit (0.0f, 0.0f, 0.0f, 0.0f);
	posit.m_y = FindFloor (system.m_world, 0.0f, 0.0f) + 4.0f;
	InitEyePoint (dVector (1.0f, 0.0f, 0.0f), posit);

	OGLModel ragDoll;
	char fullPathName[2048];
	GetWorkingFileName ("gymnast.dae", fullPathName);
	OGLLoaderContext context;
	dMatrix rotMatrix (dYawMatrix (-3.14159265f * 0.5f));

_ASSERTE (0);
//	ragDoll.LoadCollada(fullPathName, context, rotMatrix, 1.0f);
	
	int bonesCount = sizeof (gymnastDefinition) / sizeof (gymnastDefinition[0]);
	for (int x = 0; x < 3; x ++) {
		for (int z = 0; z < 3; z ++) {
			dVector point (cameraEyepoint + dVector (x * 3.0f + 5.0f, 0.0f, z * 3.0f, 1.0f));
			point.m_w = 1.0f;
			dMatrix matrix (GetIdentityMatrix());
			matrix.m_posit = point;
			matrix.m_posit.m_y = FindFloor (system.m_world, point.m_x, point.m_z) + 1.2f;

			RagDoll* ragdoll;
			ragdoll = RagDoll::Create (ragDoll, bonesCount, gymnastDefinition, &system, system.m_world, matrix);
		}
	}
*/
}



void DescreteRagDoll (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	CreateLevelMesh (scene, "flatPlane.ngd", true);

/*
	dVector posit (0.0f, 0.0f, 0.0f, 0.0f);
	posit.m_y = FindFloor (system.m_world, 0.0f, 0.0f) + 2.0f;
	InitEyePoint (dVector (1.0f, 0.0f, 0.0f), posit);


	OGLModel ragDoll;
	char fullPathName[2048];
	GetWorkingFileName ("snowman.dae", fullPathName);
	OGLLoaderContext context;
	dMatrix rotMatrix (dYawMatrix (-3.14159265f * 0.5f));

_ASSERTE (0);
//	ragDoll.LoadCollada(fullPathName, context, rotMatrix, 1.0f);

	int bonesCount = sizeof (snowManDefinition) / sizeof (snowManDefinition[0]);
	for (int x = 0; x < 3; x ++) {
		for (int z = 0; z < 3; z ++) {
			dVector point (cameraEyepoint + dVector (x * 3.0f + 5.0f, 0.0f, z * 3.0f, 1.0f));
			point.m_w = 1.0f;
			dMatrix matrix (GetIdentityMatrix());
			matrix.m_posit = point;
			matrix.m_posit.m_y = FindFloor (system.m_world, point.m_x, point.m_z) + 1.2f;

			RagDoll* ragdoll;
			ragdoll = RagDoll::Create (ragDoll, bonesCount, snowManDefinition, &system, system.m_world, matrix);
		}
	}
*/

	dVector origin (-10.0f, 2.0f, 0.0f, 0.0f);
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
}



