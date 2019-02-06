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


enum dRigType
{
	m_root,
	m_bone,
};

struct dSkeletonRigDefinition
{
	const char* m_name;
	dRigType m_type;
};


static dSkeletonRigDefinition inverseKinematicsRidParts[] =
{
	{ "mixamorig:Hips", m_root},
	{ "mixamorig:RightUpLeg", m_bone},
	{ "mixamorig:LeftUpLeg", m_bone},
//	{ "mixamorig:RightLeg" },
//	{ "mixamorig:RightFoot" },
//	{ "mixamorig:RightToeBase" },
//	{ "mixamorig:RightToe_End" },

//	{ "mixamorig:LeftLeg" },
//	{ "mixamorig:LeftFoot" },
//	{ "mixamorig:LeftToeBase" },
//	{ "mixamorig:LeftToe_End" },
//	{ "mixamorig:Spine" },
//	{ "mixamorig:Spine1" },
//	{ "mixamorig:Spine2" },
//	{ "mixamorig:RightShoulder" },
//	{ "mixamorig:RightArm" },
//	{ "mixamorig:RightForeArm" },
//	{ "mixamorig:RightHand" },
//	{ "mixamorig:LeftShoulder" },
//	{ "mixamorig:LeftArm" },
//	{ "mixamorig:LeftForeArm" },
//	{ "mixamorig:LeftHand" },
//	{ "mixamorig:Neck" },
//	{ "mixamorig:Head" },
//	{ "mixamorig:HeadTop_End" },
};



class InverseKinematicAnimationManager: public dAnimIKManager
{
	public:
/*
	class dAnimCharacterUserData: public DemoEntity::UserData
	{
		public:
		dAnimCharacterUserData(dAnimIKController* const controller)
			:DemoEntity::UserData()
			,m_controller(controller)
//			,m_walk(walk)
//			,m_posture(posture)
//			,m_hipHigh(0.0f)
//			,m_walkSpeed(0.0f)
		{
		}

		void OnRender(dFloat timestep) const
		{
		}

		dAnimIKController* m_controller;
		//dAnimIKController* m_rig;
		//dAnimationEffectorBlendTwoWay* m_walk;
		//dAnimationBipeHipController* m_posture;
		//dFloat m_hipHigh;
		//dFloat m_walkSpeed;
	};
*/

	InverseKinematicAnimationManager(DemoEntityManager* const scene)
		:dAnimIKManager(scene->GetNewton())
		,m_currentRig(NULL)
	{
		scene->Set2DDisplayRenderFunction(RenderHelpMenu, NULL, this);
	}

	~InverseKinematicAnimationManager()
	{
		while (m_animCache) {
			dAnimTakeData* const data = m_animCache.GetRoot()->GetInfo();
			data->Release();
			m_animCache.Remove(m_animCache.GetRoot());
		}
	}

	static void RenderHelpMenu(DemoEntityManager* const scene, void* const context)
	{
/*
		InverseKinematicAnimationManager* const me = (InverseKinematicAnimationManager*)context;
		if (me->m_currentRig) {
			DemoEntity* const entiry = (DemoEntity*)NewtonBodyGetUserData(me->m_currentRig->GetNewtonBody());
			dAnimationCharacterUserData* const controlData = (dAnimationCharacterUserData*)entiry->GetUserData();

			dVector color(1.0f, 1.0f, 0.0f, 0.0f);
			scene->Print(color, "Sliders control");

			dFloat32 val0 = dFloat32(controlData->m_walkSpeed);
			ImGui::SliderFloat_DoubleSpace("walkSpeed", &val0, 0.0f, 1.0f);
			controlData->m_walkSpeed = val0;

			dFloat32 val1 = dFloat32(controlData->m_hipHigh);
			ImGui::SliderFloat_DoubleSpace("hip high", &val1, -0.5f, 1.5f);
			controlData->m_hipHigh = val1;
		}
*/
	}

	void OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
	{
		dAnimIKManager::OnDebug(debugContext);
		//for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
		//	dSixAxisController* const controller = &node->GetInfo();
		//	controller->Debug(debugContext);
		//}
	}
/*
	DemoEntity* FindMesh(const DemoEntity* const bodyPart) const
	{
		for (DemoEntity* child = bodyPart->GetChild(); child; child = child->GetSibling()) {
			if (child->GetMesh()) {
				return child;
			}
		}
		dAssert(0);
		return NULL;
	}

	NewtonCollision* MakeConvexHull(const DemoEntity* const bodyPart) const
	{
		dVector points[1024 * 16];

		const DemoEntity* const meshEntity = FindMesh(bodyPart);

		DemoMesh* const mesh = (DemoMesh*)meshEntity->GetMesh();
		dAssert(mesh->IsType(DemoMesh::GetRttiType()));
		dAssert(mesh->m_vertexCount && (mesh->m_vertexCount < int(sizeof(points) / sizeof(points[0]))));

		// go over the vertex array and find and collect all vertices's weighted by this bone.
		const dFloat* const array = mesh->m_vertex;
		for (int i = 0; i < mesh->m_vertexCount; i++) {
			points[i][0] = array[i * 3 + 0];
			points[i][1] = array[i * 3 + 1];
			points[i][2] = array[i * 3 + 2];
			points[i][3] = 0.0f;
		}
		dMatrix matrix(meshEntity->GetMeshMatrix());
		matrix = matrix * meshEntity->GetCurrentMatrix();
		//matrix = matrix * bodyPart->GetParent()->GetCurrentMatrix();
		matrix.TransformTriplex(&points[0][0], sizeof(dVector), &points[0][0], sizeof(dVector), mesh->m_vertexCount);
		//return NewtonCreateConvexHull(GetWorld(), mesh->m_vertexCount, &points[0][0], sizeof(dVector), 1.0e-3f, SERVO_VEHICLE_DEFINITION::m_bodyPart, NULL);
		return NewtonCreateConvexHull(GetWorld(), mesh->m_vertexCount, &points[0][0], sizeof(dVector), 1.0e-3f, 0, NULL);
	}

	NewtonBody* CreateBodyPart(DemoEntity* const bodyPart, const dRagDollConfig& definition)
	{
		NewtonCollision* const shape = MakeConvexHull(bodyPart);

		// calculate the bone matrix
		dMatrix matrix(bodyPart->CalculateGlobalMatrix());

		NewtonWorld* const world = GetWorld();

		// create the rigid body that will make this bone
		NewtonBody* const body = NewtonCreateDynamicBody(world, shape, &matrix[0][0]);

		// destroy the collision helper shape 
		NewtonDestroyCollision(shape);

		// get the collision from body
		NewtonCollision* const collision = NewtonBodyGetCollision(body);

		// calculate the moment of inertia and the relative center of mass of the solid
		NewtonBodySetMassProperties(body, definition.m_mass, collision);

		// save the user lifterData with the bone body (usually the visual geometry)
		NewtonBodySetUserData(body, bodyPart);

		// assign a body part id
		//NewtonCollisionSetUserID(collision, definition.m_bodyPartID);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);
		return body;
	}

	void OnUpdateTransform(const dAnimIDRigJoint* const bone, const dMatrix& localMatrix) const
	{
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(GetWorld());
		NewtonBody* const newtonBody = bone->GetNewtonBody();
		DemoEntity* const meshEntity = (DemoEntity*)NewtonBodyGetUserData(newtonBody);

		dQuaternion rot(localMatrix);
		meshEntity->SetMatrix(*scene, rot, localMatrix.m_posit);
	}
*/

	void UpdatePlayer(dAnimIKController* const controller, dFloat timestep) 
	{
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(GetWorld());
		dAnimIKManager::UpdatePlayer(controller, timestep);

//		const dAnimPose& pose = controller->GetBasePose();
		const dAnimPose& pose = controller->GetAnimationTree()->GetPose();
		for (dAnimPose::dListNode* node = pose.GetFirst(); node; node = node->GetNext()) {
			const dAnimKeyframe& frame = node->GetInfo();
			DemoEntity* const entity = (DemoEntity*)frame.m_userData;

//if (
//(entity->GetName() == "mixamorig:Hips") ||
//(entity->GetName() == "mixamorig:RightUpLeg") ||
//(entity->GetName() == "mixamorig:RightLeg") ||
//(entity->GetName() == "mixamorig:RightFoot") ||
//(entity->GetName() == "mixamorig:RightToeBase") ||
//(entity->GetName() == "mixamorig:LeftLeg") ||
//(entity->GetName() == "mixamorig:LeftUpLeg") ||
//(entity->GetName() == "mixamorig:LeftShoulder") ||
//(entity->GetName() == "mixamorig:Spine1") ||
//(entity->GetName() == "Bone002") ||
//(entity->GetName() == "xxxxxxxx")) 
//{
//dVector euler0;
//dVector euler1;
//dMatrix xxxx(frame.m_rotation, frame.m_posit);
//xxxx.GetEulerAngles(euler0, euler1);
//euler0 = euler0.Scale(dRadToDegree);
//dTrace(("%s %f %f  %f\n", entity->GetName().GetStr(), euler0.m_x, euler0.m_y, euler0.m_z));
//frame.m_rotation = dQuaternion (dYawMatrix(15.0f * dDegreeToRad)) * frame.m_rotation; 

			entity->SetMatrix(*scene, frame.m_rotation, frame.m_posit);
//}
		
		}
	}

	void PreUpdate(dFloat timestep)
	{

/*
		if (m_currentRig) {
			DemoEntity* const entiry = (DemoEntity*)NewtonBodyGetUserData(m_currentRig->GetNewtonBody());
			dAnimationCharacterUserData* const controlData = (dAnimationCharacterUserData*)entiry->GetUserData();

			dAnimationEffectorBlendTwoWay* const walkBlend = controlData->m_walk;
			walkBlend->SetParam(controlData->m_walkSpeed);

			dAnimationBipeHipController* const posture = controlData->m_posture;
			posture->m_position.m_y = 0.25f * controlData->m_hipHigh;
		}
*/
		dAnimIKManager::PreUpdate(timestep);
	}

	dAnimTakeData* LoadAnimation(dAnimIKController* const controller, const char* const animName)
	{
		dTree<dAnimTakeData*, dString>::dTreeNode* cachedAnimNode = m_animCache.Find(animName);
		if (!cachedAnimNode) {
			dScene scene(GetWorld());
			char pathName[2048];
			dGetWorkingFileName(animName, pathName);
			scene.Deserialize(pathName);

			dScene::dTreeNode* const animTakeNode = scene.FindChildByType(scene.GetRootNode(), dAnimationTake::GetRttiType());
			if (animTakeNode) {
				dTree<dAnimTakeData::dAnimTakeTrack*, dString> map;
				const dAnimPose& basePose = controller->GetBasePose();

				dAnimTakeData* const animdata = new dAnimTakeData(basePose.GetCount());
				dAnimationTake* const animTake = (dAnimationTake*)scene.GetInfoFromNode(animTakeNode);
				animdata->SetPeriod(animTake->GetPeriod());

				cachedAnimNode = m_animCache.Insert(animdata, animName);

				dList<dAnimTakeData::dAnimTakeTrack>& tracks = animdata->GetTracks();
				dList<dAnimTakeData::dAnimTakeTrack>::dListNode* ptr = tracks.GetFirst();
				for (dAnimPose::dListNode* ptrNode = basePose.GetFirst(); ptrNode; ptrNode = ptrNode->GetNext()) {
					DemoEntity* const entity = (DemoEntity*)ptrNode->GetInfo().m_userData;
					map.Insert(&ptr->GetInfo(), entity->GetName());
					ptr = ptr->GetNext();
				}

				for (void* link = scene.GetFirstChildLink(animTakeNode); link; link = scene.GetNextChildLink(animTakeNode, link)) {
					dScene::dTreeNode* const node = scene.GetNodeFromLink(link);
					dAnimationTrack* const srcTrack = (dAnimationTrack*)scene.GetInfoFromNode(node);
					if (srcTrack->IsType(dAnimationTrack::GetRttiType())) {

						dTree<dAnimTakeData::dAnimTakeTrack*, dString>::dTreeNode* const ptrNode = map.Find(srcTrack->GetName());
						dAssert(ptrNode);
						dAnimTakeData::dAnimTakeTrack* const dstTrack = ptrNode->GetInfo();

						const dList<dAnimationTrack::dCurveValue>& rotations = srcTrack->GetRotations();
						dstTrack->m_rotation.Resize(rotations.GetCount());
						int index = 0;
						for (dList<dAnimationTrack::dCurveValue>::dListNode* node = rotations.GetFirst(); node; node = node->GetNext()) {
							dAnimationTrack::dCurveValue keyFrame (node->GetInfo());

							dMatrix matrix(dPitchMatrix(keyFrame.m_x) * dYawMatrix(keyFrame.m_y) * dRollMatrix(keyFrame.m_z));
							dQuaternion rot(matrix);
							dstTrack->m_rotation[index].m_rotation = rot;
							dstTrack->m_rotation[index].m_time = keyFrame.m_time;
							index++;
						}

						for (int i = 0; i < rotations.GetCount() - 1; i++) {
							dFloat dot = dstTrack->m_rotation[i].m_rotation.DotProduct(dstTrack->m_rotation[i + 1].m_rotation);
							if (dot < 0.0f) {
								dstTrack->m_rotation[i + 1].m_rotation.m_x *= -1.0f;
								dstTrack->m_rotation[i + 1].m_rotation.m_y *= -1.0f;
								dstTrack->m_rotation[i + 1].m_rotation.m_z *= -1.0f;
								dstTrack->m_rotation[i + 1].m_rotation.m_w *= -1.0f;
							}
						}

						const dList<dAnimationTrack::dCurveValue>& positions = srcTrack->GetPositions();
						dstTrack->m_position.Resize(positions.GetCount());
						index = 0;
						for (dList<dAnimationTrack::dCurveValue>::dListNode* node = positions.GetFirst(); node; node = node->GetNext()) {
							dAnimationTrack::dCurveValue keyFrame(node->GetInfo());
							dstTrack->m_position[index].m_posit = dVector(keyFrame.m_x, keyFrame.m_y, keyFrame.m_z, dFloat(1.0f));
							dstTrack->m_position[index].m_time = keyFrame.m_time;
							index++;
						}
					}
				}
			}
		}

		dAssert(cachedAnimNode);
		return cachedAnimNode->GetInfo();
	}

	void PopulateBasePose(dAnimPose& basePose, DemoEntity* const character)
	{
		basePose.Clear();

		int stack = 1;
		DemoEntity* pool[32];
		pool[0] = character;

		while (stack) {
			stack--;
			DemoEntity* const entity = pool[stack];

			dAnimKeyframe& transform = basePose.Append()->GetInfo();
			dMatrix matrix(entity->GetCurrentMatrix());
			transform.m_posit = matrix.m_posit;
			transform.m_rotation = dQuaternion(matrix);
			transform.m_userData = entity;

			for (DemoEntity* node = entity->GetChild(); node; node = node->GetSibling()) {
				pool[stack] = node;
				stack++;
			}
		}
	}

	DemoEntity* FindRigRoot(DemoEntity* const character)
	{
		int stack = 1;
		DemoEntity* pool[32];
		pool[0] = character;

		const int nodesCount = sizeof(inverseKinematicsRidParts) / sizeof(inverseKinematicsRidParts[0]);

		while (stack) {
			stack--;
			DemoEntity* const entity = pool[stack];
			for (int i = 0; i < nodesCount; i++) {
				if (entity->GetName() == inverseKinematicsRidParts[i].m_name) {
					if (inverseKinematicsRidParts[i].m_type == dRigType::m_root)
					{
						return entity;
					}
					break;
				}
			}

			for (DemoEntity* node = entity->GetChild(); node; node = node->GetSibling()) {
				pool[stack] = node;
				stack++;
			}
		}
		dAssert(0);
		return NULL;
	}

	dSkeletonRigDefinition* FindDefinition(DemoEntity* const entity) const
	{
		const int nodesCount = sizeof(inverseKinematicsRidParts) / sizeof(inverseKinematicsRidParts[0]);
		for (int i = 0; i < nodesCount; i++) {
			if (entity->GetName() == inverseKinematicsRidParts[i].m_name) {
				switch (inverseKinematicsRidParts[i].m_type) 
				{
					case dRigType::m_root:
					case dRigType::m_bone:
					{
						return &inverseKinematicsRidParts[i];
					}

					default:;
				}
			}
		}
		return NULL;

	}

	dAnimIKController* CreateSkeletonRig(DemoEntity* const character)
	{
		DemoEntity* const rootEntity = FindRigRoot(character);
		dAnimIKController* const controller = CreateIKController();
		controller->SetUserData(rootEntity);

		int stack = 0;
		DemoEntity* entityStack[32];
		dAnimIKRigJoint* parentJointStack[32];

		for (DemoEntity* node = rootEntity->GetChild(); node; node = node->GetSibling()) {
			entityStack[stack] = node;
			parentJointStack[stack] = controller->GetAsIKRigJoint();
			stack ++;
		}

		while (stack) {
			stack--;
			DemoEntity* const entity = entityStack[stack];
			dAnimIKRigJoint* const parentJoint = parentJointStack[stack];
			dSkeletonRigDefinition* const definitions = FindDefinition(entity);
			if (definitions) {
				dAnimIK3dofJoint* const joint = new dAnimIK3dofJoint(parentJoint);
				joint->SetUserData(entity);

				for (DemoEntity* node = entity->GetChild(); node; node = node->GetSibling()) {
					entityStack[stack] = node;
					parentJointStack[stack] = joint;
					stack++;
				}
			}
		}

		return controller;
	}

	dAnimIKController* CreateHuman(DemoEntity* const model, const dMatrix& origin)
	{
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(GetWorld());

		DemoEntity* const character = (DemoEntity*)model->CreateClone();
		character->SetNameID("dommyRoot");
		character->ResetMatrix(*scene, character->GetCurrentMatrix() * origin);
		scene->Append(character);

		dAnimIKController* const controller = CreateSkeletonRig(character);

//		dAnimCharacterUserData* const userData = new dAnimCharacterUserData(controller);
//		character->SetUserData(userData);
		
		// populate base pose
		PopulateBasePose(controller->GetBasePose(), character);
		//dAnimTakeData* const walkCycle = LoadAnimation(controller, "whiteman_walk.ngd");
		dAnimTakeData* const walkCycle = LoadAnimation(controller, "whiteman_idle.ngd");

		dAnimIKBlendNodeTake* const walk = new dAnimIKBlendNodeTake(controller, walkCycle);
		//dAnimIKBlendNodePose* const walk = new dAnimIKBlendNodePose(controller);
		//dAnimIKBlendNodePose* const pose = new dAnimIKBlendNodePose(controller);
		dAnimIKBlendNodeRoot* const animTree = new dAnimIKBlendNodeRoot(controller, walk);
		//dAnimIKBlendNodeRoot* const animTree = new dAnimIKBlendNodeRoot(controller, pose);

		controller->SetAnimationTree(animTree);
		return controller;
	}

	dAnimIKController* m_currentRig;
	dTree<dAnimTakeData*, dString> m_animCache;
};

void AnimatedPlayerController(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();
	CreateLevelMesh(scene, "flatPlane.ngd", true);

	dMatrix origin (dGetIdentityMatrix());
	origin.m_posit.m_y = -0.0f;

	dMatrix origin1 (origin);
	InverseKinematicAnimationManager* const animationManager = new InverseKinematicAnimationManager(scene);

	dPointer<DemoEntity> humanModel (DemoEntity::LoadNGD_mesh("whiteman.ngd", scene->GetNewton(), scene->GetShaderCache()));

	//dAnimIKController* const human = animationManager->CreateHuman("whiteman.ngd", origin1);
	dAnimIKController* const human = animationManager->CreateHuman(&(*humanModel), origin1);
	//dAnimIKController* const human = animationManager->CreateHuman("skintest.ngd", origin1);
	

DemoEntity* const referenceModel = DemoEntity::LoadNGD_mesh("viper.ngd", scene->GetNewton(), scene->GetShaderCache());
origin1.m_posit.m_z = 2.0f;
referenceModel->ResetMatrix(*scene, referenceModel->GetCurrentMatrix() * origin1);
scene->Append(referenceModel);

dMatrix xxxx(origin1);
for (int i = 0; i < 10; i ++) {
	xxxx.m_posit.m_x += 2;
	dMatrix xxxx1(xxxx);
	for (int j = 0; j < 10; j ++) {
		xxxx1.m_posit.m_z -= 2;
		animationManager->CreateHuman(&(*humanModel), xxxx1);
	}
}
	
	origin.m_posit = dVector(-4.0f, 1.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(dGetIdentityMatrix(), origin.m_posit);
}




