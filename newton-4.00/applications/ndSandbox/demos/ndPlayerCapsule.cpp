/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndMeshLoader.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoCameraNode.h"
#include "ndContactCallback.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndBasicPlayerCapsule.h"
#include "ndDemoCameraNodeFollow.h"

#define ND_THIRD_PERSON_CAMERA_DIST ndFloat32(-5.0f)

#if 0
class ndMopcapRetargetMeshLoader : public ndMeshLoader
{
	public:
	ndMopcapRetargetMeshLoader(ndFloat32 scale)
		:ndMeshLoader()
		,m_scale(scale)
	{
	}

	virtual ~ndMopcapRetargetMeshLoader()
	{
	}

	//ndMesh* LoadMesh(const char* const fbxMeshName, bool loadAnimation)
	ndSharedPtr<ndRenderSceneNode> LoadEntity(ndRender* const renderer, const ndString& fbxPathMeshName) override
	{
		ndSharedPtr<ndRenderSceneNode> mesh (ndMeshLoader::LoadEntity(renderer, fbxPathMeshName));

		//ndMesh* mesh = ndMeshLoader::LoadMesh(fbxMeshName, loadAnimation);

		//// re target animation names
		//ndList<ndSharedPtr<ndMeshEffect>> meshes;
		//for (ndMesh* node = mesh->GetFirstIterator(); node; node = node->GetNextIterator())
		//{
		//	ndSharedPtr<ndMeshEffect> meshEffect(node->GetMesh());
		//	if (*meshEffect && meshEffect->GetVertexWeights().GetCount())
		//	{
		//		meshes.Append(meshEffect);
		//	}
		//}
		//if (meshes.GetCount())
		//{
		//	for (ndMesh* node = mesh->GetFirstIterator(); node; node = node->GetNextIterator())
		//	{
		//		const char* const ptr = strchr(node->GetName().GetStr(), ':');
		//		if (ptr)
		//		{
		//			ndInt32 newHashId = ndInt32(ndCRC64(ptr + 1) & 0xffffffff);
		//			ndInt32 oldHashId = ndInt32(ndCRC64(node->GetName().GetStr()) & 0xffffffff);
		//			for (ndList<ndSharedPtr<ndMeshEffect>>::ndNode* meshNode = meshes.GetFirst(); meshNode; meshNode = meshNode->GetNext())
		//			{
		//				ndArray<ndMeshEffect::ndVertexWeight>& weights = meshNode->GetInfo()->GetVertexWeights();
		//				for (ndInt32 i = 0; i < weights.GetCount(); ++i)
		//				{
		//					ndMeshEffect::ndVertexWeight& w = weights[i];
		//					for (ndInt32 j = ND_VERTEX_WEIGHT_SIZE - 1; j >= 0; --j)
		//					{
		//						if (w.m_boneId[j] == oldHashId)
		//						{
		//							w.m_boneId[j] = newHashId;
		//						}
		//					}
		//				}
		//			}
		//		}
		//	}
		//}
		//
		//// re target bone names
		//for (ndMesh* node = mesh->GetFirstIterator(); node; node = node->GetNextIterator())
		//{
		//	const char* const ptr = strchr(node->GetName().GetStr(), ':');
		//	if (ptr)
		//	{
		//		node->SetName(ptr + 1);
		//	}
		//}
		//
		//if (m_scale != ndFloat32(1.0f))
		//{
		//	ndMatrix scaleMatrix(ndGetIdentityMatrix());
		//	scaleMatrix[0][0] = m_scale;
		//	scaleMatrix[1][1] = m_scale;
		//	scaleMatrix[2][2] = m_scale;
		//	mesh->ApplyTransform(scaleMatrix);
		//}
		//
		//static int xxxxx;
		//if (loadAnimation)
		//{
		//	if (xxxxx==0)
		//	{
		//		//ndMesh::Save(mesh, "xxx.ndm");
		//		//delete mesh;
		//		//mesh = ndMesh::Load("xxx.ndm");
		//	}
		//	xxxxx++;
		//}

		return mesh;
	}

	virtual ndAnimationSequence* LoadAnimation(const char* const clipName)
	{
		ndAnimationSequence* const sequence = ndMeshLoader::LoadAnimation(clipName);
		ndAssert(sequence);

		// extract translation for hip node.
		if (!strcmp(clipName, "mocap_walk.fbx"))
		{
			for (ndList<ndAnimationKeyFramesTrack>::ndNode* node = sequence->GetTracks().GetFirst(); node; node = node->GetNext())
			{
				ndAnimationKeyFramesTrack& track = node->GetInfo();
				if (track.GetName() == "Hips")
				{
					ndAnimationKeyFramesTrack& translationTrack = sequence->GetTranslationTrack();
					ndVector translation(ndVector::m_zero);
					ndReal offset = ndReal(track.m_position[0].m_x);
					for (ndInt32 i = 0; i < track.m_position.GetCount(); ++i)
					{
						translation.m_x = track.m_position[i].m_x - offset;
						translationTrack.m_position.PushBack(translation);
						translationTrack.m_position.m_time.PushBack(track.m_position.m_time[i]);
						track.m_position[i].m_x = offset;
					}
					break;
				}
			}
		}

		return sequence;
	}

	ndFloat32 m_scale;
};
#endif


class ndPlayerCapsuleController : public ndModelNotify
{
	public:
	ndPlayerCapsuleController(ndDemoEntityManager* const scene, const ndSharedPtr<ndBody>& body)
		:ndModelNotify()
		,m_scene(scene)
		,m_playerBody(body)
		,m_camera(nullptr)
		,m_cameraAngle(0.0f)
	{
	}

	static ndSharedPtr<ndModelNotify> CreatePlayer(
		ndDemoEntityManager* const scene,
		ndMeshLoader& loader,
		const ndMatrix& matrix)
	{
		ndMatrix location(matrix);
		location.m_posit.m_y += 2.0f;
		
		ndMatrix localAxis(ndGetIdentityMatrix());
		localAxis[0] = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
		localAxis[1] = ndVector(1.0f, 0.0f, 0.0f, 0.0f);
		localAxis[2] = localAxis[0].CrossProduct(localAxis[1]);
		
		ndFloat32 height = 1.9f;
		ndFloat32 radio = 0.15f;
		ndFloat32 mass = 100.0f;
		ndSharedPtr<ndRenderSceneNode> entityDuplicate(loader.m_renderMesh->Clone());
		//ndSharedPtr<ndRenderSceneNode> entityDuplicate(loader.m_renderMesh);
		ndSharedPtr<ndBody> playerBody(new ndBasicPlayerCapsule(scene, loader, entityDuplicate, localAxis, location, mass, radio, height, height / 4.0f, true));

		ndSharedPtr<ndModel> model(new ndModel());
		ndSharedPtr<ndModelNotify> controller(new ndPlayerCapsuleController(scene, playerBody));
		model->SetNotifyCallback(controller);
		
		// add body and mesh to the world
		ndWorld* const world = scene->GetWorld();
		scene->AddEntity(entityDuplicate);
		world->AddBody(playerBody);
		world->AddModel(model);
		return controller;
	}

	void SetCamera()
	{
		// create a follow camera and set as teh active camera
		const ndVector cameraPivot(0.0f, 0.5f, 0.0f, 0.0f);
		ndRender* const renderer = *m_scene->GetRenderer();
		ndSharedPtr<ndRenderSceneNode> camera(new ndDemoCameraNodeFollow(renderer, cameraPivot, ND_THIRD_PERSON_CAMERA_DIST));
		renderer->SetCamera(camera);
		m_camera = (ndDemoCameraNodeFollow*)*camera;

		// attach the camera to the pivot node
		ndDemoEntityNotify* const playerNotify = (ndDemoEntityNotify*)(m_playerBody->GetAsBodyKinematic()->GetNotifyCallback());
		ndSharedPtr<ndRenderSceneNode> playerMesh(playerNotify->GetUserData());
		ndRenderSceneNode* const cameraNodePivot = playerMesh->FindByName("cameraPivot");
		ndAssert(cameraNodePivot);
		cameraNodePivot->AddChild(camera);
	}
	private:
	void Update(ndFloat32 timestep) override
	{
		ndModelNotify::Update(timestep);

		ndBasicPlayerCapsule* const player = (ndBasicPlayerCapsule*)m_playerBody->GetAsBodyPlayerCapsule();
		ndAssert(player);
		player->m_playerInput.m_forwardSpeed = ndFloat32 (0.0f);
		if (m_scene->GetKeyState(ImGuiKey_W))
		{
			player->m_playerInput.m_forwardSpeed = ndFloat32(2.0f);
		}
		else if (m_scene->GetKeyState(ImGuiKey_S))
		{
			player->m_playerInput.m_forwardSpeed = ndFloat32(-1.0f);
		}

		const ndFloat32 headingSpeed = ndFloat32(0.25f);
		m_cameraAngle = ndAnglesAdd(m_cameraAngle, m_camera->m_yaw);
		m_headingAngle = ndAnglesAdd(m_headingAngle, headingSpeed * ndAnglesSub(m_cameraAngle, m_headingAngle));
		player->m_playerInput.m_heading = m_headingAngle;

		m_camera->m_yaw = ndFloat32(0.0f);
	}

	ndDemoEntityManager* m_scene;
	ndSharedPtr<ndBody> m_playerBody;
	ndDemoCameraNodeFollow* m_camera;
	ndFloat32 m_cameraAngle;
	ndFloat32 m_headingAngle;
};

//static void AddSomeProps(ndDemoEntityManager* const scene)
static void AddSomeProps(ndDemoEntityManager* const)
{
	//class PlaceMatrix : public ndMatrix
	//{
	//	public:
	//	PlaceMatrix(ndFloat32 x, ndFloat32 y, ndFloat32 z)
	//		:ndMatrix(ndGetIdentityMatrix())
	//	{
	//		m_posit.m_x = x;
	//		m_posit.m_y = y;
	//		m_posit.m_z = z;
	//	}
	//};
	//AddCapsulesStacks___(scene, PlaceMatrix(32.0f, 0.0f, 0.0f), 10.0f, 0.5f, 0.5f, 1.0f, 10, 10, 7);
	//AddBox(scene, PlaceMatrix(10.0f, 0.0f, 0.0f), 30.0f, 2.0f, 0.25f, 2.5f);
	//AddBox(scene, PlaceMatrix(10.0f, 0.5f, 1.125f), 30.0f, 2.0f, 0.25f, 2.5f);
	//AddBox(scene, PlaceMatrix(10.0f, 1.0f, 1.250f), 30.0f, 2.0f, 0.25f, 2.5f);
}

void ndPlayerCapsule_ThirdPerson (ndDemoEntityManager* const scene)
{
	// build a floor
	//ndSharedPtr<ndBody> bodyFloor(BuildFloorBox(scene, ndGetIdentityMatrix(), "blueCheckerboard.png", 0.1f, true));
	ndSharedPtr<ndBody> bodyFloor(BuildFloorBox(scene, ndGetIdentityMatrix(), "marblecheckboard.png", 0.1f, true));

	AddSomeProps(scene);
	 
	// load the visual mesh, and animations.
	ndMeshLoader loader;
	loader.LoadEntity(*scene->GetRenderer(), ndGetWorkingFileName("whiteMan.fbx"));
	//loader.LoadEntity(*scene->GetRenderer(), ndGetWorkingFileName("humanoidRobot.fbx"));
	
	// create one player capsule, the mesh will be duplicated
	ndMatrix location(ndGetIdentityMatrix());
	
	ndSharedPtr<ndModelNotify> modelNotity0(ndPlayerCapsuleController::CreatePlayer(scene, loader, location));
	ndPlayerCapsuleController* const playerController0 = (ndPlayerCapsuleController*)*modelNotity0;
	playerController0->SetCamera();

	////loader.LoadEntity(*scene->GetRenderer(), ndGetWorkingFileName("tpot.fbx"));
	//ndSharedPtr<ndRenderSceneNode> entityDuplicate(loader.m_renderMesh->Clone());
	////ndSharedPtr<ndRenderSceneNode> entityDuplicate(loader.m_renderMesh);
	////ndSharedPtr<ndRenderSceneNode> entityDuplicate(loader1.m_renderMesh);
	//scene->AddEntity(entityDuplicate);
#if 0		
	//ndSharedPtr<ndBody> player1(new ndBasicPlayerCapsule(scene, loader, *entity, localAxis, location, mass, radio, height, height/4.0f));
	//world->AddBody(player1);

	location.m_posit.m_z += 2.0f;
	ndSharedPtr<ndBody> player2(new ndBasicPlayerCapsule(scene, loader, *entity, localAxis, location, mass, radio, height, height / 4.0f));
	//world->AddBody(player2);
	
	location.m_posit.m_z += 2.0f;
	ndSharedPtr<ndBody> player3(new ndBasicPlayerCapsule(scene, loader, *entity, localAxis, location, mass, radio, height, height / 4.0f));
	//world->AddBody(player3);
#endif

}
