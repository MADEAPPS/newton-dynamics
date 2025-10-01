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

class ndPlayerCapsuleController : public ndModelNotify
{
	class ndAnimationBlendTransition : public ndAnimationTwoWayBlend
	{
		public:
		ndAnimationBlendTransition(const ndSharedPtr<ndAnimationBlendTreeNode>& node0, const ndSharedPtr<ndAnimationBlendTreeNode>& node1)
			:ndAnimationTwoWayBlend(node0, node1)
			, m_paramMemory(0.0f)
		{
		}

		void SetTransition(ndFloat32 param)
		{
			m_paramMemory = m_paramMemory + 0.15f * (param - m_paramMemory);
			SetParam(m_paramMemory);
		}

		ndFloat32 m_paramMemory;
	};

	class ndPlayerCamera : public ndDemoCameraNodeFollow
	{
		public:
		ndPlayerCamera(ndBodyPlayerCapsule* const playerBody, ndRender* const owner, const ndVector& pivot, ndFloat32 distance)
			:ndDemoCameraNodeFollow(owner, pivot, distance)
			,m_playerBody(playerBody)
			,m_cameraAngle(0.0f)
			,m_headingAngle(0.0f)
		{
		}

		virtual ndMatrix CalculateLocalTransform() const override
		{
			const ndMatrix camMatrix(m_parent->CalculateGlobalTransform().OrthoInverse());

			ndMatrix playerMatrix(m_playerBody->GetMatrix());
			playerMatrix.m_posit.m_y += ndFloat32 (2.5f);

			// for debugging
			#if 0
				playerMatrix = ndGetIdentityMatrix();
				playerMatrix.m_posit = m_playerBody->GetMatrix().m_posit;
				playerMatrix.m_posit.m_y += ndFloat32(2.5f);
			#endif

			ndMatrix const pithcAngle(ndRollMatrix(m_pitch));

			return pithcAngle * playerMatrix * camMatrix;
		}

		ndFloat32 CameraHeadingAngle()
		{
			const ndFloat32 headingSpeed = ndFloat32(0.25f);
			m_cameraAngle = ndAnglesAdd(m_cameraAngle, m_yaw);
			m_headingAngle = ndAnglesAdd(m_headingAngle, headingSpeed * ndAnglesSub(m_cameraAngle, m_headingAngle));
			m_yaw = ndFloat32(0.0f);
			return m_headingAngle;
		}

		ndBodyPlayerCapsule* m_playerBody;
		ndFloat32 m_cameraAngle;
		ndFloat32 m_headingAngle;
	};

	public:
	ndPlayerCapsuleController(ndDemoEntityManager* const scene, const ndSharedPtr<ndBody>& body)
		:ndModelNotify()
		,m_scene(scene)
		,m_playerBody(body)
		,m_camera(nullptr)
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
		ndSharedPtr<ndBody> playerBody(new ndBasicPlayerCapsule(scene, entityDuplicate, localAxis, location, mass, radio, height, height / 4.0f));
		ndSharedPtr<ndModel> model(new ndModel());
		ndSharedPtr<ndModelNotify> controller(new ndPlayerCapsuleController(scene, playerBody));
		model->SetNotifyCallback(controller);
		
		// add body and mesh to the world
		ndWorld* const world = scene->GetWorld();
		scene->AddEntity(entityDuplicate);
		world->AddBody(playerBody);
		world->AddModel(model);

		ndPlayerCapsuleController* const playerCapsule = (ndPlayerCapsuleController*)*controller;
		playerCapsule->BindAnimations(loader);
		return controller;
	}

	void BindAnimations(ndMeshLoader& loader)
	{
		ndDemoEntityNotify* const playerNotify = (ndDemoEntityNotify*)(m_playerBody->GetAsBodyKinematic()->GetNotifyCallback());
		ndSharedPtr<ndRenderSceneNode> playerMesh(playerNotify->GetUserData());

		ndSharedPtr<ndAnimationSequence> idleSequence(loader.FindSequence(ndGetWorkingFileName("mocap_idle.fbx")));
		ndSharedPtr<ndAnimationSequence> walkSequence(loader.FindSequence(ndGetWorkingFileName("mocap_walk.fbx")));
		ndAssert(*idleSequence);
		ndAssert(*walkSequence);

		// create bind pose to animation sequences.
		const ndList<ndAnimationKeyFramesTrack>& tracks = walkSequence->GetTracks();
		for (ndList<ndAnimationKeyFramesTrack>::ndNode* node = tracks.GetFirst(); node; node = node->GetNext())
		{
			ndAnimationKeyFramesTrack& track = node->GetInfo();
			const ndString& name = track.GetName();
			ndRenderSceneNode* const ent = playerMesh->FindByName(name.GetStr());
			ndAssert(ent);

			ndAnimKeyframe keyFrame;
			keyFrame.m_userData = ent;
			m_keyFrameOutput.PushBack(keyFrame);
		}

		// create an animation blend tree
		ndSharedPtr<ndAnimationBlendTreeNode> idle (new ndAnimationSequencePlayer(idleSequence));
		ndSharedPtr<ndAnimationBlendTreeNode> walk (new ndAnimationSequencePlayer(walkSequence));
		//ndAnimationSequencePlayer* const run = new ndAnimationSequencePlayer(runSequence);
		
		m_idleWalkBlend = ndSharedPtr<ndAnimationBlendTreeNode>(new ndAnimationBlendTransition(idle, walk));
		m_animBlendTree = ndSharedPtr<ndAnimationBlendTreeNode>(m_idleWalkBlend);
	}

	void SetCamera()
	{
		// create a follow camera and set as teh active camera
		const ndVector cameraPivot(0.0f, 0.5f, 0.0f, 0.0f);
		ndRender* const renderer = *m_scene->GetRenderer();
		ndSharedPtr<ndRenderSceneNode> camera(new ndPlayerCamera(m_playerBody->GetAsBodyPlayerCapsule(), renderer, cameraPivot, ND_THIRD_PERSON_CAMERA_DIST));
		renderer->SetCamera(camera);
		m_camera = (ndPlayerCamera*)*camera;

		// attach the camera to the pivot node
		ndDemoEntityNotify* const playerNotify = (ndDemoEntityNotify*)(m_playerBody->GetAsBodyKinematic()->GetNotifyCallback());
		ndSharedPtr<ndRenderSceneNode> playerMesh(playerNotify->GetUserData());
		ndRenderSceneNode* const cameraNodePivot = playerMesh->FindByName("cameraPivot");
		ndAssert(cameraNodePivot);
		cameraNodePivot->AddChild(camera);
	}

	private:
	virtual void PostTransformUpdate(ndFloat32 timestep) override
	{
		ndModelNotify::PostTransformUpdate(timestep);

		ndBasicPlayerCapsule* const player = (ndBasicPlayerCapsule*)m_playerBody->GetAsBodyPlayerCapsule();
		ndAssert(player);

		const ndQuaternion rot(player->GetRotation());

		for (ndInt32 i = 0; i < m_keyFrameOutput.GetCount(); ++i)
		{
			const ndAnimKeyframe& keyFrame = m_keyFrameOutput[i];
			ndRenderSceneNode* const entity = (ndRenderSceneNode*)keyFrame.m_userData;
			if (entity)
			{
				entity->SetTransform(keyFrame.m_rotation, keyFrame.m_posit);
			}
		}

		ndFloat32 timestepSign = ndFloat32(1.0f);
		ndAnimationBlendTransition* const blender = (ndAnimationBlendTransition*)*m_idleWalkBlend;
		if (m_scene->GetKeyState(ImGuiKey_W))
		{
			blender->SetTransition(1.0f);
		}
		else if (m_scene->GetKeyState(ImGuiKey_S))
		{
			timestepSign = ndFloat32(-1.0f);
			blender->SetTransition(1.0f);
		}
		else
		{
			blender->SetTransition(0.0f);
		}
		
		ndVector velocity;
		m_animBlendTree->Update(timestep * timestepSign);
		m_animBlendTree->Evaluate(m_keyFrameOutput, velocity);

		player->m_playerInput.m_forwardSpeed = velocity.m_x;
		
		if (m_camera)
		{
			player->m_playerInput.m_heading = m_camera->CameraHeadingAngle();
		}
		else
		{
			ndAssert(0);
		}
	}

	ndAnimationPose m_keyFrameOutput;
	ndDemoEntityManager* m_scene;
	ndSharedPtr<ndBody> m_playerBody;
	ndPlayerCamera* m_camera;
	ndSharedPtr<ndAnimationBlendTreeNode> m_idleWalkBlend;
	ndSharedPtr<ndAnimationBlendTreeNode> m_animBlendTree;
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

static void LoadAnimations(ndMeshLoader& loader)
{
	// load animation clips
	loader.GetAnimationSequence(ndGetWorkingFileName("mocap_idle.fbx"));
	ndSharedPtr<ndAnimationSequence> walkCycle (loader.GetAnimationSequence(ndGetWorkingFileName("mocap_walk.fbx")));

	// add the translation track
	for (ndList<ndAnimationKeyFramesTrack>::ndNode* node = walkCycle->GetTracks().GetFirst(); node; node = node->GetNext())
	{
		ndAnimationKeyFramesTrack& track = node->GetInfo();
		ndString name(track.GetName());
		name.ToLower();
		if (name.Find("hips") != -1)
		{
			ndAnimationKeyFramesTrack& translationTrack = walkCycle->GetTranslationTrack();
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

void ndPlayerCapsule_ThirdPerson (ndDemoEntityManager* const scene)
{
	// build a floor
	//ndSharedPtr<ndBody> bodyFloor(BuildFloorBox(scene, ndGetIdentityMatrix(), "marblecheckboard.png", 0.1f, true));
	ndSharedPtr<ndBody> compoundScene(BuildCompoundScene(scene, ndGetIdentityMatrix()));

	AddSomeProps(scene);
	 
	// load the visual mesh, and animations.
	ndMeshLoader loader;
	// load mesh model and skeleton
	//loader.LoadEntity(*scene->GetRenderer(), ndGetWorkingFileName("whiteMan.fbx"));
	loader.LoadEntity(*scene->GetRenderer(), ndGetWorkingFileName("humanoidRobot.fbx"));

	// load play animations stack
	LoadAnimations(loader);
	
	// create one player capsule, the mesh will be duplicated
	ndMatrix location(ndGetIdentityMatrix());
	
	ndSharedPtr<ndModelNotify> modelNotity0(ndPlayerCapsuleController::CreatePlayer(scene, loader, location));
	ndPlayerCapsuleController* const playerController0 = (ndPlayerCapsuleController*)*modelNotity0;
	playerController0->SetCamera();

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
