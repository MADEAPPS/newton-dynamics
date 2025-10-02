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
	public:
	class ndHelpLegend : public ndDemoEntityManager::ndDemoHelper
	{
		virtual void PresentHelp(ndDemoEntityManager* const scene) override
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			scene->Print(color, "implements a basic player capsule mechanic");
			scene->Print(color, "w key for moving walking forward");
			scene->Print(color, "s key for going walking backward");
			scene->Print(color, "shift w key for running forwad");
			scene->Print(color, "left click on the scene for turning and look up and down");
			scene->Print(color, "left click on dynamics body for pickin the body");
		}
	};

	class ndAnimationBlendTransition : public ndAnimationTwoWayBlend
	{
		public:
		ndAnimationBlendTransition(const ndSharedPtr<ndAnimationBlendTreeNode>& node0, const ndSharedPtr<ndAnimationBlendTreeNode>& node1)
			:ndAnimationTwoWayBlend(node0, node1)
			,m_paramMemory(0.0f)
		{
		}

		void SetTransition(ndFloat32 param)
		{
			m_paramMemory = m_paramMemory + 0.05f * (param - m_paramMemory);
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
		,m_playerBody(body)
		,m_cameraNode(nullptr)
		,m_scene(scene)
	{
		// add a camera 
		const ndVector cameraPivot(0.0f, 0.5f, 0.0f, 0.0f);
		ndRender* const renderer = *m_scene->GetRenderer();
		m_cameraNode = ndSharedPtr<ndRenderSceneNode>(new ndPlayerCamera(m_playerBody->GetAsBodyPlayerCapsule(), renderer, cameraPivot, ND_THIRD_PERSON_CAMERA_DIST));

		// attach the camera to the pivot node
		ndDemoEntityNotify* const playerNotify = (ndDemoEntityNotify*)(m_playerBody->GetAsBodyKinematic()->GetNotifyCallback());
		ndSharedPtr<ndRenderSceneNode> playerMesh(playerNotify->GetUserData());
		ndRenderSceneNode* const cameraNodePivot = playerMesh->FindByName("cameraPivot");
		ndAssert(cameraNodePivot);
		cameraNodePivot->AddChild(m_cameraNode);
	}

	ndSharedPtr<ndRenderSceneNode> GetCamera()
	{
		return m_cameraNode;
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
		
		ndFloat32 height = ndFloat32(1.9f);
		ndFloat32 radio = ndFloat32(0.4f);
		ndFloat32 mass = ndFloat32(100.0f);
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

		ndSharedPtr<ndAnimationSequence> runSequence(loader.FindSequence(ndGetWorkingFileName("mocap_run.fbx")));
		ndSharedPtr<ndAnimationSequence> idleSequence(loader.FindSequence(ndGetWorkingFileName("mocap_idle.fbx")));
		ndSharedPtr<ndAnimationSequence> walkSequence(loader.FindSequence(ndGetWorkingFileName("mocap_walk.fbx")));
		ndAssert(*runSequence);
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
		ndSharedPtr<ndAnimationBlendTreeNode> run(new ndAnimationSequencePlayer(runSequence));
		ndSharedPtr<ndAnimationBlendTreeNode> idle (new ndAnimationSequencePlayer(idleSequence));
		ndSharedPtr<ndAnimationBlendTreeNode> walk (new ndAnimationSequencePlayer(walkSequence));
		
		m_walkRunBlend = ndSharedPtr<ndAnimationBlendTreeNode>(new ndAnimationBlendTransition(walk, run));
		m_idleWalkBlend = ndSharedPtr<ndAnimationBlendTreeNode>(new ndAnimationBlendTransition(idle, m_walkRunBlend));
		m_animBlendTree = ndSharedPtr<ndAnimationBlendTreeNode>(m_idleWalkBlend);
	}

	//void SetCamera()
	//{
	//	// create a follow camera and set as teh active camera
	//	const ndVector cameraPivot(0.0f, 0.5f, 0.0f, 0.0f);
	//	ndRender* const renderer = *m_scene->GetRenderer();
	//	ndSharedPtr<ndRenderSceneNode> camera(new ndPlayerCamera(m_playerBody->GetAsBodyPlayerCapsule(), renderer, cameraPivot, ND_THIRD_PERSON_CAMERA_DIST));
	//	renderer->SetCamera(camera);
	//	m_camera = (ndPlayerCamera*)*camera;
	//
	//	// attach the camera to the pivot node
	//	ndDemoEntityNotify* const playerNotify = (ndDemoEntityNotify*)(m_playerBody->GetAsBodyKinematic()->GetNotifyCallback());
	//	ndSharedPtr<ndRenderSceneNode> playerMesh(playerNotify->GetUserData());
	//	ndRenderSceneNode* const cameraNodePivot = playerMesh->FindByName("cameraPivot");
	//	ndAssert(cameraNodePivot);
	//	cameraNodePivot->AddChild(camera);
	//}

	private:
	void UpdateSkeleton()
	{
		for (ndInt32 i = 0; i < m_keyFrameOutput.GetCount(); ++i)
		{
			const ndAnimKeyframe& keyFrame = m_keyFrameOutput[i];
			ndRenderSceneNode* const entity = (ndRenderSceneNode*)keyFrame.m_userData;
			if (entity)
			{
				entity->SetTransform(keyFrame.m_rotation, keyFrame.m_posit);
			}
		}
	}

	void PlayerUpdate(ndFloat32 timestep)
	{
		ndBasicPlayerCapsule* const player = (ndBasicPlayerCapsule*)m_playerBody->GetAsBodyPlayerCapsule();

		UpdateSkeleton();

		ndFloat32 timestepSign = ndFloat32(1.0f);

		ndAnimationBlendTransition* const walkRunBlender = (ndAnimationBlendTransition*)*m_walkRunBlend;
		ndAnimationBlendTransition* const idleWalkBlender = (ndAnimationBlendTransition*)*m_idleWalkBlend;

		// run the input mibni state machine
		if (m_scene->GetKeyState(ImGuiKey_W))
		{
			idleWalkBlender->SetTransition(1.0f);
			if (m_scene->GetKeyState(ImGuiKey_LeftShift))
			{
				walkRunBlender->SetTransition(1.0f);
			}
			else
			{
				walkRunBlender->SetTransition(0.0f);
			}
		}
		else if (m_scene->GetKeyState(ImGuiKey_S))
		{
			timestepSign = ndFloat32(-1.0f);
			walkRunBlender->SetTransition(0.0f);
			idleWalkBlender->SetTransition(1.0f);
		}
		else
		{
			walkRunBlender->SetTransition(0.0f);
			idleWalkBlender->SetTransition(0.0f);
		}

		ndVector veloc;
		m_animBlendTree->Update(timestep * timestepSign);
		m_animBlendTree->Evaluate(m_keyFrameOutput, veloc);

		player->m_playerInput.m_forwardSpeed = veloc.m_x;

		ndPlayerCamera* const cameraNode = (ndPlayerCamera*)*m_cameraNode;
		player->m_playerInput.m_heading = cameraNode->CameraHeadingAngle();
	}

	// player to be update to a game script, for now only update the animation
	void NonPlayerUpdate(ndFloat32 timestep)
	{
		ndVector veloc;
		m_animBlendTree->Update(timestep);
		m_animBlendTree->Evaluate(m_keyFrameOutput, veloc);
		UpdateSkeleton();
	}

	virtual void PostTransformUpdate(ndFloat32 timestep) override
	{
		ndModelNotify::PostTransformUpdate(timestep);

		ndRenderSceneCamera* const playerCamera = m_cameraNode->FindCameraNode();
		ndRenderSceneCamera* const sceneCamera = m_scene->GetRenderer()->GetCamera()->FindCameraNode();
		if (playerCamera == sceneCamera)
		{
			PlayerUpdate(timestep);
		}
		else
		{
			NonPlayerUpdate(timestep);
		}
	}

	ndAnimationPose m_keyFrameOutput;
	ndSharedPtr<ndBody> m_playerBody;
	ndSharedPtr<ndAnimationBlendTreeNode> m_walkRunBlend;
	ndSharedPtr<ndAnimationBlendTreeNode> m_idleWalkBlend;
	ndSharedPtr<ndAnimationBlendTreeNode> m_animBlendTree;
	ndSharedPtr<ndRenderSceneNode> m_cameraNode;
	ndDemoEntityManager* m_scene;
};

static void AddSomeProps(ndDemoEntityManager* const scene)
{
	auto AddMerryGoRound = [scene](const ndMatrix& location)
	{
		ndMatrix matrix(ndRollMatrix(ndFloat32(90.0f) * ndDegreeToRad) * location);
		ndSharedPtr<ndBody> pole(AddCylinder(scene, matrix, ndFloat32(0.0f), ndFloat32(0.1f), ndFloat32(0.1f), ndFloat32(3.0f)));
		ndSharedPtr<ndBody> platform(AddCylinder(scene, matrix, ndFloat32(200.0f), ndFloat32(5.0f), ndFloat32(5.0f), ndFloat32(0.2f)));
		
		matrix.m_posit.m_y += 0.3f;
		platform->SetMatrix(matrix);
		platform->SetOmega(ndVector(0.0f, 2.5f, 0.0f, 0.0f));
		
		ndSharedPtr<ndJointBilateralConstraint> hinge (new ndJointHinge(matrix, platform->GetAsBodyDynamic(), pole->GetAsBodyDynamic()));
		scene->GetWorld()->AddJoint(hinge);
	};

	class PlaceMatrix : public ndMatrix
	{
		public:
		PlaceMatrix(ndFloat32 x, ndFloat32 y, ndFloat32 z)
			:ndMatrix(ndGetIdentityMatrix())
		{
			m_posit.m_x = x;
			m_posit.m_y = y;
			m_posit.m_z = z;
		}
	};
	AddBox(scene, PlaceMatrix(20.0f, 0.0f, 0.0f), 30.0f, 2.0f, 0.25f, 2.5f);
	AddBox(scene, PlaceMatrix(25.0f, 0.5f, 2.0f), 30.0f, 2.0f, 0.25f, 2.5f);
	AddBox(scene, PlaceMatrix(15.0f, 1.0f, 4.0f), 30.0f, 2.0f, 0.25f, 2.5f);
	
	AddMerryGoRound(PlaceMatrix(32.0f, 0.0f, 10.0f));
}

static void LoadAnimations(ndMeshLoader& loader)
{
	// load animation clips
	loader.GetAnimationSequence(ndGetWorkingFileName("mocap_run.fbx"));
	loader.GetAnimationSequence(ndGetWorkingFileName("mocap_walk.fbx"));
	loader.GetAnimationSequence(ndGetWorkingFileName("mocap_idle.fbx"));

	// add the translation track, that tanslation is contained in the hip bone 
	loader.SetTranslationTracks(ndString("hips"));
}

void ndPlayerCapsule_ThirdPerson (ndDemoEntityManager* const scene)
{
	// build a floor
	//ndSharedPtr<ndBody> bodyFloor(BuildPlayground(scene));
	//ndSharedPtr<ndBody> bodyFloor(BuildFloorBox(scene, ndGetIdentityMatrix(), "marblecheckboard.png", 0.1f, true));
	ndSharedPtr<ndBody> bodyFloor(BuildCompoundScene(scene, ndGetIdentityMatrix()));

	// add a help menu
	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndPlayerCapsuleController::ndHelpLegend());
	scene->SetDemoHelp(demoHelper);

	// load the visual mesh, and animations.
	ndMeshLoader loader;
	loader.LoadEntity(*scene->GetRenderer(), ndGetWorkingFileName("humanoidRobot.fbx"));

	// load play animations stack
	LoadAnimations(loader);
	
	// create one player capsule, the mesh will be duplicated
	ndMatrix location(ndGetIdentityMatrix());
	
	ndSharedPtr<ndModelNotify> modelNotity(ndPlayerCapsuleController::CreatePlayer(scene, loader, location));

	// set this player as teh active camera
	ndPlayerCapsuleController* const playerController = (ndPlayerCapsuleController*)*modelNotity;
	ndRender* const renderer = *scene->GetRenderer();
	renderer->SetCamera(playerController->GetCamera());

#if 1
	{
		// populate the world with props and other players
		AddSomeProps(scene);

		// load the write man model and resuse the animations
		loader.LoadEntity(*scene->GetRenderer(), ndGetWorkingFileName("whiteMan.fbx"));

		location.m_posit.m_x = 25.0f;
		location.m_posit.m_z = -5.0f;
		ndPlayerCapsuleController::CreatePlayer(scene, loader, location);

		location.m_posit.m_x = 25.0f;
		location.m_posit.m_z = 5.0f;
		ndPlayerCapsuleController::CreatePlayer(scene, loader, location);

		location.m_posit.m_x = 30.0f;
		location.m_posit.m_z = 10.0f;
		ndPlayerCapsuleController::CreatePlayer(scene, loader, location);

		location.m_posit.m_x = 30.0f;
		location.m_posit.m_z = 15.0f;
		ndPlayerCapsuleController::CreatePlayer(scene, loader, location);
	}
#endif
}
