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
#include "ndSkyBox.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndAnimationSequence.h"
#include "ndBasicPlayerCapsule.h"
#include "ndAnimationTwoWayBlend.h"
#include "ndAnimationSequencePlayer.h"

#define PLAYER_WALK_SPEED				8.0f
#define PLAYER_JUMP_SPEED				5.0f

//#define PLAYER_FIRST_PERSON	

#ifdef PLAYER_FIRST_PERSON	
	#define PLAYER_THIRD_PERSON_VIEW_DIST	0.0f
#else
	#define PLAYER_THIRD_PERSON_VIEW_DIST	6.0f
#endif

void ndBasicPlayerCapsule::ndFileBasicPlayerCapsule::SaveBody(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndBody* const body)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_BODY_CLASS, ndBasicPlayerCapsule::StaticClassName());
	ndFileFormatBodyKinematicPlayerCapsule::SaveBody(scene, classNode, body);
}

ndBody* ndBasicPlayerCapsule::ndFileBasicPlayerCapsule::LoadBody(const nd::TiXmlElement* const node, const ndTree<ndShape*, ndInt32>& shapeMap)
{
	ndBasicPlayerCapsule* const player = new ndBasicPlayerCapsule();
	ndFileFormatBodyKinematicPlayerCapsule::LoadBody((nd::TiXmlElement*)node->FirstChild(D_BODY_CLASS), shapeMap, player);
	return player;
}

class ndBasicPlayerCapsuleNotify : public ndDemoEntityNotify
{
	public:
	ndBasicPlayerCapsuleNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity)
		:ndDemoEntityNotify(manager, entity)
		,m_localRotation(entity->GetRenderMatrix())
		,m_meshOrigin(entity->GetRenderMatrix().m_posit)
	{
	}

	void OnTransform(ndInt32, const ndMatrix& matrix)
	{
		//ndDemoEntityNotify::OnTransform(thread, matrix);
		const ndBody* const body = GetBody();
		const ndQuaternion rot(body->GetRotation());
		m_entity->SetMatrix(m_localRotation * rot, matrix.TransformVector(m_meshOrigin));
		ndWorld* const word = m_manager->GetWorld();
		ndBasicPlayerCapsule* const player = (ndBasicPlayerCapsule*)GetBody();

		//ndAssert(0);
		//ndTrace(("Play animation here!!!!\n"));
		ndFloat32 timestep = word->GetScene()->GetTimestep();
		timestep *= 0.25f;
		//timestep = 1.0f/(30.0f * 4.0f);
		//timestep *= 0.05f;
		//player->m_animBlendTree->Evaluate(player->m_output, timestep);
		player->m_animBlendTree->Evaluate(player->m_output);
		
		for (ndInt32 i = 0; i < player->m_output.GetCount(); ++i)
		{
			const ndAnimKeyframe& keyFrame = player->m_output[i];
			ndDemoEntity* const entity = (ndDemoEntity*)keyFrame.m_userData;
			entity->SetMatrix(keyFrame.m_rotation, keyFrame.m_posit);
		}
	}

	ndQuaternion m_localRotation;
	ndQuaternion m_meshOrigin;
};

ndBasicPlayerCapsule::ndBasicPlayerCapsule()
	:ndBodyPlayerCapsule()
	,m_isPlayer(false)
{
}

ndBasicPlayerCapsule::ndBasicPlayerCapsule(
	ndDemoEntityManager* const scene, const ndDemoEntity* const modelEntity,
	const ndMatrix& localAxis, const ndMatrix& location,
	ndFloat32 mass, ndFloat32 radius, ndFloat32 height, ndFloat32 stepHeight, bool isPlayer)
	:ndBodyPlayerCapsule(localAxis, mass, radius, height, stepHeight)
	//,m_scene(scene)
	,m_isPlayer(isPlayer)
	,m_output()
	,m_animBlendTree(nullptr)
{
	static ndFileBasicPlayerCapsule loadSave;

	ndMatrix matrix(location);
	ndPhysicsWorld* const world = scene->GetWorld();
	ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y = floor.m_y;
	
	ndDemoEntity* const entity = modelEntity->CreateClone();
	//entity->ResetMatrix(entity->GetRenderMatrix() * matrix);
	//entity->ResetMatrix(matrix);

	SetMatrix(matrix);
	scene->AddEntity(entity);

	SetNotifyCallback(new ndBasicPlayerCapsuleNotify(scene, entity));

	if (isPlayer)
	{
		scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
	}

	ndTrace(("Not animation yet  !!!!\n"));

	//// create bind pose to animation sequences.
	//ndAnimationSequence* const sequence = scene->GetAnimationSequence("white_Man_idle.fbx");
	ndSharedPtr<ndAnimationSequence> sequence(scene->GetAnimationSequence("mocapWalker_walk.fbx"));

	const ndList<ndAnimationKeyFramesTrack>& tracks = sequence->GetTracks();
	for (ndList<ndAnimationKeyFramesTrack>::ndNode* node = tracks.GetFirst(); node; node = node->GetNext()) 
	{
		ndAnimationKeyFramesTrack& track = node->GetInfo();
		ndDemoEntity* const ent = entity->Find(track.GetName().GetStr());
		ndAnimKeyframe keyFrame;
		keyFrame.m_userData = ent;
		m_output.PushBack(keyFrame);
	}
	
	//// create an animation blend tree
	//ndAnimationSequence* const idleSequence = scene->GetAnimationSequence("white_Man_idle.fbx");
	//ndAnimationSequence* const walkSequence = scene->GetAnimationSequence("white_man_walk.fbx");
	//ndAnimationSequence* const runSequence = scene->GetAnimationSequence("white_man_run.fbx");

	ndSharedPtr<ndAnimationSequence> walkSequence(scene->GetAnimationSequence("mocapWalker_walk.fbx"));
	
	//ndAnimationSequencePlayer* const idle = new ndAnimationSequencePlayer(idleSequence);
	ndAnimationSequencePlayer* const walk = new ndAnimationSequencePlayer(walkSequence);
	//ndAnimationSequencePlayer* const run = new ndAnimationSequencePlayer(runSequence);
	//
	//////dFloat scale0 = walkSequence->GetPeriod() / runSequence->GetPeriod();
	////ndFloat32 scale1 = runSequence->GetPeriod() / walkSequence->GetPeriod();
	//ndAnimationTwoWayBlend* const walkRunBlend = new ndAnimationTwoWayBlend(walk, run);
	//ndAnimationTwoWayBlend* const idleMoveBlend = new ndAnimationTwoWayBlend(idle, walkRunBlend);
	//
	//walkRunBlend->SetParam(0.0f);
	////idleMoveBlend->SetParam(0.0f);
	//idleMoveBlend->SetParam(1.0f);
	////walkRunBlend->SetTimeDilation1(scale1);
	//m_animBlendTree = idleMoveBlend;
	m_animBlendTree = walk;
	 
	//// evaluate twice that interpolation is reset
	//ndAssert(0);
	////m_animBlendTree->Evaluate(m_output, ndFloat32(0.0f));
	////m_animBlendTree->Evaluate(m_output, ndFloat32(0.0f));
}

ndBasicPlayerCapsule::~ndBasicPlayerCapsule()
{
	if (m_animBlendTree)
	{
		delete m_animBlendTree;
	}
}

void ndBasicPlayerCapsule::ApplyInputs(ndFloat32 timestep)
{
	//calculate the gravity contribution to the velocity, 
	const ndVector gravity(GetNotifyCallback()->GetGravity());
	const ndVector totalImpulse(m_impulse + gravity.Scale(m_mass * timestep));
	m_impulse = totalImpulse;

	//dTrace(("  frame: %d    player camera: %f\n", m_scene->GetWorld()->GetFrameIndex(), m_playerInput.m_heading * dRadToDegree));
	if (m_playerInput.m_jump)
	{
		ndVector jumpImpule(0.0f, PLAYER_JUMP_SPEED * m_mass, 0.0f, 0.0f);
		m_impulse += jumpImpule;
		m_playerInput.m_jump = false;
	}

	SetForwardSpeed(m_playerInput.m_forwardSpeed);
	SetLateralSpeed(m_playerInput.m_strafeSpeed);
	SetHeadingAngle(m_playerInput.m_heading);
}

//ndFloat32 ndBasicPlayerCapsule::ContactFrictionCallback(const ndVector& position, const ndVector& normal, ndInt32 contactId, const ndBodyKinematic* const otherbody) const
ndFloat32 ndBasicPlayerCapsule::ContactFrictionCallback(const ndVector&, const ndVector& normal, ndInt32, const ndBodyKinematic* const) const
{
	//return ndFloat32(2.0f);
	if (ndAbs(normal.m_y) < 0.8f)
	{
		return 0.4f;
	}
	return ndFloat32(2.0f);
}

void ndBasicPlayerCapsule::SetCamera(ndDemoEntityManager* const scene)
{
	if (m_isPlayer)
	{
		ndDemoCamera* const camera = scene->GetCamera();
		ndMatrix camMatrix(camera->GetNextMatrix());

		ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)GetNotifyCallback();
		ndDemoEntity* const player = (ndDemoEntity*)notify->GetUserData();
		ndMatrix playerMatrix(player->GetNextMatrix());

		const ndFloat32 height = m_height;
		const ndVector frontDir(camMatrix[0]);
		const ndVector upDir(0.0f, 1.0f, 0.0f, 0.0f);
		ndVector camOrigin = playerMatrix.TransformVector(upDir.Scale(height));
		camOrigin -= frontDir.Scale(PLAYER_THIRD_PERSON_VIEW_DIST);

		camera->SetNextMatrix(camMatrix, camOrigin);

		ndFloat32 angle0 = camera->GetYawAngle();
		ndFloat32 angle1 = GetHeadingAngle();
		ndFloat32 error = ndAnglesAdd(angle1, -angle0);

		if ((ndAbs (error) > 1.0e-3f) ||
			scene->GetKeyState(' ') ||
			scene->GetKeyState('A') ||
			scene->GetKeyState('D') ||
			scene->GetKeyState('W') ||
			scene->GetKeyState('S'))
		{
			SetSleepState(false);
		}

		m_playerInput.m_heading = camera->GetYawAngle();
		m_playerInput.m_forwardSpeed = (ndFloat32)(ndInt32(scene->GetKeyState('W')) - ndInt32(scene->GetKeyState('S'))) * PLAYER_WALK_SPEED;
		m_playerInput.m_strafeSpeed = (ndFloat32)(ndInt32(scene->GetKeyState('D')) - ndInt32(scene->GetKeyState('A'))) * PLAYER_WALK_SPEED;
		m_playerInput.m_jump = scene->GetKeyState(' ') && IsOnFloor();

		if (m_playerInput.m_forwardSpeed && m_playerInput.m_strafeSpeed)
		{
			ndFloat32 invMag = PLAYER_WALK_SPEED / ndSqrt(m_playerInput.m_forwardSpeed * m_playerInput.m_forwardSpeed + m_playerInput.m_strafeSpeed * m_playerInput.m_strafeSpeed);
			m_playerInput.m_forwardSpeed *= invMag;
			m_playerInput.m_strafeSpeed *= invMag;
		}
	}
}

void ndBasicPlayerCapsule::UpdateCameraCallback(ndDemoEntityManager* const manager, void* const context, ndFloat32)
{
	ndBasicPlayerCapsule* const me = (ndBasicPlayerCapsule*)context;
	me->SetCamera(manager);
}
