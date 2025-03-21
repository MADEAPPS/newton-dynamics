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
#include "ndMeshLoader.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndCompoundScene.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"
#include "ndBasicPlayerCapsule.h"

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

	ndMesh* LoadMesh(const char* const fbxMeshName, bool loadAnimation)
	{
		ndMesh* mesh = ndMeshLoader::LoadMesh(fbxMeshName, loadAnimation);

		// re target animation names
		ndList<ndSharedPtr<ndMeshEffect>> meshes;
		for (ndMesh* node = mesh->GetFirstIterator(); node; node = node->GetNextIterator())
		{
			ndSharedPtr<ndMeshEffect> meshEffect(node->GetMesh());
			if (*meshEffect && meshEffect->GetVertexWeights().GetCount())
			{
				meshes.Append(meshEffect);
			}
		}
		if (meshes.GetCount())
		{
			for (ndMesh* node = mesh->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				const char* const ptr = strchr(node->GetName().GetStr(), ':');
				if (ptr)
				{
					ndInt32 newHashId = ndInt32(ndCRC64(ptr + 1) & 0xffffffff);
					ndInt32 oldHashId = ndInt32(ndCRC64(node->GetName().GetStr()) & 0xffffffff);
					for (ndList<ndSharedPtr<ndMeshEffect>>::ndNode* meshNode = meshes.GetFirst(); meshNode; meshNode = meshNode->GetNext())
					{
						ndArray<ndMeshEffect::ndVertexWeight>& weights = meshNode->GetInfo()->GetVertexWeights();
						for (ndInt32 i = 0; i < weights.GetCount(); ++i)
						{
							ndMeshEffect::ndVertexWeight& w = weights[i];
							for (ndInt32 j = ND_VERTEX_WEIGHT_SIZE - 1; j >= 0; --j)
							{
								if (w.m_boneId[j] == oldHashId)
								{
									w.m_boneId[j] = newHashId;
								}
							}
						}
					}
				}
			}
		}

		// re target bone names
		for (ndMesh* node = mesh->GetFirstIterator(); node; node = node->GetNextIterator())
		{
			const char* const ptr = strchr(node->GetName().GetStr(), ':');
			if (ptr)
			{
				node->SetName(ptr + 1);
			}
		}

		if (m_scale != ndFloat32(1.0f))
		{
			ndMatrix scaleMatrix(ndGetIdentityMatrix());
			scaleMatrix[0][0] = m_scale;
			scaleMatrix[1][1] = m_scale;
			scaleMatrix[2][2] = m_scale;
			mesh->ApplyTransform(scaleMatrix);
		}

		static int xxxxx;
		if (loadAnimation)
		{
			if (xxxxx==0)
			{
				//ndMesh::Save(mesh, "xxx.ndm");
				//delete mesh;
				//mesh = ndMesh::Load("xxx.ndm");
			}
			xxxxx++;
		}

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

void ndPlayerCapsuleDemo (ndDemoEntityManager* const scene)
{
	// build a floor
	//BuildPlayArena(scene);
	BuildFloorBox(scene, ndGetIdentityMatrix());
	//BuildCompoundScene(scene, ndGetIdentityMatrix());

	ndMatrix location(ndGetIdentityMatrix());
	location.m_posit.m_y += 2.0f;

	ndMatrix localAxis(ndGetIdentityMatrix());
	localAxis[0] = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
	localAxis[1] = ndVector(1.0f, 0.0f, 0.0f, 0.0f);
	localAxis[2] = localAxis[0].CrossProduct(localAxis[1]);

	ndFloat32 height = 1.9f;
	ndFloat32 radio = 0.5f;
	ndFloat32 mass = 100.0f;

	ndMopcapRetargetMeshLoader loader(1.0f);
	ndPhysicsWorld* const world = scene->GetWorld();
	//ndSharedPtr<ndDemoEntity> entity(loader.LoadEntity("box.fbx", scene));
	//ndSharedPtr<ndDemoEntity> entity(loader.LoadEntity("skinTest.fbx", scene));
	//ndSharedPtr<ndDemoEntity> entity(loader.LoadEntity("dummy.fbx", scene));
	//ndSharedPtr<ndDemoEntity> entity(loader.LoadEntity("robotsuit.fbx", scene));
	//ndSharedPtr<ndDemoEntity> entity(loader.LoadEntity("YBot.fbx", scene));
	//ndSharedPtr<ndDemoEntity> entity(loader.LoadEntity("YBot_mixamo.fbx", scene));
	ndSharedPtr<ndDemoEntity> entity(loader.LoadEntity("YBot_blender.fbx", scene));
	
	ndSharedPtr<ndBody> player0(new ndBasicPlayerCapsule(scene, loader, *entity, localAxis, location, mass, radio, height, height / 4.0f, true));
	world->AddBody(player0);

#if 0	
	ndSharedPtr<ndBody> player1(new ndBasicPlayerCapsule(scene, loader, *entity, localAxis, location, mass, radio, height, height/4.0f));
	//world->AddBody(player1);
	
	location.m_posit.m_z += 2.0f;
	ndSharedPtr<ndBody> player2(new ndBasicPlayerCapsule(scene, loader, *entity, localAxis, location, mass, radio, height, height / 4.0f));
	//world->AddBody(player2);
	
	location.m_posit.m_z += 2.0f;
	ndSharedPtr<ndBody> player3(new ndBasicPlayerCapsule(scene, loader, *entity, localAxis, location, mass, radio, height, height / 4.0f));
	//world->AddBody(player3);
#endif

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

	//AddCapsulesStacks___(scene, PlaceMatrix(32.0f, 0.0f, 0.0f), 10.0f, 0.5f, 0.5f, 1.0f, 10, 10, 7);
	//AddBox(scene, PlaceMatrix(10.0f, 0.0f, 0.0f), 30.0f, 2.0f, 0.25f, 2.5f);
	//AddBox(scene, PlaceMatrix(10.0f, 0.5f, 1.125f), 30.0f, 2.0f, 0.25f, 2.5f);
	//AddBox(scene, PlaceMatrix(10.0f, 1.0f, 1.250f), 30.0f, 2.0f, 0.25f, 2.5f);

	ndQuaternion rot;
	ndVector origin(-10.0f, 5.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}
