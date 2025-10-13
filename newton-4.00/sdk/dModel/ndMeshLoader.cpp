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

#include "ndModelStdafx.h"
#include "ndMeshLoader.h"
#include "ndAnimationSequence.h"
#include "ndAnimationKeyframesTrack.h"

ndMeshLoader::ndMeshLoader()
	:ndFbxMeshLoader()
	,m_mesh(nullptr)
	,m_renderMesh(nullptr)
{
}

ndMeshLoader::ndMeshLoader(const ndMeshLoader& src)
	:ndFbxMeshLoader(src)
	,m_mesh(nullptr)
	,m_renderMesh(nullptr)
{
	ndAssert(0);
}

ndMeshLoader::~ndMeshLoader()
{
}

bool ndMeshLoader::LoadEntity(ndRender* const renderer, const ndString& fbxPathMeshName)
{
	m_renderMesh = ndSharedPtr<ndRenderSceneNode>(nullptr);
	m_mesh = ndSharedPtr<ndMesh>(ndFbxMeshLoader::LoadMesh(fbxPathMeshName.GetStr(), false));

	if (*m_mesh)
	{	
		class EntityMeshPair
		{
			public:
			EntityMeshPair()
				:m_mesh(nullptr)
				,m_entity(nullptr)
			{
			}

			EntityMeshPair(const EntityMeshPair& src)
				:m_mesh(src.m_mesh)
				,m_entity(src.m_entity)
			{
			}

			EntityMeshPair(const ndSharedPtr<ndRenderSceneNode>& entity, const ndSharedPtr<ndMesh>& mesh)
				:m_mesh(mesh)
				,m_entity(entity)
			{
			}

			ndSharedPtr<ndMesh> m_mesh;
			ndSharedPtr<ndRenderSceneNode> m_entity;
		};

		ndList<EntityMeshPair> meshList;
		ndList<ndSharedPtr<ndMesh>> effectNodeList;
		ndList<ndSharedPtr<ndRenderSceneNode>> parentEntityList;

		effectNodeList.Append(m_mesh);
		parentEntityList.Append(ndSharedPtr<ndRenderSceneNode>(nullptr));

		while (effectNodeList.GetCount())
		{
			ndSharedPtr<ndMesh> mesh (effectNodeList.GetLast()->GetInfo());
			ndSharedPtr<ndRenderSceneNode> parentNode (parentEntityList.GetLast()->GetInfo());

			effectNodeList.Remove(effectNodeList.GetLast());
			parentEntityList.Remove(parentEntityList.GetLast());

			ndSharedPtr<ndRenderSceneNode> entity(nullptr);
			if (!(*parentNode))
			{
				m_renderMesh = ndSharedPtr<ndRenderSceneNode>(new ndRenderSceneNode(mesh->m_matrix));
				entity = m_renderMesh;
			}
			else
			{
				ndSharedPtr<ndRenderSceneNode> childNode(new ndRenderSceneNode(mesh->m_matrix));
				parentNode->AddChild(childNode);
				entity = childNode;
			}
			entity->m_name = mesh->GetName();

			ndSharedPtr<ndMeshEffect> meshEffect(mesh->GetMesh());
			if (*meshEffect)
			{
				meshList.Append(EntityMeshPair(entity, mesh));
			}

			for (ndList<ndSharedPtr<ndMesh>>::ndNode* childNode = mesh->GetChildren().GetFirst(); childNode; childNode = childNode->GetNext())
			{
				parentEntityList.Append(entity);
				effectNodeList.Append(childNode->GetInfo());
			}
		}

		const char* ptr = strrchr(fbxPathMeshName.GetStr(), '/');
		if (!ptr)
		{
			ptr = strrchr(fbxPathMeshName.GetStr(), '\\');
		}
		const ndString path(fbxPathMeshName.GetStr(), ndInt32 (fbxPathMeshName.Size() - strlen(ptr + 1)));

		for (ndList<EntityMeshPair>::ndNode* node = meshList.GetFirst(); node; node = node->GetNext())
		{
			EntityMeshPair& pair = node->GetInfo();

			ndAssert(*pair.m_mesh);
			ndAssert(*pair.m_entity);
			
			ndSharedPtr<ndMeshEffect> meshEffect(pair.m_mesh->GetMesh());
			ndArray<ndMeshEffect::ndMaterial>& materials = meshEffect->GetMaterials();

			ndRenderPrimitive::ndDescriptor descriptor(renderer);
			descriptor.m_meshNode = meshEffect;
			descriptor.m_skeleton = pair.m_entity;

			for (ndInt32 j = 0; j < materials.GetCount(); ++j)
			{
				const ndString texturePathName(path + materials[j].m_textureName);
				ndRenderPrimitiveMaterial& material = descriptor.AddMaterial(renderer->GetTextureCache()->GetTexture(texturePathName));
				material.m_diffuse = materials[j].m_diffuse;
				material.m_specular = materials[j].m_specular;
				material.m_reflection = materials[j].m_reflection;
				material.m_specularPower = ndReal(materials[j].m_shiness);
				material.m_opacity = ndReal(materials[j].m_opacity);
				material.m_castShadows = true;
			}
			//ndSharedPtr<ndRenderPrimitive> geometry(ndRenderPrimitive::CreateMeshPrimitive(descriptor));
			ndSharedPtr<ndRenderPrimitive> geometry(new ndRenderPrimitive(descriptor));

			//pair.m_entity->m_name = effectNode->GetName();
			pair.m_entity->SetPrimitive(geometry);
			//pair.m_entity->SetPrimitiveMatrix(pair.m_mesh->m_meshMatrix);
			pair.m_entity->SetPrimitiveMatrix(ndGetIdentityMatrix());

			//if ((effectNode->GetName().Find("hidden") >= 0) || (effectNode->GetName().Find("Hidden") >= 0))
			//{
			//	ndAssert(0);
			//	//mesh->m_isVisible = false;
			//	//entity->m_isVisible = false;
			//	//entity->m_castShadow = false;
			//}
		}
	}

	return *m_mesh && *m_renderMesh;
}

const ndSharedPtr<ndAnimationSequence> ndMeshLoader::FindSequence(const ndString& fbxPathAnimName) const
{
	ndTree<ndSharedPtr<ndAnimationSequence>, ndString>::ndNode* const node = m_animationCache.Find(fbxPathAnimName);
	if (node)
	{
		return node->GetInfo();
	}
	return ndSharedPtr<ndAnimationSequence>(nullptr);
}

ndSharedPtr<ndAnimationSequence> ndMeshLoader::GetAnimationSequence(const ndString& fbxPathAnimName)
{
	ndTree<ndSharedPtr<ndAnimationSequence>, ndString>::ndNode* node = m_animationCache.Find(fbxPathAnimName);
	if (!node)
	{
		ndSharedPtr<ndAnimationSequence> sequence (LoadAnimation(fbxPathAnimName.GetStr()));
		if (sequence)
		{
			node = m_animationCache.Insert(sequence, fbxPathAnimName);
		}
	}
	return node ? node->GetInfo() : ndSharedPtr<ndAnimationSequence>(nullptr);
}

void ndMeshLoader::SetTranslationTracks(const ndString& boneName)
{
	ndTree<ndSharedPtr<ndAnimationSequence>, ndString>::Iterator it(m_animationCache);
	for (it.Begin(); it; it++)
	{
		const ndSharedPtr<ndAnimationSequence>& cycle = it.GetNode()->GetInfo();
		for (ndList<ndAnimationKeyFramesTrack>::ndNode* node = cycle->GetTracks().GetFirst(); node; node = node->GetNext())
		{
			ndAnimationKeyFramesTrack& track = node->GetInfo();
			ndString name(track.GetName());
			name.ToLower();
			if (name.Find(boneName) != -1)
			{
				ndAnimationKeyFramesTrack& translationTrack = cycle->GetTranslationTrack();
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
}