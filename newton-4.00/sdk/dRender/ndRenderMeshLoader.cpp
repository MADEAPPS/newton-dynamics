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

#include "ndRenderStdafx.h"
#include "ndRender.h"
#include "ndRenderTexture.h"
#include "ndRenderPrimitive.h"
#include "ndRenderSceneNode.h"
#include "ndRenderMeshLoader.h"
#include "ndRenderTextureCache.h"

ndRenderMeshLoader::ndRenderMeshLoader(ndRender* const renderer)
	:m_owner(renderer) 
{
}

//ndRenderMeshLoader(const ndRenderMeshLoader& src) {};
ndRenderMeshLoader::~ndRenderMeshLoader()
{
}

bool ndRenderMeshLoader::MeshToRenderSceneNode(const ndString& materialBasePath)
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

	m_renderMesh = ndSharedPtr<ndRenderSceneNode>(nullptr);
	while (effectNodeList.GetCount())
	{
		ndSharedPtr<ndMesh> mesh(effectNodeList.GetLast()->GetInfo());
		ndSharedPtr<ndRenderSceneNode> parentNode(parentEntityList.GetLast()->GetInfo());

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

	for (ndList<EntityMeshPair>::ndNode* node = meshList.GetFirst(); node; node = node->GetNext())
	{
		EntityMeshPair& pair = node->GetInfo();
	
		ndAssert(*pair.m_mesh);
		ndAssert(*pair.m_entity);
	
		ndSharedPtr<ndMeshEffect> meshEffect(pair.m_mesh->GetMesh());
		ndArray<ndMeshEffect::ndMaterial>& materials = meshEffect->GetMaterials();
	
		ndRenderPrimitive::ndDescriptor descriptor(m_owner);
		descriptor.m_meshNode = meshEffect;
		descriptor.m_skeleton = pair.m_entity;
	
		for (ndInt32 j = 0; j < materials.GetCount(); ++j)
		{
			const ndString texturePathName(materialBasePath + materials[j].m_textureName);
			ndRenderPrimitiveMaterial& material = descriptor.AddMaterial(m_owner->GetTextureCache()->GetTexture(texturePathName));
			material.m_diffuse = materials[j].m_diffuse;
			material.m_specular = materials[j].m_specular;
			material.m_reflection = materials[j].m_reflection;
			material.m_specularPower = ndReal(materials[j].m_shiness);
			material.m_opacity = ndReal(materials[j].m_opacity);
			material.m_castShadows = true;
		}
	
		//pair.m_entity->m_name = effectNode->GetName();
		//ndSharedPtr<ndRenderPrimitive> geometry(ndRenderPrimitive::CreateMeshPrimitive(descriptor));
		ndSharedPtr<ndRenderPrimitive> geometry(new ndRenderPrimitive(descriptor));
		pair.m_entity->SetPrimitive(geometry);
	
		//if ((effectNode->GetName().Find("hidden") >= 0) || (effectNode->GetName().Find("Hidden") >= 0))
		//{
		//	ndAssert(0);
		//	//mesh->m_isVisible = false;
		//	//entity->m_isVisible = false;
		//	//entity->m_castShadow = false;
		//}
	}
	return m_renderMesh;
}

bool ndRenderMeshLoader::LoadMesh(const ndString& fullPathMeshName)
{
	if (ndMeshLoader::LoadMesh(fullPathMeshName))
	{
		return MeshToRenderSceneNode(GetPath(fullPathMeshName));
	}
	return false;
}

bool ndRenderMeshLoader::ImportFbx(const ndString& fbxPathMeshName)
{
	if (ndMeshLoader::ImportFbx(fbxPathMeshName))
	{
		return MeshToRenderSceneNode(GetPath(fbxPathMeshName));
	}
	return false;
}

