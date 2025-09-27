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

ndMeshLoader::ndMeshLoader()
	:ndFbxMeshLoader()
	,m_mesh(nullptr)
	,m_renderMesh(nullptr)
{
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

			ndRenderPrimitiveMesh::ndDescriptor descriptor(renderer);
			descriptor.m_meshNode = meshEffect;
			descriptor.m_skeleton = pair.m_entity;

			for (ndInt32 j = 0; j < materials.GetCount(); ++j)
			{
				const ndString texturePathName(path + materials[j].m_textureName);
				ndRenderPrimitiveMeshMaterial& material = descriptor.AddMaterial(renderer->GetTextureCache()->GetTexture(texturePathName));
				material.m_diffuse = materials[j].m_diffuse;
				material.m_specular = materials[j].m_specular;
				material.m_reflection = materials[j].m_specular;
				material.m_specularPower = ndReal(materials[j].m_shiness);
				material.m_opacity = ndReal(materials[j].m_opacity);
				material.m_castShadows = true;
			}
			ndSharedPtr<ndRenderPrimitive> geometry(ndRenderPrimitiveMesh::CreateMeshPrimitive(descriptor));

			//pair.m_entity->m_name = effectNode->GetName();
			pair.m_entity->SetPrimitive(geometry);
			pair.m_entity->SetPrimitiveMatrix(pair.m_mesh->m_meshMatrix);

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