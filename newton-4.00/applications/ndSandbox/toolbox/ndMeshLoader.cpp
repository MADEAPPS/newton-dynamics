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
{
}

ndMeshLoader::~ndMeshLoader()
{
}

ndSharedPtr<ndRenderSceneNode> ndMeshLoader::LoadEntity(ndRender* const renderer, const ndString& fbxPathMeshName)
{
	m_mesh = ndSharedPtr<ndMesh>(ndFbxMeshLoader::LoadMesh(fbxPathMeshName.GetStr(), false));

	ndSharedPtr<ndRenderSceneNode> sceneNode;
	if (*m_mesh)
	{	
		class EntityMeshPair
		{
			public:
			EntityMeshPair()
			{
			}

			EntityMeshPair(ndRenderSceneNode* const entity, ndMesh* const effectNode)
				:m_effectNode(effectNode)
				,m_entity(entity)
			{
			}

			ndMesh* m_effectNode;
			ndRenderSceneNode* m_entity;
		};

		ndFixSizeArray<EntityMeshPair, 2048> meshList;
		ndFixSizeArray<ndMesh*, 1024> effectNodeBuffer;
		ndFixSizeArray<const ndRenderSceneNode*, 1024> parentEntityBuffer;

		parentEntityBuffer.PushBack(nullptr);
		effectNodeBuffer.PushBack(*m_mesh);

		while (effectNodeBuffer.GetCount())
		{
			ndMesh* const mesh = effectNodeBuffer.Pop();
			const ndRenderSceneNode* parentNode = parentEntityBuffer.Pop();

			ndRenderSceneNode* entity = nullptr;
			if (!parentNode)
			{
				sceneNode = ndSharedPtr<ndRenderSceneNode>(new ndRenderSceneNode(mesh->m_matrix));

				entity = *sceneNode;
			}
			else
			{
				ndAssert(0);
			}

			ndSharedPtr<ndMeshEffect> meshEffect(mesh->GetMesh());
			if (*meshEffect)
			{
				meshList.PushBack(EntityMeshPair(entity, mesh));
			}

			for (ndList<ndSharedPtr<ndMesh>>::ndNode* childNode = mesh->GetChildren().GetFirst(); childNode; childNode = childNode->GetNext())
			{
				ndAssert(0);
				ndMesh* const child = *childNode->GetInfo();
				effectNodeBuffer.PushBack(child);
				parentEntityBuffer.PushBack(entity);
			}
		}

		const char* ptr = strrchr(fbxPathMeshName.GetStr(), '/');
		if (!ptr)
		{
			ptr = strrchr(fbxPathMeshName.GetStr(), '\\');
		}
		const ndString path(fbxPathMeshName.GetStr(), ndInt32 (fbxPathMeshName.Size() - strlen(ptr + 1)));

		for (ndInt32 i = 0; i < meshList.GetCount(); ++i)
		{
			ndRenderSceneNode* const entity = meshList[i].m_entity;
			ndMesh* const effectNode = meshList[i].m_effectNode;

			ndAssert(effectNode);
			ndSharedPtr<ndMeshEffect> meshEffect(effectNode->GetMesh());
			ndArray<ndMeshEffect::ndMaterial>& materials = meshEffect->GetMaterials();

			ndSharedPtr<ndRenderPrimitive> geometry(nullptr);
			if (meshEffect->GetVertexWeights().GetCount())
			{
				ndAssert(0);
				//mesh = new ndDemoSkinMesh(entity, *meshEffect, scene->GetShaderCache());
			}
			else
			{
				ndRenderPrimitiveMesh::ndDescriptor descriptor(renderer);
				descriptor.m_meshNode = *meshEffect;
				for (ndInt32 j = 0; j < materials.GetCount(); ++j)
				{
					const ndString teturePath(path + materials[i].m_textureName);
					ndRenderPrimitiveMeshMaterial& material = descriptor.AddMaterial(renderer->GetTextureCache()->GetTexture(teturePath));
					material.m_castShadows = true;
					material.m_diffuse = materials[i].m_diffuse;
					material.m_specular = materials[i].m_specular;
					material.m_reflection = materials[i].m_specular;
					material.m_specularPower = materials[i].m_shiness;
					material.m_opacity = materials[i].m_opacity;
				}
				geometry = ndSharedPtr<ndRenderPrimitive>(ndRenderPrimitiveMesh::CreateMeshPrimitive(descriptor));
			}

			entity->SetPrimitive(geometry);
			entity->SetPrimitiveMatrix(effectNode->m_meshMatrix);

			if ((effectNode->GetName().Find("hidden") >= 0) || (effectNode->GetName().Find("Hidden") >= 0))
			{
				ndAssert(0);
				//mesh->m_isVisible = false;
				//entity->m_isVisible = false;
				//entity->m_castShadow = false;
			}

		}

	}


	return sceneNode;
}