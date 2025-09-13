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
#include "ndRenderContext.h"
#include "ndRenderTexture.h"
#include "ndRenderPrimitiveMesh.h"
#include "ndRenderSceneNodeInstance.h"
#include "ndRenderPrimitiveMeshImplement.h"
#include "ndRenderSceneNodeInstanceImplement.h"

ndRenderSceneNodeInstanceImplement::ndRenderSceneNodeInstanceImplement(ndRenderSceneNodeInstance* const owner)
	:ndContainersFreeListAlloc<ndRenderSceneNodeInstanceImplement>()
	,m_owner(owner)
{
}

void ndRenderSceneNodeInstanceImplement::Finalize()
{
	ndRenderSceneNodeInstance* const owner = (ndRenderSceneNodeInstance*)m_owner;

	const ndRenderPrimitiveMesh::ndDescriptor& descriptor = owner->m_descriptor;
	ndAssert(descriptor.m_numberOfInstances > 0);
	ndAssert(descriptor.m_meshBuildMode == ndRenderPrimitiveMesh::m_instancePrimitve);
	
	ndSharedPtr<ndRenderPrimitive> mesh(ndRenderPrimitiveMesh::CreateMeshPrimitive(descriptor));
	owner->SetPrimitive(mesh);
}

ndRenderSceneNodeInstanceImplement::~ndRenderSceneNodeInstanceImplement()
{
	//glDeleteBuffers(1, &m_instanceRenderMatrixPalleteBuffer);
}

void ndRenderSceneNodeInstanceImplement::Render(const ndRender* const owner, ndFloat32, const ndMatrix& parentMatrix, ndRenderPassMode renderMode) const
{
	if (renderMode == m_directionalDiffusseShadow)
	{
		const ndMatrix primitiveMatrix(m_owner->m_primitiveMatrix);

		ndRenderPrimitiveMesh* const mesh = (ndRenderPrimitiveMesh*)*m_owner->m_primitive;
		ndRenderPrimitiveMeshImplement* const meshImplement = *mesh->m_implement;
		
		ndArray<glMatrix>& matrixPallete = meshImplement->m_instanceRenderMatrixPallete;

		matrixPallete.SetCount(0);
		const ndList<ndSharedPtr<ndRenderSceneNode>>& children = m_owner->GetChilden();
		for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = children.GetFirst(); node; node = node->GetNext())
		{
			ndRenderSceneNode* const child = *node->GetInfo();
			matrixPallete.PushBack(glMatrix(primitiveMatrix * child->GetMatrix()));
		}
		mesh->Render(owner, parentMatrix, m_directionalDiffusseInstanceShadow);
	}
}
