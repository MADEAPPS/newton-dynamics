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
#include "ndRenderSceneNodeInstanceImplement.h"

ndRenderSceneNodeInstanceImplement::ndRenderSceneNodeInstanceImplement(ndRenderSceneNodeInstance* const owner)
	:ndContainersFreeListAlloc<ndRenderSceneNodeInstanceImplement>()
	,m_owner(owner)
{
}

void ndRenderSceneNodeInstanceImplement::Finalize()
{
	//const ndList<ndSharedPtr<ndRenderSceneNode>>& children = m_owner->GetChilden();
	
	ndRenderSceneNodeInstance* const owner = (ndRenderSceneNodeInstance*)m_owner;

	const ndRenderPrimitiveMesh::ndDescriptor& descriptor = owner->m_descriptor;
	ndAssert(descriptor.m_numberOfInstances > 0);
	ndAssert(descriptor.m_meshBuildMode == ndRenderPrimitiveMesh::m_instancePrimitve);

	m_matrixPallete.SetCount(descriptor.m_numberOfInstances);
	ndSharedPtr<ndRenderPrimitive> mesh(ndRenderPrimitiveMesh::CreateMeshPrimitive(descriptor));
	owner->SetPrimitive(mesh);
	
	//m_renderShader.GetShaderParameters()
	//glGenBuffers(1, &m_matrixOffsetBuffer);
	//glBindBuffer(GL_ARRAY_BUFFER, m_matrixOffsetBuffer);
	//glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(instanceCount * sizeof(glMatrix)), &m_matrixPallete[0][0][0], GL_STATIC_DRAW);
}

ndRenderSceneNodeInstanceImplement::~ndRenderSceneNodeInstanceImplement()
{
	//glDeleteBuffers(1, &m_matrixOffsetBuffer);
}

void ndRenderSceneNodeInstanceImplement::Render(const ndRender* const owner, ndFloat32, const ndMatrix& parentMatrix, ndRenderPassMode renderMode) const
{
	if (renderMode == m_directionalDiffusseShadow)
	{
		const ndList<ndSharedPtr<ndRenderSceneNode>>& children = m_owner->GetChilden();

		const ndRenderPrimitive* const mesh = *m_owner->m_primitive;
		m_matrixPallete.SetCount(0);
		for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = children.GetFirst(); node; node = node->GetNext())
		{
			ndRenderSceneNode* const child = *node->GetInfo();
			//const ndMatrix matrix(child->GetMatrix() * parentMatrix);
			m_matrixPallete.PushBack(glMatrix(child->GetMatrix()));

			//	// for now just call render
			//	//child->Render(owner, timeStep, parentMatrix, renderMode);
		}
		mesh->Render(owner, parentMatrix, m_directionalDiffusseInstanceShadow);
	}
}
