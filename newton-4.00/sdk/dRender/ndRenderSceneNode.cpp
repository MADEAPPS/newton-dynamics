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
#include "ndRenderSceneNode.h"
#include "ndRenderPrimitive.h"

ndRenderSceneNode::ndRenderSceneNode(const ndMatrix& matrix)
	:ndContainersFreeListAlloc<ndRenderSceneNode>()
	,m_matrix(matrix)
	,m_primitiveMatrix(ndGetIdentityMatrix())
	,m_owner(nullptr)
	,m_parent(nullptr)
	,m_children()
	,m_sceneHandle(nullptr)
	,m_isVisible(true)
{
}

ndRenderSceneNode::~ndRenderSceneNode()
{
}

void ndRenderSceneNode::SetPrimitiveMatrix(const ndMatrix& matrix)
{
	m_primitiveMatrix = matrix;
}

void ndRenderSceneNode::SetPrimitive(const ndSharedPtr<ndRenderPrimitive>& primitive)
{
	m_primitve = primitive;
}

ndRender* ndRenderSceneNode::GetOwner() const
{
	return m_owner;
}

ndMatrix ndRenderSceneNode::GetMatrix() const
{
	return m_matrix;
}

void ndRenderSceneNode::SetMatrix(const ndQuaternion& rotation, const ndVector& position)
{
	// for now jut set the matrix;
	m_matrix = ndCalculateMatrix(rotation, position);
}

void ndRenderSceneNode::Render(const ndRender* const owner, ndFloat32 timestep, const ndMatrix& parentMatrix) const
{
	ndAssert(!m_owner || (m_owner == owner));
	const ndMatrix nodeMatrix(m_matrix * parentMatrix);
	const ndRenderPrimitive* const mesh = *m_primitve;
	if (m_isVisible && mesh)
	{
		// Render mesh if there is one 
		const ndMatrix modelMatrix(m_primitiveMatrix * nodeMatrix);
		mesh->Render(owner, modelMatrix);
	}

	//RenderBone(scene, nodeMatrix);
	for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = m_children.GetFirst(); node; node = node->GetNext())
	{
		ndAssert(0);
		ndRenderSceneNode* const childNode = *node->GetInfo();
		childNode->Render(owner, timestep, nodeMatrix);
	}
}