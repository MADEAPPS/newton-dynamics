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

ndTransform::ndTransform()
	:m_position(ndVector::m_wOne)
	,m_rotation()
{
}

ndTransform::ndTransform(const ndMatrix& matrix)
	:m_position(matrix.m_posit)
	,m_rotation(matrix)
{
}

ndRenderSceneNode::ndRenderSceneNode(const ndMatrix& matrix)
	:ndContainersFreeListAlloc<ndRenderSceneNode>()
	,m_matrix(matrix)
	,m_primitiveMatrix(ndGetIdentityMatrix())
	,m_transform0(matrix)
	,m_transform1(m_transform0)
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
	// for now just set the matrix;
	m_matrix = ndCalculateMatrix(rotation, position);
}

void ndRenderSceneNode::SetTransform(const ndQuaternion& rotation, const ndVector& position)
{
	ndScopeSpinLock lock(m_lock);
	m_transform0 = m_transform1;
	m_transform1.m_position = position;
	m_transform1.m_rotation = rotation;
}

void ndRenderSceneNode::InterpolateTransforms(ndFloat32 param)
{
	ndFixSizeArray<ndRenderSceneNode*, 128> stack;
	stack.PushBack(this);
	while (stack.GetCount())
	{
		ndRenderSceneNode* const rooNode = stack.Pop();
		{
			ndScopeSpinLock lock(m_lock);
			const ndVector p0(rooNode->m_transform0.m_position);
			const ndVector p1(rooNode->m_transform1.m_position);
			const ndQuaternion r0(rooNode->m_transform0.m_rotation);
			const ndQuaternion r1(rooNode->m_transform1.m_rotation);
			const ndVector posit(p0 + (p1 - p0).Scale(param));
			const ndQuaternion rotation(r0.Slerp(r1, param));
			rooNode->SetMatrix(rotation, posit);
		}

		for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = m_children.GetFirst(); node; node = node->GetNext())
		{
			ndRenderSceneNode* const childNode = *node->GetInfo();
			stack.PushBack(childNode);
		}
	}
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