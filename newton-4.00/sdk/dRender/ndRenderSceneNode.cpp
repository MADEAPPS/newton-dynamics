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

ndTransform::ndTransform(const ndQuaternion& rotation, const ndVector& position)
	:m_position(position)
	,m_rotation(rotation)
{
}

ndMatrix ndTransform::GetMatrix() const
{
	return ndCalculateMatrix(m_rotation, m_position);
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

const ndRenderSceneNodeInstance* ndRenderSceneNode::GetAsInstance() const
{
	return nullptr;
}

ndRenderSceneNode* ndRenderSceneNode::GetParent() const
{
	return m_parent;
}

const ndList<ndSharedPtr<ndRenderSceneNode>>& ndRenderSceneNode::GetChilden() const
{
	return m_children;
}

void ndRenderSceneNode::AddChild(const ndSharedPtr<ndRenderSceneNode>& child)
{
	ndAssert(child->m_parent == nullptr);
	child->m_parent = this;
	m_children.Append(child);
}

void ndRenderSceneNode::RemoveChild(const ndSharedPtr<ndRenderSceneNode> child)
{
	ndAssert(child->m_parent && (child->m_parent == this));
	for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = m_children.GetFirst(); node; node = node->GetNext())
	{
		ndRenderSceneNode* const childNode = *node->GetInfo();
		if (childNode == *child)
		{
			child->m_parent = nullptr;
			m_children.Remove(node);
			break;
		}
	}
}

void ndRenderSceneNode::SetPrimitiveMatrix(const ndMatrix& matrix)
{
	m_primitiveMatrix = matrix;
}

void ndRenderSceneNode::SetPrimitive(const ndSharedPtr<ndRenderPrimitive>& primitive)
{
	m_primitive = primitive;
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
	m_transform0 = m_transform1;
	m_transform1 = ndTransform(rotation, position);
	if (m_transform0.m_rotation.DotProduct(m_transform1.m_rotation).GetScalar() < ndFloat32(0.0f))
	{
		m_transform1.m_rotation = m_transform1.m_rotation.Scale(ndFloat32 (-1.0f));
	}
}

ndMatrix ndRenderSceneNode::CalculateGlobalMatrix(const ndRenderSceneNode* const root) const
{
	ndMatrix matrix(GetTransform().GetMatrix());
	if (this != root)
	{
		for (const ndRenderSceneNode* parent = GetParent(); parent != root; parent = parent->GetParent())
		{
			const ndMatrix parentMatrix(parent->GetTransform().GetMatrix());
			matrix = matrix * parentMatrix;
		}
	}
	return matrix;
}

ndTransform ndRenderSceneNode::GetTransform() const
{
	return m_transform1;
}

void ndRenderSceneNode::SetTransform(const ndTransform& transform)
{
	SetTransform(transform.m_rotation, transform.m_position);
}

void ndRenderSceneNode::InterpolateTransforms(ndFloat32 param)
{
	const ndVector p0(m_transform0.m_position);
	const ndVector p1(m_transform1.m_position);
	const ndQuaternion r0(m_transform0.m_rotation);
	const ndQuaternion r1(m_transform1.m_rotation);
	const ndVector posit(p0 + (p1 - p0).Scale(param));
	const ndQuaternion rotation(r0.Slerp(r1, param));

	SetMatrix(rotation, posit);
	for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = m_children.GetFirst(); node; node = node->GetNext())
	{
		node->GetInfo()->InterpolateTransforms(param);
	}
}

void ndRenderSceneNode::Render(const ndRender* const owner, ndFloat32 timestep, const ndMatrix& parentMatrix, ndRenderPassMode renderMode) const
{
	ndAssert(!m_owner || (m_owner == owner));
	const ndMatrix nodeMatrix(m_matrix * parentMatrix);
	const ndRenderPrimitive* const mesh = *m_primitive;
	if (m_isVisible && mesh)
	{
		// Render mesh if there is one 
		const ndMatrix modelMatrix(m_primitiveMatrix * nodeMatrix);
		mesh->Render(owner, modelMatrix, renderMode);
	}

	for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = m_children.GetFirst(); node; node = node->GetNext())
	{
		ndRenderSceneNode* const childNode = *node->GetInfo();
		childNode->Render(owner, timestep, nodeMatrix, renderMode);
	}
}
