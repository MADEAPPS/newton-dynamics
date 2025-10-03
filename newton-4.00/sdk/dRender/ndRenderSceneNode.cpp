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
#include "ndRenderPrimitiveImplement.h"

#define ND_STACK_DEPTH 512

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
	,m_globalMatrix(matrix)
	,m_primitiveMatrix(ndGetIdentityMatrix())
	,m_transform0(matrix)
	,m_transform1(m_transform0)
	,m_name()
	,m_owner(nullptr)
	,m_parent(nullptr)
	,m_primitive(nullptr)
	,m_children()
	,m_childNode(nullptr)
	,m_sceneHandle(nullptr)
	,m_isVisible(true)
{
}

ndRenderSceneNode::ndRenderSceneNode(const ndRenderSceneNode& src)
	:ndContainersFreeListAlloc<ndRenderSceneNode>()
	,m_matrix(src.m_matrix)
	,m_globalMatrix(src.m_globalMatrix)
	,m_primitiveMatrix(src.m_primitiveMatrix)
	,m_transform0(src.m_transform0)
	,m_transform1(src.m_transform1)
	,m_name(src.m_name)
	,m_owner(nullptr)
	,m_parent(nullptr)
	,m_primitive(nullptr)
	,m_children()
	,m_childNode(nullptr)
	,m_sceneHandle(nullptr)
	,m_isVisible(true)
{
	for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = src.m_children.GetFirst(); node; node = node->GetNext())
	{
		ndRenderSceneNode* const childNode = *node->GetInfo();
		AddChild(childNode->CloneSkeleton());
	}
}

ndRenderSceneNode::~ndRenderSceneNode()
{
}

void ndRenderSceneNode::ClonePrimitives(const ndRenderSceneNode& src)
{
	const ndRenderSceneNode* srcEntity = src.IteratorFirst();
	for (ndRenderSceneNode* entity = IteratorFirst(); entity; entity = entity->IteratorNext())
	{
		ndAssert(entity->m_name == srcEntity->m_name);
		if (*srcEntity->m_primitive)
		{
			entity->m_primitive = ndSharedPtr<ndRenderPrimitive>(new ndRenderPrimitive(**srcEntity->m_primitive, entity));
		}
		srcEntity = srcEntity->IteratorNext();
	}
}

void ndRenderSceneNode::ApplyPrimitiveTransforms()
{
	ndAssert(*m_primitive);
	if (m_primitive->IsSKinnedMesh())
	{
		m_primitive->UpdateSkinPaletteMatrix();
	}
}

ndRenderSceneNode* ndRenderSceneNode::Clone() const
{
	ndRenderSceneNode* const rootNode = new ndRenderSceneNode(*this);
	rootNode->ClonePrimitives(*this);
	return rootNode;
}

ndRenderSceneNode* ndRenderSceneNode::CloneSkeleton() const
{
	return new ndRenderSceneNode(*this);
}

ndRenderSceneCamera* ndRenderSceneNode::GetAsCamera()
{
	return nullptr;
}

const ndRenderSceneCamera* ndRenderSceneNode::GetAsCamera() const
{
	return nullptr;
}

ndRenderSceneNodeInstance* ndRenderSceneNode::GetAsInstance()
{
	return nullptr;
}

const ndRenderSceneNodeInstance* ndRenderSceneNode::GetAsInstance() const
{
	return nullptr;
}

ndRenderSceneNode* ndRenderSceneNode::GetParent() const
{
	return m_parent;
}

ndRenderSceneNode* ndRenderSceneNode::GetRoot() const
{
	const ndRenderSceneNode* self = this;
	while (self->m_parent)
	{
		self = self->m_parent;
	}
	return (ndRenderSceneNode*)self;
}

const ndList<ndSharedPtr<ndRenderSceneNode>>& ndRenderSceneNode::GetChildren() const
{
	return m_children;
}

ndRenderSceneNode* ndRenderSceneNode::IteratorFirst()
{
	ndRenderSceneNode* ptr = this;
	while (ptr->m_children.GetCount())
	{
		ptr = *ptr->m_children.GetFirst()->GetInfo();
	}
	return ptr;
}

const ndRenderSceneNode* ndRenderSceneNode::IteratorFirst() const
{
	const ndRenderSceneNode* ptr = this;
	while (ptr->m_children.GetCount())
	{
		ptr = *ptr->m_children.GetFirst()->GetInfo();
	}
	return ptr;
}

ndRenderSceneNode* ndRenderSceneNode::IteratorNext()
{
	if (m_childNode)
	{
		ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* next = m_childNode->GetNext();
		if (next)
		{
			if (next->GetInfo()->m_children.GetCount())
			{
				return next->GetInfo()->IteratorFirst();
			}
			return *next->GetInfo();
		}
		return m_parent;
	}
	return nullptr;
}

const ndRenderSceneNode* ndRenderSceneNode::IteratorNext() const
{
	if (m_childNode)
	{
		ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* next = m_childNode->GetNext();
		if (next)
		{
			if (next->GetInfo()->m_children.GetCount())
			{
				return next->GetInfo()->IteratorFirst();
			}
			return *next->GetInfo();
		}
		return m_parent;
	}
	return nullptr;
}

ndRenderSceneCamera* ndRenderSceneNode::FindCameraNode()
{
	for (ndRenderSceneNode* node = IteratorFirst(); node; node = node->IteratorNext())
	{
		ndRenderSceneCamera* const camera = node->GetAsCamera();
		if (camera)
		{
			return camera;
		}
	}
	return nullptr;
}

const ndRenderSceneCamera* ndRenderSceneNode::FindCameraNode() const
{
	ndRenderSceneNode* const self = (ndRenderSceneNode*)this;
	for (ndRenderSceneNode* node = self->IteratorFirst(); node; node = node->IteratorNext())
	{
		ndRenderSceneCamera* const camera = node->GetAsCamera();
		if (camera)
		{
			return camera;
		}
	}
	ndAssert(0);
	return nullptr;
}

void ndRenderSceneNode::AddChild(const ndSharedPtr<ndRenderSceneNode>& child)
{
	ndAssert(child->m_parent == nullptr);

	ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* const childNode = m_children.Append(child);
	child->m_parent = this;
	child->m_childNode = childNode;
}

void ndRenderSceneNode::RemoveChild(const ndSharedPtr<ndRenderSceneNode> child)
{
	//ndAssert(child->m_parent && (child->m_parent == this));
	//for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = m_children.GetFirst(); node; node = node->GetNext())
	//{
	//	ndRenderSceneNode* const childNode = *node->GetInfo();
	//	if (childNode == *child)
	//	{
	//		child->m_parent = nullptr;
	//		m_children.Remove(node);
	//		break;
	//	}
	//}
	ndAssert(child->m_childNode);
	ndAssert(child->m_parent && (child->m_parent == this));

	ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* const node = child->m_childNode;
	child->m_parent = nullptr;
	child->m_childNode = nullptr;

	m_children.Remove(node);
}

void ndRenderSceneNode::SetPrimitiveMatrix(const ndMatrix& matrix)
{
	m_primitiveMatrix = matrix;
}

void ndRenderSceneNode::SetPrimitive(const ndSharedPtr<ndRenderPrimitive>& primitive)
{
	m_primitive = primitive;
}

ndSharedPtr<ndRenderPrimitive> ndRenderSceneNode::GetPrimitive() const
{
	return m_primitive;
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
	ndMatrix matrix(m_matrix);
	for (const ndRenderSceneNode* parent = GetParent(); parent != root; parent = parent->GetParent())
	{
		const ndMatrix& parentMatrix = parent->m_matrix;
		matrix = matrix * parentMatrix;
	}
	return matrix;
}

ndMatrix ndRenderSceneNode::CalculateGlobalTransform(const ndRenderSceneNode* const root) const
{
	ndMatrix matrix(GetTransform().GetMatrix());
	for (const ndRenderSceneNode* parent = GetParent(); parent != root; parent = parent->GetParent())
	{
		const ndMatrix parentMatrix(parent->GetTransform().GetMatrix());
		matrix = matrix * parentMatrix;
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

	const ndQuaternion rotation(r0.Slerp(r1, param));
	const ndVector posit(p0 + (p1 - p0).Scale(param));

	SetMatrix(rotation, posit);

	for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = m_children.GetFirst(); node; node = node->GetNext())
	{
		node->GetInfo()->InterpolateTransforms(param);
	}
}

void ndRenderSceneNode::Render(const ndRender* const owner, const ndMatrix& modelViewMatrix, ndRenderPassMode renderMode) const
{
	ndAssert(!m_owner || (m_owner == owner));
	if (m_isVisible && *m_primitive)
	{
		const ndRenderPrimitive* const mesh = *m_primitive;
		const ndMatrix modelMatrix(m_primitiveMatrix * m_globalMatrix * modelViewMatrix);
		mesh->Render(owner, modelMatrix, renderMode);
	}

	for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = m_children.GetFirst(); node; node = node->GetNext())
	{
		ndRenderSceneNode* const childNode = *node->GetInfo();
		childNode->Render(owner, modelViewMatrix, renderMode);
	}
}

ndSharedPtr<ndRenderSceneNode> ndRenderSceneNode::GetSharedPtr() const
{
	if (m_childNode)
	{
		return m_childNode->GetInfo();
	}
	return ndSharedPtr<ndRenderSceneNode>(nullptr);
}

ndRenderSceneNode* ndRenderSceneNode::FindByName(const ndString& name) const
{
	ndRenderSceneNode* const self = (ndRenderSceneNode*)this;
	for (ndRenderSceneNode* node = self->IteratorFirst(); node; node = node->IteratorNext())
	{
		if (name == node->m_name)
		{
			return node;
		}
	}
	return nullptr;
}

ndRenderSceneNode* ndRenderSceneNode::FindByClosestMatch(const ndString& name) const
{
	ndRenderSceneNode* closestMatch = FindByName(name);
	if (!closestMatch)
	{
		ndInt32 bestScore = 10000;
		ndRenderSceneNode* const self = (ndRenderSceneNode*)this;
		for (ndRenderSceneNode* node = self->IteratorFirst(); node && bestScore; node = node->IteratorNext())
		{
			ndInt32 distance = node->m_name.Distance(name);
			ndAssert(distance >= 0);
			if (distance < bestScore)
			{
				bestScore = distance;
				closestMatch = node;
			}
		}
	}
	return closestMatch;
}
