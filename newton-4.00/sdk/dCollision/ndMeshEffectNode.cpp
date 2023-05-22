/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
*
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
*
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
*
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndMeshEffectNode.h"

ndMeshEffectNode::ndMeshEffectNode(ndMeshEffectNode* const parent)
	:ndNodeHierarchy<ndMeshEffectNode>()
	,m_matrix(ndGetIdentityMatrix())
	,m_meshMatrix(ndGetIdentityMatrix())
	,m_name()
	,m_mesh()
	,m_scale()
	,m_posit()
	,m_rotation()
{
	if (parent)
	{
		Attach(parent);
	}
}

ndMeshEffectNode::ndMeshEffectNode(const ndMeshEffectNode& src)
	:ndNodeHierarchy<ndMeshEffectNode>(src)
	,m_matrix(src.m_matrix)
	,m_meshMatrix(src.m_meshMatrix)
	,m_name(src.m_name)
	,m_mesh(src.m_mesh)
	,m_scale()
	,m_posit()
	,m_rotation()
{
	for (ndCurve::ndNode* node = src.m_scale.GetFirst(); node; node = node->GetNext())
	{
		m_scale.Append(node->GetInfo());
	}

	for (ndCurve::ndNode* node = src.m_posit.GetFirst(); node; node = node->GetNext())
	{
		m_posit.Append(node->GetInfo());
	}

	for (ndCurve::ndNode* node = src.m_rotation.GetFirst(); node; node = node->GetNext())
	{
		m_rotation.Append(node->GetInfo());
	}
}

ndMeshEffectNode::~ndMeshEffectNode()
{
}

const ndString& ndMeshEffectNode::GetName() const
{
	return m_name;
}

ndMeshEffectNode::ndCurve& ndMeshEffectNode::GetScaleCurve()
{
	return m_scale;
}

ndMeshEffectNode::ndCurve& ndMeshEffectNode::GetPositCurve()
{
	return m_posit;
}

ndMeshEffectNode::ndCurve& ndMeshEffectNode::GetRotationCurve()
{
	return m_rotation;
}

void ndMeshEffectNode::SetName(const ndString& name)
{
	m_name = name;
}

ndMeshEffectNode* ndMeshEffectNode::CreateClone() const
{
	return new ndMeshEffectNode(*this);
}

ndSharedPtr<ndMeshEffect> ndMeshEffectNode::GetMesh()
{
	return m_mesh;
}

void ndMeshEffectNode::SetMesh(const ndSharedPtr<ndMeshEffect>& mesh)
{
	m_mesh = mesh;
}
