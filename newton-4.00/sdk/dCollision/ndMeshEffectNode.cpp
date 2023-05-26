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

void ndMeshEffectNode::ApplyTransform(const ndMatrix& transform)
{
	ndInt32 stack = 1;
	ndMeshEffectNode* entBuffer[1024];

	auto GetKeyframe = [](const ndCurveValue& scale, const ndCurveValue& position, const ndCurveValue& rotation)
	{
		ndMatrix scaleMatrix(ndGetIdentityMatrix());
		scaleMatrix[0][0] = scale.m_x;
		scaleMatrix[1][1] = scale.m_y;
		scaleMatrix[2][2] = scale.m_z;
		ndMatrix matrix(scaleMatrix * ndPitchMatrix(rotation.m_x) * ndYawMatrix(rotation.m_y) * ndRollMatrix(rotation.m_z));
		matrix.m_posit = ndVector(position.m_x, position.m_y, position.m_z, 1.0f);
		return matrix;
	};

	entBuffer[0] = this;
	ndMatrix invTransform(transform.Inverse4x4());
	while (stack)
	{
		stack--;
		ndMeshEffectNode* const ent = entBuffer[stack];

		ndMatrix entMatrix(invTransform * ent->m_matrix * transform);
		ent->m_matrix = entMatrix;

		ndSharedPtr<ndMeshEffect> mesh = ent->GetMesh();
		if (*mesh)
		{
			ndMatrix meshMatrix(invTransform * ent->m_meshMatrix * transform);
			ent->m_meshMatrix = meshMatrix;
			mesh->ApplyTransform(transform);
		}

		if (ent->GetScaleCurve().GetCount())
		{
			ndMeshEffectNode::ndCurve::ndNode* positNode = ent->GetPositCurve().GetFirst();
			ndMeshEffectNode::ndCurve::ndNode* rotationNode = ent->GetRotationCurve().GetFirst();

			ndMeshEffectNode::ndCurveValue scaleValue;
			scaleValue.m_x = 1.0f;
			scaleValue.m_y = 1.0f;
			scaleValue.m_z = 1.0f;
			for (ndInt32 i = 0; i < ent->GetScaleCurve().GetCount(); ++i)
			{
				ndMeshEffectNode::ndCurveValue& positValue = positNode->GetInfo();
				ndMeshEffectNode::ndCurveValue& rotationValue = rotationNode->GetInfo();

				ndVector animScale;
				ndMatrix stretchAxis;
				ndMatrix animTransformMatrix;
				ndMatrix keyframe(invTransform * GetKeyframe(scaleValue, positValue, rotationValue) * transform);
				keyframe.PolarDecomposition(animTransformMatrix, animScale, stretchAxis);

				ndVector euler0;
				ndVector euler(animTransformMatrix.CalcPitchYawRoll(euler0));

				rotationValue.m_x = euler.m_x;
				rotationValue.m_y = euler.m_y;
				rotationValue.m_z = euler.m_z;

				positValue.m_x = animTransformMatrix.m_posit.m_x;
				positValue.m_y = animTransformMatrix.m_posit.m_y;
				positValue.m_z = animTransformMatrix.m_posit.m_z;

				positNode = positNode->GetNext();
				rotationNode = rotationNode->GetNext();
			}
		}

		for (ndMeshEffectNode* child = ent->GetFirstChild(); child; child = child->GetNext())
		{
			entBuffer[stack] = child;
			stack++;
			ndAssert(stack < sizeof(entBuffer) / sizeof (entBuffer[0]));
		}
	}
}