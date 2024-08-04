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

#include "ndModelStdafx.h"
#include "ndMesh.h"

ndMesh::ndMesh(ndMesh* const parent)
	:ndNodeHierarchy<ndMesh>()
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

ndMesh::ndMesh(const ndMesh& src)
	:ndNodeHierarchy<ndMesh>(src)
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

//ndMesh::ndMesh(const ndShapeInstance& src)
ndMesh::ndMesh(const ndShapeInstance&)
	:ndNodeHierarchy<ndMesh>()
	,m_matrix(ndGetIdentityMatrix())
	,m_meshMatrix(ndGetIdentityMatrix())
	,m_name()
	,m_mesh()
	,m_scale()
	,m_posit()
	,m_rotation()
{
	ndAssert(0);
	// TO DO: build the mesh form the collision shape;
}

ndMesh::~ndMesh()
{
}

const ndString& ndMesh::GetName() const
{
	return m_name;
}

ndMesh::ndCurve& ndMesh::GetScaleCurve()
{
	return m_scale;
}

const ndMesh::ndCurve& ndMesh::GetScaleCurve() const
{
	return m_scale;
}

ndMesh::ndCurve& ndMesh::GetPositCurve()
{
	return m_posit;
}

const ndMesh::ndCurve& ndMesh::GetPositCurve() const
{
	return m_posit;
}

ndMesh::ndCurve& ndMesh::GetRotationCurve()
{
	return m_rotation;
}

const ndMesh::ndCurve& ndMesh::GetRotationCurve() const
{
	return m_rotation;
}

void ndMesh::SetName(const ndString& name)
{
	m_name = name;
}

ndMesh* ndMesh::CreateClone() const
{
	return new ndMesh(*this);
}

ndSharedPtr<ndMeshEffect>& ndMesh::GetMesh()
{
	return m_mesh;
}

const ndSharedPtr<ndMeshEffect>& ndMesh::GetMesh() const
{
	return m_mesh;
}

void ndMesh::SetMesh(const ndSharedPtr<ndMeshEffect>& mesh)
{
	m_mesh = mesh;
}

ndMatrix ndMesh::CalculateGlobalMatrix(ndMesh* const parent) const
{
	ndMatrix matrix(ndGetIdentityMatrix());
	for (const ndMesh* ptr = this; ptr != parent; ptr = ptr->GetParent())
	{
		matrix = matrix * ptr->m_matrix;
	}
	return matrix;
}

void ndMesh::ApplyTransform(const ndMatrix& transform)
{
	ndInt32 stack = 1;
	ndMesh* entBuffer[1024];

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
		ndMesh* const node = entBuffer[stack];

		ndMatrix entMatrix(invTransform * node->m_matrix * transform);
		node->m_matrix = entMatrix;

		ndSharedPtr<ndMeshEffect> mesh (node->GetMesh());
		if (mesh)
		{
			ndMatrix meshMatrix(invTransform * node->m_meshMatrix * transform);
			node->m_meshMatrix = meshMatrix;
			mesh->ApplyTransform(transform);
		}

		ndMesh::ndCurve& positCurve = node->GetPositCurve();
		ndMesh::ndCurve& rotationCurve = node->GetRotationCurve();
		if (positCurve.GetCount() || rotationCurve.GetCount())
		{
			ndMesh::ndCurve::ndNode* positNode = node->GetPositCurve().GetFirst();
			ndMesh::ndCurve::ndNode* rotationNode = node->GetRotationCurve().GetFirst();

			ndMesh::ndCurveValue scaleValue;
			scaleValue.m_x = 1.0f;
			scaleValue.m_y = 1.0f;
			scaleValue.m_z = 1.0f;
			for (ndInt32 i = 0; i < positCurve.GetCount(); ++i)
			{
				ndMesh::ndCurveValue& positValue = positNode->GetInfo();
				ndMesh::ndCurveValue& rotationValue = rotationNode->GetInfo();

				ndVector animScale;
				ndMatrix stretchAxis;
				ndMatrix animTransformMatrix;
				ndMatrix keyframe(invTransform * GetKeyframe(scaleValue, positValue, rotationValue) * transform);
				keyframe.PolarDecomposition(animTransformMatrix, animScale, stretchAxis);

				ndVector euler0;
				ndVector euler(animTransformMatrix.CalcPitchYawRoll(euler0));

				rotationValue.m_x = ndReal (euler.m_x);
				rotationValue.m_y = ndReal (euler.m_y);
				rotationValue.m_z = ndReal (euler.m_z);

				positValue.m_x = ndReal (animTransformMatrix.m_posit.m_x);
				positValue.m_y = ndReal (animTransformMatrix.m_posit.m_y);
				positValue.m_z = ndReal (animTransformMatrix.m_posit.m_z);

				positNode = positNode->GetNext();
				rotationNode = rotationNode->GetNext();
			}
		}

		for (ndMesh* child = node->GetFirstChild(); child; child = child->GetNext())
		{
			entBuffer[stack] = child;
			stack++;
			ndAssert(stack < sizeof(entBuffer) / sizeof (entBuffer[0]));
		}
	}
}
