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

#define ND_MESH_MAX_STACK_DEPTH	2048

ndMesh::ndMesh()
	:ndClassAlloc()
	,m_matrix(ndGetIdentityMatrix())
	,m_meshMatrix(ndGetIdentityMatrix())
	,m_name()
	,m_mesh()
	,m_scale()
	,m_posit()
	,m_rotation()
	,m_parent(nullptr)
	,m_childNode(nullptr)
{
}

ndMesh::ndMesh(const ndMesh& src)
	:ndClassAlloc()
	,m_matrix(src.m_matrix)
	,m_meshMatrix(src.m_meshMatrix)
	,m_name(src.m_name)
	,m_mesh(src.m_mesh)
	,m_scale()
	,m_posit()
	,m_rotation()
	,m_parent(nullptr)
	,m_childNode(nullptr)
{
	ndAssert(0);
	//for (ndCurve::ndNode* node = src.m_scale.GetFirst(); node; node = node->GetNext())
	//{
	//	m_scale.Append(node->GetInfo());
	//}
	//
	//for (ndCurve::ndNode* node = src.m_posit.GetFirst(); node; node = node->GetNext())
	//{
	//	m_posit.Append(node->GetInfo());
	//}
	//
	//for (ndCurve::ndNode* node = src.m_rotation.GetFirst(); node; node = node->GetNext())
	//{
	//	m_rotation.Append(node->GetInfo());
	//}
}

//ndMesh::ndMesh(const ndShapeInstance& src)
ndMesh::ndMesh(const ndShapeInstance&)
	:ndClassAlloc()
	,m_matrix(ndGetIdentityMatrix())
	,m_meshMatrix(ndGetIdentityMatrix())
	,m_name()
	,m_mesh()
	,m_scale()
	,m_posit()
	,m_rotation()
	,m_parent(nullptr)
	,m_childNode(nullptr)
{
	ndAssert(0);
	// TO DO: build the mesh form the collision shape;
}

ndMesh::~ndMesh()
{
}

void ndMesh::AddChild(const ndSharedPtr<ndMesh>& child)
{
	ndAssert(!child->m_parent);
	child->m_parent = this;
	child->m_childNode = m_children.Append(child);
}

void ndMesh::RemoveChild(const ndSharedPtr<ndMesh>& child)
{
	ndAssert(child->m_childNode);
	ndAssert(child->m_parent && (child->m_parent == this));

	ndList<ndSharedPtr<ndMesh>>::ndNode* const node = child->m_childNode;
	child->m_parent = nullptr;
	child->m_childNode = nullptr;

	m_children.Remove(node);
}

ndList<ndSharedPtr<ndMesh>>& ndMesh::GetChildren()
{
	return m_children;
}

const ndList<ndSharedPtr<ndMesh>>& ndMesh::GetChildren() const
{
	return m_children;
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

ndMesh* ndMesh::FindChild(const char* const name) const
{
	ndFixSizeArray<ndMesh*, ND_MESH_MAX_STACK_DEPTH> stack(0);
	
	stack.PushBack((ndMesh*)this);
	while (stack.GetCount())
	{
		ndMesh* const node = stack.Pop();
		if (!strcmp(node->m_name.GetStr(), name))
		{
			return node;
		}

		for (ndList<ndSharedPtr<ndMesh>>::ndNode* childNode = node->GetChildren().GetFirst(); childNode; childNode = childNode->GetNext())
		{
			ndMesh* const child = *childNode->GetInfo();
			stack.PushBack(child);
		}
	}

	return nullptr;
}

ndMatrix ndMesh::CalculateGlobalMatrix(ndMesh* const parent) const
{
	ndMatrix matrix(ndGetIdentityMatrix());
	for (const ndMesh* ptr = this; ptr != parent; ptr = ptr->m_parent)
	{
		matrix = matrix * ptr->m_matrix;
	}
	return matrix;
}

void ndMesh::ApplyTransform(const ndMatrix& transform)
{
	ndFixSizeArray<ndMesh*, ND_MESH_MAX_STACK_DEPTH> entBuffer(0);

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

	entBuffer.PushBack(this);
	ndMatrix invTransform(transform.Inverse4x4());
	while (entBuffer.GetCount())
	{
		ndMesh* const node = entBuffer.Pop();

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

		for (ndList<ndSharedPtr<ndMesh>>::ndNode* childNode = node->GetChildren().GetFirst(); childNode; childNode = childNode->GetNext())
		{
			ndMesh* const child = *childNode->GetInfo();
			entBuffer.PushBack(child);
		}
	}
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionTree(bool optimize)
{
	ndPolygonSoupBuilder meshBuilder;
	meshBuilder.Begin();

	ndFixSizeArray<ndMesh*, 1024> entBuffer;
	ndFixSizeArray<ndMatrix, 1024> matrixBuffer;

	entBuffer.PushBack(this);
	matrixBuffer.PushBack(m_matrix.OrthoInverse());

	while (entBuffer.GetCount())
	{
		ndMesh* node = entBuffer.Pop();
		ndMatrix matrix(node->m_matrix * matrixBuffer.Pop());

		ndSharedPtr<ndMeshEffect> meshEffect = node->GetMesh();
		if (*meshEffect)
		{
			ndInt32 vertexStride = meshEffect->GetVertexStrideInByte() / ndInt32(sizeof(ndFloat64));
			const ndFloat64* const vertexData = meshEffect->GetVertexPool();
		
			ndInt32 mark = meshEffect->IncLRU();
			ndPolyhedra::Iterator iter(*(*meshEffect));
		
			ndMatrix worldMatrix(node->m_meshMatrix * matrix);
			for (iter.Begin(); iter; iter++)
			{
				ndEdge* const edge = &(*iter);
				if ((edge->m_incidentFace >= 0) && (edge->m_mark != mark))
				{
					ndFixSizeArray<ndVector, 256> face;
					ndEdge* ptr = edge;
					do
					{
						ndInt32 i = ptr->m_incidentVertex * vertexStride;
						ndVector point(ndFloat32(vertexData[i + 0]), ndFloat32(vertexData[i + 1]), ndFloat32(vertexData[i + 2]), ndFloat32(1.0f));
						face.PushBack(worldMatrix.TransformVector(point));
						ptr->m_mark = mark;
						ptr = ptr->m_next;
					} while (ptr != edge);
		
					ndInt32 materialIndex = meshEffect->GetFaceMaterial(edge);
					meshBuilder.AddFace(&face[0].m_x, sizeof(ndVector), face.GetCount(), materialIndex);
				}
			}
		}

		for (ndList<ndSharedPtr<ndMesh>>::ndNode* childNode = node->GetChildren().GetFirst(); childNode; childNode = childNode->GetNext())
		{
			ndMesh* const child = *childNode->GetInfo();
			entBuffer.PushBack(child);
			matrixBuffer.PushBack(matrix);
		}
	}
	meshBuilder.End(optimize);
	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeStatic_bvh(meshBuilder)));
	return shape;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionCompound(bool lowDetail)
{
	const ndInt32 pointsCount = m_mesh->GetVertexCount();
	const ndInt32 pointsStride = ndInt32 (m_mesh->GetVertexStrideInByte() / sizeof (ndFloat64));
	const ndFloat64* const pointsBuffer = m_mesh->GetVertexPool();

	ndArray<ndReal> meshPoints;
	for (ndInt32 i = 0; i < pointsCount; ++i)
	{
		meshPoints.PushBack(ndReal(pointsBuffer[i * pointsStride + 0]));
		meshPoints.PushBack(ndReal(pointsBuffer[i * pointsStride + 1]));
		meshPoints.PushBack(ndReal(pointsBuffer[i * pointsStride + 2]));
	}

	ndArray<ndInt32> indices;
	ndInt32 mark = m_mesh->IncLRU();
	ndPolyhedra::Iterator iter(**m_mesh);
	for (iter.Begin(); iter; iter++)
	{
		ndEdge* const face = &iter.GetNode()->GetInfo();
		if ((face->m_mark != mark) && (face->m_incidentFace > 0))
		{
			ndEdge* ptr = face;
			ptr->m_mark = mark;
			indices.PushBack(ptr->m_incidentVertex);

			ptr = ptr->m_next;
			ptr->m_mark = mark;
			indices.PushBack(ptr->m_incidentVertex);

			ptr = ptr->m_next;
			do
			{
				indices.PushBack(ptr->m_incidentVertex);
				ptr->m_mark = mark;
	
				ptr = ptr->m_next;
			} while (ptr != face);
		}
	}
	
	nd::VHACD::IVHACD* const interfaceVHACD = nd::VHACD::CreateVHACD();
	nd::VHACD::IVHACD::Parameters paramsVHACD;
	paramsVHACD.m_concavityToVolumeWeigh = lowDetail ? 1.0f : 0.5f;
	interfaceVHACD->Compute(&meshPoints[0], uint32_t(meshPoints.GetCount() / 3), (uint32_t*)&indices[0], uint32_t(indices.GetCount() / 3), paramsVHACD);
	
	ndSharedPtr<ndShapeInstance> compoundShapeInstance (new ndShapeInstance(new ndShapeCompound()));
	ndShapeCompound* const compoundShape = compoundShapeInstance->GetShape()->GetAsShapeCompound();
	compoundShape->BeginAddRemove();
	ndInt32 hullCount = ndInt32(interfaceVHACD->GetNConvexHulls());
	ndArray<ndVector> convexMeshPoints;
	for (ndInt32 i = 0; i < hullCount; ++i)
	{
		nd::VHACD::IVHACD::ConvexHull ch;
		interfaceVHACD->GetConvexHull(uint32_t(i), ch);
		convexMeshPoints.SetCount(ndInt32(ch.m_nPoints));
		for (ndInt32 j = 0; j < ndInt32(ch.m_nPoints); ++j)
		{
			ndVector p(ndFloat32(ch.m_points[j * 3 + 0]), ndFloat32(ch.m_points[j * 3 + 1]), ndFloat32(ch.m_points[j * 3 + 2]), ndFloat32(0.0f));
			convexMeshPoints[j] = p;
		}
		ndShapeInstance hullShape(new ndShapeConvexHull(ndInt32(convexMeshPoints.GetCount()), sizeof(ndVector), 0.01f, &convexMeshPoints[0].m_x));
		compoundShape->AddCollision(&hullShape);
	}
	compoundShape->EndAddRemove();
	compoundShapeInstance->SetLocalMatrix(m_meshMatrix);
	
	interfaceVHACD->Clean();
	interfaceVHACD->Release();

	return compoundShapeInstance;
}