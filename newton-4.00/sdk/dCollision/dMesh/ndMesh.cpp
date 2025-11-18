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
#include "VHACD.h"
#include "ndMesh.h"
#include "ndCollision.h"

#define ND_MESH_MAX_STACK_DEPTH	2048

ndMesh::ndMesh()
	:ndClassAlloc()
	,m_matrix(ndGetIdentityMatrix())
	,m_name()
	,m_mesh()
	,m_scale()
	,m_posit()
	,m_rotation()
	,m_parent(nullptr)
	,m_childNode(nullptr)
	,m_boneTarget(ndVector::m_wOne)
	,m_type(m_node)
{
}

ndMesh::ndMesh(const ndMesh&)
	:ndClassAlloc()
{
	ndAssert(0);
}

ndMesh::ndMesh(const ndShapeInstance&)
	:ndClassAlloc()
	,m_matrix(ndGetIdentityMatrix())
	,m_name()
	,m_mesh()
	,m_scale()
	,m_posit()
	,m_rotation()
	,m_parent(nullptr)
	,m_childNode(nullptr)
	,m_boneTarget(ndVector::m_wOne)
	,m_type(m_node)
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

ndMesh* ndMesh::GetParent()
{
	return m_parent;
}

const ndMesh* ndMesh::GetParent() const
{
	return m_parent;
}

ndList<ndSharedPtr<ndMesh>>& ndMesh::GetChildren()
{
	return m_children;
}

const ndList<ndSharedPtr<ndMesh>>& ndMesh::GetChildren() const
{
	return m_children;
}

ndMesh::ndNodeType ndMesh::GetNodeType() const
{
	return m_type;
}

void ndMesh::SetNodeType(ndNodeType type)
{
	m_type = type;
}

ndVector ndMesh::GetBoneTarget() const
{
	return m_boneTarget;
}

void ndMesh::SetBoneTarget(const ndVector& target)
{
	m_boneTarget = target;
	m_boneTarget.m_w = ndFloat32(1.0f);
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

ndMesh* ndMesh::FindByName(const ndString& name) const
{
	ndMesh* const self = (ndMesh*)this;
	for (ndMesh* node = self->IteratorFirst(); node; node = node->IteratorNext())
	{
		if (name == node->m_name)
		{
			return node;
		}
	}
	return nullptr;
}

ndMesh* ndMesh::FindByClosestMatch(const ndString& name) const
{
	ndMesh* closestMatch = FindByName(name);
	if (!closestMatch)
	{
		ndInt32 bestScore = 10000;
		ndMesh* const self = (ndMesh*)this;
		for (ndMesh* node = self->IteratorFirst(); node && bestScore; node = node->IteratorNext())
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

ndMesh* ndMesh::IteratorFirst()
{
	ndMesh* ptr = this;
	while (ptr->m_children.GetCount())
	{
		ptr = *ptr->m_children.GetFirst()->GetInfo();
	}
	return ptr;
}

ndMesh* ndMesh::IteratorNext()
{
	if (m_childNode)
	{
		ndList<ndSharedPtr<ndMesh>>::ndNode* next = m_childNode->GetNext();
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

	const ndMatrix invTransform(transform.Inverse4x4());
	for (ndMesh* node = IteratorFirst(); node; node = node->IteratorNext())
	{
		const ndMatrix entMatrix(invTransform * node->m_matrix * transform);
		node->m_matrix = entMatrix;

		ndSharedPtr<ndMeshEffect> mesh (node->GetMesh());
		if (mesh)
		{
			//const ndMatrix meshMatrix(invTransform * node->m_meshMatrix * transform);
			//node->m_meshMatrix = meshMatrix;
			//mesh->ApplyTransform(transform);
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
	}
}

ndMatrix ndMesh::CalculateLocalMatrix(ndVector& sizeOut) const
{
	ndSharedPtr<ndMeshEffect> meshEffect = GetMesh();

	const ndInt32 pointsCount = meshEffect->GetVertexCount();
	const ndInt32 pointsStride = ndInt32(meshEffect->GetVertexStrideInByte() / sizeof(ndFloat64));
	const ndFloat64* const pointsBuffer = meshEffect->GetVertexPool();

	ndVector minP(ndFloat32(1.0e10f));
	ndVector maxP(ndFloat32(-1.0e10f));
	for (ndInt32 i = 0; i < pointsCount; ++i)
	{
		ndFloat32 x = ndFloat32(pointsBuffer[i * pointsStride + 0]);
		ndFloat32 y = ndFloat32(pointsBuffer[i * pointsStride + 1]);
		ndFloat32 z = ndFloat32(pointsBuffer[i * pointsStride + 2]);
		const ndVector p(x, y, z, ndFloat32(0.0f));
		minP = minP.GetMin(p);
		maxP = maxP.GetMax(p);
	}

	ndVector size(ndVector::m_half * (maxP - minP));
	ndVector origin(ndVector::m_half * (maxP + minP));
	ndMatrix covariance(ndGetZeroMatrix());
	for (ndInt32 i = 0; i < pointsCount; ++i)
	{
		ndFloat32 x = ndFloat32(pointsBuffer[i * pointsStride + 0]);
		ndFloat32 y = ndFloat32(pointsBuffer[i * pointsStride + 1]);
		ndFloat32 z = ndFloat32(pointsBuffer[i * pointsStride + 2]);
		const ndVector q(x, y, z, ndFloat32(0.0f));
		const ndVector p((q - origin) & ndVector::m_triplexMask);
		ndAssert(p.m_w == ndFloat32(0.0f));
		const ndMatrix matrix(ndCovarianceMatrix(p, p));
		covariance.m_front += matrix.m_front;
		covariance.m_up += matrix.m_up;
		covariance.m_right += matrix.m_right;
	}
	const ndVector eigen(covariance.EigenVectors() & ndVector::m_triplexMask);
	covariance.m_posit = origin;
	covariance.m_posit.m_w = ndFloat32(1.0f);

	sizeOut = size;
	sizeOut.m_w = ndFloat32(0.0f);
	return covariance;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionBox()
{
	ndSharedPtr<ndMeshEffect> meshEffect = GetMesh();
	ndAssert(*meshEffect);
	
	ndVector size;
	ndMatrix localMatrix(CalculateLocalMatrix(size));
	size = size.Scale(ndFloat32(2.0f));
	ndSharedPtr<ndShapeInstance> box(new ndShapeInstance(new ndShapeBox(size.m_x, size.m_y, size.m_z)));
	box->SetLocalMatrix(localMatrix);
	return box;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionSphere()
{
	ndSharedPtr<ndMeshEffect> meshEffect = GetMesh();
	ndAssert(*meshEffect);

	ndVector size;
	ndMatrix localMatrix(CalculateLocalMatrix(size));
	ndSharedPtr<ndShapeInstance> sphere (new ndShapeInstance(new ndShapeSphere(size.m_x)));
	
	sphere->SetLocalMatrix(localMatrix);
	return sphere;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionCapsule()
{
	ndSharedPtr<ndMeshEffect> meshEffect = GetMesh();
	ndAssert(*meshEffect);

	ndVector size;
	ndMatrix localMatrix(CalculateLocalMatrix(size));
	if ((size.m_y >= size.m_x) && (size.m_y >= size.m_z))
	{
		ndSwap(size.m_x, size.m_y);
		localMatrix = ndRollMatrix(ndFloat32(90.0f) * ndDegreeToRad) * localMatrix;
	}
	else if ((size.m_z >= size.m_x) && (size.m_z >= size.m_y))
	{
		ndSwap(size.m_x, size.m_z);
		localMatrix = ndYawMatrix(ndFloat32(90.0f) * ndDegreeToRad) * localMatrix;
	}

	ndFloat32 radios = size.m_y;
	ndFloat32 high = ndFloat32(2.0f) * ndMax(size.m_x - size.m_y, ndFloat32(0.025f));

	ndSharedPtr<ndShapeInstance> capsule(new ndShapeInstance(new ndShapeCapsule(radios, radios, high)));

	capsule->SetLocalMatrix(localMatrix);
	return capsule;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionTire()
{
	ndSharedPtr<ndMeshEffect> meshEffect = GetMesh();
	ndAssert(*meshEffect);

	ndVector size;
	ndMatrix localMatrix(CalculateLocalMatrix(size));

	ndFloat32 width = size.m_x;
	ndFloat32 radius = size.m_y;
	ndSharedPtr<ndShapeInstance> tireShape(new ndShapeInstance(new ndShapeChamferCylinder(ndFloat32(0.75f), ndFloat32(0.5f))));
	ndVector scale(ndFloat32(4.0f) * width, radius, radius, 0.0f);
	tireShape->SetScale(scale);
	tireShape->SetLocalMatrix(localMatrix);

	return tireShape;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionChamferCylinder()
{
	ndSharedPtr<ndMeshEffect> meshEffect = GetMesh();
	ndAssert(*meshEffect);

	ndVector size;
	ndMatrix localMatrix(CalculateLocalMatrix(size));

	ndFloat32 radius = size.m_x - size.m_z;
	ndFloat32 width = size.m_z * ndFloat32(2.0f);
	ndSharedPtr<ndShapeInstance> tireShape(new ndShapeInstance(new ndShapeChamferCylinder(radius, width)));
	tireShape->SetLocalMatrix(localMatrix);
	return tireShape;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionConvex()
{
	ndAssert(*m_mesh);
	ndSharedPtr<ndShapeInstance>shape(m_mesh->CreateConvexCollision(1.0e-3f));

	const ndMatrix matrix(shape->GetLocalMatrix());
	shape->SetLocalMatrix(matrix);
	return shape;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionCompound(bool lowDetail)
{
	const ndInt32 pointsCount = m_mesh->GetVertexCount();
	const ndInt32 pointsStride = ndInt32(m_mesh->GetVertexStrideInByte() / sizeof(ndFloat64));
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
	
	ndSharedPtr<nd::VHACD::IVHACD> interfaceVHACD (nd::VHACD::CreateVHACD());
	nd::VHACD::IVHACD::Parameters paramsVHACD;
	paramsVHACD.m_concavityToVolumeWeigh = lowDetail ? 1.0f : 0.5f;
	interfaceVHACD->Compute(&meshPoints[0], uint32_t(meshPoints.GetCount() / 3), (uint32_t*)&indices[0], uint32_t(indices.GetCount() / 3), paramsVHACD);
	
	ndSharedPtr<ndShapeInstance> compoundShapeInstance(new ndShapeInstance(new ndShapeCompound()));
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
	
	compoundShapeInstance->SetLocalMatrix(ndGetIdentityMatrix());
	
	return compoundShapeInstance;
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
		
			//ndMatrix worldMatrix(node->m_meshMatrix * matrix);
			const ndMatrix worldMatrix(matrix);
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

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollision()
{
	ndString tmpName(GetName());
	tmpName.ToLower();
	const char* const name = tmpName.GetStr();

	ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeNull()));
	if (strstr(name, "box"))
	{
		shape = CreateCollisionBox();
	}
	else if (strstr(name, "sphere"))
	{
		shape = CreateCollisionSphere();
	}
	else if (strstr(name, "capsule"))
	{
		shape = CreateCollisionCapsule();
	}
	else if (strstr(name, "tire"))
	{
		shape = CreateCollisionTire();
	}
	else if (strstr(name, "convexhull"))
	{
		shape = CreateCollisionConvex();
	}
	else if (strstr(name, "collisiontree"))
	{
		shape = CreateCollisionTree();
	}
	else if(strstr(name, "convexapproximation"))
	{
		shape = CreateCollisionCompound();
	}
	else
	{
		ndAssert(0);
	}
	return shape;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionFromChildren()
{
	ndArray<ndVector> points;
	ndFixSizeArray<ndSharedPtr<ndShapeInstance>, 256> shapeArray;

	for (ndList<ndSharedPtr<ndMesh>>::ndNode* ptr = GetChildren().GetFirst(); ptr; ptr = ptr->GetNext())
	{
		ndMesh* const meshNode = *ptr->GetInfo();
		ndString tmpName(meshNode->GetName());
		tmpName.ToLower();
		const char* const name = tmpName.GetStr();
	
		if (strstr(name, "sphere"))
		{
			ndSharedPtr<ndShapeInstance> subShape(meshNode->CreateCollision());
			const ndMatrix matrix(subShape->GetLocalMatrix() * meshNode->m_matrix);
			subShape->SetLocalMatrix(matrix);
			shapeArray.PushBack(subShape);
		}
		else if (strstr(name, "box"))
		{
			ndSharedPtr<ndShapeInstance> subShape(meshNode->CreateCollision());
			const ndMatrix matrix(subShape->GetLocalMatrix() * meshNode->m_matrix);
			subShape->SetLocalMatrix(matrix);
			shapeArray.PushBack(subShape);
		}
		else if (strstr(name, "capsule"))
		{
			ndSharedPtr<ndShapeInstance> subShape(meshNode->CreateCollision());
			const ndMatrix matrix(subShape->GetLocalMatrix() * meshNode->m_matrix);
			subShape->SetLocalMatrix(matrix);
			shapeArray.PushBack(subShape);
		}
		else if (strstr(name, "convexhull"))
		{
			ndSharedPtr<ndShapeInstance> subShape(meshNode->CreateCollision());
			const ndMatrix matrix(subShape->GetLocalMatrix() * meshNode->m_matrix);
			subShape->SetLocalMatrix(matrix);
			shapeArray.PushBack(subShape);
		}
		else if (strstr(name, "vhacd"))
		{
			ndAssert(0);
			//ndArray<ndInt32> indices;
			//ndDemoMesh* const mesh = (ndDemoMesh*)*node->GetInfo()->GetMesh();
			//ndAssert(mesh);
			//mesh->GetVertexArray(points);
			//mesh->GetIndexArray(indices);
			//
			//ndArray<ndTriplex> meshPoints;
			//for (ndInt32 i = 0; i < points.GetCount(); ++i)
			//{
			//	ndTriplex p;
			//	p.m_x = points[i].m_x;
			//	p.m_y = points[i].m_y;
			//	p.m_z = points[i].m_z;
			//	meshPoints.PushBack(p);
			//}
			//nd_::VHACD::IVHACD* const interfaceVHACD = nd_::VHACD::CreateVHACD();
			//
			//nd_::VHACD::IVHACD::Parameters paramsVHACD;
			////paramsVHACD.m_concavityToVolumeWeigh = 1.0;
			//paramsVHACD.m_concavityToVolumeWeigh = 0.5f;
			//interfaceVHACD->Compute(&meshPoints[0].m_x, uint32_t(points.GetCount()),
			//	(uint32_t*)&indices[0], uint32_t(indices.GetCount()) / 3, paramsVHACD);
			//
			//ndInt32 hullCount = ndInt32(interfaceVHACD->GetNConvexHulls());
			//ndArray<ndVector> convexMeshPoints;
			//for (ndInt32 i = 0; i < hullCount; ++i)
			//{
			//	nd_::VHACD::IVHACD::ConvexHull ch;
			//	interfaceVHACD->GetConvexHull(uint32_t(i), ch);
			//	convexMeshPoints.SetCount(ndInt32(ch.m_nPoints));
			//	for (ndInt32 j = 0; j < ndInt32(ch.m_nPoints); ++j)
			//	{
			//		ndVector p(ndFloat32(ch.m_points[j * 3 + 0]), ndFloat32(ch.m_points[j * 3 + 1]), ndFloat32(ch.m_points[j * 3 + 2]), ndFloat32(0.0f));
			//		convexMeshPoints[j] = p;
			//	}
			//	shapeArray.PushBack(new ndShapeInstance(new ndShapeConvexHull(ndInt32(convexMeshPoints.GetCount()), sizeof(ndVector), 0.01f, &convexMeshPoints[0].m_x)));
			//	const ndMatrix matrix(node->GetInfo()->GetMeshMatrix() * node->GetInfo()->GetCurrentMatrix());
			//	shapeArray[shapeArray.GetCount() - 1]->SetLocalMatrix(matrix);
			//}
			//
			//interfaceVHACD->Clean();
			//interfaceVHACD->Release();
		}
	}
	
	ndAssert(shapeArray.GetCount());
	if (shapeArray.GetCount() > 1)
	{
		ndSharedPtr<ndShapeInstance> compoundInstance (new ndShapeInstance(new ndShapeCompound()));
		ndShapeCompound* const compound = compoundInstance->GetShape()->GetAsShapeCompound();
		
		compound->BeginAddRemove();
		for (ndInt32 i = 0; i < shapeArray.GetCount(); ++i)
		{
			compound->AddCollision(*shapeArray[i]);
		}
		compound->EndAddRemove();
		shapeArray[0] = compoundInstance;
	}
	return shapeArray[0];
}