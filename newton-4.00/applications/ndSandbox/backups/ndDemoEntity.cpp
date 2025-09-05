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

#include "ndSandboxStdafx.h"
#include "ndDemoMesh.h"
#include "ndMeshLoader.h"
#include "ndDemoEntity.h"
#include "ndDemoCamera.h"
#include "ndDemoSkinMesh.h"

ndDemoEntity::ndDemoEntity(const ndMatrix& matrix)
	:ndSharedNodeHierarchy<ndDemoEntity>()
	,m_matrix(matrix)
	,m_curPosition(matrix.m_posit)
	,m_nextPosition(matrix.m_posit)
	,m_curRotation(matrix)
	,m_nextRotation(matrix)
	,m_meshMatrix(ndGetIdentityMatrix())
	,m_mesh()
	,m_rootNode(nullptr)
	,m_name(nullptr)
	,m_lock()
	,m_isDead(false)
	,m_isVisible(true)
	,m_castShadow(true)
{
}

ndDemoEntity::ndDemoEntity(ndDemoEntityManager* const scene, ndMesh* const meshEffectNode)
	:ndSharedNodeHierarchy<ndDemoEntity>()
	,m_matrix(meshEffectNode->m_matrix)
	,m_curPosition(meshEffectNode->m_matrix.m_posit)
	,m_nextPosition(meshEffectNode->m_matrix.m_posit)
	,m_curRotation(meshEffectNode->m_matrix)
	,m_nextRotation(m_curRotation)
	,m_meshMatrix(ndGetIdentityMatrix())
	,m_mesh()
	,m_rootNode(nullptr)
	,m_name(nullptr)
	,m_lock()
	,m_isDead(false)
	,m_isVisible(true)
	,m_castShadow(true)
{
	ndFixSizeArray<ndMesh*, 1024> effectNodeBuffer;
	ndFixSizeArray<ndDemoEntity*, 1024> parentEntityBuffer;

	struct EntityMeshPair
	{
		EntityMeshPair()
		{
		}

		EntityMeshPair(ndDemoEntity* const entity, ndMesh* const effectNode)
			:m_effectNode(effectNode)
			,m_entity(entity)
		{
		}

		ndMesh* m_effectNode;
		ndDemoEntity* m_entity;
	};
	ndFixSizeArray<EntityMeshPair, 1024> meshList;

	bool isRoot = true;
	parentEntityBuffer.PushBack(nullptr);
	effectNodeBuffer.PushBack(meshEffectNode);
	while (effectNodeBuffer.GetCount())
	{
		ndMesh* const effectNode = effectNodeBuffer.Pop();
		ndDemoEntity* const parent = parentEntityBuffer.Pop();

		ndDemoEntity* entity = isRoot ? this : new ndDemoEntity(effectNode->m_matrix);
		if (!isRoot)
		{
			ndSharedPtr<ndDemoEntity> sharedEnt(entity);
			parent->AddChild(sharedEnt);
		}
		isRoot = false;

		entity->SetName(effectNode->GetName().GetStr());

		ndSharedPtr<ndMeshEffect> meshEffect (effectNode->GetMesh());
		if (*meshEffect)
		{
			meshList.PushBack(EntityMeshPair(entity, effectNode));
		}

		for (ndMesh* child = effectNode->GetLastChild(); child; child = child->GetPrev())
		{
			effectNodeBuffer.PushBack(child);
			parentEntityBuffer.PushBack(entity);
		}
	}

	for (ndInt32 i = 0; i < meshList.GetCount(); ++i)
	{
		ndDemoMeshInterface* mesh = nullptr;
		ndDemoEntity* const entity = meshList[i].m_entity;
		ndMesh* const effectNode = meshList[i].m_effectNode;

		ndAssert(effectNode);
		ndSharedPtr<ndMeshEffect> meshEffect (effectNode->GetMesh());
		if (meshEffect->GetVertexWeights().GetCount())
		{
			mesh = new ndDemoSkinMesh(entity, *meshEffect, scene->GetShaderCache());
		}
		else
		{
			mesh = new ndDemoMesh(effectNode->GetName().GetStr(), *meshEffect, scene->GetShaderCache());
		}
		entity->SetMesh(ndSharedPtr<ndDemoMeshInterface>(mesh), effectNode->m_meshMatrix);
		
		if ((effectNode->GetName().Find("hidden") >= 0) || (effectNode->GetName().Find("Hidden") >= 0))
		{
			mesh->m_isVisible = false;
			entity->m_isVisible = false;
			entity->m_castShadow = false;
		}
	}
}

ndDemoEntity::ndDemoEntity(const ndDemoEntity& copyFrom)
	:ndSharedNodeHierarchy<ndDemoEntity>(copyFrom)
	,m_matrix(copyFrom.m_matrix)
	,m_curPosition(copyFrom.m_curPosition)
	,m_nextPosition(copyFrom.m_nextPosition)
	,m_curRotation(copyFrom.m_curRotation)
	,m_nextRotation(copyFrom.m_nextRotation)
	,m_meshMatrix(copyFrom.m_meshMatrix)
	,m_mesh(copyFrom.m_mesh)
	,m_rootNode(nullptr)
	,m_name(copyFrom.m_name)
	,m_lock()
	,m_isDead(false)
	,m_isVisible(copyFrom.m_isVisible)
	,m_castShadow(copyFrom.m_castShadow)
{
	if (*m_mesh && m_mesh->GetAsDemoSkinMesh())
	{
		m_mesh = ndSharedPtr<ndDemoMeshInterface> (m_mesh->Clone(this));
	}
}

ndDemoEntity::~ndDemoEntity(void)
{
}

ndDemoEntity* ndDemoEntity::CreateClone () const
{
	return new ndDemoEntity(*this);
}

const ndString& ndDemoEntity::GetName() const
{
	return m_name;
}

void ndDemoEntity::SetName(const ndString& name)
{
	m_name = name;
}

void ndDemoEntity::Hide()
{
	m_isVisible = false;
}

void ndDemoEntity::UnHide()
{
	m_isVisible = true;
}

void ndDemoEntity::SetShadowMode(bool mode)
{
	m_castShadow = mode;
}

bool ndDemoEntity::CastShadow() const
{
	return m_castShadow;
}

ndSharedPtr<ndDemoMeshInterface> ndDemoEntity::GetMesh()
{
	return m_mesh;
}

ndSharedPtr<ndDemoMeshInterface> ndDemoEntity::GetMesh() const
{
	return m_mesh;
}

void ndDemoEntity::SetMesh(ndSharedPtr<ndDemoMeshInterface> mesh, const ndMatrix& meshMatrix)
{
	m_mesh = mesh;
	m_meshMatrix = meshMatrix;
}

const ndMatrix& ndDemoEntity::GetMeshMatrix() const
{
	return m_meshMatrix;
}

void ndDemoEntity::SetMeshMatrix(const ndMatrix& matrix)
{
	m_meshMatrix = matrix;
}

ndMatrix ndDemoEntity::GetCurrentMatrix () const
{
	return ndCalculateMatrix(m_curRotation, m_curPosition);
}

ndAnimKeyframe ndDemoEntity::GetCurrentTransform() const
{
	return ndAnimKeyframe(m_curPosition, m_curRotation);
}

ndMatrix ndDemoEntity::GetNextMatrix () const
{
	return ndCalculateMatrix(m_nextRotation, m_nextPosition);
}

ndMatrix ndDemoEntity::CalculateGlobalMatrix (const ndDemoEntity* const root) const
{
	//ndMatrix matrix (ndGetIdentityMatrix());
	//for (const ndDemoEntity* ptr = this; ptr != root; ptr = ptr->GetParent()) 
	//{
	//	matrix = matrix * ptr->GetCurrentMatrix ();
	//}

	ndMatrix matrix(GetCurrentMatrix());
	if (this != root)
	{
		for (const ndDemoEntity* parent = GetParent(); parent != root; parent = parent->GetParent())
		{
			const ndMatrix parentMatrix(parent->GetCurrentMatrix());
			matrix = matrix * parentMatrix;
		}
	}
	return matrix;
}

//ndMatrix ndDemoEntity::CalculateInterpolatedGlobalMatrix (const ndDemoEntity* const root) const
ndMatrix ndDemoEntity::CalculateInterpolatedGlobalMatrix(const ndDemoEntity* const) const
{
	ndAssert(0);
	return ndMatrix();
#if 0
	ndMatrix matrix (ndGetIdentityMatrix());
	for (const ndDemoEntity* ptr = this; ptr != root; ptr = ptr->GetParent()) 
	{
		matrix = matrix * ptr->m_matrix;
	}
	return matrix;
#endif
}

void ndDemoEntity::SetMatrix(const ndQuaternion& rotation, const ndVector& position)
{
	ndScopeSpinLock lock(m_lock);
	m_curPosition = m_nextPosition;
	m_curRotation = m_nextRotation;

	m_nextPosition = position;
	m_nextRotation = rotation;
	ndAssert(position.m_w == ndFloat32(1.0f));

	ndFloat32 angle = m_curRotation.DotProduct(m_nextRotation).GetScalar();
	if (angle < 0.0f) 
	{
		m_curRotation = m_curRotation.Scale(ndFloat32(-1.0f));
	}
}

void ndDemoEntity::SetNextMatrix (const ndQuaternion& rotation, const ndVector& position)
{
	// read the data in a critical section to prevent race condition from other thread  
	m_nextPosition = position;
	m_nextRotation = rotation;
	ndAssert(position.m_w == ndFloat32(1.0f));

	ndFloat32 angle = m_curRotation.DotProduct(m_nextRotation).GetScalar();
	if (angle < 0.0f) 
	{
		m_curRotation = m_curRotation.Scale(ndFloat32 (-1.0f));
	}
}

void ndDemoEntity::InterpolateMatrix(ndFloat32 param)
{
	{
		ndScopeSpinLock lock(m_lock);
		ndVector p0(m_curPosition);
		ndVector p1(m_nextPosition);
		ndQuaternion r0(m_curRotation);
		ndQuaternion r1(m_nextRotation);

		ndVector posit(p0 + (p1 - p0).Scale(param));
		ndQuaternion rotation(r0.Slerp(r1, param));
		m_matrix = ndCalculateMatrix(rotation, posit);
	}

	//for (ndSharedPtr<ndDemoEntity> child(GetFirstChild()); *child; child = child->GetNext())
	for (ndList<ndSharedPtr<ndDemoEntity>>::ndNode* node = GetChildren().GetFirst(); node; node = node->GetNext())
	{
		node->GetInfo()->InterpolateMatrix(param);
	}
}

void ndDemoEntity::ResetMatrix(const ndMatrix& matrix)
{
	ndQuaternion rot(matrix);
	SetMatrix(rot, matrix.m_posit);
	SetMatrix(rot, matrix.m_posit);
	InterpolateMatrix(ndFloat32(0.0f));
}

const ndMatrix& ndDemoEntity::GetRenderMatrix () const
{
	return m_matrix;
}

//void ndDemoEntity::RenderBone(ndDemoEntityManager* const scene, const ndMatrix& nodeMatrix) const
void ndDemoEntity::RenderBone(ndDemoEntityManager* const, const ndMatrix&) const
{
#if !defined (__APPLE__)
	class ndSkelDebug : public ndConstraintDebugCallback
	{
		public:
		ndSkelDebug(ndDemoEntityManager* const scene)
		{
			ndDemoCamera* const camera = scene->GetCamera();
			const glMatrix viewProjectionMatrix(camera->GetInvViewProjectionMatrix());
			m_shader = scene->GetShaderCache().m_wireFrame;

			glUseProgram(m_shader);

			m_shadeColorLocation = glGetUniformLocation(m_shader, "shadeColor");
			m_projectionViewModelMatrixLocation = glGetUniformLocation(m_shader, "projectionViewModelMatrix");
			glUniformMatrix4fv(m_projectionViewModelMatrixLocation, 1, false, &viewProjectionMatrix[0][0]);

			glEnableClientState(GL_VERTEX_ARRAY);
			glVertexPointer(3, GL_FLOAT, sizeof(glVector3), m_line);
		}

		~ndSkelDebug()
		{
			glDisableClientState(GL_VERTEX_ARRAY);
			glUseProgram(0);
		}

		void DrawPoint(const ndVector& point, const ndVector& color, ndFloat32 thickness)
		{
			m_line[0].m_x = GLfloat(point.m_x);
			m_line[0].m_y = GLfloat(point.m_y);
			m_line[0].m_z = GLfloat(point.m_z);
			glVector4 c(color);

			glPointSize(GLfloat(thickness));
			glUniform4fv(m_shadeColorLocation, 1, &c[0]);
			glDrawArrays(GL_POINTS, 0, 1);
			glPointSize(1);
		}

		void DrawLine(const ndVector& p0, const ndVector& p1, const ndVector& color, ndFloat32 thickness)
		{
			m_line[0].m_x = GLfloat(p0.m_x);
			m_line[0].m_y = GLfloat(p0.m_y);
			m_line[0].m_z = GLfloat(p0.m_z);
			m_line[1].m_x = GLfloat(p1.m_x);
			m_line[1].m_y = GLfloat(p1.m_y);
			m_line[1].m_z = GLfloat(p1.m_z);
			glVector4 c(color);

			glLineWidth(GLfloat(thickness));
			glUniform4fv(m_shadeColorLocation, 1, &c[0]);
			glDrawArrays(GL_LINES, 0, 2);
			glLineWidth(1);
		}

		GLuint m_shader;
		ndInt32 m_shadeColorLocation;
		ndInt32 m_projectionViewModelMatrixLocation;

		glVector3 m_line[2];
	};
#endif
}

void ndDemoEntity::RenderSkeleton(ndDemoEntityManager* const scene, const ndMatrix& matrix) const
{
	ndMatrix nodeMatrix(m_matrix * matrix);
	RenderBone(scene, nodeMatrix);

	ndAssert(0);
#if 0
	for (ndDemoEntity* child = GetFirstChild(); child; child = child->GetNext())
	{
		child->RenderSkeleton(scene, nodeMatrix);
	}
#endif
}

//ndDemoEntity* ndDemoEntity::Find(const char* const name) const
//{
//	ndAssert(0);
//	ndString string(name);
//
//	ndFixSizeArray<const ndDemoEntity*, 1024> pool;
//	pool.PushBack(this);
//	while (pool.GetCount())
//	{
//		const ndDemoEntity* const entity = pool.Pop();
//		const ndString& entName = entity->GetName();
//		if (entName == string)
//		{
//			return (ndDemoEntity*) entity;
//		}
//
//		for (ndList<ndSharedPtr<ndDemoEntity>>::ndNode* node = entity->GetChildren().GetFirst(); node; node = node->GetNext())
//		{
//			pool.PushBack(*node->GetInfo());
//		}
//	}
//	return nullptr;
//}

ndSharedPtr<ndDemoEntity> ndDemoEntity::Find(const ndSharedPtr<ndDemoEntity>& self, const char* const name) const
{
	ndString string(name);
	ndAssert(this == *self);

	ndList<ndSharedPtr<ndDemoEntity>> stack;
	stack.Append(self);
	while (stack.GetCount())
	{
		ndSharedPtr<ndDemoEntity> entity = stack.GetLast()->GetInfo();
		stack.Remove(stack.GetLast());
		const ndString& entName = entity->GetName();
		if (entName == string)
		{
			return entity;
		}
	
		for (ndList<ndSharedPtr<ndDemoEntity>>::ndNode* node = entity->GetChildren().GetFirst(); node; node = node->GetNext())
		{
			stack.Append(node->GetInfo());
		}
	}
	return ndSharedPtr<ndDemoEntity>();
}

ndDemoEntity* ndDemoEntity::FindBySubString(const char* const subString) const
{
	//for (ndDemoEntity* child = GetFirstChild(); child; child = child->GetNext())
	//{
	//	ndString tmpName(child->GetName());
	//	tmpName.ToLower();
	//	const char* const name = tmpName.GetStr();
	//	if (strstr(name, subString))
	//	{
	//		return child;
	//	}
	//}
	
	ndInt32 stack = 1;
	const ndDemoEntity* pool[1024 * 4];
	pool[0] = this;
	while (stack)
	{
		stack--;
		const ndDemoEntity* const entity = pool[stack];
		ndString tmpName(entity->GetName());
		tmpName.ToLower();
		const char* const name = tmpName.GetStr();
		if (strstr(name, subString))
		{
			return (ndDemoEntity*)entity;
		}

		ndAssert(0);
#if 0
		for (ndDemoEntity* child = entity->GetFirstChild(); child; child = child->GetNext())
		{
			pool[stack] = child;
			stack++;
			ndAssert(stack < sizeof(pool) / sizeof(pool[0]));
		}
#endif
	}

	
	return nullptr;
}

ndShapeInstance* ndDemoEntity::CreateCompoundFromMesh(bool lowDetail)
{
	ndArray<ndVector> points;
	ndArray<ndInt32> indices;
	ndArray<ndInt32> remapIndex;
	const ndSharedPtr<ndDemoMeshInterface> meshPtr = GetMesh();
	ndDemoMesh* const mesh = (ndDemoMesh*)*meshPtr;
	ndAssert(mesh);
	mesh->GetVertexArray(points);
	mesh->GetIndexArray(indices);

	remapIndex.SetCount(points.GetCount());

	ndInt32 vCount = ndVertexListToIndexList(&points[0].m_x, sizeof(ndVector), 3, ndInt32(points.GetCount()), &remapIndex[0], ndFloat32(1.0e-6f));
	for (ndInt32 i = ndInt32 (indices.GetCount()) - 1; i >= 0; --i)
	{
		ndInt32 j = indices[i];
		indices[i] = remapIndex[j];
	}

	ndArray<ndReal> meshPoints;
	for (ndInt32 i = 0; i < vCount; ++i)
	{
		meshPoints.PushBack(ndReal(points[i].m_x));
		meshPoints.PushBack(ndReal(points[i].m_y));
		meshPoints.PushBack(ndReal(points[i].m_z));
	}
	
	nd_::VHACD::IVHACD* const interfaceVHACD = nd_::VHACD::CreateVHACD();
	nd_::VHACD::IVHACD::Parameters paramsVHACD;
	paramsVHACD.m_concavityToVolumeWeigh = lowDetail ? 1.0f : 0.5f;
	interfaceVHACD->Compute(&meshPoints[0], uint32_t(meshPoints.GetCount() / 3), (uint32_t*)&indices[0], uint32_t(indices.GetCount() / 3), paramsVHACD);

	ndShapeInstance* const compoundShapeInstance = new ndShapeInstance(new ndShapeCompound());

	ndShapeCompound* const compoundShape = compoundShapeInstance->GetShape()->GetAsShapeCompound();
	compoundShape->BeginAddRemove();
	ndInt32 hullCount = ndInt32(interfaceVHACD->GetNConvexHulls());
	ndArray<ndVector> convexMeshPoints;
	for (ndInt32 i = 0; i < hullCount; ++i)
	{
		nd_::VHACD::IVHACD::ConvexHull ch;
		interfaceVHACD->GetConvexHull(uint32_t(i), ch);
		convexMeshPoints.SetCount(ndInt32 (ch.m_nPoints));
		for (ndInt32 j = 0; j < ndInt32(ch.m_nPoints); ++j)
		{
			ndVector p(ndFloat32(ch.m_points[j * 3 + 0]), ndFloat32(ch.m_points[j * 3 + 1]), ndFloat32(ch.m_points[j * 3 + 2]), ndFloat32(0.0f));
			convexMeshPoints[j] = p;
		}
		ndShapeInstance hullShape(new ndShapeConvexHull(ndInt32(convexMeshPoints.GetCount()), sizeof(ndVector), 0.01f, &convexMeshPoints[0].m_x));
		compoundShape->AddCollision(&hullShape);
	}
	compoundShape->EndAddRemove();
	compoundShapeInstance->SetLocalMatrix(GetMeshMatrix());

	interfaceVHACD->Clean();
	interfaceVHACD->Release();
	return compoundShapeInstance;
}

ndShapeInstance* ndDemoEntity::CreateCollision() const
{
	ndString tmpName(GetName());
	tmpName.ToLower();
	const char* const name = tmpName.GetStr();

	ndShapeInstance* instance = nullptr;
	ndArray<ndVector> points;
	if (strstr(name, "sphere"))
	{
		ndDemoMesh* const mesh = (ndDemoMesh*)*GetMesh();
		ndAssert(mesh);
		mesh->GetVertexArray(points);

		ndVector minP(ndFloat32(1.0e10f));
		ndVector maxP(ndFloat32(-1.0e10f));
		for (ndInt32 i = 0; i < mesh->m_vertexCount; ++i)
		{
			minP = minP.GetMin(points[i]);
			maxP = maxP.GetMax(points[i]);
		}
		ndVector size(ndVector::m_half * (maxP - minP));
		ndMatrix alighMatrix(ndGetIdentityMatrix());
		alighMatrix.m_posit = ndVector::m_half * (maxP + minP);
		alighMatrix.m_posit.m_w = ndFloat32(1.0f);

		const ndMatrix matrix(alighMatrix * GetMeshMatrix());
		instance = new ndShapeInstance(new ndShapeSphere(size.m_x));
		instance->SetLocalMatrix(matrix);
	}
	else if (strstr(name, "box"))
	{
		ndDemoMesh* const mesh = (ndDemoMesh*)*GetMesh();
		ndAssert(mesh);
		mesh->GetVertexArray(points);
	
		ndVector minP(ndFloat32(1.0e10f));
		ndVector maxP(ndFloat32(-1.0e10f));
		for (ndInt32 i = 0; i < mesh->m_vertexCount; ++i)
		{
			minP = minP.GetMin(points[i]);
			maxP = maxP.GetMax(points[i]);
		}
		ndVector size(maxP - minP);
		const ndVector origin((maxP + minP).Scale(ndFloat32(0.5f)));
		ndMatrix alighMatrix(ndGetIdentityMatrix());
		alighMatrix.m_posit = ndVector::m_half * (maxP + minP);
		alighMatrix.m_posit.m_w = ndFloat32(1.0f);
	
		const ndMatrix matrix(alighMatrix * GetMeshMatrix());
		instance = new ndShapeInstance(new ndShapeBox(size.m_x, size.m_y, size.m_z));
		instance->SetLocalMatrix(matrix);
	}
	else if (strstr(name, "convexhull"))
	{
		ndDemoMesh* const mesh = (ndDemoMesh*)*GetMesh();
		ndAssert(mesh);
		mesh->GetVertexArray(points);

		const ndMatrix matrix(GetMeshMatrix());
		instance = new ndShapeInstance(new ndShapeConvexHull(mesh->m_vertexCount, sizeof(ndVector), 0.001f, &points[0].m_x));
		instance->SetLocalMatrix(matrix);
	}
	else if (strstr(name, "capsule"))
	{
		ndDemoMesh* const mesh = (ndDemoMesh*)*GetMesh();
		ndAssert(mesh);
		mesh->GetVertexArray(points);
		ndVector minP(ndFloat32(1.0e10f));
		ndVector maxP(ndFloat32(-1.0e10f));
		for (ndInt32 i = 0; i < mesh->m_vertexCount; ++i)
		{
			minP = minP.GetMin(points[i]);
			maxP = maxP.GetMax(points[i]);
		}
		ndVector size(ndVector::m_half * (maxP - minP));
		ndVector origin(ndVector::m_half * (maxP + minP));
		ndFloat32 high = 2.0f * ndMax(size.m_y - size.m_x, ndFloat32(0.05f));
		ndMatrix alighMatrix(ndRollMatrix(90.0f * ndDegreeToRad));
		alighMatrix.m_posit = origin;
		alighMatrix.m_posit.m_w = ndFloat32(1.0f);
		const ndMatrix matrix(alighMatrix * GetMeshMatrix());
	
		instance = new ndShapeInstance(new ndShapeCapsule(size.m_x, size.m_x, high));
		instance->SetLocalMatrix(matrix);
	}
	//else if (strstr(name, "vhacd"))
	//{
	//	ndArray<ndInt32> indices;
	//	ndDemoMesh* const mesh = (ndDemoMesh*)*node->GetInfo()->GetMesh();
	//	ndAssert(mesh);
	//	mesh->GetVertexArray(points);
	//	mesh->GetIndexArray(indices);
	//
	//	ndArray<ndTriplex> meshPoints;
	//	for (ndInt32 i = 0; i < points.GetCount(); ++i)
	//	{
	//		ndTriplex p;
	//		p.m_x = points[i].m_x;
	//		p.m_y = points[i].m_y;
	//		p.m_z = points[i].m_z;
	//		meshPoints.PushBack(p);
	//	}
	//	nd_::VHACD::IVHACD* const interfaceVHACD = nd_::VHACD::CreateVHACD();
	//
	//	nd_::VHACD::IVHACD::Parameters paramsVHACD;
	//	//paramsVHACD.m_concavityToVolumeWeigh = 1.0;
	//	paramsVHACD.m_concavityToVolumeWeigh = 0.5f;
	//	interfaceVHACD->Compute(&meshPoints[0].m_x, uint32_t(points.GetCount()),
	//		(uint32_t*)&indices[0], uint32_t(indices.GetCount()) / 3, paramsVHACD);
	//
	//	ndInt32 hullCount = ndInt32(interfaceVHACD->GetNConvexHulls());
	//	ndArray<ndVector> convexMeshPoints;
	//	for (ndInt32 i = 0; i < hullCount; ++i)
	//	{
	//		nd_::VHACD::IVHACD::ConvexHull ch;
	//		interfaceVHACD->GetConvexHull(uint32_t(i), ch);
	//		convexMeshPoints.SetCount(ndInt32(ch.m_nPoints));
	//		for (ndInt32 j = 0; j < ndInt32(ch.m_nPoints); ++j)
	//		{
	//			ndVector p(ndFloat32(ch.m_points[j * 3 + 0]), ndFloat32(ch.m_points[j * 3 + 1]), ndFloat32(ch.m_points[j * 3 + 2]), ndFloat32(0.0f));
	//			convexMeshPoints[j] = p;
	//		}
	//		shapeArray.PushBack(new ndShapeInstance(new ndShapeConvexHull(ndInt32(convexMeshPoints.GetCount()), sizeof(ndVector), 0.01f, &convexMeshPoints[0].m_x)));
	//		const ndMatrix matrix(node->GetInfo()->GetMeshMatrix() * node->GetInfo()->GetCurrentMatrix());
	//		shapeArray[shapeArray.GetCount() - 1]->SetLocalMatrix(matrix);
	//	}
	//
	//	interfaceVHACD->Clean();
	//	interfaceVHACD->Release();
	//}
	else
	{
		ndAssert(0);
	}

	return instance;
}

ndShapeInstance* ndDemoEntity::CreateCollisionFromChildren() const
{
	ndFixSizeArray<ndShapeInstance*, 128> shapeArray;

	ndArray<ndVector> points;
	
	shapeArray.PushBack(nullptr);
	for (ndList<ndSharedPtr<ndDemoEntity>>::ndNode* node = GetChildren().GetFirst(); node; node = node->GetNext())
	{
		ndString tmpName(node->GetInfo()->GetName());
		tmpName.ToLower();
		const char* const name = tmpName.GetStr();

		if (strstr (name, "sphere")) 
		{
			ndDemoMesh* const mesh = (ndDemoMesh*)*node->GetInfo()->GetMesh();
			ndAssert(mesh);
			mesh->GetVertexArray(points);

			ndVector minP(ndFloat32(1.0e10f));
			ndVector maxP(ndFloat32(-1.0e10f));
			for (ndInt32 i = 0; i < mesh->m_vertexCount; ++i)
			{
				minP = minP.GetMin(points[i]);
				maxP = maxP.GetMax(points[i]);
			}
			ndVector size(ndVector::m_half * (maxP - minP));
			ndMatrix alighMatrix(ndGetIdentityMatrix());
			alighMatrix.m_posit = ndVector::m_half * (maxP + minP);
			alighMatrix.m_posit.m_w = ndFloat32(1.0f);

			const ndMatrix matrix(alighMatrix * node->GetInfo()->GetMeshMatrix() * node->GetInfo()->GetCurrentMatrix());
			shapeArray.PushBack(new ndShapeInstance(new ndShapeSphere(size.m_x)));
			shapeArray[shapeArray.GetCount()-1]->SetLocalMatrix(matrix);
		} 
		else if (strstr (name, "box")) 
		{
			ndDemoMesh* const mesh = (ndDemoMesh*)*node->GetInfo()->GetMesh();
			ndAssert(mesh);
			mesh->GetVertexArray(points);

			ndVector minP(ndFloat32(1.0e10f));
			ndVector maxP(ndFloat32(-1.0e10f));
			for (ndInt32 i = 0; i < mesh->m_vertexCount; ++i)
			{
				minP = minP.GetMin(points[i]);
				maxP = maxP.GetMax(points[i]);
			}
			ndVector size(maxP - minP);
			shapeArray.PushBack(new ndShapeInstance(new ndShapeBox(size.m_x, size.m_y, size.m_z)));
		
			const ndVector origin((maxP + minP).Scale (ndFloat32 (0.5f)));
			ndMatrix alighMatrix(ndGetIdentityMatrix());
			alighMatrix.m_posit = ndVector::m_half * (maxP + minP);
			alighMatrix.m_posit.m_w = ndFloat32(1.0f);

			const ndMatrix matrix(alighMatrix * node->GetInfo()->GetMeshMatrix() * node->GetInfo()->GetCurrentMatrix());
			shapeArray[shapeArray.GetCount() - 1]->SetLocalMatrix(matrix);
		} 
		else if (strstr (name, "capsule")) 
		{
			ndDemoMesh* const mesh = (ndDemoMesh*)*node->GetInfo()->GetMesh();
			ndAssert(mesh);
			mesh->GetVertexArray(points);
			ndVector minP(ndFloat32(1.0e10f));
			ndVector maxP(ndFloat32(-1.0e10f));
			for (ndInt32 i = 0; i < mesh->m_vertexCount; ++i)
			{
				minP = minP.GetMin(points[i]);
				maxP = maxP.GetMax(points[i]);
			}
			ndVector size(ndVector::m_half * (maxP - minP));
			ndVector origin(ndVector::m_half * (maxP + minP));
			ndFloat32 high = 2.0f * ndMax (size.m_y - size.m_x, ndFloat32 (0.05f));
			ndMatrix alighMatrix(ndRollMatrix(90.0f * ndDegreeToRad));
			alighMatrix.m_posit = origin;
			alighMatrix.m_posit.m_w = ndFloat32(1.0f);

			const ndMatrix matrix(alighMatrix * node->GetInfo()->GetMeshMatrix() * node->GetInfo()->GetCurrentMatrix());

			shapeArray.PushBack(new ndShapeInstance(new ndShapeCapsule(size.m_x, size.m_x, high)));
			shapeArray[shapeArray.GetCount() - 1]->SetLocalMatrix(matrix);
		} 
		else if (strstr(name, "convexhull")) 
		{
			ndDemoMesh* const mesh = (ndDemoMesh*)*node->GetInfo()->GetMesh();
			ndAssert(mesh);
			mesh->GetVertexArray(points);
			shapeArray.PushBack(new ndShapeInstance(new ndShapeConvexHull(mesh->m_vertexCount, sizeof(ndVector), 0.01f, &points[0].m_x)));
			const ndMatrix matrix(node->GetInfo()->GetMeshMatrix() * node->GetInfo()->GetCurrentMatrix());
			shapeArray[shapeArray.GetCount() - 1]->SetLocalMatrix(matrix);
		}
		else if (strstr(name, "vhacd"))
		{
			ndArray<ndInt32> indices;
			ndDemoMesh* const mesh = (ndDemoMesh*)*node->GetInfo()->GetMesh();
			ndAssert(mesh);
			mesh->GetVertexArray(points);
			mesh->GetIndexArray(indices);

			ndArray<ndTriplex> meshPoints;
			for (ndInt32 i = 0; i < points.GetCount(); ++i)
			{
				ndTriplex p;
				p.m_x = points[i].m_x;
				p.m_y = points[i].m_y;
				p.m_z = points[i].m_z;
				meshPoints.PushBack(p);
			}
			nd_::VHACD::IVHACD* const interfaceVHACD = nd_::VHACD::CreateVHACD();
		
			nd_::VHACD::IVHACD::Parameters paramsVHACD;
			//paramsVHACD.m_concavityToVolumeWeigh = 1.0;
			paramsVHACD.m_concavityToVolumeWeigh = 0.5f;
			interfaceVHACD->Compute(&meshPoints[0].m_x, uint32_t(points.GetCount()),
				(uint32_t*)&indices[0], uint32_t(indices.GetCount()) / 3, paramsVHACD);
		
			ndInt32 hullCount = ndInt32(interfaceVHACD->GetNConvexHulls());
			ndArray<ndVector> convexMeshPoints;
			for (ndInt32 i = 0; i < hullCount; ++i)
			{
				nd_::VHACD::IVHACD::ConvexHull ch;
				interfaceVHACD->GetConvexHull(uint32_t(i), ch);
				convexMeshPoints.SetCount(ndInt32(ch.m_nPoints));
				for (ndInt32 j = 0; j < ndInt32(ch.m_nPoints); ++j)
				{
					ndVector p(ndFloat32(ch.m_points[j * 3 + 0]), ndFloat32(ch.m_points[j * 3 + 1]), ndFloat32(ch.m_points[j * 3 + 2]), ndFloat32(0.0f));
					convexMeshPoints[j] = p;
				}
				shapeArray.PushBack(new ndShapeInstance(new ndShapeConvexHull(ndInt32(convexMeshPoints.GetCount()), sizeof(ndVector), 0.01f, &convexMeshPoints[0].m_x)));
				const ndMatrix matrix(node->GetInfo()->GetMeshMatrix() * node->GetInfo()->GetCurrentMatrix());
				shapeArray[shapeArray.GetCount() - 1]->SetLocalMatrix(matrix);
			}
		
			interfaceVHACD->Clean();
			interfaceVHACD->Release();
		}
	}
	
	if (shapeArray.GetCount() > 2)
	{
		ndShapeInstance* const compoundInstance = new ndShapeInstance(new ndShapeCompound());
		ndShapeCompound* const compound = compoundInstance->GetShape()->GetAsShapeCompound();

		compound->BeginAddRemove ();
		for (ndInt32 i = 1; i < shapeArray.GetCount(); ++i)
		{
			compound->AddCollision(shapeArray[i]);
			delete shapeArray[i];
		}
		compound->EndAddRemove ();
		shapeArray[0] = compoundInstance;
	} 
	else if (shapeArray.GetCount() == 2)
	{
		shapeArray[0] = shapeArray[1];
	}
	return shapeArray[0];
}

void ndDemoEntity::Render(ndFloat32 timestep, ndDemoEntityManager* const scene, const ndMatrix& matrix) const
{
	ndMatrix nodeMatrix(m_matrix * matrix);
	ndDemoMeshInterface* const mesh = (ndDemoMeshInterface*)*m_mesh;
	if (m_isVisible && mesh)
	{
		// Render mesh if there is one 
		const ndMatrix modelMatrix(m_meshMatrix * nodeMatrix);
		mesh->Render(scene, modelMatrix);
	}

	//RenderBone(scene, nodeMatrix);
	for (ndList<ndSharedPtr<ndDemoEntity>>::ndNode* node = GetChildren().GetFirst(); node; node = node->GetNext())
	{
		node->GetInfo()->Render(timestep, scene, nodeMatrix);
	}
}

void ndDemoEntity::RenderShadowMap(ndShadowMapRenderPass* const shadowMap, const ndMatrix& matrix)
{
	ndMatrix nodeMatrix(m_matrix * matrix);

	ndDemoMeshInterface* const mesh = (ndDemoMeshInterface*)*m_mesh;
	if (m_isVisible && mesh && m_castShadow)
	{
		// Render mesh if there is one 
		const ndMatrix modelMatrix(m_meshMatrix * nodeMatrix);
		mesh->RenderShadowMap(shadowMap, modelMatrix);
	}

	for (ndList<ndSharedPtr<ndDemoEntity>>::ndNode* node = GetChildren().GetFirst(); node; node = node->GetNext())
	{
		node->GetInfo()->RenderShadowMap(shadowMap, nodeMatrix);
	}
}
