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
#include "ndDemoEntity.h"
#include "ndDemoCamera.h"
#include "ndAnimationPose.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndDemoEntityNotify)

ndDemoEntityNotify::ndDemoEntityNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity, ndBodyDynamic* const parentBody, ndFloat32 gravity)
	:ndBodyNotify(ndVector (ndFloat32 (0.0f), gravity, ndFloat32(0.0f), ndFloat32(0.0f)))
	,m_entity(entity)
	,m_parentBody(parentBody)
	,m_manager(manager)
{
}

// member a fill in a post process pass
ndDemoEntityNotify::ndDemoEntityNotify(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndBodyNotify(ndLoadSaveBase::ndLoadDescriptor(desc))
	,m_entity(nullptr)
	,m_parentBody(nullptr)
	,m_manager(nullptr)
{
}

ndDemoEntityNotify::~ndDemoEntityNotify()
{
	if (m_entity && m_entity->m_rootNode)
	{
		m_manager->RemoveEntity(m_entity);
		delete m_entity;
	}
}

void ndDemoEntityNotify::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	ndBodyNotify::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));
	xmlSaveParam(childNode, "comment", "string", "body notification for Newton 4.0 demos");
}

void ndDemoEntityNotify::OnObjectPick() const
{
	ndTrace(("picked body id: %d\n", GetBody()->GetId()));
}

void ndDemoEntityNotify::OnApplyExternalForce(ndInt32, ndFloat32)
{
	ndBodyKinematic* const body = GetBody()->GetAsBodyKinematic();
	ndAssert(body);
	if (body->GetInvMass() > 0.0f)
	{
		ndVector massMatrix(body->GetMassMatrix());
		ndVector force(GetGravity().Scale(massMatrix.m_w));
		body->SetForce(force);
		body->SetTorque(ndVector::m_zero);

		const ndFloat32 velocCap = 100.0f;
		// clamp execive velocites.
		ndVector omega(body->GetOmega());
		ndVector veloc(body->GetVelocity());
		ndFloat32 omega2(omega.DotProduct(omega).GetScalar());
		if (omega2 > velocCap * velocCap)
		{
			omega = omega.Normalize().Scale(velocCap);
			body->SetOmega(omega);
		}

		ndFloat32 veloc2(veloc.DotProduct(veloc).GetScalar());
		if (veloc2 > velocCap * velocCap)
		{
			veloc = veloc.Normalize().Scale(velocCap);
			body->SetVelocity(veloc);
		}
	}
}

void ndDemoEntityNotify::OnTransform(ndInt32, const ndMatrix& matrix)
{
	// apply this transformation matrix to the application user data.
	if (m_entity)
	{
		const ndBody* const body = GetBody();
		if (!m_parentBody)
		{
			const ndQuaternion rot(body->GetRotation());
			m_entity->SetMatrix(rot, matrix.m_posit);
		}
		else
		{
			const ndMatrix parentMatrix(m_parentBody->GetMatrix());
			const ndMatrix localMatrix(matrix * parentMatrix.Inverse());
			const ndQuaternion rot(localMatrix);
			m_entity->SetMatrix(rot, localMatrix.m_posit);
		}
	}
}

ndDemoEntity::ndDemoEntity(const ndMatrix& matrix, ndDemoEntity* const parent)
	:ndNodeHierarchy<ndDemoEntity>()
	,m_matrix(matrix) 
	,m_curPosition (matrix.m_posit)
	,m_nextPosition (matrix.m_posit)
	,m_curRotation (matrix)
	,m_nextRotation (matrix)
	,m_meshMatrix(ndGetIdentityMatrix())
	,m_mesh(nullptr)
	,m_userData(nullptr)
	,m_rootNode(nullptr)
	,m_lock()
	,m_isVisible(true)
{
	if (parent) 
	{
		Attach(parent);
	}
}

/*
ndDemoEntity::ndDemoEntity(ndDemoEntityManager& world, const dScene* const scene, dScene::dNode* const rootSceneNode, dTree<ndDemoMeshInterface*, dScene::dNode*>& meshCache, ndDemoEntityManager::EntityDictionary& entityDictionary, ndDemoEntity* const parent)
	:dClassInfo()
	,dHierarchy<ndDemoEntity>()
	,m_matrix(ndGetIdentityMatrix())
	,m_curPosition(0.0f, 0.0f, 0.0f, 1.0f)
	,m_nextPosition(0.0f, 0.0f, 0.0f, 1.0f)
	,m_curRotation(0.0f, 0.0f, 0.0f, 1.0f)
	,m_nextRotation(0.0f, 0.0f, 0.0f, 1.0f)
	,m_meshMatrix(ndGetIdentityMatrix())
	,m_mesh(nullptr)
	,m_userData(nullptr)
	,m_rootNode(nullptr)
	,m_lock()
	,m_isVisible(true)
{
	// add this entity to the dictionary
	entityDictionary.Insert(this, rootSceneNode);

	// if this is a child mesh set it as child of the entity
//	ndMatrix parentMatrix (GetIdentityMatrix());
	if (parent) {
		Attach (parent);
		ndAssert (scene->FindParentByType(rootSceneNode, dSceneNodeInfo::GetRttiType()));
//		dScene::dNode* const parentNode = scene->FindParentByType(rootSceneNode, dSceneNodeInfo::GetRttiType());
//		dSceneNodeInfo* const parentInfo = (dSceneNodeInfo*)scene->GetInfoFromNode (parentNode);
//		ndAssert (parentInfo->IsType(dSceneNodeInfo::GetRttiType()));
//		parentMatrix = parentInfo->GetTransform();
	}

	dSceneNodeInfo* const sceneInfo = (dSceneNodeInfo*) scene->GetInfoFromNode (rootSceneNode);
	//ndMatrix matrix (sceneInfo->GetTransform() * parentMatrix.Inverse4x4());
	ndMatrix matrix (sceneInfo->GetTransform());
	ResetMatrix (world, matrix);
	SetNameID(sceneInfo->GetName());

	// if this node has a mesh, find it and attach it to this entity
	dScene::dNode* const meshNode = scene->FindChildByType(rootSceneNode, dMeshNodeInfo::GetRttiType());
	if (meshNode) {
		ndDemoMeshInterface* const mesh = meshCache.Find(meshNode)->GetInfo();
		SetMesh(mesh, sceneInfo->GetGeometryTransform());
	}

	// we now scan for all dSceneNodeInfo node with direct connection to this rootSceneNode, 
	// and we load the as children of this entity
	for (void* child = scene->GetFirstChildLink(rootSceneNode); child; child = scene->GetNextChildLink (rootSceneNode, child)) {
		dScene::dNode* const node = scene->GetNodeFromLink(child);
		dNodeInfo* const info = scene->GetInfoFromNode(node);
		if (info->IsType(dSceneNodeInfo::GetRttiType())) {
			new ndDemoEntity (world, scene, node, meshCache, entityDictionary, this);
		}
	}
}
*/

ndDemoEntity::ndDemoEntity(const ndDemoEntity& copyFrom)
	:ndNodeHierarchy<ndDemoEntity>(copyFrom)
	,m_matrix(copyFrom.m_matrix)
	,m_curPosition(copyFrom.m_curPosition)
	,m_nextPosition(copyFrom.m_nextPosition)
	,m_curRotation(copyFrom.m_curRotation)
	,m_nextRotation(copyFrom.m_nextRotation)
	,m_meshMatrix(copyFrom.m_meshMatrix)
	,m_mesh(nullptr)
	,m_userData(nullptr)
	,m_rootNode(nullptr)
	,m_lock()
	,m_isVisible(copyFrom.m_isVisible)
{
	m_mesh = copyFrom.m_mesh ? copyFrom.m_mesh->Clone(this) : nullptr;
}

ndDemoEntity::~ndDemoEntity(void)
{
	if (m_userData) 
	{
		delete m_userData;
	}
	SetMesh(nullptr, ndGetIdentityMatrix());
}

ndNodeBaseHierarchy* ndDemoEntity::CreateClone () const
{
	return new ndDemoEntity(*this);
}

ndDemoEntity::UserData* ndDemoEntity::GetUserData ()
{
	return m_userData;
}

void ndDemoEntity::SetUserData (UserData* const data)
{
	m_userData = data;
}

/*
void ndDemoEntity::TransformCallback(const NewtonBody* body, const ndFloat32* matrix, ndInt32 threadIndex)
{
	ndDemoEntity* const ent = (ndDemoEntity*) NewtonBodyGetUserData(body);
	if (ent) {
		ndDemoEntityManager* const scene = (ndDemoEntityManager*)NewtonWorldGetUserData(NewtonBodyGetWorld(body));
		ndMatrix transform(matrix);
		ndQuaternion rot;
		NewtonBodyGetRotation(body, &rot.m_x);

		scene->Lock(ent->m_lock);
		ent->SetMatrixUsafe(rot, transform.m_posit);
		scene->Unlock(ent->m_lock);
	}
}
*/

ndDemoMeshInterface* ndDemoEntity::GetMesh() const
{
	return m_mesh;
}

void ndDemoEntity::SetMesh(ndDemoMeshInterface* const mesh, const ndMatrix& meshMatrix)
{
	m_meshMatrix = meshMatrix;
	if (m_mesh) 
	{
		m_mesh->Release();
	}
	m_mesh = nullptr;
	if (mesh) 
	{
		m_mesh = mesh->AddRef();
	}
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
	return ndMatrix (m_curRotation, m_curPosition);
}

ndAnimKeyframe ndDemoEntity::GetCurrentTransform() const
{
	return ndAnimKeyframe(m_curPosition, m_curRotation);
}

ndMatrix ndDemoEntity::GetNextMatrix () const
{
	return ndMatrix (m_nextRotation, m_nextPosition);
}

ndMatrix ndDemoEntity::CalculateGlobalMatrix (const ndDemoEntity* const root) const
{
	ndMatrix matrix (ndGetIdentityMatrix());
	for (const ndDemoEntity* ptr = this; ptr != root; ptr = ptr->GetParent()) 
	{
		matrix = matrix * ptr->GetCurrentMatrix ();
	}
	return matrix;
}

ndMatrix ndDemoEntity::CalculateInterpolatedGlobalMatrix (const ndDemoEntity* const root) const
{
	ndMatrix matrix (ndGetIdentityMatrix());
	for (const ndDemoEntity* ptr = this; ptr != root; ptr = ptr->GetParent()) 
	{
		matrix = matrix * ptr->m_matrix;
	}
	return matrix;
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
		m_matrix = ndMatrix(rotation, posit);
	}

	for (ndDemoEntity* child = GetChild(); child; child = child->GetSibling()) 
	{
		child->InterpolateMatrix(param);
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

void ndDemoEntity::RenderBone(ndDemoEntityManager* const scene, const ndMatrix& nodeMatrix) const
{
	class ndSkelDebug : public ndConstraintDebugCallback
	{
		public:
		ndSkelDebug(ndDemoEntityManager* const scene)
		{
			ndDemoCamera* const camera = scene->GetCamera();
			const glMatrix viewProjectionMatrix(camera->GetViewMatrix() * camera->GetProjectionMatrix());
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

	ndSkelDebug debug(scene);
	ndDemoEntity* const parent = GetParent();
	if (parent)
	{
		//glDisable(GL_TEXTURE_2D);
		ndMatrix parentMatrix(m_matrix.Inverse() * nodeMatrix);
		ndVector p0(nodeMatrix.m_posit);
		ndVector p1(parentMatrix.m_posit);
		ndVector color(0.0f, 0.0f, 0.0f, 1.0f);
		debug.DrawLine(p0, p1, color, 1.0);
		debug.SetScale(0.125f);
		debug.DrawFrame(nodeMatrix);
	}
}

void ndDemoEntity::RenderSkeleton(ndDemoEntityManager* const scene, const ndMatrix& matrix) const
{
	ndMatrix nodeMatrix(m_matrix * matrix);
	RenderBone(scene, nodeMatrix);
	for (ndDemoEntity* child = GetChild(); child; child = child->GetSibling())
	{
		child->RenderSkeleton(scene, nodeMatrix);
	}
}

void ndDemoEntity::Render(ndFloat32 timestep, ndDemoEntityManager* const scene, const ndMatrix& matrix) const
{
	ndMatrix nodeMatrix (m_matrix * matrix);
	if (m_isVisible && m_mesh) 
	{
		// Render mesh if there is one 
		ndMatrix modelMatrix (m_meshMatrix * nodeMatrix);
		m_mesh->Render(scene, modelMatrix);
	}

	//RenderBone(scene, nodeMatrix);
	for (ndDemoEntity* child = GetChild(); child; child = child->GetSibling()) 
	{
		child->Render(timestep, scene, nodeMatrix);
	}
}

ndShapeInstance* ndDemoEntity::CreateCompoundFromMesh(bool lowDetail) const
{
	ndArray<ndVector> points;
	ndArray<ndInt32> indices;
	ndDemoMesh* const mesh = (ndDemoMesh*)GetMesh();
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

#ifdef _D_USE_NEW_HACD
	lowDetail = 0;
	VHACD::IVHACD* const interfaceVHACD = ndCreateVHACD();
	VHACD::IVHACD::Parameters paramsVHACD;
	interfaceVHACD->Compute(&meshPoints[0].m_x, points.GetCount(), (uint32_t*)&indices[0], indices.GetCount() / 3, paramsVHACD);
#else
	nd_::VHACD::IVHACD* const interfaceVHACD = nd_::VHACD::CreateVHACD();
	nd_::VHACD::IVHACD::Parameters paramsVHACD;
	//paramsVHACD.m_maxConvexHulls = 24;
	paramsVHACD.m_concavityToVolumeWeigh = lowDetail ? 1.0f : 0.5f;
	interfaceVHACD->Compute(&meshPoints[0].m_x, points.GetCount(), (uint32_t*)&indices[0], indices.GetCount() / 3, paramsVHACD);
#endif

	ndShapeInstance* const compoundShapeInstance = new ndShapeInstance(new ndShapeCompound());

	ndShapeCompound* const compoundShape = compoundShapeInstance->GetShape()->GetAsShapeCompound();
	compoundShape->BeginAddRemove();
	ndInt32 hullCount = interfaceVHACD->GetNConvexHulls();
	ndArray<ndVector> convexMeshPoints;
	for (ndInt32 i = 0; i < hullCount; ++i)
	{
#ifdef _D_USE_NEW_HACD
		VHACD::IVHACD::ConvexHull ch;
#else
		nd_::VHACD::IVHACD::ConvexHull ch;
#endif
		interfaceVHACD->GetConvexHull(i, ch);
		convexMeshPoints.SetCount(ch.m_nPoints);
		for (ndInt32 j = 0; j < ndInt32(ch.m_nPoints); ++j)
		{
			ndVector p(ndFloat32(ch.m_points[j * 3 + 0]), ndFloat32(ch.m_points[j * 3 + 1]), ndFloat32(ch.m_points[j * 3 + 2]), ndFloat32(0.0f));
			convexMeshPoints[j] = p;
		}
		ndShapeInstance hullShape(new ndShapeConvexHull(convexMeshPoints.GetCount(), sizeof(ndVector), 0.01f, &convexMeshPoints[0].m_x));
		compoundShape->AddCollision(&hullShape);
	}
	compoundShape->EndAddRemove();
	compoundShapeInstance->SetLocalMatrix(GetMeshMatrix());

	interfaceVHACD->Clean();
	interfaceVHACD->Release();
	return compoundShapeInstance;
}

ndShapeInstance* ndDemoEntity::CreateCollisionFromChildren() const
{
	ndFixSizeArray<ndShapeInstance*, 128> shapeArray;

	ndArray<ndVector> points;
	
	shapeArray.PushBack(nullptr);
	for (ndDemoEntity* child = GetChild(); child; child = child->GetSibling()) 
	{
		ndString tmpName(child->GetName());
		tmpName.ToLower();
		const char* const name = tmpName.GetStr();
	
		if (strstr (name, "sphere")) 
		{
			ndDemoMesh* const mesh = (ndDemoMesh*)child->GetMesh();
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
			const ndMatrix matrix(child->GetMeshMatrix() * alighMatrix * child->GetCurrentMatrix());
			shapeArray.PushBack(new ndShapeInstance(new ndShapeSphere(size.m_x)));
			shapeArray[shapeArray.GetCount()-1]->SetLocalMatrix(matrix);
		} 
		else if (strstr (name, "box")) 
		{
			ndDemoMesh* const mesh = (ndDemoMesh*)child->GetMesh();
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

			ndVector origin((maxP + minP).Scale (ndFloat32 (0.5f)));

			ndMatrix matrix(child->GetMeshMatrix());
			matrix.m_posit += origin;
			matrix = matrix * child->GetCurrentMatrix();
			shapeArray[shapeArray.GetCount() - 1]->SetLocalMatrix(matrix);
		} 
		else if (strstr (name, "capsule")) 
		{
			ndDemoMesh* const mesh = (ndDemoMesh*)child->GetMesh();
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
			const ndMatrix matrix (alighMatrix * child->GetMeshMatrix() * child->GetCurrentMatrix());

			shapeArray.PushBack(new ndShapeInstance(new ndShapeCapsule(size.m_x, size.m_x, high)));
			shapeArray[shapeArray.GetCount() - 1]->SetLocalMatrix(matrix);
		} 
		else if (strstr(name, "convexhull")) 
		{
			ndDemoMesh* const mesh = (ndDemoMesh*)child->GetMesh();
			mesh->GetVertexArray(points);
			shapeArray.PushBack(new ndShapeInstance(new ndShapeConvexHull(mesh->m_vertexCount, sizeof(ndVector), 0.01f, &points[0].m_x)));
			const ndMatrix matrix(child->GetMeshMatrix() * child->GetCurrentMatrix());
			shapeArray[shapeArray.GetCount() - 1]->SetLocalMatrix(matrix);
		}
		else if (strstr(name, "vhacd"))
		{
			ndArray<ndInt32> indices;
			ndDemoMesh* const mesh = (ndDemoMesh*)child->GetMesh();
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
			interfaceVHACD->Compute(&meshPoints[0].m_x, points.GetCount(),
				(uint32_t*)&indices[0], indices.GetCount() / 3, paramsVHACD);

			ndInt32 hullCount = interfaceVHACD->GetNConvexHulls();
			ndArray<ndVector> convexMeshPoints;
			for (ndInt32 i = 0; i < hullCount; ++i)
			{
				nd_::VHACD::IVHACD::ConvexHull ch;
				interfaceVHACD->GetConvexHull(i, ch);
				convexMeshPoints.SetCount(ch.m_nPoints);
				for (ndInt32 j = 0; j < ndInt32(ch.m_nPoints); ++j)
				{
					ndVector p(ndFloat32(ch.m_points[j * 3 + 0]), ndFloat32(ch.m_points[j * 3 + 1]), ndFloat32(ch.m_points[j * 3 + 2]), ndFloat32(0.0f));
					convexMeshPoints[j] = p;
				}
				shapeArray.PushBack(new ndShapeInstance(new ndShapeConvexHull(convexMeshPoints.GetCount(), sizeof(ndVector), 0.01f, &convexMeshPoints[0].m_x)));
				const ndMatrix matrix(child->GetMeshMatrix() * child->GetCurrentMatrix());
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

