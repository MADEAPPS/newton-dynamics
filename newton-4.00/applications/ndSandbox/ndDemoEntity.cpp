/* Copyright (c) <2003-2021> <Newton Game Dynamics>
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
ndDemoEntityNotify::ndDemoEntityNotify(const ndLoadSaveBase::dLoadDescriptor& desc)
	:ndBodyNotify(ndLoadSaveBase::dLoadDescriptor(desc))
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
	dTrace(("picked body id: %d\n", GetBody()->GetId()));
}

void ndDemoEntityNotify::OnApplyExternalForce(ndInt32, ndFloat32)
{
	ndBodyKinematic* const body = GetBody()->GetAsBodyKinematic();
	dAssert(body);
	if (body->GetInvMass() > 0.0f)
	{
		ndVector massMatrix(body->GetMassMatrix());
		ndVector force(GetGravity().Scale(massMatrix.m_w));
		body->SetForce(force);
		body->SetTorque(ndVector::m_zero);

		//ndVector L(body->CalculateAngularMomentum());
		//dTrace(("%f %f %f\n", L.m_x, L.m_y, L.m_z));
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
	,m_meshMatrix(dGetIdentityMatrix())
	,m_mesh(nullptr)
	,m_userData(nullptr)
	,m_rootNode(nullptr)
	,m_isVisible(true)
{
	if (parent) 
	{
		Attach (parent);
	}
}

/*
ndDemoEntity::ndDemoEntity(ndDemoEntityManager& world, const dScene* const scene, dScene::dNode* const rootSceneNode, dTree<ndDemoMeshInterface*, dScene::dNode*>& meshCache, ndDemoEntityManager::EntityDictionary& entityDictionary, ndDemoEntity* const parent)
	:dClassInfo()
	,dHierarchy<ndDemoEntity>()
	,m_matrix(dGetIdentityMatrix())
	,m_curPosition(0.0f, 0.0f, 0.0f, 1.0f)
	,m_nextPosition(0.0f, 0.0f, 0.0f, 1.0f)
	,m_curRotation(0.0f, 0.0f, 0.0f, 1.0f)
	,m_nextRotation(0.0f, 0.0f, 0.0f, 1.0f)
	,m_meshMatrix(dGetIdentityMatrix())
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
		dAssert (scene->FindParentByType(rootSceneNode, dSceneNodeInfo::GetRttiType()));
//		dScene::dNode* const parentNode = scene->FindParentByType(rootSceneNode, dSceneNodeInfo::GetRttiType());
//		dSceneNodeInfo* const parentInfo = (dSceneNodeInfo*)scene->GetInfoFromNode (parentNode);
//		dAssert (parentInfo->IsType(dSceneNodeInfo::GetRttiType()));
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
	SetMesh(nullptr, dGetIdentityMatrix());
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
	ndMatrix matrix (dGetIdentityMatrix());
	for (const ndDemoEntity* ptr = this; ptr != root; ptr = ptr->GetParent()) 
	{
		matrix = matrix * ptr->GetCurrentMatrix ();
	}
	return matrix;
}

ndMatrix ndDemoEntity::CalculateInterpolatedGlobalMatrix (const ndDemoEntity* const root) const
{
	ndMatrix matrix (dGetIdentityMatrix());
	for (const ndDemoEntity* ptr = this; ptr != root; ptr = ptr->GetParent()) 
	{
		matrix = matrix * ptr->m_matrix;
	}
	return matrix;
}

void ndDemoEntity::SetMatrix(const ndQuaternion& rotation, const ndVector& position)
{
	m_curPosition = m_nextPosition;
	m_curRotation = m_nextRotation;

	m_nextPosition = position;
	m_nextRotation = rotation;

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

	ndFloat32 angle = m_curRotation.DotProduct(m_nextRotation).GetScalar();
	if (angle < 0.0f) 
	{
		m_curRotation = m_curRotation.Scale(ndFloat32 (-1.0f));
	}
}

void ndDemoEntity::ResetMatrix(const ndMatrix& matrix)
{
	ndQuaternion rot (matrix);
	SetMatrix(rot, matrix.m_posit);
	SetMatrix(rot, matrix.m_posit);
	InterpolateMatrix (ndFloat32 (0.0f));
}

void ndDemoEntity::InterpolateMatrix(ndFloat32 param)
{
	ndVector p0(m_curPosition);
	ndVector p1(m_nextPosition);
	ndQuaternion r0(m_curRotation);
	ndQuaternion r1(m_nextRotation);

	ndVector posit(p0 + (p1 - p0).Scale(param));
	ndQuaternion rotation(r0.Slerp(r1, param));
	m_matrix = ndMatrix(rotation, posit);

	for (ndDemoEntity* child = GetChild(); child; child = child->GetSibling()) 
	{
		child->InterpolateMatrix(param);
	}
}

const ndMatrix& ndDemoEntity::GetRenderMatrix () const
{
	return m_matrix;
}

void ndDemoEntity::RenderBone() const
{
	if (GetParent()) 
	{
		dAssert(0);
		//glDisable(GL_TEXTURE_2D);
		//
		//glColor3f(0.5f, 0.5f, 0.5f);
		//glBegin(GL_LINES);
		//for (ndDemoEntity* child = GetChild(); child; child = child->GetSibling()) 
		//{
		//	const dVector& posit = child->m_matrix.m_posit;
		//	glVertex3f(GLfloat(0.0f), GLfloat(0.0f), GLfloat(0.0f));
		//	glVertex3f(GLfloat(posit.m_x), GLfloat(posit.m_y), GLfloat(posit.m_z));
		//}
		//
		//glEnd();
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

//	RenderBone();
	for (ndDemoEntity* child = GetChild(); child; child = child->GetSibling()) 
	{
		child->Render(timestep, scene, nodeMatrix);
	}
}

ndShapeInstance* ndDemoEntity::CreateCollisionFromchildren() const
{
	ndInt32 count = 1;
	ndShapeInstance* shapeArray[128];
	
	shapeArray[0] = nullptr;
	for (ndDemoEntity* child = GetChild(); child; child = child->GetSibling()) 
	{
		ndString tmpName(child->GetName());
		tmpName.ToLower();
		const char* const name = tmpName.GetStr();
	
		if (strstr (name, "sphere")) 
		{
			ndArray<ndVector> points;
			ndDemoMesh* const mesh = (ndDemoMesh*)child->GetMesh();
			mesh->GetVertexArray(points);

			ndVector minP(ndFloat32(1.0e10f));
			ndVector maxP(ndFloat32(-1.0e10f));
			for (ndInt32 i = 0; i < mesh->m_vertexCount; i++)
			{
				minP = minP.GetMin(points[i]);
				maxP = maxP.GetMax(points[i]);
			}
			ndVector size(ndVector::m_half * (maxP - minP));
			ndMatrix alighMatrix(dGetIdentityMatrix());
			alighMatrix.m_posit = ndVector::m_half * (maxP + minP);
			alighMatrix.m_posit.m_w = ndFloat32(1.0f);
			const ndMatrix matrix(child->GetMeshMatrix() * alighMatrix * child->GetCurrentMatrix());
			shapeArray[count] = new ndShapeInstance(new ndShapeSphere(size.m_x));
			shapeArray[count]->SetLocalMatrix(matrix);
			count++;
			dAssert(count < ndInt32 (sizeof(shapeArray) / sizeof(shapeArray[0])));
		} 
		else if (strstr (name, "box")) 
		{
			ndArray<ndVector> points;
			ndDemoMesh* const mesh = (ndDemoMesh*)child->GetMesh();
			mesh->GetVertexArray(points);
			
			ndVector minP(ndFloat32(1.0e10f));
			ndVector maxP(ndFloat32(-1.0e10f));
			for (ndInt32 i = 0; i < mesh->m_vertexCount; i++)
			{
				minP = minP.GetMin(points[i]);
				maxP = maxP.GetMax(points[i]);
			}
			ndVector size(maxP - minP);
			shapeArray[count] = new ndShapeInstance(new ndShapeBox(size.m_x, size.m_y, size.m_z));

			ndVector origin((maxP + minP).Scale (ndFloat32 (0.5f)));

			ndMatrix matrix(child->GetMeshMatrix());
			matrix.m_posit += origin;
			matrix = matrix * child->GetCurrentMatrix();
			
			shapeArray[count]->SetLocalMatrix(matrix);
			count++;
			dAssert(count < ndInt32(sizeof(shapeArray) / sizeof(shapeArray[0])));
		} 
		else if (strstr (name, "capsule")) 
		{
			ndArray<ndVector> points;
			ndDemoMesh* const mesh = (ndDemoMesh*)child->GetMesh();
			mesh->GetVertexArray(points);

			ndVector minP(ndFloat32(1.0e10f));
			ndVector maxP(ndFloat32(-1.0e10f));
			for (ndInt32 i = 0; i < mesh->m_vertexCount; i++)
			{
				minP = minP.GetMin(points[i]);
				maxP = maxP.GetMax(points[i]);
			}
			ndVector size(ndVector::m_half * (maxP - minP));
			ndVector origin(ndVector::m_half * (maxP + minP));
			ndFloat32 high = 2.0f * dMax (size.m_y - size.m_x, ndFloat32 (0.05f));
			ndMatrix alighMatrix(dRollMatrix(90.0f * ndDegreeToRad));
			alighMatrix.m_posit = origin;
			alighMatrix.m_posit.m_w = ndFloat32(1.0f);
			const ndMatrix matrix (alighMatrix * child->GetMeshMatrix() * child->GetCurrentMatrix());

			shapeArray[count] = new ndShapeInstance(new ndShapeCapsule(size.m_x, size.m_x, high));
			shapeArray[count]->SetLocalMatrix(matrix);
			count++;
			dAssert(count < ndInt32 (sizeof(shapeArray)/ sizeof (shapeArray[0])));
		} 
		else if (strstr(name, "convexhull")) 
		{
			ndArray<ndVector> points;
			ndDemoMesh* const mesh = (ndDemoMesh*)child->GetMesh();
			mesh->GetVertexArray(points);
			shapeArray[count] = new ndShapeInstance(new ndShapeConvexHull(mesh->m_vertexCount, sizeof(ndVector), 0.01f, &points[0].m_x));
			const ndMatrix matrix(child->GetMeshMatrix() * child->GetCurrentMatrix());
			shapeArray[count]->SetLocalMatrix(matrix);
			count++;
			dAssert(count < ndInt32 (sizeof(shapeArray) / sizeof(shapeArray[0])));
		}
	}
	
	if (count > 2) 
	{
		ndShapeInstance* const compoundInstance = new ndShapeInstance(new ndShapeCompound());
		ndShapeCompound* const compound = compoundInstance->GetShape()->GetAsShapeCompound();
		//compound->SetOwner(compoundInstance);

		compound->BeginAddRemove ();
		for (ndInt32 i = 1; i < count; i ++) 
		{
			compound->AddCollision(shapeArray[i]);
			delete shapeArray[i];
		}
		compound->EndAddRemove ();
		shapeArray[0] = compoundInstance;
		count = 1;
	} 
	else if (count == 2) 
	{
		shapeArray[0] = shapeArray[1];
	}
	return shapeArray[0];
}
