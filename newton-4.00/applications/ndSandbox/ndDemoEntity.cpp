/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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

ndDemoEntityNotify::ndDemoEntityNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity, dFloat32 gravity)
	:ndBodyNotify()
	,m_entity(entity)
	,m_manager(manager)
	,m_gravity(gravity)
{
}

ndDemoEntityNotify::~ndDemoEntityNotify()
{
	dAssert(m_entity->m_rootNode);
	m_manager->RemoveEntity(m_entity);
	delete m_entity;
}

void ndDemoEntityNotify::OnApplyExternalForce(dInt32 threadIndex, dFloat32 timestep)
{
	ndBodyKinematic* const body = GetBody()->GetAsBodyKinematic();
	dAssert(body);
	if (body->GetInvMass() > 0.0f)
	{
		dVector massMatrix(body->GetMassMatrix());
		dVector force(0.0f, m_gravity * massMatrix.m_w, 0.0f, 0.0f);
		body->SetForce(force);
		body->SetTorque(dVector::m_zero);

		//dVector L(body->CalculateAngularMomentum());
		//dTrace(("%f %f %f\n", L.m_x, L.m_y, L.m_z));
	}
}

void ndDemoEntityNotify::OnTranform(dInt32 threadIndex, const dMatrix& matrix)
{
	// apply this transformation matrix to the application user data.
	ndBody* const body = GetBody();
	dMatrix transform(matrix);
	dQuaternion rot(body->GetRotation());

	dScopeSpinLock lock(m_entity->m_lock);
	m_entity->SetMatrixUsafe(rot, transform.m_posit);
}

ndDemoEntity::ndDemoEntity(const dMatrix& matrix, ndDemoEntity* const parent)
	:dNodeHierarchy<ndDemoEntity>()
	,m_matrix(matrix) 
	,m_curPosition (matrix.m_posit)
	,m_nextPosition (matrix.m_posit)
	,m_curRotation (dQuaternion (matrix))
	,m_nextRotation (dQuaternion (matrix))
	,m_meshMatrix(dGetIdentityMatrix())
	,m_mesh(nullptr)
	,m_userData(nullptr)
	,m_rootNode(nullptr)
	,m_lock() 
	,m_isVisible(true)
{
	if (parent) 
	{
		Attach (parent);
	}
}

/*
ndDemoEntity::ndDemoEntity(ndDemoEntityManager& world, const dScene* const scene, dScene::dTreeNode* const rootSceneNode, dTree<ndDemoMeshInterface*, dScene::dTreeNode*>& meshCache, ndDemoEntityManager::EntityDictionary& entityDictionary, ndDemoEntity* const parent)
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
//	dMatrix parentMatrix (GetIdentityMatrix());
	if (parent) {
		Attach (parent);
		dAssert (scene->FindParentByType(rootSceneNode, dSceneNodeInfo::GetRttiType()));
//		dScene::dTreeNode* const parentNode = scene->FindParentByType(rootSceneNode, dSceneNodeInfo::GetRttiType());
//		dSceneNodeInfo* const parentInfo = (dSceneNodeInfo*)scene->GetInfoFromNode (parentNode);
//		dAssert (parentInfo->IsType(dSceneNodeInfo::GetRttiType()));
//		parentMatrix = parentInfo->GetTransform();
	}

	dSceneNodeInfo* const sceneInfo = (dSceneNodeInfo*) scene->GetInfoFromNode (rootSceneNode);
	//dMatrix matrix (sceneInfo->GetTransform() * parentMatrix.Inverse4x4());
	dMatrix matrix (sceneInfo->GetTransform());
	ResetMatrix (world, matrix);
	SetNameID(sceneInfo->GetName());

	// if this node has a mesh, find it and attach it to this entity
	dScene::dTreeNode* const meshNode = scene->FindChildByType(rootSceneNode, dMeshNodeInfo::GetRttiType());
	if (meshNode) {
		ndDemoMeshInterface* const mesh = meshCache.Find(meshNode)->GetInfo();
		SetMesh(mesh, sceneInfo->GetGeometryTransform());
	}

	// we now scan for all dSceneNodeInfo node with direct connection to this rootSceneNode, 
	// and we load the as children of this entity
	for (void* child = scene->GetFirstChildLink(rootSceneNode); child; child = scene->GetNextChildLink (rootSceneNode, child)) {
		dScene::dTreeNode* const node = scene->GetNodeFromLink(child);
		dNodeInfo* const info = scene->GetInfoFromNode(node);
		if (info->IsType(dSceneNodeInfo::GetRttiType())) {
			new ndDemoEntity (world, scene, node, meshCache, entityDictionary, this);
		}
	}
}
*/

ndDemoEntity::ndDemoEntity(const ndDemoEntity& copyFrom)
	//:dClassInfo()
	:dNodeHierarchy<ndDemoEntity>(copyFrom)
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
	SetMesh(nullptr, dGetIdentityMatrix());
}

dNodeBaseHierarchy* ndDemoEntity::CreateClone () const
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
void ndDemoEntity::TransformCallback(const NewtonBody* body, const dFloat32* matrix, int threadIndex)
{
	ndDemoEntity* const ent = (ndDemoEntity*) NewtonBodyGetUserData(body);
	if (ent) {
		ndDemoEntityManager* const scene = (ndDemoEntityManager*)NewtonWorldGetUserData(NewtonBodyGetWorld(body));
		dMatrix transform(matrix);
		dQuaternion rot;
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

void ndDemoEntity::SetMesh(ndDemoMeshInterface* const mesh, const dMatrix& meshMatrix)
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

const dMatrix& ndDemoEntity::GetMeshMatrix() const
{
	return m_meshMatrix;
}

void ndDemoEntity::SetMeshMatrix(const dMatrix& matrix)
{
	m_meshMatrix = matrix;
}

dMatrix ndDemoEntity::GetCurrentMatrix () const
{
	return dMatrix (m_curRotation, m_curPosition);
}

dMatrix ndDemoEntity::GetNextMatrix () const
{
	return dMatrix (m_nextRotation, m_nextPosition);
}

dMatrix ndDemoEntity::CalculateGlobalMatrix (const ndDemoEntity* const root) const
{
	dMatrix matrix (dGetIdentityMatrix());
	for (const ndDemoEntity* ptr = this; ptr != root; ptr = ptr->GetParent()) {
		matrix = matrix * ptr->GetCurrentMatrix ();
	}
	return matrix;
}

dMatrix ndDemoEntity::CalculateInterpolatedGlobalMatrix (const ndDemoEntity* const root) const
{
	dMatrix matrix (dGetIdentityMatrix());
	for (const ndDemoEntity* ptr = this; ptr != root; ptr = ptr->GetParent()) {
		matrix = matrix * ptr->m_matrix;
	}
	return matrix;
}

void ndDemoEntity::SetMatrixUsafe(const dQuaternion& rotation, const dVector& position)
{
	m_curPosition = m_nextPosition;
	m_curRotation = m_nextRotation;

	m_nextPosition = position;
	m_nextRotation = rotation;

	dFloat32 angle = m_curRotation.DotProduct(m_nextRotation);
	if (angle < 0.0f) {
		m_curRotation.Scale(-1.0f);
	}
}

void ndDemoEntity::SetMatrix(ndDemoEntityManager& world, const dQuaternion& rotation, const dVector& position)
{
	// read the data in a critical section to prevent race condition from other thread  
	dScopeSpinLock lock(m_lock);
	SetMatrixUsafe(rotation, position);
}

void ndDemoEntity::SetNextMatrix (ndDemoEntityManager& world, const dQuaternion& rotation, const dVector& position)
{
	// read the data in a critical section to prevent race condition from other thread  
	dScopeSpinLock lock(m_lock);

	m_nextPosition = position;
	m_nextRotation = rotation;

	dFloat32 angle = m_curRotation.DotProduct(m_nextRotation);
	if (angle < 0.0f) {
		m_curRotation.Scale(-1.0f);
	}
}

void ndDemoEntity::ResetMatrix(ndDemoEntityManager& scene, const dMatrix& matrix)
{
	dQuaternion rot (matrix);
	SetMatrix(scene, rot, matrix.m_posit);
	SetMatrix(scene, rot, matrix.m_posit);
	InterpolateMatrix (scene, 0.0f);
}

void ndDemoEntity::InterpolateMatrixUnsafe(dFloat32 param)
{
	dVector p0(m_curPosition);
	dVector p1(m_nextPosition);
	dQuaternion r0(m_curRotation);
	dQuaternion r1(m_nextRotation);

	dVector posit(p0 + (p1 - p0).Scale(param));
	dQuaternion rotation(r0.Slerp(r1, param));
	m_matrix = dMatrix(rotation, posit);

	for (ndDemoEntity* child = GetChild(); child; child = child->GetSibling()) 
	{
		child->InterpolateMatrixUnsafe(param);
	}
}

void ndDemoEntity::InterpolateMatrix (ndDemoEntityManager& world, dFloat32 param)
{
	// read the data in a critical section to prevent race condition from other thread  
	dScopeSpinLock lock(m_lock);
	InterpolateMatrixUnsafe(param);
}

const dMatrix& ndDemoEntity::GetRenderMatrix () const
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

void ndDemoEntity::Render(dFloat32 timestep, ndDemoEntityManager* const scene, const dMatrix& matrix) const
{
	dMatrix nodeMatrix (m_matrix * matrix);
	if (m_isVisible && m_mesh) 
	{
		// Render mesh if there is one 
		dMatrix modelMatrix (m_meshMatrix * nodeMatrix);
		m_mesh->Render(scene, modelMatrix);
	}

//	RenderBone();
	for (ndDemoEntity* child = GetChild(); child; child = child->GetSibling()) 
	{
		child->Render(timestep, scene, nodeMatrix);
	}
}

/*
ndDemoEntity* ndDemoEntity::LoadNGD_mesh(const char* const fileName, NewtonWorld* const world, const ndShaderPrograms& shaderCache)
{
	dScene scene(world);

	char pathName[2048];
	dGetWorkingFileName(fileName, pathName);
	scene.Deserialize(pathName);

	// this will apply all global the scale to the mesh
	scene.FreezeScale();

	// this will apply all local scale and transform to the mesh
	scene.FreezeGeometryPivot();

	int rootCount = 0;
	dScene::dTreeNode* meshRootNodeArray[256];
	dScene::dTreeNode* const root = scene.GetRootNode();
	for (void* child = scene.GetFirstChildLink(root); child; child = scene.GetNextChildLink(root, child)) {
		dScene::dTreeNode* const sceneNode = scene.GetNodeFromLink(child);
		dNodeInfo* const nodeInfo = scene.GetInfoFromNode(sceneNode);
		if (nodeInfo->GetTypeId() == dSceneNodeInfo::GetRttiType()) {
			meshRootNodeArray[rootCount] = sceneNode;
			rootCount ++;
		}
	}

	dTree<ndDemoEntity*, dScene::dTreeNode*> boneMap;
	ndDemoEntity* returnEntity = nullptr;
	ndDemoEntity* entity__[256];
	entity__[0] = nullptr;
	if (rootCount) {
		int stack = 0;
		int modifiersCount = 0;
		ndDemoEntity* entityStack[256];
		dScene::dTreeNode* nodeStack[256];
		ndDemoEntity* entityModifiers[256];
		dScene::dTreeNode* nodeModifiers[256];

		ndDemoEntity* parent = nullptr;
		if (rootCount > 1) {
			parent = new ndDemoEntity(dGetIdentityMatrix(), nullptr);
		}
		for (dInt32 i = 0; i < rootCount; i ++) {
			entity__[stack] = new ndDemoEntity(dGetIdentityMatrix(), parent);
			entityStack[stack] = entity__[stack];
			nodeStack[stack] = meshRootNodeArray[stack];
			stack ++;
		}

		dTree<ndDemoMeshInterface*, dScene::dTreeNode*> meshDictionary;
		while (stack) {
			stack--;

			ndDemoEntity* const entity = entityStack[stack];
			dScene::dTreeNode* sceneNode = nodeStack[stack];

			boneMap.Insert(entity, sceneNode);

			dSceneNodeInfo* const sceneInfo = (dSceneNodeInfo*)scene.GetInfoFromNode(sceneNode);
			dMatrix matrix(sceneInfo->GetTransform());
			dQuaternion rot(matrix);
			entity->m_curPosition = matrix.m_posit;
			entity->m_nextPosition = matrix.m_posit;
			entity->m_curRotation = rot;
			entity->m_nextRotation = rot;
			entity->m_matrix = matrix;
			entity->SetNameID(sceneInfo->GetName());
			const char* const name = entity->GetName().GetStr();
			//dTrace(("%s\n", name));
			if (strstr(name, "Hidden") || strstr(name, "hidden")) {
				entity->m_isVisible = false;
				//dTrace(("%s %s\n", name, entity->GetParent()->GetName().GetStr()));
			}

			for (void* child = scene.GetFirstChildLink(sceneNode); child; child = scene.GetNextChildLink(sceneNode, child)) {
				dScene::dTreeNode* const meshNode = scene.GetNodeFromLink(child);
				dNodeInfo* const nodeInfo = scene.GetInfoFromNode(meshNode);

				if (nodeInfo->GetTypeId() == dMeshNodeInfo::GetRttiType()) {
					dTree<ndDemoMeshInterface*, dScene::dTreeNode*>::dTreeNode* cacheNode = meshDictionary.Find(meshNode);
					if (!cacheNode) {
						ndDemoMeshInterface* const mesh = new ndDemoMesh(&scene, meshNode, shaderCache);
						cacheNode = meshDictionary.Insert(mesh, meshNode);
					}
					ndDemoMeshInterface* const mesh = cacheNode->GetInfo();
					entity->SetMesh(mesh, sceneInfo->GetGeometryTransform());

					for (void* modifierChild = scene.GetFirstChildLink(meshNode); modifierChild; modifierChild = scene.GetNextChildLink(sceneNode, modifierChild)) {
						dScene::dTreeNode* const modifierNode = scene.GetNodeFromLink(modifierChild);
						dGeometryNodeSkinClusterInfo* const modifierInfo = (dGeometryNodeSkinClusterInfo*)scene.GetInfoFromNode(modifierNode);
						if (modifierInfo->GetTypeId() == dGeometryNodeSkinClusterInfo::GetRttiType()) {
							entityModifiers[modifiersCount] = entity;
							nodeModifiers[modifiersCount] = meshNode;
							modifiersCount++;
							break;
						}
					}

				} else if (nodeInfo->GetTypeId() == dLineNodeInfo::GetRttiType()) {
					dTree<ndDemoMeshInterface*, dScene::dTreeNode*>::dTreeNode* cacheNode = meshDictionary.Find(meshNode);
					if (!cacheNode) {
						ndDemoMeshInterface* const mesh = new ndDemoBezierCurve(&scene, meshNode);
						cacheNode = meshDictionary.Insert(mesh, meshNode);
					}
					ndDemoMeshInterface* const mesh = cacheNode->GetInfo();
					entity->SetMesh(mesh, sceneInfo->GetGeometryTransform());
				}
			}

			for (void* child = scene.GetFirstChildLink(sceneNode); child; child = scene.GetNextChildLink(sceneNode, child)) {
				dScene::dTreeNode* const node = scene.GetNodeFromLink(child);
				dNodeInfo* const nodeInfo = scene.GetInfoFromNode(node);
				if (nodeInfo->IsType(dSceneNodeInfo::GetRttiType())) {
					nodeStack[stack] = node;
					entityStack[stack] = new ndDemoEntity(dGetIdentityMatrix(), entity);
					stack++;
				}
			}
		}

		while (meshDictionary.GetCount()) {
			dTree<ndDemoMeshInterface*, dScene::dTreeNode*>::dTreeNode* const node = meshDictionary.GetRoot();
			ndDemoMeshInterface* const mesh = node->GetInfo();
			mesh->Release();
			meshDictionary.Remove(node);
		}

		returnEntity = entity__[0] ? (entity__[0]->GetParent() ? entity__[0]->GetParent() : entity__[0]) : nullptr;
		if (modifiersCount) {
			for (dInt32 i = 0; i < modifiersCount; i++) {
				dScene::dTreeNode* const skinMeshNode = nodeModifiers[i];
				dAssert (((dMeshNodeInfo*)scene.GetInfoFromNode(skinMeshNode))->GetTypeId() == dMeshNodeInfo::GetRttiType());
				ndDemoEntity* const skinEntity = entityModifiers[i];
				ndDemoSkinMesh* const skinMesh = new ndDemoSkinMesh(&scene, skinEntity, skinMeshNode, boneMap, shaderCache);
				skinEntity->SetMesh(skinMesh, skinEntity->GetMeshMatrix());
				skinMesh->Release();
			}
		}
	}

	return returnEntity;
}


NewtonCollision* ndDemoEntity::CreateCollisionFromchildren(NewtonWorld* const world) const
{
	int count = 1;
	NewtonCollision* shapeArray[128];
	
	shapeArray[0] = nullptr;
	for (ndDemoEntity* child = GetChild(); child; child = child->GetSibling()) {
		//const char* const name = child->GetName().GetStr();
		dString tmpName(child->GetName());
		tmpName.ToLower();
		const char* const name = tmpName.GetStr();

		if (strstr (name, "sphere")) {
			ndDemoMesh* const mesh = (ndDemoMesh*)child->GetMesh();
			dAssert(0);
			//dAssert(mesh->IsType(ndDemoMesh::GetRttiType()));
			dFloat32* const array = mesh->m_vertex;
			dVector extremes(0.0f);
			for (dInt32 i = 0; i < mesh->m_vertexCount; i++) {
				extremes.m_x = dMax(extremes.m_x, array[i * 3 + 0]);
				extremes.m_y = dMax(extremes.m_y, array[i * 3 + 1]);
				extremes.m_z = dMax(extremes.m_z, array[i * 3 + 2]);
			}

			dMatrix matrix(child->GetCurrentMatrix());
			shapeArray[count] = NewtonCreateSphere(world, extremes.m_x, 0, &matrix[0][0]);
			count++;
			dAssert(count < sizeof(shapeArray) / sizeof (shapeArray[0]));
		} else if (strstr (name, "box")) {
			ndDemoMesh* const mesh = (ndDemoMesh*)child->GetMesh();
			dAssert(0);
			//dAssert(mesh->IsType(ndDemoMesh::GetRttiType()));
			// go over the vertex array and find and collect all vertices's weighted by this bone.
			dFloat32* const array = mesh->m_vertex;
			dVector extremes(0.0f);
			for (dInt32 i = 0; i < mesh->m_vertexCount; i++) {
				extremes.m_x = dMax(extremes.m_x, array[i * 3 + 0]);
				extremes.m_y = dMax(extremes.m_y, array[i * 3 + 1]);
				extremes.m_z = dMax(extremes.m_z, array[i * 3 + 2]);
			}

			extremes = extremes.Scale (2.0f); 
			dMatrix matrix(child->GetCurrentMatrix());
			shapeArray[count] = NewtonCreateBox(world, extremes.m_x, extremes.m_y, extremes.m_z, 0, &matrix[0][0]);
			count++;
			dAssert(count < sizeof(shapeArray) / sizeof (shapeArray[0]));

		} else if (strstr (name, "capsule")) {
			ndDemoMesh* const mesh = (ndDemoMesh*)child->GetMesh();
			dAssert(0);
			//dAssert(mesh->IsType(ndDemoMesh::GetRttiType()));
			dFloat32* const array = mesh->m_vertex;
			dVector extremes(0.0f);
			for (dInt32 i = 0; i < mesh->m_vertexCount; i++) {
				extremes.m_x = dMax(extremes.m_x, array[i * 3 + 0]);
				extremes.m_y = dMax(extremes.m_y, array[i * 3 + 1]);
				extremes.m_z = dMax(extremes.m_z, array[i * 3 + 2]);
			}
			dFloat32 high = 2.0f * dMax (extremes.m_y - extremes.m_x, dFloat32 (0.0f));

			dMatrix alighMatrix(dRollMatrix(90.0f * dDegreeToRad));
			dMatrix matrix (alighMatrix * child->GetCurrentMatrix());
			shapeArray[count] = NewtonCreateCapsule(world, extremes.m_x, extremes.m_x, high, 0, &matrix[0][0]);
			count++;
			dAssert(count < sizeof(shapeArray)/ sizeof (shapeArray[0]));
		} else if (strstr(name, "convexhull")) {
			ndDemoMesh* const mesh = (ndDemoMesh*)child->GetMesh();
			dAssert(0);
			//dAssert(mesh->IsType(ndDemoMesh::GetRttiType()));
			dFloat32* const array = mesh->m_vertex;
			dMatrix matrix(child->GetCurrentMatrix());
			shapeArray[count] = NewtonCreateConvexHull(world, mesh->m_vertexCount, array, 3 * sizeof (dFloat32), 0.01f, 0, &matrix[0][0]);
			count++;
			dAssert(count < sizeof(shapeArray) / sizeof(shapeArray[0]));
		}
	}

	if (count > 2) {
		NewtonCollision* const compound = NewtonCreateCompoundCollision (world, 0);
		NewtonCompoundCollisionBeginAddRemove (compound);	
		for (dInt32 i = 1; i < count; i ++) {
			NewtonCompoundCollisionAddSubCollision (compound, shapeArray[i]);
			NewtonDestroyCollision(shapeArray[i]);
		}
		NewtonCompoundCollisionEndAddRemove (compound);	
		shapeArray[0] = compound;
		count = 1;
	} if (count == 2) {
		shapeArray[0] = shapeArray[1];
	}
	return shapeArray[0];
}
*/