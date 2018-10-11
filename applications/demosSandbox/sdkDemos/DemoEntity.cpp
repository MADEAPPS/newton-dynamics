/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "toolbox_stdafx.h"
#include "DemoMesh.h"
#include "DemoEntity.h"

dInitRtti(DemoEntity);


DemoEntity::DemoEntity(const dMatrix& matrix, DemoEntity* const parent)
	:dClassInfo()
	,dHierarchy<DemoEntity>()
	,m_matrix(matrix) 
	,m_curPosition (matrix.m_posit)
	,m_nextPosition (matrix.m_posit)
	,m_curRotation (dQuaternion (matrix))
	,m_nextRotation (dQuaternion (matrix))
	,m_meshMatrix(dGetIdentityMatrix())
	,m_mesh (NULL)
	,m_userData(NULL)
	,m_lock(0) 
{
	if (parent) {
		Attach (parent);
	}
}

DemoEntity::DemoEntity(DemoEntityManager& world, const dScene* const scene, dScene::dTreeNode* const rootSceneNode, dTree<DemoMeshInterface*, dScene::dTreeNode*>& meshCache, DemoEntityManager::EntityDictionary& entityDictionary, DemoEntity* const parent)
	:dClassInfo()
	,dHierarchy<DemoEntity>() 
	,m_matrix(dGetIdentityMatrix()) 
	,m_curPosition (0.0f, 0.0f, 0.0f, 1.0f)
	,m_nextPosition (0.0f, 0.0f, 0.0f, 1.0f)
	,m_curRotation (1.0f, 0.0f, 0.0f, 0.0f)
	,m_nextRotation (1.0f, 0.0f, 0.0f, 0.0f)
	,m_meshMatrix(dGetIdentityMatrix())
	,m_mesh (NULL)
	,m_userData(NULL)
	,m_lock(0) 
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
		DemoMeshInterface* const mesh = meshCache.Find(meshNode)->GetInfo();
		SetMesh(mesh, sceneInfo->GetGeometryTransform());
	}

	// we now scan for all dSceneNodeInfo node with direct connection to this rootSceneNode, 
	// and we load the as children of this entity
	for (void* child = scene->GetFirstChildLink(rootSceneNode); child; child = scene->GetNextChildLink (rootSceneNode, child)) {
		dScene::dTreeNode* const node = scene->GetNodeFromLink(child);
		dNodeInfo* const info = scene->GetInfoFromNode(node);
		if (info->IsType(dSceneNodeInfo::GetRttiType())) {
			new DemoEntity (world, scene, node, meshCache, entityDictionary, this);
		}
	}
}

DemoEntity::DemoEntity(const DemoEntity& copyFrom)
	:dClassInfo()
	,dHierarchy<DemoEntity>(copyFrom)
	,m_matrix(copyFrom.m_matrix)
	,m_curPosition(copyFrom.m_curPosition)
	,m_nextPosition(copyFrom.m_nextPosition)
	,m_curRotation(copyFrom.m_curRotation)
	,m_nextRotation(copyFrom.m_nextRotation)
	,m_meshMatrix(copyFrom.m_meshMatrix)
	,m_mesh(copyFrom.m_mesh)
	,m_userData(NULL)
	,m_lock(0)
{
	if (m_mesh) {
		m_mesh->AddRef();
	}
}

DemoEntity::~DemoEntity(void)
{
	if (m_userData) {
		delete m_userData;
	}
	SetMesh(NULL, dGetIdentityMatrix());
}


dBaseHierarchy* DemoEntity::CreateClone () const
{
	return new DemoEntity(*this);
}


DemoEntity::UserData* DemoEntity::GetUserData ()
{
	return m_userData;
}

void DemoEntity::SetUserData (UserData* const data)
{
	m_userData = data;
}

void DemoEntity::TransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex)
{
	DemoEntity* const ent = (DemoEntity*) NewtonBodyGetUserData(body);
	if (ent) {
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(NewtonBodyGetWorld(body));
		dMatrix transform(matrix);
		//dQuaternion rot1(transform);
		dQuaternion rot;
		NewtonBodyGetRotation(body, &rot.m_q0);

		scene->Lock(ent->m_lock);
		ent->SetMatrixUsafe(rot, transform.m_posit);
		if (ent->m_userData) {
			ent->m_userData->OnTransformCallback(*scene);
		}
		scene->Unlock(ent->m_lock);
	}
}

DemoMeshInterface* DemoEntity::GetMesh() const
{
	return m_mesh;
}

void DemoEntity::SetMesh(DemoMeshInterface* const mesh, const dMatrix& meshMatrix)
{
	m_meshMatrix = meshMatrix;
	if (m_mesh) {
		m_mesh->Release();
	}
	m_mesh = mesh;
	if (mesh) {
		mesh->AddRef();
	}
}

const dMatrix& DemoEntity::GetMeshMatrix() const
{
	return m_meshMatrix;
}

void DemoEntity::SetMeshMatrix(const dMatrix& matrix)
{
	m_meshMatrix = matrix;
}

dMatrix DemoEntity::GetCurrentMatrix () const
{
	return dMatrix (m_curRotation, m_curPosition);
}

dMatrix DemoEntity::GetNextMatrix () const
{
	return dMatrix (m_nextRotation, m_nextPosition);
}

dMatrix DemoEntity::CalculateGlobalMatrix (const DemoEntity* const root) const
{
	dMatrix matrix (dGetIdentityMatrix());
	for (const DemoEntity* ptr = this; ptr != root; ptr = ptr->GetParent()) {
		matrix = matrix * ptr->GetCurrentMatrix ();
	}
	return matrix;
}

dMatrix DemoEntity::CalculateInterpolatedGlobalMatrix (const DemoEntity* const root) const
{
	dMatrix matrix (dGetIdentityMatrix());
	for (const DemoEntity* ptr = this; ptr != root; ptr = ptr->GetParent()) {
		matrix = matrix * ptr->m_matrix;
	}
	return matrix;
}

void DemoEntity::SetMatrixUsafe(const dQuaternion& rotation, const dVector& position)
{
	m_curPosition = m_nextPosition;
	m_curRotation = m_nextRotation;

	m_nextPosition = position;
	m_nextRotation = rotation;

	dFloat angle = m_curRotation.DotProduct(m_nextRotation);
	if (angle < 0.0f) {
		m_curRotation.Scale(-1.0f);
	}
}

void DemoEntity::SetMatrix(DemoEntityManager& world, const dQuaternion& rotation, const dVector& position)
{
	// read the data in a critical section to prevent race condition from other thread  
	world.Lock(m_lock);

	SetMatrixUsafe(rotation, position);

	// release the critical section
	world.Unlock(m_lock);
}

void DemoEntity::SetNextMatrix (DemoEntityManager& world, const dQuaternion& rotation, const dVector& position)
{
	// read the data in a critical section to prevent race condition from other thread  
	world.Lock(m_lock);

	m_nextPosition = position;
	m_nextRotation = rotation;

	dFloat angle = m_curRotation.DotProduct(m_nextRotation);
	if (angle < 0.0f) {
		m_curRotation.Scale(-1.0f);
	}

	// release the critical section
	world.Unlock(m_lock);
}


void DemoEntity::ResetMatrix(DemoEntityManager& world, const dMatrix& matrix)
{
	dQuaternion rot (matrix);
	SetMatrix(world, rot, matrix.m_posit);
	SetMatrix(world, rot, matrix.m_posit);
	InterpolateMatrix (world, 0.0f);
}

void DemoEntity::InterpolateMatrixUsafe(dFloat param)
{
	dVector p0(m_curPosition);
	dVector p1(m_nextPosition);
	dQuaternion r0(m_curRotation);
	dQuaternion r1(m_nextRotation);

	dVector posit(p0 + (p1 - p0).Scale(param));
	dQuaternion rotation(r0.Slerp(r1, param));

	m_matrix = dMatrix(rotation, posit);
}


void DemoEntity::InterpolateMatrix (DemoEntityManager& world, dFloat param)
{
	// read the data in a critical section to prevent race condition from other thread  
	world.Lock(m_lock);

	InterpolateMatrixUsafe(param);
	if (m_userData) {
		m_userData->OnInterpolateMatrix(world, param);
	}

	// release the critical section
	world.Unlock(m_lock);
}


const dMatrix& DemoEntity::GetRenderMatrix () const
{
	return m_matrix;
}


void DemoEntity::Render(dFloat timestep, DemoEntityManager* const scene) const
{
	// save the model matrix before changing it Matrix
	glPushMatrix();

	// Set The matrix for this entity Node
	glMultMatrix(&m_matrix[0][0]);

	// Render mesh if there is one 
	if (m_mesh) {
		glPushMatrix();
		glMultMatrix(&m_meshMatrix[0][0]);
		m_mesh->Render (scene);
//		m_mesh->RenderNormals ();

		if (m_userData) {
			m_userData->OnRender(timestep);
		}
		glPopMatrix();
	}

	for (DemoEntity* child = GetChild(); child; child = child->GetSibling()) {
		child->Render(timestep, scene);
	}

	// restore the matrix before leaving
	glPopMatrix();
	
}

DemoEntity* DemoEntity::LoadNGD_mesh(const char* const fileName, NewtonWorld* const world)
{
	dScene scene(world);

	char pathName[2048];
	dGetWorkingFileName(fileName, pathName);
	scene.Deserialize(pathName);

	// this will apply all global the scale to the mesh
	scene.FreezeScale();

	// this will apply all local scale and transform to the mesh
	scene.FreezeGeometryPivot();

	dScene::dTreeNode* meshRootNode = NULL;
	dScene::dTreeNode* const root = scene.GetRootNode();
	for (void* child = scene.GetFirstChildLink(root); child; child = scene.GetNextChildLink(root, child)) {
		dScene::dTreeNode* const sceneNode = scene.GetNodeFromLink(child);
		dNodeInfo* const scaneInfo = scene.GetInfoFromNode(sceneNode);
		if (scaneInfo->GetTypeId() == dSceneNodeInfo::GetRttiType()) {
			meshRootNode = sceneNode;
			break;
		}
	}

	DemoEntity* const entity = new DemoEntity(dGetIdentityMatrix(), NULL);
	if (meshRootNode) {
		int stack;
		int modifiersCount = 0;
		DemoEntity* entityStack[256];
		dScene::dTreeNode* nodeStack[256];
		dScene::dTreeNode* nodeModifiers[256];

		entityStack[0] = entity;
		nodeStack[0] = meshRootNode;

		stack = 1;

		dTree<DemoMeshInterface*, dScene::dTreeNode*> meshDictionary;
		while (stack) {
			stack--;

			DemoEntity* const entity = entityStack[stack];
			dScene::dTreeNode* sceneNode = nodeStack[stack];

			//dMatrix parentMatrix (GetIdentityMatrix());
			//dScene::dTreeNode* const parentNode = scene.FindParentByType(rootSceneNode, dSceneNodeInfo::GetRttiType());
			//if (parentNode) {
			//	dSceneNodeInfo* const parentInfo = (dSceneNodeInfo*)scene.GetInfoFromNode (parentNode);
			//	dAssert (parentInfo->IsType(dSceneNodeInfo::GetRttiType()));
			//	parentMatrix = parentInfo->GetTransform();
			//}

			dSceneNodeInfo* const sceneInfo = (dSceneNodeInfo*)scene.GetInfoFromNode(sceneNode);
			//dMatrix matrix (sceneInfo->GetTransform() * parentMatrix.Inverse4x4());
			dMatrix matrix(sceneInfo->GetTransform());
			dQuaternion rot(matrix);
			entity->m_curPosition = matrix.m_posit;
			entity->m_nextPosition = matrix.m_posit;
			entity->m_curRotation = rot;
			entity->m_nextRotation = rot;
			entity->m_matrix = matrix;
			entity->SetNameID(sceneInfo->GetName());

			for (void* child = scene.GetFirstChildLink(sceneNode); child; child = scene.GetNextChildLink(sceneNode, child)) {
				dScene::dTreeNode* const node = scene.GetNodeFromLink(child);
				dNodeInfo* const nodeInfo = scene.GetInfoFromNode(node);

				if (nodeInfo->GetTypeId() == dMeshNodeInfo::GetRttiType()) {
					dTree<DemoMeshInterface*, dScene::dTreeNode*>::dTreeNode* cacheNode = meshDictionary.Find(node);
					if (!cacheNode) {
						DemoMeshInterface* const mesh = new DemoMesh(&scene, node);
						cacheNode = meshDictionary.Insert(mesh, node);
					}
					DemoMeshInterface* const mesh = cacheNode->GetInfo();
					entity->SetMesh(mesh, sceneInfo->GetGeometryTransform());

					// save the modifiers
					for (void* ptr = scene.GetFirstChildLink(node); ptr; ptr = scene.GetNextChildLink(node, ptr)) {
						dScene::dTreeNode* const modifierNode = scene.GetNodeFromLink(ptr);
						dNodeInfo* const modifierInfo = scene.GetInfoFromNode(modifierNode);
						if (modifierInfo->IsType(dGeometryNodeModifierInfo::GetRttiType())) {
							nodeModifiers[modifiersCount] = modifierNode;
							modifiersCount++;
						}
					}

				}
				else if (nodeInfo->GetTypeId() == dLineNodeInfo::GetRttiType()) {
					dTree<DemoMeshInterface*, dScene::dTreeNode*>::dTreeNode* cacheNode = meshDictionary.Find(node);
					if (!cacheNode) {
						DemoMeshInterface* const mesh = new DemoBezierCurve(&scene, node);
						cacheNode = meshDictionary.Insert(mesh, node);
					}
					DemoMeshInterface* const mesh = cacheNode->GetInfo();
					entity->SetMesh(mesh, sceneInfo->GetGeometryTransform());
				}
			}

			for (void* child = scene.GetFirstChildLink(sceneNode); child; child = scene.GetNextChildLink(sceneNode, child)) {
				dScene::dTreeNode* const node = scene.GetNodeFromLink(child);
				dNodeInfo* const info = scene.GetInfoFromNode(node);
				if (info->IsType(dSceneNodeInfo::GetRttiType())) {
					nodeStack[stack] = node;
					entityStack[stack] = new DemoEntity(dGetIdentityMatrix(), entity);
					stack++;
				}
			}
		}

		while (meshDictionary.GetCount()) {
			dTree<DemoMeshInterface*, dScene::dTreeNode*>::dTreeNode* const node = meshDictionary.GetRoot();
			DemoMeshInterface* const mesh = node->GetInfo();
			mesh->Release();
			meshDictionary.Remove(node);
		}

		for (int i = 0; i < modifiersCount; i++) {
			dScene::dTreeNode* const node = nodeModifiers[i];
			dNodeInfo* const nodeInfo = scene.GetInfoFromNode(node);
			if (nodeInfo->GetTypeId() == dGeometryNodeSkinModifierInfo::GetRttiType()) {
				//				dAssert (0);
				//skinMesh = (dGeometryNodeSkinModifierInfo*)info;
			}
		}
	}

	return entity;
}

DemoEntity* DemoEntity::LoadOBJ_mesh (const char* const fileName, NewtonWorld* const world, const dMatrix& convertMatrix)
{
	char pathName[2048];
	dGetWorkingFileName(fileName, pathName);

	FILE* const file = fopen(pathName, "rb");
//	materialLibraryOut[0] = 0;

	if (file) {
		int uvCount = 0;

		int vertexCount = 0;
		int normalCount = 0;
		int uvMaxCount = 4096;
		int vertexMaxCount = 4096;
		int normalMaxCount = 4096;
		dVector* vertex = new dVector[4096];
		dVector* normal = new dVector[4096];
		dVector* uv = new dVector[4096];
		dTree<int, dString> materialMap;

		int materialId = 0;
		int materialIndex = 0;
		bool hasUV = false;
		bool hasNormal = false;
		char line[1024];

		dNewtonMesh mesh(world);
		mesh.BeginBuild();

		while (!feof(file)) {
			fgets(line, sizeof(line) - 1, file);

			int index = 0;
			while (line[index] && (line[index] != '\r') && (line[index] != '\n')) {
				dFloat32 x;
				dFloat32 y;
				dFloat32 z;
				char token[256];
				sscanf(&line[index], "%s", token);
				if (strcmp(token, "#") == 0) {
					index = int (strlen(line));
				} else if (strcmp(token, "mtllib") == 0) {
					char* ptr = strstr(line, token);
					ptr += int (strlen(token));
					//dAssert(0);
					//sscanf(ptr, "%s", materialLibraryOut);

					index = int (strlen(line));
				} else if (strcmp(token, "v") == 0) {
					sscanf(&line[index + 1], "%f %f %f", &x, &y, &z);
					vertex[vertexCount] = dVector(x, y, z, 0.0f);
					vertexCount++;
					if (vertexCount >= vertexMaxCount) {
						dAssert(0);
					}
					index = int (strlen(line));
				} else if (strcmp(token, "vn") == 0) {
					hasNormal = true;
					sscanf(&line[index + 1], "%f %f %f", &x, &y, &z);
					normal[vertexCount] = dVector(x, y, z, 0.0f);
					normalCount++;
					if (normalCount >= normalMaxCount) {
						dAssert(0);
					}
					index = int (strlen(line));
				} else if (strcmp(token, "vt") == 0) {
					hasUV = true;
					sscanf(&line[index + 1], "%f %f %f", &x, &y, &z);
					uv[vertexCount] = dVector(x, y, 0.0f, 0.0f);
					uvCount++;
					if (uvCount >= uvMaxCount) {
						dAssert(0);
					}
					index = int (strlen(line));
				} else if (strcmp(token, "g") == 0) {
					sscanf(&line[index + 1], "%s", token);
					index = int (strlen(line));
				} else if (strcmp(token, "usemtl") == 0) {
					char* ptr = strstr(line, token);
					ptr += strlen(token);
					sscanf(ptr, "%s", token);
					dTree<int, dString>::dTreeNode* node = materialMap.Find(token);
					if (!node) {
						node = materialMap.Insert(materialIndex, token);
						materialIndex++;
					}
					materialId = node->GetInfo();

					index = int (strlen(line));
				} else if (strcmp(token, "s") == 0) {
					//fscanf(file, "%d", &material);
					index = int (strlen(line));
				} else if (strcmp(token, "f") == 0) {
					mesh.BeginPolygon();
					char* ptr = &line[index + 1];
					do {
						token[0] = 0;
						sscanf(ptr, "%s", token);
						if (*token) {
							if (hasUV && hasNormal) {
								int v;
								int n;
								int t;
								sscanf(token, "%d/%d/%d", &v, &t, &n);
								v--;
								t--;
								n--;
								mesh.AddPoint(vertex[v].m_x, vertex[v].m_y, vertex[v].m_z);
								mesh.AddNormal(dFloat32(normal[n].m_x), dFloat32(normal[n].m_y), dFloat32(normal[n].m_z));
								mesh.AddUV0(dFloat32(uv[t].m_x), dFloat32(uv[t].m_y));
								mesh.AddMaterial(materialId);
							} else {
								dAssert(0);
							}

							ptr = strstr(ptr, token);
							ptr += strlen(token);
						}
					} while (*token);
					mesh.EndPolygon();
					index = int (strlen(line));
				} else {
					dAssert(0);
				}
			}
		}
		mesh.EndBuild();
		delete[] uv;
		delete[] normal;
		delete[] vertex;
		fclose(file);

		mesh.ApplyTransform(&convertMatrix[0][0]);
		mesh.CalculateVertexNormals(20.0f * dDegreeToRad);
		DemoMesh* const visualMesh = new DemoMesh(mesh.GetMesh());

		DemoEntity* const entity = new DemoEntity(dGetIdentityMatrix(), NULL);
		entity->SetMesh(visualMesh, dGetIdentityMatrix());
		visualMesh->Release();
		return entity;
	}
	return NULL;
}