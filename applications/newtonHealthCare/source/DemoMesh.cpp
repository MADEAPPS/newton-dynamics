/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// DemoMesh.cpp: implementation of the DemoMesh class.
//
//////////////////////////////////////////////////////////////////////
#include "toolbox_stdafx.h"
#include "DemoMesh.h"
#include "TargaToOpenGl.h"

dInitRtti(DemoMesh);


#define USING_DISPLAY_LIST

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
DemoSubMesh::DemoSubMesh ()
	:m_indexCount(0)
	,m_indexes(NULL)
	,m_textureHandle(0)
	,m_shiness(80.0f)
	,m_ambient (0.8f, 0.8f, 0.8f, 1.0f)
	,m_diffuse (0.8f, 0.8f, 0.8f, 1.0f)
	,m_specular (1.0f, 1.0f, 1.0f, 1.0f)
{
}

DemoSubMesh::~DemoSubMesh ()
{
	if (m_indexes) {
		free (m_indexes);
	}
}



void DemoSubMesh::Render() const
{
	glMaterialfv(GL_FRONT, GL_SPECULAR, &m_specular.m_x);
	glMaterialfv(GL_FRONT, GL_AMBIENT, &m_ambient.m_x);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, &m_diffuse.m_x);
	glMaterialf(GL_FRONT, GL_SHININESS, m_shiness);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	if (m_textureHandle) {
		glEnable(GL_TEXTURE_2D);		
		glBindTexture(GL_TEXTURE_2D, GLuint (m_textureHandle));
	} else {
		glDisable(GL_TEXTURE_2D);
	}

	glDrawElements (GL_TRIANGLES, m_indexCount, GL_UNSIGNED_INT, m_indexes);
}

void DemoSubMesh::OptimizeForRender(const DemoMesh* const mesh) const
{
	glMaterialfv(GL_FRONT, GL_SPECULAR, &m_specular.m_x);
	glMaterialfv(GL_FRONT, GL_AMBIENT, &m_ambient.m_x);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, &m_diffuse.m_x);
	glMaterialf(GL_FRONT, GL_SHININESS, m_shiness);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	if (m_textureHandle) {
		glEnable(GL_TEXTURE_2D);		
		glBindTexture(GL_TEXTURE_2D, GLuint (m_textureHandle));
	} else {
		glDisable(GL_TEXTURE_2D);
	}

	glBegin(GL_TRIANGLES);
	const dFloat* const uv = mesh->m_uv;
	const dFloat* const normal = mesh->m_normal;
	const dFloat* const vertex = mesh->m_vertex;
	for (int i = 0; i < m_indexCount; i ++) {
		int index = m_indexes[i];
		glTexCoord2f(uv[index * 2 + 0], uv[index * 2 + 1]); 
		glNormal3f (normal[index * 3 + 0], normal[index * 3 + 1], normal[index * 3 + 2]); 
		glVertex3f(vertex[index * 3 + 0], vertex[index * 3 + 1], vertex[index * 3 + 2]);
	}

	glEnd();
}

void DemoSubMesh::AllocIndexData (int indexCount)
{
	m_indexCount = indexCount;
	if (m_indexes) {
		free (m_indexes);
	}
	m_indexes = (unsigned *) malloc (m_indexCount * sizeof (unsigned)); 
}



DemoMesh::DemoMesh(const char* const name)
	:dClassInfo()
	,dList<DemoSubMesh>()
	,m_vertexCount(0)
	,m_uv (NULL)
	,m_vertex(NULL)
	,m_normal(NULL)
	,m_optilizedDipalyList(0)		
{
	strcpy (m_name, name);
}

DemoMesh::DemoMesh(const dScene* const scene, dScene::dTreeNode* const meshNode)
	:dClassInfo()
	,dList<DemoSubMesh>()
	,m_uv(NULL)
	,m_vertex(NULL)
	,m_normal(NULL)
	,m_optilizedDipalyList(0)		
{
	dMeshNodeInfo* const info = (dMeshNodeInfo*)scene->GetInfoFromNode(meshNode);
	strcpy (m_name, info->GetName());
	
	NewtonMesh* const mesh = info->GetMesh();

	// extract vertex data  from the newton mesh		
	AllocVertexData(NewtonMeshGetPointCount (mesh));
	NewtonMeshGetVertexStreams (mesh, 3 * sizeof (dFloat), (dFloat*) m_vertex,
									  3 * sizeof (dFloat), (dFloat*) m_normal,
									  2 * sizeof (dFloat), (dFloat*) m_uv, 
								      2 * sizeof (dFloat), (dFloat*) m_uv);

	// bake the matrix into the vertex array
	dMatrix matrix (info->GetPivotMatrix());
	matrix.TransformTriplex(m_vertex, 3 * sizeof (dFloat), m_vertex, 3 * sizeof (dFloat), m_vertexCount);
	matrix.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);
	matrix = matrix.Inverse4x4() * matrix.Transpose() * matrix;
	matrix.TransformTriplex(m_normal, 3 * sizeof (dFloat), m_normal, 3 * sizeof (dFloat), m_vertexCount);

	dTree<dScene::dTreeNode*, dCRCTYPE> materialMap;
	for (void* ptr = scene->GetFirstChild(meshNode); ptr; ptr = scene->GetNextChild (meshNode, ptr)) {
		dScene::dTreeNode* node = scene->GetNodeFromLink(ptr);
		dNodeInfo* info = scene->GetInfoFromNode(node);
		if (info->GetTypeId() == dMaterialNodeInfo::GetRttiType()) {
			dMaterialNodeInfo* const material = (dMaterialNodeInfo*)info;
			dCRCTYPE id = material->GetId();
			materialMap.Insert(node, id);
		}
	}

	// extract the materials index array for mesh
	void* const meshCookie = NewtonMeshBeginHandle (mesh); 
	for (int handle = NewtonMeshFirstMaterial (mesh, meshCookie); handle != -1; handle = NewtonMeshNextMaterial (mesh, meshCookie, handle)) {
		int materialIndex = NewtonMeshMaterialGetMaterial (mesh, meshCookie, handle); 
		int indexCount = NewtonMeshMaterialGetIndexCount (mesh, meshCookie, handle); 
		DemoSubMesh* const segment = AddSubMesh();

		dTree<dScene::dTreeNode*, dCRCTYPE>::dTreeNode* matNodeCache = materialMap.Find(materialIndex);
		if (matNodeCache) {
			dScene::dTreeNode* const matNode = matNodeCache->GetInfo();
			dMaterialNodeInfo* const material = (dMaterialNodeInfo*) scene->GetInfoFromNode(matNode);

			if (material->GetDiffuseTextId() != -1) {
				dScene::dTreeNode* const node = scene->FindTextureByTextId(matNode, material->GetDiffuseTextId());
				_ASSERTE (node);
				dTextureNodeInfo* const texture = (dTextureNodeInfo*)scene->GetInfoFromNode(node);

				segment->m_textureHandle = LoadTexture(texture->GetPathName());
				strcpy (segment->m_textureName, texture->GetPathName());
			}
			segment->m_shiness = material->GetShininess();
			segment->m_ambient = material->GetAmbientColor();
			segment->m_diffuse = material->GetDiffuseColor();
			segment->m_specular = material->GetSpecularColor();
		}

		segment->AllocIndexData (indexCount);
		// for 16 bit indices meshes
		//NewtonMeshMaterialGetIndexStreamShort (mesh, meshCookie, handle, (short int*)segment->m_indexes); 

		// for 32 bit indices mesh
		NewtonMeshMaterialGetIndexStream (mesh, meshCookie, handle, (int*)segment->m_indexes); 
	}
	NewtonMeshEndHandle (mesh, meshCookie); 

	// see if this mesh can be optimized
	OptimizeForRender ();
}

DemoMesh::DemoMesh(NewtonMesh* const mesh)
	:dClassInfo()
	,m_uv(NULL)
	,m_vertex(NULL)
	,m_normal(NULL)
	,m_optilizedDipalyList(0)		
{
	// extract vertex data  from the newton mesh		
	AllocVertexData(NewtonMeshGetPointCount (mesh));
	NewtonMeshGetVertexStreams (mesh, 3 * sizeof (dFloat), (dFloat*) m_vertex, 3 * sizeof (dFloat), (dFloat*) m_normal,	2 * sizeof (dFloat), (dFloat*) m_uv, 2 * sizeof (dFloat), (dFloat*) m_uv);

//	dTree<dScene::dTreeNode*, int> materialMap;
//	for (void* ptr = scene->GetFirstChild(meshNode); ptr; ptr = scene->GetNextChild (meshNode, ptr)) {
//		dScene::dTreeNode* node = scene->GetNodeFromLink(ptr);
//		dNodeInfo* info = scene->GetInfoFromNode(node);
//		if (info->GetTypeId() == dMaterialNodeInfo::GetRttiType()) {
//			dMaterialNodeInfo* const material = (dMaterialNodeInfo*)info;
//			materialMap.Insert(node, material->GetId());
//		}
//	}

	// extract the materials index array for mesh
	void* const meshCookie = NewtonMeshBeginHandle (mesh); 
	for (int handle = NewtonMeshFirstMaterial (mesh, meshCookie); handle != -1; handle = NewtonMeshNextMaterial (mesh, meshCookie, handle)) {
		int textureId = NewtonMeshMaterialGetMaterial (mesh, meshCookie, handle); 
		int indexCount = NewtonMeshMaterialGetIndexCount (mesh, meshCookie, handle); 
		DemoSubMesh* const segment = AddSubMesh();

//		dTree<dScene::dTreeNode*, int>::dTreeNode* matNodeCache = materialMap.Find(materialIndex);
//		if (matNodeCache) {
//			dScene::dTreeNode* const matNode = matNodeCache->GetInfo();
//			dMaterialNodeInfo* const material = (dMaterialNodeInfo*) scene->GetInfoFromNode(matNode);
//			if (material->GetDiffuseTextId() != -1) {
//				dScene::dTreeNode* const node = scene->FindTextureByTextId(matNode, material->GetDiffuseTextId());
//				_ASSERTE (node);
//				dTextureNodeInfo* const texture = (dTextureNodeInfo*)scene->GetInfoFromNode(node);
//
//				segment->m_textureHandle = LoadTexture(texture->GetPathName());
//				strcpy (segment->m_textureName, texture->GetPathName());
//			}


			segment->m_shiness = 1.0f;
			segment->m_ambient = dVector (0.8f, 0.8f, 0.8f, 1.0f);
			segment->m_diffuse = dVector (0.8f, 0.8f, 0.8f, 1.0f);
			segment->m_specular = dVector (0.0f, 0.0f, 0.0f, 1.0f);
//		}

		segment->m_textureHandle = textureId;

		segment->AllocIndexData (indexCount);
		// for 16 bit indices meshes
		//NewtonMeshMaterialGetIndexStreamShort (mesh, meshCookie, handle, (short int*)segment->m_indexes); 

		// for 32 bit indices mesh
		NewtonMeshMaterialGetIndexStream (mesh, meshCookie, handle, (int*)segment->m_indexes); 
	}
	NewtonMeshEndHandle (mesh, meshCookie); 

	// see if this mesh can be optimized
	OptimizeForRender ();
}

DemoMesh::DemoMesh(const DemoMesh& mesh)
	:dClassInfo()
	,dList<DemoSubMesh>()
	,m_uv(NULL)
	,m_vertex(NULL)
	,m_normal(NULL)
	,m_optilizedDipalyList(0)		
{
	AllocVertexData(mesh.m_vertexCount);
	memcpy (m_vertex, mesh.m_vertex, 3 * m_vertexCount * sizeof (float));
	memcpy (m_normal, mesh.m_normal, 3 * m_vertexCount * sizeof (float));
	memcpy (m_uv, mesh.m_uv, 2 * m_vertexCount * sizeof (float));

	for (dListNode* nodes = mesh.GetFirst(); nodes; nodes = nodes->GetNext()) {
		DemoSubMesh* const segment = AddSubMesh();
		DemoSubMesh& srcSegment = nodes->GetInfo();

		segment->AllocIndexData (srcSegment.m_indexCount);
		memcpy (segment->m_indexes, srcSegment.m_indexes, srcSegment.m_indexCount * sizeof (unsigned));

		segment->m_shiness = srcSegment.m_shiness;
		segment->m_ambient = srcSegment.m_ambient;
		segment->m_diffuse = srcSegment.m_diffuse;
		segment->m_specular = srcSegment.m_specular;
		segment->m_textureHandle = srcSegment.m_textureHandle;
		strncpy (segment->m_textureName, srcSegment.m_textureName, sizeof (segment->m_textureName) - 1);
	}

	// see if this mesh can be optimized
	OptimizeForRender ();
}

DemoMesh::DemoMesh(const char* const name, const NewtonCollision* const collision, const char* const texture0, const char* const texture1, const char* const texture2)
	:dClassInfo()
	,dList<DemoSubMesh>()
	,m_uv(NULL)
	,m_vertex(NULL)
	,m_normal(NULL)
	,m_optilizedDipalyList(0)		
{
	// create a helper mesh from the collision collision
	NewtonMesh* const mesh = NewtonMeshCreateFromCollision(collision);

	// apply the vertex normals
	NewtonMeshCalculateVertexNormals(mesh, 30.0f * 3.141592f/180.0f);

	// apply uv projections
	NewtonCollisionInfoRecord info;
	NewtonCollisionGetInfo (collision, &info);
	switch (info.m_collisionType) 
	{
		case SERIALIZE_ID_SPHERE:
		{
			NewtonMeshApplySphericalMapping(mesh, LoadTexture (texture0));
			break;
		}

		case SERIALIZE_ID_CONE:
		case SERIALIZE_ID_CAPSULE:
		case SERIALIZE_ID_CYLINDER:
		case SERIALIZE_ID_TAPEREDCYLINDER:
		case SERIALIZE_ID_TAPEREDCAPSULE:
		case SERIALIZE_ID_CHAMFERCYLINDER:
		{
			//NewtonMeshApplySphericalMapping(mesh, LoadTexture(texture0));
			NewtonMeshApplyCylindricalMapping(mesh, LoadTexture(texture0), LoadTexture(texture1));
			break;
		}

		default:
		{
			int tex0 = LoadTexture(texture0);
			int tex1 = LoadTexture(texture1);
			int tex2 = LoadTexture(texture2);
			NewtonMeshApplyBoxMapping(mesh, tex0, tex1, tex2);
			break;
		}
	}


	// extract vertex data  from the newton mesh		
	int vertexCount = NewtonMeshGetPointCount (mesh); 
	AllocVertexData(vertexCount);
	NewtonMeshGetVertexStreams (mesh, 
								3 * sizeof (dFloat), (dFloat*) m_vertex,
								3 * sizeof (dFloat), (dFloat*) m_normal,
								2 * sizeof (dFloat), (dFloat*) m_uv, 
								2 * sizeof (dFloat), (dFloat*) m_uv);

	// extract the materials index array for mesh
	void* const geometryHandle = NewtonMeshBeginHandle (mesh); 
	for (int handle = NewtonMeshFirstMaterial (mesh, geometryHandle); handle != -1; handle = NewtonMeshNextMaterial (mesh, geometryHandle, handle)) {
		int material = NewtonMeshMaterialGetMaterial (mesh, geometryHandle, handle); 
		int indexCount = NewtonMeshMaterialGetIndexCount (mesh, geometryHandle, handle); 

		DemoSubMesh* const segment = AddSubMesh();

		segment->m_textureHandle = (GLuint)material;

		segment->AllocIndexData (indexCount);
		NewtonMeshMaterialGetIndexStream (mesh, geometryHandle, handle, (int*)segment->m_indexes); 
	}
	NewtonMeshEndHandle (mesh, geometryHandle); 

	// destroy helper mesh
	NewtonMeshDestroy(mesh);

	// optimize this mesh for hardware buffers if possible
	OptimizeForRender ();
}


DemoMesh::DemoMesh(const char* const name, dFloat* const elevation, int size, dFloat cellSize, dFloat texelsDensity, int tileSize)
	:dClassInfo()
	,dList<DemoSubMesh>()
	,m_uv(NULL)
	,m_vertex(NULL)
	,m_normal(NULL)
	,m_optilizedDipalyList(0)		
{
	float* elevationMap[4096];
	dVector* normalMap[4096];
	dVector* const normals = new dVector [size * size];

	for (int i = 0; i < size; i ++) {
		elevationMap[i] = &elevation[i * size];
		normalMap[i] = &normals[i * size];
	}

	memset (normals, 0, (size * size) * sizeof (dVector));
	for (int z = 0; z < size - 1; z ++) {
		for (int x = 0; x < size - 1; x ++) {
			dVector p0 ((x + 0) * cellSize, elevationMap[z + 0][x + 0], (z + 0) * cellSize);
			dVector p1 ((x + 1) * cellSize, elevationMap[z + 0][x + 1], (z + 0) * cellSize);
			dVector p2 ((x + 1) * cellSize, elevationMap[z + 1][x + 1], (z + 1) * cellSize);
			dVector p3 ((x + 0) * cellSize, elevationMap[z + 1][x + 0], (z + 1) * cellSize);

			dVector e10 (p1 - p0);
			dVector e20 (p2 - p0);
			dVector n0 (e20 * e10);
			n0 = n0.Scale ( 1.0f / dSqrt (n0 % n0));
			normalMap [z + 0][x + 0] += n0;
			normalMap [z + 0][x + 1] += n0;
			normalMap [z + 1][x + 1] += n0;

			dVector e30 (p3 - p0);
			dVector n1 (e30 * e20);
			n1 = n1.Scale ( 1.0f / dSqrt (n1 % n1));
			normalMap [z + 0][x + 0] += n1;
			normalMap [z + 1][x + 0] += n1;
			normalMap [z + 1][x + 1] += n1;
		}
	}

	for (int i = 0; i < size * size; i ++) {
		normals[i] = normals[i].Scale (1.0f / sqrtf (normals[i] % normals[i]));
	}
	
	AllocVertexData (size * size);

	dFloat* const vertex = m_vertex;
	dFloat* const normal = m_normal;
	dFloat* const uv = m_uv;

	int index = 0;
	for (int z = 0; z < size; z ++) {
		for (int x = 0; x < size; x ++) {
			vertex[index * 3 + 0] = x * cellSize;
			vertex[index * 3 + 1] = elevationMap[z][x];
			vertex[index * 3 + 2] = z * cellSize;

			normal[index * 3 + 0] = normalMap[z][x].m_x;
			normal[index * 3 + 1] = normalMap[z][x].m_y;
			normal[index * 3 + 2] = normalMap[z][x].m_z;

			uv[index * 2 + 0] = x * texelsDensity;
			uv[index * 2 + 1] = z * texelsDensity;
			index ++;
		}
	}

	int segmentsCount = (size - 1) / tileSize;
	for (int z0 = 0; z0 < segmentsCount; z0 ++) {
		int z = z0 * tileSize;
		for (int x0 = 0; x0 < segmentsCount; x0 ++ ) {
			int x = x0 * tileSize;

			DemoSubMesh* const tile = AddSubMesh();
			tile->AllocIndexData (tileSize * tileSize * 6);
			unsigned* const indexes = tile->m_indexes;

			strcpy (tile->m_textureName, "grassanddirt.tga");
			tile->m_textureHandle = LoadTexture(tile->m_textureName);

			int index = 0;
			int x1 = x + tileSize;
			int z1 = z + tileSize;
			for (int z0 = z; z0 < z1; z0 ++) {
				for (int x0 = x; x0 < x1; x0 ++) {
					int i0 = x0 + 0 + (z0 + 0) * size;
					int i1 = x0 + 1 + (z0 + 0) * size;
					int i2 = x0 + 1 + (z0 + 1) * size;
					int i3 = x0 + 0 + (z0 + 1) * size;

					indexes[index + 0] = i0;
					indexes[index + 1] = i2;
					indexes[index + 2] = i1;

					indexes[index + 3] = i0;
					indexes[index + 4] = i3;
					indexes[index + 5] = i2;
					index += 6;
				}
			}

			index*=1;
		}
	}
	delete[] normals; 
	OptimizeForRender();
}



DemoMesh::~DemoMesh()
{
	if (m_vertex) {
		free (m_vertex);
		free (m_normal);
		free (m_uv);
		ResetOptimization();
	}
}

void DemoMesh::GetName (char* const nameOut) const
{
	strcpy (nameOut, m_name);
}


void  DemoMesh::OptimizeForRender()
{
	// first make sure the previous optimization is removed
	ResetOptimization();

#ifdef USING_DISPLAY_LIST
	m_optilizedDipalyList = glGenLists(1);

	glNewList(m_optilizedDipalyList, GL_COMPILE);

	//glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );

	for (dListNode* nodes = GetFirst(); nodes; nodes = nodes->GetNext()) {
		DemoSubMesh& segment = nodes->GetInfo();
		segment.OptimizeForRender(this);
	}
	glEndList();
#endif
}

void  DemoMesh::ResetOptimization()
{
	if (m_optilizedDipalyList) {
		glDeleteLists(m_optilizedDipalyList, 1);
		m_optilizedDipalyList = 0;
	}
}



void DemoMesh::GetTextureName (const DemoSubMesh* const subMesh, char* const nameOut) const
{
	strcpy (nameOut, subMesh->m_textureName);
}


void DemoMesh::AllocVertexData (int vertexCount)
{
	m_vertexCount = vertexCount;

	m_vertex = (dFloat*) malloc (3 * m_vertexCount * sizeof (dFloat)); 
	m_normal = (dFloat*) malloc (3 * m_vertexCount * sizeof (dFloat)); 
	m_uv = (dFloat*) malloc (2 * m_vertexCount * sizeof (dFloat)); 
	memset (m_uv, 0, 2 * m_vertexCount * sizeof (dFloat));
}

DemoSubMesh* DemoMesh::AddSubMesh()
{
	return &Append()->GetInfo();
}


void DemoMesh::Render ()
{
	
	if (m_optilizedDipalyList) {
//		glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
		glCallList(m_optilizedDipalyList);
	} else {
		glEnableClientState (GL_VERTEX_ARRAY);
		glEnableClientState (GL_NORMAL_ARRAY);
		glEnableClientState (GL_TEXTURE_COORD_ARRAY);

		glVertexPointer (3, GL_FLOAT, 0, m_vertex);
		glNormalPointer (GL_FLOAT, 0, m_normal);
		glTexCoordPointer (2, GL_FLOAT, 0, m_uv);

//	glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );

		for (dListNode* nodes = GetFirst(); nodes; nodes = nodes->GetNext()) {
			DemoSubMesh& segment = nodes->GetInfo();
			segment.Render();
		}
		glDisableClientState(GL_VERTEX_ARRAY);	// disable vertex arrays
		glDisableClientState(GL_NORMAL_ARRAY);	// disable normal arrays
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);	// disable normal arrays
	}
}

void DemoMesh::RenderNormals ()
{
	glDisable (GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);

	glColor3f(1.0f, 1.0f, 1.0f);

	float length = 0.1f;
	glBegin(GL_LINES);

	for (int i = 0; i < m_vertexCount; i ++) {
		glVertex3f (m_vertex[i * 3 + 0], m_vertex[i * 3 + 1], m_vertex[i * 3 + 2]);
		glVertex3f (m_vertex[i * 3 + 0] + m_normal[i * 3 + 0] * length, m_vertex[i * 3 + 1] + m_normal[i * 3 + 1] * length, m_vertex[i * 3 + 2] + m_normal[i * 3 + 2] * length);
	}

	glEnd();
}