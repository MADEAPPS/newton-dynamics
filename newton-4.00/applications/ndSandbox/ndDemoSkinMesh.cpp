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
#include "ndDemoCamera.h"
#include "ndDemoEntity.h"
#include "ndDemoSkinMesh.h"
#include "ndTargaToOpenGl.h"
#include "ndDemoEntityManager.h"

/*
ndDemoSkinMesh::ndDemoSkinMesh(dScene* const scene, ndDemoEntity* const owner, dScene::dNode* const meshNode, const dTree<ndDemoEntity*, dScene::dNode*>& boneMap, const ndShaderPrograms& shaderCache)
	:ndDemoMeshInterface()
	,m_mesh((ndDemoMesh*)owner->GetMesh()->Clone(nullptr))
	,m_entity(owner)
	,m_bindingMatrixArray(nullptr)
	,m_nodeCount(0)
	,m_shader(shaderCache.m_skinningDiffuseEffect)
{
	ndDemoEntity* root = owner;
	while (root->GetParent()) {
		root = root->GetParent();
	}

	dMeshNodeInfo* const meshInfo = (dMeshNodeInfo*)scene->GetInfoFromNode(meshNode);
	dAssert (meshInfo->GetTypeId() == dMeshNodeInfo::GetRttiType());

	dTree<const dGeometryNodeSkinClusterInfo*, ndDemoEntity*> nodeClusterEnumerator;
	dTree<ndDemoEntity*, dScene::dNode*>::Iterator iter (boneMap);
	for (iter.Begin(); iter; iter++) {
		dScene::dNode* const boneNode = iter.GetKey();
		const dGeometryNodeSkinClusterInfo* const cluster = FindSkinModifier(scene, boneNode);
		if (cluster) {
			ndDemoEntity* const boneEntity = iter.GetNode()->GetInfo();
			boneEntity->SetMatrixUsafe (cluster->m_basePoseMatrix, cluster->m_basePoseMatrix.m_posit);
			boneEntity->SetMatrixUsafe (cluster->m_basePoseMatrix, cluster->m_basePoseMatrix.m_posit);
			nodeClusterEnumerator.Insert(cluster, boneEntity);
		}
	}

	dInt32 stack = 1;
	dInt32 entityCount = 0;
	ndDemoEntity* pool[32];
	dMatrix parentMatrix[32];

//	dMatrix* const bindMatrix = dAlloca (dMatrix, 2048);
//	ndDemoEntity** const entityArray = dAlloca (ndDemoEntity*, 2048);
	dArray<dMatrix> bindMatrix(2048);
	dArray<ndDemoEntity*> entityArray(2048);

	pool[0] = root;
	parentMatrix[0] = dGetIdentityMatrix();
	dMatrix shapeBindMatrix(m_entity->GetMeshMatrix() * m_entity->CalculateGlobalMatrix());

	const dInt32 boneCount = boneMap.GetCount() + 1024;
//	dInt32* const boneClusterRemapIndex = dAlloca (dInt32, boneCount);
	dArray<dInt32> boneClusterRemapIndex (boneCount);
	memset (&boneClusterRemapIndex[0], -1, boneCount * sizeof (dInt32));
	while (stack) {
		stack--;
		ndDemoEntity* const entity = pool[stack];
		dMatrix boneMatrix(entity->GetCurrentMatrix() * parentMatrix[stack]);

		entityArray[entityCount] = entity;
		bindMatrix[entityCount] = shapeBindMatrix * boneMatrix.Inverse();
		dAssert (entityCount < 2048);

		dTree<const dGeometryNodeSkinClusterInfo*, ndDemoEntity*>::dNode* const clusterNode = nodeClusterEnumerator.Find(entity);
		if (clusterNode) {
			const dGeometryNodeSkinClusterInfo* const cluster = nodeClusterEnumerator.Find(entity)->GetInfo();
			dAssert (boneClusterRemapIndex[cluster->GetNodeID()] == -1);
			boneClusterRemapIndex[cluster->GetNodeID()] = entityCount;
		}
		entityCount++;
		
		for (ndDemoEntity* node = entity->GetChild(); node; node = node->GetSibling()) {
			pool[stack] = node;
			parentMatrix[stack] = boneMatrix;
			stack++;
		}
	}

	m_nodeCount = entityCount;
	m_bindingMatrixArray = new dMatrix[m_nodeCount];
	memcpy(m_bindingMatrixArray, &bindMatrix[0], entityCount * sizeof (dMatrix));

//	dVector* const weight = dAlloca (dVector, m_mesh->m_vertexCount);
//	dWeightBoneIndex* const skinBone = dAlloca(dWeightBoneIndex, m_mesh->m_vertexCount);
	dArray<dVector> weight (m_mesh->m_vertexCount);
	dArray<dWeightBoneIndex> skinBone (m_mesh->m_vertexCount);
	memset (&weight[0], 0, m_mesh->m_vertexCount * sizeof (dVector));
	memset (&skinBone[0], -1, m_mesh->m_vertexCount * sizeof (dWeightBoneIndex));

	dInt32 vCount = 0;
	for (iter.Begin(); iter; iter++) {
		dScene::dNode* const boneNode = iter.GetKey();
		const dGeometryNodeSkinClusterInfo* const cluster = FindSkinModifier(scene, boneNode);
		if (cluster) {
			dInt32 boneIndex = boneClusterRemapIndex[cluster->GetNodeID()];
			dAssert (boneIndex != -1);
			for (dInt32 i = 0; i < cluster->m_vertexIndex.GetSize(); i ++) {
				dInt32 vertexIndex = cluster->m_vertexIndex[i];

				vCount = dMax (vertexIndex + 1, vCount);
				dFloat32 vertexWeight = cluster->m_vertexWeight[i];
				if (vertexWeight >= weight[vertexIndex][3]) {
					weight[vertexIndex][3] = vertexWeight;
					skinBone[vertexIndex].m_boneIndex[3] = boneIndex;

					for (dInt32 j = 2; j >= 0; j --) {
						if (weight[vertexIndex][j] < weight[vertexIndex][j + 1]) {
							dSwap (weight[vertexIndex][j], weight[vertexIndex][j + 1]);
							dSwap (skinBone[vertexIndex].m_boneIndex[j], skinBone[vertexIndex].m_boneIndex[j + 1]);
						}
					}
				}
			}
		}
	}

	dInt32 weightcount = 0;
	const dInt32* const indexToPointMap = meshInfo->GetIndexToVertexMap();
	const dInt32 vertexBaseCount = NewtonMeshGetVertexBaseCount(meshInfo->GetMesh());

	for (dInt32 i = 0; i < vertexBaseCount; i ++) {

		dVector w (weight[i]);
		dFloat32 invMag = w.m_x + w.m_y + w.m_z + w.m_w;
		dAssert (invMag > 0.0f);
		invMag = 1.0f/invMag;
		weight[i].m_x = w.m_x * invMag;
		weight[i].m_y = w.m_y * invMag;
		weight[i].m_z = w.m_z * invMag;
		weight[i].m_w = w.m_w * invMag;

		dAssert (skinBone[i].m_boneIndex[0] != -1);
		for (dInt32 j = 0; j < 4; j++) {
			if (skinBone[i].m_boneIndex[j] != -1) {
				weightcount = dMax(weightcount, j + 1);
			} else {
				skinBone[i].m_boneIndex[j] = 0;
			}
		}
	}

//	dVector* const pointWeights = dAlloca(dVector, m_mesh->m_vertexCount);
//	dWeightBoneIndex* const pointSkinBone = dAlloca(dWeightBoneIndex, m_mesh->m_vertexCount);
	dArray<dVector> pointWeights (m_mesh->m_vertexCount);
	dArray<dWeightBoneIndex> pointSkinBone(m_mesh->m_vertexCount);
	memset(&pointSkinBone[0], 0, m_mesh->m_vertexCount * sizeof(dWeightBoneIndex));

	dList<dInt32> pendingVertices;
	for (dInt32 i = 0; i < m_mesh->m_vertexCount; i ++) {
		dInt32 index = indexToPointMap[i];

		dAssert (index >= 0);
		//dAssert (index < vCount);
		if (index < vCount) {
			pointWeights[i] = weight[index];
			pointSkinBone[i] = skinBone[index];
		} else {
			pendingVertices.Append(i);
		}
	}

	for (dList<dInt32>::dNode* ptr = pendingVertices.GetFirst(); ptr; ptr = ptr->GetNext()) {
		dInt32 i = ptr->GetInfo();
		dVector p (m_mesh->m_vertex[i * 3 + 0], m_mesh->m_vertex[i * 3 + 1], m_mesh->m_vertex[i * 3 + 2], 0.0f);
		for (dInt32 j = 0; j < m_mesh->m_vertexCount; j ++) {
			if (i != j) {
				dVector q (m_mesh->m_vertex[j * 3 + 0], m_mesh->m_vertex[j * 3 + 1], m_mesh->m_vertex[j * 3 + 2], 0.0f);
				dVector diff (q - p);
				if (diff.DotProduct3(diff) < 1.0e-6f) {
					pointWeights[i] = pointWeights[j];
					pointSkinBone[i] = pointSkinBone[j];
					break;
				}
			}
		}
	}

	for (ndDemoMesh::dNode* node = m_mesh->GetFirst(); node; node = node->GetNext()) {
		ndDemoSubMesh& segment = node->GetInfo();
		segment.m_shader = m_shader;
	}
	m_mesh->ResetOptimization();

	dInt32 count = CalculateMatrixPalette(&bindMatrix[0]);
	GLfloat* const glMatrixPallete = dAlloca (GLfloat, 16 * count);
	ConvertToGlMatrix(count, &bindMatrix[0], glMatrixPallete);

	glUseProgram(m_shader);
	dInt32 matrixPalette = glGetUniformLocation(m_shader, "matrixPallete");
	glUniformMatrix4fv(matrixPalette, count, GL_FALSE, glMatrixPallete);
	dAssert (glGetError() == GL_NO_ERROR);

	m_mesh->m_optimizedOpaqueDiplayList = glGenLists(1);
	glNewList(m_mesh->m_optimizedOpaqueDiplayList, GL_COMPILE);
	for (ndDemoMesh::dNode* node = m_mesh->GetFirst(); node; node = node->GetNext()) {
		const ndDemoSubMesh& segment = node->GetInfo();
		OptimizeForRender(segment, &pointWeights[0], &pointSkinBone[0]);
	}
	glEndList();
}

ndDemoSkinMesh::ndDemoSkinMesh(const ndDemoSkinMesh& clone, ndDemoEntity* const owner)
	:ndDemoMeshInterface(clone)
	,m_mesh((ndDemoMesh*)clone.m_mesh->Clone(nullptr))
	,m_entity(owner)
//	,m_bindingMatrixArray(new dMatrix[clone.m_nodeCount])
	,m_nodeCount(clone.m_nodeCount)
	,m_shader(clone.m_shader)
{
	dAssert(0);
	//memcpy(m_bindingMatrixArray, clone.m_bindingMatrixArray, clone.m_nodeCount * sizeof(dMatrix));
}
*/

ndDemoSkinMesh::~ndDemoSkinMesh()
{
	//if (m_bindingMatrixArray) 
	//{
	//	delete[] m_bindingMatrixArray; 
	//}
}

//ndDemoMeshInterface* ndDemoSkinMesh::Clone(ndDemoEntity* const owner)
ndDemoMeshInterface* ndDemoSkinMesh::Clone(ndDemoEntity* const)
{
	dAssert(0);
	//return (ndDemoSkinMesh*)new ndDemoSkinMesh(*this, owner);
	return nullptr;
}

/*
//void ndDemoSkinMesh::OptimizeForRender(const ndDemoSubMesh& segment, const dVector* const pointWeights, const dWeightBoneIndex* const pointSkinBone) const
void ndDemoSkinMesh::OptimizeForRender(const ndDemoSubMesh&, const dVector* const, const dWeightBoneIndex* const) const
{
	dAssert(0);

	glUniform1i(glGetUniformLocation(segment.m_shader, "texture"), 0);

	glMaterialParam(GL_FRONT, GL_AMBIENT, &segment.m_ambient.m_x);
	glMaterialParam(GL_FRONT, GL_DIFFUSE, &segment.m_diffuse.m_x);
	glMaterialParam(GL_FRONT, GL_SPECULAR, &segment.m_specular.m_x);
	glMaterialf(GL_FRONT, GL_SHININESS, GLfloat(segment.m_shiness));
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	if (segment.m_textureHandle) {
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, GLuint(segment.m_textureHandle));
	} else {
		glDisable(GL_TEXTURE_2D);
	}

	glBindAttribLocation(m_shader, 10, "boneIndices");
	glBindAttribLocation(m_shader, 11, "boneWeights");

	dInt32 boneIndices = glGetAttribLocation(m_shader, "boneIndices");
	dInt32 boneWeights = glGetAttribLocation(m_shader, "boneWeights");

	glBegin(GL_TRIANGLES);
	const dFloat32* const uv = m_mesh->m_uv;
	const dFloat32* const normal = m_mesh->m_normal;
	const dFloat32* const vertex = m_mesh->m_vertex;
	for (dInt32 i = 0; i < segment.m_indexCount; i++) 
	{
		dInt32 index = segment.m_indexes[i];

		const dVector& weights = pointWeights[index];
		const dWeightBoneIndex& boneIndex = pointSkinBone[index];
		glTexCoord2f(GLfloat(uv[index * 2 + 0]), GLfloat(uv[index * 2 + 1]));
		glVertexAttrib4f(boneWeights, GLfloat(weights[0]), GLfloat(weights[1]), GLfloat(weights[2]), GLfloat(weights[3]));
		glVertexAttrib4f(boneIndices, GLfloat(boneIndex.m_boneIndex[0]), GLfloat(boneIndex.m_boneIndex[1]), GLfloat(boneIndex.m_boneIndex[2]), GLfloat(boneIndex.m_boneIndex[3]));
		glNormal3f(GLfloat(normal[index * 3 + 0]), GLfloat(normal[index * 3 + 1]), GLfloat(normal[index * 3 + 2]));
		glVertex3f(GLfloat(vertex[index * 3 + 0]), GLfloat(vertex[index * 3 + 1]), GLfloat(vertex[index * 3 + 2]));
	}
	glEnd();
	glUseProgram(0);
}
*/

dInt32 ndDemoSkinMesh::CalculateMatrixPalette(dMatrix* const bindMatrix) const
{
	dInt32 stack = 1;
	ndDemoEntity* pool[32];
	dMatrix parentMatrix[32];

	ndDemoEntity* root = m_entity;
	while (root->GetParent()) 
	{
		root = root->GetParent();
	}

	dInt32 count = 0;
	pool[0] = root;
	parentMatrix[0] = dGetIdentityMatrix();
	dMatrix shapeBindMatrix((m_entity->GetMeshMatrix() * m_entity->CalculateGlobalMatrix()).Inverse());
	while (stack) {
		stack--;
		ndDemoEntity* const entity = pool[stack];
		dMatrix boneMatrix(entity->GetCurrentMatrix() * parentMatrix[stack]);
		bindMatrix[count] = m_bindingMatrixArray[count] * boneMatrix * shapeBindMatrix;

		count++;
		dAssert(count <= 128);
		dAssert(count <= m_nodeCount);
		for (ndDemoEntity* node = entity->GetChild(); node; node = node->GetSibling()) {
			pool[stack] = node;
			parentMatrix[stack] = boneMatrix;
			stack++;
		}
	}

	return count;
}

void ndDemoSkinMesh::ConvertToGlMatrix(dInt32 count, const dMatrix* const bindMatrix, GLfloat* const glMatrices) const
{
	for (dInt32 i = 0; i < count; i++) 
	{
		const dMatrix& src = bindMatrix[i];
		GLfloat* dst = &glMatrices[i * 16];
		for (dInt32 j = 0; j < 4; j++) 
		{
			for (dInt32 k = 0; k < 4; k++) 
			{
				dst[j * 4 + k] = GLfloat (src[j][k]);
			}
		}
	}
}

/*
dGeometryNodeSkinClusterInfo* ndDemoSkinMesh::FindSkinModifier(dScene* const scene, dScene::dNode* const node) const
{
	for (void* modifierChild = scene->GetFirstChildLink(node); modifierChild; modifierChild = scene->GetNextChildLink(node, modifierChild)) {
		dScene::dNode* const modifierNode = scene->GetNodeFromLink(modifierChild);
		dGeometryNodeSkinClusterInfo* const modifierInfo = (dGeometryNodeSkinClusterInfo*)scene->GetInfoFromNode(modifierNode);
		if (modifierInfo->GetTypeId() == dGeometryNodeSkinClusterInfo::GetRttiType()) {
			return modifierInfo;
		}
	}
	return nullptr;
}
*/

void ndDemoSkinMesh::Render(ndDemoEntityManager* const scene, const dMatrix& modelMatrix)
//void ndDemoSkinMesh::Render(ndDemoEntityManager* const, const dMatrix&)
{
	//dAssert(0);
	//dMatrix* const bindMatrix = dAlloca(dMatrix, m_nodeCount);
	//dInt32 count = CalculateMatrixPalette(bindMatrix);
	//GLfloat* const glMatrixPallete = dAlloca(GLfloat, 16 * count);
	//ConvertToGlMatrix(count, bindMatrix, glMatrixPallete);
	//
	//glUseProgram(m_shader);
	//dInt32 matrixPalette = glGetUniformLocation(m_shader, "matrixPallete");
	//glUniformMatrix4fv(matrixPalette, count, GL_FALSE, glMatrixPallete);
	//glCallList(m_mesh->m_optimizedOpaqueDiplayList);
	//m_mesh->Render(scene, modelMatrix);
	ndDemoMesh::Render(scene, modelMatrix);
}

ndDemoSkinMesh::ndDemoSkinMesh(ndDemoEntity* const owner, ndMeshEffect* const meshNode, const ndShaderPrograms& shaderCache)
	:ndDemoMesh(owner->GetName().GetStr())
	,m_entity(owner)
	,m_bindingMatrixArray()
{
	//m_name = owner->GetName();
	m_shader = shaderCache.m_diffuseEffect;

	ndDemoEntity* root = owner;
	while (root->GetParent()) 
	{
		root = root->GetParent();
	}

	dInt32 stack = 1;
	ndDemoEntity* pool[128];
	dMatrix parentMatrix[128];
	dArray<dMatrix> bindMatrixArray;
	dArray<ndDemoEntity*> entityArray;

	pool[0] = root;
	parentMatrix[0] = dGetIdentityMatrix();
	dMatrix shapeBindMatrix(m_entity->GetMeshMatrix() * m_entity->CalculateGlobalMatrix());

	dTree<dInt32, dString> boneClusterRemapIndex;
	const ndMeshEffect::dClusterMap& clusterMap = meshNode->GetCluster();
	
	while (stack) 
	{
		stack--;
		ndDemoEntity* const entity = pool[stack];
		const dMatrix boneMatrix(entity->GetCurrentMatrix() * parentMatrix[stack]);
	
		const dMatrix bindMatrix(shapeBindMatrix * boneMatrix.Inverse());
		entityArray.PushBack(entity);
		bindMatrixArray.PushBack(bindMatrix);
	
		ndMeshEffect::dClusterMap::dNode* const clusterNode = clusterMap.Find(entity->GetName());
		if (clusterNode) 
		{
			boneClusterRemapIndex.Insert(entityArray.GetCount() - 1, entity->GetName());
		}
	
		for (ndDemoEntity* node = entity->GetChild(); node; node = node->GetSibling()) 
		{
			pool[stack] = node;
			parentMatrix[stack] = boneMatrix;
			stack++;
		}
	}
	
	m_nodeCount = entityArray.GetCount();
	m_bindingMatrixArray.SetCount(m_nodeCount);
	memcpy(&m_bindingMatrixArray[0], &bindMatrixArray[0], m_nodeCount * sizeof(dMatrix));
	
	dArray<dVector> weight;
	dArray<dWeightBoneIndex> skinBone;
	weight.SetCount(meshNode->GetVertexCount());
	skinBone.SetCount(meshNode->GetVertexCount());
	memset(&weight[0], 0, meshNode->GetVertexCount() * sizeof(dVector));
	memset(&skinBone[0], -1, meshNode->GetVertexCount() * sizeof(dWeightBoneIndex));
	
	dInt32 vCount = 0;
	ndMeshEffect::dClusterMap::Iterator iter(clusterMap);
	for (iter.Begin(); iter; iter++) 
	{
		const ndMeshEffect::dVertexCluster* const cluster = &iter.GetNode()->GetInfo();
		dInt32 boneIndex = boneClusterRemapIndex.Find(iter.GetKey())->GetInfo();
		for (dInt32 i = 0; i < cluster->m_vertexIndex.GetCount(); i++) 
		{
			dInt32 vertexIndex = cluster->m_vertexIndex[i];
			vCount = dMax(vertexIndex + 1, vCount);
			dFloat32 vertexWeight = cluster->m_vertexWeigh[i];
			if (vertexWeight >= weight[vertexIndex][3]) 
			{
				weight[vertexIndex][3] = vertexWeight;
				skinBone[vertexIndex].m_boneIndex[3] = boneIndex;
			
				for (dInt32 j = 2; j >= 0; j--) 
				{
					if (weight[vertexIndex][j] < weight[vertexIndex][j + 1]) 
					{
						dSwap(weight[vertexIndex][j], weight[vertexIndex][j + 1]);
						dSwap(skinBone[vertexIndex].m_boneIndex[j], skinBone[vertexIndex].m_boneIndex[j + 1]);
					}
				}
			}
		}
	}
	
	dInt32 weightcount = 0;
	//const dInt32* const indexToPointMap = meshInfo->GetIndexToVertexMap();
	//const dInt32 vertexBaseCount = NewtonMeshGetVertexBaseCount(meshInfo->GetMesh());
	
	for (dInt32 i = 0; i < weight.GetCount(); i++)
	{
		dVector w(weight[i]);
		dFloat32 invMag = w.m_x + w.m_y + w.m_z + w.m_w;
		dAssert(invMag > 0.0f);
		invMag = 1.0f / invMag;
		weight[i].m_x = w.m_x * invMag;
		weight[i].m_y = w.m_y * invMag;
		weight[i].m_z = w.m_z * invMag;
		weight[i].m_w = w.m_w * invMag;
	
		dAssert(skinBone[i].m_boneIndex[0] != -1);
		for (dInt32 j = 0; j < 4; j++) 
		{
			if (skinBone[i].m_boneIndex[j] != -1) 
			{
				weightcount = dMax(weightcount, j + 1);
			}
			else 
			{
				skinBone[i].m_boneIndex[j] = 0;
			}
		}
	}

	// extract the materials index array for mesh
	ndIndexArray* const geometryHandle = meshNode->MaterialGeometryBegin();

	// extract vertex data  from the newton mesh		
	dInt32 indexCount = 0;
	dInt32 vertexCount = meshNode->GetPropertiesCount();
	for (dInt32 handle = meshNode->GetFirstMaterial(geometryHandle); handle != -1; handle = meshNode->GetNextMaterial(geometryHandle, handle))
	{
		indexCount += meshNode->GetMaterialIndexCount(geometryHandle, handle);
	}
	
	struct dTmpData
	{
		dFloat32 m_posit[3];
		dFloat32 m_normal[3];
		dFloat32 m_uv[2];
	};
	
	dArray<dTmpData> tmp;
	dArray<dInt32> indices;
	dArray<dInt32> vertexIndex;
	dArray<glPositionNormalUV> points;
	
	indices.SetCount(indexCount);
	points.SetCount(vertexCount);
	tmp.SetCount(vertexCount);
	vertexIndex.SetCount(vertexCount);
	
	meshNode->GetVertexChannel(sizeof(dTmpData), &tmp[0].m_posit[0]);
	meshNode->GetNormalChannel(sizeof(dTmpData), &tmp[0].m_normal[0]);
	meshNode->GetUV0Channel(sizeof(dTmpData), &tmp[0].m_uv[0]);
	meshNode->GetVertexIndexChannel(&vertexIndex[0]);

	for (dInt32 i = 0; i < vertexCount; i++)
	{
		points[i].m_posit.m_x = GLfloat(tmp[i].m_posit[0]);
		points[i].m_posit.m_y = GLfloat(tmp[i].m_posit[1]);
		points[i].m_posit.m_z = GLfloat(tmp[i].m_posit[2]);
		points[i].m_normal.m_x = GLfloat(tmp[i].m_normal[0]);
		points[i].m_normal.m_y = GLfloat(tmp[i].m_normal[1]);
		points[i].m_normal.m_z = GLfloat(tmp[i].m_normal[2]);
		points[i].m_uv.m_u = GLfloat(tmp[i].m_uv[0]);
		points[i].m_uv.m_v = GLfloat(tmp[i].m_uv[1]);
	}
	
	dInt32 segmentStart = 0;
	bool hasTransparency = false;
	const dArray<ndMeshEffect::dMaterial>& materialArray = meshNode->GetMaterials();
	for (dInt32 handle = meshNode->GetFirstMaterial(geometryHandle); handle != -1; handle = meshNode->GetNextMaterial(geometryHandle, handle))
	{
		dInt32 materialIndex = meshNode->GetMaterialID(geometryHandle, handle);
		ndDemoSubMesh* const segment = AddSubMesh();
	
		const ndMeshEffect::dMaterial& material = materialArray[materialIndex];
		segment->m_material.m_ambient = glVector4(material.m_ambient);
		segment->m_material.m_diffuse = glVector4(material.m_diffuse);
		segment->m_material.m_specular = glVector4(material.m_specular);
		segment->m_material.m_opacity = GLfloat(material.m_opacity);
		segment->m_material.m_shiness = GLfloat(material.m_shiness);
		strcpy(segment->m_material.m_textureName, material.m_textureName);
		//segment->m_material.m_textureHandle = (GLuint)material.m_textureHandle;
		segment->m_material.m_textureHandle = LoadTexture(material.m_textureName);
		segment->SetOpacity(material.m_opacity);
		hasTransparency = hasTransparency | segment->m_hasTranparency;
	
		segment->m_indexCount = meshNode->GetMaterialIndexCount(geometryHandle, handle);
	
		segment->m_segmentStart = segmentStart;
		meshNode->GetMaterialGetIndexStream(geometryHandle, handle, &indices[segmentStart]);
		segmentStart += segment->m_indexCount;
	}
	meshNode->MaterialGeometryEnd(geometryHandle);
	m_hasTransparency = hasTransparency;



	//dArray<dVector> pointWeights(m_mesh->m_vertexCount);
	//dArray<dWeightBoneIndex> pointSkinBone(m_mesh->m_vertexCount);
	//memset(&pointSkinBone[0], 0, m_mesh->m_vertexCount * sizeof(dWeightBoneIndex));
	//
	//dList<dInt32> pendingVertices;
	//for (dInt32 i = 0; i < m_mesh->m_vertexCount; i++) {
	//	dInt32 index = indexToPointMap[i];
	//
	//	dAssert(index >= 0);
	//	//dAssert (index < vCount);
	//	if (index < vCount) {
	//		pointWeights[i] = weight[index];
	//		pointSkinBone[i] = skinBone[index];
	//	}
	//	else {
	//		pendingVertices.Append(i);
	//	}
	//}
	//
	//for (dList<dInt32>::dNode* ptr = pendingVertices.GetFirst(); ptr; ptr = ptr->GetNext()) {
	//	dInt32 i = ptr->GetInfo();
	//	dVector p(m_mesh->m_vertex[i * 3 + 0], m_mesh->m_vertex[i * 3 + 1], m_mesh->m_vertex[i * 3 + 2], 0.0f);
	//	for (dInt32 j = 0; j < m_mesh->m_vertexCount; j++) {
	//		if (i != j) {
	//			dVector q(m_mesh->m_vertex[j * 3 + 0], m_mesh->m_vertex[j * 3 + 1], m_mesh->m_vertex[j * 3 + 2], 0.0f);
	//			dVector diff(q - p);
	//			if (diff.DotProduct3(diff) < 1.0e-6f) {
	//				pointWeights[i] = pointWeights[j];
	//				pointSkinBone[i] = pointSkinBone[j];
	//				break;
	//			}
	//		}
	//	}
	//}
	//
	//for (ndDemoMesh::dNode* node = m_mesh->GetFirst(); node; node = node->GetNext()) {
	//	ndDemoSubMesh& segment = node->GetInfo();
	//	segment.m_shader = m_shader;
	//}
	//m_mesh->ResetOptimization();
	//


	//dInt32 count = CalculateMatrixPalette(&bindMatrix[0]);
	//GLfloat* const glMatrixPallete = dAlloca(GLfloat, 16 * count);
	//ConvertToGlMatrix(count, &bindMatrix[0], glMatrixPallete);
	//
	//glUseProgram(m_shader);
	//dInt32 matrixPalette = glGetUniformLocation(m_shader, "matrixPallete");
	//glUniformMatrix4fv(matrixPalette, count, GL_FALSE, glMatrixPallete);
	//dAssert(glGetError() == GL_NO_ERROR);
	//
	//m_mesh->m_optimizedOpaqueDiplayList = glGenLists(1);
	//glNewList(m_mesh->m_optimizedOpaqueDiplayList, GL_COMPILE);
	//for (ndDemoMesh::dNode* node = m_mesh->GetFirst(); node; node = node->GetNext()) {
	//	const ndDemoSubMesh& segment = node->GetInfo();
	//	OptimizeForRender(segment, &pointWeights[0], &pointSkinBone[0]);
	//}
	//glEndList();

	// optimize this mesh for hardware buffers if possible
	OptimizeForRender(&points[0], vertexCount, &indices[0], indexCount);
}