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

#include "ndRenderStdafx.h"
#include "ndRender.h"
#include "ndRenderContext.h"
#include "ndRenderTexture.h"
#include "ndRenderPrimitive.h"
#include "ndRenderOpenGlUtil.h"
#include "ndRenderShaderCache.h"
#include "ndRenderSceneCamera.h"
#include "ndRenderTextureImage.h"
#include "ndRenderPassEnvironment.h"
#include "ndRenderPrimitiveImplement.h"
#include "ndRenderPassShadowsImplement.h"

#define ND_MAX_SKINNED_BONES	128

ndRenderPrimitiveImplement::ndRenderPrimitiveImplement(ndRenderPrimitive* const owner, const ndRenderPrimitive::ndDescriptor& descriptor)
	:ndContainersFreeListAlloc<ndRenderPrimitiveImplement>()
	,m_owner(owner)
	,m_context(*descriptor.m_render->m_context)
	,m_skinSceneNode(nullptr)
	,m_indexCount(0)
	,m_vertexCount(0)
	,m_vertexSize(0)
	,m_indexBuffer(0)
	,m_vertexBuffer(0)
	,m_vertextArrayBuffer(0)
	,m_instanceRenderMatrixPalleteBuffer(0)
{
	if (*descriptor.m_collision)
	{
		BuildFromCollisionShape(descriptor);
	}
	else
	{
		ndAssert(*descriptor.m_meshNode);
		BuildFromMesh(descriptor);
	}
	InitShaderBlocks();
}

ndRenderPrimitiveImplement::ndRenderPrimitiveImplement(
	const ndRenderPrimitiveImplement& src)
	:ndContainersFreeListAlloc<ndRenderPrimitiveImplement>()
	,m_owner(nullptr)
	,m_context(src.m_context)
	,m_skinSceneNode(nullptr)
	,m_indexCount(0)
	,m_vertexCount(0)
	,m_vertexSize(0)
	,m_indexBuffer(0)
	,m_vertexBuffer(0)
	,m_vertextArrayBuffer(0)
	,m_instanceRenderMatrixPalleteBuffer(0)
{
	ndAssert(0);
}

ndRenderPrimitiveImplement::ndRenderPrimitiveImplement(
	const ndRenderPrimitiveImplement& src, const ndRenderSceneNode* const skeleton)
	:ndContainersFreeListAlloc<ndRenderPrimitiveImplement>()
	,m_owner(nullptr)
	,m_context(src.m_context)
	,m_skinSceneNode(nullptr)
	,m_indexCount(src.m_indexCount)
	,m_vertexCount(src.m_vertexCount)
	,m_vertexSize(src.m_vertexSize)
	,m_indexBuffer(0)
	,m_vertexBuffer(0)
	,m_vertextArrayBuffer(0)
	,m_instanceRenderMatrixPalleteBuffer(0)
{
	if (src.m_skeleton.GetCount())
	{
		m_skinSceneNode = (ndRenderSceneNode*) skeleton;
	}
	
	if (src.m_instanceRenderMatrixPalleteBuffer)
	{
		ndAssert(src.m_skeleton.GetCount());
		glGenBuffers(1, &m_instanceRenderMatrixPalleteBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, m_instanceRenderMatrixPalleteBuffer);
		glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(src.m_bindingSkinMatrixArray.GetCount() * sizeof(glMatrix)), &src.m_bindingSkinMatrixArray[0], GL_STATIC_DRAW);
	}

	if (src.m_indexBuffer)
	{
		ndArray<ndInt32> indices;
		indices.SetCount(m_indexCount);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, src.m_indexBuffer);
		ndInt32* const srcData = (ndInt32*)glMapBuffer(GL_ELEMENT_ARRAY_BUFFER, GL_READ_ONLY);
		ndMemCpy(&indices[0], srcData, m_indexCount);

		glGenBuffers(1, &m_indexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(m_indexCount * sizeof(GLuint)), &indices[0], GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	}

	if (src.m_vertexBuffer)
	{
		ndArray<ndReal> vertexBuffer;
		ndInt32 sizeInReals = ndInt32 (m_vertexSize * m_vertexCount / sizeof(ndReal));
		vertexBuffer.SetCount(sizeInReals);
		glBindBuffer(GL_ARRAY_BUFFER, src.m_vertexBuffer);
		const ndReal* const srcData = (ndReal*)glMapBuffer(GL_ARRAY_BUFFER, GL_READ_ONLY);
		ndMemCpy(&vertexBuffer[0], srcData, sizeInReals);
		
		glGenBuffers(1, &m_vertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
		glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(sizeInReals * sizeof(ndReal)), &vertexBuffer[0], GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	}

	if (src.m_vertextArrayBuffer)
	{
		glGenVertexArrays(1, &m_vertextArrayBuffer);
		glBindVertexArray(m_vertextArrayBuffer);

		size_t offset = 0;
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, GLsizei(m_vertexSize), (void*)offset);
		offset += 3 * sizeof(ndReal);
		if (offset < size_t(m_vertexSize))
		{
			glEnableVertexAttribArray(1);
			glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, GLsizei(m_vertexSize), (void*)offset);
			offset += 3 * sizeof(ndReal);
			if (offset < size_t(m_vertexSize))
			{
				glEnableVertexAttribArray(2);
				glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, GLsizei(m_vertexSize), (void*)offset);
				offset += 2 * sizeof(ndReal);
				if (offset < size_t(m_vertexSize))
				{
					// clone a skinned mesh weights and matrix index
					 glEnableVertexAttribArray(3);
					 glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(m_vertexSize), (void*)offset);
					 offset += 4 * sizeof(ndReal);

					 glEnableVertexAttribArray(4);
					 glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, sizeof(m_vertexSize), (void*)offset);
					 offset += 4 * sizeof(ndReal);
				}
			}
		}
		glBindVertexArray(0);
	}

	// bind the matric pallete 
	const ndRenderSceneNode* const root = skeleton->GetRoot();
	for (ndInt32 i = 0; i < src.m_skeleton.GetCount(); ++i)
	{
		ndRenderSceneNode* const bone = root->FindByName(src.m_skeleton[i]->m_name);
		ndAssert(bone);
		m_skeleton.PushBack(bone);
		m_bindingSkinMatrixArray.PushBack(src.m_bindingSkinMatrixArray[i]);
	}

	InitShaderBlocks();
}

ndRenderPrimitiveImplement::~ndRenderPrimitiveImplement()
{
	if (m_instanceRenderMatrixPalleteBuffer)
	{
		glDeleteBuffers(1, &m_instanceRenderMatrixPalleteBuffer);
	}

	if (m_indexBuffer)
	{
		glDeleteBuffers(1, &m_indexBuffer);
	}

	if (m_vertexBuffer)
	{
		glDeleteBuffers(1, &m_vertexBuffer);
	}

	if (m_vertextArrayBuffer)
	{
		glDeleteVertexArrays(1, &m_vertextArrayBuffer);
	}
}

void ndRenderPrimitiveImplement::InitShaderBlocks()
{
	m_generateShadowMapsBlock.GetShaderParameters(*m_context->m_shaderCache);
	m_transparencyDiffusedBlock.GetShaderParameters(*m_context->m_shaderCache);
	m_generateSkinShadowMapsBlock.GetShaderParameters(*m_context->m_shaderCache);
	m_opaqueDiffusedColorShadowBlock.GetShaderParameters(*m_context->m_shaderCache);
	m_generateIntanceShadowMapsBlock.GetShaderParameters(*m_context->m_shaderCache);
	m_opaqueDifusedColorNoShadowBlock.GetShaderParameters(*m_context->m_shaderCache);
	m_opaqueDiffusedColorShadowSkinBlock.GetShaderParameters(*m_context->m_shaderCache);
	m_opaqueDifusedColorNoShadowInstanceBlock.GetShaderParameters(*m_context->m_shaderCache);
}

bool ndRenderPrimitiveImplement::IsSKinnedMesh() const
{
	return m_skinSceneNode ? true : false;
}

void ndRenderPrimitiveImplement::UpdateSkinPalleteMatrix()
{
	if (!IsSKinnedMesh())
	{
		return;
	}

	m_genericMatricArray.SetCount(0);
	ndMatrix shapeBindMatrix((m_skinSceneNode->m_primitiveMatrix * m_skinSceneNode->m_matrix).OrthoInverse());
	for (ndInt32 i = 0; i < ndInt32(m_skeleton.GetCount()); ++i)
	{
		const ndRenderSceneNode* const bone = m_skeleton[i];
		m_genericMatricArray.PushBack(glMatrix(m_bindingSkinMatrixArray[i] * bone->m_globalMatrix * shapeBindMatrix));
	}
}

ndRenderPrimitiveImplement* ndRenderPrimitiveImplement::Clone(
	ndRenderPrimitive* const primiteveOwner, const ndRenderSceneNode* const skeletonOwner) const
{
	ndRenderPrimitiveImplement* const mesh = new ndRenderPrimitiveImplement(*this, skeletonOwner);
	mesh->m_owner = primiteveOwner;
	return mesh;
}

void ndRenderPrimitiveImplement::BuildFromCollisionShape(const ndRenderPrimitive::ndDescriptor& descriptor)
{
	switch (descriptor.m_meshBuildMode)
	{
		case ndRenderPrimitive::m_simplePrimitve:
		{
			BuildRenderMeshFromCollisionShape(descriptor);
			break;
		}

		case ndRenderPrimitive::m_instancePrimitve:
		{
			BuildRenderInstanceMesh(descriptor);
			break;
		}

		case ndRenderPrimitive::m_debugWireFrame:
		{
			BuildWireframeDebugMesh(descriptor);
			break;
		}

		case ndRenderPrimitive::m_debugFlatShaded:
		{
			BuildDebugFlatShadedMesh(descriptor);
			break;
		}

		case ndRenderPrimitive::m_debugHiddenLines:
		{
			BuildSetZBufferDebugMesh(descriptor);
			break;
		}

		default:
		{
			ndAssert(0);
		}
	}
}

void ndRenderPrimitiveImplement::BuildFromMesh(const ndRenderPrimitive::ndDescriptor& descriptor)
{
	switch (descriptor.m_meshBuildMode)
	{
		case ndRenderPrimitive::m_simplePrimitve:
		{
			if (descriptor.m_meshNode->GetVertexWeights().GetCount())
			{
				//BuildRenderSkinnedMeshFromMeshEffect(descriptor);
				BuildRenderSimpleMeshFromMeshEffect(descriptor);
			}
			else
			{
				BuildRenderSimpleMeshFromMeshEffect(descriptor);
			}
			break;
		}
		default:
		{
			ndAssert(0);
		}
	}
}

void ndRenderPrimitiveImplement::BuildRenderMeshFromCollisionShape(const ndRenderPrimitive::ndDescriptor& descriptor)
{
	ndAssert(*descriptor.m_collision);
	ndMeshEffect mesh(**descriptor.m_collision);

	ndAssert(descriptor.m_materials.GetCount());
	const ndRenderPrimitiveMaterial& material = descriptor.m_materials.GetFirst()->GetInfo();
	ndRenderTextureImageCommon* const image = (ndRenderTextureImageCommon*)*material.m_texture;
	ndInt32 textureId = ndInt32(image->m_texture);
	switch (descriptor.m_mapping)
	{
		case ndRenderPrimitive::m_capsule:
		case ndRenderPrimitive::m_spherical:
		{
			ndMatrix flipMatrix(ndGetIdentityMatrix());
			flipMatrix[0][0] = ndFloat32(-1.0f);
			ndMatrix aligmentUV(flipMatrix * descriptor.m_uvMatrix);
			mesh.SphericalMapping(textureId, aligmentUV);
			break;
		}

		case ndRenderPrimitive::m_cylindrical:
		{
			ndMatrix flipMatrix(ndGetIdentityMatrix());
			flipMatrix[0][0] = ndFloat32(-1.0f);
			ndMatrix aligmentUV(flipMatrix * descriptor.m_uvMatrix);
			mesh.CylindricalMapping(textureId, aligmentUV);
			break;
		}

		case ndRenderPrimitive::m_box:
		{
			if (descriptor.m_stretchMaping)
			{
				mesh.BoxMapping(textureId, textureId, textureId, descriptor.m_uvMatrix);
			}
			else
			{
				mesh.UniformBoxMapping(textureId, descriptor.m_uvMatrix);
			}
			break;
		}
		default:
		{
			mesh.UniformBoxMapping(textureId, descriptor.m_uvMatrix);
		}
	}

	ndIndexArray* const geometryHandle = mesh.MaterialGeometryBegin();

	// extract vertex data  from the newton mesh
	ndInt32 indexCount = 0;
	ndInt32 vertexCount = mesh.GetPropertiesCount();
	for (ndInt32 handle = mesh.GetFirstMaterial(geometryHandle); handle != -1; handle = mesh.GetNextMaterial(geometryHandle, handle))
	{
		indexCount += mesh.GetMaterialIndexCount(geometryHandle, handle);
	}

	ndArray<ndInt32> indices;
	ndArray<glPositionNormalUV> points;

	points.SetCount(vertexCount);
	indices.SetCount(indexCount);

	mesh.GetVertexChannel(sizeof(glPositionNormalUV), &points[0].m_posit[0]);
	mesh.GetNormalChannel(sizeof(glPositionNormalUV), &points[0].m_normal[0]);
	mesh.GetUV0Channel(sizeof(glPositionNormalUV), &points[0].m_uv.m_u);

	ndInt32 segmentStart = 0;
	for (ndInt32 handle = mesh.GetFirstMaterial(geometryHandle); handle != -1; handle = mesh.GetNextMaterial(geometryHandle, handle))
	{
		ndRenderPrimitiveSegment& segment = m_owner->m_segments.Append()->GetInfo();

		segment.m_material.m_texture = material.m_texture;
		segment.m_material.m_diffuse = material.m_diffuse;
		segment.m_material.m_opacity = material.m_opacity;
		segment.m_material.m_specular = material.m_specular;
		segment.m_material.m_castShadows = material.m_castShadows;
		segment.m_material.m_specularPower = material.m_specularPower;

		segment.m_indexCount = mesh.GetMaterialIndexCount(geometryHandle, handle);

		segment.m_segmentStart = segmentStart;
		mesh.GetMaterialGetIndexStream(geometryHandle, handle, &indices[segmentStart]);
		segmentStart += segment.m_indexCount;
	}
	mesh.MaterialGeometryEnd(geometryHandle);

	// optimize this mesh for hardware buffers if possible
	m_indexCount = indexCount;
	m_vertexCount = ndInt32(points.GetCount()); 

	glGenBuffers(1, &m_indexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(indexCount * sizeof(GLuint)), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glGenVertexArrays(1, &m_vertextArrayBuffer);
	glBindVertexArray(m_vertextArrayBuffer);

	glGenBuffers(1, &m_vertexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(points.GetCount() * sizeof(glPositionNormalUV)), &points[0], GL_STATIC_DRAW);
	m_vertexSize = sizeof(glPositionNormalUV);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_posit));

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_normal));

	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_uv));

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void ndRenderPrimitiveImplement::BuildRenderSimpleMeshFromMeshEffect(const ndRenderPrimitive::ndDescriptor& descriptor)
{
	ndMeshEffect& mesh = *((ndMeshEffect*)*descriptor.m_meshNode);
	ndAssert(descriptor.m_materials.GetCount());

	// extract vertex data  from the newton mesh
	ndInt32 indexCount = 0;
	ndInt32 vertexCount = mesh.GetPropertiesCount();
	ndIndexArray* const geometryHandle = mesh.MaterialGeometryBegin();
	for (ndInt32 handle = mesh.GetFirstMaterial(geometryHandle); handle != -1; handle = mesh.GetNextMaterial(geometryHandle, handle))
	{
		indexCount += mesh.GetMaterialIndexCount(geometryHandle, handle);
	}

	ndArray<ndInt32> indices;
	ndArray<glPositionNormalUV> points;

	points.SetCount(vertexCount);
	indices.SetCount(indexCount);

	mesh.GetVertexChannel(sizeof(glPositionNormalUV), &points[0].m_posit[0]);
	mesh.GetNormalChannel(sizeof(glPositionNormalUV), &points[0].m_normal[0]);
	mesh.GetUV0Channel(sizeof(glPositionNormalUV), &points[0].m_uv.m_u);

	ndInt32 segmentStart = 0;
	for (ndInt32 handle = mesh.GetFirstMaterial(geometryHandle); handle != -1; handle = mesh.GetNextMaterial(geometryHandle, handle))
	{
		ndInt32 materialIndex = mesh.GetMaterialID(geometryHandle, handle);
		ndList<ndRenderPrimitiveMaterial>::ndNode* materialNodes = descriptor.m_materials.GetFirst();
		for (ndInt32 i = 0; i < materialIndex; ++i)
		{
			materialNodes = materialNodes->GetNext();
		}
		const ndRenderPrimitiveMaterial& material = materialNodes->GetInfo();

		ndRenderPrimitiveSegment& segment = m_owner->m_segments.Append()->GetInfo();

		segment.m_material.m_texture = material.m_texture;
		segment.m_material.m_diffuse = material.m_diffuse;
		segment.m_material.m_opacity = material.m_opacity;
		segment.m_material.m_specular = material.m_specular;
		segment.m_material.m_reflection = material.m_reflection;
		segment.m_material.m_castShadows = material.m_castShadows;
		segment.m_material.m_specularPower = material.m_specularPower;

		segment.m_indexCount = mesh.GetMaterialIndexCount(geometryHandle, handle);

		segment.m_segmentStart = segmentStart;
		mesh.GetMaterialGetIndexStream(geometryHandle, handle, &indices[segmentStart]);
		segmentStart += segment.m_indexCount;
	}
	mesh.MaterialGeometryEnd(geometryHandle);

	// optimize this mesh for hardware buffers if possible
	m_indexCount = indexCount;
	m_vertexCount = ndInt32(points.GetCount());

	glGenBuffers(1, &m_indexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(indexCount * sizeof(GLuint)), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glGenVertexArrays(1, &m_vertextArrayBuffer);
	glBindVertexArray(m_vertextArrayBuffer);

	glGenBuffers(1, &m_vertexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(points.GetCount() * sizeof(glPositionNormalUV)), &points[0], GL_STATIC_DRAW);
	m_vertexSize = sizeof(glPositionNormalUV);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_posit));

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_normal));

	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_uv));

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void ndRenderPrimitiveImplement::BuildRenderSkinnedMeshFromMeshEffect(const ndRenderPrimitive::ndDescriptor& descriptor)
{
	ndMeshEffect& mesh = *((ndMeshEffect*)*descriptor.m_meshNode);
	ndAssert(*descriptor.m_skeleton);
	ndAssert(descriptor.m_materials.GetCount());

	// extract vertex data  from the newton mesh
	ndInt32 indexCount = 0;
	ndInt32 vertexCount = mesh.GetPropertiesCount();
	ndIndexArray* const geometryHandle = mesh.MaterialGeometryBegin();
	for (ndInt32 handle = mesh.GetFirstMaterial(geometryHandle); handle != -1; handle = mesh.GetNextMaterial(geometryHandle, handle))
	{
		indexCount += mesh.GetMaterialIndexCount(geometryHandle, handle);
	}

	ndArray<ndInt32> indices;
	ndArray<ndInt32> vertexIndex;
	ndArray<glSkinVertex> points;

	points.SetCount(vertexCount);
	indices.SetCount(indexCount);
	vertexIndex.SetCount(vertexCount);

	mesh.GetVertexIndexChannel(&vertexIndex[0]);
	mesh.GetVertexChannel(sizeof(glSkinVertex), &points[0].m_posit[0]);
	mesh.GetNormalChannel(sizeof(glSkinVertex), &points[0].m_normal[0]);
	mesh.GetUV0Channel(sizeof(glSkinVertex), &points[0].m_uv.m_u);
	mesh.GetVertexWeightChannel(sizeof(glSkinVertex), (ndMeshEffect::ndVertexWeight*)&points[0].m_weighs[0]);

	m_skinSceneNode = (ndRenderSceneNode*)*descriptor.m_skeleton;

	// encode all bone by theier hash name
	ndTree<ndRenderSceneNode*, ndInt32> boneHashIdMap;
	ndRenderSceneNode* const root = descriptor.m_skeleton->GetRoot();
	for (ndRenderSceneNode* entity = root->IteratorFirst(); entity; entity = entity->IteratorNext())
	{
		ndInt32 hash = ndInt32(ndCRC64(entity->m_name.GetStr()) & 0xffffffff);
		boneHashIdMap.Insert(entity, hash);
	}
	// build a mapping for bone to index for all bone active in the vertex weigh
	ndTree<ndInt32, ndInt32> boneToIndexMap;
	ndMatrix shapeBindMatrix(descriptor.m_skeleton->m_primitiveMatrix * descriptor.m_skeleton->CalculateGlobalTransform());
	for (ndInt32 i = 0; i < vertexCount; ++i)
	{
		for (ndInt32 j = 0; j < ND_VERTEX_WEIGHT_SIZE; ++j)
		{
			if (points[i].m_boneIndex[j] != -1)
			{
				if (!boneToIndexMap.Find(points[i].m_boneIndex[j]))
				{
					ndTree<ndRenderSceneNode*, ndInt32>::ndNode* const boneNode = boneHashIdMap.Find(points[i].m_boneIndex[j]);
					ndAssert(boneNode);
					ndRenderSceneNode* const entity = boneNode->GetInfo();
					boneToIndexMap.Insert(ndInt32(m_skeleton.GetCount()), points[i].m_boneIndex[j]);
					m_skeleton.PushBack(entity);

					//const ndMatrix boneMatrix(entity->GetTransform().GetMatrix() * parentMatrix.Pop());
					const ndMatrix boneMatrix(entity->CalculateGlobalTransform());
					const ndMatrix palleteMatrix(shapeBindMatrix * boneMatrix.OrthoInverse());
					m_bindingSkinMatrixArray.PushBack(palleteMatrix);
				}
			}
		}
	}

	// map all vetex weight to that ordinal bone id.
	for (ndInt32 i = 0; i < vertexCount; ++i)
	{
		ndAssert(ND_VERTEX_WEIGHT_SIZE == 4);
		ndInt32 boneIndexSet[4];
		glVector4 weighs (points[i].m_weighs);
		ndMemCpy(boneIndexSet, points[i].m_boneIndex, 4);
		ndMemSet(points[i].m_boneIndex, ndInt32(0), 4);

		for (ndInt32 j = 0; j < ND_VERTEX_WEIGHT_SIZE; ++j)
		{
			ndInt32 boneIndex = 0;
			ndInt32 hashId = boneIndexSet[j];
			if (hashId != -1)
			{
				ndTree<ndInt32, ndInt32>::ndNode* const entNode = boneToIndexMap.Find(hashId);
				ndAssert(entNode);
				boneIndex = entNode->GetInfo();

				points[i].m_weighs[j] = weighs[j];
				points[i].m_boneIndex[j] = boneIndex;
			}
		}
	}

	// build the skin mesh 
	ndInt32 segmentStart = 0;
	for (ndInt32 handle = mesh.GetFirstMaterial(geometryHandle); handle != -1; handle = mesh.GetNextMaterial(geometryHandle, handle))
	{
		ndInt32 materialIndex = mesh.GetMaterialID(geometryHandle, handle);
		ndList<ndRenderPrimitiveMaterial>::ndNode* materialNodes = descriptor.m_materials.GetFirst();
		for (ndInt32 i = 0; i < materialIndex; ++i)
		{
			materialNodes = materialNodes->GetNext();
		}
		const ndRenderPrimitiveMaterial& material = materialNodes->GetInfo();

		ndRenderPrimitiveSegment& segment = m_owner->m_segments.Append()->GetInfo();

		segment.m_material.m_texture = material.m_texture;
		segment.m_material.m_diffuse = material.m_diffuse;
		segment.m_material.m_opacity = material.m_opacity;
		segment.m_material.m_specular = material.m_specular;
		segment.m_material.m_reflection = material.m_reflection;
		segment.m_material.m_castShadows = material.m_castShadows;
		segment.m_material.m_specularPower = material.m_specularPower;

		segment.m_indexCount = mesh.GetMaterialIndexCount(geometryHandle, handle);

		segment.m_segmentStart = segmentStart;
		mesh.GetMaterialGetIndexStream(geometryHandle, handle, &indices[segmentStart]);
		segmentStart += segment.m_indexCount;
	}
	mesh.MaterialGeometryEnd(geometryHandle);

	// optimize this mesh for hardware buffers if possible
	m_indexCount = indexCount;
	m_vertexCount = ndInt32(points.GetCount());

	glGenBuffers(1, &m_instanceRenderMatrixPalleteBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_instanceRenderMatrixPalleteBuffer);
	glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(descriptor.m_numberOfInstances * sizeof(glMatrix)), &m_bindingSkinMatrixArray[0], GL_STATIC_DRAW);

	glGenBuffers(1, &m_indexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(indexCount * sizeof(GLuint)), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glGenVertexArrays(1, &m_vertextArrayBuffer);
	glBindVertexArray(m_vertextArrayBuffer);

	glGenBuffers(1, &m_vertexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(points.GetCount() * sizeof(glSkinVertex)), &points[0], GL_STATIC_DRAW);
	m_vertexSize = sizeof(glSkinVertex);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glSkinVertex), (void*)OFFSETOF(glSkinVertex, m_posit));

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glSkinVertex), (void*)OFFSETOF(glSkinVertex, m_normal));

	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(glSkinVertex), (void*)OFFSETOF(glSkinVertex, m_uv));

	glEnableVertexAttribArray(3);
	glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(glSkinVertex), (void*)OFFSETOF(glSkinVertex, m_weighs));

	glEnableVertexAttribArray(4);
	glVertexAttribPointer(4, 4, GL_INT, GL_FALSE, sizeof(glSkinVertex), (void*)OFFSETOF(glSkinVertex, m_boneIndex));

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void ndRenderPrimitiveImplement::BuildDebugFlatShadedMesh(const ndRenderPrimitive::ndDescriptor& descriptor)
{
	class ndDrawShape : public ndShapeDebugNotify
	{
		public:
		ndDrawShape()
			:ndShapeDebugNotify()
			,m_triangles(1024)
		{
		}

		virtual void DrawPolygon(ndInt32 vertexCount, const ndVector* const faceVertex, const ndEdgeType* const)
		{
			ndVector p0(faceVertex[0]);
			ndVector p1(faceVertex[1]);
			ndVector p2(faceVertex[2]);

			ndVector normal((p1 - p0).CrossProduct(p2 - p0));
			normal = normal.Normalize();
			for (ndInt32 i = 2; i < vertexCount; ++i)
			{
				glPositionNormal point;
				point.m_posit.m_x = GLfloat(faceVertex[0].m_x);
				point.m_posit.m_y = GLfloat(faceVertex[0].m_y);
				point.m_posit.m_z = GLfloat(faceVertex[0].m_z);
				point.m_normal.m_x = GLfloat(normal.m_x);
				point.m_normal.m_y = GLfloat(normal.m_y);
				point.m_normal.m_z = GLfloat(normal.m_z);
				m_triangles.PushBack(point);

				point.m_posit.m_x = GLfloat(faceVertex[i - 1].m_x);
				point.m_posit.m_y = GLfloat(faceVertex[i - 1].m_y);
				point.m_posit.m_z = GLfloat(faceVertex[i - 1].m_z);
				point.m_normal.m_x = GLfloat(normal.m_x);
				point.m_normal.m_y = GLfloat(normal.m_y);
				point.m_normal.m_z = GLfloat(normal.m_z);
				m_triangles.PushBack(point);

				point.m_posit.m_x = GLfloat(faceVertex[i].m_x);
				point.m_posit.m_y = GLfloat(faceVertex[i].m_y);
				point.m_posit.m_z = GLfloat(faceVertex[i].m_z);
				point.m_normal.m_x = GLfloat(normal.m_x);
				point.m_normal.m_y = GLfloat(normal.m_y);
				point.m_normal.m_z = GLfloat(normal.m_z);
				m_triangles.PushBack(point);
			}
		}

		ndArray<glPositionNormal> m_triangles;
	};

	ndDrawShape drawShapes;
	descriptor.m_collision->DebugShape(ndGetIdentityMatrix(), drawShapes);
	if (drawShapes.m_triangles.GetCount())
	{
		ndArray<ndInt32> m_triangles(drawShapes.m_triangles.GetCount());
		m_triangles.SetCount(drawShapes.m_triangles.GetCount());

		m_indexCount = ndInt32(m_triangles.GetCount());
		ndInt32 vertexCount = ndVertexListToIndexList(&drawShapes.m_triangles[0].m_posit.m_x, sizeof(glPositionNormal), 6, ndInt32(drawShapes.m_triangles.GetCount()), &m_triangles[0], GLfloat(1.0e-6f));

		glGenVertexArrays(1, &m_vertextArrayBuffer);
		glBindVertexArray(m_vertextArrayBuffer);

		glGenBuffers(1, &m_vertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);

		glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(vertexCount * sizeof(glPositionNormal)), &drawShapes.m_triangles[0].m_posit.m_x, GL_STATIC_DRAW);
		m_vertexSize = sizeof(glPositionNormal);

		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormal), (void*)OFFSETOF(glPositionNormal, m_posit));

		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormal), (void*)OFFSETOF(glPositionNormal, m_normal));

		glGenBuffers(1, &m_indexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(m_indexCount * sizeof(GLuint)), &m_triangles[0], GL_STATIC_DRAW);

		glBindVertexArray(0);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		ndRenderPrimitiveSegment& segment = m_owner->m_segments.Append()->GetInfo();

		segment.m_material.m_specular = ndVector::m_zero;
		segment.m_material.m_reflection = ndVector::m_zero;

		segment.m_segmentStart = 0;
		segment.m_indexCount = m_indexCount;
		m_debugFlatShadedColorBlock.GetShaderParameters(*m_context->m_shaderCache);
	}
}

void ndRenderPrimitiveImplement::BuildWireframeDebugMesh(const ndRenderPrimitive::ndDescriptor& descriptor)
{
	class ndDrawShape : public ndShapeDebugNotify
	{
		public:
		ndDrawShape()
			:ndShapeDebugNotify()
			,m_lines(1024)
		{
		}

		virtual void DrawPolygon(ndInt32 vertexCount, const ndVector* const faceVertex, const ndEdgeType* const)
		{
			ndVector p0(faceVertex[0]);
			ndVector p1(faceVertex[1]);
			ndVector p2(faceVertex[2]);

			ndVector normal((p1 - p0).CrossProduct(p2 - p0));
			normal = normal.Normalize();
			ndInt32 i0 = vertexCount - 1;
			for (ndInt32 i = 0; i < vertexCount; ++i)
			{
				glPositionNormal point;
				point.m_posit.m_x = GLfloat(faceVertex[i].m_x);
				point.m_posit.m_y = GLfloat(faceVertex[i].m_y);
				point.m_posit.m_z = GLfloat(faceVertex[i].m_z);
				point.m_normal.m_x = GLfloat(normal.m_x);
				point.m_normal.m_y = GLfloat(normal.m_y);
				point.m_normal.m_z = GLfloat(normal.m_z);
				m_lines.PushBack(point);

				point.m_posit.m_x = GLfloat(faceVertex[i0].m_x);
				point.m_posit.m_y = GLfloat(faceVertex[i0].m_y);
				point.m_posit.m_z = GLfloat(faceVertex[i0].m_z);
				point.m_normal.m_x = GLfloat(normal.m_x);
				point.m_normal.m_y = GLfloat(normal.m_y);
				point.m_normal.m_z = GLfloat(normal.m_z);
				m_lines.PushBack(point);
				i0 = i;
			}
		}

		ndArray<glPositionNormal> m_lines;
	};

	ndDrawShape drawShapes;
	descriptor.m_collision->DebugShape(ndGetIdentityMatrix(), drawShapes);
	if (drawShapes.m_lines.GetCount())
	{
		ndArray<ndInt32> m_lines(drawShapes.m_lines.GetCount());
		m_lines.SetCount(drawShapes.m_lines.GetCount());

		m_indexCount = ndInt32(m_lines.GetCount());
		ndInt32 vertexCount = ndVertexListToIndexList(&drawShapes.m_lines[0].m_posit.m_x, sizeof(glPositionNormal), 6, ndInt32(drawShapes.m_lines.GetCount()), &m_lines[0], GLfloat(1.0e-6f));

		glGenVertexArrays(1, &m_vertextArrayBuffer);
		glBindVertexArray(m_vertextArrayBuffer);

		glGenBuffers(1, &m_vertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);

		glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(vertexCount * sizeof(glPositionNormal)), &drawShapes.m_lines[0].m_posit.m_x, GL_STATIC_DRAW);
		m_vertexSize = sizeof(glPositionNormal);

		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormal), (void*)OFFSETOF(glPositionNormal, m_posit));

		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormal), (void*)OFFSETOF(glPositionNormal, m_normal));

		glGenBuffers(1, &m_indexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(m_indexCount * sizeof(GLuint)), &m_lines[0], GL_STATIC_DRAW);

		glBindVertexArray(0);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		ndRenderPrimitiveSegment& segment = m_owner->m_segments.Append()->GetInfo();

		segment.m_material.m_specular = ndVector::m_zero;
		segment.m_material.m_reflection = ndVector::m_zero;

		segment.m_segmentStart = 0;
		segment.m_indexCount = m_indexCount;

		m_debugWireframeColorBlock.GetShaderParameters(*m_context->m_shaderCache);
	}
}

void ndRenderPrimitiveImplement::BuildSetZBufferDebugMesh(const ndRenderPrimitive::ndDescriptor& descriptor)
{
	class ndDrawShape : public ndShapeDebugNotify
	{
		public:
		ndDrawShape()
			:ndShapeDebugNotify()
			,m_triangles(1024)
		{
		}

		virtual void DrawPolygon(ndInt32 vertexCount, const ndVector* const faceVertex, const ndEdgeType* const)
		{
			ndVector p0(faceVertex[0]);
			ndVector p1(faceVertex[1]);
			ndVector p2(faceVertex[2]);

			for (ndInt32 i = 2; i < vertexCount; ++i)
			{
				glVector3 point;
				point.m_x = GLfloat(faceVertex[0].m_x);
				point.m_y = GLfloat(faceVertex[0].m_y);
				point.m_z = GLfloat(faceVertex[0].m_z);
				m_triangles.PushBack(point);

				point.m_x = GLfloat(faceVertex[i - 1].m_x);
				point.m_y = GLfloat(faceVertex[i - 1].m_y);
				point.m_z = GLfloat(faceVertex[i - 1].m_z);
				m_triangles.PushBack(point);

				point.m_x = GLfloat(faceVertex[i].m_x);
				point.m_y = GLfloat(faceVertex[i].m_y);
				point.m_z = GLfloat(faceVertex[i].m_z);
				m_triangles.PushBack(point);
			}
		}

		ndArray<glVector3> m_triangles;
	};

	ndDrawShape drawShapes;
	descriptor.m_collision->DebugShape(ndGetIdentityMatrix(), drawShapes);
	if (drawShapes.m_triangles.GetCount())
	{
		ndArray<ndInt32> m_triangles(drawShapes.m_triangles.GetCount());
		m_triangles.SetCount(drawShapes.m_triangles.GetCount());

		m_indexCount = ndInt32(m_triangles.GetCount());
		ndInt32 vertexCount = ndVertexListToIndexList(&drawShapes.m_triangles[0].m_x, sizeof(glVector3), 3, ndInt32(drawShapes.m_triangles.GetCount()), &m_triangles[0], GLfloat(1.0e-6f));

		glGenVertexArrays(1, &m_vertextArrayBuffer);
		glBindVertexArray(m_vertextArrayBuffer);

		glGenBuffers(1, &m_vertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);

		glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(vertexCount * sizeof(glVector3)), &drawShapes.m_triangles[0].m_x, GL_STATIC_DRAW);
		m_vertexSize = sizeof(glVector3);

		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glVector3), 0);

		glGenBuffers(1, &m_indexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(m_indexCount * sizeof(GLuint)), &m_triangles[0], GL_STATIC_DRAW);

		glBindVertexArray(0);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		ndRenderPrimitiveSegment& segment = m_owner->m_segments.Append()->GetInfo();

		segment.m_material.m_specular = ndVector::m_zero;
		segment.m_material.m_reflection = ndVector::m_zero;

		segment.m_segmentStart = 0;
		segment.m_indexCount = m_indexCount;
		m_setZbufferBlock.GetShaderParameters(*m_context->m_shaderCache);
	}
}

void ndRenderPrimitiveImplement::BuildRenderInstanceMesh(const ndRenderPrimitive::ndDescriptor& descriptor)
{
	ndAssert(descriptor.m_collision);
	ndMeshEffect mesh(**descriptor.m_collision);

	ndAssert(descriptor.m_materials.GetCount());
	const ndRenderPrimitiveMaterial& material = descriptor.m_materials.GetFirst()->GetInfo();
	ndRenderTextureImageCommon* const image = (ndRenderTextureImageCommon*)*material.m_texture;
	ndInt32 textureId = ndInt32(image->m_texture);
	switch (descriptor.m_mapping)
	{
		case ndRenderPrimitive::m_capsule:
		case ndRenderPrimitive::m_spherical:
		{
			ndMatrix flipMatrix(ndGetIdentityMatrix());
			flipMatrix[0][0] = ndFloat32(-1.0f);
			ndMatrix aligmentUV(flipMatrix * descriptor.m_uvMatrix);
			mesh.SphericalMapping(textureId, aligmentUV);
			break;
		}

		case ndRenderPrimitive::m_cylindrical:
		{
			ndMatrix flipMatrix(ndGetIdentityMatrix());
			flipMatrix[0][0] = ndFloat32(-1.0f);
			ndMatrix aligmentUV(flipMatrix * descriptor.m_uvMatrix);
			mesh.CylindricalMapping(textureId, aligmentUV);
			break;
		}

		case ndRenderPrimitive::m_box:
		{
			if (descriptor.m_stretchMaping)
			{
				mesh.BoxMapping(textureId, textureId, textureId, descriptor.m_uvMatrix);
			}
			else
			{
				mesh.UniformBoxMapping(textureId, descriptor.m_uvMatrix);
			}
			break;
		}
		default:
		{
			mesh.UniformBoxMapping(textureId, descriptor.m_uvMatrix);
		}
	}

	ndIndexArray* const geometryHandle = mesh.MaterialGeometryBegin();

	// extract vertex data  from the newton mesh
	ndInt32 indexCount = 0;
	ndInt32 vertexCount = mesh.GetPropertiesCount();
	for (ndInt32 handle = mesh.GetFirstMaterial(geometryHandle); handle != -1; handle = mesh.GetNextMaterial(geometryHandle, handle))
	{
		indexCount += mesh.GetMaterialIndexCount(geometryHandle, handle);
	}

	ndArray<ndInt32> indices;
	ndArray<glPositionNormalUV> points;

	points.SetCount(vertexCount);
	indices.SetCount(indexCount);

	mesh.GetVertexChannel(sizeof(glPositionNormalUV), &points[0].m_posit[0]);
	mesh.GetNormalChannel(sizeof(glPositionNormalUV), &points[0].m_normal[0]);
	mesh.GetUV0Channel(sizeof(glPositionNormalUV), &points[0].m_uv.m_u);

	ndInt32 segmentStart = 0;
	for (ndInt32 handle = mesh.GetFirstMaterial(geometryHandle); handle != -1; handle = mesh.GetNextMaterial(geometryHandle, handle))
	{
		ndRenderPrimitiveSegment& segment = m_owner->m_segments.Append()->GetInfo();

		segment.m_material.m_texture = material.m_texture;
		segment.m_material.m_diffuse = material.m_diffuse;
		segment.m_material.m_opacity = material.m_opacity;
		segment.m_material.m_specular = material.m_specular;
		segment.m_material.m_castShadows = material.m_castShadows;
		segment.m_material.m_specularPower = material.m_specularPower;

		segment.m_indexCount = mesh.GetMaterialIndexCount(geometryHandle, handle);

		segment.m_segmentStart = segmentStart;
		mesh.GetMaterialGetIndexStream(geometryHandle, handle, &indices[segmentStart]);
		segmentStart += segment.m_indexCount;
	}
	mesh.MaterialGeometryEnd(geometryHandle);

	// optimize this mesh for hardware buffers if possible
	m_indexCount = indexCount;
	m_vertexCount = ndInt32(points.GetCount());

	glGenBuffers(1, &m_indexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(indexCount * sizeof(GLuint)), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glGenVertexArrays(1, &m_vertextArrayBuffer);
	glBindVertexArray(m_vertextArrayBuffer);

	glGenBuffers(1, &m_vertexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(points.GetCount() * sizeof(glPositionNormalUV)), &points[0], GL_STATIC_DRAW);
	m_vertexSize = sizeof(glPositionNormalUV);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_posit));

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_normal));

	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_uv));

	// set vertex buffer for matrix instances
	m_genericMatricArray.SetCount(descriptor.m_numberOfInstances);

	glGenBuffers(1, &m_instanceRenderMatrixPalleteBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_instanceRenderMatrixPalleteBuffer);
	glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(descriptor.m_numberOfInstances * sizeof(glMatrix)), &m_genericMatricArray[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(3);
	glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(glMatrix), (void*)(0 * sizeof(glVector4)));
	glVertexAttribDivisor(3, 1);

	glEnableVertexAttribArray(4);
	glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, sizeof(glMatrix), (void*)(1 * sizeof(glVector4)));
	glVertexAttribDivisor(4, 1);

	glEnableVertexAttribArray(5);
	glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, sizeof(glMatrix), (void*)(2 * sizeof(glVector4)));
	glVertexAttribDivisor(5, 1);

	glEnableVertexAttribArray(6);
	glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, sizeof(glMatrix), (void*)(3 * sizeof(glVector4)));
	glVertexAttribDivisor(6, 1);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void ndRenderPrimitiveImplement::Render(const ndRender* const render, const ndMatrix& modelMatrix, ndRenderPassMode renderMode) const
{
	switch (renderMode)
	{
		case m_generateShadowMaps:
			RenderGenerateShadowMaps(render, modelMatrix);
			break;

		case m_directionalDiffusseNoShadow:
			RenderDirectionalDiffuseColorNoShadow(render, modelMatrix);
			break;

		case m_directionalDiffusseShadow:
			RenderDirectionalDiffuseColorShadow(render, modelMatrix);
			break;

		case m_transparencyBackface:
			RenderTransparency(render, modelMatrix, true);
			break;

		case m_transparencyFrontface:
			RenderTransparency(render, modelMatrix, false);
			break;

		case m_debugDisplaySolidMesh:
			RenderDebugShapeSolid(render, modelMatrix);
			break;

		case m_debugDisplayWireFrameMesh:
			RenderDebugShapeWireFrame(render, modelMatrix);
			break;

		case m_debugDisplaySetZbuffer:
			RenderDebugSetZbuffer(render, modelMatrix);
			break;

		case m_directionalDiffusseInstanceShadow:
			RenderDirectionalDiffuseColorInstanceShadow(render, modelMatrix);
			break;

		case m_m_generateInstanceShadowMaps:
			RenderGenerateInstancedShadowMaps(render, modelMatrix);
			break;

		default:
			ndAssert(0);
	}
}

void ndRenderPrimitiveImplement::RenderDebugSetZbuffer(const ndRender* const render, const ndMatrix& modelMatrix) const
{
	m_setZbufferBlock.Render(this, render, modelMatrix);
}

void ndRenderPrimitiveImplement::RenderDebugShapeSolid(const ndRender* const render, const ndMatrix& modelMatrix) const
{
	m_debugFlatShadedColorBlock.Render(this, render, modelMatrix);
}

void ndRenderPrimitiveImplement::RenderDebugShapeWireFrame(const ndRender* const render, const ndMatrix& modelMatrix) const
{
	m_debugWireframeColorBlock.Render(this, render, modelMatrix);
}

void ndRenderPrimitiveImplement::RenderGenerateShadowMaps(const ndRender* const render, const ndMatrix& lightMatrix) const
{
	bool castShadow = true;
	for (ndList<ndRenderPrimitiveSegment>::ndNode* node = m_owner->m_segments.GetFirst(); node && castShadow; node = node->GetNext())
	{
		ndRenderPrimitiveSegment& segment = node->GetInfo();
		castShadow = castShadow && segment.m_material.m_castShadows;
	}

	if (castShadow)
	{
		if (IsSKinnedMesh())
		{
			m_generateSkinShadowMapsBlock.Render(this, render, lightMatrix);
		}
		else
		{
			m_generateShadowMapsBlock.Render(this, render, lightMatrix);
		}
	}
}

void ndRenderPrimitiveImplement::RenderGenerateInstancedShadowMaps(const ndRender* const render, const ndMatrix& lightMatrix) const
{
	bool castShadow = true;
	for (ndList<ndRenderPrimitiveSegment>::ndNode* node = m_owner->m_segments.GetFirst(); node && castShadow; node = node->GetNext())
	{
		ndRenderPrimitiveSegment& segment = node->GetInfo();
		castShadow = castShadow && segment.m_material.m_castShadows;
	}

	if (castShadow && !IsSKinnedMesh())
	{
		m_generateIntanceShadowMapsBlock.Render(this, render, lightMatrix);
	}
}

void ndRenderPrimitiveImplement::RenderDirectionalDiffuseColorNoShadow(const ndRender* const render, const ndMatrix& modelMatrix) const
{
	bool castShadow = true;
	for (ndList<ndRenderPrimitiveSegment>::ndNode* node = m_owner->m_segments.GetFirst(); node && castShadow; node = node->GetNext())
	{
		ndRenderPrimitiveSegment& segment = node->GetInfo();
		castShadow = castShadow && segment.m_material.m_castShadows;
	}

	if (castShadow)
	{
		return;
	}

	m_opaqueDifusedColorNoShadowBlock.Render(this, render, modelMatrix);
}

void ndRenderPrimitiveImplement::RenderDirectionalDiffuseColorShadow(const ndRender* const render, const ndMatrix& modelMatrix) const
{
	//bool castShadow = true;
	//for (ndList<ndRenderPrimitiveSegment>::ndNode* node = m_segments.GetFirst(); node && castShadow; node = node->GetNext())
	//{
	//	ndRenderPrimitiveSegment& segment = node->GetInfo();
	//	castShadow = castShadow && segment.m_material.m_castShadows;
	//}
	//
	//if (!castShadow)
	//{
	//	return;
	//}

	if (IsSKinnedMesh())
	{
		m_opaqueDiffusedColorShadowSkinBlock.Render(this, render, modelMatrix);
	}
	else
	{
		m_opaqueDiffusedColorShadowBlock.Render(this, render, modelMatrix);
	}
}

void ndRenderPrimitiveImplement::RenderTransparency(const ndRender* const render, const ndMatrix& modelMatrix, bool backface) const
{
	bool isOpaque = true;
	for (ndList<ndRenderPrimitiveSegment>::ndNode* node = m_owner->m_segments.GetFirst(); node && isOpaque; node = node->GetNext())
	{
		ndRenderPrimitiveSegment& segment = node->GetInfo();
		isOpaque = isOpaque && (segment.m_material.m_opacity > ndFloat32(0.99f));
	}
	if (isOpaque)
	{
		return;
	}

	m_transparencyDiffusedBlock.SetWidingMode(backface);
	m_transparencyDiffusedBlock.Render(this, render, modelMatrix);
}

void ndRenderPrimitiveImplement::RenderDirectionalDiffuseColorInstanceShadow(const ndRender* const render, const ndMatrix& modelMatrix) const
{
	m_opaqueDifusedColorNoShadowInstanceBlock.Render(this, render, modelMatrix);
}
