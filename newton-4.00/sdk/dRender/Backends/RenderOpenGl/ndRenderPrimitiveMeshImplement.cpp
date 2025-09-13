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
#include "ndRenderOpenGlUtil.h"
#include "ndRenderShaderCache.h"
#include "ndRenderSceneCamera.h"
#include "ndRenderTextureImage.h"
#include "ndRenderPrimitiveMesh.h"
#include "ndRenderPassEnvironment.h"
#include "ndRenderPassShadowsImplement.h"
#include "ndRenderPrimitiveMeshImplement.h"

ndRenderPrimitiveMeshImplement::ndRenderPrimitiveMeshImplement(ndRenderPrimitiveMesh* const owner, const ndRenderPrimitiveMesh::ndDescriptor& descriptor)
	:ndContainersFreeListAlloc<ndRenderPrimitiveMeshImplement>()
	,m_owner(owner)
	,m_context(*descriptor.m_render->m_context)
	,m_indexCount(0)
	,m_vertexCount(0)
	,m_indexBuffer(0)
	,m_vertexBuffer(0)
	,m_vertextArrayBuffer(0)
	,m_instanceRenderMatrixPalleteBuffer(0)
{
	ndAssert(descriptor.m_collision);
	switch (descriptor.m_meshBuildMode)
	{
		case ndRenderPrimitiveMesh::m_simplePrimitve:
		{
			BuildRenderMesh(descriptor);
			break;
		}

		case ndRenderPrimitiveMesh::m_instancePrimitve:
		{
			BuildRenderInstanceMesh(descriptor);
			break;
		}

		case ndRenderPrimitiveMesh::m_debugWireFrame:
		{
			BuildWireframeDebugMesh(descriptor);
			break;
		}

		case ndRenderPrimitiveMesh::m_debugFlatShaded:
		{
			BuildDebugFlatShadedMesh(descriptor);
			break;
		}

		case ndRenderPrimitiveMesh::m_debugHiddenLines:
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

ndRenderPrimitiveMeshImplement::~ndRenderPrimitiveMeshImplement()
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

void ndRenderPrimitiveMeshImplement::BuildRenderMesh(const ndRenderPrimitiveMesh::ndDescriptor& descriptor)
{
	ndAssert(descriptor.m_collision);
	ndMeshEffect mesh(*descriptor.m_collision);

	ndRenderTextureImageCommon* const image = (ndRenderTextureImageCommon*)*descriptor.m_material.m_texture;
	ndInt32 textureId = ndInt32(image->m_texture);
	switch (descriptor.m_mapping)
	{
		case ndRenderPrimitiveMesh::m_spherical:
		case ndRenderPrimitiveMesh::m_cylindrical:
		{
			ndMatrix flipMatrix(ndGetIdentityMatrix());
			flipMatrix[0][0] = ndFloat32(-1.0f);
			ndMatrix aligmentUV(flipMatrix * descriptor.m_uvMatrix);
			mesh.SphericalMapping(textureId, aligmentUV);
			break;
		}

		case ndRenderPrimitiveMesh::m_box:
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

	struct dTmpData
	{
		ndReal m_posit[3];
		ndReal m_normal[3];
		ndReal m_uv[2];
	};

	ndArray<dTmpData> tmp;
	ndArray<ndInt32> indices;
	ndArray<glPositionNormalUV> points;

	tmp.SetCount(vertexCount);
	points.SetCount(vertexCount);
	indices.SetCount(indexCount);

	mesh.GetVertexChannel(sizeof(dTmpData), &tmp[0].m_posit[0]);
	mesh.GetNormalChannel(sizeof(dTmpData), &tmp[0].m_normal[0]);
	mesh.GetUV0Channel(sizeof(dTmpData), &tmp[0].m_uv[0]);

	for (ndInt32 i = 0; i < vertexCount; ++i)
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

	ndInt32 segmentStart = 0;
	for (ndInt32 handle = mesh.GetFirstMaterial(geometryHandle); handle != -1; handle = mesh.GetNextMaterial(geometryHandle, handle))
	{
		ndRenderPrimitiveMeshSegment& segment = m_owner->m_segments.Append()->GetInfo();

		segment.m_material.m_texture = descriptor.m_material.m_texture;
		segment.m_material.m_diffuse = descriptor.m_material.m_diffuse;
		segment.m_material.m_opacity = descriptor.m_material.m_opacity;
		segment.m_material.m_specular = descriptor.m_material.m_specular;
		segment.m_material.m_castShadows = descriptor.m_material.m_castShadows;
		segment.m_material.m_specularPower = descriptor.m_material.m_specularPower;

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

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_posit));

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_normal));

	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_uv));

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	m_transparencyDiffusedBlock.GetShaderParameters(*m_context->m_shaderCache);
	m_opaqueDifusedColorShadowBlock.GetShaderParameters(*m_context->m_shaderCache);
	m_opaqueDifusedColorNoShadowBlock.GetShaderParameters(*m_context->m_shaderCache);
}

void ndRenderPrimitiveMeshImplement::BuildDebugFlatShadedMesh(const ndRenderPrimitiveMesh::ndDescriptor& descriptor)
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

		ndRenderPrimitiveMeshSegment& segment = m_owner->m_segments.Append()->GetInfo();

		segment.m_material.m_specular = ndVector::m_zero;
		segment.m_material.m_reflection = ndVector::m_zero;

		segment.m_segmentStart = 0;
		segment.m_indexCount = m_indexCount;
		m_debugFlatShadedColorBlock.GetShaderParameters(*m_context->m_shaderCache);
	}
}

void ndRenderPrimitiveMeshImplement::BuildWireframeDebugMesh(const ndRenderPrimitiveMesh::ndDescriptor& descriptor)
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

		ndRenderPrimitiveMeshSegment& segment = m_owner->m_segments.Append()->GetInfo();

		segment.m_material.m_specular = ndVector::m_zero;
		segment.m_material.m_reflection = ndVector::m_zero;

		segment.m_segmentStart = 0;
		segment.m_indexCount = m_indexCount;

		m_debugWireframeColorBlock.GetShaderParameters(*m_context->m_shaderCache);
	}
}

void ndRenderPrimitiveMeshImplement::BuildSetZBufferDebugMesh(const ndRenderPrimitiveMesh::ndDescriptor& descriptor)
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

		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glVector3), 0);

		glGenBuffers(1, &m_indexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(m_indexCount * sizeof(GLuint)), &m_triangles[0], GL_STATIC_DRAW);

		glBindVertexArray(0);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		ndRenderPrimitiveMeshSegment& segment = m_owner->m_segments.Append()->GetInfo();

		segment.m_material.m_specular = ndVector::m_zero;
		segment.m_material.m_reflection = ndVector::m_zero;

		segment.m_segmentStart = 0;
		segment.m_indexCount = m_indexCount;
		m_setZbufferBlock.GetShaderParameters(*m_context->m_shaderCache);
	}
}

void ndRenderPrimitiveMeshImplement::BuildRenderInstanceMesh(const ndRenderPrimitiveMesh::ndDescriptor& descriptor)
{
	ndAssert(descriptor.m_collision);
	ndMeshEffect mesh(*descriptor.m_collision);

	ndRenderTextureImageCommon* const image = (ndRenderTextureImageCommon*)*descriptor.m_material.m_texture;
	ndInt32 textureId = ndInt32(image->m_texture);
	switch (descriptor.m_mapping)
	{
		case ndRenderPrimitiveMesh::m_spherical:
		case ndRenderPrimitiveMesh::m_cylindrical:
		{
			ndMatrix flipMatrix(ndGetIdentityMatrix());
			flipMatrix[0][0] = ndFloat32(-1.0f);
			ndMatrix aligmentUV(flipMatrix * descriptor.m_uvMatrix);
			mesh.SphericalMapping(textureId, aligmentUV);
			break;
		}

		case ndRenderPrimitiveMesh::m_box:
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
			ndAssert(0);
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

	struct dTmpData
	{
		ndReal m_posit[3];
		ndReal m_normal[3];
		ndReal m_uv[2];
	};

	ndArray<dTmpData> tmp;
	ndArray<ndInt32> indices;
	ndArray<glPositionNormalUV> points;

	tmp.SetCount(vertexCount);
	points.SetCount(vertexCount);
	indices.SetCount(indexCount);

	mesh.GetVertexChannel(sizeof(dTmpData), &tmp[0].m_posit[0]);
	mesh.GetNormalChannel(sizeof(dTmpData), &tmp[0].m_normal[0]);
	mesh.GetUV0Channel(sizeof(dTmpData), &tmp[0].m_uv[0]);

	for (ndInt32 i = 0; i < vertexCount; ++i)
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

	ndInt32 segmentStart = 0;
	for (ndInt32 handle = mesh.GetFirstMaterial(geometryHandle); handle != -1; handle = mesh.GetNextMaterial(geometryHandle, handle))
	{
		ndRenderPrimitiveMeshSegment& segment = m_owner->m_segments.Append()->GetInfo();

		segment.m_material.m_texture = descriptor.m_material.m_texture;
		segment.m_material.m_diffuse = descriptor.m_material.m_diffuse;
		segment.m_material.m_opacity = descriptor.m_material.m_opacity;
		segment.m_material.m_specular = descriptor.m_material.m_specular;
		segment.m_material.m_castShadows = descriptor.m_material.m_castShadows;
		segment.m_material.m_specularPower = descriptor.m_material.m_specularPower;

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

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_posit));

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_normal));

	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_uv));

	// set vertex buffer for matrix instances
	m_instanceRenderMatrixPallete.SetCount(descriptor.m_numberOfInstances);

	glGenBuffers(1, &m_instanceRenderMatrixPalleteBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_instanceRenderMatrixPalleteBuffer);
	glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(descriptor.m_numberOfInstances * sizeof(glMatrix)), &m_instanceRenderMatrixPallete[0], GL_STATIC_DRAW);

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

	//m_transparencyDiffusedBlock.GetShaderParameters(*m_context->m_shaderCache);
	//m_opaqueDifusedColorShadowBlock.GetShaderParameters(*m_context->m_shaderCache);
	//m_opaqueDifusedColorNoShadowBlock.GetShaderParameters(*m_context->m_shaderCache);
	m_opaqueDifusedColorNoShadowInstanceBlock.GetShaderParameters(*m_context->m_shaderCache);
}

void ndRenderPrimitiveMeshImplement::Render(const ndRender* const render, const ndMatrix& modelMatrix, ndRenderPassMode renderMode) const
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

		default:
			ndAssert(0);
	}
}

void ndRenderPrimitiveMeshImplement::RenderDebugSetZbuffer(const ndRender* const render, const ndMatrix& modelMatrix) const
{
	m_setZbufferBlock.Render(this, render, modelMatrix);
}

void ndRenderPrimitiveMeshImplement::RenderDebugShapeSolid(const ndRender* const render, const ndMatrix& modelMatrix) const
{
	m_debugFlatShadedColorBlock.Render(this, render, modelMatrix);
}

void ndRenderPrimitiveMeshImplement::RenderDebugShapeWireFrame(const ndRender* const render, const ndMatrix& modelMatrix) const
{
	m_debugWireframeColorBlock.Render(this, render, modelMatrix);
}

void ndRenderPrimitiveMeshImplement::RenderGenerateShadowMaps(const ndRender* const render, const ndMatrix& lightMatrix) const
{
	ndRenderPassShadowsImplement* const owner = render->m_cachedShadowPass;
	ndAssert(owner);

	bool castShadow = true;
	for (ndList<ndRenderPrimitiveMeshSegment>::ndNode* node = m_owner->m_segments.GetFirst(); node && castShadow; node = node->GetNext())
	{
		ndRenderPrimitiveMeshSegment& segment = node->GetInfo();
		castShadow = castShadow && segment.m_material.m_castShadows;
	}

	if (castShadow)
	{
		const ndRenderPassShadowsImplement* const shadowPass = render->m_cachedShadowPass;
		shadowPass->m_generateShadowMapsBlock.Render(this, render, lightMatrix);
	}
}

void ndRenderPrimitiveMeshImplement::RenderDirectionalDiffuseColorNoShadow(const ndRender* const render, const ndMatrix& modelMatrix) const
{
	bool castShadow = true;
	for (ndList<ndRenderPrimitiveMeshSegment>::ndNode* node = m_owner->m_segments.GetFirst(); node && castShadow; node = node->GetNext())
	{
		ndRenderPrimitiveMeshSegment& segment = node->GetInfo();
		castShadow = castShadow && segment.m_material.m_castShadows;
	}

	if (castShadow)
	{
		return;
	}

	m_opaqueDifusedColorNoShadowBlock.Render(this, render, modelMatrix);
}

void ndRenderPrimitiveMeshImplement::RenderDirectionalDiffuseColorShadow(const ndRender* const render, const ndMatrix& modelMatrix) const
{
	//bool castShadow = true;
	//for (ndList<ndRenderPrimitiveMeshSegment>::ndNode* node = m_segments.GetFirst(); node && castShadow; node = node->GetNext())
	//{
	//	ndRenderPrimitiveMeshSegment& segment = node->GetInfo();
	//	castShadow = castShadow && segment.m_material.m_castShadows;
	//}
	//
	//if (!castShadow)
	//{
	//	return;
	//}

	m_opaqueDifusedColorShadowBlock.Render(this, render, modelMatrix);
}

void ndRenderPrimitiveMeshImplement::RenderTransparency(const ndRender* const render, const ndMatrix& modelMatrix, bool backface) const
{
	bool isOpaque = true;
	for (ndList<ndRenderPrimitiveMeshSegment>::ndNode* node = m_owner->m_segments.GetFirst(); node && isOpaque; node = node->GetNext())
	{
		ndRenderPrimitiveMeshSegment& segment = node->GetInfo();
		isOpaque = isOpaque && (segment.m_material.m_opacity > ndFloat32(0.99f));
	}
	if (isOpaque)
	{
		return;
	}

	m_transparencyDiffusedBlock.SetWidingMode(backface);
	m_transparencyDiffusedBlock.Render(this, render, modelMatrix);
}

void ndRenderPrimitiveMeshImplement::RenderDirectionalDiffuseColorInstanceShadow(const ndRender* const render, const ndMatrix& modelMatrix) const
{
	m_opaqueDifusedColorNoShadowInstanceBlock.Render(this, render, modelMatrix);
}