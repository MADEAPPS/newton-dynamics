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
#include "ndDemoCamera.h"
#include "ndDemoEntity.h"
#include "ndDebugDisplay.h"
#include "ndDemoDebugMesh.h"
#include "ndDemoEntityManager.h"

ndFlatShadedDebugMesh::ndFlatShadedDebugMesh(const ndShaderCache& shaderCache, const ndShapeInstance* const collision)
	:ndDemoMeshInterface()
	,m_indexCount(0)
	,m_shadeColorLocation(0)
	,m_normalMatrixLocation(0)
	,m_projectMatrixLocation(0)
	,m_viewModelMatrixLocation(0)
	,m_shader(0)
	,m_vertexBuffer(0)
	,m_vertextArrayBuffer(0)
	,m_triangleIndexBuffer(0)
{
	class ndDrawShape : public ndDebugNotify
	{
		public:
		ndDrawShape()
			:ndDebugNotify()
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
	collision->DebugShape(ndGetIdentityMatrix(), drawShapes);
	if (drawShapes.m_triangles.GetCount())
	{
		ndArray<ndInt32> m_triangles(drawShapes.m_triangles.GetCount());
		m_triangles.SetCount(drawShapes.m_triangles.GetCount());
		ndInt32 vertexCount = ndVertexListToIndexList(&drawShapes.m_triangles[0].m_posit.m_x, sizeof(glPositionNormal), 6, ndInt32(drawShapes.m_triangles.GetCount()), &m_triangles[0], GLfloat(1.0e-6f));

		m_shader = shaderCache.m_flatShaded;
		m_indexCount = ndInt32(m_triangles.GetCount());

		m_color.m_x = 1.0f;
		m_color.m_y = 1.0f;
		m_color.m_z = 1.0f;
		m_color.m_w = 1.0f;

		glGenVertexArrays(1, &m_vertextArrayBuffer);
		glBindVertexArray(m_vertextArrayBuffer);

		glGenBuffers(1, &m_vertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);

		glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(vertexCount * sizeof(glPositionNormal)), &drawShapes.m_triangles[0].m_posit.m_x, GL_STATIC_DRAW);

		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormal), (void*)OFFSETOF(glPositionNormal, m_posit));

		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormal), (void*)OFFSETOF(glPositionNormal, m_normal));

		glGenBuffers(1, &m_triangleIndexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_triangleIndexBuffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(m_indexCount * sizeof(GLuint)), &m_triangles[0], GL_STATIC_DRAW);

		glBindVertexArray(0);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glUseProgram(m_shader);
		m_shadeColorLocation = glGetUniformLocation(m_shader, "shadeColor");
		m_normalMatrixLocation = glGetUniformLocation(m_shader, "normalMatrix");
		m_projectMatrixLocation = glGetUniformLocation(m_shader, "projectionMatrix");
		m_viewModelMatrixLocation = glGetUniformLocation(m_shader, "viewModelMatrix");

		glUseProgram(0);
	}
}

ndFlatShadedDebugMesh::~ndFlatShadedDebugMesh()
{
	if (m_vertextArrayBuffer)
	{
		glDeleteBuffers(1, &m_triangleIndexBuffer);
		glDeleteBuffers(1, &m_vertexBuffer);
		glDeleteVertexArrays(1, &m_vertextArrayBuffer);
	}
}

void ndFlatShadedDebugMesh::Render(ndDemoEntityManager* const scene, const ndMatrix& modelMatrix)
{
	if (m_shader)
	{
		glUseProgram(m_shader);

		ndDemoCamera* const camera = scene->GetCamera();

		const ndMatrix& invViewMatrix = camera->GetInvViewMatrix();
		const ndMatrix& projectionMatrix = camera->GetProjectionMatrix();
		const glMatrix viewModelMatrix(modelMatrix * invViewMatrix);

		const glMatrix projMatrix(projectionMatrix);
		const glVector4 color(m_color);

		glUniform4fv(m_shadeColorLocation, 1, &color[0]);
		glUniformMatrix4fv(m_normalMatrixLocation, 1, false, &viewModelMatrix[0][0]);
		glUniformMatrix4fv(m_projectMatrixLocation, 1, false, &projMatrix[0][0]);
		glUniformMatrix4fv(m_viewModelMatrixLocation, 1, false, &viewModelMatrix[0][0]);

		glBindVertexArray(m_vertextArrayBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_triangleIndexBuffer);

		glDrawElements(GL_TRIANGLES, m_indexCount, GL_UNSIGNED_INT, (void*)0);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);
	}
}

// *********************************************************************************
//
// *********************************************************************************
ndZbufferDebugMesh::ndZbufferDebugMesh(const ndShaderCache& shaderCache, const ndShapeInstance* const collision)
	:ndDemoMeshInterface()
	,m_indexCount(0)
	,m_viewModelProjectMatrixLocation(0)
	,m_shader(0)
	,m_vertexBuffer(0)
	,m_vertextArrayBuffer(0)
	,m_triangleIndexBuffer(0)
{
	class ndDrawShape : public ndDebugNotify
	{
		public:
		ndDrawShape()
			:ndDebugNotify()
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
	collision->DebugShape(ndGetIdentityMatrix(), drawShapes);
	if (drawShapes.m_triangles.GetCount())
	{
		ndArray<ndInt32> m_triangles(drawShapes.m_triangles.GetCount());
		m_triangles.SetCount(drawShapes.m_triangles.GetCount());
		ndInt32 vertexCount = ndVertexListToIndexList(&drawShapes.m_triangles[0].m_x, sizeof(glVector3), 3, ndInt32(drawShapes.m_triangles.GetCount()), &m_triangles[0], GLfloat(1.0e-6f));

		m_shader = shaderCache.m_zBufferDebug;
		m_indexCount = ndInt32(m_triangles.GetCount());

		glGenVertexArrays(1, &m_vertextArrayBuffer);
		glBindVertexArray(m_vertextArrayBuffer);

		glGenBuffers(1, &m_vertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);

		glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(vertexCount * sizeof(glVector3)), &drawShapes.m_triangles[0].m_x, GL_STATIC_DRAW);

		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glVector3), 0);

		glGenBuffers(1, &m_triangleIndexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_triangleIndexBuffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(m_indexCount * sizeof(GLuint)), &m_triangles[0], GL_STATIC_DRAW);

		glBindVertexArray(0);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glUseProgram(m_shader);
		m_viewModelProjectMatrixLocation = glGetUniformLocation(m_shader, "viewModelProjectionMatrix");

		glUseProgram(0);
	}
}

ndZbufferDebugMesh::~ndZbufferDebugMesh()
{
	if (m_vertextArrayBuffer)
	{
		glDeleteBuffers(1, &m_triangleIndexBuffer);
		glDeleteBuffers(1, &m_vertexBuffer);
		glDeleteVertexArrays(1, &m_vertextArrayBuffer);
	}
}

void ndZbufferDebugMesh::Render(ndDemoEntityManager* const scene, const ndMatrix& modelMatrix)
{
	if (m_shader)
	{
		glUseProgram(m_shader);

		ndDemoCamera* const camera = scene->GetCamera();
		const glMatrix viewModelProjectMatrixLocation(modelMatrix * camera->GetInvViewProjectionMatrix());
		glUniformMatrix4fv(m_viewModelProjectMatrixLocation, 1, false, &viewModelProjectMatrixLocation[0][0]);

		glBindVertexArray(m_vertextArrayBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_triangleIndexBuffer);

		glDrawElements(GL_TRIANGLES, m_indexCount, GL_UNSIGNED_INT, (void*)0);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);
	}
}

// *********************************************************************************
//
// *********************************************************************************
ndWireFrameDebugMesh::ndWireFrameDebugMesh(const ndShaderCache& shaderCache, const ndShapeInstance* const collision, ndShapeDebugNotify::ndEdgeType edgeTypefilter)
	:ndDemoMeshInterface()
	,m_indexCount(0)
	,m_shadeColorLocation(0)
	,m_projectionViewModelMatrixLocation(0)
	,m_shader(0)
	,m_vertexBuffer(0)
	,m_vertextArrayBuffer(0)
	,m_lineIndexBuffer(0)
{
	class ndDrawShape : public ndDebugNotify
	{
		public:
		ndDrawShape(ndEdgeType edgeTypefilter)
			:ndDebugNotify()
			,m_lines(1024)
			,m_edgeType(edgeTypefilter)
		{
		}

		virtual void DrawPolygon(ndInt32 vertexCount, const ndVector* const faceVertex, const ndEdgeType* const edgeType)
		{
			ndInt32 i0 = vertexCount - 1;
			for (ndInt32 i = 0; i < vertexCount; ++i)
			{
				if (edgeType[i0] == m_edgeType)
				{
					glVector3 point;
					point.m_x = GLfloat(faceVertex[i0].m_x);
					point.m_y = GLfloat(faceVertex[i0].m_y);
					point.m_z = GLfloat(faceVertex[i0].m_z);
					m_lines.PushBack(point);

					point.m_x = GLfloat(faceVertex[i].m_x);
					point.m_y = GLfloat(faceVertex[i].m_y);
					point.m_z = GLfloat(faceVertex[i].m_z);
					m_lines.PushBack(point);
				}

				i0 = i;
			}
		}

		ndArray<glVector3> m_lines;
		ndEdgeType m_edgeType;
	};

	SetColor(ndVector::m_zero);
	ndDrawShape drawShapes(edgeTypefilter);
	collision->DebugShape(ndGetIdentityMatrix(), drawShapes);
	
	if (drawShapes.m_lines.GetCount())
	{
		ndArray<ndInt32> m_lines(drawShapes.m_lines.GetCount());
		m_lines.SetCount(drawShapes.m_lines.GetCount());
		ndInt32 vertexCount = ndVertexListToIndexList(&drawShapes.m_lines[0].m_x, sizeof(glVector3), 3, ndInt32(drawShapes.m_lines.GetCount()), &m_lines[0], GLfloat(1.0e-6f));

		m_indexCount = ndInt32(m_lines.GetCount());
		ndTree<ndUnsigned64, ndUnsigned64> filter;
		for (ndInt32 i = ndInt32(m_lines.GetCount()) - 1; i >= 0; i -= 2)
		{
			union
			{
				ndUnsigned64 m_key;
				struct
				{
					ndUnsigned32 m_low;
					ndUnsigned32 m_high;
				};
			} key;
			ndInt32 i0 = m_lines[i - 1];
			ndInt32 i1 = m_lines[i - 0];
			key.m_low = ndUnsigned32(ndMin(i0, i1));
			key.m_high = ndUnsigned32(ndMax(i0, i1));
			if (filter.Find(key.m_key))
			{
				m_lines[i - 1] = m_lines[m_indexCount - 2];
				m_lines[i - 0] = m_lines[m_indexCount - 1];
				m_indexCount -= 2;
			}
			else
			{
				filter.Insert(key.m_key, key.m_key);
			}
		}

		m_shader = shaderCache.m_wireFrame;
		m_color.m_x = 1.0f;
		m_color.m_y = 1.0f;
		m_color.m_z = 1.0f;
		m_color.m_w = 1.0f;

		glGenVertexArrays(1, &m_vertextArrayBuffer);
		glBindVertexArray(m_vertextArrayBuffer);

		glGenBuffers(1, &m_vertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);

		glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(vertexCount * sizeof(glVector3)), &drawShapes.m_lines[0].m_x, GL_STATIC_DRAW);

		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glVector3), (void*)0);

		glGenBuffers(1, &m_lineIndexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_lineIndexBuffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(m_lines.GetCount() * sizeof(GLuint)), &m_lines[0], GL_STATIC_DRAW);

		glBindVertexArray(0);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glUseProgram(m_shader);
		m_shadeColorLocation = glGetUniformLocation(m_shader, "shadeColor");
		m_projectionViewModelMatrixLocation = glGetUniformLocation(m_shader, "projectionViewModelMatrix");
		glUseProgram(0);
	}
}

ndWireFrameDebugMesh::~ndWireFrameDebugMesh()
{
	if (m_vertextArrayBuffer)
	{
		glDeleteBuffers(1, &m_lineIndexBuffer);
		glDeleteBuffers(1, &m_vertexBuffer);
		glDeleteVertexArrays(1, &m_vertextArrayBuffer);
	}
}

void ndWireFrameDebugMesh::Render(ndDemoEntityManager* const scene, const ndMatrix& modelMatrix)
{
	if (m_shader)
	{
		ndDemoCamera* const camera = scene->GetCamera();
		const glMatrix projectionViewModelMatrix(modelMatrix * camera->GetInvViewProjectionMatrix());

		//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		//glDepthFunc(GL_LESS);

		const glVector4 color(m_color);

		glUseProgram(m_shader);
		glUniform4fv(m_shadeColorLocation, 1, &color[0]);
		glUniformMatrix4fv(m_projectionViewModelMatrixLocation, 1, false, &projectionViewModelMatrix[0][0]);

		glBindVertexArray(m_vertextArrayBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_lineIndexBuffer);

		glDrawElements(GL_LINES, m_indexCount, GL_UNSIGNED_INT, (void*)0);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);

		//glDepthFunc(GL_EQUAL);
		//glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}
}

