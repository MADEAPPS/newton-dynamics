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
#include "ndRenderContext.h"
#include "ndRenderTexture.h"
#include "ndRenderOpenGlUtil.h"
#include "ndRenderSceneCamera.h"
#include "ndRenderShaderCache.h"
#include "ndRenderTextureImage.h"
#include "ndEnvironmentRenderPassImplement.h"

ndEnvironmentRenderPassImplement::ndEnvironmentRenderPassImplement(ndRenderContext* const context)
	:ndClassAlloc()
	,m_context(context)
{
	GLfloat vertices[] =
	{
	  -1.0f, -1.0f, 0.999f,
	   1.0f, -1.0f, 0.999f,
	   1.0f,  1.0f, 0.999f,
	  -1.0f,  1.0f, 0.999f
	};

	ndInt32 indices[] =
	{
		0, 1, 3,
		3, 1, 2,
	};

	glGenVertexArrays(1, &m_vertextArrayBuffer);
	glBindVertexArray(m_vertextArrayBuffer);
	
	glGenBuffers(1, &m_vertexBuffer); //m_vbo
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
	
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), &vertices[0], GL_STATIC_DRAW);
	
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*)0);
	
	glGenBuffers(1, &m_indexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), &indices[0], GL_STATIC_DRAW);
	
	glBindVertexArray(0);
	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	
	GLuint shader = m_context->m_shaderCache->m_skyBox;
	glUseProgram(shader);
	m_invViewModelProjectionTextureMatrix = glGetUniformLocation(shader, "invViewModelProjectionTextureMatrix");
	glUseProgram(0);
	
	//SetShadowMode(false);
}

ndEnvironmentRenderPassImplement::~ndEnvironmentRenderPassImplement()
{
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

void ndEnvironmentRenderPassImplement::RenderScene(const ndRenderSceneCamera* const camera, const ndRenderTexture* const texture)
{
	ndMatrix viewMatrix(camera->m_viewMatrix);
	viewMatrix.m_posit = ndVector::m_wOne;
	viewMatrix.m_posit.m_y = ndFloat32(-10.0f);
	glMatrix invViewModelProjectionTextureMatrix(camera->m_invProjectionMatrix * viewMatrix);

	//GLint viewport[4];
	//glGetIntegerv(GL_VIEWPORT, viewport);
	//if (memcmp(previousViewport, viewport, sizeof(viewport))) {
	//	printf("Viewport set to (%d, %d, %d, %d)\n", viewport[0], viewport[1], viewport[2], viewport[3]);
	//	memcpy(previousViewport, viewport, sizeof(viewport));
	//}

	glDisable(GL_SCISSOR_TEST);
	GLuint shader = m_context->m_shaderCache->m_skyBox;
	glUseProgram(shader);
	glUniformMatrix4fv(m_invViewModelProjectionTextureMatrix, 1, false, &invViewModelProjectionTextureMatrix[0][0]);
	
	ndRenderTextureCubeMapImage* const glTexture = (ndRenderTextureCubeMapImage*)texture;
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_CUBE_MAP, glTexture->m_texture);
	glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);
	
	glBindVertexArray(m_vertextArrayBuffer);
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
	
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	
	glUseProgram(0);
}

