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
#include "ndSkyBox.h"
#include "ndDemoCamera.h"
#include "ndTargaToOpenGl.h"

ndSkyBox::ndSkyBox(GLuint shader)
	:ndDemoEntity(ndGetIdentityMatrix(), nullptr)
	,m_textureMatrix(ndGetIdentityMatrix())
	,m_shader(shader)
	,m_indexBuffer(0)
	,m_vertexBuffer(0)
	,m_texturecubemap(0)
	,m_vertextArrayBuffer(0)
	,m_invViewModelProjectionTextureMatrix(0)
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

	m_textureMatrix[1][1] = ndFloat32(-1.0f);

	//m_texturecubemap = LoadCubeMapTexture(
	//	"NewtonSky0003.png", "NewtonSky0001.png",
	//	"NewtonSky0006.png", "NewtonSky0005.png",
	//	"NewtonSky0002.png", "NewtonSky0004.png");

	m_texturecubemap = LoadCubeMapTexture(
		"skyBox_front.png", "skyBox_back.png",
		"skyBox_botton.png", "skyBox_top.png",
		"skyBox_left.png", "skyBox_right.png");

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

	glUseProgram(m_shader);
	m_invViewModelProjectionTextureMatrix = glGetUniformLocation(m_shader, "invViewModelProjectionTextureMatrix");
	glUseProgram(0);

	SetShadowMode(false);
}

ndSkyBox::~ndSkyBox()
{
	if (m_texturecubemap)
	{
		ReleaseTexture(m_texturecubemap);
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

void ndSkyBox::Render(ndFloat32, ndDemoEntityManager* const scene, const ndMatrix&) const
{
return;
	//glDepthMask(GL_FALSE);

	ndDemoCamera* const camera = scene->GetCamera();
	//glMatrix invViewModelProjectionTextureMatrix(camera->GetInvViewProjectionMatrix().Inverse4x4() * m_textureMatrix);
	glMatrix invViewModelProjectionTextureMatrix(camera->GetInvProjectionMatrix() * camera->GetViewMatrix() * m_textureMatrix);
	
	glUseProgram(m_shader);
	glUniformMatrix4fv(m_invViewModelProjectionTextureMatrix, 1, false, &invViewModelProjectionTextureMatrix[0][0]);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_CUBE_MAP, m_texturecubemap);
	glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);

	glBindVertexArray(m_vertextArrayBuffer);
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);

	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glUseProgram(0);
	//glDepthMask(GL_TRUE);
}
