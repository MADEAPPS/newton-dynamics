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
#include "ndRenderSceneNode.h"
#include "ndRenderShaderCache.h"
#include "ndRenderPassShadowsImplement.h"

#define ND_SHADOW_MAP_RESOLUTION (1024 * 4)

ndRenderPassShadowsImplement::ndRenderPassShadowsImplement(ndRenderContext* const context)
	:ndClassAlloc()
	,m_context(context)
{
	m_width = ND_SHADOW_MAP_RESOLUTION;
	m_height = ND_SHADOW_MAP_RESOLUTION;

	glGenFramebuffers(1, &m_frameBufferObject);

	glBindFramebuffer(GL_FRAMEBUFFER, m_frameBufferObject);
	// Disable writes to the color buffer
	glDrawBuffer(GL_NONE);
	glReadBuffer(GL_NONE);
	
	// Create the depth buffer textures, onle one 4 x 4 tile
	glGenTextures(1, &m_shadowMapTexture);
	glBindTexture(GL_TEXTURE_2D, m_shadowMapTexture);
	
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, m_width * 2, m_height * 2, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_shadowMapTexture, 0);
	
	GLuint shader = m_context->m_shaderCache->m_shadowMaps;
	glUseProgram(shader);
	m_modelProjectionMatrixLocation = GLuint(glGetUniformLocation(shader, "viewModelProjectionMatrix"));
	glUseProgram(0);
	
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	ndAssert(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
	
	m_viewPortTiles[0] = ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.5f), ndFloat32(0.5f));
	m_viewPortTiles[1] = ndVector(ndFloat32(0.5f), ndFloat32(0.0f), ndFloat32(0.5f), ndFloat32(0.5f));
	m_viewPortTiles[2] = ndVector(ndFloat32(0.0f), ndFloat32(0.5f), ndFloat32(0.5f), ndFloat32(0.5f));
	m_viewPortTiles[3] = ndVector(ndFloat32(0.5f), ndFloat32(0.5f), ndFloat32(0.5f), ndFloat32(0.5f));
	
	m_lightProjectToTextureSpace = ndGetIdentityMatrix();
	m_lightProjectToTextureSpace[0][0] = ndFloat32(0.5f);
	m_lightProjectToTextureSpace[1][1] = ndFloat32(0.5f);
	m_lightProjectToTextureSpace[2][2] = ndFloat32(0.5f);
	m_lightProjectToTextureSpace.m_posit = ndVector(ndFloat32(0.5f), ndFloat32(0.5f), ndFloat32(0.5f), ndFloat32(1.0f));
}

ndRenderPassShadowsImplement::~ndRenderPassShadowsImplement()
{
	if (m_shadowMapTexture != 0)
	{
		glDeleteTextures(1, &m_shadowMapTexture);
	}

	if (m_frameBufferObject != 0)
	{
		glDeleteFramebuffers(1, &m_frameBufferObject);
	}
}

//void ndRenderPassShadowsImplement::RenderScene(ndFloat32 timestep)
//{
//	//m_owner->m_context->SetCollorPassRenderStates();
//	//
//	//const ndMatrix globalMatrix(ndGetIdentityMatrix());
//	//ndList<ndSharedPtr<ndRenderSceneNode>>& scene = m_owner->m_scene;
//	//for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = scene.GetFirst(); node; node = node->GetNext())
//	//{
//	//	ndRenderSceneNode* const sceneNode = *node->GetInfo();
//	//	sceneNode->Render(sceneNode->m_owner, timestep, globalMatrix);
//	//}
//}
