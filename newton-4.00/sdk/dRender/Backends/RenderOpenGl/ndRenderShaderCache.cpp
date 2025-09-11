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
#include "ndRenderOpenGlUtil.h"
#include "ndRenderSceneCamera.h"
#include "ndRenderShaderCache.h"
#include "ndRenderTextureImage.h"
#include "ndRenderPassShadowsImplement.h"
#include "ndRenderPrimitiveMeshImplement.h"

ShaderBlockBlock::~ShaderBlockBlock()
{
}

ndRenderShaderCache::ndRenderShaderCache(void)
{
	ndMemSet(m_shaders, GLuint(0), sizeof(m_shaders)/sizeof(GLuint));
	CreateAllEffects();
}

ndRenderShaderCache::~ndRenderShaderCache(void)
{
}

// *********************************************************************
// 
// *********************************************************************
bool ndRenderShaderCache::CreateAllEffects()
{
	m_skyBoxEffect = CreateShaderEffect(m_skyBoxVertex, m_skyBoxPixel);
	m_setZbufferEffect = CreateShaderEffect(m_setZbufferVertex, m_doNothingPixel);
	m_shadowMapsEffect = CreateShaderEffect(m_shadowMapVertex, m_doNothingPixel);
	m_diffuseEffect = CreateShaderEffect(m_directionalDiffuseVertex, m_directionalDiffusePixel);
	m_diffuseIntanceEffect = CreateShaderEffect(m_directionalDiffuseInstanceVertex, m_directionalDiffusePixel);
	m_debugDiffuseSolidEffect = CreateShaderEffect(m_debugFlatDiffuseVertex, m_debugFlatDiffusePixel);
	m_diffuseShadowEffect = CreateShaderEffect(m_directionalDiffuseShadowVertex, m_directionalDiffuseShadowPixel);
	m_diffuseTransparentEffect = CreateShaderEffect(m_directionalDiffuseVertex, m_directionalDiffuseTransparentPixel);

	//m_wireFrame = CreateShaderEffect("WireFrame", "FlatShaded");
	//m_flatShaded = CreateShaderEffect("FlatShaded", "FlatShaded");
	//m_colorPoint = CreateShaderEffect("ColorPoint", "FlatShaded");
	//m_texturedDecal = CreateShaderEffect ("TextureDecal", "TextureDecal");
	//m_diffuseDebrisEffect = CreateShaderEffect("DirectionalDebriDiffuse", "DirectionalDebriDiffuse");
	//m_skinningDiffuseEffect = CreateShaderEffect ("SkinningDirectionalDiffuse", "DirectionalDiffuse");
	//m_diffuseIntanceEffect = CreateShaderEffect ("DirectionalDiffuseInstance", "DirectionalDiffuse");
	//m_thickPoints = CreateShaderEffect("ThickPoint", "ThickPoint", "ThickPoint");
	//m_spriteSpheres = CreateShaderEffect("DirectionalDiffuseSprite", "DirectionalDiffuseSprite", "DirectionalDiffuseSprite");

	return true;
}

GLuint ndRenderShaderCache::CreateShaderEffect (const char* const vertexShaderCode, const char* const pixelShaderCode, const char* const geometryShaderCode)
{
	GLint state;
	char errorLog[GL_INFO_LOG_LENGTH];

	GLuint program = glCreateProgram();
	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	
	glShaderSource(vertexShader, 1, &vertexShaderCode, nullptr);
	glCompileShader(vertexShader);
	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &state); 
	if (state != GL_TRUE ) 
	{
		GLsizei length;  
		glGetShaderInfoLog(vertexShader, sizeof (errorLog), &length, errorLog);
		ndTrace ((errorLog));
		ndAssert(0);
	}
	glAttachShader(program, vertexShader);
	
	// load and compile geometry shader, if any
	GLuint geometryShader = 0;
	if (geometryShaderCode)
	{
		ndAssert(0);
		//snprintf(tmpName, sizeof(tmpName), "shaders/%s.gs", geometryShaderCode);
		//LoadShaderCode(tmpName, buffer);
		//geometryShader = glCreateShader(GL_GEOMETRY_SHADER);
	
		//glShaderSource(geometryShader, 1, &vPtr, nullptr);
		//glCompileShader(geometryShader);
		//glGetShaderiv(geometryShader, GL_COMPILE_STATUS, &state);
		//if (state != GL_TRUE)
		//{
		//	GLsizei length;
		//	glGetShaderInfoLog(geometryShader, sizeof(buffer), &length, errorLog);
		//	ndTrace((errorLog));
		//}
		//glAttachShader(program, geometryShader);
	}
	
	GLuint pixelShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(pixelShader, 1, &pixelShaderCode, nullptr);
	glCompileShader(pixelShader);
	glGetShaderiv(pixelShader, GL_COMPILE_STATUS, &state); 
	if (state != GL_TRUE ) 
	{
		GLsizei length;  
		glGetShaderInfoLog(pixelShader, sizeof(errorLog), &length, errorLog);
		ndTrace((errorLog));
		ndAssert(0);
	}
	glAttachShader(program, pixelShader);
	
	glLinkProgram(program);
	glGetProgramiv(program, GL_LINK_STATUS, &state);   
	if (state != GL_TRUE ) 
	{
		GLsizei length;  
		glGetProgramInfoLog(program, sizeof(errorLog), &length, errorLog);
		ndTrace((errorLog));
		ndAssert(0);
	}
	
	glValidateProgram(program);
	glGetProgramiv(program,  GL_VALIDATE_STATUS, &state);   
	if (state != GL_TRUE)
	{
		GLsizei length;
		glGetProgramInfoLog(program, sizeof(errorLog), &length, errorLog);
		ndTrace((errorLog));
		ndAssert(0);
	}
	
	if (geometryShader)
	{
		glDeleteShader(geometryShader);
	}

	glDeleteShader(pixelShader);
	glDeleteShader(vertexShader);
	return program;
}

void ndRenderShaderCache::Cleanup()
{
	for (ndInt32 i = 0; i < ndInt32(sizeof(m_shaders) / sizeof(m_shaders[0])); ++i)
	{
		if (m_shaders[i])
		{
			glDeleteProgram(m_shaders[i]);
			m_shaders[i] = 0;
		}
	}
}

// *********************************************************************
// 
// *********************************************************************
void SetZbufferCleanBlock::GetShaderParameters(ndRenderPrimitiveMeshImplement* const self)
{
	GLuint shader = self->m_context->m_shaderCache->m_setZbufferEffect;
	glUseProgram(shader);
	viewModelProjectionMatrix = glGetUniformLocation(shader, "viewModelProjectionMatrix");
	glUseProgram(0);
}

void SetZbufferCleanBlock::Render(const ndRenderPrimitiveMeshImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const
{
	const ndSharedPtr<ndRenderSceneCamera>& camera = render->GetCamera();

	//const ndMatrix modelViewProjectionMatrixMatrix(modelMatrix * camera->m_invViewMatrix * camera->m_projectionMatrix);
	const ndMatrix modelViewProjectionMatrixMatrix(modelMatrix * camera->m_invViewRrojectionMatrix);
	const glMatrix glViewModelProjectionMatrix(modelViewProjectionMatrixMatrix);

	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);
	glDisable(GL_BLEND);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);

	GLuint shader = self->m_context->m_shaderCache->m_setZbufferEffect;
	glUseProgram(shader);

	glUniformMatrix4fv(viewModelProjectionMatrix, 1, false, &glViewModelProjectionMatrix[0][0]);

	glBindVertexArray(self->m_vertextArrayBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self->m_indexBuffer);

	glDrawElements(GL_TRIANGLES, self->m_indexCount, GL_UNSIGNED_INT, (void*)0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}
