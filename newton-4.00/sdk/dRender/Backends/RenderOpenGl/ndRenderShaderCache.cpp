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
#include "ndRenderPassShadows.h"
#include "ndRenderTextureImage.h"
#include "ndRenderPassEnvironment.h"
#include "ndRenderPrimitiveImplement.h"
#include "ndRenderPassShadowsImplement.h"

ndRenderShaderCache::ndRenderShaderCache(void)
{
	ndMemSet(m_shaders, GLuint(0), sizeof(m_shaders) / sizeof(GLuint));
	CreateAllEffects();
}

ndRenderShaderCache::~ndRenderShaderCache(void)
{
}

bool ndRenderShaderCache::CreateAllEffects()
{
	// unique render effect
	m_skyBoxEffect = CreateShaderEffect(m_skyBoxVertex, m_skyBoxPixel);
	m_setZbufferEffect = CreateShaderEffect(m_setZbufferVertex, m_doNothingPixel);
	m_diffuseEffect = CreateShaderEffect(m_directionalDiffuseVertex, m_directionalDiffusePixel);
	m_diffuseTransparentEffect = CreateShaderEffect(m_directionalDiffuseVertex, m_directionalDiffuseTransparentPixel);
	m_debugFlatShadedDiffuseEffect = CreateShaderEffect(m_debugFlatShadedDiffuseVertex, m_debugFlatShadedDiffusePixel);

	// generation of shadow map render effects
	m_generateShadowMapsEffect = CreateShaderEffect(m_generateShadowMapVertex, m_doNothingPixel);
	m_generateShadowMapsSkinEffect = CreateShaderEffect(m_generateShadowMapSkinVertex, m_doNothingPixel);
	m_generateInstancedShadowMapsEffect = CreateShaderEffect(m_generateInstancedShadowMapVertex, m_doNothingPixel);

	// basic primitive effect, a mesh with a Blinn ilumination, 
	// with environment reflection, casting and recieving shadows
	m_diffuseShadowEffect = CreateShaderEffect(m_directionalDiffuseShadowVertex, m_directionalDiffuseShadowPixel);
	m_diffuseShadowIntanceEffect = CreateShaderEffect(m_directionalDiffuseInstanceVertex, m_directionalDiffuseShadowPixel);
	m_diffuseShadowSkinEffect = CreateShaderEffect(m_directionalDiffuseShadowSkinVertex, m_directionalDiffuseShadowPixel);

	return true;
}

GLuint ndRenderShaderCache::CreateShaderEffect (const char* const vertexShaderCode, const char* const pixelShaderCode, const char* const geometryShaderCode)
{
	GLint state;
	char errorLog[1024 * 16];

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
