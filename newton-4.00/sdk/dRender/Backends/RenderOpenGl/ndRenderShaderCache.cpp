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
#include "ndRenderShaderCache.h"

ndRenderShaderCache::ndRenderShaderCache(void)
{
	ndMemSet(m_shaders, GLuint(0), sizeof(m_shaders)/sizeof(GLuint));
	CreateAllEffects();
}

ndRenderShaderCache::~ndRenderShaderCache(void)
{
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

bool ndRenderShaderCache::CreateAllEffects()
{
	m_skyBox = CreateShaderEffect(m_skyBoxVertex, m_skyBoxPixel);
	m_shadowMaps = CreateShaderEffect(m_shadowMapVertex, m_shadowMapPixel);
	m_diffuseEffect = CreateShaderEffect(m_directionalDiffuseVertex, m_directionalDiffusePixel);
	m_diffuseShadowEffect = CreateShaderEffect(m_directionalDiffuseShadowVertex, m_directionalDiffuseShadowPixel);
	
	//m_wireFrame = CreateShaderEffect("WireFrame", "FlatShaded");
	//m_flatShaded = CreateShaderEffect("FlatShaded", "FlatShaded");
	//m_colorPoint = CreateShaderEffect("ColorPoint", "FlatShaded");
	//m_zBufferDebug = CreateShaderEffect("zBufferDebug", "zBufferDebug");
	//m_texturedDecal = CreateShaderEffect ("TextureDecal", "TextureDecal");
	
	//m_diffuseDebrisEffect = CreateShaderEffect("DirectionalDebriDiffuse", "DirectionalDebriDiffuse");
	//m_skinningDiffuseEffect = CreateShaderEffect ("SkinningDirectionalDiffuse", "DirectionalDiffuse");
	//m_diffuseIntanceEffect = CreateShaderEffect ("DirectionalDiffuseInstance", "DirectionalDiffuse");
	//m_thickPoints = CreateShaderEffect("ThickPoint", "ThickPoint", "ThickPoint");
	//m_spriteSpheres = CreateShaderEffect("DirectionalDiffuseSprite", "DirectionalDiffuseSprite", "DirectionalDiffuseSprite");
	//
	//// shadow programs
	//m_shadowMaps = CreateShaderEffect("ShadowMap", "ShadowMap");
	//m_diffuseShadowEffect = CreateShaderEffect("DirectionalDiffuseShadow", "DirectionalDiffuseShadow");

	return true;
}

//void ndRenderShaderCache::LoadShaderCode (const char* const filename, char* const buffer)
//{
//	ndAssert(0);
//	//size_t size;
//	//FILE* file;
//	//char fullPathName[2048];
//	//
//	//ndGetWorkingFileName (filename, fullPathName);
//	//
//	//file = fopen (fullPathName, "rb");
//	//ndAssert (file);
//	//fseek (file, 0, SEEK_END); 
//	//
//	//size_t error = 0;
//	//size = size_t(ftell (file));
//	//fseek (file, 0, SEEK_SET); 
//	//error = fread (buffer, size_t(size), 1, file);
//	//
//	//ndAssert (error);
//	//fclose (file);
//	//buffer[size] = 0;
//	//buffer[size + 1] = 0;
//}

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
	}
	glAttachShader(program, pixelShader);
	
	glLinkProgram(program);
	glGetProgramiv(program, GL_LINK_STATUS, &state);   
	if (state != GL_TRUE ) 
	{
		GLsizei length;  
		glGetProgramInfoLog(program, sizeof(errorLog), &length, errorLog);
		ndTrace((errorLog));
	}
	
	glValidateProgram(program);
	glGetProgramiv(program,  GL_VALIDATE_STATUS, &state);   
	ndAssert (state == GL_TRUE);
	
	if (geometryShader)
	{
		glDeleteShader(geometryShader);
	}

	glDeleteShader(pixelShader);
	glDeleteShader(vertexShader);
	return program;
}
