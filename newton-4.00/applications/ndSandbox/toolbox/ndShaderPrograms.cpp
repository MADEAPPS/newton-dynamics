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
#include "ndShaderPrograms.h"

ndShaderPrograms::ndShaderPrograms(void)
{
	m_skyBox = 0;
	m_wireFrame = 0;
	m_flatShaded = 0;
	m_decalEffect = 0;
	m_texturedDecal = 0;
	m_diffuseEffect = 0;
	m_spriteSpheres = 0;
	m_diffuseDebrisEffect = 0;
	m_diffuseIntanceEffect = 0;
	m_skinningDiffuseEffect = 0;
	m_diffuseNoTextureEffect = 0;
}

ndShaderPrograms::~ndShaderPrograms(void)
{
	if (m_skyBox)
	{
		glDeleteShader(m_skyBox);
	}
	if (m_wireFrame)
	{
		glDeleteShader(m_wireFrame);
	}
	if (m_flatShaded)
	{
		glDeleteShader(m_flatShaded);
	}
	if (m_texturedDecal) 
	{
		glDeleteShader(m_texturedDecal);
	}
	if (m_decalEffect) 
	{
		glDeleteShader(m_decalEffect);
	}
	if (m_diffuseEffect) 
	{
		glDeleteShader(m_diffuseEffect);
	}
	if (m_spriteSpheres)
	{
		glDeleteShader(m_spriteSpheres);
	}
	if (m_diffuseDebrisEffect)
	{
		glDeleteShader(m_diffuseDebrisEffect);
	}
	if (m_diffuseNoTextureEffect) 
	{
		glDeleteShader(m_diffuseNoTextureEffect);
	}
	if (m_skinningDiffuseEffect) 
	{
		glDeleteShader(m_skinningDiffuseEffect);
	}
	if (m_diffuseIntanceEffect) 
	{
		glDeleteShader(m_diffuseIntanceEffect);
	}
}

bool ndShaderPrograms::CreateAllEffects()
{
	m_skyBox = CreateShaderEffect("SkyBox", "SkyBox");
	m_wireFrame = CreateShaderEffect("WireFrame", "FlatShaded");
	m_flatShaded = CreateShaderEffect("FlatShaded", "FlatShaded");
	m_texturedDecal = CreateShaderEffect ("TextureDecal", "TextureDecal");

	m_diffuseEffect = CreateShaderEffect ("DirectionalDiffuse", "DirectionalDiffuse");
	m_diffuseDebrisEffect = CreateShaderEffect("DirectionalDebriDiffuse", "DirectionalDebriDiffuse");
	m_skinningDiffuseEffect = CreateShaderEffect ("SkinningDirectionalDiffuse", "DirectionalDiffuse");
	m_diffuseNoTextureEffect = CreateShaderEffect ("DirectionalDiffuse", "DirectionalDiffuseNoTexture");
	m_diffuseIntanceEffect = CreateShaderEffect ("DirectionalDiffuseInstance", "DirectionalDiffuse");

	m_spriteSpheres = CreateShaderEffect("DirectionalDiffuseSprite", "DirectionalDiffuseSprite", "DirectionalDiffuseSprite");

	return true;
}

void ndShaderPrograms::LoadShaderCode (const char* const filename, char* const buffer)
{
	ndInt32 size;
	FILE* file;
	char fullPathName[2048];

	dGetWorkingFileName (filename, fullPathName);

	file = fopen (fullPathName, "rb");
	dAssert (file);
	fseek (file, 0, SEEK_END); 
	
	size = ftell (file);
	fseek (file, 0, SEEK_SET); 
	size_t error = fread (buffer, size, 1, file);
	// for GCC shit
	dAssert (error); error = 0;
	fclose (file);
	buffer[size] = 0;
	buffer[size + 1] = 0;
}

GLuint ndShaderPrograms::CreateShaderEffect (const char* const vertexShaderName, const char* const pixelShaderName, const char* const geometryShaderName)
{
	GLint state;
	char tmpName[256];
	char buffer[1024 * 64];
	char errorLog[GL_INFO_LOG_LENGTH];

	const char* const vPtr = buffer;
	GLuint program = glCreateProgram();

	// load and compile vertex shader
	sprintf (tmpName, "shaders/%s.vtx", vertexShaderName);
	LoadShaderCode (tmpName, buffer);
	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);

	glShaderSource(vertexShader, 1, &vPtr, nullptr);
	glCompileShader(vertexShader);
	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &state); 
	if (state != GL_TRUE ) 
	{
		GLsizei length;  
		glGetShaderInfoLog(vertexShader, sizeof (buffer), &length, errorLog);
		dTrace ((errorLog));
	}
	glAttachShader(program, vertexShader);

	// load and compile geometry shader, if any
	GLuint geometryShader = 0;
	if (geometryShaderName)
	{
		sprintf(tmpName, "shaders/%s.gs", geometryShaderName);
		LoadShaderCode(tmpName, buffer);
		geometryShader = glCreateShader(GL_GEOMETRY_SHADER);

		glShaderSource(geometryShader, 1, &vPtr, nullptr);
		glCompileShader(geometryShader);
		glGetShaderiv(geometryShader, GL_COMPILE_STATUS, &state);
		if (state != GL_TRUE)
		{
			GLsizei length;
			glGetShaderInfoLog(geometryShader, sizeof(buffer), &length, errorLog);
			dTrace((errorLog));
		}
		glAttachShader(program, geometryShader);
	}

	sprintf (tmpName, "shaders/%s.ps", pixelShaderName);
	LoadShaderCode (tmpName, buffer);
	GLuint pixelShader = glCreateShader(GL_FRAGMENT_SHADER);

	glShaderSource(pixelShader, 1, &vPtr, nullptr);
	glCompileShader(pixelShader);
	glGetShaderiv(pixelShader, GL_COMPILE_STATUS, &state); 
	if (state != GL_TRUE ) 
	{
		GLsizei length;  
		glGetShaderInfoLog(vertexShader, sizeof (buffer), &length, errorLog);
		dTrace((errorLog));
	}
	glAttachShader(program, pixelShader);

	glLinkProgram(program);
	glGetProgramiv(program, GL_LINK_STATUS, &state);   
	if (state != GL_TRUE ) 
	{
		GLsizei length;  
		glGetProgramInfoLog(program, sizeof (buffer), &length, errorLog);
		dTrace((errorLog));
	}
	
	glValidateProgram(program);
	glGetProgramiv(program,  GL_VALIDATE_STATUS, &state);   
	dAssert (state == GL_TRUE);

	glDeleteShader(pixelShader);
	if (geometryShader)
	{
		glDeleteShader(geometryShader);
	}
	glDeleteShader(vertexShader);
	return program;
}
