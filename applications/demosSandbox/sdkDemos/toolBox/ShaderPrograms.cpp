/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "toolbox_stdafx.h"
#include "ShaderPrograms.h"

ShaderPrograms::ShaderPrograms(void)
{
	m_solidColor = 0;
	m_decalEffect = 0;
	m_diffuseEffect = 0;
	m_diffuseIntanceEffect = 0;
	m_skinningDiffuseEffect = 0;
	m_diffuseNoTextureEffect = 0;
}

ShaderPrograms::~ShaderPrograms(void)
{
	if (m_solidColor) {
		glDeleteShader(m_solidColor);
	}
	if (m_decalEffect) {
		glDeleteShader(m_decalEffect);
	}
	if (m_diffuseEffect) {
		glDeleteShader(m_diffuseEffect);
	}
	if (m_diffuseNoTextureEffect) {
		glDeleteShader(m_diffuseNoTextureEffect);
	}
	if (m_skinningDiffuseEffect) {
		glDeleteShader(m_skinningDiffuseEffect);
	}
	if (m_diffuseIntanceEffect) {
		glDeleteShader(m_diffuseIntanceEffect);
	}
}

bool ShaderPrograms::CreateAllEffects()
{
	m_solidColor = CreateShaderEffect ("TextureDecal", "TextureDecal");
	m_diffuseEffect = CreateShaderEffect ("DirectionalDiffuse", "DirectionalDiffuse");
	m_skinningDiffuseEffect = CreateShaderEffect ("SkinningDirectionalDiffuse", "DirectionalDiffuse");
	m_diffuseNoTextureEffect = CreateShaderEffect ("DirectionalDiffuse", "DirectionalDiffuseNoTexture");

	m_diffuseIntanceEffect = CreateShaderEffect ("DirectionalDiffuseInstance", "DirectionalDiffuseInstance");
	return true;
}

void ShaderPrograms::LoadShaderCode (const char* const filename, char* const buffer)
{
	int size;
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

GLuint ShaderPrograms::CreateShaderEffect (const char* const vertexShaderName, const char* const pixelShaderName)
{
	GLint state;
	char tmpName[256];
	char buffer[1024 * 64];
	char errorLog[GL_INFO_LOG_LENGTH];

	const char* const vPtr = buffer;
	GLuint program = glCreateProgram();

	sprintf (tmpName, "shaders/%s.vs", vertexShaderName);
	LoadShaderCode (tmpName, buffer);
	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);

	glShaderSource(vertexShader, 1, &vPtr, NULL);
	glCompileShader(vertexShader);
	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &state); 
	if (state != GL_TRUE ) {
		GLsizei length;  
		glGetShaderInfoLog(vertexShader, sizeof (buffer), &length, errorLog);
		dTrace ((errorLog));
	}
	glAttachShader(program, vertexShader);


	sprintf (tmpName, "shaders/%s.ps", pixelShaderName);
	LoadShaderCode (tmpName, buffer);
	GLuint pixelShader = glCreateShader(GL_FRAGMENT_SHADER);

	glShaderSource(pixelShader, 1, &vPtr, NULL);
	glCompileShader(pixelShader);
	glGetShaderiv(pixelShader, GL_COMPILE_STATUS, &state); 
	if (state != GL_TRUE ) {
		GLsizei length;  
		glGetShaderInfoLog(vertexShader, sizeof (buffer), &length, errorLog);
		dTrace((errorLog));
	}
	glAttachShader(program, pixelShader);

	glLinkProgram(program);
	glGetProgramiv(program, GL_LINK_STATUS, &state);   
	if (state != GL_TRUE ) {
		GLsizei length;  
		glGetProgramInfoLog(program, sizeof (buffer), &length, errorLog);
		dTrace((errorLog));
	}
	
	glValidateProgram(program);
	glGetProgramiv(program,  GL_VALIDATE_STATUS, &state);   
	dAssert (state == GL_TRUE);

	glDeleteShader(pixelShader);
	glDeleteShader(vertexShader);
	return program;
}
