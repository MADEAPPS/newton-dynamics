/* Copyright (c) <2003-2016> <Newton Game Dynamics>
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


ShaderPrograms::ShaderPrograms(void)
{
	m_solidColor = 0;
	m_decalEffect = 0;
	m_diffuseEffect = 0;
	m_skinningDiffuseEffect = 0;
}

ShaderPrograms::~ShaderPrograms(void)
{
}

bool ShaderPrograms::CreateAllEffects()
{

	m_solidColor = CreateShaderEffect ("SolidColor");
	m_decalEffect = CreateShaderEffect ("TextureDecal");
	m_diffuseEffect = CreateShaderEffect ("DirectionalDiffuse");
	m_skinningDiffuseEffect = CreateShaderEffect ("SkinningDirectionalDiffuse");


//glUseProgram(m_skinningDiffuseEffect);
//GLuint matrixPalleteID; 
//matrixPalleteID = glGetUniformLocation(m_skinningDiffuseEffect, "matrixPallete1"); 

	if (m_solidColor && m_decalEffect && m_diffuseEffect && m_skinningDiffuseEffect) {
		return true;
	} else {
		m_solidColor = 0;
		m_decalEffect = 0;
		m_diffuseEffect = 0;
		m_skinningDiffuseEffect = 0;
		return false;
	}

}

ShaderPrograms& ShaderPrograms::GetCache()
{
	static ShaderPrograms shaderCache;
	return shaderCache;
}


void ShaderPrograms::LoadShaderCode (const char* filename, char *buffer)
{
	int size;
	FILE* file;
	char fullPathName[2048];

	GetWorkingFileName (filename, fullPathName);

	file = fopen (fullPathName, "rb");
	fseek (file, 0, SEEK_END); 
	
	size = ftell (file);
	fseek (file, 0, SEEK_SET); 
	fread (buffer, size, 1, file);
	fclose (file);
	buffer[size] = 0;
	buffer[size + 1] = 0;
}

GLuint ShaderPrograms::CreateShaderEffect (const char* name)
{
	GLint state;
	GLuint program;
	GLuint pixelShader; 
	GLuint vertexShader; 
	
	
	const char *vPtr;
	char tmpName[128];
	char buffer[1024 * 64];

	vPtr = buffer;
	program = glCreateProgram();

	sprintf (tmpName, "shaders/%s.vs", name);
	LoadShaderCode (tmpName, buffer);
	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertexShader, 1, &vPtr, NULL);
	glCompileShader(vertexShader);
	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &state); 
	if (state != GL_TRUE ) {
		GLsizei length;  
		glGetShaderInfoLog(vertexShader, sizeof (buffer), &length, buffer);
	}
	glAttachShader(program, vertexShader);


	sprintf (tmpName, "shaders/%s.ps", name);
	LoadShaderCode (tmpName, buffer);
	pixelShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(pixelShader, 1, &vPtr, NULL);
	glCompileShader(pixelShader);
	glGetShaderiv(pixelShader, GL_COMPILE_STATUS, &state); 
	if (state != GL_TRUE ) {
		GLsizei length;  
		glGetShaderInfoLog(pixelShader, sizeof (buffer), &length, buffer);
	}
	glAttachShader(program, pixelShader);

	glLinkProgram(program);
	glGetProgramiv(program, GL_LINK_STATUS, &state);   
	_ASSERTE (state == GL_TRUE);
	return program;

}
