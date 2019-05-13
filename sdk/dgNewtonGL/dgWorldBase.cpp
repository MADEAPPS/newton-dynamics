/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
*
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
*
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
*
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "dgNewtonPluginStdafx.h"
#include "dgWorldBase.h"

//char dgWorldBase::m_shaderDirectory[1024];

// This is an example of an exported function.
dgWorldPlugin* GetPlugin(dgWorld* const world, dgMemoryAllocator* const allocator)
{
	const GLubyte* const version = glGetString(GL_SHADING_LANGUAGE_VERSION);
	if (strcmp((char*)version, "4.50") < 0) {
		return NULL;
	}

	GLuint shaderArray[1024];
	int shaderCount = dgSolver::CompileAllShaders(shaderArray);
	if (!shaderCount) {
		return NULL;
	}

	static dgWorldBase module(world, allocator);
	module.m_score = 1;
#ifdef _DEBUG
	sprintf(module.m_hardwareDeviceName, "Newton opengl_d");
#else
	sprintf(module.m_hardwareDeviceName, "Newton opengl");
#endif

	module.SetShaders(shaderCount, shaderArray);
	return &module;
}

dgWorldBase::dgWorldBase(dgWorld* const world, dgMemoryAllocator* const allocator)
	:dgWorldPlugin(world, allocator)
	,dgSolver(world, allocator)
{
}

dgWorldBase::~dgWorldBase()
{
}


const char* dgWorldBase::GetId() const
{
	return m_hardwareDeviceName;
}

dgInt32 dgWorldBase::GetScore() const
{
	return m_score;
}

void dgWorldBase::CalculateJointForces(const dgBodyCluster& cluster, dgBodyInfo* const bodyArray, dgJointInfo* const jointArray, dgFloat32 timestep)
{
	dgSolver::CalculateJointForces(cluster, bodyArray, jointArray, timestep);
}

/*
dgShaderPrograms::dgShaderPrograms(void)
{
	m_testShader = 0;
}

dgShaderPrograms::~dgShaderPrograms(void)
{
	if (m_testShader) {
		glDeleteShader(m_testShader);
	}
}

bool dgShaderPrograms::LoadAllShaders()
{
	m_testShader = LoadComputeShader("dgTestShader.glsl");

	return true;
}

void dgShaderPrograms::LoadShaderCode(const char* const shaderName, char* const buffer)
{
	int size;
	FILE* file;
	char fullPathName[2048];

	sprintf(fullPathName, "%s%s", dgWorldBase::m_shaderDirectory, shaderName);

	file = fopen(fullPathName, "rb");
	dgAssert(file);
	fseek(file, 0, SEEK_END);

	size = ftell(file);
	fseek(file, 0, SEEK_SET);
	size_t error = fread(buffer, size, 1, file);
	// for GCC shit
	dgAssert(error); error = 0;
	fclose(file);
}

GLuint dgShaderPrograms::LoadComputeShader(const char* const shaderName)
{
	GLint state;
	GLchar shaderSource[1024 * 64];
	char errorLog[GL_INFO_LOG_LENGTH];

	memset(shaderSource, 0, sizeof(shaderSource));
	LoadShaderCode(shaderName, shaderSource);

	GLuint program = glCreateProgram();
	GLuint computeShader = glCreateShader(GL_COMPUTE_SHADER);

	GLchar* ptr[2];
	ptr[0] = shaderSource;
	ptr[1] = NULL;
	glShaderSource(computeShader, 1, ptr, NULL);
//	error = glGetError();
//	dgAssert(error == GL_NO_ERROR);

	glCompileShader(computeShader);
	glGetShaderiv(computeShader, GL_COMPILE_STATUS, &state);
	if (state != GL_TRUE) {
		GLsizei length;
		glGetShaderInfoLog(computeShader, sizeof(shaderSource), &length, errorLog);
		dgTrace((errorLog));
	}
	glAttachShader(program, computeShader);

	glLinkProgram(program);
	glGetProgramiv(program, GL_LINK_STATUS, &state);
	dgAssert(state == GL_TRUE);

	glValidateProgram(program);
	glGetProgramiv(program, GL_VALIDATE_STATUS, &state);
	dgAssert(state == GL_TRUE);

	glDeleteShader(computeShader);
	return program;
}
*/