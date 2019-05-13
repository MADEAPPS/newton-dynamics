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

#include "NewtonCSGPU.glsl"


// This is an example of an exported function.
dgWorldPlugin* GetPlugin(dgWorld* const world, dgMemoryAllocator* const allocator)
{
//	char *cs_Err;
	GLsizei length = 0;

//	GLuint mShaderID;
//	GLuint mComputeShader;
	GLuint mShaderID = glCreateProgram();
	if (mShaderID != 0) {
		GLuint mComputeShader = glCreateShader(GL_COMPUTE_SHADER);
		if (mComputeShader != 0) {
			glShaderSource(mComputeShader, 1, &shaderName, NULL);
			glCompileShader(mComputeShader);
			
			int CSErr;
			glGetShaderiv(mComputeShader, GL_COMPILE_STATUS, &CSErr);
			if (CSErr != GL_TRUE) {
				glGetShaderiv(mComputeShader, GL_INFO_LOG_LENGTH, &length);
				char cs_Err[GL_INFO_LOG_LENGTH + 1024];
				glGetShaderInfoLog(mComputeShader, GL_INFO_LOG_LENGTH, &length, cs_Err);
				printf("Error: Compiler log:\n%s\n", cs_Err);
			}

			// Attach and link the shader against the compute program.
			glAttachShader(mShaderID, mComputeShader);
			glLinkProgram(mShaderID);
			//
			// Check if there were any issues linking the shader.
			glGetProgramiv(mShaderID, GL_LINK_STATUS, &CSErr);
			//
/*
			if (!CSErr)
			{
				glGetProgramiv(mShaderID, GL_INFO_LOG_LENGTH, &length);
				cs_Err = (char *)malloc(length);
				glGetProgramInfoLog(mShaderID, GL_INFO_LOG_LENGTH, &length, cs_Err);
				//printf("Error: Linker log:\n%s\n", cs_Err);
			}
			if (mComputeShader != 0) {
				glDeleteShader(mComputeShader);
			}
*/
		}
	}

	static dgWorldBase module(world, allocator);
	module.m_score = 1;
#ifdef _DEBUG
	sprintf(module.m_hardwareDeviceName, "Newton opengl_d");
#else
	sprintf(module.m_hardwareDeviceName, "Newton opengl");
#endif
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