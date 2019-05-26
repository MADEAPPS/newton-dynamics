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

// This is an example of an exported function.
dgWorldPlugin* GetPlugin(dgWorld* const world, dgMemoryAllocator* const allocator)
{
	const GLubyte* const version = glGetString(GL_SHADING_LANGUAGE_VERSION);

	dgInt32 versionNumber = dgInt32 (dgCeil (100.0f * atof ((char*)version)));
	if (versionNumber < 450) {
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

// test making a buffer this works
dgArrayGPU<dgVector> xxx;
xxx.Alloc (DG_GPU_WORKGROUP_SIZE);

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
// this does not works
//const GLubyte* const version = glGetString(GL_SHADING_LANGUAGE_VERSION);
//GLuint program = glCreateProgram();
//GLuint computeShader = glCreateShader(GL_COMPUTE_SHADER);
//dgArrayGPU<dgVector> xxx;
//xxx.Alloc (DG_GPU_WORKGROUP_SIZE);
//glfwInit();
//glfwMakeContextCurrent(NULL);


// here the same function fail 
dgArrayGPU<dgVector> xxx;
xxx.Alloc (DG_GPU_WORKGROUP_SIZE);

	dgSolver::CalculateJointForces(cluster, bodyArray, jointArray, timestep);
}