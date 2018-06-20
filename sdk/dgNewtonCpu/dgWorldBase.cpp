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
	union cpuInfo
	{
		int m_data[4];
		struct
		{
			int m_eax;
			int m_ebx;
			int m_ecx;
			int m_edx;
		};
	} info;
/*
	//check Intel CPU
	// avx2 support in code 7, register ebx bit5
	__cpuid(info.m_data, 7);
	if (info.m_ebx & (1 << 5)) {
		dgFloatSse::m_one = dgFloatSse(1.0f);
		dgFloatSse::m_zero = dgFloatSse(0.0f);
		dgFloatAvx::m_one = dgFloatAvx(1.0f);
		dgFloatAvx::m_zero = dgFloatAvx(0.0f);
		dgFloatAvx2::m_one = dgFloatAvx2(1.0f);
		dgFloatAvx2::m_zero = dgFloatAvx2(0.0f);

		//cpu support avx2
		//static dgWorldSse module(world, allocator);
		static dgWorldAvx module(world, allocator);
		//static dgWorldAvx2 module(world, allocator);
		return &module;
	}
*/
	// avx support is encoded on register ecx bit 28
	__cpuid(info.m_data, 1);
	if (!(info.m_ecx & (1 << 28))) {
		return NULL;
	}
//return NULL;

//	dgFloatSse::m_one = dgFloatSse(1.0f);
//	dgFloatSse::m_zero = dgFloatSse(0.0f);
//	dgFloatAvx::m_one = dgFloatAvx(1.0f);
//	dgFloatAvx::m_zero = dgFloatAvx(0.0f);
//	dgFloatAvx2::m_one = dgFloatAvx2(1.0f);
//	dgFloatAvx2::m_zero = dgFloatAvx2(0.0f);

	//cpu support avx
	static dgWorldBase module(world, allocator);
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
#ifdef _DEBUG
	return "newtonAVX_d";
#else
	return "newtonAVX";
#endif
}

void dgWorldBase::CalculateJointForces(const dgBodyCluster& cluster, dgBodyInfo* const bodyArray, dgJointInfo* const jointArray, dgFloat32 timestep)
{
	dgSolver::CalculateJointForces(cluster, bodyArray, jointArray, timestep);
}