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
#include "dgWorldAvx.h"
#include "dgMathAvx2.h"
#include "dgMathAvx2.h"
#include "dgWorldAvx2.h"

dgWorldBase::dgWorldBase(dgMemoryAllocator* const allocator)
	:dgWorldPlugin(allocator)
	, m_bodyArray(NULL)
	, m_jointArray(NULL)
	, m_cluster(NULL)
	, m_timestep(dgFloat32(0.0f))
{
}

dgWorldBase::~dgWorldBase()
{
}

// This is an example of an exported function.
dgWorldPlugin* GetPlugin(dgMemoryAllocator* const allocator)
{
	union cpuInfo
	{
		int data[4];
		struct
		{
			int eax;
			int ebx;
			int ecx;
			int edx;
		};
	} info;

	// avx2 support in code 7, register ebx bit5
	__cpuid(info.data, 7);
	if (info.ebx & (1 << 5)) {
		//cpu support avx2
		dgFloatAvx::m_one = dgFloatAvx(1.0f);
		dgFloatAvx::m_zero = dgFloatAvx(0.0f);
		dgFloatAvx2::m_one = dgFloatAvx2(1.0f);
		dgFloatAvx2::m_zero = dgFloatAvx2(0.0f);
		static dgWorldAvx module(allocator);
		//static dgWorldAvx2 module(allocator);
		return &module;
	}

	// avx support is encoded on register ecx bit 28
	__cpuid(info.data, 1);
	if (info.ecx & (1 << 28)) {
		//cpu support avx
		dgFloatAvx::m_one = dgFloatAvx(1.0f);
		dgFloatAvx::m_zero = dgFloatAvx(0.0f);
		dgFloatAvx2::m_one = dgFloatAvx2(1.0f);
		dgFloatAvx2::m_zero = dgFloatAvx2(0.0f);
		static dgWorldAvx module(allocator);
		return &module;
	}

	dgAssert(0);
	return NULL;
}
