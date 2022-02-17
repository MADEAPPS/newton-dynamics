/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndMemory.h"

ndAtomic<ndUnsigned64> ndMemory::m_memoryUsed(0);

static ndMemFreeCallback m_freeMemory = free;
static ndMemAllocCallback m_allocMemory = malloc;

class dMemoryHeader
{
	public:
	void* m_ptr;
	ndInt32 m_size;
};

#define D_MEMORY_ALIGMNET 32
#define dGetBufferSize ndInt32(D_MEMORY_ALIGMNET - 1 + sizeof (dMemoryHeader))

ndInt32 ndMemory::CalculateBufferSize(size_t size)
{
	return ndInt32 (size + dGetBufferSize);
}

void* ndMemory::Malloc(size_t size)
{
	size += dGetBufferSize;
	void* const ptr = m_allocMemory(size);
	ndInt64 val = ndUnsigned64(ptr) + dGetBufferSize;
	ndInt64 mask = -ndInt64(D_MEMORY_ALIGMNET);
	val = val & mask;
	dMemoryHeader* const ret = (dMemoryHeader*)val;
	dMemoryHeader* const info = ret - 1;
	info->m_ptr = ptr;
	info->m_size = ndInt32 (size);
	m_memoryUsed.fetch_add(size);
	return ret;
}

void ndMemory::Free(void* const ptr)
{
	if (ptr)
	{
		dMemoryHeader* const ret = ((dMemoryHeader*)ptr) - 1;
		m_memoryUsed.fetch_sub(ret->m_size);
		m_freeMemory(ret->m_ptr);
	}
}

ndInt32 ndMemory::GetSize(void* const ptr)
{
	dMemoryHeader* const ret = ((dMemoryHeader*)ptr) - 1;
	return ret->m_size;
}

ndUnsigned64 ndMemory::GetMemoryUsed()
{
	return m_memoryUsed.load();
}

void ndMemory::SetMemoryAllocators(ndMemAllocCallback alloc, ndMemFreeCallback free)
{
	m_allocMemory = alloc;
	m_freeMemory = free;
}
