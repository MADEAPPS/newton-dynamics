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

dAtomic<dUnsigned64> dMemory::m_memoryUsed(0);

static dMemFreeCallback m_freeMemory = free;
static dMemAllocCallback m_allocMemory = malloc;

class dMemoryHeader
{
	public:
	void* m_ptr;
	dInt32 m_size;
};

#define D_MEMORY_ALIGMNET 32
#define dGetBufferSize dInt32(D_MEMORY_ALIGMNET - 1 + sizeof (dMemoryHeader))

dInt32 dMemory::CalculateBufferSize(size_t size)
{
	return dInt32 (size + dGetBufferSize);
}

void* dMemory::Malloc(size_t size)
{
	size += dGetBufferSize;
	void* const ptr = m_allocMemory(size);
	dInt64 val = dUnsigned64(ptr) + dGetBufferSize;
	dInt64 mask = -dInt64(D_MEMORY_ALIGMNET);
	val = val & mask;
	dMemoryHeader* const ret = (dMemoryHeader*)val;
	dMemoryHeader* const info = ret - 1;
	info->m_ptr = ptr;
	info->m_size = dInt32 (size);
	m_memoryUsed.fetch_add(size);
	return ret;
}

void dMemory::Free(void* const ptr)
{
	dMemoryHeader* const ret = ((dMemoryHeader*)ptr) - 1;
	m_memoryUsed.fetch_sub(ret->m_size);
	m_freeMemory(ret->m_ptr);
}

dInt32 dMemory::GetSize(void* const ptr)
{
	dMemoryHeader* const ret = ((dMemoryHeader*)ptr) - 1;
	return ret->m_size;
}

dUnsigned64 dMemory::GetMemoryUsed()
{
	return m_memoryUsed.load();
}

void dMemory::SetMemoryAllocators(dMemAllocCallback alloc, dMemFreeCallback free)
{
	m_allocMemory = alloc;
	m_freeMemory = free;
}
