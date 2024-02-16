/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

class ndMemoryHeader
{
	public:
	void* m_ptr;
	ndUnsigned32 m_bufferSize;
	ndUnsigned32 m_requestedSize;
};

#define ndGetBufferPaddingSize size_t(D_MEMORY_ALIGMNET - 1 + sizeof (ndMemoryHeader))

size_t ndMemory::CalculateBufferSize(size_t size)
{
	return size + ndGetBufferPaddingSize;
}

void* ndMemory::Malloc(size_t requestSize)
{
	ndIntPtr metToVal;
	size_t size = requestSize + ndGetBufferPaddingSize;
	metToVal.m_ptr = m_allocMemory(size);
	ndUnsigned64 val = ndUnsigned64(metToVal.m_int) + ndGetBufferPaddingSize;
	ndInt64 mask = -ndInt64(D_MEMORY_ALIGMNET);
	val = val & mask;
	ndMemoryHeader* const ret = (ndMemoryHeader*)val;
	ndMemoryHeader* const info = ret - 1;
	ndAssert((char*)info >= (char*)metToVal.m_ptr);
	info->m_ptr = metToVal.m_ptr;
	info->m_bufferSize = ndUnsigned32 (size);
	info->m_requestedSize = ndUnsigned32(requestSize);
	m_memoryUsed.fetch_add(size);
	return ret;
}

void ndMemory::Free(void* const ptr)
{
	if (ptr)
	{
		ndMemoryHeader* const ret = ((ndMemoryHeader*)ptr) - 1;
		m_memoryUsed.fetch_sub(ndUnsigned64(ret->m_bufferSize));
		m_freeMemory(ret->m_ptr);
	}
}

size_t ndMemory::GetSize(void* const ptr)
{
	ndMemoryHeader* const ret = ((ndMemoryHeader*)ptr) - 1;
	return ret->m_bufferSize;
}

size_t ndMemory::GetOriginalSize(void* const ptr)
{
	ndMemoryHeader* const ret = ((ndMemoryHeader*)ptr) - 1;
	return ret->m_requestedSize;
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

void ndMemory::GetMemoryAllocators(ndMemAllocCallback& alloc, ndMemFreeCallback& free)
{
	alloc = m_allocMemory;
	free = m_freeMemory;
}
