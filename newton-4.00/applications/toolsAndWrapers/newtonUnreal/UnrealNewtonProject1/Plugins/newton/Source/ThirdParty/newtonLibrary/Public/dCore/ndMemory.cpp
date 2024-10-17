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

#define ndGetBufferPaddingInBytes size_t(D_MEMORY_ALIGMNET - 1 + sizeof (ndMemoryHeader))

size_t ndMemory::CalculateBufferSize(size_t size)
{
	return size + ndGetBufferPaddingInBytes;
}

void* ndMemory::Malloc(size_t size)
{
	ndIntPtr metToVal;
	size_t bufferSize = size + ndGetBufferPaddingInBytes;
	metToVal.m_ptr = m_allocMemory(bufferSize);
	ndUnsigned64 val = ndUnsigned64(metToVal.m_int) + ndGetBufferPaddingInBytes;
	ndInt64 mask = -ndInt64(D_MEMORY_ALIGMNET);
	val = val & mask;
	ndMemoryHeader* const ret = (ndMemoryHeader*)val;
	ndMemoryHeader* const info = ret - 1;
	ndAssert((char*)info >= (char*)metToVal.m_ptr);
	info->m_ptr = metToVal.m_ptr;
	info->m_bufferSize = ndUnsigned32 (bufferSize);
	info->m_requestedSize = ndUnsigned32(size);
	m_memoryUsed.fetch_add(bufferSize);
	return ret;
}

void ndMemory::Free(void* const ptr)
{
	if (ptr)
	{
		ndMemoryHeader* const info = ((ndMemoryHeader*)ptr) - 1;
		m_memoryUsed.fetch_sub(ndUnsigned64(info->m_bufferSize));
		m_freeMemory(info->m_ptr);
	}
}

size_t ndMemory::GetSize(void* const ptr)
{
	ndMemoryHeader* const info = ((ndMemoryHeader*)ptr) - 1;
	return info->m_bufferSize;
}

size_t ndMemory::GetOriginalSize(void* const ptr)
{
	ndMemoryHeader* const info = ((ndMemoryHeader*)ptr) - 1;
	return info->m_requestedSize;
}

ndUnsigned64 ndMemory::GetMemoryUsed()
{
	return m_memoryUsed.load();
}

void ndMemory::SetMemoryAllocators(ndMemAllocCallback alloc, ndMemFreeCallback free)
{
	m_freeMemory = free;
	m_allocMemory = alloc;
}

void ndMemory::GetMemoryAllocators(ndMemAllocCallback& alloc, ndMemFreeCallback& free)
{
	free = m_freeMemory;
	alloc = m_allocMemory;
}
