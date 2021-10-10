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

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dMemory.h"

dAtomic<dUnsigned64> dMemory::m_memoryUsed(0);

static dMemFreeCallback m_freeMemory = free;
static dMemAllocCallback m_allocMemory = malloc;

class dMemoryHeader
{
	public:
	union
	{
		char m_padd[32];
		struct
		{
			void* m_ptr;
			dInt32 m_size;
		};
	};
};

void* dMemory::Malloc(size_t size)
{
	size += 2 * sizeof(dMemoryHeader) - 1;
	void* const ptr = m_allocMemory(size);
	dInt64 val = dUnsigned64(ptr) + sizeof(dMemoryHeader) - 1;
	dInt64 mask = -dInt64(sizeof(dMemoryHeader));
	val = val & mask;
	dMemoryHeader* const ret = (dMemoryHeader*)val;
	ret->m_size = dInt32 (size);
	ret->m_ptr = ptr;
	m_memoryUsed.fetch_add(size);
	return &ret[1];
}

void dMemory::Free(void* const ptr)
{
	dMemoryHeader* const ret = ((dMemoryHeader*)ptr) - 1;
	m_memoryUsed.fetch_sub(ret->m_size);
	m_freeMemory(ret->m_ptr);
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
