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
#include "ndTree.h"
#include "ndTypes.h"
#include "ndUtils.h"
#include "ndMemory.h"

ndAtomic<ndUnsigned64> ndMemory::m_memoryUsed(0);

static ndMemFreeCallback m_freeMemory = free;
static ndMemAllocCallback m_allocMemory = malloc;

#define ND_CHECK_CORRUPT_MEM 0x0a

class ndMemoryHeader
{
	public:
	void* m_ptr;
	size_t m_bufferSize;
	size_t m_requestedSize;
};

#define D_MEMORY_ALIGMNET			32
#define ndGetBufferPaddingInBytes	size_t(D_MEMORY_ALIGMNET - 1 + sizeof (ndMemoryHeader))

#ifdef ND_VALIDATE_HEAP 
class ndMemoryCorruptionAllocatorCheck
{
	public:
	ndMemoryCorruptionAllocatorCheck()
	{
	}

	void* operator new (size_t size)
	{
		void* const mem = malloc(size);
		return mem;
	}

	void operator delete (void* ptr)
	{
		free(ptr);
	}
};

// keep track of all allocation, this should be disabled for production build 
class ndMemoryCorruptionCheck : public ndTree<ndInt64, void*, ndMemoryCorruptionAllocatorCheck>
{
	public:
	ndMemoryCorruptionCheck()
		:ndTree<ndInt64, void*, ndMemoryCorruptionAllocatorCheck>()
		,m_lock()
		,m_allocIndex(1)
	{
	}

	~ndMemoryCorruptionCheck()
	{
		// need to make sure the free list are cleaned up after the last world was deleted
		ndFreeListAlloc::Flush();

		ndScopeSpinLock lock(m_lock);
		Iterator it(*this);
		for (it.Begin(); it; )
		{
			ndNode* const node = it.GetNode();
			it++;
			if (node->GetInfo() < 0)
			{
				Remove(node->GetKey());
			}
		}
		ndAssert(!GetCount());
	}

	void InsertPointer(void* const ptr)
	{
		ndScopeSpinLock lock(m_lock);
		Insert(m_allocIndex, ptr);
		m_allocIndex++;
	}

	void RemovePointer(void* const ptr)
	{
		ndScopeSpinLock lock(m_lock);
		ndAssert(Find(ptr));
		Remove(ptr);
	}

	static ndMemoryCorruptionCheck& GetTracker()
	{
		static ndMemoryCorruptionCheck leakTracker;
		return leakTracker;
	}

	bool ValidateHeap() const
	{
		Iterator it(*this);
		ndInt64 allocIndex = 0;
		bool valid = true;
		for (it.Begin(); it; it++)
		{
			void* const key = it.GetKey();
			allocIndex = it.GetNode()->GetInfo();
			valid = valid && ndMemory::CheckMemory(key);
		}
		return true;
	}

	ndSpinLock m_lock;
	ndInt64 m_allocIndex;
};
#endif

size_t ndMemory::GetMemoryAligment()
{
	return D_MEMORY_ALIGMNET;
}

size_t ndMemory::CalculateBufferSize(size_t size)
{
	#if defined (D_MEMORY_SANITY_CHECK) && defined(_DEBUG)
	size += D_MEMORY_SAFE_GUARD * 2;
	#endif
	return size + ndGetBufferPaddingInBytes;
}

void* ndMemory::Malloc(size_t size)
{
	ndIntPtr metToVal;
	#if defined (D_MEMORY_SANITY_CHECK) && defined(_DEBUG)
		size += D_MEMORY_SAFE_GUARD * 2;
	#endif

	size_t bufferSize = size + ndGetBufferPaddingInBytes;
	metToVal.m_ptr = m_allocMemory(bufferSize);
	ndUnsigned64 val = ndUnsigned64(metToVal.m_int) + ndGetBufferPaddingInBytes;
	ndInt64 mask = -ndInt64(D_MEMORY_ALIGMNET);
	val = val & mask;
	ndMemoryHeader* ret = (ndMemoryHeader*)val;
	ndMemoryHeader* const info = ret - 1;
	ndAssert((char*)info >= (char*)metToVal.m_ptr);
	info->m_ptr = metToVal.m_ptr;
	info->m_bufferSize = bufferSize;
	info->m_requestedSize = size;
	m_memoryUsed.fetch_add(bufferSize);

	#if defined (D_MEMORY_SANITY_CHECK) && defined(_DEBUG)
		char code = ND_CHECK_CORRUPT_MEM;
		char* const ptr = (char*)ret;
		ndMemSet(ptr, code, D_MEMORY_SAFE_GUARD);
		ndMemSet(ptr + size - D_MEMORY_SAFE_GUARD, code, D_MEMORY_SAFE_GUARD);
		ret = (ndMemoryHeader*) (ptr + D_MEMORY_SAFE_GUARD);
	#endif

#ifdef ND_VALIDATE_HEAP 
	//ndAssert(ValidateHeap());
	ndMemoryCorruptionCheck::GetTracker().InsertPointer(ret);
#endif

	return ret;
}

void ndMemory::Free(void* const ptr)
{
	if (ptr)
	{

#ifdef ND_VALIDATE_HEAP 
		//ndAssert(ValidateHeap());
		ndMemoryCorruptionCheck::GetTracker().RemovePointer(ptr);
#endif

		#if defined (D_MEMORY_SANITY_CHECK) && defined(_DEBUG)
		ndAssert(CheckMemory(ptr));
		const char* const mem = ((char*)ptr) - D_MEMORY_SAFE_GUARD;
		ndMemoryHeader* const info = ((ndMemoryHeader*)mem) - 1;
		#else
		ndMemoryHeader* const info = ((ndMemoryHeader*)ptr) - 1;
		#endif		
		
		m_memoryUsed.fetch_sub(ndUnsigned64(info->m_bufferSize));
		m_freeMemory(info->m_ptr);
	}
}

#if defined (D_MEMORY_SANITY_CHECK) && defined(_DEBUG)
bool ndMemory::CheckMemory(const void* const ptr)
{
	const char* const mem0 = ((char*)ptr) - D_MEMORY_SAFE_GUARD;
	ndMemoryHeader* const info = ((ndMemoryHeader*)mem0) - 1;
	const char* const mem1 = mem0 + info->m_requestedSize - D_MEMORY_SAFE_GUARD;
	
	for (ndInt32 i = 0; i < D_MEMORY_SAFE_GUARD; ++i)
	{
		if (mem0[i] != ND_CHECK_CORRUPT_MEM)
		{
			return false;
		}
		if (mem1[i] != ND_CHECK_CORRUPT_MEM)
		{
			return false;
		}
	}
	return true;
}
#else
bool ndMemory::CheckMemory(const void* const)
{
	return true;
}
#endif

size_t ndMemory::GetSize(void* const ptr)
{
	#if defined (D_MEMORY_SANITY_CHECK) && defined(_DEBUG)
	ndAssert(CheckMemory(ptr));
	const char* const mem = ((char*)ptr) - D_MEMORY_SAFE_GUARD;
	ndMemoryHeader* const info = ((ndMemoryHeader*)mem) - 1;
	#else
	ndMemoryHeader* const info = ((ndMemoryHeader*)ptr) - 1;
	#endif
	
	return info->m_bufferSize;
}

size_t ndMemory::GetOriginalSize(void* const ptr)
{
#if defined (D_MEMORY_SANITY_CHECK) && defined(_DEBUG)
	ndAssert(CheckMemory(ptr));
	const char* const mem = ((char*)ptr) - D_MEMORY_SAFE_GUARD;
	ndMemoryHeader* const info = ((ndMemoryHeader*)mem) - 1;
#else
	ndMemoryHeader* const info = ((ndMemoryHeader*)ptr) - 1;
#endif
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

bool ndMemory::ValidateHeap()
{
#ifdef ND_VALIDATE_HEAP 
	return ndMemoryCorruptionCheck::GetTracker().ValidateHeap();
#else	
	return true;
#endif
}
