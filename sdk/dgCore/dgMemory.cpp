/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#include "dgStdafx.h"
#include "dgList.h"
#include "dgDebug.h"
#include "dgMemory.h"


#ifdef DG_OLD_ALLOCATOR
dgInt32 dgMemoryAllocator::m_lock0 = 0;
dgInt32 dgMemoryAllocator::m_lock1 = 0;
#define DG_MEMORY_LOCK() dgScopeSpinPause lock (&dgMemoryAllocator::m_lock0);
#define DG_MEMORY_LOCK_LOW() dgScopeSpinPause lock (&dgMemoryAllocator::m_lock1);

class dgMemoryAllocator::dgMemoryBin
{
	public:
	class dgMemoryBinInfo
	{
		public:
		dgInt32 m_count;
		dgInt32 m_totalCount;
		dgInt32 m_stepInBytes;
		dgMemoryBin* m_next;
		dgMemoryBin* m_prev;
	};
	char m_pool[DG_MEMORY_BIN_SIZE - sizeof (dgMemoryBinInfo)-DG_MEMORY_GRANULARITY * 2];
	dgMemoryBinInfo m_info;
};

class dgMemoryAllocator::dgMemoryCacheEntry
{
	public:
	dgMemoryCacheEntry* m_next;
	dgMemoryCacheEntry* m_prev;
};

class dgMemoryAllocator::dgMemoryInfo
{
	public:
	void *m_ptr;
	dgMemoryAllocator* m_allocator;
	dgInt32 m_size;
	dgInt32 m_enum;

	#ifdef _DEBUG
	dgInt32 m_workingSize;
	#endif

	DG_INLINE void SaveInfo(dgMemoryAllocator* const allocator, void* const ptr, dgInt32 size, dgInt32& enumerator, dgInt32 workingSize = 0)
	{
		m_ptr = ptr;
		m_size = size;
		m_enum = enumerator;
		enumerator++;
		m_allocator = allocator;
#ifdef _DEBUG
		m_workingSize = workingSize;
#endif
	}
};

class dgGlobalAllocator: public dgMemoryAllocator, public dgList<dgMemoryAllocator*>
{
	public:
	dgGlobalAllocator ()
		:dgMemoryAllocator (__malloc__, __free__), dgList<dgMemoryAllocator*> (NULL)
	{
		SetAllocator (this);
	}

	~dgGlobalAllocator ()
	{
		dgAssert (GetCount() == 0);
	}

	static void* dgApi __malloc__ (dgUnsigned32 size) 
	{
		return malloc (size);
	}

	static void dgApi __free__ (void* const ptr, dgUnsigned32 size)
	{
		free (ptr);
	}

	void operator delete (void* const ptr)
	{
		dgAssert (0);
		free (ptr);
	}

	dgInt32 GetMemoryUsed () const
	{
		dgInt32 mem = m_memoryUsed;
		for (dgList<dgMemoryAllocator*>::dgListNode* node = GetFirst(); node; node = node->GetNext()) {
			mem += node->GetInfo()->GetMemoryUsed();
		}
		return mem;
	}

	static dgGlobalAllocator& GetGlobalAllocator()
	{
		static dgGlobalAllocator m_globalAllocator;
		return m_globalAllocator;
	}
};

dgMemoryAllocator::dgMemoryAllocator ()
	:m_free(NULL)
	,m_malloc(NULL)
	,m_enumerator(0)
	,m_memoryUsed(0)
	,m_isInList(1)
{
	SetAllocatorsCallback (dgGlobalAllocator::GetGlobalAllocator().m_malloc, dgGlobalAllocator::GetGlobalAllocator().m_free);
	memset (m_memoryDirectory, 0, sizeof (m_memoryDirectory));
	dgGlobalAllocator::GetGlobalAllocator().Append(this);
}

dgMemoryAllocator::dgMemoryAllocator (dgMemAlloc memAlloc, dgMemFree memFree)
	:m_free(NULL)
	,m_malloc(NULL)
	,m_enumerator(0)
	,m_memoryUsed(0)
	,m_isInList(0)
{
	SetAllocatorsCallback (memAlloc, memFree);
	memset (m_memoryDirectory, 0, sizeof (m_memoryDirectory));
}

dgMemoryAllocator::~dgMemoryAllocator  ()
{
	if (m_isInList) {
		dgGlobalAllocator::GetGlobalAllocator().Remove(this);
	}
	dgAssert (m_memoryUsed == 0);
}


void *dgMemoryAllocator::operator new (size_t size) 
{ 
	return dgMallocStack(size);
}

void dgMemoryAllocator::operator delete (void* const ptr) 
{ 
	dgFreeStack(ptr); 
}

dgInt32 dgMemoryAllocator::GetMemoryUsed() const
{
	return m_memoryUsed;
}

void dgMemoryAllocator::SetAllocatorsCallback (dgMemAlloc memAlloc, dgMemFree memFree)
{
	m_free = memFree;
	m_malloc = memAlloc;
}

void *dgMemoryAllocator::MallocLow (dgInt32 workingSize, dgInt32 alignment)
{
	DG_MEMORY_LOCK_LOW();
	alignment = dgMax (alignment, DG_MEMORY_GRANULARITY);
	dgAssert (((-alignment) & (alignment - 1)) == 0);
	dgInt32 size = workingSize + alignment * 2;
	void* const ptr = m_malloc(dgUnsigned32 (size));
	dgAssert (ptr);
#ifdef _DEBUG
	memset(ptr, 99, size_t(size));
#endif

	dgUnsigned64 val = dgUnsigned64 (PointerToInt(ptr));
	val = (val & dgUnsigned64(-alignment)) + alignment * 2;
	void* const retPtr = IntToPointer (val);

	dgMemoryInfo* const info = ((dgMemoryInfo*) (retPtr)) - 1;
	info->SaveInfo(this, ptr, size, m_enumerator, workingSize);

	dgAtomicExchangeAndAdd (&m_memoryUsed, size);
	return retPtr;
}

void dgMemoryAllocator::FreeLow (void* const retPtr)
{
	DG_MEMORY_LOCK_LOW();
	dgMemoryInfo* const info = ((dgMemoryInfo*) (retPtr)) - 1;
	dgAssert (info->m_allocator == this);

	dgAtomicExchangeAndAdd (&m_memoryUsed, -info->m_size);

#ifdef _DEBUG
	memset (retPtr, 0, size_t(info->m_workingSize));
#endif

	m_free (info->m_ptr, dgUnsigned32 (info->m_size));
}

void *dgMemoryAllocator::Malloc (dgInt32 memsize)
{
	dgAssert (dgInt32 (sizeof (dgMemoryCacheEntry) + sizeof (dgInt32) + sizeof(dgInt32)) <= DG_MEMORY_GRANULARITY);

	dgInt32 size = memsize + DG_MEMORY_GRANULARITY - 1;
	size &= (-DG_MEMORY_GRANULARITY);

	dgInt32 paddedSize = size + DG_MEMORY_GRANULARITY; 
	dgInt32 entry = paddedSize >> DG_MEMORY_GRANULARITY_BITS;	

	void* ptr;
	if (entry >= DG_MEMORY_BIN_ENTRIES) {
		ptr = MallocLow (size);
	} else {
		DG_MEMORY_LOCK();
		if (!m_memoryDirectory[entry].m_cache) {
			dgMemoryBin* const bin = (dgMemoryBin*) MallocLow (sizeof (dgMemoryBin));

			dgInt32 count = dgInt32 (sizeof (bin->m_pool) / paddedSize);
			bin->m_info.m_count = 0;
			bin->m_info.m_totalCount = count;
			bin->m_info.m_stepInBytes = paddedSize;
			bin->m_info.m_next = m_memoryDirectory[entry].m_first;
			bin->m_info.m_prev = NULL;
			if (bin->m_info.m_next) {
				bin->m_info.m_next->m_info.m_prev = bin;
			}

			m_memoryDirectory[entry].m_first = bin;

			dgInt8* charPtr = reinterpret_cast<dgInt8*>(bin->m_pool);
			m_memoryDirectory[entry].m_cache = (dgMemoryCacheEntry*)charPtr;

			for (dgInt32 i = 0; i < count; i ++) {
				dgMemoryCacheEntry* const cashe = (dgMemoryCacheEntry*) charPtr;
				cashe->m_next = (dgMemoryCacheEntry*) (charPtr + paddedSize);
				cashe->m_prev = (dgMemoryCacheEntry*) (charPtr - paddedSize);
				dgMemoryInfo* const info = ((dgMemoryInfo*) (charPtr + DG_MEMORY_GRANULARITY)) - 1;						
				info->SaveInfo(this, bin, entry, m_enumerator, memsize);
				charPtr += paddedSize;
			}
			dgMemoryCacheEntry* const cashe = (dgMemoryCacheEntry*) (charPtr - paddedSize);
			cashe->m_next = NULL;
			m_memoryDirectory[entry].m_cache->m_prev = NULL;
		}


		dgAssert (m_memoryDirectory[entry].m_cache);

		dgMemoryCacheEntry* const cashe = m_memoryDirectory[entry].m_cache;
		m_memoryDirectory[entry].m_cache = cashe->m_next;
		if (cashe->m_next) {
			cashe->m_next->m_prev = NULL;
		}

		ptr = ((dgInt8*)cashe) + DG_MEMORY_GRANULARITY;

		dgMemoryInfo* info;
		info = ((dgMemoryInfo*) (ptr)) - 1;
		dgAssert (info->m_allocator == this);

		dgMemoryBin* const bin = (dgMemoryBin*) info->m_ptr;
		bin->m_info.m_count ++;
	}
	return ptr;
}

void dgMemoryAllocator::Free (void* const retPtr)
{
	dgMemoryInfo* const info = ((dgMemoryInfo*) (retPtr)) - 1;
	dgAssert (info->m_allocator == this);

	dgInt32 entry = info->m_size;

	if (entry >= DG_MEMORY_BIN_ENTRIES) {
		FreeLow (retPtr);
	} else {
		DG_MEMORY_LOCK();
		dgMemoryCacheEntry* const cashe = (dgMemoryCacheEntry*) (((char*)retPtr) - DG_MEMORY_GRANULARITY) ;
		
		dgMemoryCacheEntry* const tmpCashe = m_memoryDirectory[entry].m_cache;
		if (tmpCashe) {
			dgAssert (!tmpCashe->m_prev);
			tmpCashe->m_prev = cashe;
		}
		cashe->m_next = tmpCashe;
		cashe->m_prev = NULL;

		m_memoryDirectory[entry].m_cache = cashe;

		dgMemoryBin* const bin = (dgMemoryBin *) info->m_ptr;

		dgAssert (bin);
#ifdef _DEBUG
		dgAssert ((bin->m_info.m_stepInBytes - DG_MEMORY_GRANULARITY) > 0);
		memset (retPtr, 0, size_t(bin->m_info.m_stepInBytes - DG_MEMORY_GRANULARITY));
#endif

		bin->m_info.m_count --;
		if (bin->m_info.m_count == 0) {

			dgInt32 count = bin->m_info.m_totalCount;
			dgInt32 sizeInBytes = bin->m_info.m_stepInBytes;
			char* charPtr = bin->m_pool;
			for (dgInt32 i = 0; i < count; i ++) {
				dgMemoryCacheEntry* const tmpCashe1 = (dgMemoryCacheEntry*)charPtr;
				charPtr += sizeInBytes;

				if (tmpCashe1 == m_memoryDirectory[entry].m_cache) {
					m_memoryDirectory[entry].m_cache = tmpCashe1->m_next;
				}

				if (tmpCashe1->m_prev) {
					tmpCashe1->m_prev->m_next = tmpCashe1->m_next;
				}

				if (tmpCashe1->m_next) {
					tmpCashe1->m_next->m_prev = tmpCashe1->m_prev;
				}
			}

			if (m_memoryDirectory[entry].m_first == bin) {
				m_memoryDirectory[entry].m_first = bin->m_info.m_next;
			}

			if (bin->m_info.m_next) {
				bin->m_info.m_next->m_info.m_prev = bin->m_info.m_prev;
			}
			if (bin->m_info.m_prev) {
				bin->m_info.m_prev->m_info.m_next = bin->m_info.m_next;
			}

			FreeLow (bin);
		}
	}
}

dgInt32 dgMemoryAllocator::GetSize (void* const retPtr)
{
	dgMemoryInfo* const info = ((dgMemoryInfo*)(retPtr)) - 1;
	return info->m_size;
}

void dgMemoryAllocator::SetGlobalAllocators (dgMemAlloc malloc, dgMemFree free)
{
	dgGlobalAllocator::GetGlobalAllocator().SetAllocatorsCallback (malloc, free);
}

dgInt32 dgMemoryAllocator::GetGlobalMemoryUsed ()
{
	return dgGlobalAllocator::GetGlobalAllocator().GetMemoryUsed();
}

// this can be used by function that allocates large memory pools memory locally on the stack
// this by pases the pool allocation because this should only be used for very large memory blocks.
// this was using virtual memory on windows but 
// but because of many complaint I changed it to use malloc and free
void* dgApi dgMallocStack (size_t size)
{
	void * const ptr = dgGlobalAllocator::GetGlobalAllocator().MallocLow (dgInt32 (size));
	return ptr;
}

void* dgApi dgMallocAligned (size_t size, dgInt32 align)
{
	void * const ptr = dgGlobalAllocator::GetGlobalAllocator().MallocLow (dgInt32 (size), align);
	return ptr;
}

// this can be used by function that allocates large memory pools memory locally on the stack
// this by pases the pool allocation because this should only be used for very large memory blocks.
// this was using virtual memory on windows but 
// but because of many complaint I changed it to use malloc and free
void  dgApi dgFreeStack (void* const ptr)
{
	dgGlobalAllocator::GetGlobalAllocator().FreeLow (ptr);
}

// general memory allocation for all data in the library
void* dgApi dgMalloc (size_t size, dgMemoryAllocator* const allocator) 
{
	void* ptr = NULL;
	dgAssert (allocator);

	if (size) {
		ptr = allocator->Malloc (dgInt32 (size));
	}
	return ptr;
}

// general deletion allocation for all data in the library
void dgApi dgFree (void* const ptr)
{
	if (ptr) {
		dgMemoryAllocator::dgMemoryInfo* const info = ((dgMemoryAllocator::dgMemoryInfo*) ptr) - 1;
		dgAssert (info->m_allocator);
		info->m_allocator->Free (ptr);
	}
}

#else


void* dgGlobalAllocator::__malloc__(dgUnsigned32 size)
{
	return malloc(size);
}

void dgGlobalAllocator::__free__(void* const ptr, dgUnsigned32 size)
{
	free(ptr);
}

void dgGlobalAllocator::SetAllocatorsCallback (dgMemAlloc malloc, dgMemFree free)
{
	m_free = free;
	m_malloc = malloc;
}


void* dgGlobalAllocator::Malloc(dgInt32 size)
{
	dgInt32 paddedSize = size + sizeof (dgMemoryAllocator::dgMemoryGranularity) + sizeof (dgMemoryAllocator::dgMemoryHeader);
	char* const ptr = (char*)m_malloc(paddedSize);
	
	dgUnsigned64 address = dgUnsigned64(ptr + sizeof (dgMemoryAllocator::dgMemoryHeader) + sizeof (dgMemoryAllocator::dgMemoryGranularity)-1) & ~(sizeof (dgMemoryAllocator::dgMemoryGranularity)-1);
	
	dgMemoryAllocator::dgMemoryHeader* const info = ((dgMemoryAllocator::dgMemoryHeader*)address) - 1;
	info->m_ptr = ptr;
	info->m_allocator = this;
	info->m_size = size;
	info->m_paddedSize = paddedSize;
	dgAtomicExchangeAndAdd(&m_memoryUsed, paddedSize);
	return &info[1];
}

void dgGlobalAllocator::Free(void* const ptr)
{
	dgMemoryAllocator::dgMemoryHeader* const info = ((dgMemoryAllocator::dgMemoryHeader*)ptr) - 1;
	dgAssert(info->m_allocator == this);

	dgAtomicExchangeAndAdd(&m_memoryUsed, -info->m_paddedSize);
	m_free(info->m_ptr, info->m_paddedSize);
}

dgMemoryAllocatorBase& dgGlobalAllocator::GetGlobalAllocator()
{
	static dgGlobalAllocator m_globalAllocator;
	return m_globalAllocator;
}


dgMemoryAllocator::dgMemoryPage::dgMemoryPage(dgInt32 size, dgMemoryPage* const root, dgMemoryAllocator* const allocator)
	:m_next(root)
	,m_prev(NULL)
	,m_fullPageNext(NULL)
	,m_fullPagePrev(NULL)
	,m_freeList (NULL)
	,m_count(0)
	,m_capacity(0)
{
	if (root) {
		dgAssert (!root->m_prev);
		root->m_prev = this;
	}
	dgInt32 paddSize = size + sizeof (dgMemoryHeader);
	dgAssert ((paddSize & (sizeof (dgMemoryGranularity) - 1)) == 0);
	
	const char* const ptr1 = &m_buffer[sizeof (m_buffer) - paddSize];
	for (char* ptr = &m_buffer[sizeof (dgMemoryGranularity)]; ptr <= ptr1; ptr += paddSize) {
		dgAssert ((dgUnsigned64(ptr) & (sizeof (dgMemoryGranularity) - 1)) == 0);

		dgMemoryGranularity* const freeList = (dgMemoryGranularity*) ptr;
		dgMemoryHeader* const header = ((dgMemoryHeader*)freeList) - 1;
		header->m_page = this;
		header->m_allocator = allocator;
		header->m_size = 0;
		header->m_paddedSize = size;

		freeList->m_next = m_freeList;
		m_freeList = freeList;
		m_count ++;
	}

	m_capacity = m_count;
}

dgMemoryAllocator::dgMemoryPage::~dgMemoryPage()
{
	dgAssert(m_count == m_capacity);
}

void *dgMemoryAllocator::dgMemoryPage::operator new (size_t size)
{
	dgMemoryAllocatorBase& globalAllocator = dgGlobalAllocator::GetGlobalAllocator();
	return globalAllocator.Malloc(dgInt32 (size));
}

void dgMemoryAllocator::dgMemoryPage::operator delete (void* const ptr)
{
	dgMemoryAllocatorBase& globalAllocator = dgGlobalAllocator::GetGlobalAllocator();
	globalAllocator.Free(ptr);
}

void* dgMemoryAllocator::dgMemoryPage::Malloc(dgInt32 size)
{
	m_count --;
	dgAssert (m_count >= 0);
	dgMemoryGranularity* const freeList = m_freeList;
	m_freeList = m_freeList->m_next;

	dgMemoryHeader* const header = ((dgMemoryHeader*)freeList) - 1;
	header->m_size = size;
	return freeList;
}

void dgMemoryAllocator::dgMemoryPage::Free(void* const ptr)
{
	m_count++;
	dgAssert(m_count <= m_capacity);
	dgMemoryHeader* const info = ((dgMemoryHeader*)ptr) - 1;
	info->m_size = 0;

	dgMemoryGranularity* const freeList = (dgMemoryGranularity*) ptr;
	freeList->m_next = m_freeList;
	m_freeList = freeList;
}

dgMemoryAllocator::dgMemoryBeam::dgMemoryBeam()
	:m_firstPage(NULL)
	,m_fullPage(NULL)
	,m_beamSize(0)
	,m_inUsedCount(0)
	,m_fullPageCount(0)
{
}

dgMemoryAllocator::dgMemoryBeam::~dgMemoryBeam()
{
	while (m_firstPage) {
		if (m_firstPage->m_next) {
			m_firstPage->m_next->m_prev = NULL;
		}
		dgMemoryPage* const firstPage = m_firstPage;
		m_firstPage = m_firstPage->m_next;
		delete firstPage;
	}

	while (m_fullPage) {
		dgAssert(0);
		if (m_fullPage->m_fullPageNext) {
			m_fullPage->m_fullPageNext->m_fullPagePrev = NULL;
		}
		dgMemoryPage* const fullPage = m_fullPage;
		m_fullPage = m_fullPage->m_fullPageNext;
		delete fullPage;
	}

	m_fullPage = NULL;
	m_firstPage = NULL;
}

void* dgMemoryAllocator::dgMemoryBeam::Malloc(dgInt32 size)
{
	dgAssert (size <= m_beamSize);
	if (!m_firstPage) {
		m_firstPage = new dgMemoryPage (m_beamSize, m_firstPage, m_allocator);
		m_inUsedCount ++;
	}

	dgAssert (m_firstPage->m_count);
	//if (m_firstPage->m_count == 0) {
	//	m_firstPage = new dgMemoryPage (m_beamSize, m_firstPage, m_allocator);
	//}

	void* ptr = m_firstPage->Malloc(size);
	if (m_firstPage->m_count == 0) {
		dgMemoryPage* const page = m_firstPage;

		m_inUsedCount --;
		m_fullPageCount ++;

		m_firstPage = page->m_next;
		if (m_firstPage) {
			m_firstPage->m_prev = NULL;
		}
		page->m_next = NULL;
		page->m_prev = NULL;

		dgAssert(!page->m_fullPageNext);
		dgAssert(!page->m_fullPagePrev);
		page->m_fullPageNext = m_fullPage;
		if (m_fullPage) {
			m_fullPage->m_fullPagePrev = page;
		}
		m_fullPage = page;
	}
	return ptr;
}

void dgMemoryAllocator::dgMemoryBeam::Free(void* const ptr)
{
	dgMemoryHeader* const info = ((dgMemoryHeader*)ptr) - 1;
	dgMemoryPage* const page = info->m_page;

	page->Free(ptr);
	dgAssert (page->m_count <= page->m_capacity);
	if (page->m_count == page->m_capacity) {
		m_inUsedCount --;
		if (page == m_firstPage) {
			m_firstPage = m_firstPage->m_next;
		}
		if (page->m_next) {
			page->m_next->m_prev = page->m_prev;
		}
		if (page->m_prev) {
			page->m_prev->m_next = page->m_next;
		}
		page->m_next = NULL;
		page->m_prev = NULL;
		delete page;
	} else if (page->m_count == 1) {

		m_inUsedCount++;
		m_fullPageCount--;

		if (page == m_fullPage) {
			m_fullPage = m_fullPage->m_fullPageNext;
		}
		if (page->m_fullPageNext) {
			page->m_fullPageNext->m_fullPagePrev = page->m_fullPagePrev;
		}
		if (page->m_fullPagePrev) {
			page->m_fullPagePrev->m_fullPageNext = page->m_fullPageNext;
		}
		page->m_fullPagePrev = NULL;
		page->m_fullPageNext = NULL;

		dgAssert(!page->m_next);
		dgAssert(!page->m_prev);
		page->m_next = m_firstPage;
		if (m_firstPage) {
			m_firstPage->m_prev = page;
		}
		m_firstPage = page;

//	} else if (page->m_prev) {
//		dgInt32 key = page->GetSortKey();
//		dgMemoryPage* prevPage = page->m_prev;
//		while (prevPage && prevPage->GetSortKey() < key)
//		{
//			prevPage = prevPage->m_prev;
//		}
//
//		if (prevPage) {
//			dgAssert(0);
//		} else {
//			dgAssert(0);
//		}
	}
}

void dgMemoryAllocator::dgMemoryBeam::Init(dgInt32 size, dgMemoryAllocator* const allocator)
{
	m_beamSize = size;
	m_allocator = allocator;
}

dgMemoryAllocator::dgMemoryAllocator()
{
//	for (dgInt32 i = 0; i < DG_MEMORY_BEAMS_COUNT; i++) {
//		dgInt32 size = ((dgInt32 (sizeof (dgMemoryGranularity) * (dgPow(dgFloat32(1.6f), i + 2) - dgPow(dgFloat32(1.6f), i + 1))) + sizeof(dgMemoryGranularity) - 1) & -dgInt32 (sizeof(dgMemoryGranularity))) - sizeof (dgMemoryHeader);
//		m_beams[i].Init (size, this);
//	}

	dgInt32 index = 0;
	dgInt32 size0 = 0;
	dgFloat32 base = dgFloat32(1.3f);
	dgFloat32 exp = dgFloat32 (0.0f);
	while (index < DG_MEMORY_BEAMS_COUNT) {
		dgFloat32 x0 = dgPow(base, exp);
		dgFloat32 x1 = dgPow(base, exp + dgFloat32(1.0f));
		dgInt32 size = ((dgInt32(sizeof(dgMemoryGranularity) * (x1 - x0) + sizeof(dgMemoryGranularity) - 1)) & -dgInt32(sizeof(dgMemoryGranularity))) - sizeof(dgMemoryHeader);
		exp += dgFloat32(1.0f);
		if (size > size0) {
			size0 = size;
			m_beams[index].Init(size, this);
			index++;
		}
	}
}

dgMemoryAllocator::~dgMemoryAllocator()
{
}

void *dgMemoryAllocator::operator new (size_t size)
{
	dgMemoryAllocatorBase& globalAllocator = dgGlobalAllocator::GetGlobalAllocator();
	return globalAllocator.Malloc(dgInt32 (size));
}

void dgMemoryAllocator::operator delete (void* const ptr)
{
	dgMemoryAllocatorBase& globalAllocator = dgGlobalAllocator::GetGlobalAllocator();
	globalAllocator.Free (ptr);
}

void dgMemoryAllocator::SetGlobalAllocators(dgMemAlloc malloc, dgMemFree free)
{
	dgGlobalAllocator* const globalAllocator = (dgGlobalAllocator*)&dgGlobalAllocator::GetGlobalAllocator();
	globalAllocator->SetAllocatorsCallback(malloc, free);
}

dgInt32 dgMemoryAllocator::GetGlobalMemoryUsed()
{
	dgGlobalAllocator* const globalAllocator = (dgGlobalAllocator*)&dgGlobalAllocator::GetGlobalAllocator();
	return globalAllocator->GetMemoryUsed();
}

dgInt32 dgMemoryAllocator::GetSize (void* const ptr)
{
	dgMemoryHeader* const info = ((dgMemoryHeader*)ptr) - 1;
	return info->m_size;
}

void* dgMemoryAllocator::Malloc(dgInt32 size)
{
	dgMemoryAllocatorBase& globalAllocator = dgGlobalAllocator::GetGlobalAllocator();
	if (size > m_beams[DG_MEMORY_BEAMS_COUNT-1].m_beamSize) {
		return globalAllocator.Malloc(size);
	} else {
		dgMemoryBeam* const beam = FindBeam(size);
		return beam->Malloc(size);
	}
}

void dgMemoryAllocator::Free(void* const ptr)
{
	dgMemoryHeader* const info = ((dgMemoryHeader*)ptr) - 1;
	dgMemoryAllocatorBase& globalAllocator = dgGlobalAllocator::GetGlobalAllocator();
	if (info->m_size > m_beams[DG_MEMORY_BEAMS_COUNT - 1].m_beamSize) {
		globalAllocator.Free (ptr);
	} else {
		dgMemoryBeam* const beam = FindBeam(info->m_size);
		beam->Free(ptr);
	}
}

dgMemoryAllocator::dgMemoryBeam* dgMemoryAllocator::FindBeam(dgInt32 size)
{
	dgInt32 i = m_beams[DG_MEMORY_BEAMS_COUNT / 2].m_beamSize >= size ? 0 : DG_MEMORY_BEAMS_COUNT / 2;
	for (; i < DG_MEMORY_BEAMS_COUNT; i++) {
		if (m_beams[i].m_beamSize >= size) {
			return &m_beams[i];
		}
	}
	dgAssert(0);
	return NULL;
}


#endif