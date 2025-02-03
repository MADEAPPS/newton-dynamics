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
#include "ndUtils.h"
#include "ndMemory.h"
#include "ndClassAlloc.h"
#include "ndFixSizeArray.h"
#include "ndContainersAlloc.h"
#include "ndThreadSyncUtils.h"

#define D_FREELIST_DICTIONARY_SIZE 64

class ndFreeListEntry
{
	public:
	ndFreeListEntry* m_next;
};

class ndFreeListHeader
{
	public:
	ndInt32 m_count;
	ndInt32 m_schunkSize;
	ndFreeListEntry* m_headPointer;
};

class ndFreeListDictionary: public ndFixSizeArray<ndFreeListHeader, D_FREELIST_DICTIONARY_SIZE>
{
	public:
	ndFreeListDictionary()
		:ndFixSizeArray<ndFreeListHeader, D_FREELIST_DICTIONARY_SIZE>()
		,m_lock()
	{
	}

	~ndFreeListDictionary()
	{
		Flush();
	}

	static ndFreeListDictionary& GetHeader()
	{
		static ndFreeListDictionary dictionary;
		return dictionary;
	}

	void Flush(ndFreeListHeader* const header)
	{
		ndFreeListEntry* next;
		for (ndFreeListEntry* node = header->m_headPointer; node; node = next)
		{
			next = node->m_next;
			ndMemory::Free(node);
		}
		header->m_count = 0;
		header->m_headPointer = nullptr;
	}

	void* Malloc(ndInt32 size)
	{
		{
			ndScopeSpinLock lock(m_lock);
			ndFreeListHeader* const header = FindEntry(ndInt32(ndMemory::CalculateBufferSize(size_t(size))));
			ndAssert(header->m_count >= 0);
			if (header->m_count)
			{
				header->m_count--;
				ndFreeListEntry* const self = header->m_headPointer;
				header->m_headPointer = self->m_next;

				#if defined (D_MEMORY_SANITY_CHECK) && defined(_DEBUG)
				ndAssert(ndMemory::CheckMemory(self));
				#endif
				return self;
			}
		}
		void* const ptr = ndMemory::Malloc(size_t(size));
		ndAssert(ndMemory::GetSize(ptr) == ndMemory::CalculateBufferSize(size_t(size)));
		return ptr;
	}

	void Free(void* const ptr)
	{
		ndScopeSpinLock lock(m_lock);
		#if defined (D_MEMORY_SANITY_CHECK) && defined(_DEBUG)
		ndAssert(ndMemory::CheckMemory(ptr));
		#endif

		ndFreeListHeader* const header = FindEntry(ndInt32(ndMemory::GetSize(ptr)));
		ndAssert(header);
		ndFreeListEntry* const self = (ndFreeListEntry*)ptr;

		self->m_next = header->m_headPointer;
		header->m_count++;
		header->m_headPointer = self;
	}

	void Flush()
	{
		ndScopeSpinLock lock(m_lock);
		ndFreeListDictionary& me = *this;
		for (ndInt32 i = 0; i < GetCount(); ++i)
		{
			ndFreeListHeader* const header = &me[i];
			Flush(header);
		}
		SetCount(0);
	}

	void Flush(ndInt32 size)
	{
		ndScopeSpinLock lock(m_lock);
		ndFreeListHeader* const header = FindEntry(ndInt32(ndMemory::CalculateBufferSize(size_t(size))));
		Flush(header);
	}

	private:
	ndFreeListHeader* FindEntry(ndInt32 size)
	{
		ndInt32 i0 = 0;
		ndInt32 i1 = GetCount() - 1;
		ndFreeListDictionary& me = *this;
		while ((i1 - i0 > 4))
		{
			ndInt32 mid = (i1 + i0) / 2;
			if (me[mid].m_schunkSize <= size)
			{
				i0 = mid;
			}
			else
			{
				i1 = mid;
			}
		}

		for (ndInt32 i = i0; i <= i1; ++i)
		{
			if (me[i].m_schunkSize == size)
			{
				return &me[i];
			}
		}

		#ifdef _DEBUG
			for (ndInt32 i = 0; i < GetCount(); ++i)
			{
				ndAssert(me[i].m_schunkSize != size);
			}
		#endif

		ndFreeListHeader header;
		header.m_count = 0;
		header.m_schunkSize = size;
		header.m_headPointer = nullptr;
		PushBack(header);
		ndInt32 index = GetCount() - 1;
		for (ndInt32 i = GetCount() - 2; i >= 0; --i)
		{
			if (size < me[i].m_schunkSize)
			{
				me[i + 1] = me[i];
				me[i + 0] = header;
				index = i;
			}
			else
			{
				break;
			}
		}
		return &me[index];
	}

	ndSpinLock m_lock;
};

void ndFreeListAlloc::Flush()
{
	ndFreeListDictionary& dictionary = ndFreeListDictionary::GetHeader();
	dictionary.Flush();
}

void* ndFreeListAlloc::operator new (size_t size)
{
	ndFreeListDictionary& dictionary = ndFreeListDictionary::GetHeader();
	return dictionary.Malloc(ndInt32 (size));
}

void ndFreeListAlloc::operator delete (void* ptr)
{
	ndFreeListDictionary& dictionary = ndFreeListDictionary::GetHeader();
	dictionary.Free(ptr);
}

void ndFreeListAlloc::Flush(ndInt32 size)
{
	ndFreeListDictionary& dictionary = ndFreeListDictionary::GetHeader();
	dictionary.Flush(size);
}

