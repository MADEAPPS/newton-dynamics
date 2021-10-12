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
#include "dMemory.h"
#include "dClassAlloc.h"
#include "dFixSizeArray.h"
#include "dContainersAlloc.h"

#define D_FREELIST_DICTIONARY_SIZE 64

class dFreeListEntry
{
	public:
	dFreeListEntry* m_next;
};

class dFreeListHeader
{
	public:
	dFreeListEntry* m_headPointer;
	dInt32 m_count;
	dInt32 m_schunkSize;
};

class dFreeListDictionary: public dFixSizeArray<dFreeListHeader, D_FREELIST_DICTIONARY_SIZE>
{
	public:
	dFreeListDictionary()
		:dFixSizeArray<dFreeListHeader, D_FREELIST_DICTIONARY_SIZE>()
		,m_lock()
	{
	}

	static dFreeListDictionary& GetHeader()
	{
		static dFreeListDictionary dictionary;
		return dictionary;
	}

	dFreeListHeader* FindEntry(dInt32 size)
	{
		dInt32 i0 = 0;
		dInt32 i1 = GetCount() - 1;
		dFreeListDictionary& me = *this;
		while ((i1 - i0 > 4))
		{
			dInt32 mid = (i1 + i0) / 2;
			if (me[mid].m_schunkSize <= size)
			{
				i0 = mid;
			}
			else
			{
				i1 = mid;
			}
		}

		for (dInt32 i = i0; i <= i1; i++)
		{
			if (me[i].m_schunkSize == size)
			{
				return &me[i];
			}
		}

#ifdef _DEBUG
		for (dInt32 i = 0; i < GetCount(); i++)
		{
			dAssert(me[i].m_schunkSize != size);
		}
#endif

		dFreeListHeader header;
		header.m_count = 0;
		header.m_schunkSize = size;
		header.m_headPointer = nullptr;
		PushBack(header);
		dInt32 index = 0;
		for (dInt32 i = GetCount() - 2; i >= 0; i--)
		{
			if (size < me[i].m_schunkSize)
			{
				me[i + 1] = me[i];
				me[i] = header;
				index = i;
			}
			else
			{
				break;
			}
		}
		return &me[index];
	}

	void* Malloc(dInt32 size)
	{
		dScopeSpinLock lock(m_lock);
		dFreeListHeader* const header = FindEntry(dMemory::CalculateBufferSize(size));
		dAssert(header->m_count >= 0);
		if (header->m_count)
		{
			header->m_count--;
			dFreeListEntry* const self = header->m_headPointer;
			header->m_headPointer = self->m_next;
			return self;
		}
		void* const ptr = dMemory::Malloc(size);
		dAssert(dMemory::GetSize(ptr) == dMemory::CalculateBufferSize(size));
		return ptr;
	}

	void Free(void* ptr)
	{
		dScopeSpinLock lock(m_lock);
		dFreeListHeader* const header = FindEntry(dMemory::GetSize(ptr));
		dAssert(header);
		dFreeListEntry* const self = (dFreeListEntry*)ptr;

		self->m_next = header->m_headPointer;
		header->m_count++;
		header->m_headPointer = self;
	}

	void Flush(dFreeListHeader* const header)
	{
		dFreeListEntry* next;
		for (dFreeListEntry* node = header->m_headPointer; node; node = next)
		{
			next = node->m_next;
			dMemory::Free(node);
		}
		header->m_count = 0;
		header->m_headPointer = nullptr;
	}

	void Flush()
	{
		dScopeSpinLock lock(m_lock);
		dFreeListDictionary& me = *this;
		for (dInt32 i = 0; i < GetCount(); i++)
		{
			dFreeListHeader* const header = &me[i];
			Flush(header);
		}
		SetCount(0);
	}

	void Flush(dInt32 size)
	{
		dScopeSpinLock lock(m_lock);
		dFreeListHeader* const header = FindEntry(dMemory::CalculateBufferSize(size));
		Flush(header);
	}

	dSpinLock m_lock;
};

void dFreeListAlloc::Flush()
{
	dFreeListDictionary& dictionary = dFreeListDictionary::GetHeader();
	dictionary.Flush();
}

void* dFreeListAlloc::operator new (size_t size)
{
	dFreeListDictionary& dictionary = dFreeListDictionary::GetHeader();
	return dictionary.Malloc(dInt32 (size));
}

void dFreeListAlloc::operator delete (void* ptr)
{
	dFreeListDictionary& dictionary = dFreeListDictionary::GetHeader();
	dictionary.Free(ptr);
}

void dFreeListAlloc::Flush(dInt32 size)
{
	dFreeListDictionary& dictionary = dFreeListDictionary::GetHeader();
	dictionary.Flush(size);
}

