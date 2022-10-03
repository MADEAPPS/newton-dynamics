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

/****************************************************************************
*
*  Visual C++ 6.0 created by: Julio Jerez
*
****************************************************************************/
#ifndef __D_HEAP_H__
#define __D_HEAP_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndMemory.h"
#include "ndClassAlloc.h"

//#define ND_HEAP_DEBUG_CHECK


template <class dItem, class dKey>
class ndHeap : public ndClassAlloc
{
	public:
	ndHeap(ndInt32 maxElements);
	ndHeap(const void * const buffer, ndInt32 sizeInBytes);
	~ndHeap();

	void Flush();
	dKey MaxValue() const;
	dKey Value(ndInt32 i = 0) const;
	ndInt32 GetCount() const;
	ndInt32 GetMaxCount() const;
	const dItem& operator[] (ndInt32 i) const;
	ndInt32 Find(dItem &obj);
	ndInt32 Find(dKey key);

	void Pop();
	void Sort();
	void Remove(ndInt32 Index);
	void Push(dItem &obj, dKey key);
	bool SanityCheck();

	protected:
	struct dRecord
	{
		dRecord(dKey key, const dItem& obj)
			:m_key(key)
			,m_obj(obj)
		{
		}

		dKey m_key;
		dItem m_obj;
	};

	dRecord* m_pool;
	ndInt32 m_curCount;
	ndInt32 m_maxCount;
	bool m_bufferIsOwnned;
};



// *************************
//
// implementation
//
// *************************

template <class dItem, class dKey>
ndHeap<dItem, dKey>::ndHeap(ndInt32 maxElements)
	:ndClassAlloc()
	,m_pool((dRecord *)ndMemory::Malloc(maxElements * sizeof(dRecord)))
	,m_curCount(0)
	,m_maxCount(maxElements)
	,m_bufferIsOwnned(true)
{
	Flush();
}

template <class dItem, class dKey>
ndHeap<dItem, dKey>::ndHeap(const void * const buffer, ndInt32 sizeInBytes)
	:ndClassAlloc()
	,m_pool((dRecord *)buffer)
	,m_curCount(0)
	,m_maxCount(ndInt32(sizeInBytes / sizeof(dRecord)))
	,m_bufferIsOwnned(false)
{
	Flush();
}

template <class dItem, class dKey>
ndHeap<dItem, dKey>::~ndHeap()
{
	if (m_bufferIsOwnned)
	{
		ndMemory::Free(m_pool);
	}
}

template <class dItem, class dKey>
dKey ndHeap<dItem, dKey>::Value(ndInt32 i) const
{
	return m_pool[i].m_key;
}

template <class dItem, class dKey>
ndInt32 ndHeap<dItem, dKey>::GetCount() const
{
	return m_curCount;
}

template <class dItem, class dKey>
void ndHeap<dItem, dKey>::Flush()
{
	m_curCount = 0;

#ifdef _DEBUG
	//	ndHeap<dItem,dKey>::m_pool[ndHeap<dItem,dKey>::m_curCount].m_key = dKey (0);
#endif
}

template <class dItem, class dKey>
dKey ndHeap<dItem, dKey>::MaxValue() const
{
	return m_pool[0].m_key;
}

template <class dItem, class dKey>
ndInt32 ndHeap<dItem, dKey>::GetMaxCount() const
{
	return m_maxCount;
}

template <class dItem, class dKey>
ndInt32 ndHeap<dItem, dKey>::Find(dItem &obj)
{
	// For now let perform a linear search
	// this is efficient if the size of the heap is small
	// ex: m_curCount < 32
	// this will be change to a binary search in the heap should the 
	// the size of the heap get larger than 32
	//	ndAssert (m_curCount <= 32);
	for (ndInt32 i = 0; i < m_curCount; ++i)
	{
		if (m_pool[i].obj == obj)
		{
			return i;
		}
	}
	return -1;
}

template <class dItem, class dKey>
ndInt32 ndHeap<dItem, dKey>::Find(dKey key)
{
	// ex: m_curCount < 32
	// this will be change to a binary search in the heap should the 
	// the size of the heap get larger than 32
	ndAssert(m_curCount <= 32);
	for (ndInt32 i = 0; i < m_curCount; ++i)
	{
		if (m_pool[i].m_key == key)
		{
			return i;
		}
	}
	return -1;
}

template <class dItem, class dKey>
const dItem& ndHeap<dItem, dKey>::operator[] (ndInt32 i) const
{
	ndAssert(i <= m_curCount);
	return m_pool[i].m_obj;
}

template <class dItem, class dKey>
bool ndHeap<dItem, dKey>::SanityCheck()
{
	#ifdef ND_HEAP_DEBUG_CHECK
	for (ndInt32 i = 0; i < m_curCount; ++i)
	{
		ndInt32 i1 = 2 * i + 1;
		ndInt32 i2 = 2 * i + 2;
		if ((i1 < m_curCount) && (ndHeap<dItem, dKey>::m_pool[i].m_key < ndHeap<dItem, dKey>::m_pool[i1].m_key))
		{
			return false;
		}
		if ((i2 < m_curCount) && (ndHeap<dItem, dKey>::m_pool[i].m_key < ndHeap<dItem, dKey>::m_pool[i2].m_key))
		{
			return false;
		}
	}
	#endif
	return true;
}

template <class dItem, class dKey>
void ndHeap<dItem, dKey>::Push(dItem &obj, dKey key)
{
	ndHeap<dItem, dKey>::m_curCount++;

	ndInt32 i = ndHeap<dItem, dKey>::m_curCount;
	for (ndInt32 j = 0; i; i = j)
	{
		j = i >> 1;
		if (!j || (ndHeap<dItem, dKey>::m_pool[j - 1].m_key > key))
		{
			break;
		}
		ndHeap<dItem, dKey>::m_pool[i - 1] = ndHeap<dItem, dKey>::m_pool[j - 1];
	}
	ndAssert(i);
	ndHeap<dItem, dKey>::m_pool[i - 1].m_key = key;
	ndHeap<dItem, dKey>::m_pool[i - 1].m_obj = obj;

	ndAssert(SanityCheck());
}

template <class dItem, class dKey>
void ndHeap<dItem, dKey>::Pop()
{
	Remove(0);
}

template <class dItem, class dKey>
void ndHeap<dItem, dKey>::Remove(ndInt32 index)
{
	ndHeap<dItem, dKey>::m_curCount--;
	ndHeap<dItem, dKey>::m_pool[index] = ndHeap<dItem, dKey>::m_pool[ndHeap<dItem, dKey>::m_curCount];
	while (index && ndHeap<dItem, dKey>::m_pool[(index - 1) >> 1].m_key < ndHeap<dItem, dKey>::m_pool[index].m_key)
	{
		ndSwap(ndHeap<dItem, dKey>::m_pool[(index - 1) >> 1], ndHeap<dItem, dKey>::m_pool[index]);
		index = (index - 1) >> 1;
	}

	while ((2 * index + 1) < ndHeap<dItem, dKey>::m_curCount)
	{
		ndInt32 i0 = 2 * index + 1;
		ndInt32 i1 = 2 * index + 2;
		if (i1 < ndHeap<dItem, dKey>::m_curCount)
		{
			i0 = (ndHeap<dItem, dKey>::m_pool[i0].m_key > ndHeap<dItem, dKey>::m_pool[i1].m_key) ? i0 : i1;
			if (ndHeap<dItem, dKey>::m_pool[i0].m_key <= ndHeap<dItem, dKey>::m_pool[index].m_key)
			{
				break;
			}
			ndSwap(ndHeap<dItem, dKey>::m_pool[i0], ndHeap<dItem, dKey>::m_pool[index]);
			index = i0;
		}
		else
		{
			if (ndHeap<dItem, dKey>::m_pool[i0].m_key > ndHeap<dItem, dKey>::m_pool[index].m_key)
			{
				ndSwap(ndHeap<dItem, dKey>::m_pool[i0], ndHeap<dItem, dKey>::m_pool[index]);
			}
			index = i0;
		}
	}
	ndAssert(SanityCheck());
}

template <class dItem, class dKey>
void ndHeap<dItem, dKey>::Sort()
{
	ndInt32 count = ndHeap<dItem, dKey>::m_curCount;
	for (ndInt32 i = 1; i < count; ++i)
	{
		dKey key(ndHeap<dItem, dKey>::m_pool[0].m_key);
		dItem obj(ndHeap<dItem, dKey>::m_pool[0].m_obj);

		Pop();

		ndHeap<dItem, dKey>::m_pool[ndHeap<dItem, dKey>::m_curCount].m_key = key;
		ndHeap<dItem, dKey>::m_pool[ndHeap<dItem, dKey>::m_curCount].m_obj = obj;
	}

	ndHeap<dItem, dKey>::m_curCount = count;
	for (ndInt32 i = 0; i < count / 2; ++i)
	{
		dKey key(ndHeap<dItem, dKey>::m_pool[i].m_key);
		dItem obj(ndHeap<dItem, dKey>::m_pool[i].m_obj);

		ndHeap<dItem, dKey>::m_pool[i].m_key = ndHeap<dItem, dKey>::m_pool[count - i - 1].m_key;
		ndHeap<dItem, dKey>::m_pool[i].m_obj = ndHeap<dItem, dKey>::m_pool[count - i - 1].m_obj;

		ndHeap<dItem, dKey>::m_pool[count - i - 1].m_key = key;
		ndHeap<dItem, dKey>::m_pool[count - i - 1].m_obj = obj;
	}
	ndAssert(SanityCheck());
}

// *****************************************
//
//  two typical instances of heaps, up and down.
//
// *****************************************
template <class dKey>
class ndDownHeapCompare
{
	public:
	ndDownHeapCompare(dKey key)
		:m_key(key)
	{
	}

	bool operator> (const ndDownHeapCompare<dKey>& key) const
	{
		return m_key > key.m_key;
	}

	bool operator< (const ndDownHeapCompare<dKey>& key) const
	{
		return m_key < key.m_key;
	}

	bool operator<= (const ndDownHeapCompare<dKey>& key) const
	{
		return m_key <= key.m_key;
	}

	dKey m_key;
};

template <class dItem, class dKey>
class ndDownHeap : public ndHeap<dItem, ndDownHeapCompare<dKey>>
{
	public:
	ndDownHeap(ndInt32 maxElements)
		:ndHeap<dItem, ndDownHeapCompare<dKey>>(maxElements)
	{
	}

	ndDownHeap(const void* const buffer, ndInt32 sizeInBytes)
		:ndHeap<dItem, ndDownHeapCompare<dKey>>(buffer, sizeInBytes)
	{
	}

	dKey Value(ndInt32 i = 0) const
	{
		const ndDownHeapCompare<dKey> key(ndHeap<dItem, ndDownHeapCompare<dKey>>::Value(i));
		return key.m_key;
	}
};

template <class dKey>
class ndUpHeapCompare
{
	public:
	ndUpHeapCompare(dKey key)
		:m_key(key)
	{
	}

	bool operator> (const ndUpHeapCompare<dKey>& key) const
	{
		return m_key < key.m_key;
	}

	bool operator< (const ndUpHeapCompare<dKey>& key) const
	{
		return m_key > key.m_key;
	}

	bool operator<= (const ndUpHeapCompare<dKey>& key) const
	{
		return m_key >= key.m_key;
	}

	dKey m_key;
};

template <class dItem, class dKey>
class ndUpHeap : public ndHeap<dItem, ndUpHeapCompare<dKey>>
{
	public:
	ndUpHeap(ndInt32 maxElements)
		:ndHeap<dItem, ndUpHeapCompare<dKey>>(maxElements)
	{
	}

	ndUpHeap(const void* const buffer, ndInt32 sizeInBytes)
		:ndHeap<dItem, ndUpHeapCompare<dKey>>(buffer, sizeInBytes)
	{
	}

	dKey Value(ndInt32 i = 0) const
	{
		const ndUpHeapCompare<dKey> key(ndHeap<dItem, ndUpHeapCompare<dKey>>::Value(i));
		return key.m_key;
	}
};

#endif
