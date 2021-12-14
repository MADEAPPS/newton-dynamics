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

/****************************************************************************
*
*  Visual C++ 6.0 created by: Julio Jerez
*
****************************************************************************/
#ifndef __NDHeapBase__
#define __NDHeapBase__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndMemory.h"
#include "ndClassAlloc.h"

//#define ND_HEAP_DEBUG_CHECK

template <class OBJECT, class KEY>
class ndHeapBase: public ndClassAlloc
{
	protected:
	struct RECORD 
	{
		KEY m_key;
		OBJECT m_obj;

		RECORD (KEY key, const OBJECT& obj)
			:m_key(key), m_obj(obj)
		{
		}
	};

	ndHeapBase (ndInt32 maxElements);
	ndHeapBase (const void * const buffer, ndInt32 sizeInBytes);
	~ndHeapBase ();
	
	public:
	void Flush (); 
	KEY MaxValue() const; 
	KEY Value(ndInt32 i = 0) const;
	ndInt32 GetCount() const;
	ndInt32 GetMaxCount() const;
	const OBJECT& operator[] (ndInt32 i) const;
	ndInt32 Find (OBJECT &obj);
	ndInt32 Find (KEY key);

	RECORD *m_pool;
	ndInt32 m_curCount;
	ndInt32 m_maxCount;
	bool m_bufferIsOwnned;
};

template <class OBJECT, class KEY>
class ndDownHeap: public ndHeapBase<OBJECT, KEY>
{
	public:
	ndDownHeap (ndInt32 maxElements);
	ndDownHeap (const void * const buffer, ndInt32 sizeInBytes);

	void Pop();
	void Push (OBJECT &obj, KEY key);
	void Sort ();
	void Remove (ndInt32 Index);
	bool SanityCheck();
};

template <class OBJECT, class KEY>
class ndUpHeap: public ndHeapBase<OBJECT, KEY>
{
	public:
	ndUpHeap (ndInt32 maxElements);
	ndUpHeap (const void * const buffer, ndInt32 sizeInBytes);

	void Pop();
	void Push (OBJECT &obj, KEY key);
	void Sort ();
	void Remove (ndInt32 Index);
	bool SanityCheck();
};

template <class OBJECT, class KEY>
ndHeapBase<OBJECT,KEY>::ndHeapBase (ndInt32 maxElements)
	:ndClassAlloc()
	,m_pool((RECORD *)ndMemory::Malloc(maxElements * sizeof(RECORD)))
	,m_curCount(0)
	,m_maxCount(maxElements)
	,m_bufferIsOwnned(true)
{
	Flush();
}

template <class OBJECT, class KEY>
ndHeapBase<OBJECT,KEY>::ndHeapBase (const void * const buffer, ndInt32 sizeInBytes)
	:ndClassAlloc()
	,m_pool((RECORD *)buffer)
	,m_curCount(0)
	,m_maxCount(ndInt32(sizeInBytes / sizeof(RECORD)))
	,m_bufferIsOwnned(false)
{
	Flush();
}

template <class OBJECT, class KEY>
ndHeapBase<OBJECT,KEY>::~ndHeapBase ()
{   
	if (m_bufferIsOwnned)
	{
		ndMemory::Free (m_pool);
	}
}

template <class OBJECT, class KEY>
KEY ndHeapBase<OBJECT,KEY>::Value(ndInt32 i) const
{
	return m_pool[i].m_key;
}

template <class OBJECT, class KEY>
ndInt32 ndHeapBase<OBJECT,KEY>::GetCount() const
{ 
	return m_curCount;
}

template <class OBJECT, class KEY>
void ndHeapBase<OBJECT,KEY>::Flush () 
{
	m_curCount = 0;

	#ifdef _DEBUG
//	ndHeapBase<OBJECT,KEY>::m_pool[ndHeapBase<OBJECT,KEY>::m_curCount].m_key = KEY (0);
	#endif
}

template <class OBJECT, class KEY>
KEY ndHeapBase<OBJECT,KEY>::MaxValue() const 
{
	return m_pool[0].m_key;
}

template <class OBJECT, class KEY>
ndInt32 ndHeapBase<OBJECT,KEY>::GetMaxCount() const
{ 
	return m_maxCount;
}

template <class OBJECT, class KEY>
ndInt32 ndHeapBase<OBJECT,KEY>::Find (OBJECT &obj)
{
	// For now let perform a linear search
	// this is efficient if the size of the heap is small
	// ex: m_curCount < 32
	// this will be change to a binary search in the heap should the 
	// the size of the heap get larger than 32
	//	dAssert (m_curCount <= 32);
	for (ndInt32 i = 0; i < m_curCount; i ++) 
	{
		if (m_pool[i].obj == obj) 
		{
			return i;
		}
	}
	return - 1;
}

template <class OBJECT, class KEY>
ndInt32 ndHeapBase<OBJECT,KEY>::Find (KEY key)
{
	// ex: m_curCount < 32
	// this will be change to a binary search in the heap shoud the 
	// the size of the heap get larger than 32
	dAssert (m_curCount <= 32);
	for (ndInt32 i = 0; i < m_curCount; i ++)
	{
		if (m_pool[i].m_key == key) 
		{
			return i;
		}
	}
	return - 1;
}

template <class OBJECT, class KEY>
const OBJECT& ndHeapBase<OBJECT,KEY>::operator[] (ndInt32 i) const
{ 
	dAssert (i<= m_curCount);
	return m_pool[i].m_obj;
}

// **************************************************************************
//
// down Heap
//
// **************************************************************************
template <class OBJECT, class KEY>
ndDownHeap<OBJECT,KEY>::ndDownHeap (ndInt32 maxElements)
	:ndHeapBase<OBJECT, KEY> (maxElements)
{
}

template <class OBJECT, class KEY>
ndDownHeap<OBJECT,KEY>::ndDownHeap (const void * const buffer, ndInt32 sizeInBytes)
	:ndHeapBase<OBJECT, KEY> (buffer, sizeInBytes)
{
}

template <class OBJECT, class KEY>
void ndDownHeap<OBJECT,KEY>::Push (OBJECT &obj, KEY key)
{
	ndHeapBase<OBJECT,KEY>::m_curCount ++;

	ndInt32 i = ndHeapBase<OBJECT,KEY>::m_curCount;
	for (ndInt32 j = 0; i; i = j)
	{
		j = i >> 1;
		if (!j || (ndHeapBase<OBJECT,KEY>::m_pool[j - 1].m_key > key)) 
		{
			break;
		}
		ndHeapBase<OBJECT,KEY>::m_pool[i - 1] = ndHeapBase<OBJECT,KEY>::m_pool[j - 1];
	}
	dAssert (i);
	ndHeapBase<OBJECT,KEY>::m_pool[i - 1].m_key = key;
	ndHeapBase<OBJECT,KEY>::m_pool[i - 1].m_obj = obj;

	dAssert (SanityCheck());
}

template <class OBJECT, class KEY>
void ndDownHeap<OBJECT, KEY>::Pop()
{
	//ndDownHeap<OBJECT, KEY>::Remove(0);
	Remove(0);
}

template <class OBJECT, class KEY>
void ndDownHeap<OBJECT,KEY>::Remove (ndInt32 index)
{
	ndHeapBase<OBJECT, KEY>::m_curCount--;
	ndHeapBase<OBJECT, KEY>::m_pool[index] = ndHeapBase<OBJECT, KEY>::m_pool[ndHeapBase<OBJECT, KEY>::m_curCount];
	while (index && ndHeapBase<OBJECT, KEY>::m_pool[(index - 1) >> 1].m_key < ndHeapBase<OBJECT, KEY>::m_pool[index].m_key) 
	{
		dSwap(ndHeapBase<OBJECT, KEY>::m_pool[(index - 1) >> 1], ndHeapBase<OBJECT, KEY>::m_pool[index]);
		index = (index - 1) >> 1;
	}

	while ((2 * index + 1) < ndHeapBase<OBJECT, KEY>::m_curCount) 
	{
		ndInt32 i0 = 2 * index + 1;
		ndInt32 i1 = 2 * index + 2;
		if (i1 < ndHeapBase<OBJECT, KEY>::m_curCount) 
		{
			i0 = (ndHeapBase<OBJECT, KEY>::m_pool[i0].m_key > ndHeapBase<OBJECT, KEY>::m_pool[i1].m_key) ? i0 : i1;
			if (ndHeapBase<OBJECT, KEY>::m_pool[i0].m_key <= ndHeapBase<OBJECT, KEY>::m_pool[index].m_key) 
			{
				break;
			}
			dSwap(ndHeapBase<OBJECT, KEY>::m_pool[i0], ndHeapBase<OBJECT, KEY>::m_pool[index]);
			index = i0;
		} 
		else 
		{
			if (ndHeapBase<OBJECT, KEY>::m_pool[i0].m_key > ndHeapBase<OBJECT, KEY>::m_pool[index].m_key) 
			{
				dSwap(ndHeapBase<OBJECT, KEY>::m_pool[i0], ndHeapBase<OBJECT, KEY>::m_pool[index]);
			}
			index = i0;
		}
	}
	dAssert (SanityCheck());
}

template <class OBJECT, class KEY>
void ndDownHeap<OBJECT,KEY>::Sort ()
{
	ndInt32 count = ndHeapBase<OBJECT,KEY>::m_curCount;
	for (ndInt32 i = 1; i < count; i ++) 
	{
		KEY key (ndHeapBase<OBJECT,KEY>::m_pool[0].m_key);
		OBJECT obj (ndHeapBase<OBJECT,KEY>::m_pool[0].m_obj);

		Pop();

		ndHeapBase<OBJECT,KEY>::m_pool[ndHeapBase<OBJECT,KEY>::m_curCount].m_key = key;
		ndHeapBase<OBJECT,KEY>::m_pool[ndHeapBase<OBJECT,KEY>::m_curCount].m_obj = obj;
	}

	ndHeapBase<OBJECT,KEY>::m_curCount = count;
	for (ndInt32 i = 0; i < count / 2; i ++) 
	{
		KEY key (ndHeapBase<OBJECT,KEY>::m_pool[i].m_key);
		OBJECT obj (ndHeapBase<OBJECT,KEY>::m_pool[i].m_obj);

		ndHeapBase<OBJECT,KEY>::m_pool[i].m_key = ndHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_key;
		ndHeapBase<OBJECT,KEY>::m_pool[i].m_obj = ndHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_obj;

		ndHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_key = key;
		ndHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_obj = obj;
	}
	dAssert (SanityCheck());
}

template <class OBJECT, class KEY>
bool ndDownHeap<OBJECT,KEY>::SanityCheck()
{
#ifdef ND_HEAP_DEBUG_CHECK
	for (ndInt32 i = 0; i < m_curCount; i++) 
	{
		ndInt32 i1 = 2 * i + 1;
		ndInt32 i2 = 2 * i + 2;
		if ((i1 < m_curCount) && (ndHeapBase<OBJECT, KEY>::m_pool[i].m_key < ndHeapBase<OBJECT, KEY>::m_pool[i1].m_key)) 
		{
			return false;
		}
		if ((i2 < m_curCount) && (ndHeapBase<OBJECT, KEY>::m_pool[i].m_key < ndHeapBase<OBJECT, KEY>::m_pool[i2].m_key)) 
		{
			return false;
		}
	}
#endif
	return true;
}

// **************************************************************************
//
// down Heap
//
// **************************************************************************
template <class OBJECT, class KEY>
ndUpHeap<OBJECT,KEY>::ndUpHeap (ndInt32 maxElements)
//	:ndHeapBase<OBJECT, KEY> (maxElements, allocator)
	:ndHeapBase<OBJECT, KEY>(maxElements)
{
}

template <class OBJECT, class KEY>
ndUpHeap<OBJECT,KEY>::ndUpHeap (const void * const buffer, ndInt32 sizeInBytes)
	:ndHeapBase<OBJECT, KEY> (buffer, sizeInBytes)
{
}

template <class OBJECT, class KEY>
bool ndUpHeap<OBJECT,KEY>::SanityCheck()
{
#ifdef ND_HEAP_DEBUG_CHECK
	for (ndInt32 i = 0; i < m_curCount; i ++) 
	{
		ndInt32 i1 = 2 * i + 1; 
		ndInt32 i2 = 2 * i + 2; 
		if ((i1 < m_curCount) && (ndHeapBase<OBJECT,KEY>::m_pool[i].m_key > ndHeapBase<OBJECT,KEY>::m_pool[i1].m_key)) 
		{
			return false;
		}
		if ((i2 < m_curCount) && (ndHeapBase<OBJECT,KEY>::m_pool[i].m_key > ndHeapBase<OBJECT,KEY>::m_pool[i2].m_key)) 
		{
			return false;
		}
	}
#endif
	return true;
}

template <class OBJECT, class KEY>
void ndUpHeap<OBJECT,KEY>::Push (OBJECT &obj, KEY key)
{
	ndHeapBase<OBJECT,KEY>::m_curCount ++;

	ndInt32 i = ndHeapBase<OBJECT,KEY>::m_curCount;
	for (ndInt32 j = 0; i; i = j)
	{
		j = i >> 1;
		if (!j || (ndHeapBase<OBJECT,KEY>::m_pool[j - 1].m_key < key)) 
		{
			break;
		}
		ndHeapBase<OBJECT,KEY>::m_pool[i - 1] = ndHeapBase<OBJECT,KEY>::m_pool[j - 1];
	}
	dAssert (i);
	ndHeapBase<OBJECT,KEY>::m_pool[i - 1].m_key = key;
	ndHeapBase<OBJECT,KEY>::m_pool[i - 1].m_obj = obj;
	dAssert (SanityCheck());
}

template <class OBJECT, class KEY>
void ndUpHeap<OBJECT, KEY>::Pop()
{ 
	Remove(0); 
}

template <class OBJECT, class KEY>
void ndUpHeap<OBJECT,KEY>::Sort ()
{
	ndInt32 count = ndHeapBase<OBJECT,KEY>::m_curCount;
	for (ndInt32 i = 1; i < count; i ++) 
	{
		KEY key (ndHeapBase<OBJECT,KEY>::m_pool[0].m_key);
		OBJECT obj (ndHeapBase<OBJECT,KEY>::m_pool[0].m_obj);

		Pop();

		ndHeapBase<OBJECT,KEY>::m_pool[ndHeapBase<OBJECT,KEY>::m_curCount].m_key = key;
		ndHeapBase<OBJECT,KEY>::m_pool[ndHeapBase<OBJECT,KEY>::m_curCount].m_obj = obj;
	}

	ndHeapBase<OBJECT,KEY>::m_curCount = count;
	for (ndInt32 i = 0; i < count / 2; i ++) 
	{
		KEY key (ndHeapBase<OBJECT,KEY>::m_pool[i].m_key);
		OBJECT obj (ndHeapBase<OBJECT,KEY>::m_pool[i].m_obj);

		ndHeapBase<OBJECT,KEY>::m_pool[i].m_key = ndHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_key;
		ndHeapBase<OBJECT,KEY>::m_pool[i].m_obj = ndHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_obj;

		ndHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_key = key;
		ndHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_obj = obj;
	}
	dAssert (SanityCheck());
}

template <class OBJECT, class KEY>
void ndUpHeap<OBJECT,KEY>::Remove (ndInt32 index)
{
	ndHeapBase<OBJECT, KEY>::m_curCount--;
	ndHeapBase<OBJECT, KEY>::m_pool[index] = ndHeapBase<OBJECT, KEY>::m_pool[ndHeapBase<OBJECT, KEY>::m_curCount];
	while (index && ndHeapBase<OBJECT, KEY>::m_pool[(index - 1) >> 1].m_key > ndHeapBase<OBJECT, KEY>::m_pool[index].m_key) 
	{
		dSwap(ndHeapBase<OBJECT, KEY>::m_pool[(index - 1) >> 1], ndHeapBase<OBJECT, KEY>::m_pool[index]);
		index = (index - 1) >> 1;
	}

	while ((2 * index + 1) < ndHeapBase<OBJECT, KEY>::m_curCount) 
	{
		ndInt32 i0 = 2 * index + 1;
		ndInt32 i1 = 2 * index + 2;
		if (i1 < ndHeapBase<OBJECT, KEY>::m_curCount) 
		{
			i0 = (ndHeapBase<OBJECT, KEY>::m_pool[i0].m_key < ndHeapBase<OBJECT, KEY>::m_pool[i1].m_key) ? i0 : i1;
			if (ndHeapBase<OBJECT, KEY>::m_pool[i0].m_key >= ndHeapBase<OBJECT, KEY>::m_pool[index].m_key) 
			{
				break;
			}
			dSwap(ndHeapBase<OBJECT, KEY>::m_pool[i0], ndHeapBase<OBJECT, KEY>::m_pool[index]);
			index = i0;
		} 
		else 
		{
			if (ndHeapBase<OBJECT, KEY>::m_pool[i0].m_key < ndHeapBase<OBJECT, KEY>::m_pool[index].m_key) 
			{
				dSwap(ndHeapBase<OBJECT, KEY>::m_pool[i0], ndHeapBase<OBJECT, KEY>::m_pool[index]);
			}
			index = i0;
		}
	}
	dAssert (SanityCheck());
}

#endif