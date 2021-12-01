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

//#define DG_HEAP_DEBUG_CHECK

template <class OBJECT, class KEY>
class dHeapBase: public dClassAlloc
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

	dHeapBase (dInt32 maxElements);
	dHeapBase (const void * const buffer, dInt32 sizeInBytes);
	~dHeapBase ();
	
	public:
	void Flush (); 
	KEY MaxValue() const; 
	KEY Value(dInt32 i = 0) const;
	dInt32 GetCount() const;
	dInt32 GetMaxCount() const;
	const OBJECT& operator[] (dInt32 i) const;
	dInt32 Find (OBJECT &obj);
	dInt32 Find (KEY key);

	RECORD *m_pool;
	dInt32 m_curCount;
	dInt32 m_maxCount;
	bool m_bufferIsOwnned;
};

template <class OBJECT, class KEY>
class dDownHeap: public dHeapBase<OBJECT, KEY>
{
	public:
	dDownHeap (dInt32 maxElements);
	dDownHeap (const void * const buffer, dInt32 sizeInBytes);

	void Pop();
	void Push (OBJECT &obj, KEY key);
	void Sort ();
	void Remove (dInt32 Index);
	bool SanityCheck();
};

template <class OBJECT, class KEY>
class dUpHeap: public dHeapBase<OBJECT, KEY>
{
	public:
	dUpHeap (dInt32 maxElements);
	dUpHeap (const void * const buffer, dInt32 sizeInBytes);

	void Pop();
	void Push (OBJECT &obj, KEY key);
	void Sort ();
	void Remove (dInt32 Index);
	bool SanityCheck();
};

template <class OBJECT, class KEY>
dHeapBase<OBJECT,KEY>::dHeapBase (dInt32 maxElements)
	:dClassAlloc()
	,m_pool((RECORD *)dMemory::Malloc(maxElements * sizeof(RECORD)))
	,m_curCount(0)
	,m_maxCount(maxElements)
	,m_bufferIsOwnned(true)
{
	Flush();
}

template <class OBJECT, class KEY>
dHeapBase<OBJECT,KEY>::dHeapBase (const void * const buffer, dInt32 sizeInBytes)
	:dClassAlloc()
	,m_pool((RECORD *)buffer)
	,m_curCount(0)
	,m_maxCount(dInt32(sizeInBytes / sizeof(RECORD)))
	,m_bufferIsOwnned(false)
{
	Flush();
}

template <class OBJECT, class KEY>
dHeapBase<OBJECT,KEY>::~dHeapBase ()
{   
	if (m_bufferIsOwnned)
	{
		dMemory::Free (m_pool);
	}
}

template <class OBJECT, class KEY>
KEY dHeapBase<OBJECT,KEY>::Value(dInt32 i) const
{
	return m_pool[i].m_key;
}

template <class OBJECT, class KEY>
dInt32 dHeapBase<OBJECT,KEY>::GetCount() const
{ 
	return m_curCount;
}

template <class OBJECT, class KEY>
void dHeapBase<OBJECT,KEY>::Flush () 
{
	m_curCount = 0;

	#ifdef _DEBUG
//	dHeapBase<OBJECT,KEY>::m_pool[dHeapBase<OBJECT,KEY>::m_curCount].m_key = KEY (0);
	#endif
}

template <class OBJECT, class KEY>
KEY dHeapBase<OBJECT,KEY>::MaxValue() const 
{
	return m_pool[0].m_key;
}

template <class OBJECT, class KEY>
dInt32 dHeapBase<OBJECT,KEY>::GetMaxCount() const
{ 
	return m_maxCount;
}

template <class OBJECT, class KEY>
dInt32 dHeapBase<OBJECT,KEY>::Find (OBJECT &obj)
{
	// For now let perform a linear search
	// this is efficient if the size of the heap is small
	// ex: m_curCount < 32
	// this will be change to a binary search in the heap should the 
	// the size of the heap get larger than 32
	//	dAssert (m_curCount <= 32);
	for (dInt32 i = 0; i < m_curCount; i ++) 
	{
		if (m_pool[i].obj == obj) 
		{
			return i;
		}
	}
	return - 1;
}

template <class OBJECT, class KEY>
dInt32 dHeapBase<OBJECT,KEY>::Find (KEY key)
{
	// ex: m_curCount < 32
	// this will be change to a binary search in the heap shoud the 
	// the size of the heap get larger than 32
	dAssert (m_curCount <= 32);
	for (dInt32 i = 0; i < m_curCount; i ++)
	{
		if (m_pool[i].m_key == key) 
		{
			return i;
		}
	}
	return - 1;
}

template <class OBJECT, class KEY>
const OBJECT& dHeapBase<OBJECT,KEY>::operator[] (dInt32 i) const
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
dDownHeap<OBJECT,KEY>::dDownHeap (dInt32 maxElements)
	:dHeapBase<OBJECT, KEY> (maxElements)
{
}

template <class OBJECT, class KEY>
dDownHeap<OBJECT,KEY>::dDownHeap (const void * const buffer, dInt32 sizeInBytes)
	:dHeapBase<OBJECT, KEY> (buffer, sizeInBytes)
{
}

template <class OBJECT, class KEY>
void dDownHeap<OBJECT,KEY>::Push (OBJECT &obj, KEY key)
{
	dHeapBase<OBJECT,KEY>::m_curCount ++;

	//dInt32 j;
	dInt32 i = dHeapBase<OBJECT,KEY>::m_curCount;
	for (dInt32 j = 0; i; i = j)
	{
		j = i >> 1;
		if (!j || (dHeapBase<OBJECT,KEY>::m_pool[j - 1].m_key > key)) 
		{
			break;
		}
		dHeapBase<OBJECT,KEY>::m_pool[i - 1] = dHeapBase<OBJECT,KEY>::m_pool[j - 1];
	}
	dAssert (i);
	dHeapBase<OBJECT,KEY>::m_pool[i - 1].m_key = key;
	dHeapBase<OBJECT,KEY>::m_pool[i - 1].m_obj = obj;

	dAssert (SanityCheck());
}

template <class OBJECT, class KEY>
void dDownHeap<OBJECT, KEY>::Pop()
{
	//dDownHeap<OBJECT, KEY>::Remove(0);
	Remove(0);
}

template <class OBJECT, class KEY>
void dDownHeap<OBJECT,KEY>::Remove (dInt32 index)
{
	dHeapBase<OBJECT, KEY>::m_curCount--;
	dHeapBase<OBJECT, KEY>::m_pool[index] = dHeapBase<OBJECT, KEY>::m_pool[dHeapBase<OBJECT, KEY>::m_curCount];
	while (index && dHeapBase<OBJECT, KEY>::m_pool[(index - 1) >> 1].m_key < dHeapBase<OBJECT, KEY>::m_pool[index].m_key) 
	{
		dSwap(dHeapBase<OBJECT, KEY>::m_pool[(index - 1) >> 1], dHeapBase<OBJECT, KEY>::m_pool[index]);
		index = (index - 1) >> 1;
	}

	while ((2 * index + 1) < dHeapBase<OBJECT, KEY>::m_curCount) 
	{
		dInt32 i0 = 2 * index + 1;
		dInt32 i1 = 2 * index + 2;
		if (i1 < dHeapBase<OBJECT, KEY>::m_curCount) 
		{
			i0 = (dHeapBase<OBJECT, KEY>::m_pool[i0].m_key > dHeapBase<OBJECT, KEY>::m_pool[i1].m_key) ? i0 : i1;
			if (dHeapBase<OBJECT, KEY>::m_pool[i0].m_key <= dHeapBase<OBJECT, KEY>::m_pool[index].m_key) 
			{
				break;
			}
			dSwap(dHeapBase<OBJECT, KEY>::m_pool[i0], dHeapBase<OBJECT, KEY>::m_pool[index]);
			index = i0;
		} 
		else 
		{
			if (dHeapBase<OBJECT, KEY>::m_pool[i0].m_key > dHeapBase<OBJECT, KEY>::m_pool[index].m_key) 
			{
				dSwap(dHeapBase<OBJECT, KEY>::m_pool[i0], dHeapBase<OBJECT, KEY>::m_pool[index]);
			}
			index = i0;
		}
	}
	dAssert (SanityCheck());
}

template <class OBJECT, class KEY>
void dDownHeap<OBJECT,KEY>::Sort ()
{
	dInt32 count = dHeapBase<OBJECT,KEY>::m_curCount;
	for (dInt32 i = 1; i < count; i ++) 
	{
		KEY key (dHeapBase<OBJECT,KEY>::m_pool[0].m_key);
		OBJECT obj (dHeapBase<OBJECT,KEY>::m_pool[0].m_obj);

		Pop();

		dHeapBase<OBJECT,KEY>::m_pool[dHeapBase<OBJECT,KEY>::m_curCount].m_key = key;
		dHeapBase<OBJECT,KEY>::m_pool[dHeapBase<OBJECT,KEY>::m_curCount].m_obj = obj;
	}

	dHeapBase<OBJECT,KEY>::m_curCount = count;
	for (dInt32 i = 0; i < count / 2; i ++) 
	{
		KEY key (dHeapBase<OBJECT,KEY>::m_pool[i].m_key);
		OBJECT obj (dHeapBase<OBJECT,KEY>::m_pool[i].m_obj);

		dHeapBase<OBJECT,KEY>::m_pool[i].m_key = dHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_key;
		dHeapBase<OBJECT,KEY>::m_pool[i].m_obj = dHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_obj;

		dHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_key = key;
		dHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_obj = obj;
	}
	dAssert (SanityCheck());
}

template <class OBJECT, class KEY>
bool dDownHeap<OBJECT,KEY>::SanityCheck()
{
#ifdef DG_HEAP_DEBUG_CHECK
	for (dInt32 i = 0; i < m_curCount; i++) 
	{
		dInt32 i1 = 2 * i + 1;
		dInt32 i2 = 2 * i + 2;
		if ((i1 < m_curCount) && (dHeapBase<OBJECT, KEY>::m_pool[i].m_key < dHeapBase<OBJECT, KEY>::m_pool[i1].m_key)) 
		{
			return false;
		}
		if ((i2 < m_curCount) && (dHeapBase<OBJECT, KEY>::m_pool[i].m_key < dHeapBase<OBJECT, KEY>::m_pool[i2].m_key)) 
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
dUpHeap<OBJECT,KEY>::dUpHeap (dInt32 maxElements)
//	:dHeapBase<OBJECT, KEY> (maxElements, allocator)
	:dHeapBase<OBJECT, KEY>(maxElements)
{
}

template <class OBJECT, class KEY>
dUpHeap<OBJECT,KEY>::dUpHeap (const void * const buffer, dInt32 sizeInBytes)
	:dHeapBase<OBJECT, KEY> (buffer, sizeInBytes)
{
}

template <class OBJECT, class KEY>
bool dUpHeap<OBJECT,KEY>::SanityCheck()
{
#ifdef DG_HEAP_DEBUG_CHECK
	for (dInt32 i = 0; i < m_curCount; i ++) 
	{
		dInt32 i1 = 2 * i + 1; 
		dInt32 i2 = 2 * i + 2; 
		if ((i1 < m_curCount) && (dHeapBase<OBJECT,KEY>::m_pool[i].m_key > dHeapBase<OBJECT,KEY>::m_pool[i1].m_key)) 
		{
			return false;
		}
		if ((i2 < m_curCount) && (dHeapBase<OBJECT,KEY>::m_pool[i].m_key > dHeapBase<OBJECT,KEY>::m_pool[i2].m_key)) 
		{
			return false;
		}
	}
#endif
	return true;
}

template <class OBJECT, class KEY>
void dUpHeap<OBJECT,KEY>::Push (OBJECT &obj, KEY key)
{
	dHeapBase<OBJECT,KEY>::m_curCount ++;

	dInt32 i = dHeapBase<OBJECT,KEY>::m_curCount;
	for (dInt32 j = 0; i; i = j)
	{
		j = i >> 1;
		if (!j || (dHeapBase<OBJECT,KEY>::m_pool[j - 1].m_key < key)) 
		{
			break;
		}
		dHeapBase<OBJECT,KEY>::m_pool[i - 1] = dHeapBase<OBJECT,KEY>::m_pool[j - 1];
	}
	dAssert (i);
	dHeapBase<OBJECT,KEY>::m_pool[i - 1].m_key = key;
	dHeapBase<OBJECT,KEY>::m_pool[i - 1].m_obj = obj;
	dAssert (SanityCheck());
}

template <class OBJECT, class KEY>
void dUpHeap<OBJECT, KEY>::Pop()
{ 
	Remove(0); 
}

template <class OBJECT, class KEY>
void dUpHeap<OBJECT,KEY>::Sort ()
{
	dInt32 count = dHeapBase<OBJECT,KEY>::m_curCount;
	for (dInt32 i = 1; i < count; i ++) 
	{
		KEY key (dHeapBase<OBJECT,KEY>::m_pool[0].m_key);
		OBJECT obj (dHeapBase<OBJECT,KEY>::m_pool[0].m_obj);

		Pop();

		dHeapBase<OBJECT,KEY>::m_pool[dHeapBase<OBJECT,KEY>::m_curCount].m_key = key;
		dHeapBase<OBJECT,KEY>::m_pool[dHeapBase<OBJECT,KEY>::m_curCount].m_obj = obj;
	}

	dHeapBase<OBJECT,KEY>::m_curCount = count;
	for (dInt32 i = 0; i < count / 2; i ++) 
	{
		KEY key (dHeapBase<OBJECT,KEY>::m_pool[i].m_key);
		OBJECT obj (dHeapBase<OBJECT,KEY>::m_pool[i].m_obj);

		dHeapBase<OBJECT,KEY>::m_pool[i].m_key = dHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_key;
		dHeapBase<OBJECT,KEY>::m_pool[i].m_obj = dHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_obj;

		dHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_key = key;
		dHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_obj = obj;
	}
	dAssert (SanityCheck());
}

template <class OBJECT, class KEY>
void dUpHeap<OBJECT,KEY>::Remove (dInt32 index)
{
	dHeapBase<OBJECT, KEY>::m_curCount--;
	dHeapBase<OBJECT, KEY>::m_pool[index] = dHeapBase<OBJECT, KEY>::m_pool[dHeapBase<OBJECT, KEY>::m_curCount];
	while (index && dHeapBase<OBJECT, KEY>::m_pool[(index - 1) >> 1].m_key > dHeapBase<OBJECT, KEY>::m_pool[index].m_key) 
	{
		dSwap(dHeapBase<OBJECT, KEY>::m_pool[(index - 1) >> 1], dHeapBase<OBJECT, KEY>::m_pool[index]);
		index = (index - 1) >> 1;
	}

	while ((2 * index + 1) < dHeapBase<OBJECT, KEY>::m_curCount) 
	{
		dInt32 i0 = 2 * index + 1;
		dInt32 i1 = 2 * index + 2;
		if (i1 < dHeapBase<OBJECT, KEY>::m_curCount) 
		{
			i0 = (dHeapBase<OBJECT, KEY>::m_pool[i0].m_key < dHeapBase<OBJECT, KEY>::m_pool[i1].m_key) ? i0 : i1;
			if (dHeapBase<OBJECT, KEY>::m_pool[i0].m_key >= dHeapBase<OBJECT, KEY>::m_pool[index].m_key) 
			{
				break;
			}
			dSwap(dHeapBase<OBJECT, KEY>::m_pool[i0], dHeapBase<OBJECT, KEY>::m_pool[index]);
			index = i0;
		} 
		else 
		{
			if (dHeapBase<OBJECT, KEY>::m_pool[i0].m_key < dHeapBase<OBJECT, KEY>::m_pool[index].m_key) 
			{
				dSwap(dHeapBase<OBJECT, KEY>::m_pool[i0], dHeapBase<OBJECT, KEY>::m_pool[index]);
			}
			index = i0;
		}
	}
	dAssert (SanityCheck());
}

#endif