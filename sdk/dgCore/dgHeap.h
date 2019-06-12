/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
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
#ifndef __dgHeapBase__
#define __dgHeapBase__

#include "dgStdafx.h"
#include "dgMemory.h"

//#define DG_HEAP_DEBUG_CHECK


template <class OBJECT, class KEY>
class dgHeapBase
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

	dgHeapBase (dgInt32 maxElements, dgMemoryAllocator* const allocator);
	dgHeapBase (const void * const buffer, dgInt32 sizeInBytes);
	~dgHeapBase ();
	
	public:
	DG_CLASS_ALLOCATOR(allocator)

	void Flush (); 
	KEY MaxValue() const; 
	KEY Value(dgInt32 i = 0) const;
	dgInt32 GetCount() const;
	dgInt32 GetMaxCount() const;
	const OBJECT& operator[] (dgInt32 i) const;
	dgInt32 Find (OBJECT &obj);
	dgInt32 Find (KEY key);

	dgInt32 m_curCount;
	dgInt32 m_maxCount;
	dgMemoryAllocator* m_allocator;
	RECORD *m_pool;
};

template <class OBJECT, class KEY>
class dgDownHeap: public dgHeapBase<OBJECT, KEY>
{
	public:
	dgDownHeap (dgInt32 maxElements, dgMemoryAllocator* const allocator);
	dgDownHeap (const void * const buffer, dgInt32 sizeInBytes);

	void Pop () {Remove (0);}
	void Push (OBJECT &obj, KEY key);
	void Sort ();
	void Remove (dgInt32 Index);
	bool SanityCheck();
};

template <class OBJECT, class KEY>
class dgUpHeap: public dgHeapBase<OBJECT, KEY>
{
	public:
	dgUpHeap (dgInt32 maxElements, dgMemoryAllocator* const allocator);
	dgUpHeap (const void * const buffer, dgInt32 sizeInBytes);

	void Pop () {Remove (0);}
	void Push (OBJECT &obj, KEY key);
	void Sort ();
	void Remove (dgInt32 Index);
	bool SanityCheck();
};

template <class OBJECT, class KEY>
dgHeapBase<OBJECT,KEY>::dgHeapBase (dgInt32 maxElements, dgMemoryAllocator* const allocator)
{
	m_allocator = allocator;
	m_pool = (RECORD *)m_allocator->Malloc (maxElements * sizeof (RECORD));
	m_maxCount = maxElements;
	Flush();
}

template <class OBJECT, class KEY>
dgHeapBase<OBJECT,KEY>::dgHeapBase (const void * const buffer, dgInt32 sizeInBytes)
{
	m_allocator = NULL;
	m_pool = (RECORD *) buffer;
	m_maxCount = dgInt32 (sizeInBytes / sizeof (RECORD));
	Flush();
}

template <class OBJECT, class KEY>
dgHeapBase<OBJECT,KEY>::~dgHeapBase ()
{   
	if (m_allocator) {
		m_allocator->Free (m_pool);
	}
}


template <class OBJECT, class KEY>
KEY dgHeapBase<OBJECT,KEY>::Value(dgInt32 i) const
{
	return m_pool[i].m_key;
}


template <class OBJECT, class KEY>
dgInt32 dgHeapBase<OBJECT,KEY>::GetCount() const
{ 
	return m_curCount;
}

template <class OBJECT, class KEY>
void dgHeapBase<OBJECT,KEY>::Flush () 
{
	m_curCount = 0;

	#ifdef _DEBUG
//	dgHeapBase<OBJECT,KEY>::m_pool[dgHeapBase<OBJECT,KEY>::m_curCount].m_key = KEY (0);
	#endif
}

template <class OBJECT, class KEY>
KEY dgHeapBase<OBJECT,KEY>::MaxValue() const 
{
	return m_pool[0].m_key;
}

template <class OBJECT, class KEY>
dgInt32 dgHeapBase<OBJECT,KEY>::GetMaxCount() const
{ 
	return m_maxCount;
}


template <class OBJECT, class KEY>
dgInt32 dgHeapBase<OBJECT,KEY>::Find (OBJECT &obj)
{
	// For now let perform a linear search
	// this is efficient if the size of the heap is small
	// ex: m_curCount < 32
	// this will be change to a binary search in the heap should the 
	// the size of the heap get larger than 32
	//	dgAssert (m_curCount <= 32);
	for (dgInt32 i = 0; i < m_curCount; i ++) {
		if (m_pool[i].obj == obj) {
			return i;
		}
	}
	return - 1;
}


template <class OBJECT, class KEY>
dgInt32 dgHeapBase<OBJECT,KEY>::Find (KEY key)
{
	// ex: m_curCount < 32
	// this will be change to a binary search in the heap shoud the 
	// the size of the heap get larger than 32
	dgAssert (m_curCount <= 32);
	for (dgInt32 i = 0; i < m_curCount; i ++)	{
		if (m_pool[i].m_key == key) {
			return i;
		}
	}
	return - 1;
}


template <class OBJECT, class KEY>
const OBJECT& dgHeapBase<OBJECT,KEY>::operator[] (dgInt32 i) const
{ 
	dgAssert (i<= m_curCount);
	return m_pool[i].m_obj;
}


// **************************************************************************
//
// down Heap
//
// **************************************************************************
template <class OBJECT, class KEY>
dgDownHeap<OBJECT,KEY>::dgDownHeap (dgInt32 maxElements, dgMemoryAllocator* const allocator)
	:dgHeapBase<OBJECT, KEY> (maxElements, allocator)
{
}

template <class OBJECT, class KEY>
dgDownHeap<OBJECT,KEY>::dgDownHeap (const void * const buffer, dgInt32 sizeInBytes)
	:dgHeapBase<OBJECT, KEY> (buffer, sizeInBytes)
{
}


template <class OBJECT, class KEY>
void dgDownHeap<OBJECT,KEY>::Push (OBJECT &obj, KEY key)
{
	dgHeapBase<OBJECT,KEY>::m_curCount ++;

	dgInt32 j;
	dgInt32 i = dgHeapBase<OBJECT,KEY>::m_curCount;
	for (; i; i = j) {
		j = i >> 1;
		if (!j || (dgHeapBase<OBJECT,KEY>::m_pool[j - 1].m_key > key)) {
			break;
		}
		dgHeapBase<OBJECT,KEY>::m_pool[i - 1] = dgHeapBase<OBJECT,KEY>::m_pool[j - 1];
	}
	dgAssert (i);
	dgHeapBase<OBJECT,KEY>::m_pool[i - 1].m_key = key;
	dgHeapBase<OBJECT,KEY>::m_pool[i - 1].m_obj = obj;

	dgAssert (SanityCheck());
}

template <class OBJECT, class KEY>
void dgDownHeap<OBJECT,KEY>::Remove (dgInt32 index)
{
	dgHeapBase<OBJECT, KEY>::m_curCount--;
	dgHeapBase<OBJECT, KEY>::m_pool[index] = dgHeapBase<OBJECT, KEY>::m_pool[dgHeapBase<OBJECT, KEY>::m_curCount];
	while (index && dgHeapBase<OBJECT, KEY>::m_pool[(index - 1) >> 1].m_key < dgHeapBase<OBJECT, KEY>::m_pool[index].m_key) {
		dgSwap(dgHeapBase<OBJECT, KEY>::m_pool[(index - 1) >> 1], dgHeapBase<OBJECT, KEY>::m_pool[index]);
		index = (index - 1) >> 1;
	}

	while ((2 * index + 1) < dgHeapBase<OBJECT, KEY>::m_curCount) {
		dgInt32 i0 = 2 * index + 1;
		dgInt32 i1 = 2 * index + 2;
		if (i1 < dgHeapBase<OBJECT, KEY>::m_curCount) {
			i0 = (dgHeapBase<OBJECT, KEY>::m_pool[i0].m_key > dgHeapBase<OBJECT, KEY>::m_pool[i1].m_key) ? i0 : i1;
			if (dgHeapBase<OBJECT, KEY>::m_pool[i0].m_key <= dgHeapBase<OBJECT, KEY>::m_pool[index].m_key) {
				break;
			}
			dgSwap(dgHeapBase<OBJECT, KEY>::m_pool[i0], dgHeapBase<OBJECT, KEY>::m_pool[index]);
			index = i0;
		} else {
			if (dgHeapBase<OBJECT, KEY>::m_pool[i0].m_key > dgHeapBase<OBJECT, KEY>::m_pool[index].m_key) {
				dgSwap(dgHeapBase<OBJECT, KEY>::m_pool[i0], dgHeapBase<OBJECT, KEY>::m_pool[index]);
			}
			index = i0;
		}
	}
	dgAssert (SanityCheck());
}

template <class OBJECT, class KEY>
void dgDownHeap<OBJECT,KEY>::Sort ()
{
	dgInt32 count = dgHeapBase<OBJECT,KEY>::m_curCount;
	for (dgInt32 i = 1; i < count; i ++) {
		KEY key (dgHeapBase<OBJECT,KEY>::m_pool[0].m_key);
		OBJECT obj (dgHeapBase<OBJECT,KEY>::m_pool[0].m_obj);

		Pop();

		dgHeapBase<OBJECT,KEY>::m_pool[dgHeapBase<OBJECT,KEY>::m_curCount].m_key = key;
		dgHeapBase<OBJECT,KEY>::m_pool[dgHeapBase<OBJECT,KEY>::m_curCount].m_obj = obj;
	}

	dgHeapBase<OBJECT,KEY>::m_curCount = count;
	for (dgInt32 i = 0; i < count / 2; i ++) {
		KEY key (dgHeapBase<OBJECT,KEY>::m_pool[i].m_key);
		OBJECT obj (dgHeapBase<OBJECT,KEY>::m_pool[i].m_obj);

		dgHeapBase<OBJECT,KEY>::m_pool[i].m_key = dgHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_key;
		dgHeapBase<OBJECT,KEY>::m_pool[i].m_obj = dgHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_obj;

		dgHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_key = key;
		dgHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_obj = obj;
	}
	dgAssert (SanityCheck());
}

template <class OBJECT, class KEY>
bool dgDownHeap<OBJECT,KEY>::SanityCheck()
{
#ifdef DG_HEAP_DEBUG_CHECK
	for (dgInt32 i = 0; i < m_curCount; i++) {
		dgInt32 i1 = 2 * i + 1;
		dgInt32 i2 = 2 * i + 2;
		if ((i1 < m_curCount) && (dgHeapBase<OBJECT, KEY>::m_pool[i].m_key < dgHeapBase<OBJECT, KEY>::m_pool[i1].m_key)) {
			return false;
		}
		if ((i2 < m_curCount) && (dgHeapBase<OBJECT, KEY>::m_pool[i].m_key < dgHeapBase<OBJECT, KEY>::m_pool[i2].m_key)) {
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
dgUpHeap<OBJECT,KEY>::dgUpHeap (dgInt32 maxElements, dgMemoryAllocator* const allocator)
	:dgHeapBase<OBJECT, KEY> (maxElements, allocator)
{
}

template <class OBJECT, class KEY>
dgUpHeap<OBJECT,KEY>::dgUpHeap (const void * const buffer, dgInt32 sizeInBytes)
	:dgHeapBase<OBJECT, KEY> (buffer, sizeInBytes)
{
}

template <class OBJECT, class KEY>
bool dgUpHeap<OBJECT,KEY>::SanityCheck()
{
#ifdef DG_HEAP_DEBUG_CHECK
	for (dgInt32 i = 0; i < m_curCount; i ++) {
		dgInt32 i1 = 2 * i + 1; 
		dgInt32 i2 = 2 * i + 2; 
		if ((i1 < m_curCount) && (dgHeapBase<OBJECT,KEY>::m_pool[i].m_key > dgHeapBase<OBJECT,KEY>::m_pool[i1].m_key)) {
			return false;
		}
		if ((i2 < m_curCount) && (dgHeapBase<OBJECT,KEY>::m_pool[i].m_key > dgHeapBase<OBJECT,KEY>::m_pool[i2].m_key)) {
			return false;
		}
	}
#endif
	return true;
}

template <class OBJECT, class KEY>
void dgUpHeap<OBJECT,KEY>::Push (OBJECT &obj, KEY key)
{
	dgHeapBase<OBJECT,KEY>::m_curCount ++;

	dgInt32 j;
	dgInt32 i = dgHeapBase<OBJECT,KEY>::m_curCount;
	for (; i; i = j) {
		j = i >> 1;
		if (!j || (dgHeapBase<OBJECT,KEY>::m_pool[j - 1].m_key < key)) {
			break;
		}
		dgHeapBase<OBJECT,KEY>::m_pool[i - 1] = dgHeapBase<OBJECT,KEY>::m_pool[j - 1];
	}
	dgAssert (i);
	dgHeapBase<OBJECT,KEY>::m_pool[i - 1].m_key = key;
	dgHeapBase<OBJECT,KEY>::m_pool[i - 1].m_obj = obj;
	dgAssert (SanityCheck());
}

template <class OBJECT, class KEY>
void dgUpHeap<OBJECT,KEY>::Sort ()
{
	dgInt32 count = dgHeapBase<OBJECT,KEY>::m_curCount;
	for (dgInt32 i = 1; i < count; i ++) {
		KEY key (dgHeapBase<OBJECT,KEY>::m_pool[0].m_key);
		OBJECT obj (dgHeapBase<OBJECT,KEY>::m_pool[0].m_obj);

		Pop();

		dgHeapBase<OBJECT,KEY>::m_pool[dgHeapBase<OBJECT,KEY>::m_curCount].m_key = key;
		dgHeapBase<OBJECT,KEY>::m_pool[dgHeapBase<OBJECT,KEY>::m_curCount].m_obj = obj;
	}

	dgHeapBase<OBJECT,KEY>::m_curCount = count;
	for (dgInt32 i = 0; i < count / 2; i ++) {
		KEY key (dgHeapBase<OBJECT,KEY>::m_pool[i].m_key);
		OBJECT obj (dgHeapBase<OBJECT,KEY>::m_pool[i].m_obj);

		dgHeapBase<OBJECT,KEY>::m_pool[i].m_key = dgHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_key;
		dgHeapBase<OBJECT,KEY>::m_pool[i].m_obj = dgHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_obj;

		dgHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_key = key;
		dgHeapBase<OBJECT,KEY>::m_pool[count - i - 1].m_obj = obj;
	}
	dgAssert (SanityCheck());
}

template <class OBJECT, class KEY>
void dgUpHeap<OBJECT,KEY>::Remove (dgInt32 index)
{
	dgHeapBase<OBJECT, KEY>::m_curCount--;
	dgHeapBase<OBJECT, KEY>::m_pool[index] = dgHeapBase<OBJECT, KEY>::m_pool[dgHeapBase<OBJECT, KEY>::m_curCount];
	while (index && dgHeapBase<OBJECT, KEY>::m_pool[(index - 1) >> 1].m_key > dgHeapBase<OBJECT, KEY>::m_pool[index].m_key) {
		dgSwap(dgHeapBase<OBJECT, KEY>::m_pool[(index - 1) >> 1], dgHeapBase<OBJECT, KEY>::m_pool[index]);
		index = (index - 1) >> 1;
	}

	while ((2 * index + 1) < dgHeapBase<OBJECT, KEY>::m_curCount) {
		dgInt32 i0 = 2 * index + 1;
		dgInt32 i1 = 2 * index + 2;
		if (i1 < dgHeapBase<OBJECT, KEY>::m_curCount) {
			i0 = (dgHeapBase<OBJECT, KEY>::m_pool[i0].m_key < dgHeapBase<OBJECT, KEY>::m_pool[i1].m_key) ? i0 : i1;
			if (dgHeapBase<OBJECT, KEY>::m_pool[i0].m_key >= dgHeapBase<OBJECT, KEY>::m_pool[index].m_key) {
				break;
			}
			dgSwap(dgHeapBase<OBJECT, KEY>::m_pool[i0], dgHeapBase<OBJECT, KEY>::m_pool[index]);
			index = i0;
		} else {
			if (dgHeapBase<OBJECT, KEY>::m_pool[i0].m_key < dgHeapBase<OBJECT, KEY>::m_pool[index].m_key) {
				dgSwap(dgHeapBase<OBJECT, KEY>::m_pool[i0], dgHeapBase<OBJECT, KEY>::m_pool[index]);
			}
			index = i0;
		}
	}
	dgAssert (SanityCheck());
}


#endif