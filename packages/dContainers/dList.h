/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


#ifndef __dList__
#define __dList__

#include "dContainersStdAfx.h"

#define D_MAX_ENTRIES_IN_FREELIST	32

template<class T>
class dListAllocator
{
	public:
	dListAllocator();
	~dListAllocator();
	void* Alloc();
	void Free(void* const ptr);
	void Flush ();
	bool IsAlive() const;

	private:
	class dFreeListNode
	{
		public:
		int m_count;
		dFreeListNode* m_next;
	};
	
	void Prefetch ();
	dFreeListNode* m_freeListNode;
	bool m_alive;
};


// this is a small double link list container similar to STL, 
// it is used to provide low level support for this demos.
// it implements an iterators, and all the basics operation on list found on the STL class

template<class T, class Allocator = dListAllocator<T> >
class dList 
{
	public:
	class dListNode
	{
		dListNode (dListNode* const prev, dListNode* const next) 
			:m_info () 
		{
			m_prev = prev;
			m_next = next;
			if (m_prev) {
				m_prev->m_next = this;
			}
			if (m_next) {
				m_next->m_prev = this;
			}
		}

		dListNode (const T &info, dListNode* prev, dListNode* next) 
			:m_info (info) 
		{
			m_prev = prev;
			m_next = next;
			if (m_prev) {
				m_prev->m_next = this;
			}
			if (m_next) {
				m_next->m_prev = this;
			}
		}

		virtual ~dListNode()
		{
		}

		void Unlink ()
		{
			if (m_prev) {
				m_prev->m_next = m_next;
			}

			if (m_next) {
				m_next->m_prev = m_prev;
			}
			m_prev = NULL;
			m_next = NULL;
		}

		void Remove()
		{
			Unlink();
			this->~dListNode();
		}

		void AddLast(dListNode* const node) 
		{
			m_next = node;
			node->m_prev = this;
		}

		void AddFirst(dListNode* const node) 
		{
			m_prev = node;
			node->m_next = this;
		}

		public:
		T& GetInfo()
		{
			return m_info;
		}

		const T& GetInfo() const
		{
			return m_info;
		}


		dListNode* GetNext() const
		{
			return m_next;
		}

		dListNode* GetPrev() const
		{
			return m_prev;
		}

		private:
		T m_info;
		dListNode* m_next;
		dListNode* m_prev;
		friend class dList<T, Allocator>;
	};

	class Iterator
	{
		public:
		Iterator (const dList<T> &me)
		{
			m_ptr = NULL;
			m_list = (dList *)&me;
		}

		~Iterator ()
		{
		}

		operator int() const
		{
			return m_ptr != NULL;
		}

		bool operator== (const Iterator &target) const
		{
			return (m_ptr == target.m_ptr) && (m_list == target.m_list);
		}

		void Begin()
		{
			m_ptr = m_list->GetFirst();
		}

		void End()
		{
			m_ptr = m_list->GetLast();
		}

		void Set (dListNode* const node)
		{
			m_ptr = node;
		}

		void operator++ ()
		{
			m_ptr = m_ptr->m_next();
		}

		void operator++ (int)
		{
			m_ptr = m_ptr->GetNext();
		}

		void operator-- () 
		{
			m_ptr = m_ptr->GetPrev();
		}

		void operator-- (int) 
		{
			m_ptr = m_ptr->GetPrev();
		}

		T &operator* () const
		{
			return m_ptr->GetInfo();
		}

		dListNode* GetNode() const
		{
			return m_ptr;
		}

		private:
		dList *m_list;
		dListNode* m_ptr;
	};

	// ***********************************************************
	// member functions
	// ***********************************************************
	public:
	dList ();
	virtual ~dList ();

	void* operator new (size_t size);
	void operator delete (void* ptr);

	operator int() const;
	int GetCount() const;
	dListNode* GetLast() const;
	dListNode* GetFirst() const;
	dListNode* Append ();
	dListNode* Append (const T &element);
	dListNode* Addtop ();
	dListNode* Addtop (const T &element);
	dListNode* AppendAfter (dListNode* const node);

	void RotateToEnd (dListNode* const node);
	void RotateToBegin (dListNode* const node);
	void InsertAfter (dListNode* const root, dListNode* const node);

	dListNode* Find (const T &element) const;
	dListNode* GetNodeFromInfo (T &m_info) const;
	void Remove (dListNode* const node);
	void Remove (const T &element);
	void RemoveAll ();
	

	// ***********************************************************
	// member variables
	// ***********************************************************
	private:
/*
	bool Sanity() const
	{
		int count = 0;
		for (dListNode* ptr = GetFirst(); ptr; ptr = ptr->GetNext())
			count ++;
		return count == m_count;
	}
*/
	bool Sanity() const {return true;}
	Allocator& GetAllocator()
	{
		static Allocator allocator;
		return allocator;
	}

	int m_count;
	dListNode* m_first;
	dListNode* m_last;
	friend class dListNode;
};


template<class T, class Allocator>
dList<T, Allocator>::dList ()
{
	m_count = 0;
	m_first = NULL;
	m_last = NULL;
}


template<class T, class Allocator>
dList<T, Allocator>::~dList () 
{
	RemoveAll ();
	if (!GetAllocator().IsAlive()) {
		GetAllocator().Flush();
	}
}


template<class T, class Allocator>
void* dList<T, Allocator>::operator new (size_t size)
{
	return new char[size];
}

template<class T, class Allocator>
void dList<T, Allocator>::operator delete (void *ptr)
{
	delete[] (char*)ptr;
}


template<class T, class Allocator>
int dList<T, Allocator>::GetCount() const
{
	return m_count;
}

template<class T, class Allocator>
dList<T, Allocator>::operator int() const
{
	return m_first != NULL;
}

template<class T, class Allocator>
typename dList<T, Allocator>::dListNode* dList<T, Allocator>::GetFirst() const
{
	return m_first;
}

template<class T, class Allocator>
typename dList<T, Allocator>::dListNode* dList<T, Allocator>::GetLast() const
{
	return m_last;
}

template<class T, class Allocator>
typename dList<T, Allocator>::dListNode* dList<T, Allocator>::Append ()
{
	m_count	++;
	if (m_first == NULL) {
		m_first = new (GetAllocator().Alloc()) dListNode(NULL, NULL);
		m_last = m_first;
	} else {
		m_last = new (GetAllocator().Alloc()) dListNode(m_last, NULL);
	}
	dAssert (Sanity());
	return m_last;
}

template<class T, class Allocator>
typename dList<T, Allocator>::dListNode* dList<T, Allocator>::AppendAfter (dListNode* const node)
{
	dListNode* const ptr = Append ();
	InsertAfter (node, ptr);
	return ptr;
}

template<class T, class Allocator>
typename dList<T, Allocator>::dListNode* dList<T, Allocator>::Append (const T &element)
{
	m_count	++;
	if (m_first == NULL) {
		m_first = new (GetAllocator().Alloc()) dListNode(element, NULL, NULL);
		m_last = m_first;
	} else {
		m_last = new (GetAllocator().Alloc()) dListNode(element, m_last, NULL);
	}
	dAssert (Sanity());
	return m_last;
}

template<class T, class Allocator>
typename dList<T, Allocator>::dListNode* dList<T, Allocator>::Addtop ()
{
	m_count	++;
	if (m_last == NULL) {
		m_last = new (GetAllocator().Alloc()) dListNode(NULL, NULL);
		m_first = m_last;
	} else {
		m_first = new (GetAllocator().Alloc()) dListNode(NULL, m_first);
	}
	dAssert (Sanity());
	return m_first;
}


template<class T, class Allocator>
typename dList<T, Allocator>::dListNode* dList<T, Allocator>::Addtop (const T &element)
{
	m_count	++;
	if (m_last == NULL) {
		m_last = new (GetAllocator().Alloc()) dListNode(element, NULL, NULL);
		m_first = m_last;
	} else {
		m_first = new (GetAllocator().Alloc()) dListNode(element, NULL, m_first);
	}
	dAssert (Sanity());
	return m_first;
}

template<class T, class Allocator>
void dList<T, Allocator>::InsertAfter (dListNode* const root, dListNode* const node)
{
	dAssert (root != node);
	if (node == m_last) {
		m_last = m_last->GetPrev(); 
	}
	node->Unlink();

	node->m_prev = root;
	node->m_next = root->m_next;
	if (root->m_next) {
		root->m_next->m_prev = node;
	} 
	root->m_next = node;

	if (root == m_last) {
		m_last = node;
	}

	dAssert (Sanity());
}

template<class T, class Allocator>
void dList<T, Allocator>::RotateToEnd (dListNode* const node)
{
	if (node != m_last) {
		if (m_last != m_first) {
			if (node == m_first) {
				m_first = m_first->GetNext();
			}
			node->Unlink();
			m_last->AddLast(node);
			m_last = node; 
		}
	}
}

template<class T, class Allocator>
void dList<T, Allocator>::RotateToBegin (dListNode* const node)
{
	if (node != m_first) {
		if (m_last != m_first) {
			if (node == m_last) {
				m_last = m_last->GetPrev();
			}
			node->Unlink();
			m_first->AddFirst(node);
			m_first = node; 
		}
	}
}


template<class T, class Allocator>
typename dList<T, Allocator>::dListNode* dList<T, Allocator>::Find (const T &element) const
{
	dListNode* node;
	for (node = m_first; node; node = node->GetNext()) {
		if (element	== node->m_info) {
			break;
		}
	}
	return node;
}


template<class T, class Allocator>
typename dList<T, Allocator>::dListNode* dList<T, Allocator>::GetNodeFromInfo (T &info) const
{
	dListNode* const node = (dListNode*) &info;
	long long offset = ((char*) &node->m_info) - ((char*)node);
	dListNode* const retnode = (dListNode*) (((char *) node) - offset);
	dAssert (&retnode->GetInfo () == &info);
	return retnode;
}


template<class T, class Allocator> 
void dList<T, Allocator>::Remove (const T &element)
{
	dListNode* const node = Find (element);
	if (node) {
		Remove (node);
	}
}

template<class T, class Allocator>
void dList<T, Allocator>::Remove (dListNode* const node)
{
	m_count --;
	if (node == m_first) {
		m_first = m_first->GetNext();
	}
	if (node == m_last) {
		m_last = m_last->GetPrev();
	}
	node->Remove();
	GetAllocator().Free (node);
	dAssert (Sanity());
}

template<class T, class Allocator>
void dList<T, Allocator>::RemoveAll ()
{
	while (m_first) {
		Remove(m_first);
	}
	dAssert (!m_count);
}


template<class T>
dListAllocator<T>::dListAllocator()
	:m_freeListNode(NULL)
	,m_alive(true)
{
	Prefetch ();
}

template<class T>
dListAllocator<T>::~dListAllocator()
{
	Flush();
	m_alive = false;
}

template<class T>
bool dListAllocator<T>::IsAlive() const
{
	return m_alive;
}

template<class T>
void dListAllocator<T>::Prefetch ()
{
	int sizeInBytes = sizeof (typename dList<T, dListAllocator<T> >::dListNode);
	for (int i = 0; i < D_MAX_ENTRIES_IN_FREELIST; i ++) {
		dFreeListNode* const data = (dFreeListNode*) new char[sizeInBytes];
		data->m_count = i + 1; 
		data->m_next = m_freeListNode; 
		m_freeListNode = data;
	}
}

template<class T>
void dListAllocator<T>::Flush ()
{
	for (int i = 0; m_freeListNode && (i < D_MAX_ENTRIES_IN_FREELIST); i ++) {
		dFreeListNode* const ptr = m_freeListNode;
		m_freeListNode = m_freeListNode->m_next;
		delete[] (char*) ptr;
	}
}

template<class T>
void* dListAllocator<T>::Alloc() 
{
	if (!m_freeListNode) {
		Prefetch ();
	}
	dFreeListNode* const data = m_freeListNode;
	m_freeListNode = m_freeListNode->m_next;
	return data;
}

template<class T>
void dListAllocator<T>::Free(void* const ptr) 
{
	dFreeListNode* const data = (dFreeListNode*) ptr;
	data->m_count = m_freeListNode ? m_freeListNode->m_count + 1 : 1;
	data->m_next = m_freeListNode;
	m_freeListNode = data;
	if (data->m_count >= 2 * D_MAX_ENTRIES_IN_FREELIST) {
		Flush();
	}
}

#endif


