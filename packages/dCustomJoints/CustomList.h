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


#ifndef _CUSTOM_LIST_H_
#define _CUSTOM_LIST_H_
#include "CustomAlloc.h"

// this is a small double link list container similar to STL, 
// it is used to provide low level support for this demos.
// it implements an iterators, and all the basics operation on list found on the STL class

template<class T>
class CustomList: public CustomAlloc   
{
	public:
	class CustomListNode: public CustomAlloc  
	{
		CustomListNode (CustomListNode* const prev, CustomListNode* const next) 
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

		CustomListNode (const T &info, CustomListNode* prev, CustomListNode* next) 
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

		virtual ~CustomListNode()
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
			//this->~CustomListNode();
			delete this;
		}

		void AddLast(CustomListNode* const node) 
		{
			m_next = node;
			node->m_prev = this;
		}

		void AddFirst(CustomListNode* const node) 
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


		CustomListNode* GetNext() const
		{
			return m_next;
		}

		CustomListNode* GetPrev() const
		{
			return m_prev;
		}

		private:
		T m_info;
		CustomListNode* m_next;
		CustomListNode* m_prev;
		friend class CustomList<T>;
	};
	
	// ***********************************************************
	// member functions
	// ***********************************************************
	public:
	CustomList ();
	virtual ~CustomList ();

	operator int() const;
	int GetCount() const;
	CustomListNode* GetLast() const;
	CustomListNode* GetFirst() const;
	CustomListNode* Append ();
	CustomListNode* Append (const T &element);
	CustomListNode* Addtop ();
	CustomListNode* Addtop (const T &element);
	CustomListNode* AppendAfter (CustomListNode* const node);

	void RotateToEnd (CustomListNode* const node);
	void RotateToBegin (CustomListNode* const node);
	void InsertAfter (CustomListNode* const root, CustomListNode* const node);

	CustomListNode* Find (const T &element) const;
	CustomListNode* GetNodeFromInfo (T &m_info) const;
	void Remove (CustomListNode* const node);
	void Remove (const T &element);
	void RemoveAll ();
	

	// ***********************************************************
	// member variables
	// ***********************************************************
	private:
	int m_count;
	CustomListNode* m_first;
	CustomListNode* m_last;
	friend class CustomListNode;
};


template<class T>
CustomList<T>::CustomList ()
{
	m_count = 0;
	m_first = NULL;
	m_last = NULL;
}


template<class T>
CustomList<T>::~CustomList () 
{
	RemoveAll ();
}



template<class T>
int CustomList<T>::GetCount() const
{
	return m_count;
}

template<class T>
CustomList<T>::operator int() const
{
	return m_first != NULL;
}

template<class T>
typename CustomList<T>::CustomListNode* CustomList<T>::GetFirst() const
{
	return m_first;
}

template<class T>
typename CustomList<T>::CustomListNode* CustomList<T>::GetLast() const
{
	return m_last;
}

template<class T>
typename CustomList<T>::CustomListNode* CustomList<T>::Append ()
{
	m_count	++;
	if (m_first == NULL) {
		m_first = new CustomListNode(NULL, NULL);
		m_last = m_first;
	} else {
		m_last = new CustomListNode(m_last, NULL);
	}
	return m_last;
}

template<class T>
typename CustomList<T>::CustomListNode* CustomList<T>::AppendAfter (CustomListNode* const node)
{
	CustomListNode* const ptr = Append ();
	InsertAfter (node, ptr);
	return ptr;
}

template<class T>
typename CustomList<T>::CustomListNode* CustomList<T>::Append (const T &element)
{
	m_count	++;
	if (m_first == NULL) {
		m_first = new CustomListNode(element, NULL, NULL);
		m_last = m_first;
	} else {
		m_last = new CustomListNode(element, m_last, NULL);
	}
	return m_last;
}

template<class T>
typename CustomList<T>::CustomListNode* CustomList<T>::Addtop ()
{
	m_count	++;
	if (m_last == NULL) {
		m_last = new CustomListNode(NULL, NULL);
		m_first = m_last;
	} else {
		m_first = new CustomListNode(NULL, m_first);
	}
	return m_first;
}


template<class T>
typename CustomList<T>::CustomListNode* CustomList<T>::Addtop (const T &element)
{
	m_count	++;
	if (m_last == NULL) {
		m_last = new CustomListNode(element, NULL, NULL);
		m_first = m_last;
	} else {
		m_first = new CustomListNode(element, NULL, m_first);
	}
	return m_first;
}

template<class T>
void CustomList<T>::InsertAfter (CustomListNode* const root, CustomListNode* const node)
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
}

template<class T>
void CustomList<T>::RotateToEnd (CustomListNode* const node)
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

template<class T>
void CustomList<T>::RotateToBegin (CustomListNode* const node)
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


template<class T>
typename CustomList<T>::CustomListNode* CustomList<T>::Find (const T &element) const
{
	CustomListNode* node;
	for (node = m_first; node; node = node->GetNext()) {
		if (element	== node->m_info) {
			break;
		}
	}
	return node;
}


template<class T>
typename CustomList<T>::CustomListNode* CustomList<T>::GetNodeFromInfo (T &info) const
{
	CustomListNode* const node = (CustomListNode*) &info;
	long long offset = ((char*) &node->m_info) - ((char*)node);
	CustomListNode* const retnode = (CustomListNode*) (((char *) node) - offset);
	dAssert (&retnode->GetInfo () == &info);
	return retnode;
}


template<class T> 
void CustomList<T>::Remove (const T &element)
{
	CustomListNode* const node = Find (element);
	if (node) {
		Remove (node);
	}
}

template<class T>
void CustomList<T>::Remove (CustomListNode* const node)
{
	m_count --;
	if (node == m_first) {
		m_first = m_first->GetNext();
	}
	if (node == m_last) {
		m_last = m_last->GetPrev();
	}
	node->Remove();
}

template<class T>
void CustomList<T>::RemoveAll ()
{
	while (m_first) {
		Remove(m_first);
	}
	dAssert (!m_count);
}


#endif


