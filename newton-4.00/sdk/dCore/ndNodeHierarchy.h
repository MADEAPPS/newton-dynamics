/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __ND_NODE_HIERARCHY_H__
#define __ND_NODE_HIERARCHY_H__

#include "ndCoreStdafx.h"
#include "ndContainersAlloc.h"

template<class T>
class ndNodeHierarchy: public ndContainersFreeListAlloc<T>
{
	public:
	ndNodeHierarchy ();
	ndNodeHierarchy<T>* CreateClone() const;
	
	void Attach(ndNodeHierarchy<T>* const parent);
	void Detach ();

	T* GetParent() const;
	T* GetChild () const;
	T* GetLastChild() const;

	T* GetPrev____() const;
	
	T* GetSibling () const;
	T* GetRoot () const;
	T* GetFirst() const;
	T* GetLast() const;
	T* GetNext() const;
	T* GetPrev() const;

	T* IteratorFirst() const;
	T* IteratorNext() const;

	protected:
	ndNodeHierarchy (const ndNodeHierarchy<T>& clone);
	virtual ~ndNodeHierarchy ();

	ndNodeHierarchy<T>* m_next;
	ndNodeHierarchy<T>* m_prev;
	ndNodeHierarchy<T>* m_parent;
	ndNodeHierarchy<T>* m_lastChild;
	ndNodeHierarchy<T>* m_firstChild;
};


template<class T>
ndNodeHierarchy<T>::ndNodeHierarchy ()
	:ndContainersFreeListAlloc<T>()
	,m_next(nullptr)
	,m_prev(nullptr)
	,m_parent(nullptr)
	,m_lastChild(nullptr)
	,m_firstChild(nullptr)
{
}

template<class T>
ndNodeHierarchy<T>::ndNodeHierarchy (const ndNodeHierarchy<T>& clone)
	:ndContainersFreeListAlloc<T>()
{
	ndAssert(0);
}

template<class T>
ndNodeHierarchy<T>::~ndNodeHierarchy () 
{
	while (m_firstChild)
	{
		delete m_firstChild;
	}
	if (m_parent)
	{
		ndAssert(!m_prev);
		if (m_next)
		{
			m_next->m_prev = nullptr;
		}
		m_parent->m_firstChild = m_next;
		if (!m_next)
		{
			m_parent->m_lastChild = nullptr;
		}
	} 
	else if (m_next)
	{
		m_next->m_prev = nullptr;
	}
	m_next = nullptr;
	ndAssert(!m_prev);
	ndAssert(!m_lastChild);
	ndAssert(!m_firstChild);
}

template<class T>
ndNodeHierarchy<T>* ndNodeHierarchy<T>::CreateClone() const
{
	ndAssert(0);
	return new ndNodeHierarchy<T>(*this);
}

template<class T>
void ndNodeHierarchy<T>::Attach(ndNodeHierarchy<T>* const parent)
{
	ndAssert(!m_parent);
	m_parent = parent;
	
	if (m_parent->m_firstChild)
	{
		ndAssert(!m_prev);
		m_prev = m_parent->m_lastChild;
		m_parent->m_lastChild->m_next = this;
	}
	else
	{
		m_parent->m_firstChild = this;
	}
	m_parent->m_lastChild = this;
}

template<class T>
T* ndNodeHierarchy<T>::GetChild () const
{
	return (T*)m_firstChild;
}

template<class T>
T* ndNodeHierarchy<T>::GetLastChild() const
{
	return (T*)m_lastChild;
}

template<class T>
T* ndNodeHierarchy<T>::GetSibling () const
{
	return (T*) m_next;
}

template<class T>
T* ndNodeHierarchy<T>::GetPrev____() const
{
	return (T*)m_prev;
}

template<class T>
T* ndNodeHierarchy<T>::GetParent () const
{
	return (T*) m_parent;
}

template<class T>
T* ndNodeHierarchy<T>::GetRoot () const
{
	const ndNodeHierarchy<T>* root = this;
	for (; root->m_parent; root = root->m_parent);
	return (T*)root;
}


template<class T>
T* ndNodeHierarchy<T>::IteratorFirst() const
{
	const ndNodeHierarchy<T>* ptr = this;
	for (; ptr->m_firstChild; ptr = ptr->m_firstChild);
	return (T*) ptr;
}

template<class T>
T* ndNodeHierarchy<T>::IteratorNext() const
{
	if (m_next)
	{
		return m_next->IteratorFirst();
	}

	const ndNodeHierarchy<T>* x = this;
	const ndNodeHierarchy<T>* ptr = m_parent;
	for (; ptr && (x == ptr->m_next); ptr = ptr->m_parent)
	{
		x = ptr;
	}
	return (T*)ptr;
}

template<class T>
T* ndNodeHierarchy<T>::GetFirst() const
{
	ndAssert(0);
	if (m_parent)
	{
		return (T*)m_parent->m_firstChild;
	}
	else
	{
		const ndNodeHierarchy<T>* ptr = this;
		for (; ptr->m_prev; ptr = ptr->m_prev);
		return (T*)ptr;
	}
}


// **************************************************
template<class T>
void ndNodeHierarchy<T>::Detach()
{
	ndAssert(0);
	//NodeBaseHierarchy::Detach ();
}

template<class T>
T* ndNodeHierarchy<T>::GetLast() const
{
	ndAssert(0);
	if (m_parent)
	{
		return (T*)m_parent->m_lastChild;
	}
	else
	{
		const ndNodeHierarchy<T>* ptr = this;
		for (; ptr->m_next; ptr = ptr->m_next);
		return (T*)ptr;
	}
}

template<class T>
T* ndNodeHierarchy<T>::GetNext() const
{
	ndAssert(0);
	return nullptr;
	//return (T*) ndNodeBaseHierarchy::GetNext ();
}

template<class T>
T* ndNodeHierarchy<T>::GetPrev() const
{
	ndAssert(0);
	return nullptr;
	//return (T*) ndNodeBaseHierarchy::GetPrev ();
}


#endif

