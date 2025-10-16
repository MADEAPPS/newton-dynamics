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
	virtual T* CreateClone() const;
	
	void Attach(ndNodeHierarchy<T>* const parent);
	void Detach ();

	T* GetParent() const;
	T* GetLastChild() const;
	T* GetFirstChild() const;

	T* GetPrev() const;
	T* GetNext () const;
	T* GetRoot () const;

	T* GetFirstIterator() const;
	T* GetNextIterator() const;

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
	,m_next(nullptr)
	,m_prev(nullptr)
	,m_parent(nullptr)
	,m_lastChild(nullptr)
	,m_firstChild(nullptr)
{
	for (ndNodeHierarchy<T>* obj = clone.m_firstChild; obj; obj = obj->m_next)
	{
		T* const child = obj->CreateClone();
		child->Attach(this);
	}
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
		if (m_next)
		{
			m_next->m_prev = m_prev;
		}
		else
		{
			m_parent->m_lastChild = m_prev;
		}
		if (m_prev)
		{
			m_prev->m_next = m_next;
		}
		else
		{
			m_parent->m_firstChild = m_next;
		}
	} 
	else if (m_next)
	{
		m_next->m_prev = m_prev;
	}
	else if (m_prev)
	{
		m_prev->m_next = m_next;
	}
	m_next = nullptr;
	m_prev = nullptr;

	ndAssert(!m_lastChild);
	ndAssert(!m_firstChild);
}

template<class T>
T* ndNodeHierarchy<T>::CreateClone() const
{
	return (T*) new ndNodeHierarchy<T>(*this);
}

template<class T>
void ndNodeHierarchy<T>::Attach(ndNodeHierarchy<T>* const parent)
{
	ndAssert(parent);
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
void ndNodeHierarchy<T>::Detach()
{
	ndAssert(0);
	//NodeBaseHierarchy::Detach ();
}

template<class T>
T* ndNodeHierarchy<T>::GetFirstChild () const
{
	return (T*)m_firstChild;
}

template<class T>
T* ndNodeHierarchy<T>::GetLastChild() const
{
	return (T*)m_lastChild;
}

template<class T>
T* ndNodeHierarchy<T>::GetNext() const
{
	return (T*)m_next;
}

template<class T>
T* ndNodeHierarchy<T>::GetPrev() const
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
T* ndNodeHierarchy<T>::GetFirstIterator() const
{
	const ndNodeHierarchy<T>* ptr = this;
	for (; ptr->m_firstChild; ptr = ptr->m_firstChild);
	return (T*)ptr;
}

template<class T>
T* ndNodeHierarchy<T>::GetNextIterator() const
{
	if (m_next)
	{
		return m_next->GetFirstIterator();
	}

	const ndNodeHierarchy<T>* x = this;
	const ndNodeHierarchy<T>* ptr = m_parent;
	for (; ptr && (x == ptr->m_next); ptr = ptr->m_parent)
	{
		x = ptr;
	}
	return (T*)ptr;
}

#endif

