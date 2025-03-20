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

#ifndef __ND_SHARED_NODE_HIERARCHY_H__
#define __ND_SHARED_NODE_HIERARCHY_H__

#include "ndCoreStdafx.h"
#include "ndList.h"
#include "ndWeakPtr.h"
#include "ndSharedPtr.h"
#include "ndContainersAlloc.h"

template<class T>
class ndSharedNodeHierarchy: public ndContainersFreeListAlloc<T>
{
	public:
	ndSharedNodeHierarchy ();
	virtual ~ndSharedNodeHierarchy();

	virtual T* CreateClone() const;
	
	void Detach();
	void Attach(ndSharedPtr<T>& self, ndSharedPtr<T>& parent);

	const T* GetRoot() const;
	const T* GetParent() const;
	const ndList<ndSharedPtr<T>>& GetChildren() const;

	protected:
	ndSharedNodeHierarchy(const ndSharedNodeHierarchy<T>& src);

	ndList<ndSharedPtr<T>> m_children;
	//ndWeakPtr<T> m_parent;
	T* m_parent;
};

template<class T>
ndSharedNodeHierarchy<T>::ndSharedNodeHierarchy ()
	:ndContainersFreeListAlloc<T>()
	,m_children()
	,m_parent(nullptr)
{
}

template<class T>
ndSharedNodeHierarchy<T>::ndSharedNodeHierarchy (const ndSharedNodeHierarchy<T>& src)
	:ndContainersFreeListAlloc<T>()
	,m_children()
	,m_parent(nullptr)
{
	ndAssert(0);
	//for (ndSharedNodeHierarchy<T>* obj = clone.m_firstChild; obj; obj = obj->m_next)
	//{
	//	T* const child = obj->CreateClone();
	//	child->Attach(this);
	//}
}

template<class T>
ndSharedNodeHierarchy<T>::~ndSharedNodeHierarchy () 
{
	//m_parent = ndWeakPtr<T>();
	//if (m_parent)
	//{
	//	for (ndList<ndSharedPtr<T>>::ndNode* node = m_parent->m_children.GetFirst(); node; node = node->GetNext())
	//	{
	//		if (*node->GetInfo() == this)
	//		{
	//			m_parent->m_children->Remove()
	//			break;
	//		}
	//	}
	//}
	//m_parent = nullptr;
}

template<class T>
T* ndSharedNodeHierarchy<T>::CreateClone() const
{
	ndAssert(0);
	return nullptr;
	//return (T*) new ndSharedNodeHierarchy<T>(*this);
}


template<class T>
void ndSharedNodeHierarchy<T>::Attach(ndSharedPtr<T>& self, ndSharedPtr<T>& parent)
{
	ndAssert(*parent);
	ndAssert(!m_parent);
	ndAssert(*self == this);

	m_parent = *parent;
	m_parent->m_children.Append(self);
}

template<class T>
void ndSharedNodeHierarchy<T>::Detach()
{
	ndAssert(0);
	//NodeBaseHierarchy::Detach ();
}

template<class T>
const T* ndSharedNodeHierarchy<T>::GetRoot() const
{
	ndAssert(0);
	const T* root;
	for (root = this; root; root = root->m_parent);
	return root;
}

template<class T>
//const ndSharedPtr<T>& ndSharedNodeHierarchy<T>::GetParent() const
const T* ndSharedNodeHierarchy<T>::GetParent() const
{
	return m_parent;
}

template<class T>
typename const ndList<ndSharedPtr<T>>& ndSharedNodeHierarchy<T>::GetChildren() const
{
	return m_children;
}

#endif

