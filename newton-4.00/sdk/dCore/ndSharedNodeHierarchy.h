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
	//void Attach(ndSharedPtr<T>& self, T* const parent);
	void AddChild(const ndSharedPtr<T>& child);

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
	for (typename ndList<ndSharedPtr<T>>::ndNode* node = src.m_children.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<T> child(node->GetInfo()->CreateClone());
		AddChild(child);
	}
}

template<class T>
ndSharedNodeHierarchy<T>::~ndSharedNodeHierarchy () 
{
	while (m_children.GetCount())
	{
		typename ndList<ndSharedPtr<T>>::ndNode* const node = m_children.GetLast();
		T* const childParerent = *node->GetInfo();
		childParerent->m_parent = nullptr;
		m_children.Remove(node);
	}
}

template<class T>
T* ndSharedNodeHierarchy<T>::CreateClone() const
{
	ndAssert(0);
	return nullptr;
	//return (T*) new ndSharedNodeHierarchy<T>(*this);
}

//template<class T>
//void ndSharedNodeHierarchy<T>::Attach(ndSharedPtr<T>& self, T* const parent)
//{
//	ndAssert(parent);
//	ndAssert(!m_parent);
//	ndAssert(*self == this);
//
//	m_parent = parent;
//	m_parent->m_children.Append(self);
//}

template<class T>
void ndSharedNodeHierarchy<T>::AddChild(const ndSharedPtr<T>& child)
{
	ndAssert(!child->m_parent);
	child->m_parent = (T*)this;
	m_children.Append(child);
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
	const T* root = (T*)this;
	for (; root->m_parent; root = root->m_parent);
	return root;
}

template<class T>
const T* ndSharedNodeHierarchy<T>::GetParent() const
{
	return m_parent;
}

template<class T>
const ndList<ndSharedPtr<T>>& ndSharedNodeHierarchy<T>::GetChildren() const
{
	return m_children;
}

#endif

