/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef _ND_WEAK_PTR_H_
#define _ND_WEAK_PTR_H_

#include "ndCoreStdafx.h"
#include "ndSharedPtr.h"

template <typename T>
class ndWeakPtr: public ndSharedPtr<T>
{
	public:
	ndWeakPtr();
	ndWeakPtr(const ndWeakPtr<T>& other);
	ndWeakPtr(const ndSharedPtr<T>& ptr);
	~ndWeakPtr();
	ndWeakPtr<T>& operator=(const ndWeakPtr<T>& other);
};

template <typename T>
ndWeakPtr<T>::ndWeakPtr()
	:ndSharedPtr<T>()
{
	ndAssert(0);
	ndAssert(ndSharedPtr<T>::m_references);
}

template <typename T>
ndWeakPtr<T>::ndWeakPtr(const ndWeakPtr<T>& other)
	:ndSharedPtr<T>(other)
{
	ndAssert(0);
	ndAssert(ndSharedPtr<T>::m_references);
	ndSharedPtr<T>::m_references->m_weakRef.fetch_add(1);
}

template <typename T>
ndWeakPtr<T>::ndWeakPtr(const ndSharedPtr<T>& ptr)
	:ndSharedPtr<T>(ptr)
{
	ndAssert(0);
	ndAssert(ndSharedPtr<T>::m_references);
}

template <typename T>
ndWeakPtr<T>::~ndWeakPtr()
{
	ndAssert(0);
	ndAssert(ndSharedPtr<T>::m_references);
	ndSharedPtr<T>::m_references->m_weakRef.fetch_add(-1);
}

template <typename T>
//ndWeakPtr<T>& ndWeakPtr<T>::operator=(const ndWeakPtr<T>& other)
ndWeakPtr<T>& ndWeakPtr<T>::operator=(const ndWeakPtr<T>&)
{
	ndAssert(0);
	//sharedPtr = other.sharedPtr;
	return *this;
}

#endif 

