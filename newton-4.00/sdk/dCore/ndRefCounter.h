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

#ifndef __NDREF_COUNTER_H__
#define __NDREF_COUNTER_H__

#include "ndCoreStdafx.h"
#include "ndClassAlloc.h"

template<class T>
class ndRefCounter: public ndClassAlloc
{
	public:
	ndRefCounter();
	ndInt32 GetRef() const;

	T* AddRef();
	virtual ndInt32 Release();

	protected:
	virtual ~ndRefCounter(void);

	private:
	ndInt32 m_refCount;
};

template<class T>
ndRefCounter<T>::ndRefCounter(void)
	:ndClassAlloc()
	,m_refCount(1)
{
}

template<class T>
inline ndRefCounter<T>::~ndRefCounter(void)
{
}

template<class T>
inline T* ndRefCounter<T>::AddRef()
{
	m_refCount++;
	return (T*) this;
}

template<class T>
inline ndInt32 ndRefCounter<T>::Release()
{
	m_refCount--;
	dAssert(m_refCount >= 0);
	if (!m_refCount)
	{
		delete this;
		return 0;
	}
	return m_refCount;
}

template<class T>
inline ndInt32 ndRefCounter<T>::GetRef() const
{
	return m_refCount;
}

#endif