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

#ifndef __DREF_COUNTER_H__
#define __DREF_COUNTER_H__

#include "dCoreStdafx.h"
#include "dClassAlloc.h"

template<class T>
class dRefCounter: public dClassAlloc
{
	public:
	dRefCounter();
	dInt32 GetRef() const;

	T* AddRef();
	virtual dInt32 Release();

	protected:
	virtual ~dRefCounter(void);

	private:
	dInt32 m_refCount;
};

template<class T>
dRefCounter<T>::dRefCounter(void)
	:dClassAlloc()
	,m_refCount(1)
{
}

template<class T>
inline dRefCounter<T>::~dRefCounter(void)
{
}

template<class T>
inline T* dRefCounter<T>::AddRef()
{
	m_refCount++;
	return (T*) this;
}

template<class T>
inline dInt32 dRefCounter<T>::Release()
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
inline dInt32 dRefCounter<T>::GetRef() const
{
	return m_refCount;
}

#endif