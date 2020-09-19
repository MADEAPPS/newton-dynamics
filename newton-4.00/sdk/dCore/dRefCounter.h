/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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

class dRefCounter: public dClassAlloc
{
	public:
	dRefCounter(void);
	dInt32 GetRef() const;
	dRefCounter* AddRef();

	D_CORE_API dInt32 Release();

	protected:
	virtual ~dRefCounter(void);

	private:
	dInt32 m_refCount;
};

inline dRefCounter::dRefCounter(void)
	:dClassAlloc()
	, m_refCount(1)
{
}

inline dRefCounter::~dRefCounter(void)
{
}

inline dRefCounter* dRefCounter::AddRef()
{
	m_refCount++;
	return this;
}


inline dInt32 dRefCounter::GetRef() const
{
	return m_refCount;
}

#endif