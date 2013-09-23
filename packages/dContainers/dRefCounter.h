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


#ifndef __DREF_COUNTER_H__
#define __DREF_COUNTER_H__

#include "dContainersAlloc.h"


class dRefCounter: public dContainersAlloc
{
	public:
	DCONTAINER_API dRefCounter(void);
	DCONTAINER_API int GetRef() const;
	DCONTAINER_API int Release();
	DCONTAINER_API void AddRef() const;

	protected:
	DCONTAINER_API virtual ~dRefCounter(void);

	private:
	mutable int m_refCount;
};

#endif