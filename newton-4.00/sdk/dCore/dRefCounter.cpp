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

#include "dCoreStdafx.h"
#include "dRefCounter.h"


dInt32 dRefCounter::Release()
{
	m_refCount --;
	dAssert (m_refCount >= 0);
	if (!m_refCount) 
	{
		delete this;
		return 0;
	}
	return m_refCount;
}


