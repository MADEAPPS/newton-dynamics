/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __D_FIX_SIZE_BUFFER_H__
#define __D_FIX_SIZE_BUFFER_H__

#include "dCoreStdafx.h"

template<class T, dInt32 size>
class dFixSizeBuffer: public dClassAlloc
{
	public:
	dFixSizeBuffer()
		:dClassAlloc()
	{
	}

	dInt32 GetSize() const 
	{
		return size;
	}

	T& operator[] (dInt32 i)
	{
		dAssert(i >= 0);
		dAssert(i < size);
		return m_array[i];
	}

	const T& operator[] (dInt32 i) const
	{
		dAssert(i >= 0);
		dAssert(i < size);
		return m_array[i];
	}

	T m_array[size];
};

#endif
