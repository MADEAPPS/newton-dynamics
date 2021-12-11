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

#ifndef __ND_CLASS_ALLOC_H_
#define __ND_CLASS_ALLOC_H_

#include "ndCoreStdafx.h"
#include "ndMemory.h"

/// Base class for providing memory allocation for all other engine classes.
class ndClassAlloc  
{
	public:
	/// Empty
	inline ndClassAlloc()
	{
	}

	/// Empty
	inline ~ndClassAlloc()
	{
	}

	D_OPERATOR_NEW_AND_DELETE

	/// Generic allocation for any function subclass from ndClassAlloc
	D_CORE_API static void* Malloc(size_t size);

	/// Generic destruction for any function subclass from ndClassAlloc
	D_CORE_API static void Free(void* const ptr);
};

#endif
