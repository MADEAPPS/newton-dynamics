/* Copyright (c) <2003-2013> <Julio Jerez, Newton Game Dynamics>
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

#ifndef _D_NEWTON_H_
#define _D_NEWTON_H_

#include <newton.h>

typedef void* (*CNewtonAllocMemory) (int sizeInBytes);
typedef void (*CNewtonFreeMemory) (void* const ptr, int sizeInBytes);


class dNewtonBody;

class dNewton  
{
	public:
	dNewton();
	virtual ~dNewton();

	void Update (dFloat timestepInSecunds);
	void UpdateAsync (dFloat timestepInSecunds);

  
	private:
	// default memory allocation funtions if not other is provided 
	static void SetAllocationDrivers (CNewtonAllocMemory alloc, CNewtonFreeMemory free);
	static void* DefualtAlloc (int sizeInBytes);
	static void DefualtFree (void* ptr, int sizeInBytes);


	NewtonWorld* m_world;
	static int m_totalMemoryUsed;
	static bool m_memorySystemInitialized;
};

#endif
