/*
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

#include "stdafx.h"
#include "dAlloc.h"


class MemoryDriverSingleton
{
	public:
	MemoryDriverSingleton()
	{
		NewtonSetMemorySystem(Malloc, Free);
	}

	static void* Malloc(int sizeInBytes)
	{
		return malloc(sizeInBytes);
	}

	static void Free(void* const ptr)
	{
		free(ptr);
	}

	static MemoryDriverSingleton& GetMemoryDriverSingleton ()
	{
		static MemoryDriverSingleton singleton;
		return singleton;
	}

	private: 
	static void Free(void* const ptr, int sizeInBytes)
	{
		Free(ptr);
	}
};


void* dAlloc::operator new (size_t size)
{
	MemoryDriverSingleton::GetMemoryDriverSingleton();
	return NewtonAlloc(int (size));
}

void dAlloc::operator delete (void* ptr)
{
	MemoryDriverSingleton::GetMemoryDriverSingleton();
	NewtonFree(ptr);
}

void* operator new (size_t size)
{
	MemoryDriverSingleton::GetMemoryDriverSingleton();
	return MemoryDriverSingleton::Malloc(int(size));
}

void operator delete (void* ptr)
{
	MemoryDriverSingleton::GetMemoryDriverSingleton();
	MemoryDriverSingleton::Free(ptr);
}