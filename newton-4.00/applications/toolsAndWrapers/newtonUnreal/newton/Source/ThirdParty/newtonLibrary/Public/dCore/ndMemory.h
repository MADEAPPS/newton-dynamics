/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_MEMORY_H__
#define __ND_MEMORY_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndThreadSyncUtils.h"

#define D_MEMORY_ALIGMNET	32
typedef void* (*ndMemAllocCallback) (size_t size);
typedef void (*ndMemFreeCallback) (void* const ptr);

class ndMemory
{
	public:
	/// General Memory allocation function.
	/// All memory allocations used by the Newton Engine and Tools 
	/// are performed by calling this function.
	D_CORE_API static void* Malloc(size_t size);

	/// Destroy a memory buffer previously allocated by Malloc.
	D_CORE_API static void Free(void* const ptr);

	/// Get memory buffer size previously allocated by Malloc. include extra align padding.
	D_CORE_API static size_t GetSize(void* const ptr);

	/// Get memory buffer size previously allocated by Malloc.
	D_CORE_API static size_t GetOriginalSize(void* const ptr);

	/// Calculate buffer size.
	D_CORE_API static size_t CalculateBufferSize(size_t size);

	/// Return the total memory allocated by the newton engine and tools.
	D_CORE_API static ndUnsigned64 GetMemoryUsed();

	/// Install low level system memory allocation functions.
	/// \param ndMemAllocCallback alloc: is a function pointer callback to allocate a memory chunk.
	/// \param ndMemFreeCallback free: is a function pointer callback to free a memory chunk.
	/// \brief All memory allocated by alloc, does not need to be aligned, therefore an application can
	/// write them using standard malloc and free.
	/// By default the memory allocation is set to call the standard 
	/// library functions malloc and free, however if an application wants to
	/// keep track of how memory is used, it must install the memory callbacks
	/// by calling this function before any class of the Newton Engine or tool 
	/// was created or instantiated. The engine does not do any global 
	/// allocation using global operators new and delete, therefore it 
	/// is ok to install the memory allocator on the main of the 
	/// application or just before start using the engine.
	D_CORE_API static void SetMemoryAllocators(ndMemAllocCallback alloc, ndMemFreeCallback free);
	D_CORE_API static void GetMemoryAllocators(ndMemAllocCallback& alloc, ndMemFreeCallback& free);

	private:
	static ndAtomic<ndUnsigned64> m_memoryUsed;
};

#endif
