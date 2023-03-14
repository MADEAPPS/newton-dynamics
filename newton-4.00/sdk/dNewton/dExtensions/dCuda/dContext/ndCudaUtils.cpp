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


#include "ndCudaStdafx.h"
#include "ndCudaUtils.h"

static ndMemFreeCallback g_free = free;
static ndMemAllocCallback g_alloc = malloc;

void* ndCudaMalloc(size_t size)
{
	return g_alloc(size);
}

void ndCudaFree(void* const ptr)
{
	g_free(ptr);
}

void CudaSetMemoryAllocators(ndMemAllocCallback alloc, ndMemFreeCallback free)
{
	g_alloc = alloc;
	g_free = free;
}

void* operator new (size_t size)
{
	ndAssert(0);
	return g_alloc(size);
}

void operator delete (void* ptr)
{
	ndAssert(0);
	g_free(ptr);
}

void cudaExpandTraceMessage(const char* const fmt, ...)
{
	va_list v_args;
	char text[4096];

	text[0] = 0;
	va_start(v_args, fmt);
	vsprintf(text, fmt, v_args);
	va_end(v_args);

	#if (defined(WIN32) || defined(_WIN32))
		OutputDebugStringA(text);
	#else 
		printf("%s\n", text);
	#endif
}
