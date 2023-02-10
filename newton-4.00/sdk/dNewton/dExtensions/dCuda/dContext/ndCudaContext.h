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

#ifndef __ND_CUDA_CONTEXT_H__
#define __ND_CUDA_CONTEXT_H__

#include "ndCudaStdafx.h"

class ndCudaDevice;
//class ndCudaBodyProxy;
//class ndCudaSpatialVector;
class ndCudaContextImplement;

typedef void* (*ndMemAllocCallback) (size_t size);
typedef void (*ndMemFreeCallback) (void* const ptr);

class ndCudaContext
{
	public: 
	D_CUDA_API ndCudaContext();
	D_CUDA_API ~ndCudaContext();

	D_CUDA_API void* operator new (size_t size);
	D_CUDA_API void operator delete (void* ptr);

	D_CUDA_API bool IsValid() const;
	D_CUDA_API const char* GetStringId() const;

	D_CUDA_API void Begin();
	D_CUDA_API void End();
	D_CUDA_API void Cleanup();
	D_CUDA_API void PrepareCleanup();
	
#if 0
	D_CUDA_API void ResizeBuffers(int size);
	D_CUDA_API void LoadBodyData(const ndCudaBodyProxy* const src, int size);

	D_CUDA_API void InitBodyArray();
	D_CUDA_API void UpdateTransform();
	D_CUDA_API void ValidateContextBuffers();

	D_CUDA_API ndCudaSpatialVector* GetTransformBuffer();
	
	D_CUDA_API void IntegrateBodies(float timestep);
	D_CUDA_API void IntegrateUnconstrainedBodies(float timestep);
#endif

	D_CUDA_API static void SetMemoryAllocators(ndMemAllocCallback alloc, ndMemFreeCallback free);

	ndCudaDevice* m_device;
	ndCudaContextImplement* m_implement;
};

#endif