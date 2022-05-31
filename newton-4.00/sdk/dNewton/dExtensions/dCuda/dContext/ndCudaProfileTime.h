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

#ifndef __ND_CUDA_PROFILE_TIME_H__
#define __ND_CUDA_PROFILE_TIME_H__

#include <cuda.h>
#include <vector_types.h>
#include <cuda_runtime.h>
#include "ndCudaSceneInfo.h"

#define D_CUDA_PROFILE_KERNELS

class ndCudaProfileTime
{
	public:
	inline __device__ ndCudaProfileTime(ndCudaSceneInfo& info)
		:m_info(&info)
	{
		if ((blockIdx.x == 0) && (threadIdx.x == 0))
		{
			m_savedTime = clock64();
		}
	}

	inline __device__ ~ndCudaProfileTime()
	{
		if ((blockIdx.x == 0) && (threadIdx.x == 0))
		{
			long long deltaTime = clock64() - m_savedTime;
			m_info->m_deltaTicks += deltaTime;
		}
	}

	ndCudaSceneInfo* m_info;
	long long m_savedTime;
};

#ifdef D_CUDA_PROFILE_KERNELS
#define ND_CUDA_PROFILE ndCudaProfileTime __timer__(info);
#else
#define ND_CUDA_PROFILE()
#endif

#endif