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

#include <cuda.h>
#include <vector_types.h>
#include <cuda_runtime.h>
#include <ndNewtonStdafx.h>

#include "cuPrefixScan.h"

__global__ void cuLinearNaivePrefixScan(cuSceneInfo& info)
{
	if (info.m_frameIsValid)
	{
		int threadId = threadIdx.x;
		const unsigned itemsCount = info.m_histogram.m_size - 1;
		const unsigned blocks = (itemsCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;

		unsigned* histogram = info.m_histogram.m_array;
		unsigned offset = D_THREADS_PER_BLOCK;

		for (int i = 1; i < blocks; i++)
		{
			const unsigned sum = histogram[offset - 1];
			histogram[offset + threadId] += sum;
			offset += D_THREADS_PER_BLOCK;
			__syncthreads();
		}
	}
}

__global__ void cuHillisSteelePrefixScan(cuSceneInfo& info)
{
	if (info.m_frameIsValid)
	{
		int threadId = threadIdx.x;
		const unsigned itemsCount = info.m_histogram.m_size - 1;
		const unsigned blocks = (itemsCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;

		unsigned* histogram = info.m_histogram.m_array;
		unsigned offset = D_THREADS_PER_BLOCK;

		for (int i = 1; i < blocks; i++)
		{
			const unsigned sum = histogram[offset - 1];
			histogram[offset + threadId] += sum;
			offset += D_THREADS_PER_BLOCK;
			__syncthreads();
		}
	}
}

__global__ void cuPaddBuffer(cuSceneInfo& info)
{
	if (info.m_frameIsValid)
	{
		int threadId = threadIdx.x;
		const unsigned itemsCount = info.m_histogram.m_size - 1;
		const unsigned blocks = (itemsCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
		const unsigned lastBlock = D_PREFIX_SCAN_PASSES * ((blocks + D_PREFIX_SCAN_PASSES - 1) / D_PREFIX_SCAN_PASSES);

		unsigned* histogram = info.m_histogram.m_array;
		unsigned offset = blocks * D_THREADS_PER_BLOCK;

		for (int i = blocks; i < lastBlock; i++)
		{
			histogram[offset + threadId] = 0;
			offset += D_THREADS_PER_BLOCK;
			__syncthreads();
		}
	}
}

void CudaPrefixScan(ndCudaContext* const context)
{
	cuSceneInfo* const sceneInfo = context->m_sceneInfoCpu;
	cudaStream_t stream = context->m_solverComputeStream;
	cuSceneInfo* const infoGpu = context->m_sceneInfoGpu;
#if 1
	cuLinearNaivePrefixScan << <1, D_THREADS_PER_BLOCK, 0, stream >> > (*infoGpu);
#else
	cuPaddBuffer << <1, D_THREADS_PER_BLOCK, 0, stream >> > (*infoGpu);
#endif

}
