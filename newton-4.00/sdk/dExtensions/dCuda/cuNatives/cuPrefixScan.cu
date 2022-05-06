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
		const unsigned itemsCount = info.m_histogram.m_size;
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
		const unsigned itemsCount = info.m_histogram.m_size;
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
		const int threadId = threadIdx.x;
		const unsigned itemsCount = info.m_histogram.m_size;
		const unsigned blocks = (itemsCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
		const unsigned lastBlock = D_PREFIX_SCAN_PASSES * ((blocks + D_PREFIX_SCAN_PASSES - 1) / D_PREFIX_SCAN_PASSES);

		unsigned* histogram = info.m_histogram.m_array;
		unsigned offset = blocks * D_THREADS_PER_BLOCK;

		for (int i = blocks; i < lastBlock; i++)
		{
			histogram[offset + threadId] = 0;
			offset += D_THREADS_PER_BLOCK;
		}
	}
}

__global__ void cuPrefixScanAddBlocks(cuSceneInfo& info, int bit)
{
	if (info.m_frameIsValid)
	{
		const int blockId = blockIdx.x;
		const int threadId = threadIdx.x;
		const unsigned itemsCount = info.m_histogram.m_size;
		const unsigned blocks = (itemsCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
		const unsigned superBlockCount = (D_PREFIX_SCAN_PASSES / 2) * ((blocks + D_PREFIX_SCAN_PASSES - 1) / D_PREFIX_SCAN_PASSES);
		if (blockId < superBlockCount)
		{
			unsigned* histogram = info.m_histogram.m_array;
			const int stride = 1 << bit;
			const int mask = 2 * stride;
			const int blockBase = D_THREADS_PER_BLOCK * blockId / mask;
			const int blockFrac = D_THREADS_PER_BLOCK * (blockId & (mask-1));
			const int halfStrideBlock = D_THREADS_PER_BLOCK * stride;
			const unsigned value = histogram[blockBase + halfStrideBlock - 1];
			histogram[blockBase + halfStrideBlock + blockFrac + threadId] += value;
		}
	}
}

void CudaPrefixScan(ndCudaContext* const context)
{
	cudaStream_t stream = context->m_solverComputeStream;
	cuSceneInfo* const infoGpu = context->m_sceneInfoGpu;
#if 0
	cuLinearNaivePrefixScan << <1, D_THREADS_PER_BLOCK, 0, stream >> > (*infoGpu);
#else
	cuPaddBuffer << <1, D_THREADS_PER_BLOCK, 0, stream >> > (*infoGpu);

	const ndInt32 threads = context->m_histogram.GetCount();
	const ndInt32 bodyBlocksCount = (threads + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
	const ndInt32 histogramBlocks = D_PREFIX_SCAN_PASSES * ((bodyBlocksCount + D_PREFIX_SCAN_PASSES - 1) / D_PREFIX_SCAN_PASSES);
	for (ndInt32 i = 0; i < D_PREFIX_SCAN_PASSES_BITS; i++)
	{
		cuPrefixScanAddBlocks << <histogramBlocks, D_THREADS_PER_BLOCK, 0, stream >> > (*infoGpu, i);
	}
#endif

}
