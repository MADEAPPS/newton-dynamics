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
		const int threadId = threadIdx.x;
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

__global__ void cuHillisSteelePaddBuffer(cuSceneInfo& info)
{
	const unsigned itemsCount = info.m_histogram.m_size;
	const unsigned alignedItemsCount = D_PREFIX_SCAN_ALIGN * ((itemsCount + D_PREFIX_SCAN_ALIGN - 1) / D_PREFIX_SCAN_ALIGN);
	if ((alignedItemsCount + D_PREFIX_SCAN_ALIGN) > info.m_histogram.m_capacity)
	{
		if (threadIdx.x == 0)
		{
			#ifdef _DEBUG
			printf("function: cuHillisSteelePaddBuffer: buffer overflow\n");
			#endif
			info.m_frameIsValid = 1;
		}
		__syncthreads();
	}

	if (info.m_frameIsValid)
	{
		const unsigned threadId = threadIdx.x;
		const unsigned blockStart = D_THREADS_PER_BLOCK * ((itemsCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK);

		unsigned* histogram = info.m_histogram.m_array;
		for (unsigned offset = blockStart; offset < alignedItemsCount; offset += D_THREADS_PER_BLOCK)
		{
			histogram[offset + threadId] = 0;
		}
	}
}

__global__ void cuHillisSteelePrefixScanAddBlocks(cuSceneInfo& info, int bit)
{
	if (info.m_frameIsValid)
	{
		const unsigned blockId = blockIdx.x;
		const unsigned itemsCount = info.m_histogram.m_size;
		const unsigned alignedItemsCount = D_PREFIX_SCAN_ALIGN * ((itemsCount + D_PREFIX_SCAN_ALIGN - 1) / D_PREFIX_SCAN_ALIGN);
		const unsigned blocks = ((alignedItemsCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK);
		if (blockId < blocks)
		{
			const unsigned power = 1 << (bit + 1);
			const unsigned blockFrac = blockId & (power - 1);
			if (blockFrac >= (power >> 1))
			{
				const unsigned threadId = threadIdx.x;
				const unsigned dstIndex = D_THREADS_PER_BLOCK * blockId;
				const unsigned srcIndex = D_THREADS_PER_BLOCK * (blockId - blockFrac + (power >> 1)) - 1;

				unsigned* histogram = info.m_histogram.m_array;
				const unsigned value = histogram[srcIndex];
				histogram[dstIndex + threadId] += value;

				if ((power == D_PREFIX_SCAN_PASSES) && (blockFrac == (power - 1)))
				{
					__syncthreads();
					if (threadId == (D_THREADS_PER_BLOCK - 1))
					{
						unsigned dstBlock = blockId / D_PREFIX_SCAN_PASSES;
						const unsigned sum = histogram[blockId * D_THREADS_PER_BLOCK + threadId];
						histogram[alignedItemsCount + dstBlock] = sum;
					}
				}
			}
		}
	}
}

__global__ void cuHillisSteelePrefixScan(cuSceneInfo& info)
{
	if (info.m_frameIsValid)
	{
		const unsigned blockId = blockIdx.x;
		const unsigned threadId = threadIdx.x;
		const unsigned itemsCount = info.m_histogram.m_size;
		const unsigned superBlockCount = (itemsCount + D_PREFIX_SCAN_ALIGN - 1) / D_PREFIX_SCAN_ALIGN;

		unsigned* histogram = info.m_histogram.m_array;
		unsigned offset = blockId * D_THREADS_PER_BLOCK + D_PREFIX_SCAN_ALIGN;
		const unsigned superBlockOffset = superBlockCount * D_PREFIX_SCAN_ALIGN;

		unsigned value = histogram[superBlockOffset];
		for (int i = 1; i < superBlockCount; i++)
		{
			histogram[offset + threadId] += value;
			value += histogram[superBlockOffset + i];
			offset += D_PREFIX_SCAN_ALIGN;
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
	cuHillisSteelePaddBuffer << <1, D_THREADS_PER_BLOCK, 0, stream >> > (*infoGpu);

	const ndInt32 threads = context->m_histogram.GetCount();
	const ndInt32 superBlocks = (threads + D_PREFIX_SCAN_ALIGN - 1) / D_PREFIX_SCAN_ALIGN;
	const ndInt32 histogramBlocks = D_PREFIX_SCAN_PASSES * superBlocks;

#if 0
	for (ndInt32 pass = 1; pass < (D_PREFIX_SCAN_PASSES_BITS + 1); pass++)
	{
		for (unsigned blockID = 0; blockID < 100; blockID++)
		{
			//for (unsigned threadId = 0; threadId < D_THREADS_PER_BLOCK; threadId++)
			for (unsigned threadId = 0; threadId < 1; threadId++)
			{
				const unsigned itemsCount = context->m_histogram.GetCount();
				const unsigned alignedItemsCount = D_PREFIX_SCAN_ALIGN * ((itemsCount + D_PREFIX_SCAN_ALIGN - 1) / D_PREFIX_SCAN_ALIGN);
				const unsigned blocks = ((alignedItemsCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK);
				if (blockID < blocks)
				{
					const unsigned power = 1 << pass;
					const unsigned blockFrac = blockID & (power - 1);
					if (blockFrac >= (power >> 1))
					{
						const unsigned srcIndex = blockID - blockFrac + (power >> 1);
					}

					if ((pass == D_PREFIX_SCAN_PASSES_BITS) && (blockFrac == (power - 1)))
					{
						if (threadId == (D_THREADS_PER_BLOCK - 1))
						{
							const unsigned alignedItemsCount = D_PREFIX_SCAN_ALIGN * ((itemsCount + D_PREFIX_SCAN_ALIGN - 1) / D_PREFIX_SCAN_ALIGN);
							unsigned dstBlock = blockID / D_PREFIX_SCAN_PASSES_BITS;
						}
					}
				}
			}
		}
	}
#endif

	for (ndInt32 i = 0; i < D_PREFIX_SCAN_PASSES_BITS; i++)
	{
		cuHillisSteelePrefixScanAddBlocks << <histogramBlocks, D_THREADS_PER_BLOCK, 0, stream >> > (*infoGpu, i);
	}
	cuHillisSteelePrefixScan << <D_PREFIX_SCAN_PASSES, D_THREADS_PER_BLOCK, 0, stream >> > (*infoGpu);
#endif

}
