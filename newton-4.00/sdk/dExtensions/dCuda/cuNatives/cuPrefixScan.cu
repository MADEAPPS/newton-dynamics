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
		const unsigned blocks = (itemsCount + blockDim.x - 1) / blockDim.x;

		unsigned* histogram = info.m_histogram.m_array;
		unsigned offset = blockDim.x;

		for (int i = 1; i < blocks; i++)
		{
			const unsigned sum = histogram[offset - 1];
			histogram[offset + threadId] += sum;
			offset += blockDim.x;
			__syncthreads();
		}
	}
}

__global__ void cuHillisSteelePaddBuffer(cuSceneInfo& info)
{
	if (info.m_frameIsValid)
	{
		const unsigned blockId = blockIdx.x;
		const unsigned threadId = threadIdx.x;
		const unsigned itemsCount = info.m_histogram.m_size;
		const unsigned blockStart = blockDim.x * ((itemsCount + blockDim.x - 1) / blockDim.x);

		const unsigned blockOffset = blockId * blockDim.x;
		unsigned* histogram = info.m_histogram.m_array;
		
		if (blockOffset >= blockStart)
		{
			histogram[blockOffset + threadId] = 0;
		}
	}
}

__global__ void cuHillisSteelePrefixScanAddBlocks(cuSceneInfo& info, int bit)
{
	if (info.m_frameIsValid)
	{
		const unsigned blockId = blockIdx.x;
		const unsigned itemsCount = info.m_histogram.m_size;
		const unsigned prefixScanSuperBlockAlign = D_PREFIX_SCAN_PASSES * blockDim.x;
		const unsigned alignedItemsCount = prefixScanSuperBlockAlign * ((itemsCount + prefixScanSuperBlockAlign - 1) / prefixScanSuperBlockAlign);
		const unsigned blocks = ((alignedItemsCount + blockDim.x - 1) / blockDim.x);
		if (blockId < blocks)
		{
			const unsigned power = 1 << (bit + 1);
			const unsigned blockFrac = blockId & (power - 1);
			if (blockFrac >= (power >> 1))
			{
				const unsigned threadId = threadIdx.x;
				const unsigned dstIndex = blockDim.x * blockId;
				const unsigned srcIndex = blockDim.x * (blockId - blockFrac + (power >> 1)) - 1;

				unsigned* histogram = info.m_histogram.m_array;
				const unsigned value = histogram[srcIndex];
				histogram[dstIndex + threadId] += value;

				if ((power == D_PREFIX_SCAN_PASSES) && (blockFrac == (power - 1)))
				{
					__syncthreads();
					if (threadId == (blockDim.x - 1))
					{
						unsigned dstBlock = blockId / D_PREFIX_SCAN_PASSES;
						const unsigned sum = histogram[blockId * blockDim.x + threadId];
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
		const unsigned prefixScanSuperBlockAlign = D_PREFIX_SCAN_PASSES * blockDim.x;
		const unsigned superBlockCount = (itemsCount + prefixScanSuperBlockAlign - 1) / prefixScanSuperBlockAlign;

		unsigned* histogram = info.m_histogram.m_array;
		unsigned offset = blockId * blockDim.x + prefixScanSuperBlockAlign;
		const unsigned superBlockOffset = superBlockCount * prefixScanSuperBlockAlign;

		unsigned value = histogram[superBlockOffset];
		for (int i = 1; i < superBlockCount; i++)
		{
			histogram[offset + threadId] += value;
			value += histogram[superBlockOffset + i];
			offset += prefixScanSuperBlockAlign;
		}
	}
}


void CudaPrefixScan(ndCudaContext* const context, int blockSize)
{
	cudaStream_t stream = context->m_solverComputeStream;
	cuSceneInfo* const infoGpu = context->m_sceneInfoGpu;
#if 0
	cuLinearNaivePrefixScan << <1, blockSize, 0, stream >> > (*infoGpu);
#else

	const ndInt32 threads = context->m_histogram.GetCount();
	const unsigned prefixScanSuperBlockAlign = D_PREFIX_SCAN_PASSES * blockSize;
	const ndInt32 superBlocks = (threads + prefixScanSuperBlockAlign - 1) / prefixScanSuperBlockAlign;
	const ndInt32 histogramBlocks = D_PREFIX_SCAN_PASSES * superBlocks;

#if 0
	for (ndInt32 pass = 1; pass < (D_PREFIX_SCAN_PASSES_BITS + 1); pass++)
	{
		for (unsigned blockID = 0; blockID < 100; blockID++)
		{
			//for (unsigned threadId = 0; threadId < blockSize; threadId++)
			for (unsigned threadId = 0; threadId < 1; threadId++)
			{
				const unsigned itemsCount = context->m_histogram.GetCount();
				const unsigned alignedItemsCount = prefixScanSuperBlockAlign * ((itemsCount + prefixScanSuperBlockAlign - 1) / prefixScanSuperBlockAlign);
				const unsigned blocks = ((alignedItemsCount + blockSize - 1) / blockSize);
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
						if (threadId == (blockSize - 1))
						{
							const unsigned alignedItemsCount = prefixScanSuperBlockAlign * ((itemsCount + prefixScanSuperBlockAlign - 1) / prefixScanSuperBlockAlign);
							unsigned dstBlock = blockID / D_PREFIX_SCAN_PASSES_BITS;
						}
					}
				}
			}
		}
	}
#endif

	cuHillisSteelePaddBuffer << <D_PREFIX_SCAN_PASSES, blockSize, 0, stream >> > (*infoGpu);
	for (ndInt32 i = 0; i < D_PREFIX_SCAN_PASSES_BITS; i++)
	{
		cuHillisSteelePrefixScanAddBlocks << <histogramBlocks, blockSize, 0, stream >> > (*infoGpu, i);
	}
	cuHillisSteelePrefixScan << <D_PREFIX_SCAN_PASSES, blockSize, 0, stream >> > (*infoGpu);
#endif

}
