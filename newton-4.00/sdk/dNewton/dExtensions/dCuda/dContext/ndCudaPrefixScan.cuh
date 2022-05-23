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

#ifndef __CUDA_PREFIX_SCAN_H__
#define __CUDA_PREFIX_SCAN_H__

#include <cuda.h>
#include <vector_types.h>
#include <cuda_runtime.h>

#include "ndCudaContext.h"
#include "ndCudaSceneInfo.h"
#include "ndCudaIntrinsics.h"

#define D_PREFIX_SCAN_PASSES_BITS	4
#define D_PREFIX_SCAN_PASSES		(1<<D_PREFIX_SCAN_PASSES_BITS)

__global__ void ndCudaHillisSteelePaddBufferInternal(ndCudaSceneInfo& info)
{
	const unsigned blockId = blockIdx.x;
	const unsigned threadId = threadIdx.x;
	const unsigned itemsCount = info.m_histogram.m_size;
	const unsigned blockStart = blockDim.x * ((itemsCount + blockDim.x - 1) / blockDim.x);

	const unsigned blockOffset = blockId * blockDim.x;
	unsigned* histogram = info.m_histogram.m_array;

	//if (threadId == 0 && blockId == 1)
	//{
	//	printf("ndCudaHillisSteelePaddBuffer\n");
	//}
	if (blockOffset >= blockStart)
	{
		histogram[blockOffset + threadId] = 0;
	}
}

__global__ void ndCudaHillisSteelePrefixScanAddBlocksInternal(ndCudaSceneInfo& info, int bit)
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
		}
	}
}

__global__ void ndCudaHillisSteelePrefixScanAddBlocksFinalInternal(ndCudaSceneInfo& info)
{
	const unsigned blockId = blockIdx.x;
	const unsigned itemsCount = info.m_histogram.m_size;
	const unsigned prefixScanSuperBlockAlign = D_PREFIX_SCAN_PASSES * blockDim.x;
	const unsigned alignedItemsCount = prefixScanSuperBlockAlign * ((itemsCount + prefixScanSuperBlockAlign - 1) / prefixScanSuperBlockAlign);
	const unsigned blocks = ((alignedItemsCount + blockDim.x - 1) / blockDim.x);
	if (blockId < blocks)
	{
		const unsigned power = 1 << D_PREFIX_SCAN_PASSES_BITS;
		const unsigned blockFrac = blockId & (power - 1);
		if (blockFrac >= (power >> 1))
		{
			const unsigned threadId = threadIdx.x;
			const unsigned dstIndex = blockDim.x * blockId;
			const unsigned srcIndex = blockDim.x * (blockId - blockFrac + (power >> 1)) - 1;

			unsigned* histogram = info.m_histogram.m_array;
			const unsigned value = histogram[srcIndex];
			histogram[dstIndex + threadId] += value;

			if (blockFrac == (power - 1))
			{
				__syncthreads();
				if (threadId == (blockDim.x - 1))
				{
					const unsigned dstBlock = blockId / D_PREFIX_SCAN_PASSES;
					const unsigned sum = histogram[blockId * blockDim.x + threadId];
					histogram[alignedItemsCount + dstBlock] = sum;
				}
			}
		}
	}
}

__global__ void ndCudaHillisSteeleAddSupeBlocksInternal(ndCudaSceneInfo& info)
{
	const unsigned blockId = blockIdx.x;
	const unsigned threadId = threadIdx.x;
	const unsigned itemsCount = info.m_histogram.m_size;
	const unsigned prefixScanSuperBlockAlign = D_PREFIX_SCAN_PASSES * blockDim.x;
	const unsigned superBlockCount = (itemsCount + prefixScanSuperBlockAlign - 1) / prefixScanSuperBlockAlign;

	unsigned* histogram = info.m_histogram.m_array;
	unsigned offset = blockId * blockDim.x + prefixScanSuperBlockAlign;
	const unsigned superBlockOffset = superBlockCount * prefixScanSuperBlockAlign;

	//if (threadId == 0 && blockId == 1)
	//{
	//	printf("ndCudaHillisSteeleAddSupeBlocksInternal\n");
	//}
	unsigned value = histogram[superBlockOffset];
	for (int i = 1; i < superBlockCount; i++)
	{
		histogram[offset + threadId] += value;
		value += histogram[superBlockOffset + i];
		offset += prefixScanSuperBlockAlign;
	}
}

__global__ void ndCudaHillisSteelePrefixScan(ndCudaSceneInfo& info, unsigned blockSize)
{
	const unsigned threads = info.m_histogram.m_size;
	const unsigned prefixScanSuperBlockAlign = D_PREFIX_SCAN_PASSES * blockSize;
	const unsigned superBlocks = (threads + prefixScanSuperBlockAlign - 1) / prefixScanSuperBlockAlign;
	const unsigned histogramBlocks = D_PREFIX_SCAN_PASSES * superBlocks;

	ndCudaHillisSteelePaddBufferInternal << <D_PREFIX_SCAN_PASSES, blockSize, 0 >> > (info);
	for (int i = 0; i < (D_PREFIX_SCAN_PASSES_BITS - 1); i++)
	{
		ndCudaHillisSteelePrefixScanAddBlocksInternal << <histogramBlocks, blockSize, 0 >> > (info, i);
	}
	ndCudaHillisSteelePrefixScanAddBlocksFinalInternal << <histogramBlocks, blockSize, 0 >> > (info);
	ndCudaHillisSteeleAddSupeBlocksInternal << <D_PREFIX_SCAN_PASSES, blockSize, 0 >> > (info);
}

#endif