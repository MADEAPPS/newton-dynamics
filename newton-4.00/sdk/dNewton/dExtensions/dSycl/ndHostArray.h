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

#ifndef __ND_HOST_ARRAY_H__
#define __ND_HOST_ARRAY_H__

#include "ndSyclStdafx.h"
#include "ndHostAllocator.h"

#define D_HOST_SORT_BLOCK_SIZE	(1 << 10)
//#define D_HOST_SORT_BLOCK_SIZE	(1<<8)

template <class T>
class ndHostArray : public std::vector<T, ndHostAllocator<T> >
{
};

template <class T, class ndEvaluateKey, int exponentRadix>
void ndCountingSort(const ndHostArray<T>& src, ndHostArray<T>& dst, ndHostArray<int>& scansBuffer);

template <class T, class ndEvaluateKey, int exponentRadix>
void ndCountingSort(const ndHostArray<T>& src, ndHostArray<T>& dst, ndHostArray<int>& scansBuffer)
{
	auto AddPrefix = [&](int blockIdx, int blockDim, int computeUnits)
	{
		int sum[D_HOST_SORT_BLOCK_SIZE];
		int offset[D_HOST_SORT_BLOCK_SIZE];
		int localPrefixScan[D_HOST_SORT_BLOCK_SIZE / 2 + D_HOST_SORT_BLOCK_SIZE + 1];

		for (int threadId = 0; threadId < blockDim; ++threadId)
		{
			sum[threadId] = 0;
			localPrefixScan[threadId] = 0;
			offset[threadId] = threadId;
		}

		for (int i = 0; i < computeUnits; ++i)
		{
			for (int threadId = 0; threadId < blockDim; ++threadId)
			{
				int count = scansBuffer[offset[threadId]];
				scansBuffer[offset[threadId]] = sum[threadId];
				sum[threadId] += count;
				offset[threadId] += blockDim;
			}
		}

		for (int threadId = 0; threadId < blockDim; ++threadId)
		{
			scansBuffer[offset[threadId]] = sum[threadId];
			localPrefixScan[blockDim / 2 + threadId + 1] = sum[threadId];
		}

		for (int i = 1; i < blockDim; i = i << 1)
		{
			for (int threadId = 0; threadId < blockDim; ++threadId)
			{
				sum[threadId] = localPrefixScan[blockDim / 2 + threadId] + localPrefixScan[blockDim / 2 - i + threadId];
			}
			for (int threadId = 0; threadId < blockDim; ++threadId)
			{
				localPrefixScan[blockDim / 2 + threadId] = sum[threadId];
			}
		}

		for (int threadId = 0; threadId < blockDim; ++threadId)
		{
			scansBuffer[offset[threadId] + blockDim] = localPrefixScan[blockDim / 2 + threadId];
		}
	};

	auto CountItems = [&](int blockIndex, int blocksCount)
	{
		int radixCountBuffer[D_HOST_SORT_BLOCK_SIZE];

		int size = src.size();
		int blockStride = D_HOST_SORT_BLOCK_SIZE;
		int bashSize = blocksCount * blockStride * blockIndex;

		ndEvaluateKey evaluator;

		for (int threadId = 0; threadId < blockStride; ++threadId)
		{
			radixCountBuffer[threadId] = 0;
		}

		for (int i = 0; i < blocksCount; ++i)
		{
			for (int threadId = 0; threadId < blockStride; ++threadId)
			{
				int index = bashSize + threadId;
				if (index < size)
				{
					int radix = evaluator.GetRadix(src[index]);
					radixCountBuffer[radix] ++;
				}
			}
			bashSize += blockStride;
		}

		for (int threadId = 0; threadId < (1 << exponentRadix); ++threadId)
		{
			int index = threadId + (1 << exponentRadix) * blockIndex;
			scansBuffer[index] = radixCountBuffer[threadId];
		}
	};

	auto MergeBuckects = [&](int blockIdx, int blocksCount, int computeUnits)
	{
		ndEvaluateKey evaluator;
		T cachedItems[D_HOST_SORT_BLOCK_SIZE];
		int sortedRadix[D_HOST_SORT_BLOCK_SIZE];
		int radixPrefixCount[D_HOST_SORT_BLOCK_SIZE];
		int radixPrefixStart[D_HOST_SORT_BLOCK_SIZE];
		int radixPrefixBatchScan[D_HOST_SORT_BLOCK_SIZE];
		int radixPrefixScan[D_HOST_SORT_BLOCK_SIZE / 2 + D_HOST_SORT_BLOCK_SIZE + 1];

		int size = src.size();
		int radixSize = (1 << exponentRadix);
		int blockDim = D_HOST_SORT_BLOCK_SIZE;
		int radixBase = blockIdx * radixSize;
		int bashSize = blocksCount * blockDim * blockIdx;
		int radixPrefixOffset = computeUnits * radixSize + radixSize;

		for (int threadId = 0; threadId < blockDim; ++threadId)
		{
			radixPrefixScan[threadId] = 0;
		}

		for (int threadId = 0; threadId < radixSize; ++threadId)
		{
			radixPrefixStart[threadId] = scansBuffer[radixBase + threadId];
			radixPrefixBatchScan[threadId] = scansBuffer[radixPrefixOffset + threadId];
		}

		for (int i = 0; i < blocksCount; ++i)
		{
			for (int threadId = 0; threadId < blockDim; ++threadId)
			{
				radixPrefixCount[threadId] = 0;
				sortedRadix[threadId] = (radixSize << 16) + threadId;
			}

			for (int threadId = 0; threadId < blockDim; ++threadId)
			{
				int index = bashSize + threadId;
				if (index < size)
				{
					cachedItems[threadId] = src[index];
					int radix = evaluator.GetRadix(cachedItems[threadId]);
					radixPrefixCount[radix] ++;
					sortedRadix[threadId] = (radix << 16) + threadId;
				}
			}

			for (int k = 2; k <= blockDim; k = k << 1)
			{
				for (int j = k >> 1; j > 0; j = j >> 1)
				{
					for (int threadId0 = 0; threadId0 < blockDim; ++threadId0)
					{
						int threadId1 = threadId0 ^ j;
						if (threadId1 > threadId0)
						{
							const int a = sortedRadix[threadId0];
							const int b = sortedRadix[threadId1];
							const int mask0 = (-(threadId0 & k)) >> 31;
							const int mask1 = -(a > b);
							const int mask = mask0 ^ mask1;
							sortedRadix[threadId0] = (b & mask) | (a & ~mask);
							sortedRadix[threadId1] = (a & mask) | (b & ~mask);
						}
					}
				}
			}

			for (int threadId = 0; threadId < radixSize; ++threadId)
			{
				radixPrefixScan[D_HOST_SORT_BLOCK_SIZE / 2 + threadId + 1] = radixPrefixCount[threadId];
			}

			for (int k = 1; k < radixSize; k = k << 1)
			{
				int sumReg[D_HOST_SORT_BLOCK_SIZE];
				for (int threadId = 0; threadId < radixSize; ++threadId)
				{
					sumReg[threadId] = radixPrefixScan[D_HOST_SORT_BLOCK_SIZE / 2 + threadId] + radixPrefixScan[D_HOST_SORT_BLOCK_SIZE / 2 + threadId - k];
				}
				for (int threadId = 0; threadId < radixSize; ++threadId)
				{
					radixPrefixScan[D_HOST_SORT_BLOCK_SIZE / 2 + threadId] = sumReg[threadId];
				}
			}

			for (int threadId = 0; threadId < blockDim; ++threadId)
			{
				int index = bashSize + threadId;
				if (index < size)
				{
					int keyValue = sortedRadix[threadId];
					int keyHigh = keyValue >> 16;
					int keyLow = keyValue & 0xffff;
					int dstOffset1 = radixPrefixBatchScan[keyHigh] + radixPrefixStart[keyHigh];
					int dstOffset0 = threadId - radixPrefixScan[D_HOST_SORT_BLOCK_SIZE / 2 + keyHigh];
					dst[dstOffset0 + dstOffset1] = cachedItems[keyLow];
				}
			}

			for (int threadId = 0; threadId < radixSize; ++threadId)
			{
				radixPrefixStart[threadId] += radixPrefixCount[threadId];
			}

			bashSize += blockDim;
		}
	};

	ndAssert(src.size() == dst.size());
	//ndAssert(scansBuffer.size() >= src.size());

	int deviceComputeUnits = 20;
	int itemCount = src.size();
	int computeUnitsBashCount = (itemCount + D_HOST_SORT_BLOCK_SIZE - 1) / D_HOST_SORT_BLOCK_SIZE;
	int bashCount = (computeUnitsBashCount + deviceComputeUnits - 1) / deviceComputeUnits;
	int computeUnits = (itemCount + bashCount * D_HOST_SORT_BLOCK_SIZE - 1) / (bashCount * D_HOST_SORT_BLOCK_SIZE);

	ndAssert(computeUnits <= deviceComputeUnits);
	for (int block = 0; block < computeUnits; ++block)
	{
		CountItems(block, bashCount);
	}

	for (int block = 0; block < 1; ++block)
	{
		AddPrefix(block, 1 << exponentRadix, computeUnits);
	}

	for (int block = 0; block < computeUnits; ++block)
	{
		MergeBuckects(block, bashCount, computeUnits);
	}
}

#endif