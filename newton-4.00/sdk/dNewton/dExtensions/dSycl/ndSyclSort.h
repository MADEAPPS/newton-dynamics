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

#ifndef __ND_SORT_H__
#define __ND_SORT_H__

#include "ndSyclContextImpl.h"

using namespace sycl;

#define D_COUNTING_SORT_LOCAL_BLOCK_SIZE	(1<<10)

/*
template <typename BufferItem, typename SortKeyPredicate>
__global__ void ndCudaCountingSortCountItemsInternal(const BufferItem* src, unsigned* histogram, unsigned size, SortKeyPredicate sortKey, unsigned prefixKeySize)
{
	__shared__  unsigned cacheBuffer[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];

	const unsigned blockId = blockIdx.x;
	const unsigned threadId = threadIdx.x;

	cacheBuffer[threadId] = 0;
	__syncthreads();

	const unsigned index = threadId + blockDim.x * blockId;
	if (index < size)
	{
		const unsigned key = sortKey(src[index]);
		atomicAdd(&cacheBuffer[key], 1);
	}
	__syncthreads();

	if (threadId < prefixKeySize)
	{
		const unsigned dstBase = prefixKeySize * blockId;
		histogram[dstBase + threadId] = cacheBuffer[threadId];
	}
}
*/
template <class T, class ndEvaluateKey, int bitSize>
void __CountingSortCountItems__(sycl::queue& queue, buffer<T>& src, buffer<unsigned>& scansBuffer)
{
	queue.submit([&](auto& handler)
	{
		int arraySize = src.size();
		int workGroupSize = 1 << bitSize;
		int workGroupCount = (arraySize + workGroupSize - 1) / workGroupSize;
		range<1> workGroupSizeRange(workGroupSize);
		range<1> workGroupCountRange(workGroupCount);

		sycl::accessor buff0(src, handler);
		sycl::accessor scans(scansBuffer, handler);
		ndEvaluateKey evaluator;

		//sycl::stream out(1024, 256, handler);
		handler.parallel_for_work_group(workGroupCountRange, workGroupSizeRange, [=](group<1> group)
		{
			id<1> groupId = group.get_group_id();
			int base = groupId * workGroupSize;

			//out << "groupid:" << groupId << "   base:" << base << sycl::endl;
			//unsigned scanBuffer[256];
			group.parallel_for_work_item([&](h_item<1> item)
			{
				id<1> localId = item.get_local_id();
				int index = base + localId;
				scans[index] = 0;
			});

			if (groupId < (workGroupCount - 1))
			{
				group.parallel_for_work_item([&](h_item<1> item)
				{
					id<1> localId = item.get_local_id();
					int srcIndex = base + localId;
					int dstIndex = evaluator.GetCount(buff0[srcIndex]);
					atomic_ref<unsigned, memory_order::relaxed, memory_scope::work_item> atomicIndex(scans[base + dstIndex]);
					atomicIndex++;
				});
			}
			else
			{
				group.parallel_for_work_item([&](h_item<1> item)
				{
					id<1> localId = item.get_local_id();
					int srcIndex = base + localId;
					if (srcIndex < arraySize)
					{
						int dstIndex = evaluator.GetCount(buff0[srcIndex]);
						atomic_ref<unsigned, memory_order::relaxed, memory_scope::work_item> atomicIndex(scans[base + dstIndex]);
						atomicIndex++;
					}
				});

				//group.parallel_for_work_item([&](h_item<1> item)
				//{
				//	id<1> localId = item.get_local_id();
				//	//scanBuffer[localId] = 0;
				//	out << "index:" << localId << "   " << scans[localId] << sycl::endl;
				//});
			}
		});
	});
}


/*
__global__ void ndCudaCountingCellsPrefixScanInternal(unsigned* histogram, unsigned blockCount)
{
	unsigned sum = 0;
	unsigned offset = 0;
	const unsigned keySize = blockDim.x;

	const unsigned threadId = threadIdx.x;
	for (int i = 0; i < blockCount; i++)
	{
		const unsigned count = histogram[offset + threadId];
		histogram[offset + threadId] = sum;
		sum += count;
		offset += keySize;
	}
	histogram[offset + threadId] = sum;
}
*/
template <class T, class ndEvaluateKey, int bitSize>
void __CountingSortAddPrefix__(sycl::queue& queue, const buffer<T>& src, buffer<unsigned>& scansBuffer)
{
	queue.submit([&](auto& handler)
	{
		int arraySize = src.size();
		int workGroupSize = 1 << bitSize;
		int workGroupCount = (arraySize + workGroupSize - 1) / workGroupSize;
		accessor histogram(scansBuffer, handler);
		handler.parallel_for(workGroupSize, [=](id<1> item)
		{
			unsigned sum = 0;
			unsigned offset = 0;
			for (int i = 0; i < workGroupCount; ++i)
			{
				const unsigned count = histogram[offset + item];
				histogram[offset + item] = sum;
				sum += count;
				offset += workGroupSize;
			}
			histogram[offset + item] = sum;
		});
	});
}


/*
template <typename BufferItem, typename SortKeyPredicate>
__global__ void ndCudaCountingSortCountShuffleItemsInternal(const BufferItem* src, BufferItem* dst, unsigned* histogram, unsigned size, SortKeyPredicate GetSortKey, unsigned prefixKeySize)
{
	__shared__  BufferItem cachedCells[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
	__shared__  unsigned cacheSortedKey[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
	__shared__  unsigned cacheBaseOffset[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
	__shared__  unsigned cacheKeyPrefix[D_COUNTING_SORT_LOCAL_BLOCK_SIZE / 2 + D_COUNTING_SORT_LOCAL_BLOCK_SIZE + 1];
	__shared__  unsigned cacheItemCount[D_COUNTING_SORT_LOCAL_BLOCK_SIZE / 2 + D_COUNTING_SORT_LOCAL_BLOCK_SIZE + 1];

	const unsigned blockId = blockIdx.x;
	const unsigned threadId = threadIdx.x;
	const unsigned blocks = (size + blockDim.x - 1) / blockDim.x;

	const unsigned index = threadId + blockDim.x * blockId;
	cacheSortedKey[threadId] = (prefixKeySize << 16) | threadId;
	if (index < size)
	{
		cachedCells[threadId] = src[index];
		const unsigned key = GetSortKey(src[index]);
		cacheSortedKey[threadId] = (key << 16) | threadId;
	}

	const unsigned srcOffset = blockId * prefixKeySize;
	const unsigned lastRoadOffset = blocks * prefixKeySize;
	const unsigned prefixBase = D_COUNTING_SORT_LOCAL_BLOCK_SIZE / 2;

	cacheKeyPrefix[threadId] = 0;
	cacheItemCount[threadId] = 0;
	__syncthreads();

	if (threadId < prefixKeySize)
	{
		cacheBaseOffset[threadId] = histogram[srcOffset + threadId];
		cacheKeyPrefix[prefixBase + 1 + threadId] = histogram[lastRoadOffset + threadId];
		cacheItemCount[prefixBase + 1 + threadId] = histogram[srcOffset + prefixKeySize + threadId] - cacheBaseOffset[threadId];
	}

	const int threadId0 = threadId;
	for (int k = 2; k <= blockDim.x; k *= 2)
	{
		for (int j = k / 2; j > 0; j /= 2)
		{
			const int threadId1 = threadId0 ^ j;
			if (threadId1 > threadId0)
			{
				const int a = cacheSortedKey[threadId0];
				const int b = cacheSortedKey[threadId1];
				const int mask0 = (-(threadId0 & k)) >> 31;
				const int mask1 = -(a > b);
				const int mask = mask0 ^ mask1;
				cacheSortedKey[threadId0] = (b & mask) | (a & ~mask);
				cacheSortedKey[threadId1] = (a & mask) | (b & ~mask);
			}
			__syncthreads();
		}
	}

	for (int i = 1; i < prefixKeySize; i = i << 1)
	{
		const unsigned prefixSum = cacheKeyPrefix[prefixBase + threadId] + cacheKeyPrefix[prefixBase - i + threadId];
		const unsigned countSum = cacheItemCount[prefixBase + threadId] + cacheItemCount[prefixBase - i + threadId];
		__syncthreads();
		cacheKeyPrefix[prefixBase + threadId] = prefixSum;
		cacheItemCount[prefixBase + threadId] = countSum;
		__syncthreads();
	}

	if (index < size)
	{
		const unsigned keyValue = cacheSortedKey[threadId];
		const unsigned key = keyValue >> 16;
		const unsigned threadIdBase = cacheItemCount[prefixBase + key];

		const unsigned dstOffset0 = threadId - threadIdBase;
		const unsigned dstOffset1 = cacheKeyPrefix[prefixBase + key] + cacheBaseOffset[key];
		dst[dstOffset0 + dstOffset1] = cachedCells[keyValue & 0xffff];
	}
}
*/
template <class T, class ndEvaluateKey, int bitSize>
void __CountingSortMergeBuckects__(sycl::queue& queue, buffer<T>& src, buffer<T>& dst, buffer<unsigned>& scansBuffer)
{
	queue.submit([&](auto& handler)
	{
		int arraySize = src.size();
		int workGroupSize = 1 << bitSize;
		int workGroupCount = (arraySize + workGroupSize - 1) / workGroupSize;
		range<1> workGroupSizeRange(workGroupSize);
		range<1> workGroupCountRange(workGroupCount);

		range<1> xxxxx(D_COUNTING_SORT_LOCAL_BLOCK_SIZE);
		handler.parallel_for_work_group(workGroupCountRange, workGroupSizeRange, [=](group<1> group)
		{
			// make local shared memory buffers
			//sycl::local_accessor<unsigned, 1, sycl::access::mode::read_write, sycl::access::target::local> cacheSortedKey(D_COUNTING_SORT_LOCAL_BLOCK_SIZE, handler);
			
			//sycl::local_accessor<unsigned> cacheSortedKey(xxxxx, handler);
			//sycl::accessor<unsigned, 1, sycl::access::mode::read_write, sycl::access::target::local> cacheSortedKey(xxxxx, handler);
			//sycl::local_accessor<unsigned, 1> cacheSortedKey(xxxxx, handler);
			
			id<1> groupId = group.get_group_id();
			//int base = groupId * workGroupSize;
			//
			//group.parallel_for_work_item([&](h_item<1> item)
			//{
			//	//id<1> localId = item.get_local_id();
			//	//int index = base + localId;
			//	//scans[index] = 0;
			//});
		
			if (groupId < (workGroupCount - 1))
			{
				group.parallel_for_work_item([&](h_item<1> item)
				{
					//id<1> localId = item.get_local_id();
					//int srcIndex = base + localId;
					//int dstIndex = evaluator.GetCount(buff0[srcIndex]);
					//atomic_ref<unsigned, memory_order::relaxed, memory_scope::work_item> atomicIndex(scans[base + dstIndex]);
					//atomicIndex++;
				});
			}
			else
			{
				group.parallel_for_work_item([&](h_item<1> item)
				{
				//	id<1> localId = item.get_local_id();
				//	int srcIndex = base + localId;
				//	if (srcIndex < arraySize)
				//	{
				//		int dstIndex = evaluator.GetCount(buff0[srcIndex]);
				//		atomic_ref<unsigned, memory_order::relaxed, memory_scope::work_item> atomicIndex(scans[base + dstIndex]);
				//		atomicIndex++;
				//	}
				});
			}
		});
	});
}

template <class T, class ndEvaluateKey, int bitSize>
void ndCountingSort(sycl::queue& queue, buffer<T>& src, buffer<T>& dst, buffer<unsigned>& scansBuffer)
{
	__CountingSortCountItems__<T, ndEvaluateKey, bitSize>(queue, src, scansBuffer);
	__CountingSortAddPrefix__<T, ndEvaluateKey, bitSize>(queue, src, scansBuffer);
	__CountingSortMergeBuckects__<T, ndEvaluateKey, bitSize>(queue, src, dst, scansBuffer);
}

#endif
