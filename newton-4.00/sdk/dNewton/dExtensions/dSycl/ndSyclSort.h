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

#include "ndStlContainers.h"
#include "ndSyclContextImpl.h"

using namespace sycl;

template <class T, class ndEvaluateKey, int expontentRadix>
void ndCountingSort(const StlVector<T>& src, StlVector<T>& dst, StlVector<unsigned>& scansBuffer);

template <class T, class ndEvaluateKey, int expontentRadix>
void ndCountingSort(sycl::queue& queue, buffer<T>& src, buffer<T>& dst, buffer<unsigned>& scansBuffer);


// impemention support
#define D_COUNTING_SORT_LOCAL_BLOCK_SIZE	(1<<10)

template <class T, class ndEvaluateKey, int expontentRadix>
void SyclCountItems(sycl::queue& queue, buffer<T>& src, buffer<unsigned>& scansBuffer);

template <class T, class ndEvaluateKey, int expontentRadix>
void SyclMergeBuckects(sycl::queue& queue, buffer<T>& src, buffer<T>& dst, buffer<unsigned>& scansBuffer);

template <class T, class ndEvaluateKey, int expontentRadix>
void SyclAddPrefix(sycl::queue& queue, const buffer<T>& src, buffer<unsigned>& scansBuffer);

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
template <class T, class ndEvaluateKey, int expontentRadix>
void SyclCountItems(sycl::queue& queue, buffer<T>& src, buffer<unsigned>& scansBuffer)
{
	ndAssert((1 << bitSize) <= D_COUNTING_SORT_LOCAL_BLOCK_SIZE);
	queue.submit([&](auto& handler)
	{
		int arraySize = src.size();
		int workGroupSize = 1 << expontentRadix;
		int workGroupCount = (arraySize + workGroupSize - 1) / workGroupSize;
		range<1> workGroupSizeRange(workGroupSize);
		range<1> workGroupCountRange(workGroupCount);

		sycl::accessor srcAccessor(src, handler);
		sycl::accessor scanAccessor(scansBuffer, handler);
		ndEvaluateKey evaluator;

		//sycl::stream out(1024, 256, handler);
		handler.parallel_for_work_group(workGroupCountRange, workGroupSizeRange, [=](group<1> group)
		{
			id<1> groupId = group.get_group_id();
			int base = groupId * workGroupSize;

			//out << "groupid:" << groupId << "   stride:" << workGroupSizeRange << sycl::endl;
			unsigned scanBuffer[256];
			group.parallel_for_work_item([&](h_item<1> item)
			{
				id<1> localId = item.get_local_id();
				int index = base + localId;
				scanAccessor[index] = 0;
				scanBuffer[localId] = localId;
				//if (groupId == 0 && localId > 25)
				//out << "group:" << groupId << "  local:" << localId << "  value : " << scanBuffer[localId] << sycl::endl;
			});

			if (groupId < (workGroupCount - 1))
			{
				group.parallel_for_work_item([&](h_item<1> item)
				{
					id<1> localId = item.get_local_id();
					int srcIndex = base + localId;
					int dstIndex = evaluator.GetCount(srcAccessor[srcIndex]);
					atomic_ref<unsigned, memory_order::relaxed, memory_scope::work_item> atomicIndex(scanAccessor[base + dstIndex]);
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
						int dstIndex = evaluator.GetCount(srcAccessor[srcIndex]);
						atomic_ref<unsigned, memory_order::relaxed, memory_scope::work_item> atomicIndex(scanAccessor[base + dstIndex]);
						atomicIndex++;
					}
				});
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
template <class T, class ndEvaluateKey, int expontentRadix>
void SyclAddPrefix(sycl::queue& queue, const buffer<T>& src, buffer<unsigned>& scansBuffer)
{
	queue.submit([&](auto& handler)
	{
		int arraySize = src.size();
		int workGroupSize = 1 << expontentRadix;
		int workGroupCount = (arraySize + workGroupSize - 1) / workGroupSize;
		accessor scanAccessor(scansBuffer, handler);
		handler.parallel_for(workGroupSize, [=](id<1> item)
		{
			unsigned sum = 0;
			unsigned offset = 0;
			for (int i = 0; i < workGroupCount; ++i)
			{
				const unsigned count = scanAccessor[offset + item];
				scanAccessor[offset + item] = sum;
				sum += count;
				offset += workGroupSize;
			}
			scanAccessor[offset + item] = sum;
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
template <class T, class ndEvaluateKey, int expontentRadix>
void SyclMergeBuckects(sycl::queue& queue, buffer<T>& src, buffer<T>& dst, buffer<unsigned>& scansBuffer)
{
	ndAssert((1 << bitSize) <= D_COUNTING_SORT_LOCAL_BLOCK_SIZE);
	queue.submit([&](auto& handler)
	{
		int arraySize = src.size();
		int workGroupSize = 1 << expontentRadix;
		int workGroupCount = (arraySize + workGroupSize - 1) / workGroupSize;
		range<1> workGroupSizeRange(workGroupSize);
		range<1> workGroupCountRange(workGroupCount);

		ndEvaluateKey evaluator;
		sycl::accessor srcAccessor(src, handler);
		sycl::accessor scanAccessor(scansBuffer, handler);

		handler.parallel_for_work_group(workGroupCountRange, workGroupSizeRange, [=](group<1> group)
		{
			// make local shared memory buffers
			T cachedCells[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
			unsigned cacheSortedKey[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
			unsigned cacheBaseOffset[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
			unsigned cacheKeyPrefix[D_COUNTING_SORT_LOCAL_BLOCK_SIZE / 2 + D_COUNTING_SORT_LOCAL_BLOCK_SIZE + 1];
			unsigned cacheItemCount[D_COUNTING_SORT_LOCAL_BLOCK_SIZE / 2 + D_COUNTING_SORT_LOCAL_BLOCK_SIZE + 1];

			id<1> groupId = group.get_group_id();
			int base = groupId * workGroupSize;

			const unsigned srcOffset = groupId * workGroupSize;
			const unsigned lastRoadOffset = workGroupCount * workGroupSize;
			const unsigned prefixBase = D_COUNTING_SORT_LOCAL_BLOCK_SIZE / 2;

			if (groupId < (workGroupCount - 1))
			{
				group.parallel_for_work_item([&](h_item<1> item)
				{
					unsigned localId = item.get_local_id();
					unsigned index = base + localId;
					cachedCells[localId] = srcAccessor[index];
					unsigned key = evaluator.GetCount(srcAccessor[index]);
					cacheSortedKey[localId] = (key << 16) | localId;
					cacheKeyPrefix[localId] = 0;
					cacheItemCount[localId] = 0;
				});

				group.parallel_for_work_item([&](h_item<1> item)
				{
					unsigned localId = item.get_local_id();
					cacheBaseOffset[localId] = scanAccessor[srcOffset + localId];
					cacheKeyPrefix[prefixBase + 1 + localId] = scanAccessor[lastRoadOffset + localId];
					cacheItemCount[prefixBase + 1 + localId] = scanAccessor[srcOffset + workGroupSize + localId] - cacheBaseOffset[localId];
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

template <class T, class ndEvaluateKey, int expontentRadix>
void ndCountingSort(sycl::queue& queue, buffer<T>& src, buffer<T>& dst, buffer<unsigned>& scansBuffer)
{
	SyclCountItems<T, ndEvaluateKey, expontentRadix>(queue, src, scansBuffer);
	SyclAddPrefix<T, ndEvaluateKey, expontentRadix>(queue, src, scansBuffer);
	SyclMergeBuckects<T, ndEvaluateKey, expontentRadix>(queue, src, dst, scansBuffer);
}

template <class T, class ndEvaluateKey, int expontentRadix>
void ndCountingSort(const StlVector<T>& src, StlVector<T>& dst, StlVector<unsigned>& scansBuffer)
{
	//ndAssert(0);
	auto CountItems = [&]()
	{
		ndEvaluateKey evaluator;
		int arraySize = src.size();
		int workGroupSize = 1 << expontentRadix;
		int workGroupCount = (arraySize + workGroupSize - 1) / workGroupSize;

		auto CountItems = [&](int group, int item)
		{
			int base = group * workGroupSize;
			int key = evaluator.GetCount(src[base + item]);
			scansBuffer[base + key] ++;
		};

		//for (int group = workGroupCount - 1; group >= 0; --group)
		for (int group = 0; group >= 0; --group)
		{
			for (int item = workGroupSize - 1; item >= 0; --item)
			{
				int base = group * workGroupSize;
				scansBuffer[base + item] = 0;;
			}

			int start = (group < (workGroupCount - 1)) ? workGroupSize - 1 : arraySize - group * workGroupSize - 1;
			for (int item = start; item >= 0; --item)
			{
				CountItems(group, item);
			}
		}
	};

	auto AddPrefix = [&]()
	{
		int arraySize = src.size();
		int workGroupSize = 1 << expontentRadix;
		int workGroupCount = (arraySize + workGroupSize - 1) / workGroupSize;
		workGroupCount = 1;

		unsigned sumReg[256];
		unsigned offsetReg[256];
		for (int group = 0; group < workGroupCount; ++group)
		{
			for (int item = workGroupSize - 1; item >= 0; --item)
			{
				sumReg[item] = 0;
				offsetReg[item] = 0;
			}
			for (int item = workGroupSize - 1; item >= 0; --item)
			{
				unsigned sum = sumReg[item];
				unsigned offset = offsetReg[item];
				unsigned count = scansBuffer[offset + item];
				scansBuffer[offset + item] = sum;
				sumReg[item] = sum + count;
				offsetReg[item] = offset + workGroupSize;
			}
			for (int item = workGroupSize - 1; item >= 0; --item)
			{
				unsigned sum = sumReg[item];
				unsigned offset = offsetReg[item];
				scansBuffer[offset + item] = sum;
			}
		}
	};

	auto MergeBuckects = [&]()
	{
	};

	CountItems();
	AddPrefix();
	MergeBuckects();
}

#endif
