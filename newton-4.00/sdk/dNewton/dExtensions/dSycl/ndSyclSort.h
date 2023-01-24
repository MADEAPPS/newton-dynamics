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

#define D_COUNTING_SORT_LOCAL_BLOCK_SIZE	(1<<8)

template <class T, class ndEvaluateKey, int exponentRadix>
void ndCountingSort(const StlVector<T>& src, StlVector<T>& dst, StlVector<unsigned>& scansBuffer);

template <class T, class ndEvaluateKey, int exponentRadix>
void ndCountingSort(sycl::queue& queue, buffer<T>& src, buffer<T>& dst, buffer<unsigned>& scansBuffer);

// implementation support functions
template <class T, class ndEvaluateKey, int exponentRadix>
void SyclCountItems(sycl::queue& queue, buffer<T>& src, buffer<unsigned>& scansBuffer);

template <class T, class ndEvaluateKey, int exponentRadix>
void SyclMergeBuckects(sycl::queue& queue, buffer<T>& src, buffer<T>& dst, buffer<unsigned>& scansBuffer);

template <class T, class ndEvaluateKey, int exponentRadix>
void SyclAddPrefix(sycl::queue& queue, const buffer<T>& src, buffer<unsigned>& scansBuffer);

template <class T, class ndEvaluateKey, int exponentRadix>
void ndCountingSort(const StlVector<T>& src, StlVector<T>& dst, StlVector<unsigned>& scansBuffer)
{
	//ndAssert(0);
	auto CountItems = [&]()
	{
		ndEvaluateKey evaluator;
		int arraySize = src.size();
		int workGroupSize = 1 << exponentRadix;
		int workGroupCount = (arraySize + workGroupSize - 1) / workGroupSize;

		auto CountItems = [&](int group, int item)
		{
			int base = group * workGroupSize;
			int key = evaluator.GetCount(src[base + item]);
			scansBuffer[base + key] ++;
		};

		for (int group = workGroupCount - 1; group >= 0; --group)
		{
			for (int item = workGroupSize - 1; item >= 0; --item)
			{
				int base = group * workGroupSize;
				scansBuffer[base + item] = 0;
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
		int workGroupSize = 1 << exponentRadix;

		for (int group = 0; group < 1; ++group)
		{
			unsigned sumReg[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
			unsigned offsetReg[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
			for (int item = workGroupSize - 1; item >= 0; --item)
			{
				sumReg[item] = 0;
				offsetReg[item] = 0;
			}

			int workGroupCount = (arraySize + workGroupSize - 1) / workGroupSize;
			for (int i = 0; i < workGroupCount; ++i)
			{
				for (int item = workGroupSize - 1; item >= 0; --item)
				{
					unsigned sum = sumReg[item];
					unsigned offset = offsetReg[item];
					unsigned count = scansBuffer[offset + item];
					scansBuffer[offset + item] = sum;
					sumReg[item] = sum + count;
					offsetReg[item] = offset + workGroupSize;
				}
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
		ndEvaluateKey evaluator;
		int arraySize = src.size();
		int workGroupSize = 1 << exponentRadix;
		int workGroupCount = (arraySize + workGroupSize - 1) / workGroupSize;

		for (int group = workGroupCount - 1; group >= 0; --group)
		{
			int cacheSortedKey[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
			int cacheBaseOffset[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
			int cacheKeyPrefix[D_COUNTING_SORT_LOCAL_BLOCK_SIZE / 2 + D_COUNTING_SORT_LOCAL_BLOCK_SIZE + 1];
			int cacheItemCount[D_COUNTING_SORT_LOCAL_BLOCK_SIZE / 2 + D_COUNTING_SORT_LOCAL_BLOCK_SIZE + 1];

			int base = group * workGroupSize;
			int prefixBase = workGroupSize / 2;
			int lastRoadOffset = workGroupCount * workGroupSize;
			int start = (group < (workGroupCount - 1)) ? workGroupSize - 1 : arraySize - group * workGroupSize - 1;
			for (int item = workGroupSize - 1; item >= 0; --item)
			{
				cacheSortedKey[item] = (workGroupSize << 16) | item;
				cacheKeyPrefix[item] = 0;
				cacheItemCount[item] = 0;
			}

			for (int item = start; item >= 0; --item)
			{
				int index = base + item;
				int key = evaluator.GetCount(src[index]);
				cacheSortedKey[item] = (key << 16) | item;

				cacheBaseOffset[item] = scansBuffer[base + item];
				cacheKeyPrefix[prefixBase + 1 + item] = scansBuffer[lastRoadOffset + item];
				cacheItemCount[prefixBase + 1 + item] = scansBuffer[base + workGroupSize + item] - cacheBaseOffset[item];
			}

			// k is doubled every iteration
			//for (k = 2; k <= n; k *= 2) 
			//	// j is halved at every iteration, with truncation of fractional parts
			//	for (j = k / 2; j > 0; j /= 2)
			//		// in C-like languages this is "i ^ j"
			//		for (i = 0; i < n; i++)
			//			l = bitwiseXOR(i, j); 
			//			if (l > i)
			//				if ((bitwiseAND(i, k) == 0) AND(arr[i] > arr[l])
			//				OR(bitwiseAND(i, k) != 0) AND(arr[i] < arr[l]))
			//				swap the elements arr[i] and arr[l]

			//int xxxx = 0;
			for (int k = 2; k <= workGroupSize; k *= 2)
			{
				for (int j = k / 2; j > 0; j /= 2)
				{
					//xxxx++;
					for (int item = 0; item < workGroupSize; ++item)
					{
						int threadId0 = item;
						int threadId1 = threadId0 ^ j;
						if (threadId1 > threadId0)
						{
							int a = cacheSortedKey[threadId0];
							int b = cacheSortedKey[threadId1];
							int mask0 = (-(threadId0 & k)) >> 31;
							int mask1 = -(a > b);
							int mask = mask0 ^ mask1;
							cacheSortedKey[threadId0] = (b & mask) | (a & ~mask);
							cacheSortedKey[threadId1] = (a & mask) | (b & ~mask);
						}
					}
					//__syncthreads();
				}
			}
			//xxxx *= 1;
			
			for (int i = 1; i < workGroupSize; i = i << 1)
			{
				int countSumReg[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
				int prefixSumReg[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
				for (int item = 0; item < workGroupSize; ++item)
				{ 
					prefixSumReg[item] = cacheKeyPrefix[prefixBase + item] + cacheKeyPrefix[prefixBase - i + item];
					countSumReg[item] = cacheItemCount[prefixBase + item] + cacheItemCount[prefixBase - i + item];
				}
				//__syncthreads();
				for (int item = 0; item < workGroupSize; ++item)
				{ 
					cacheKeyPrefix[prefixBase + item] = prefixSumReg[item];
					cacheItemCount[prefixBase + item] = countSumReg[item];
				}
				//__syncthreads();
			}
			
			for (int item = start; item >= 0; --item)
			{
				int keyValue = cacheSortedKey[item];
				int keyHigh = keyValue >> 16;
				int keyLow = keyValue & 0xffff;
				int cacheItem = cacheItemCount[prefixBase + keyHigh];
				int dstOffset0 = item - cacheItem;
				int dstOffset1 = cacheKeyPrefix[prefixBase + keyHigh] + cacheBaseOffset[keyHigh];
				dst[dstOffset0 + dstOffset1] = src[base + keyLow];
			}
		}
	};

	CountItems();
	AddPrefix();
	MergeBuckects();
}

template <class T, class ndEvaluateKey, int exponentRadix>
void ndCountingSort(sycl::queue& queue, buffer<T>& src, buffer<T>& dst, buffer<unsigned>& scansBuffer)
{
	SyclCountItems<T, ndEvaluateKey, exponentRadix>(queue, src, scansBuffer);
	SyclAddPrefix<T, ndEvaluateKey, exponentRadix>(queue, src, scansBuffer);
	SyclMergeBuckects<T, ndEvaluateKey, exponentRadix>(queue, src, dst, scansBuffer);
}

template <class T, class ndEvaluateKey, int exponentRadix>
void SyclCountItems(sycl::queue& queue, buffer<T>& src, buffer<unsigned>& scansBuffer)
{
	ndAssert((1 << exponentRadix) <= D_COUNTING_SORT_LOCAL_BLOCK_SIZE);
	queue.submit([&](auto& handler)
	{
		ndEvaluateKey evaluator;
		int arraySize = src.size();
		int workGroupSize = 1 << exponentRadix;
		int workGroupCount = (arraySize + workGroupSize - 1) / workGroupSize;
		range<1> workGroupSizeRange(workGroupSize);
		range<1> workGroupCountRange(workGroupCount);
		
		sycl::accessor srcAccessor(src, handler);
		sycl::accessor scanAccessor(scansBuffer, handler);

		//sycl::stream out(1024, 256, handler);
		handler.parallel_for_work_group(workGroupCountRange, workGroupSizeRange, [=](group<1> group)
		{
			id<1> groupId = group.get_group_id();
			int base = groupId * workGroupSize;
	
			//out << "groupid:" << groupId << "   stride:" << workGroupSizeRange << sycl::endl;
			int scanLocalBuffer[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
			group.parallel_for_work_item([&](h_item<1> item)
			{
				id<1> localId = item.get_local_id();
				scanLocalBuffer[localId] = 0;
				//if (groupId == 0 && localId > 25)
				//out << "group:" << groupId << "  local:" << localId << "  value : " << scanBuffer[localId] << sycl::endl;
			});
		
			if (groupId < (workGroupCount - 1))
			{
				group.parallel_for_work_item([&](h_item<1> item)
				{
					id<1> localId = item.get_local_id();
					int srcIndex = base + localId;
					int scanIndex = evaluator.GetCount(srcAccessor[srcIndex]);
					sycl::atomic_ref<int, memory_order::relaxed, memory_scope::work_item> atomicIndex(scanLocalBuffer[scanIndex]);
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
						int scanIndex = evaluator.GetCount(srcAccessor[srcIndex]);
						sycl::atomic_ref<int, memory_order::relaxed, memory_scope::work_item> atomicIndex(scanLocalBuffer[scanIndex]);
						atomicIndex++;
					}
				});
			}

			group.parallel_for_work_item([&](h_item<1> item)
			{
				id<1> localId = item.get_local_id();
				scanAccessor[base + localId] = scanLocalBuffer[localId];
			});
		});
	});
}

template <class T, class ndEvaluateKey, int exponentRadix>
void SyclAddPrefix(sycl::queue& queue, const buffer<T>& src, buffer<unsigned>& scansBuffer)
{
	queue.submit([&](auto& handler)
	{
		int arraySize = src.size();
		int workGroupSize = 1 << exponentRadix;
		int workGroupCount = (arraySize + workGroupSize - 1) / workGroupSize;
		accessor scanAccessor(scansBuffer, handler);
		handler.parallel_for(workGroupSize, [=](id<1> item)
		{
			int sum = 0;
			int offset = 0;
			for (int i = 0; i < workGroupCount; ++i)
			{
				int count = scanAccessor[offset + item];
				scanAccessor[offset + item] = sum;
				sum += count;
				offset += workGroupSize;
			}
			scanAccessor[offset + item] = sum;
		});
	});
}

template <class T, class ndEvaluateKey, int exponentRadix>
void SyclMergeBuckects(sycl::queue& queue, buffer<T>& src, buffer<T>& dst, buffer<unsigned>& scansBuffer)
{
	//ndEvaluateKey evaluator;
	//int arraySize = src.size();
	//int workGroupSize = 1 << exponentRadix;
	//int workGroupCount = (arraySize + workGroupSize - 1) / workGroupSize;
	//
	//for (int group = workGroupCount - 1; group >= 0; --group)
	//{
	//	int cacheSortedKey[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
	//	int cacheBaseOffset[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
	//	int cacheKeyPrefix[D_COUNTING_SORT_LOCAL_BLOCK_SIZE / 2 + D_COUNTING_SORT_LOCAL_BLOCK_SIZE + 1];
	//	int cacheItemCount[D_COUNTING_SORT_LOCAL_BLOCK_SIZE / 2 + D_COUNTING_SORT_LOCAL_BLOCK_SIZE + 1];
	//
	//	int base = group * workGroupSize;
	//	int prefixBase = workGroupSize / 2;
	//	int lastRoadOffset = workGroupCount * workGroupSize;
	//	int start = (group < (workGroupCount - 1)) ? workGroupSize - 1 : arraySize - group * workGroupSize - 1;
	//	for (int item = workGroupSize - 1; item >= 0; --item)
	//	{
	//		cacheSortedKey[item] = (workGroupSize << 16) | item;
	//		cacheKeyPrefix[item] = 0;
	//		cacheItemCount[item] = 0;
	//	}
	//
	//	for (int item = start; item >= 0; --item)
	//	{
	//		int index = base + item;
	//		int key = evaluator.GetCount(src[index]);
	//		cacheSortedKey[item] = (key << 16) | item;
	//
	//		cacheBaseOffset[item] = scansBuffer[base + item];
	//		cacheKeyPrefix[prefixBase + 1 + item] = scansBuffer[lastRoadOffset + item];
	//		cacheItemCount[prefixBase + 1 + item] = scansBuffer[base + workGroupSize + item] - cacheBaseOffset[item];
	//	}
	//
	//	for (int k = 2; k <= workGroupSize; k *= 2)
	//	{
	//		for (int j = k / 2; j > 0; j /= 2)
	//		{
	//			for (int item = 0; item < workGroupSize; ++item)
	//			{
	//				int threadId0 = item;
	//				int threadId1 = threadId0 ^ j;
	//				if (threadId1 > threadId0)
	//				{
	//					int a = cacheSortedKey[threadId0];
	//					int b = cacheSortedKey[threadId1];
	//					int mask0 = (-(threadId0 & k)) >> 31;
	//					int mask1 = -(a > b);
	//					int mask = mask0 ^ mask1;
	//					cacheSortedKey[threadId0] = (b & mask) | (a & ~mask);
	//					cacheSortedKey[threadId1] = (a & mask) | (b & ~mask);
	//				}
	//			}
	//			//__syncthreads();
	//		}
	//	}
	//
	//	for (int i = 1; i < workGroupSize; i = i << 1)
	//	{
	//		int countSumReg[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
	//		int prefixSumReg[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
	//		for (int item = 0; item < workGroupSize; ++item)
	//		{
	//			prefixSumReg[item] = cacheKeyPrefix[prefixBase + item] + cacheKeyPrefix[prefixBase - i + item];
	//			countSumReg[item] = cacheItemCount[prefixBase + item] + cacheItemCount[prefixBase - i + item];
	//		}
	//		//__syncthreads();
	//		for (int item = 0; item < workGroupSize; ++item)
	//		{
	//			cacheKeyPrefix[prefixBase + item] = prefixSumReg[item];
	//			cacheItemCount[prefixBase + item] = countSumReg[item];
	//		}
	//		//__syncthreads();
	//	}
	//
	//	for (int item = start; item >= 0; --item)
	//	{
	//		int keyValue = cacheSortedKey[item];
	//		int keyHigh = keyValue >> 16;
	//		int keyLow = keyValue & 0xffff;
	//		int cacheItem = cacheItemCount[prefixBase + keyHigh];
	//		int dstOffset0 = item - cacheItem;
	//		int dstOffset1 = cacheKeyPrefix[prefixBase + keyHigh] + cacheBaseOffset[keyHigh];
	//		dst[dstOffset0 + dstOffset1] = src[base + keyLow];
	//	}
	//}

	queue.submit([&](auto& handler)
	{
		ndEvaluateKey evaluator;
		int arraySize = src.size();
		int workGroupSize = 1 << exponentRadix;
		int workGroupCount = (arraySize + workGroupSize - 1) / workGroupSize;
		range<1> workGroupSizeRange(workGroupSize);
		range<1> workGroupCountRange(workGroupCount);

		sycl::accessor srcAccessor(src, handler);
		sycl::accessor dstAccessor(dst, handler);
		sycl::accessor scanAccessor(scansBuffer, handler);

		//sycl::stream out(1024, 256, handler);
		handler.parallel_for_work_group(workGroupCountRange, workGroupSizeRange, [=](group<1> group)
		{
			// make local shared memory buffers
			//ndEvaluateKey evaluator;
			int cacheSortedKey[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
			int cacheBaseOffset[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
			int cacheKeyPrefix[D_COUNTING_SORT_LOCAL_BLOCK_SIZE / 2 + D_COUNTING_SORT_LOCAL_BLOCK_SIZE + 1];
			int cacheItemCount[D_COUNTING_SORT_LOCAL_BLOCK_SIZE / 2 + D_COUNTING_SORT_LOCAL_BLOCK_SIZE + 1];

			group.parallel_for_work_item([&](h_item<1> item)
			{
				int localId = item.get_local_id();
				cacheSortedKey[localId] = (workGroupSize << 16) | localId;
				cacheKeyPrefix[localId] = 0;
				cacheItemCount[localId] = 0;
			});

			id<1> groupId = group.get_group_id();
			if (groupId < (workGroupCount - 1))
			{
				//out << "groupid:" << groupId << "   base:" << base << sycl::endl;
				group.parallel_for_work_item([&](h_item<1> item)
				{
					int base = groupId * workGroupSize;
					int prefixBase = workGroupSize / 2;
					int lastRoadOffset = workGroupCount * workGroupSize;

					int localId = item.get_local_id();
					int index = base + localId;
					int key = evaluator.GetCount(srcAccessor[index]);
					cacheSortedKey[localId] = (key << 16) | localId;
				
					cacheBaseOffset[localId] = scanAccessor[base + localId];
					cacheKeyPrefix[prefixBase + 1 + localId] = scanAccessor[lastRoadOffset + localId];
					cacheItemCount[prefixBase + 1 + localId] = scanAccessor[base + workGroupSize + localId] - cacheBaseOffset[localId];
				});
			}
			else
			{

			}
		});
	});
}

#endif
