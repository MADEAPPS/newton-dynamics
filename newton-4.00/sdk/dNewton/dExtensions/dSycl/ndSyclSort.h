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

//using namespace sycl;

#define D_COUNTING_SORT_LOCAL_BLOCK_SIZE	(1<<8)

template <class T, class ndEvaluateKey, int exponentRadix>
void ndCountingSort(const StlVector<T>& src, StlVector<T>& dst, StlVector<int>& scansBuffer);

template <class T, class ndEvaluateKey, int exponentRadix>
void ndCountingSort(sycl::queue& queue, sycl::buffer<T>& src, sycl::buffer<T>& dst, sycl::buffer<int>& scansBuffer);

// *****************************************************************
// 
// support functions implementation 
//
// *****************************************************************
template <class T, class ndEvaluateKey, int exponentRadix>
void SyclCountItems(sycl::queue& queue, sycl::buffer<T>& src, sycl::buffer<int>& scansBuffer);

template <class T, class ndEvaluateKey, int exponentRadix>
void SyclMergeBuckects(sycl::queue& queue, sycl::buffer<T>& src, sycl::buffer<T>& dst, sycl::buffer<int>& scansBuffer);

template <class T, class ndEvaluateKey, int exponentRadix>
void SyclAddPrefix(sycl::queue& queue, const sycl::buffer<T>& src, sycl::buffer<int>& scansBuffer);

template <class T, class ndEvaluateKey, int exponentRadix>
void ndCountingSort(const StlVector<T>& src, StlVector<T>& dst, StlVector<int>& scansBuffer)
{
	//ndAssert(0);
	auto CountItems = [&]()
	{
		ndEvaluateKey evaluator;
		int arraySize = src.size();
arraySize = 16;
		int workGroupSize = 1 << exponentRadix;
		int workGroupCount = (arraySize + workGroupSize - 1) / workGroupSize;

		auto CountItems = [&](int group, int item)
		{
			int base = group * workGroupSize;
			int radix = evaluator.GetRadix(src[base + item]);
			scansBuffer[base + radix] ++;
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
arraySize = 16;
		int workGroupSize = 1 << exponentRadix;
		int workGroupCount = (arraySize + workGroupSize - 1) / workGroupSize;

		const int prefixBase = workGroupSize / 2;
		const int lastWorkGroup = workGroupSize * workGroupCount;
		int localPrefixScan[D_COUNTING_SORT_LOCAL_BLOCK_SIZE / 2 + D_COUNTING_SORT_LOCAL_BLOCK_SIZE + 1];

		for (int group = 0; group < 1; ++group)
		{
			unsigned sumReg[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
			unsigned offsetReg[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
			for (int item = workGroupSize - 1; item >= 0; --item)
			{
				sumReg[item] = 0;
				offsetReg[item] = 0;
				localPrefixScan[item] = 0;
			}

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
				localPrefixScan[prefixBase + item + 1] = sumReg[item];
			}

			for (int i = 1; i < workGroupSize; i = i << 1)
			{
				int countSumReg[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
				for (int item = 0; item < workGroupSize; ++item)
				{
					countSumReg[item] = localPrefixScan[prefixBase + item] + localPrefixScan[prefixBase - i + item];
				}

				for (int item = 0; item < workGroupSize; ++item)
				{
					localPrefixScan[prefixBase + item] = countSumReg[item];
				}
			}

			for (int item = workGroupSize - 1; item >= 0; --item)
			{
				scansBuffer[lastWorkGroup + workGroupSize + item] = localPrefixScan[prefixBase + item];
			}
		}
	};

	auto MergeBuckects = [&]()
	{
		ndEvaluateKey evaluator;
		int arraySize = src.size();
arraySize = 16;
		int workGroupSize = 1 << exponentRadix;
		int workGroupCount = (arraySize + workGroupSize - 1) / workGroupSize;

		for (int group = workGroupCount - 1; group >= 0; --group)
		{
			int cacheSortedKey[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
			int cacheKeyPrefix[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
			int cacheBaseOffset[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
			int cacheItemCount[D_COUNTING_SORT_LOCAL_BLOCK_SIZE / 2 + D_COUNTING_SORT_LOCAL_BLOCK_SIZE + 1];

			int base = group * workGroupSize;
			int prefixBase = workGroupSize / 2;
			int lastRoadOffset = workGroupCount * workGroupSize;
			int start = (group < (workGroupCount - 1)) ? workGroupSize - 1 : arraySize - group * workGroupSize - 1;
			for (int item = workGroupSize - 1; item >= 0; --item)
			{
				cacheSortedKey[item] = (workGroupSize << 16) | item;
				cacheItemCount[item] = 0;
				cacheKeyPrefix[item] = scansBuffer[lastRoadOffset + workGroupSize + item];
			}

			for (int item = start; item >= 0; --item)
			{
				int index = base + item;
				int radix = evaluator.GetRadix(src[index]);
				cacheSortedKey[item] = (radix << 16) | item;

				cacheBaseOffset[item] = scansBuffer[base + item];
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
				}
			}
			//xxxx *= 1;
			
			for (int i = 1; i < workGroupSize; i = i << 1)
			{
				int countSumReg[D_COUNTING_SORT_LOCAL_BLOCK_SIZE];
				for (int item = 0; item < workGroupSize; ++item)
				{ 
					countSumReg[item] = cacheItemCount[prefixBase + item] + cacheItemCount[prefixBase - i + item];
				}

				for (int item = 0; item < workGroupSize; ++item)
				{ 
					cacheItemCount[prefixBase + item] = countSumReg[item];
				}
			}
			
			for (int item = start; item >= 0; --item)
			{
				int keyValue = cacheSortedKey[item];
				int keyHigh = keyValue >> 16;
				int keyLow = keyValue & 0xffff;
				int cacheItem = cacheItemCount[prefixBase + keyHigh];
				int dstOffset0 = item - cacheItem;
				int dstOffset1 = cacheKeyPrefix[keyHigh] + cacheBaseOffset[keyHigh];
				dst[dstOffset0 + dstOffset1] = src[base + keyLow];
			}
		}
	};

	CountItems();
	AddPrefix();
	MergeBuckects();
}

template <class T, class ndEvaluateKey, int exponentRadix>
void ndCountingSort(sycl::queue& queue, sycl::buffer<T>& src, sycl::buffer<T>& dst, sycl::buffer<int>& scansBuffer)
{
	SyclCountItems<T, ndEvaluateKey, exponentRadix>(queue, src, scansBuffer);
	SyclAddPrefix<T, ndEvaluateKey, exponentRadix>(queue, src, scansBuffer);
	SyclMergeBuckects<T, ndEvaluateKey, exponentRadix>(queue, src, dst, scansBuffer);
}

template <class T, class ndEvaluateKey, int exponentRadix>
void SyclAddPrefix(sycl::queue& queue, const sycl::buffer<T>& src, sycl::buffer<int>& scansBuffer)
{
	queue.submit([&](auto& handler)
	{
		int arraySize = src.size();
		arraySize = 16;
		int workGroupSize = 1 << exponentRadix;
		int workGroupCount = (arraySize + workGroupSize - 1) / workGroupSize;
		sycl::accessor scanAccessor(scansBuffer, handler);
		handler.parallel_for(workGroupSize, [=](sycl::id<1> item)
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
void SyclCountItems(sycl::queue& queue, sycl::buffer<T>& src, sycl::buffer<int>& scansBuffer)
{
	ndAssert((1 << exponentRadix) <= D_COUNTING_SORT_LOCAL_BLOCK_SIZE);
	queue.submit([&](sycl::handler& handler)
	{
#if 1
		ndEvaluateKey evaluator;
		int arraySize = src.size();
		arraySize = 16;
		int workGroupSize = 1 << exponentRadix;
		int workGroupCount = (arraySize + workGroupSize - 1) / workGroupSize;
		sycl::range<1> workGroupSizeRange(workGroupSize);
		sycl::range<1> workGroupCountRange(workGroupCount);

		sycl::accessor srcAccessor(src, handler);
		sycl::accessor scanAccessor(scansBuffer, handler);
		sycl::local_accessor<int, 1> counters(D_COUNTING_SORT_LOCAL_BLOCK_SIZE, handler);

		//sycl::stream out(1024, 256, handler);
		handler.parallel_for_work_group(workGroupCountRange, workGroupSizeRange, [=](sycl::group<1> group)
		{
			sycl::id<1> groupId = group.get_group_id();
			int base = groupId * workGroupSize;
			group.parallel_for_work_item([&](sycl::h_item<1> item)
			{
				sycl::id<1> localId = item.get_local_id();
				counters[localId] = 0;
			});

			//if (groupId < (workGroupCount - 1))
			if (1)
			{
				group.parallel_for_work_item([&](sycl::h_item<1> item)
				{
					sycl::id<1> localId = item.get_local_id();
					int srcIndex = base + localId;
					int radix = evaluator.GetRadix(srcAccessor[srcIndex]);
					sycl::atomic_ref<int, sycl::memory_order::relaxed, sycl::memory_scope::work_item> atomicRadix(counters[radix]);
					atomicRadix++;
				});
			}
			else
			{
				group.parallel_for_work_item([&](sycl::h_item<1> item)
				{
					sycl::id<1> localId = item.get_local_id();
					int srcIndex = base + localId;
					if (srcIndex < arraySize)
					{
						int radix = evaluator.GetRadix(srcAccessor[srcIndex]);
						sycl::atomic_ref<int, sycl::memory_order::relaxed, sycl::memory_scope::work_item> atomicRadix(counters[radix]);
						atomicRadix++;
					}
				});
			}

			group.parallel_for_work_item([&](sycl::h_item<1> item)
			{
				sycl::id<1> localId = item.get_local_id();
				scanAccessor[base + localId] = counters[localId];
			});
		});
#else

		//ndEvaluateKey evaluator;
		int arraySize = src.size();
		int workGroupSize = 1 << exponentRadix;
		int workGroupCount = (arraySize + workGroupSize - 1) / workGroupSize;

		sycl::accessor<T, 1> srcAccessor(src, handler);
		sycl::accessor<int, 1>  scanAccessor(scansBuffer, handler);
		sycl::local_accessor<int, 1> counters(D_COUNTING_SORT_LOCAL_BLOCK_SIZE, handler);

		sycl::stream out(1024, 256, handler);
		handler.parallel_for(sycl::nd_range<1>{ {workGroupCount}, {workGroupSize}}, [=](sycl::nd_item<1> item)
		{
			sycl::id<1> localId = item.get_local_id();
			counters[localId] = localId;
			item.barrier();
			
			sycl::id<1> flatId = item.get_global_id();
			scanAccessor[flatId] = counters[localId];

			sycl::id<1> globalId = item.get_global_id();
			out << "flatId: " << flatId << "group:" << globalId << "  local : " << localId << sycl::endl;
		});
#endif
	});
}

template <class T, class ndEvaluateKey, int exponentRadix>
void SyclMergeBuckects(sycl::queue& queue, sycl::buffer<T>& src, sycl::buffer<T>& dst, sycl::buffer<int>& scansBuffer)
{
	queue.submit([&](sycl::handler& handler)
	{
#if 0
		ndEvaluateKey evaluator;
		int arraySize = src.size();
		arraySize = 16;
		int workGroupSize = 1 << exponentRadix;
		int workGroupCount = (arraySize + workGroupSize - 1) / workGroupSize;
		sycl::range<1> workGroupSizeRange(workGroupSize);
		sycl::range<1> workGroupCountRange(workGroupCount);

		sycl::accessor srcAccessor(src, handler);
		sycl::accessor dstAccessor(dst, handler);
		sycl::accessor scanAccessor(scansBuffer, handler);

		sycl::local_accessor<int, 1> countSumReg(D_COUNTING_SORT_LOCAL_BLOCK_SIZE, handler);
		sycl::local_accessor<int, 1> prefixSumReg(D_COUNTING_SORT_LOCAL_BLOCK_SIZE, handler);
		sycl::local_accessor<int, 1> cacheSortedKey(D_COUNTING_SORT_LOCAL_BLOCK_SIZE, handler);
		sycl::local_accessor<int, 1> cacheBaseOffset(D_COUNTING_SORT_LOCAL_BLOCK_SIZE, handler);
		sycl::local_accessor<int, 1> cacheKeyPrefix(D_COUNTING_SORT_LOCAL_BLOCK_SIZE / 2 + D_COUNTING_SORT_LOCAL_BLOCK_SIZE + 1, handler);
		sycl::local_accessor<int, 1> cacheItemCount(D_COUNTING_SORT_LOCAL_BLOCK_SIZE / 2 + D_COUNTING_SORT_LOCAL_BLOCK_SIZE + 1, handler);

		sycl::stream out(4096, 256, handler);
		handler.parallel_for_work_group(workGroupCountRange, workGroupSizeRange, [=](sycl::group<1> group)
		{
			// make local shared memory buffers
			group.parallel_for_work_item([&](sycl::h_item<1> item)
			{
				int localId = item.get_local_id();
				cacheKeyPrefix[localId] = 0;
				cacheItemCount[localId] = 0;
				cacheSortedKey[localId] = (workGroupSize << 16) | localId;
			});

			sycl::id<1> groupId = group.get_group_id();
			int base = groupId * workGroupSize;
			int prefixBase = workGroupSize / 2;
			int lastRoadOffset = workGroupCount * workGroupSize;

			//if (groupId < (workGroupCount - 1))
			if (1)
			{
				//out << "groupid:" << groupId << "   base:" << base << sycl::endl;
				group.parallel_for_work_item([&](sycl::h_item<1> item)
				{
					int localId = item.get_local_id();
					int index = base + localId;
					int radix = evaluator.GetRadix(srcAccessor[index]);

					cacheSortedKey[localId] = (radix << 16) | localId;
					cacheBaseOffset[localId] = scanAccessor[base + localId];
					cacheKeyPrefix[prefixBase + 1 + localId] = scanAccessor[lastRoadOffset + localId];
					cacheItemCount[prefixBase + 1 + localId] = scanAccessor[base + workGroupSize + localId] - cacheBaseOffset[localId];
				});
			}
			else
			{
			
			}

			group.parallel_for_work_item([&](sycl::h_item<1> item)
			{
				int localId = item.get_local_id();
				for (int i = 1; i < workGroupSize; i = i << 1)
				{
					int countSum = cacheItemCount[prefixBase + localId] + cacheItemCount[prefixBase - i + localId];
					int prefixSum = cacheKeyPrefix[prefixBase + localId] + cacheKeyPrefix[prefixBase - i + localId];
					group.mem_fence();
					cacheItemCount[prefixBase + localId] = countSum;
					cacheKeyPrefix[prefixBase + localId] = prefixSum;
					group.mem_fence();
				}
				out << "group:" << group.get_group_id() << "  item:" << item.get_local_id() << "  index:" << cacheItemCount[prefixBase + localId] << sycl::endl;
			});

			//for (int i = 1; i < workGroupSize; i = i << 1)
			//{
			//	group.parallel_for_work_item([&](sycl::h_item<1> item)
			//	{
			//		int localId = item.get_local_id();
			//		countSumReg[localId] = cacheItemCount[prefixBase + localId] + cacheItemCount[prefixBase - i + localId];
			//		prefixSumReg[localId] = cacheKeyPrefix[prefixBase + localId] + cacheKeyPrefix[prefixBase - i + localId];
			//	});
			//
			//	group.parallel_for_work_item([&](sycl::h_item<1> item)
			//	{
			//		int localId = item.get_local_id();
			//		cacheItemCount[prefixBase + localId] = countSumReg[localId];
			//		cacheKeyPrefix[prefixBase + localId] = prefixSumReg[localId];
			//	});
			//}
			//group.parallel_for_work_item([&](sycl::h_item<1> item)
			//{
			//	int localId = item.get_local_id();
			//	//scanAccessor[base + localId] = cacheItemCount[prefixBase + localId];
			//	out << "group:" << group.get_group_id() << "  item:" << item.get_local_id() << "  index:" << cacheItemCount[prefixBase + localId] << sycl::endl;
			//	//out << group.get_group_id() << sycl::endl;
			//	//out << "item:" << sycl::endl;
			//});

			//if (groupId < (workGroupCount - 1))
			if (1)
			{
				group.parallel_for_work_item([&](sycl::h_item<1> item)
				{
					int localId = item.get_local_id();
					int keyValue = cacheSortedKey[localId];
					int keyHigh = keyValue >> 16;
					int keyLow = keyValue & 0xffff;
					int cacheItem = cacheItemCount[prefixBase + keyHigh];
					int dstOffset0 = localId - cacheItem;
					int dstOffset1 = cacheKeyPrefix[prefixBase + keyHigh] + cacheBaseOffset[keyHigh];
					dstAccessor[dstOffset0 + dstOffset1] = srcAccessor[base + keyLow];
				});
			}
			else
			{

			}
		});
#else
		sycl::accessor dstAccessor(dst, handler);
		//handler.parallel_for(sycl::nd_range<1>{ 4, 4}, [=](sycl::nd_item<1> it)
		//{
		//	int i = it.get_global_id(0);
		//	int j = it.get_local_id(0);
		//	dstAccessor[i * 4 + j] = i * 4 + j + 1;
		//});


		//cgh.parallel_for<example_kernel>(
		//	cl::sycl::nd_range<2>(range<2>{size >> 3, size >> 3},   // 8, 8
		//						  range<2>{size >> 4, size >> 4}),  // 4, 4
		//	[=](cl::sycl::nd_item<2> item) 
		//{
		//	// get the 2D x and y indices
		//	const auto id_x = item.get_global_id(0);
		//	const auto id_y = item.get_global_id(1);
		//	// map the 2D x and y indices to a single linear,
		//	// 1D (kernel space) index
		//	const auto width =
		//		item.get_group_range(0) * item.get_local_range(0);
		//	// map the 2D x and y indices to a single linear,
		//	// 1D (work-group) index
		//	const auto index = id_x * width + id_y;
		//	// compute A_ptr * B_ptr into C_scratch
		//	C_scratch[index] = A_ptr[index] * B_ptr[index];
		//	// wait for result to be written (sync local memory read_write)
		//	item.barrier(cl::sycl::access::fence_space::local_space);
		//	// output result computed in local memory
		//	C_ptr[index] = C_scratch[index];
		//});


		sycl::stream out(4096, 256, handler);


		//sycl::range<1> range(8);
		//handler.parallel_for(range, [=](sycl::id<1> id)
		//{
		//	out << id << sycl::endl;
		//});


		sycl::range<1> localSize(4);
		sycl::range<1> globalSize(2);
		handler.parallel_for(sycl::nd_range<1>(globalSize, localSize), [=](sycl::nd_item<1> item)
		{
			sycl::id<1> id_x = item.get_local_id(0);
			sycl::id<1> id_y = item.get_global_id(0);
			size_t width = item.get_local_range(0);
			size_t index = id_y * width + id_x;
			out << index << sycl::endl;
		});
#endif
	});

}

#endif
