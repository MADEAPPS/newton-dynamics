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

#define D_COUNTING_SORT_BLOCK_SIZE	(1<<8)

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
void ndCountingSort(sycl::queue& queue, sycl::buffer<T>& src, sycl::buffer<T>& dst, sycl::buffer<int>& scansBuffer)
{
	//SyclCountItems<T, ndEvaluateKey, exponentRadix>(queue, src, scansBuffer);
	//SyclAddPrefix<T, ndEvaluateKey, exponentRadix>(queue, src, scansBuffer);
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
	ndAssert((1 << exponentRadix) <= D_COUNTING_SORT_BLOCK_SIZE);
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
		sycl::local_accessor<int, 1> counters(D_COUNTING_SORT_BLOCK_SIZE, handler);

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
		sycl::local_accessor<int, 1> counters(D_COUNTING_SORT_BLOCK_SIZE, handler);

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

		sycl::local_accessor<int, 1> countSumReg(D_COUNTING_SORT_BLOCK_SIZE, handler);
		sycl::local_accessor<int, 1> prefixSumReg(D_COUNTING_SORT_BLOCK_SIZE, handler);
		sycl::local_accessor<int, 1> cacheSortedKey(D_COUNTING_SORT_BLOCK_SIZE, handler);
		sycl::local_accessor<int, 1> cacheBaseOffset(D_COUNTING_SORT_BLOCK_SIZE, handler);
		sycl::local_accessor<int, 1> cacheKeyPrefix(D_COUNTING_SORT_BLOCK_SIZE / 2 + D_COUNTING_SORT_BLOCK_SIZE + 1, handler);
		sycl::local_accessor<int, 1> cacheItemCount(D_COUNTING_SORT_BLOCK_SIZE / 2 + D_COUNTING_SORT_BLOCK_SIZE + 1, handler);

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
		//sycl::accessor dstAccessor(dst, handler);

		sycl::stream out(64 * 1024, 80, handler);
		//handler.parallel_for(sycl::nd_range<3>(sycl::range<3>(1, 1, 2)* sycl::range<3>(1, 1, 4), sycl::range<3>(1, 1, 4)), [=](sycl::nd_item<3> item1)
		//{
		//	//ndCudaMerge(item1, out);
		//	const unsigned threadId = item1.get_local_id(2);
		//	const unsigned index = threadId + item1.get_local_range(2) * item1.get_group(2);
		//
		//	out << "DPCPP " << index << "\n";
		//});

		handler.parallel_for(sycl::nd_range<1>(sycl::range<1>(2 * 4), sycl::range<1>(4)), [=](sycl::nd_item<1> item1)
		{
			const unsigned threadId = item1.get_local_id(0);
			const unsigned index = threadId + item1.get_local_range(0) * item1.get_group(0);
			out << "DPCPP " << index << "\n";
		});

#endif
	});

}

#endif
