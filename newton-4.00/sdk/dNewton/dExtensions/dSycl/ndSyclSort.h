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

template <class T, class ndEvaluateKey, int bitSize>
void ndCountingSort(ndSyclContextImpl* const context, buffer<T>& src, buffer<T>& dst)
{
	int arraySize = src.size();
	int workGroupSize = 1 << bitSize;
	int workGroupCount = (arraySize + workGroupSize - 1) / workGroupSize;
	range<1> workGroupSizeRange(workGroupSize);
	range<1> workGroupCountRange(workGroupCount);

	buffer<unsigned>& scansBuffer = context->m_sortPrefixBuffer;
	ndAssert(scansBuffer.size() > arraySize);

	context->m_queue.submit([&](auto& handler)
	{
		accessor buff0(src, handler);
		accessor buff1(dst, handler);
		accessor scans(scansBuffer, handler);
		ndEvaluateKey evaluator;

		//sycl::stream out(1024, 256, handler);
		handler.parallel_for_work_group(workGroupCountRange, workGroupSizeRange, [=](group<1> group)
		{
			id<1> groupId = group.get_group_id();
			int base = groupId * workGroupSize;

			unsigned scanBuffer[256];
			group.parallel_for_work_item([&](h_item<1> item)
			{
				id<1> localId = item.get_local_id();
				int index = base + localId;
				scans[index] = 0;
				scanBuffer[localId] = 0;
				//out << "index:" << item << "   " << buff0[item] << sycl::endl;
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
			}
		});
	});

	//context->m_queue.submit([&](auto& handler)
	//{
	//	accessor buff1(dst, handler);
	//	accessor buff0(scansBuffer, handler);
	//
	//	sycl::stream out(1024, 256, handler);
	//	handler.parallel_for(arraySize, [=](id<1> item) 
	//	{
	//		out << "index:" << item << "   " << buff0[item] << sycl::endl;
	//		buff1[item] = buff0[item];
	//	});
	//});
}

#endif
