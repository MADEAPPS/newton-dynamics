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

#include "ndCoreStdafx.h"
#include "ndArray.h"
#include "ndProfiler.h"
#include "ndThreadPool.h"

template <class T, class dCompareKey>
void ndSort(T* const array, ndInt32 elements, void* const context);

template <class T, class ndEvaluateKey, ndInt32 keyBitSize>
void ndCountingSort(const T* const srcArray, T* const dstArray, ndInt32 size, ndUnsigned32* const prefixScanOut, void* const context);

template <class T, class ndEvaluateKey, ndInt32 keyBitSize>
void ndCountingSort(ndThreadPool& threadPool, const T* const srcArray, T* const dstArray, ndInt32 size, ndUnsigned32* const prefixScanOut, void* const context);

template <class T, class ndEvaluateKey, ndInt32 keyBitSize>
void ndCountingSort(ndArray<T>& array, ndArray<T>& scratchBuffer, ndUnsigned32* const prefixScanOut, void* const context);

template <class T, class ndEvaluateKey, ndInt32 keyBitSize>
void ndCountingSort(ndThreadPool& threadPool, ndArray<T>& array, ndArray<T>& scratchBuffer, ndUnsigned32* const prefixScanOut, void* const context);

template <class T, class ndEvaluateKey, ndInt32 keyBitSize>
void ndCountingSortInPlace(T* const array, T* const scratchBuffer, ndInt32 size, ndUnsigned32* const prefixScanOut, void* const context);

template <class T, class ndEvaluateKey, ndInt32 keyBitSize>
void ndCountingSortInPlace(ndThreadPool& threadPool, T* const array, T* const scratchBuffer, ndInt32 size, ndUnsigned32* const prefixScanOut, void* const context);


// *****************************************************
// Implementation 
// *****************************************************
template <class T, class dCompareKey>
void ndSort(T* const array, ndInt32 elements, void* const context)
{
	//D_TRACKTIME();
	const ndInt32 batchSize = 8;
	ndInt32 stack[128][2];

	stack[0][0] = 0;
	stack[0][1] = elements - 1;
	ndInt32 stackIndex = 1;
	const dCompareKey comparator(context);
	while (stackIndex)
	{
		stackIndex--;
		ndInt32 lo = stack[stackIndex][0];
		ndInt32 hi = stack[stackIndex][1];
		if ((hi - lo) > batchSize)
		{
			ndInt32 mid = (lo + hi) >> 1;
			if (comparator.Compare(array[lo], array[mid]) > 0)
			{
				ndSwap(array[lo], array[mid]);
			}
			if (comparator.Compare(array[mid], array[hi]) > 0)
			{
				ndSwap(array[mid], array[hi]);
			}
			if (comparator.Compare(array[lo], array[mid]) > 0)
			{
				ndSwap(array[lo], array[mid]);
			}
			ndInt32 i = lo + 1;
			ndInt32 j = hi - 1;
			const T pivot(array[mid]);
			do
			{
				while (comparator.Compare(array[i], pivot) < 0)
				{
					i++;
				}
				while (comparator.Compare(array[j], pivot) > 0)
				{
					j--;
				}

				if (i <= j)
				{
					ndSwap(array[i], array[j]);
					i++;
					j--;
				}
			} while (i <= j);

			if (i < hi)
			{
				stack[stackIndex][0] = i;
				stack[stackIndex][1] = hi;
				stackIndex++;
			}
			if (lo < j)
			{
				stack[stackIndex][0] = lo;
				stack[stackIndex][1] = j;
				stackIndex++;
			}
			ndAssert(stackIndex < ndInt32(sizeof(stack) / (2 * sizeof(stack[0][0]))));
		}
	}

	ndInt32 stride = batchSize + 1;
	if (elements < stride)
	{
		stride = elements;
	}
	for (ndInt32 i = 1; i < stride; ++i)
	{
		if (comparator.Compare(array[0], array[i]) > 0)
		{
			ndSwap(array[0], array[i]);
		}
	}

	for (ndInt32 i = 1; i < elements; ++i)
	{
		ndInt32 j = i;
		const T tmp(array[i]);
		for (; comparator.Compare(array[j - 1], tmp) > 0; --j)
		{
			ndAssert(j > 0);
			array[j] = array[j - 1];
		}
		array[j] = tmp;
	}

//#ifdef _DEBUG
#if 0
	for (ndInt32 i = 0; i < (elements - 1); ++i)
	{
		ndAssert(comparator.Compare(array[i], array[i + 1]) <= 0);
	}
#endif
}

template <class T, class ndEvaluateKey, ndInt32 keyBitSize>
void ndCountingSortInPlace(T* const array, T* const scratchBuffer, ndInt32 size, ndUnsigned32* const prefixScanOut, void* const context)
{
	//D_TRACKTIME();
	ndAssert(keyBitSize > 0);
	ndUnsigned32 scans[(1 << keyBitSize) + 1];
	ndEvaluateKey evaluator(context);
	for (ndInt32 i = 0; i < (1 << keyBitSize); ++i)
	{
		scans[i] = 0;
	}
	for (ndInt32 i = 0; i < size; ++i)
	{
		const T& entry = array[i];
		scratchBuffer[i] = entry;
		const ndInt32 key = evaluator.GetKey(entry);
		ndAssert(key >= 0);
		ndAssert(key < (1 << keyBitSize));
		scans[key] ++;
	}

	ndUnsigned32 sum = 0;
	for (ndInt32 i = 0; i < (1 << keyBitSize); ++i)
	{
		ndUnsigned32 partialSum = scans[i];
		scans[i] = sum;
		sum += partialSum;
	}

	if (prefixScanOut)
	{
		for (ndInt32 i = 0; i < ((1 << keyBitSize) + 1); ++i)
		{
			prefixScanOut[i] = scans[i];
		}
	}

	for (ndInt32 i = 0; i < size; ++i)
	{
		const T& entry = scratchBuffer[i];
		const ndInt32 key = evaluator.GetKey(entry);
		ndAssert(key >= 0);
		ndAssert(key < (1 << keyBitSize));
		const ndUnsigned32 index = scans[key];
		array[index] = entry;
		scans[key] = index + 1;
	}

	//#ifdef _DEBUG
#if 0
	for (ndInt32 i = size - 2; i >= 0; --i)
	{
		ndAssert(evaluator.GetKey(scratchBuffer[i]) <= evaluator.GetKey(scratchBuffer[i + 1]));
	}
#endif
}

template <class T, class ndEvaluateKey, ndInt32 keyBitSize>
void ndCountingSortInPlace(ndThreadPool& threadPool, T* const array, T* const scratchBuffer, ndInt32 size, ndUnsigned32* const prefixScanOut, void* const context)
{
	D_TRACKTIME();
	ndEvaluateKey evaluator(context);
	const ndInt32 threadCount = threadPool.GetThreadCount();

	ndUnsigned32* const sum = ndAlloca(ndUnsigned32, 1 << keyBitSize);
	ndUnsigned32* const scans = ndAlloca(ndUnsigned32, threadCount * (1 << keyBitSize));

	auto ndBuildHistogram = ndMakeObject::ndFunction([&array, &scratchBuffer, size, &evaluator, &scans](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(ndBuildHistogram);
		ndUnsigned32* const scan = &scans[threadIndex * (1 << keyBitSize)];

		for (ndInt32 i = 0; i < (1 << keyBitSize); ++i)
		{
			scan[i] = 0;
		}

		ndStartEnd startEnd(size, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			const T& entry = array[i];
			const ndInt32 key = evaluator.GetKey(entry);
			ndAssert(key >= 0);
			ndAssert(key < (1 << keyBitSize));
			scan[key] ++;
			scratchBuffer[i] = entry;
		}
	});

	auto ndShuffleArray = ndMakeObject::ndFunction([&array, &scratchBuffer, size, &evaluator, &scans, &sum](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(ndShuffleArray);
		ndUnsigned32* const scan = &scans[threadIndex * (1 << keyBitSize)];

		for (ndInt32 i = 0; i < (1 << keyBitSize); ++i)
		{
			scan[i] += sum[i];
		}
		ndStartEnd startEnd(size, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			const T& entry = scratchBuffer[i];
			const ndInt32 key = evaluator.GetKey(entry);
			ndAssert(key >= 0);
			ndAssert(key < (1 << keyBitSize));
			const ndUnsigned32 index = scan[key];
			array[index] = entry;
			scan[key] = index + 1;
		}
	});

	threadPool.ParallelExecute(ndBuildHistogram);

	ndAssert(keyBitSize < 11);
	for (ndInt32 i = 0; i < (1 << keyBitSize); ++i)
	{
		sum[i] = 0;
	}
	for (ndInt32 j = 0; j < threadCount; ++j)
	{
		ndUnsigned32* const scan = &scans[j * (1 << keyBitSize)];
		for (ndInt32 i = 0; i < (1 << keyBitSize); ++i)
		{
			ndUnsigned32 partialSum = scan[i];
			scan[i] = sum[i];
			sum[i] += partialSum;
		}
	}

	ndUnsigned32 accSum = 0;
	for (ndInt32 i = 0; i < (1 << keyBitSize); ++i)
	{
		ndUnsigned32 partialSum = sum[i];
		sum[i] = accSum;
		accSum += partialSum;
	}

	if (prefixScanOut)
	{
		for (ndInt32 i = 0; i < (1 << keyBitSize); ++i)
		{
			prefixScanOut[i] = sum[i];
		}
		prefixScanOut[1 << keyBitSize] = ndUnsigned32(size);
	}

	threadPool.ParallelExecute(ndShuffleArray);

//#ifdef _DEBUG
#if 0
	for (ndInt32 i = 1; i < size; ++i)
	{
		ndInt32 key0 = evaluator.GetKey(array[i - 1]);
		ndInt32 key1 = evaluator.GetKey(array[i + 0]);
		ndAssert(key0 <= key1);
	}
#endif
}

template <class T, class ndEvaluateKey, ndInt32 keyBitSize>
void ndCountingSort(const T* const srcArray, T* const dstArray, ndInt32 size, ndUnsigned32* const prefixScanOut, void* const context)
{
	//D_TRACKTIME();
	ndAssert(keyBitSize > 0);
	ndUnsigned32 scans[(1 << keyBitSize) + 1];
	ndEvaluateKey evaluator(context);
	for (ndInt32 i = 0; i < (1 << keyBitSize); ++i)
	{
		scans[i] = 0;
	}
	for (ndInt32 i = 0; i < size; ++i)
	{
		const T& entry = srcArray[i];
		const ndInt32 key = evaluator.GetKey(entry);
		ndAssert(key >= 0);
		ndAssert(key < (1 << keyBitSize));
		scans[key] ++;
	}

	ndUnsigned32 sum = 0;
	for (ndInt32 i = 0; i < (1 << keyBitSize); ++i)
	{
		ndUnsigned32 partialSum = scans[i];
		scans[i] = sum;
		sum += partialSum;
	}

	if (prefixScanOut)
	{ 
		for (ndInt32 i = 0; i < ((1 << keyBitSize) + 1); ++i)
		{
			prefixScanOut[i] = scans[i];
		}
	}

	for (ndInt32 i = 0; i < size; ++i)
	{
		const T& entry = srcArray[i];
		const ndInt32 key = evaluator.GetKey(entry);
		ndAssert(key >= 0);
		ndAssert(key < (1 << keyBitSize));
		const ndUnsigned32 index = scans[key];
		dstArray[index] = entry;
		scans[key] = index + 1;
	}

//#ifdef _DEBUG
#if 0
	for (ndInt32 i = size - 2; i >= 0; --i)
	{
		ndAssert(evaluator.GetKey(scratchBuffer[i]) <= evaluator.GetKey(scratchBuffer[i + 1]));
	}
#endif
}

template <class T, class ndEvaluateKey, ndInt32 keyBitSize>
void ndCountingSort(ndThreadPool& threadPool, const T* const srcArray, T* const dstArray, ndInt32 size, ndUnsigned32* const prefixScanOut,  void* const context)
{
	D_TRACKTIME();
	ndEvaluateKey evaluator(context);
	const ndInt32 threadCount = threadPool.GetThreadCount();

	ndUnsigned32* const sum = ndAlloca(ndUnsigned32, 1 << keyBitSize);
	ndUnsigned32* const scans = ndAlloca(ndUnsigned32, threadCount * (1 << keyBitSize));

	auto ndBuildHistogram = ndMakeObject::ndFunction([&srcArray, size, &evaluator, &scans](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(ndBuildHistogram);
		ndUnsigned32* const scan = &scans[threadIndex * (1 << keyBitSize)];

		for (ndInt32 i = 0; i < (1 << keyBitSize); ++i)
		{
			scan[i] = 0;
		}

		ndStartEnd startEnd(size, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			const T& entry = srcArray[i];
			const ndInt32 key = evaluator.GetKey(entry);
			ndAssert(key >= 0);
			ndAssert(key < (1 << keyBitSize));
			scan[key] ++;
		}
	});

	auto ndShuffleArray = ndMakeObject::ndFunction([&srcArray, &dstArray, size, &evaluator, &scans, &sum](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(ndShuffleArray);
		ndUnsigned32* const scan = &scans[threadIndex * (1 << keyBitSize)];

		for (ndInt32 i = 0; i < (1 << keyBitSize); ++i)
		{
			scan[i] += sum[i];
		}

		ndStartEnd startEnd(size, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			const T& entry = srcArray[i];
			const ndInt32 key = evaluator.GetKey(entry);
			ndAssert(key >= 0);
			ndAssert(key < (1 << keyBitSize));
			const ndUnsigned32 index = scan[key];
			dstArray[index] = entry;
			scan[key] = index + 1;
		}
	});

	threadPool.ParallelExecute(ndBuildHistogram);

	ndAssert(keyBitSize < 11);
	for (ndInt32 i = 0; i < (1 << keyBitSize); ++i)
	{
		sum[i] = 0;
	}
	for (ndInt32 j = 0; j < threadCount; ++j)
	{
		ndUnsigned32* const scan = &scans[j * (1 << keyBitSize)];
		for (ndInt32 i = 0; i < (1 << keyBitSize); ++i)
		{
			ndUnsigned32 partialSum = scan[i];
			scan[i] = sum[i];
			sum[i] += partialSum;
		}
	}

	ndUnsigned32 accSum = 0;
	for (ndInt32 i = 0; i < (1 << keyBitSize); ++i)
	{
		ndUnsigned32 partialSum = sum[i];
		sum[i] = accSum;
		accSum += partialSum;
	}

	if (prefixScanOut)
	{
		for (ndInt32 i = 0; i < (1 << keyBitSize); ++i)
		{
			prefixScanOut[i] = sum[i];
		}
		prefixScanOut[1 << keyBitSize] = ndUnsigned32(size);
	}

	threadPool.ParallelExecute(ndShuffleArray);

//#ifdef _DEBUG
#if 0
	for (ndInt32 i = 1; i < size; ++i)
	{
		ndInt32 key0 = evaluator.GetKey(scratchBuffer[i - 1]);
		ndInt32 key1 = evaluator.GetKey(scratchBuffer[i + 0]);
		ndAssert(key0 <= key1);
	}
#endif
}

template <class T, class ndEvaluateKey, ndInt32 keyBitSize>
void ndCountingSort(ndThreadPool& threadPool, ndArray<T>& array, ndArray<T>& scratchBuffer, ndUnsigned32* const prefixScanOut, void* const context)
{
	scratchBuffer.SetCount(array.GetCount());
	ndCountingSort<T, ndEvaluateKey, keyBitSize>(threadPool, &array[0], &scratchBuffer[0], ndInt32(array.GetCount()), prefixScanOut, context);
	array.Swap(scratchBuffer);
}

template <class T, class ndEvaluateKey, ndInt32 keyBitSize>
void ndCountingSort(ndArray<T>& array, ndArray<T>& scratchBuffer, ndUnsigned32* const prefixScanOut, void* const context)
{
	scratchBuffer.SetCount(array.GetCount());
	ndCountingSort<T, ndEvaluateKey, keyBitSize>(&array[0], &scratchBuffer[0], ndInt32(array.GetCount()), prefixScanOut, context);
	array.Swap(scratchBuffer);
}

#endif
