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

#include "ndCoreStdafx.h"
#include "ndSort.h"
#include "ndTypes.h"
#include "ndUtils.h"
#include "ndVector.h"
#include "ndMatrix.h"

#define D_VERTEXLIST_INDEX_LIST_BASH (1024 * 32)

ndFloat64 dRoundToFloat(ndFloat64 val)
{
	ndInt32 exp;
	ndFloat64 mantissa = frexp(val, &exp);

	const ndFloat64 power = 1 << 23;
	const ndFloat64 invPower = ndFloat64(1.0f) / power;
	mantissa = floor(mantissa * power) * invPower;

	ndFloat64 val1 = ldexp(mantissa, exp);
	return val1;
}

ndUnsigned64 dGetTimeInMicroseconds()
{
	static std::chrono::high_resolution_clock::time_point timeStampBase = std::chrono::high_resolution_clock::now();
	std::chrono::high_resolution_clock::time_point currentTimeStamp = std::chrono::high_resolution_clock::now();
	ndUnsigned64 timeStamp = std::chrono::duration_cast<std::chrono::microseconds>(currentTimeStamp - timeStampBase).count();
	return timeStamp;
}

ndFloatExceptions::ndFloatExceptions(ndUnsigned32 mask)
{
#if	(defined(WIN32) || defined(_WIN32))
	ndClearFP();
	m_mask = ndControlFP(0, 0);
	ndControlFP(m_mask & ~mask, _MCW_EM);
#endif

#if defined (__APPLE__)
	#ifndef IOS
		fesetenv(FE_DFL_DISABLE_SSE_DENORMS_ENV);
	#endif
//#elif (defined(WIN32) || defined(_WIN32))
#elif defined (__x86_64) || defined(__x86_64__) || defined(_M_IX86) || defined(_M_X64)
	_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
	ndInt32 crs = _mm_getcsr();
	ndInt32 sseDenormalMask = _MM_FLUSH_ZERO_MASK | _MM_MASK_DENORM;
	_mm_setcsr(crs | sseDenormalMask);
#elif (defined (_M_ARM) || defined (_M_ARM64))
	//_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
	#pragma message ("warning!!! do not forget to set flush to zero for arm cpus")
#endif

	//ndFloat32 a = ndFloat32(1.0f);
	//ndFloat32 b = ndFloat32(0.5f);
	//ndInt32 count = 0;
	//while (a != 0.0f)
	//{
	//	a = a * b;
	//	count++;
	//}
	//count++;
}

ndFloatExceptions::~ndFloatExceptions()
{
#if (defined (_MSC_VER) && defined (_WIN_32_VER))
	ndClearFP();
	ndControlFP(m_mask, _MCW_EM);
#endif
}

ndSetPrecisionDouble::ndSetPrecisionDouble()
{
#if (defined (_MSC_VER) && defined (_WIN_32_VER))
	ndClearFP();
	m_mask = ndInt32(ndControlFP(0, 0));
	ndControlFP(_PC_53, _MCW_PC);
#endif
}

ndSetPrecisionDouble::~ndSetPrecisionDouble()
{
#if (defined (_MSC_VER) && defined (_WIN_32_VER))
	ndClearFP();
	ndControlFP(ndUnsigned32(m_mask), _MCW_PC);
#endif
}

class ndSplitSortBatch
{
	public:
	ndBigVector m_p0;
	ndBigVector m_p1;
	ndBigVector m_sum;
	ndBigVector m_variance;
	ndInt32 m_start;
	ndInt32 m_count;
};


static inline ndInt32 cmp_vertex(const ndFloat64* const v1, const ndFloat64* const v2, ndInt32 firstSortAxis)
{
	if (v1[firstSortAxis] < v2[firstSortAxis]) 
	{
		return -1;
	}

	if (v1[firstSortAxis] > v2[firstSortAxis]) 
	{
		return 1;
	}

	return 0;
}

#if 0

static ndInt32 SortVertices(ndFloat64* const vertexList, ndInt32 stride, ndInt32 compareCount, ndInt32 vertexCount, ndFloat64 tolerance)
{
	ndBigVector xc(ndBigVector::m_zero);
	ndBigVector x2c(ndBigVector::m_zero);
	ndBigVector minP(ndFloat64(1.0e10f), ndFloat64(1.0e10f), ndFloat64(1.0e10f), ndFloat64(0.0f));
	ndBigVector maxP(ndFloat64(-1.0e10f), ndFloat64(-1.0e10f), ndFloat64(-1.0e10f), ndFloat64(0.0f));

	for (ndInt32 k = 0, i = 0; i < vertexCount; i++)
	{
		ndBigVector x(vertexList[k + 2], vertexList[k + 3], vertexList[k + 4], ndFloat64(0.0f));
		xc += x;
		x2c += x * x;
		minP = minP.GetMin(x);
		maxP = maxP.GetMax(x);
		k += stride;
	}

	ndBigVector del(maxP - minP);
	ndFloat64 minDist = dMin(dMin(del.m_x, del.m_y), del.m_z);
	if (minDist < ndFloat64(1.0e-3f))
	{
		minDist = ndFloat64(1.0e-3f);
	}

	ndFloat64 tol = tolerance * minDist + ndFloat64(1.0e-12f);
	ndFloat64 sweptWindow = ndFloat64(2.0f) * tol;
	sweptWindow += ndFloat64(1.0e-4f);

	x2c = x2c.Scale(ndFloat32(vertexCount)) - xc * xc;

	ndInt32 firstSortAxis = 2;
	if ((x2c.m_y >= x2c.m_x) && (x2c.m_y >= x2c.m_z))
	{
		firstSortAxis = 3;
	}
	else if ((x2c.m_z >= x2c.m_x) && (x2c.m_z >= x2c.m_y))
	{
		firstSortAxis = 4;
	}

	ndInt32 stack[1024][2];
	stack[0][0] = 0;
	stack[0][1] = vertexCount - 1;
	ndInt32 stackIndex = 1;
	while (stackIndex)
	{
		stackIndex--;
		ndInt32 lo = stack[stackIndex][0];
		ndInt32 hi = stack[stackIndex][1];
		if ((hi - lo) > 8)
		{
			ndInt32 i = lo;
			ndInt32 j = hi;
			ndFloat64 val[64];
			memcpy(val, &vertexList[((lo + hi) >> 1) * stride], stride * sizeof(ndFloat64));
			do
			{
				while (cmp_vertex(&vertexList[i * stride], val, firstSortAxis) < 0) i++;
				while (cmp_vertex(&vertexList[j * stride], val, firstSortAxis) > 0) j--;

				if (i <= j)
				{
					if (i < j)
					{
						ndFloat64 tmp[64];
						memcpy(tmp, &vertexList[i * stride], stride * sizeof(ndFloat64));
						memcpy(&vertexList[i * stride], &vertexList[j * stride], stride * sizeof(ndFloat64));
						memcpy(&vertexList[j * stride], tmp, stride * sizeof(ndFloat64));
					}
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
			dAssert(stackIndex < ndInt32(sizeof(stack) / (2 * sizeof(stack[0][0]))));
		}
		else
		{
			for (ndInt32 i = lo + 1; i <= hi; i++)
			{
				ndFloat64 tmp[64];
				memcpy(tmp, &vertexList[i * stride], stride * sizeof(ndFloat64));

				ndInt32 j = i;
				for (; j && (cmp_vertex(&vertexList[(j - 1) * stride], tmp, firstSortAxis) > 0); j--)
				{
					memcpy(&vertexList[j * stride], &vertexList[(j - 1)* stride], stride * sizeof(ndFloat64));
				}
				memcpy(&vertexList[j * stride], tmp, stride * sizeof(ndFloat64));
			}
		}
	}

	#ifdef _DEBUG
	for (ndInt32 i = 0; i < (vertexCount - 1); i++)
	{
		dAssert(cmp_vertex(&vertexList[i * stride], &vertexList[(i + 1) * stride], firstSortAxis) <= 0);
	}
	#endif

	ndInt32 count = 0;
	for (ndInt32 i = 0; i < vertexCount; i++)
	{
		ndInt32 m = i * stride;
		ndInt32 index = ndInt32(vertexList[m + 0]);
		if (index == ndInt32(0xffffffff))
		{
			ndFloat64 swept = vertexList[m + firstSortAxis] + sweptWindow;
			ndInt32 k = i * stride + stride;
			for (ndInt32 i1 = i + 1; i1 < vertexCount; i1++)
			{

				index = ndInt32(vertexList[k + 0]);
				if (index == ndInt32(0xffffffff))
				{
					ndFloat64 val = vertexList[k + firstSortAxis];
					if (val >= swept)
					{
						break;
					}
					bool test = true;
					for (ndInt32 t = 0; test && (t < compareCount); t++)
					{
						val = fabs(vertexList[m + t + 2] - vertexList[k + t + 2]);
						test = test && (val <= tol);
					}
					if (test)
					{
						vertexList[k + 0] = ndFloat64(count);
					}
				}
				k += stride;
			}

			dAssert(&vertexList[count * stride + 2] <= &vertexList[m + 2]);
			if (&vertexList[count * stride + 2] < &vertexList[m + 2])
			{
				memcpy(&vertexList[count * stride + 2], &vertexList[m + 2], (stride - 2) * sizeof(ndFloat64));
			}
			vertexList[m + 0] = ndFloat64(count);
			count++;
		}
	}
	return count;
}

static ndInt32 QuickSortVertices(ndFloat64* const vertList, ndInt32 stride, ndInt32 compareCount, ndInt32 vertexCount, ndFloat64 tolerance)
{
	ndInt32 count = 0;
	if (vertexCount > D_VERTEXLIST_INDEX_LIST_BASH)
	{
		ndFloat64 x = ndFloat32(0.0f);
		ndFloat64 y = ndFloat32(0.0f);
		ndFloat64 z = ndFloat32(0.0f);
		ndFloat64 xd = ndFloat32(0.0f);
		ndFloat64 yd = ndFloat32(0.0f);
		ndFloat64 zd = ndFloat32(0.0f);

		for (ndInt32 i = 0; i < vertexCount; i++)
		{
			ndFloat64 x0 = vertList[i * stride + 2];
			ndFloat64 y0 = vertList[i * stride + 3];
			ndFloat64 z0 = vertList[i * stride + 4];
			x += x0;
			y += y0;
			z += z0;
			xd += x0 * x0;
			yd += y0 * y0;
			zd += z0 * z0;
		}

		xd = vertexCount * xd - x * x;
		yd = vertexCount * yd - y * y;
		zd = vertexCount * zd - z * z;

		ndInt32 axis = 2;
		ndFloat64 axisVal = x / vertexCount;
		if ((yd > xd) && (yd > zd))
		{
			axis = 3;
			axisVal = y / vertexCount;
		}
		if ((zd > xd) && (zd > yd))
		{
			axis = 4;
			axisVal = z / vertexCount;
		}

		ndInt32 i0 = 0;
		ndInt32 i1 = vertexCount - 1;
		do
		{
			for (; vertList[i0 * stride + axis] < axisVal; i0++);
			for (; vertList[i1 * stride + axis] > axisVal; i1--);
			if (i0 <= i1)
			{
				for (ndInt32 i = 0; i < stride; i++)
				{
					dSwap(vertList[i0 * stride + i], vertList[i1 * stride + i]);
				}
				i0++;
				i1--;
			}
		} while (i0 <= i1);
		dAssert(i0 < vertexCount);

		ndInt32 count0 = QuickSortVertices(&vertList[0 * stride], stride, compareCount, i0, tolerance);
		ndInt32 count1 = QuickSortVertices(&vertList[i0 * stride], stride, compareCount, vertexCount - i0, tolerance);

		count = count0 + count1;

		for (ndInt32 i = 0; i < count1; i++)
		{
			memcpy(&vertList[(count0 + i) * stride + 2], &vertList[(i0 + i) * stride + 2], (stride - 2) * sizeof(ndFloat64));
		}

		for (ndInt32 i = i0; i < vertexCount; i++)
		{
			vertList[i * stride] += ndFloat64(count0);
		}
	}
	else
	{
		count = SortVertices(vertList, stride, compareCount, vertexCount, tolerance);
	}

	return count;
}

ndInt32 dVertexListToIndexList(ndFloat64* const vertList, ndInt32 strideInBytes, ndInt32 compareCount, ndInt32 vertexCount, ndInt32* const indexListOut, ndFloat64 tolerance)
{
	ndSetPrecisionDouble precision;

	if (strideInBytes < 3 * ndInt32(sizeof(ndFloat64)))
	{
		return 0;
	}
	if (compareCount < 3)
	{
		return 0;
	}
	dAssert(compareCount <= ndInt32(strideInBytes / sizeof(ndFloat64)));
	dAssert(strideInBytes == ndInt32(sizeof(ndFloat64) * (strideInBytes / sizeof(ndFloat64))));

	ndInt32 stride = strideInBytes / ndInt32(sizeof(ndFloat64));
	ndInt32 stride2 = stride + 2;

	ndStack<ndFloat64>pool(stride2  * vertexCount);
	ndFloat64* const tmpVertexList = &pool[0];

	ndInt32 k = 0;
	ndInt32 m = 0;
	for (ndInt32 i = 0; i < vertexCount; ++i)
	{
		memcpy(&tmpVertexList[m + 2], &vertList[k], stride * sizeof(ndFloat64));
		tmpVertexList[m + 0] = ndFloat64(-1.0f);
		tmpVertexList[m + 1] = ndFloat64(i);
		k += stride;
		m += stride2;
	}

	ndInt32 count = QuickSortVertices(tmpVertexList, stride2, compareCount, vertexCount, tolerance);

	k = 0;
	m = 0;
	for (ndInt32 i = 0; i < count; ++i)
	{
		k = i * stride;
		m = i * stride2;
		memcpy(&vertList[k], &tmpVertexList[m + 2], stride * sizeof(ndFloat64));
		k += stride;
		m += stride2;
	}

	m = 0;
	for (ndInt32 i = 0; i < vertexCount; ++i)
	{
		ndInt32 i1 = ndInt32(tmpVertexList[m + 1]);
		ndInt32 index = ndInt32(tmpVertexList[m + 0]);
		indexListOut[i1] = index;
		m += stride2;
	}
	return count;
}

#else

class ndSortKey
{
	public:
	ndInt32 m_mask;
	ndInt32 m_ordinal;
	ndInt32 m_vertexIndex;
};

static ndInt32 SortVertices(
	ndFloat64* const vertListOut, ndInt32* const indexList,
	const ndFloat64* const vertexList, ndInt32 stride, 
	ndInt32 compareCount, ndFloat64 tolerance,
	ndSortKey* const remapIndex,
	const ndSplitSortBatch& batch, ndInt32 baseCount)
{
	const ndBigVector xc(batch.m_sum.Scale(ndFloat32(1.0f) / batch.m_count));
	const ndBigVector x2c(batch.m_variance.Scale(ndFloat32(1.0f) / batch.m_count) - xc * xc);

	ndInt32 firstSortAxis = 0;
	if ((x2c.m_y >= x2c.m_x) && (x2c.m_y >= x2c.m_z))
	{
		firstSortAxis = 1;
	}
	else if ((x2c.m_z >= x2c.m_x) && (x2c.m_z >= x2c.m_y))
	{
		firstSortAxis = 2;
	}

	class dVertexSortData
	{
		public:
		ndInt32 m_stride;
		ndInt32 m_vertexSortIndex;
		const ndFloat64* m_vertex;
	};

	dVertexSortData sortContext;
	sortContext.m_vertex = vertexList;
	sortContext.m_stride = stride;
	sortContext.m_vertexSortIndex = firstSortAxis;
	class dCompareKey
	{
		public:
		ndInt32 Compare(const ndSortKey& elementA, const ndSortKey& elementB, void* const context) const
		{
			const dVertexSortData* const sortContext = (dVertexSortData*)context;
			ndInt32 index0 = elementA.m_vertexIndex * sortContext->m_stride + sortContext->m_vertexSortIndex;
			ndInt32 index1 = elementB.m_vertexIndex * sortContext->m_stride + sortContext->m_vertexSortIndex;

			if (sortContext->m_vertex[index0] < sortContext->m_vertex[index1])
			{
				return -1;
			}
			if (sortContext->m_vertex[index0] > sortContext->m_vertex[index1])
			{
				return 1;
			}
			return 0;
		}
	};
	ndSort<ndSortKey, dCompareKey>(remapIndex, batch.m_count, &sortContext);

	const ndBigVector del(batch.m_p1 - batch.m_p0);
	ndFloat64 minDist = dMin(dMin(del.m_x, del.m_y), del.m_z);
	if (minDist < ndFloat64(1.0e-3f))
	{
		minDist = ndFloat64(1.0e-3f);
	}

	ndFloat64 tol = tolerance * minDist + ndFloat64(1.0e-12f);
	ndFloat64 sweptWindow = ndFloat64(2.0f) * tol;
	sweptWindow += ndFloat64(1.0e-4f);
	
	ndInt32 newCount = 0;
	for (ndInt32 i = 0; i < batch.m_count; ++i)
	{
		const ndInt32 ii = remapIndex[i].m_mask;
		if (ii == -1)
		{
			const ndInt32 i0 = remapIndex[i].m_ordinal;
			const ndInt32 iii = remapIndex[i].m_vertexIndex;
			//const ndBigVector& p = m_vertex[iii];
			//const ndFloat64 swept = p[firstSortAxis] + sweptWindow;
			const ndFloat64 swept = vertexList[iii * stride + firstSortAxis] + sweptWindow;;
			for (ndInt32 j = i + 1; j < batch.m_count; ++j)
			{
				const ndInt32 jj = remapIndex[j].m_mask;
				if (jj == -1)
				{
					const ndInt32 j0 = remapIndex[j].m_ordinal;
					const ndInt32 jjj = remapIndex[j].m_vertexIndex;
					//const ndBigVector& q = m_vertex[jjj];
					//ndFloat64 val = q[firstSortAxis];
					ndFloat64 val = vertexList[jjj * stride + firstSortAxis];
					if (val >= swept)
					{
						break;
					}

					bool test = true;
					for (ndInt32 t = 0; test && (t < compareCount); t++) 
					{
						val = fabs(vertexList[iii * stride + t] - vertexList[jjj * stride + t]);
						test = test && (val <= tol);
					}

					if (test)
					{
						remapIndex[j].m_mask = newCount + baseCount;
					}
				}
			}
	
			remapIndex[newCount].m_vertexIndex = remapIndex[i].m_vertexIndex;
			remapIndex[i].m_mask = newCount + baseCount;
			newCount++;
		}
	}
	
	for (ndInt32 i = 0; i < newCount; ++i)
	{
		ndInt32 dst = (baseCount + i) * stride;
		ndInt32 src = remapIndex[i].m_vertexIndex * stride;
		memcpy(&vertListOut[dst], &vertexList[src], stride * sizeof(ndFloat64));
	}
	
	for (ndInt32 i = 0; i < batch.m_count; ++i)
	{
		ndInt32 i1 = remapIndex[i].m_ordinal;
		ndInt32 index = remapIndex[i].m_mask;
		indexList[i1] = index;
	}

	return newCount;
}

static ndInt32 QuickSortVertices(ndFloat64* const vertListOut, ndInt32 stride, ndInt32 compareCount, ndInt32 vertexCount, ndInt32* const indexListOut, ndFloat64 tolerance)
{
	ndSplitSortBatch batch;
	batch.m_start = 0;
	batch.m_count = vertexCount;
	batch.m_sum = ndBigVector::m_zero;
	batch.m_variance = ndBigVector::m_zero;
	batch.m_p0 = ndBigVector(ndFloat64(1.0e20f));
	batch.m_p1 = ndBigVector(ndFloat64(-1.0e20f));

	ndStack<ndFloat64>pool(stride  * vertexCount);
	ndStack<ndSortKey> indirectListBuffer(vertexCount);
	ndSortKey* const indirectList = &indirectListBuffer[0];

	ndFloat64* const vertList = &pool[0];
	memcpy(&vertList[0], &vertListOut[0], vertexCount * stride * sizeof(ndFloat64));

	for (ndInt32 i = 0; i < vertexCount; ++i)
	{
		indirectList[i].m_mask = -1;
		indirectList[i].m_ordinal = i;
		indirectList[i].m_vertexIndex = i;

		const ndBigVector x(vertList[i * stride + 0], vertList[i * stride + 1], vertList[i * stride + 2], ndFloat64(0.0f));
		batch.m_sum += x;
		batch.m_variance += x * x;
		batch.m_p0 = batch.m_p0.GetMin(x);
		batch.m_p1 = batch.m_p1.GetMax(x);
	}

	ndInt32 baseCount = 0;
	if (vertexCount > D_VERTEXLIST_INDEX_LIST_BASH)
	{
		ndSplitSortBatch spliteStack[128];
		spliteStack[0] = batch;
	
		ndInt32 stack = 1;
		while (stack)
		{
			stack--;
			batch = spliteStack[stack];

			const ndBigVector origin(batch.m_sum.Scale(ndFloat32(1.0f) / batch.m_count));
			const ndBigVector variance(batch.m_variance.Scale(ndFloat32(1.0f) / batch.m_count) - origin * origin);
			ndFloat64 maxVariance = dMax(dMax(variance.m_x, variance.m_y), variance.m_z);

			ndSortKey* const remapIndex = &indirectList[batch.m_start];
			if ((batch.m_count <= D_VERTEXLIST_INDEX_LIST_BASH) || (stack > (sizeof(spliteStack) / sizeof(spliteStack[0]) - 4)) || (maxVariance < ndFloat32(10.0f)))
			{
				baseCount += SortVertices(vertListOut, indexListOut, vertList, stride, compareCount, tolerance, remapIndex, batch, baseCount);
			}
			else
			{
				ndInt32 firstSortAxis = 0;
				if ((variance.m_y >= variance.m_x) && (variance.m_y >= variance.m_z))
				{
					firstSortAxis = 1;
				}
				else if ((variance.m_z >= variance.m_x) && (variance.m_z >= variance.m_y))
				{
					firstSortAxis = 2;
				}
				ndFloat64 axisVal = origin[firstSortAxis];
	
				ndInt32 i0 = 0;
				ndInt32 i1 = batch.m_count - 1;
				while (i0 < i1)
				{
					ndInt32 index0 = remapIndex[i0].m_vertexIndex;
					while ((vertList[index0 * stride + firstSortAxis] <= axisVal) && (i0 < i1))
					{
						++i0;
						index0 = remapIndex[i0].m_vertexIndex;
					};
	
					ndInt32 index1 = remapIndex[i1].m_vertexIndex;
					while ((vertList[index1 * stride + firstSortAxis] > axisVal) && (i0 < i1))
					{
						--i1;
						index1 = remapIndex[i1].m_vertexIndex;
					}
	
					dAssert(i0 <= i1);
					if (i0 < i1)
					{
						dSwap(remapIndex[i0], remapIndex[i1]);
						++i0;
						--i1;
					}
				}
	
				ndInt32 index0 = remapIndex[i0].m_vertexIndex;
				while ((vertList[index0 * stride + firstSortAxis] <= axisVal) && (i0 < batch.m_count))
				{
					++i0;
					index0 = remapIndex[i0].m_vertexIndex;
				};
	
				#ifdef _DEBUG
				for (ndInt32 i = 0; i < i0; ++i)
				{
					index0 = remapIndex[i].m_vertexIndex;
					dAssert(vertList[index0 * stride + firstSortAxis] <= axisVal);
				}
	
				for (ndInt32 i = i0; i < batch.m_count; ++i)
				{
					index0 = remapIndex[i].m_vertexIndex;
					dAssert(vertList[index0 * stride + firstSortAxis] > axisVal);
				}
				#endif
	
				ndBigVector xc(ndBigVector::m_zero);
				ndBigVector x2c(ndBigVector::m_zero);
				ndBigVector maxP(ndFloat64(-1.0e20f));
				for (ndInt32 i = 0; i < i0; ++i)
				{
					ndInt32 j = remapIndex[i].m_vertexIndex;
					const ndBigVector x(vertList[j * stride + 0], vertList[j * stride + 1], vertList[j * stride + 2], ndFloat64(0.0f));
					xc += x;
					x2c += x * x;
					maxP = maxP.GetMax(x);
				}
	
				ndSplitSortBatch pair_i1(batch);
				pair_i1.m_start = batch.m_start + i0;
				pair_i1.m_count = batch.m_count - i0;
				pair_i1.m_p0[firstSortAxis] = maxP[firstSortAxis];
				pair_i1.m_sum -= xc;
				pair_i1.m_variance -= x2c;
				spliteStack[stack] = pair_i1;
				stack++;
	
				ndSplitSortBatch pair_i0(batch);
				pair_i0.m_start = batch.m_start;
				pair_i0.m_count = i0;
				pair_i0.m_p1[firstSortAxis] = maxP[firstSortAxis];
				pair_i0.m_sum = xc;
				pair_i0.m_variance = x2c;
				spliteStack[stack] = pair_i0;
				stack++;
			}
		}
	}
	else
	{
		baseCount = SortVertices(vertListOut, indexListOut, vertList, stride, compareCount, tolerance, indirectList, batch, 0);
	}
	return baseCount;
}

ndInt32 dVertexListToIndexList(ndFloat64* const vertList, ndInt32 strideInBytes, ndInt32 compareCount, ndInt32 vertexCount, ndInt32* const indexListOut, ndFloat64 tolerance)
{
	ndSetPrecisionDouble precision;

	if (strideInBytes < 3 * ndInt32(sizeof(ndFloat64))) 
	{
		return 0;
	}
	if (compareCount < 3) 
	{
		return 0;
	}

	ndInt32 stride = strideInBytes / ndInt32(sizeof(ndFloat64));
	ndInt32 count = QuickSortVertices(vertList, stride, compareCount, vertexCount, indexListOut, tolerance);
	return count;
}
#endif

#ifndef D_USE_THREAD_EMULATION
void ndSpinLock::Delay(ndInt32& exp)
{
	//#if (defined(WIN32) || defined(_WIN32))
	#if defined (__x86_64) || defined(__x86_64__) || defined(_M_IX86) || defined(_M_X64)
		// adding exponential pause delay
		for (ndInt32 i = 0; i < exp; ++i)
		{
			_mm_pause();
			_mm_pause();
		}
	#else
		// use standard thread yield on non x86 platforms 
		//std::this_thread::yield();
		volatile ndInt32 acc = 0;
		for (ndInt32 i = 0; i < exp; ++i)
		{
			acc++;
			acc++;
		}
	#endif
		exp = dMin(exp * 2, 64);
}
#endif
