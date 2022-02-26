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

#define D_VERTEXLIST_INDEX_LIST_BASH (1024)

// numerical recipe in c
#define D_MAX_RAND		0x00ffffff
#define D_RAND_MUL		1664525u
#define D_ADD_ADD		1013904223u

static ndUnsigned32 ___dRandSeed___ = 0;

void dSetRandSeed(ndUnsigned32 seed)
{
	___dRandSeed___ = seed;
}

ndInt32 dRandInt()
{
	___dRandSeed___ = D_RAND_MUL * ___dRandSeed___ + D_ADD_ADD;
	return ___dRandSeed___ & D_MAX_RAND;
}

/// return a random variable between 0.0 and 1.0
ndFloat32 dRand()
{
	// numerical recipe in c
	ndFloat32 r = ndFloat32(dRandInt()) * ((ndFloat32(1.0f) / D_MAX_RAND));
	//dTrace(("%f\n", r));
	return r;
}

/// return a pseudo Gaussian random with mean 0 and variance 0.5f
ndFloat32 dGaussianRandom(ndFloat32 amp)
{
	// yes I know, this is not quite right but for now it will do for now.
	const ndInt32 count = 4;
	ndFloat32 r = ndFloat32(0.0f);
	for (ndInt32 i = 0; i < count; i++)
	{
		r += ndFloat32(2.0f) * dRand() - ndFloat32(1.0f);
	}
	r *= (amp / count);
	return r;
}

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

#ifndef D_USE_THREAD_EMULATION
void ndSpinLock::Delay(ndInt32& exp)
{
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

class dSortCluster
{
	public:
	ndBigVector m_sum;
	ndBigVector m_sum2;
	ndInt32 m_start;
	ndInt32 m_count;
};

class dSortKey
{
	public:
	ndInt32 m_mask;
	ndInt32 m_ordinal;
	ndInt32 m_vertexIndex;
};

static ndInt32 SortVertices(
	ndFloat64* const vertListOut, ndInt32* const indexList,
	const ndFloat64* const vertexList, ndInt32 stride, 
	ndInt32 compareCount, ndFloat64 tol,
	dSortKey* const remapIndex,
	const dSortCluster& cluster, ndInt32 baseCount)
{
	const ndBigVector origin(cluster.m_sum.Scale(ndFloat32(1.0f) / cluster.m_count));
	const ndBigVector variance(ndVector::m_zero.GetMax(cluster.m_sum2.Scale(ndFloat32(1.0f) / cluster.m_count) - origin * origin).Sqrt());

	ndInt32 firstSortAxis = 0;
	if ((variance.m_y >= variance.m_x) && (variance.m_y >= variance.m_z))
	{
		firstSortAxis = 1;
	}
	else if ((variance.m_z >= variance.m_x) && (variance.m_z >= variance.m_y))
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
		ndInt32 Compare(const dSortKey& elementA, const dSortKey& elementB, void* const context) const
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
	ndSort<dSortKey, dCompareKey>(remapIndex, cluster.m_count, &sortContext);

	const ndFloat64 minDist = dMin(dMin(variance.m_x, variance.m_y), variance.m_z);
	const ndFloat64 tolerance = dMax(dMin(minDist, ndFloat64(tol)), ndFloat64(1.0e-8f));
	const ndFloat64 sweptWindow = ndFloat64(2.0f) * tolerance;
	
	ndInt32 newCount = 0;
	for (ndInt32 i = 0; i < cluster.m_count; ++i)
	{
		const ndInt32 ii = remapIndex[i].m_mask;
		if (ii == -1)
		{
			const ndInt32 i0 = remapIndex[i].m_ordinal;
			const ndInt32 iii = remapIndex[i].m_vertexIndex;
			const ndFloat64 swept = vertexList[iii * stride + firstSortAxis] + sweptWindow;;
			for (ndInt32 j = i + 1; j < cluster.m_count; ++j)
			{
				const ndInt32 jj = remapIndex[j].m_mask;
				if (jj == -1)
				{
					const ndInt32 j0 = remapIndex[j].m_ordinal;
					const ndInt32 jjj = remapIndex[j].m_vertexIndex;
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
	
	for (ndInt32 i = 0; i < cluster.m_count; ++i)
	{
		ndInt32 i1 = remapIndex[i].m_ordinal;
		ndInt32 index = remapIndex[i].m_mask;
		indexList[i1] = index;
	}

	return newCount;
}

static ndInt32 QuickSortVertices(ndFloat64* const vertListOut, ndInt32 stride, ndInt32 compareCount, ndInt32 vertexCount, ndInt32* const indexListOut, ndFloat64 tolerance)
{
	dSortCluster cluster;
	cluster.m_start = 0;
	cluster.m_count = vertexCount;
	cluster.m_sum = ndBigVector::m_zero;
	cluster.m_sum2 = ndBigVector::m_zero;

	ndStack<ndFloat64>pool(stride  * cluster.m_count);
	ndStack<dSortKey> indirectListBuffer(cluster.m_count);
	dSortKey* const indirectList = &indirectListBuffer[0];

	ndFloat64* const vertList = &pool[0];
	memcpy(&vertList[0], &vertListOut[0], cluster.m_count * stride * sizeof(ndFloat64));

	for (ndInt32 i = 0; i < cluster.m_count; ++i)
	{
		indirectList[i].m_mask = -1;
		indirectList[i].m_ordinal = i;
		indirectList[i].m_vertexIndex = i;

		const ndBigVector x(vertList[i * stride + 0], vertList[i * stride + 1], vertList[i * stride + 2], ndFloat64(0.0f));
		cluster.m_sum += x;
		cluster.m_sum2 += x * x;
	}

	ndInt32 baseCount = 0;
	if (cluster.m_count > D_VERTEXLIST_INDEX_LIST_BASH)
	{
		dSortCluster spliteStack[128];
		spliteStack[0] = cluster;
	
		ndInt32 stack = 1;
		while (stack)
		{
			stack--;
			cluster = spliteStack[stack];

			const ndBigVector origin(cluster.m_sum.Scale(ndFloat32(1.0f) / cluster.m_count));
			const ndBigVector variance2(cluster.m_sum2.Scale(ndFloat32(1.0f) / cluster.m_count) - origin * origin);
			ndFloat64 maxVariance2 = dMax(dMax(variance2.m_x, variance2.m_y), variance2.m_z);

			dSortKey* const remapIndex = &indirectList[cluster.m_start];
			if ((cluster.m_count <= D_VERTEXLIST_INDEX_LIST_BASH) || (stack > (sizeof(spliteStack) / sizeof(spliteStack[0]) - 4)) || (maxVariance2 < ndFloat32(4.0f)))
			{
				baseCount += SortVertices(vertListOut, indexListOut, vertList, stride, compareCount, tolerance, remapIndex, cluster, baseCount);
			}
			else
			{
				ndInt32 firstSortAxis = 0;
				if ((variance2.m_y >= variance2.m_x) && (variance2.m_y >= variance2.m_z))
				{
					firstSortAxis = 1;
				}
				else if ((variance2.m_z >= variance2.m_x) && (variance2.m_z >= variance2.m_y))
				{
					firstSortAxis = 2;
				}
				ndFloat64 axisVal = origin[firstSortAxis];
	
				ndInt32 i0 = 0;
				ndInt32 i1 = cluster.m_count - 1;
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
				while ((vertList[index0 * stride + firstSortAxis] <= axisVal) && (i0 < cluster.m_count))
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
	
				for (ndInt32 i = i0; i < cluster.m_count; ++i)
				{
					index0 = remapIndex[i].m_vertexIndex;
					dAssert(vertList[index0 * stride + firstSortAxis] > axisVal);
				}
				#endif
	
				ndBigVector xc(ndBigVector::m_zero);
				ndBigVector x2c(ndBigVector::m_zero);
				for (ndInt32 i = 0; i < i0; ++i)
				{
					ndInt32 j = remapIndex[i].m_vertexIndex;
					const ndBigVector x(vertList[j * stride + 0], vertList[j * stride + 1], vertList[j * stride + 2], ndFloat64(0.0f));
					xc += x;
					x2c += x * x;
				}
	
				dSortCluster cluster_i1(cluster);
				cluster_i1.m_start = cluster.m_start + i0;
				cluster_i1.m_count = cluster.m_count - i0;
				cluster_i1.m_sum -= xc;
				cluster_i1.m_sum2 -= x2c;
				spliteStack[stack] = cluster_i1;
				stack++;
	
				dSortCluster cluster_i0(cluster);
				cluster_i0.m_start = cluster.m_start;
				cluster_i0.m_count = i0;
				cluster_i0.m_sum = xc;
				cluster_i0.m_sum2 = x2c;
				spliteStack[stack] = cluster_i0;
				stack++;
			}
		}
	}
	else
	{
		baseCount = SortVertices(vertListOut, indexListOut, vertList, stride, compareCount, tolerance, indirectList, cluster, 0);
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

