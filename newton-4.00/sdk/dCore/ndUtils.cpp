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

#include "ndCoreStdafx.h"
#include "ndSort.h"
#include "ndTypes.h"
#include "ndUtils.h"
#include "ndVector.h"
#include "ndMatrix.h"

#define D_USE_YZ_VARIANCE

#ifdef D_USE_YZ_VARIANCE
	#define D_VERTEXLIST_INDEX_LIST_BASH (1024 * 8)
#else
	#define D_VERTEXLIST_INDEX_LIST_BASH (1024)
#endif

ndFloat64 ndRoundToFloat(ndFloat64 val)
{
	ndInt32 exp;
	ndFloat64 mantissa = frexp(val, &exp);

	const ndFloat64 power = 1 << 23;
	const ndFloat64 invPower = ndFloat64(1.0f) / power;
	mantissa = floor(mantissa * power) * invPower;

	ndFloat64 val1 = ldexp(mantissa, exp);
	return val1;
}

ndUnsigned64 ndGetTimeInMicroseconds()
{
	static std::chrono::high_resolution_clock::time_point timeStampBase = std::chrono::high_resolution_clock::now();
	std::chrono::high_resolution_clock::time_point currentTimeStamp = std::chrono::high_resolution_clock::now();
	ndUnsigned64 timeStamp = ndUnsigned64(std::chrono::duration_cast<std::chrono::microseconds>(currentTimeStamp - timeStampBase).count());
	return timeStamp;
}

class ndSortCluster
{
	public:
	ndBigVector m_sum;
	ndBigVector m_sum2;
	ndInt32 m_start;
	ndInt32 m_count;
};

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
	ndInt32 compareCount, ndFloat64 tol,
	ndSortKey* const remapIndex,
	const ndSortCluster& cluster, ndInt32 baseCount, ndInt32 firstSortAxis)
{
#ifndef D_USE_YZ_VARIANCE
	firstSortAxis = 0;
	const ndBigVector origin(cluster.m_sum.Scale(ndFloat32(1.0f) / (ndFloat32)cluster.m_count));
	const ndBigVector variance(ndBigVector::m_zero.GetMax(cluster.m_sum2.Scale(ndFloat32(1.0f) / (ndFloat32)cluster.m_count) - origin * origin).Sqrt());

	if ((variance.m_y >= variance.m_x) && (variance.m_y >= variance.m_z))
	{
		firstSortAxis = 1;
	}
	else if ((variance.m_z >= variance.m_x) && (variance.m_z >= variance.m_y))
	{
		firstSortAxis = 2;
	}
#endif

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
	class ndCompareKey
	{
		public:
		ndCompareKey(void* const context)
			:m_sortContext((dVertexSortData*)context)
		{
		}

		ndInt32 Compare(const ndSortKey& elementA, const ndSortKey& elementB) const
		{
			ndInt32 index0 = elementA.m_vertexIndex * m_sortContext->m_stride + m_sortContext->m_vertexSortIndex;
			ndInt32 index1 = elementB.m_vertexIndex * m_sortContext->m_stride + m_sortContext->m_vertexSortIndex;

			if (m_sortContext->m_vertex[index0] < m_sortContext->m_vertex[index1])
			{
				return -1;
			}
			if (m_sortContext->m_vertex[index0] > m_sortContext->m_vertex[index1])
			{
				return 1;
			}
			return 0;
		}

		dVertexSortData* m_sortContext;
	};
	ndSort<ndSortKey, ndCompareKey>(remapIndex, cluster.m_count, &sortContext);

#ifdef D_USE_YZ_VARIANCE
	const ndFloat64 tolerance = ndMax(ndFloat64(tol), ndFloat64(1.0e-8f));
#else
	const ndFloat64 minDist = ndMin(ndMin(variance.m_x, variance.m_y), variance.m_z);
	const ndFloat64 tolerance = ndMax(ndMin(minDist, ndFloat64(tol)), ndFloat64(1.0e-8f));
#endif
	const ndFloat64 sweptWindow = ndFloat64(2.0f) * tolerance;
	
	ndInt32 newCount = 0;
	for (ndInt32 i = 0; i < cluster.m_count; ++i)
	{
		const ndInt32 ii = remapIndex[i].m_mask;
		if (ii == -1)
		{
			const ndInt32 iii = remapIndex[i].m_vertexIndex;
			const ndFloat64 swept = vertexList[iii * stride + firstSortAxis] + sweptWindow;;
			for (ndInt32 j = i + 1; j < cluster.m_count; ++j)
			{
				const ndInt32 jj = remapIndex[j].m_mask;
				if (jj == -1)
				{
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
		ndMemCpy(&vertListOut[dst], &vertexList[src], stride);
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
	ndSortCluster cluster;
	cluster.m_start = 0;
	cluster.m_count = vertexCount;
	cluster.m_sum = ndBigVector::m_zero;
	cluster.m_sum2 = ndBigVector::m_zero;

	ndStack<ndFloat64>pool(stride  * cluster.m_count);
	ndStack<ndSortKey> indirectListBuffer(cluster.m_count);
	ndSortKey* const indirectList = &indirectListBuffer[0];

	ndFloat64* const vertList = &pool[0];
	ndMemCpy(&vertList[0], &vertListOut[0], cluster.m_count * stride);

	for (ndInt32 i = 0; i < cluster.m_count; ++i)
	{
		indirectList[i].m_mask = -1;
		indirectList[i].m_ordinal = i;
		indirectList[i].m_vertexIndex = i;

		const ndBigVector x(vertList[i * stride + 0], vertList[i * stride + 1], vertList[i * stride + 2], ndFloat64(0.0f));
		cluster.m_sum += x;
		cluster.m_sum2 += x * x;
	}

	ndInt32 sortIndex = 0;
	#ifdef D_USE_YZ_VARIANCE
	const ndBigVector originScale(cluster.m_sum.Scale(ndFloat32(1.0f) / (ndFloat32)cluster.m_count));
	const ndBigVector varianceScale2((cluster.m_sum2.Scale(ndFloat32(1.0f) / (ndFloat32)cluster.m_count) - originScale * originScale).GetMax (ndBigVector(1.0e-6f)));
	ndFloat64 maxValue = varianceScale2.m_x;
	for (ndInt32 i = 1; i < 3; ++i)
	{
		if (varianceScale2[i] > maxValue)
		{
			sortIndex = i;
			maxValue = varianceScale2[i];
		}
	}
	ndInt32 varianceIndex0 = (sortIndex + 1) % 3;
	ndInt32 varianceIndex1 = (varianceIndex0 + 1) % 3;
	const ndBigVector stdScale(ndFloat64(1.0f) / varianceScale2.m_x, ndFloat64(1.0f) / varianceScale2.m_y, ndFloat64(1.0f) / varianceScale2.m_z, ndFloat64(0.0f));
	#endif

	ndInt32 baseCount = 0;
	if (cluster.m_count > D_VERTEXLIST_INDEX_LIST_BASH)
	{
		ndFixSizeArray<ndSortCluster, 128> spliteStack;
		spliteStack.PushBack(cluster);

		while (spliteStack.GetCount())
		{
			cluster = spliteStack.Pop();

			const ndBigVector origin(cluster.m_sum.Scale(ndFloat32(1.0f) / (ndFloat32)cluster.m_count));
			const ndBigVector variance2(cluster.m_sum2.Scale(ndFloat32(1.0f) / (ndFloat32)cluster.m_count) - origin * origin);
			ndSortKey* const remapIndex = &indirectList[cluster.m_start];

			#ifdef D_USE_YZ_VARIANCE
				ndFloat64 clusterVariance2_0 = stdScale[varianceIndex0] * ndMax(variance2[varianceIndex0], ndFloat64(0.0f));
				ndFloat64 clusterVariance2_1 = stdScale[varianceIndex1] * ndMax(variance2[varianceIndex1], ndFloat64(0.0f));
				ndFloat64 maxNormalizedVariance = ndMax (clusterVariance2_0, clusterVariance2_1);
				bool doSort = (maxNormalizedVariance < ndFloat32(1.0f/8.0f)) || (spliteStack.GetCount() > (spliteStack.GetCapacity() - 4));
			#else
				ndFloat64 maxVariance2 = ndMax(ndMax(variance2.m_x, variance2.m_y), variance2.m_z);
				bool doSort = (cluster.m_count <= D_VERTEXLIST_INDEX_LIST_BASH) || (spliteStack.GetCount() > (spliteStack.GetCapacity() - 4)) || (maxVariance2 < ndFloat32(4.0f));
			#endif
			if (doSort)
			{
				ndInt32 newCount = SortVertices(vertListOut, indexListOut, vertList, stride, compareCount, tolerance, remapIndex, cluster, baseCount, sortIndex);
				baseCount += newCount;
			}
			else
			{
				#ifdef D_USE_YZ_VARIANCE
					//ndInt32 firstSortAxis = (variance2.m_y >= variance2.m_z) ? 1 : 2;
					ndInt32 firstSortAxis = (clusterVariance2_0 > clusterVariance2_1) ? varianceIndex0 : varianceIndex1;
				#else
					ndInt32 firstSortAxis = 0;
					if ((variance2.m_y >= variance2.m_x) && (variance2.m_y >= variance2.m_z))
					{
						firstSortAxis = 1;
					}
					else if ((variance2.m_z >= variance2.m_x) && (variance2.m_z >= variance2.m_y))
					{
						firstSortAxis = 2;
					}
				#endif
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
	
					ndAssert(i0 <= i1);
					if (i0 < i1)
					{
						ndSwap(remapIndex[i0], remapIndex[i1]);
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
					ndAssert(vertList[index0 * stride + firstSortAxis] <= axisVal);
				}
	
				for (ndInt32 i = i0; i < cluster.m_count; ++i)
				{
					index0 = remapIndex[i].m_vertexIndex;
					ndAssert(vertList[index0 * stride + firstSortAxis] > axisVal);
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
	
				ndSortCluster cluster_i1(cluster);
				cluster_i1.m_start = cluster.m_start + i0;
				cluster_i1.m_count = cluster.m_count - i0;
				cluster_i1.m_sum -= xc;
				cluster_i1.m_sum2 -= x2c;
				spliteStack.PushBack(cluster_i1);
	
				ndSortCluster cluster_i0(cluster);
				cluster_i0.m_start = cluster.m_start;
				cluster_i0.m_count = i0;
				cluster_i0.m_sum = xc;
				cluster_i0.m_sum2 = x2c;
				spliteStack.PushBack(cluster_i0);
			}
		}
	}
	else
	{
		baseCount = SortVertices(vertListOut, indexListOut, vertList, stride, compareCount, tolerance, indirectList, cluster, 0, sortIndex);
	}
	return baseCount;
}

ndInt32 ndVertexListToIndexList(ndFloat64* const vertList, ndInt32 strideInBytes, ndInt32 compareCount, ndInt32 vertexCount, ndInt32* const indexListOut, ndFloat64 tolerance)
{
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

//ndInt32 ndVertexListToIndexList(ndFloat64* const vertexList, ndInt32 strideInBytes, ndInt32 compareCount, ndInt32 vertexCount, ndInt32* const indexListOut, ndFloat64 tolerance)
//{
//	//ndInt32 count = ndVertexListToIndexListInternal(vertexList, strideInBytes, compareCount, vertexCount, indexListOut, ndFloat64(tolerance));
//	return count;
//}

ndInt32 ndVertexListToIndexList(ndReal* const vertexList, ndInt32 strideInBytes, ndInt32 compareCount, ndInt32 vertexCount, ndInt32* const indexListOut, ndFloat64 tolerance)
{
	ndInt32 stride = ndInt32(strideInBytes / sizeof(ndReal));
	ndStack<ndFloat64> pool(vertexCount * stride);

	ndFloat64* const data = &pool[0];
	for (ndInt32 i = 0; i < vertexCount; ++i)
	{
		ndFloat64* const dst = &data[i * stride];
		const ndReal* const src = &vertexList[i * stride];
		for (ndInt32 j = 0; j < stride; ++j)
		{
			dst[j] = src[j];
		}
	}

	//ndInt32 count = ndVertexListToIndexListInternal(data, ndInt32(stride * sizeof(ndFloat64)), compareCount, vertexCount, indexListOut, ndFloat64(tolerance));
	ndInt32 count = ndVertexListToIndexList(data, ndInt32(stride * sizeof(ndFloat64)), compareCount, vertexCount, indexListOut, ndFloat64(tolerance));
	for (ndInt32 i = 0; i < count; ++i)
	{
		const ndFloat64* const src = &data[i * stride];
		ndReal* const dst = &vertexList[i * stride];
		for (ndInt32 j = 0; j < stride; ++j)
		{
			dst[j] = ndReal(src[j]);
		}
	}

	return count;
}
