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
#include "ndCollisionStdafx.h"
#include "ndScene.h"
#include "ndBodySphFluid.h"

#ifndef D_USE_NEW_FLUID

//#define D_USE_YZ_PLANE_BUCKETS


#define D_SPH_HASH_BITS				8
#define D_SPH_BUFFER_GRANULARITY	4096	

#define D_PARTICLE_BUCKET_SIZE		32
#define D_GRID_SIZE_SCALER			(1.0f)

#if 0

class ndBodySphFluid::ndGridHash
{
	public:
	enum ndGridType
	{
		m_adjacentGrid = 0,
		m_homeGrid = 1,
	};

	ndGridHash()
	{
	}

	ndGridHash(ndUnsigned64 gridHash)
		:m_gridHash(gridHash)
	{
	}

#ifdef D_USE_YZ_PLANE_BUCKETS
	ndGridHash(ndInt32 y, ndInt32 z)
	{
		m_gridHash = 0;
		m_y = ndUnsigned64(y);
		m_z = ndUnsigned64(z);
	}

	ndGridHash(const ndVector& grid, ndInt32 particleIndex)
	{
		ndAssert(grid.m_y >= ndFloat32(0.0f));
		ndAssert(grid.m_z >= ndFloat32(0.0f));
		ndAssert(grid.m_y < ndFloat32(1 << (D_SPH_HASH_BITS * 2)));
		ndAssert(grid.m_z < ndFloat32(1 << (D_SPH_HASH_BITS * 2)));

		ndVector hash(grid.GetInt());

		m_gridHash = 0;
		m_y = ndUnsigned64(hash.m_iy);
		m_z = ndUnsigned64(hash.m_iz);

		m_cellType = m_adjacentGrid;
		m_particleIndex = ndUnsigned64(particleIndex);
	}
#else
	ndGridHash(ndInt32 x, ndInt32 z)
	{
		m_gridHash = 0;
		m_x = ndUnsigned64(x);
		m_z = ndUnsigned64(z);
	}

	ndGridHash(const ndVector& grid, ndInt32 particleIndex)
	{
		ndAssert(grid.m_x >= ndFloat32(0.0f));
		ndAssert(grid.m_z >= ndFloat32(0.0f));
		ndAssert(grid.m_x < ndFloat32(1 << (D_SPH_HASH_BITS * 2)));
		ndAssert(grid.m_z < ndFloat32(1 << (D_SPH_HASH_BITS * 2)));

		ndVector hash(grid.GetInt());

		m_gridHash = 0;
		m_x = ndUnsigned64(hash.m_ix);
		m_z = ndUnsigned64(hash.m_iz);

		m_cellType = m_adjacentGrid;
		m_particleIndex = ndUnsigned64(particleIndex);
	}
#endif

	union
	{
		struct
		{
#ifdef D_USE_YZ_PLANE_BUCKETS
			ndUnsigned64 m_y				: D_SPH_HASH_BITS * 2;
			ndUnsigned64 m_z				: D_SPH_HASH_BITS * 2;
#else
			ndUnsigned64 m_x				: D_SPH_HASH_BITS * 2;
			ndUnsigned64 m_z				: D_SPH_HASH_BITS * 2;
#endif
			ndUnsigned64 m_particleIndex	: 23;
			ndUnsigned64 m_cellType			: 1;
		};
		struct
		{
#ifdef D_USE_YZ_PLANE_BUCKETS
			ndUnsigned64 m_yLow		: D_SPH_HASH_BITS;
			ndUnsigned64 m_yHigh	: D_SPH_HASH_BITS;
			ndUnsigned64 m_zLow		: D_SPH_HASH_BITS;
			ndUnsigned64 m_zHigh	: D_SPH_HASH_BITS;
#else
			ndUnsigned64 m_xLow		: D_SPH_HASH_BITS;
			ndUnsigned64 m_xHigh	: D_SPH_HASH_BITS;
			ndUnsigned64 m_zLow		: D_SPH_HASH_BITS;
			ndUnsigned64 m_zHigh	: D_SPH_HASH_BITS;
#endif
		};
		ndUnsigned64 m_gridHash		: D_SPH_HASH_BITS * 2 * 2;
	};
};

class ndBodySphFluid::ndParticlePair
{
	public:
	ndInt32 m_neighborg[D_PARTICLE_BUCKET_SIZE];
};

class ndBodySphFluid::ndParticleKernelDistance
{
	public:
	ndFloat32 m_dist[D_PARTICLE_BUCKET_SIZE];
};

class ndBodySphFluid::ndWorkingBuffers
{
	#define D_SPH_GRID_X_RESOLUTION 4

	public:
	ndWorkingBuffers()
		:m_accel(D_SPH_BUFFER_GRANULARITY)
		,m_locks(D_SPH_BUFFER_GRANULARITY)
		,m_pairCount(D_SPH_BUFFER_GRANULARITY)
		,m_gridScans(D_SPH_BUFFER_GRANULARITY)
		,m_density(D_SPH_BUFFER_GRANULARITY)
		,m_invDensity(D_SPH_BUFFER_GRANULARITY)
		,m_pairs(D_SPH_BUFFER_GRANULARITY)
		,m_hashGridMap(D_SPH_BUFFER_GRANULARITY)
		,m_hashGridMapScratchBuffer(D_SPH_BUFFER_GRANULARITY)
		,m_kernelDistance(D_SPH_BUFFER_GRANULARITY)
		,m_worlToGridOrigin(ndFloat32 (1.0f))
		,m_worlToGridScale(ndFloat32(1.0f))
		,m_hashGridSize(ndFloat32(0.0f))
		,m_hashInvGridSize(ndFloat32(0.0f))
	{
		for (ndInt32 i = 0; i < D_MAX_THREADS_COUNT; ++i)
		{
			m_partialsGridScans[i].Resize(D_SPH_BUFFER_GRANULARITY);
		}
	}

	~ndWorkingBuffers()
	{
	}

	void SetWorldToGridMapping(ndFloat32 gridSize, const ndVector& maxP, const ndVector& minP)
	{
#ifdef D_USE_YZ_PLANE_BUCKETS
		m_worlToGridOrigin = minP.m_x;
		ndFloat32 gridCount = ndFloor((maxP.m_x - minP.m_x) / gridSize + ndFloat32(1.0f));
		m_worlToGridScale = ndFloat32(1<< D_SPH_GRID_X_RESOLUTION) * gridCount / (maxP.m_x - minP.m_x);
#else
		m_worlToGridOrigin = minP.m_y;
		ndFloat32 gridCount = ndFloor((maxP.m_y - minP.m_y) / gridSize + ndFloat32(1.0f));
		m_worlToGridScale = ndFloat32(1 << D_SPH_GRID_X_RESOLUTION) * (ndFloat32)gridCount / (maxP.m_y - minP.m_y);
#endif
	}

	ndInt32 WorldToGrid(const ndVector& point) const
	{
#ifdef D_USE_YZ_PLANE_BUCKETS
		ndInt32 val = ndInt32((point.m_x - m_worlToGridOrigin) * m_worlToGridScale);
#else
		ndInt32 val = ndInt32((point.m_y - m_worlToGridOrigin) * m_worlToGridScale);
#endif
		ndAssert(val >= 0);
		return val;
	}

	ndArray<ndVector> m_accel;
	ndArray<ndSpinLock> m_locks;
	ndArray<ndInt8> m_pairCount;
	ndArray<ndInt32> m_gridScans;
	ndArray<ndFloat32> m_density;
	ndArray<ndFloat32> m_invDensity;
	ndArray<ndParticlePair> m_pairs;
	ndArray<ndGridHash> m_hashGridMap;
	ndArray<ndGridHash> m_hashGridMapScratchBuffer;
	ndArray<ndParticleKernelDistance> m_kernelDistance;
	ndArray<ndInt32> m_partialsGridScans[D_MAX_THREADS_COUNT];
	ndFloat32 m_worlToGridOrigin;
	ndFloat32 m_worlToGridScale;
	ndFloat32 m_hashGridSize;
	ndFloat32 m_hashInvGridSize;
};

ndBodySphFluid::ndBodySphFluid()
	:ndBodyParticleSet()
	,m_workingBuffers(new ndWorkingBuffers)
	,m_mass(ndFloat32(1.0f))
	,m_viscosity(ndFloat32 (1.05f))
	,m_restDensity(ndFloat32(1000.0f))
	,m_gasConstant(ndFloat32(1.0f))
{
	SetRestDensity(m_restDensity);
}

ndBodySphFluid::~ndBodySphFluid()
{
	delete m_workingBuffers;
}

void ndBodySphFluid::SortBuckets(ndThreadPool* const threadPool)
{
	D_TRACKTIME();

	class ndKey_low
	{
		public:
		ndKey_low(void* const context)
			:m_fluid((ndBodySphFluid*)context)
			,m_data(*m_fluid->m_workingBuffers)
			,m_point(m_fluid->GetPositions())
		{
		}

		ndInt32 GetKey(const ndGridHash& cell) const
		{
			ndInt32 index = ndInt32(cell.m_particleIndex);
			ndUnsigned32 key = ndUnsigned32(m_data.WorldToGrid(m_point[index]));
			return ndInt32(key & 0xff);
		}

		ndBodySphFluid* m_fluid;
		ndWorkingBuffers& m_data;
		const ndArray<ndVector>& m_point;
	};

	class ndKey_middle
	{
		public:
		ndKey_middle(void* const context)
			:m_fluid((ndBodySphFluid*)context)
			,m_data(*m_fluid->m_workingBuffers)
			,m_point(m_fluid->GetPositions())
		{
		}

		ndInt32 GetKey(const ndGridHash& cell) const
		{
			ndInt32 index = ndInt32 (cell.m_particleIndex);
			ndUnsigned32 key = ndUnsigned32(m_data.WorldToGrid(m_point[index]));
			return ndInt32((key >> 8) & 0xff);
		}

		ndBodySphFluid* m_fluid;
		ndWorkingBuffers& m_data;
		const ndArray<ndVector>& m_point;
	};

	class ndKey_high
	{
		public:
		ndKey_high(void* const context)
			:m_fluid((ndBodySphFluid*)context)
			,m_data(*m_fluid->m_workingBuffers)
			,m_point(m_fluid->GetPositions())
		{
		}

		ndInt32 GetKey(const ndGridHash& cell) const
		{
			ndInt32 index = ndInt32(cell.m_particleIndex);
			ndUnsigned32 key = ndUnsigned32(m_data.WorldToGrid(m_point[index]));
			return ndInt32((key >> 16) & 0xff);
		}

		ndBodySphFluid* m_fluid;
		ndWorkingBuffers& m_data;
		const ndArray<ndVector>& m_point;
	};

	ndWorkingBuffers& data = *m_workingBuffers;
	const ndInt32 keySize = data.WorldToGrid(m_box1);

	ndCountingSort<ndGridHash, ndKey_low, 8>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, nullptr, this);
	if (keySize >= 256)
	{
		ndCountingSort<ndGridHash, ndKey_middle, 8>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, nullptr, this);
	}
	if (keySize >= (256 * 256))
	{
		ndCountingSort<ndGridHash, ndKey_high, 8>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, nullptr, this);
	}

#ifdef _DEBUG
	const ndArray<ndVector>& point = GetPositions();
	for (ndInt32 i = 1; i < data.m_hashGridMap.GetCount(); ++i)
	{
		ndGridHash cell0(data.m_hashGridMap[i - 1]);
		ndGridHash cell1(data.m_hashGridMap[i + 0]);
		const ndVector p0(point[ndInt32(cell0.m_particleIndex)]);
		const ndVector p1(point[ndInt32(cell1.m_particleIndex)]);
		ndInt32 key0 = data.WorldToGrid(p0);
		ndInt32 key1 = data.WorldToGrid(p1);
		ndAssert(key0 <= key1);
	}
#endif
}

void ndBodySphFluid::SortCellBuckects(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
#ifdef D_USE_YZ_PLANE_BUCKETS
	class ndKey_ylow
	{
		public:
		ndKey_ylow(void* const) {}
		ndInt32 GetKey(const ndGridHash& cell) const
		{
			return ndInt32(cell.m_yLow);
		}
	};

	class ndKey_yhigh
	{
		public:
		ndKey_yhigh(void* const) {}
		ndInt32 GetKey(const ndGridHash& cell) const
		{
			return ndInt32(cell.m_yHigh);
		}
	};
#else
	class ndKey_xlow
	{
		public:
		ndKey_xlow(void* const) {}
		ndInt32 GetKey(const ndGridHash& cell) const
		{
			return ndInt32(cell.m_xLow);
		}
	};

	class ndKey_xhigh
	{
		public:
		ndKey_xhigh(void* const) {}
		ndInt32 GetKey(const ndGridHash& cell) const
		{
			return ndInt32(cell.m_xHigh);
		}
	};

#endif

	class ndKey_zlow
	{
		public:
		ndKey_zlow(void* const) {}
		ndInt32 GetKey(const ndGridHash& cell) const
		{
			return ndInt32(cell.m_zLow);
		}
	};

	class ndKey_zhigh
	{
		public:
		ndKey_zhigh(void* const) {}
		ndInt32 GetKey(const ndGridHash& cell) const
		{
			return ndInt32(cell.m_zHigh);
		}
	};

	ndWorkingBuffers& data = *m_workingBuffers;
	//const ndVector boxSize((m_box1 - m_box0).Scale(ndFloat32(1.0f) / GetSphGridSize()).GetInt());
	const ndVector boxSize((m_box1 - m_box0).Scale(data.m_hashInvGridSize).GetInt());

#ifdef D_USE_YZ_PLANE_BUCKETS
	ndCountingSort<ndGridHash, ndKey_ylow, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, nullptr, nullptr);
	if (boxSize.m_iy > (1 << D_SPH_HASH_BITS))
	{
		ndCountingSort<ndGridHash, ndKey_yhigh, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, nullptr, nullptr);
	}
#else
	ndCountingSort<ndGridHash, ndKey_xlow, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, nullptr, nullptr);
	if (boxSize.m_ix > (1 << D_SPH_HASH_BITS))
	{
		ndCountingSort<ndGridHash, ndKey_xhigh, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, nullptr, nullptr);
	}
#endif
	
	ndCountingSort<ndGridHash, ndKey_zlow, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, nullptr, nullptr);
	if (boxSize.m_iz > (1 << D_SPH_HASH_BITS))
	{
		ndCountingSort<ndGridHash, ndKey_zhigh, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, nullptr, nullptr);
	}

#ifdef _DEBUG
	for (ndInt32 i = 1; i < data.m_hashGridMap.GetCount(); ++i)
	{
		ndGridHash cell0(data.m_hashGridMap[i - 1]);
		ndGridHash cell1(data.m_hashGridMap[i + 0]);
#ifdef D_USE_YZ_PLANE_BUCKETS
		ndUnsigned64 key0 = (cell0.m_z << (D_SPH_HASH_BITS * 2)) + cell0.m_y;
		ndUnsigned64 key1 = (cell1.m_z << (D_SPH_HASH_BITS * 2)) + cell1.m_y;
#else
		ndUnsigned64 key0 = (cell0.m_z << (D_SPH_HASH_BITS * 2)) + cell0.m_x;
		ndUnsigned64 key1 = (cell1.m_z << (D_SPH_HASH_BITS * 2)) + cell1.m_x;
#endif
		ndAssert(key0 <= key1);
	}
#endif
}

void ndBodySphFluid::CalculateScans(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	ndWorkingBuffers& data = *m_workingBuffers;
	ndInt32 sums[D_MAX_THREADS_COUNT + 1];
	ndInt32 scans[D_MAX_THREADS_COUNT + 1];

	auto CountGridScans = ndMakeObject::ndFunction([&data, &scans](ndInt32 threadIndex, ndInt32)
	{
		D_TRACKTIME_NAMED(CountGridScans);
		const ndGridHash* const hashGridMap = &data.m_hashGridMap[0];

		const ndInt32 start = scans[threadIndex];
		const ndInt32 end = scans[threadIndex + 1];
		ndArray<ndInt32>& gridScans = data.m_partialsGridScans[threadIndex];
		ndUnsigned64 gridHash0 = hashGridMap[start].m_gridHash;

		ndInt32 count = 0;
		gridScans.SetCount(0);
		for (ndInt32 i = start; i < end; ++i)
		{
			ndUnsigned64 gridHash = hashGridMap[i].m_gridHash;
			if (gridHash != gridHash0)
			{
				gridScans.PushBack(count);
				count = 0;
				gridHash0 = gridHash;
			}
			count++;
		}
		gridScans.PushBack(count);
	});

	auto CalculateScans = ndMakeObject::ndFunction([&data, &scans, &sums](ndInt32 threadIndex, ndInt32)
	{
		D_TRACKTIME_NAMED(CalculateScans);
		ndArray<ndInt32>& gridScans = data.m_gridScans;
		const ndArray<ndInt32>& partialScan = data.m_partialsGridScans[threadIndex];
		const ndInt32 base = sums[threadIndex];
		ndInt32 sum = scans[threadIndex];
		for (ndInt32 i = 0; i < partialScan.GetCount(); ++i)
		{
			gridScans[base + i] = sum;
			sum += partialScan[i];
		}
	});

	memset(scans, 0, sizeof(scans));
	const ndInt32 threadCount = threadPool->GetThreadCount();
	
	ndInt32 particleCount = data.m_hashGridMap.GetCount();

	ndInt32 acc0 = 0;
	ndInt32 stride = particleCount / threadCount;
	const ndGridHash* const hashGridMap = &data.m_hashGridMap[0];
	for (ndInt32 threadIndex = 0; threadIndex < threadCount; threadIndex++)
	{
		scans[threadIndex] = acc0;
		acc0 += stride;
		while (acc0 < particleCount && (hashGridMap[acc0].m_gridHash == hashGridMap[acc0 - 1].m_gridHash))
		{
			acc0++;
		}
	}
	scans[threadCount] = particleCount;
	threadPool->ParallelExecute(CountGridScans);

	ndInt32 scansCount = 0;
	
	for (ndInt32 i = 0; i < threadCount; ++i)
	{
		sums[i] = scansCount;
		scansCount += data.m_partialsGridScans[i].GetCount();
	}
	sums[threadCount] = scansCount;

	data.m_gridScans.SetCount(scansCount + 1);
	threadPool->ParallelExecute(CalculateScans);

	data.m_gridScans[scansCount] = scans[threadCount];
}

void ndBodySphFluid::SortGrids(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	SortBuckets(threadPool);
	SortCellBuckects(threadPool);

	#ifdef _DEBUG
	ndWorkingBuffers& data = *m_workingBuffers;
	for (ndInt32 i = 0; i < (data.m_hashGridMap.GetCount() - 1); ++i)
	{
		const ndGridHash& entry0 = data.m_hashGridMap[i + 0];
		const ndGridHash& entry1 = data.m_hashGridMap[i + 1];
		ndUnsigned64 gridHashA = entry0.m_gridHash;
		ndUnsigned64 gridHashB = entry1.m_gridHash;
		ndAssert(gridHashA <= gridHashB);
	}
	#endif
}

void ndBodySphFluid::BuildBuckets(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	ndWorkingBuffers& data = *m_workingBuffers;
	ndInt32 countReset = data.m_locks.GetCount();
	data.m_pairs.SetCount(m_posit.GetCount());
	data.m_locks.SetCount(m_posit.GetCount());
	data.m_pairCount.SetCount(m_posit.GetCount());
	data.m_kernelDistance.SetCount(m_posit.GetCount());
	for (ndInt32 i = countReset; i < data.m_locks.GetCount(); ++i)
	{
		data.m_locks[i].Unlock();
	}
	for (ndInt32 i = 0; i < data.m_pairCount.GetCount(); ++i)
	{
		data.m_pairCount[i] = 0;
	}

	auto AddPairs = ndMakeObject::ndFunction([this, &data](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(AddPairs);
		const ndArray<ndGridHash>& hashGridMap = data.m_hashGridMap;
		const ndArray<ndInt32>& gridScans = data.m_gridScans;
		const ndFloat32 diameter = ndFloat32(1.5f) * ndFloat32(2.0f) * GetParticleRadius();
		const ndFloat32 diameter2 = diameter * diameter;
		const ndInt32 windowsTest = data.WorldToGrid(ndVector (data.m_worlToGridOrigin + diameter)) + 1;

		ndArray<ndSpinLock>& locks = data.m_locks;
		ndArray<ndInt8>& pairCount = data.m_pairCount;
		ndArray<ndParticlePair>& pair = data.m_pairs;
		ndArray<ndParticleKernelDistance>& distance = data.m_kernelDistance;

		auto ProccessCell = [this, &data, &hashGridMap, &pair, &pairCount, &locks, &distance, windowsTest, diameter2](ndInt32 start, ndInt32 count)
		{
			const ndInt32 count0 = count - 1;
			for (ndInt32 i = 0; i < count0; ++i)
			{
				const ndGridHash hash0 = hashGridMap[start + i];
				const ndInt32 particle0 = ndInt32(hash0.m_particleIndex);
				const ndInt32 x0 = data.WorldToGrid(m_posit[particle0]);
				const bool homeGridTest0 = (hash0.m_cellType == ndGridHash::m_homeGrid);
				for (ndInt32 j = i + 1; j < count; ++j)
				{
					const ndGridHash hash1 = hashGridMap[start + j];
					const ndInt32 particle1 = ndInt32(hash1.m_particleIndex);
					ndAssert(particle0 != particle1);
					const ndInt32 x1 = data.WorldToGrid(m_posit[particle1]);
					const ndInt32 sweeptTest = ((x1 - x0) >= windowsTest);
					if (sweeptTest)
					{
						break;
					}
					ndAssert(particle0 != particle1);
					const bool homeGridTest1 = (hash1.m_cellType == ndGridHash::m_homeGrid);
					const ndInt32 test = homeGridTest0 | homeGridTest1;
					if (test)
					{
						const ndVector p1p0(m_posit[particle0] - m_posit[particle1]);
						const ndFloat32 dist2(p1p0.DotProduct(p1p0).GetScalar());
						if (dist2 < diameter2)
						{
							const ndFloat32 dist = ndSqrt(ndMax(dist2, ndFloat32(1.0e-8f)));
							{
								ndSpinLock lock(locks[particle0]);
								ndInt8 neigborCount = pairCount[particle0];
								if (neigborCount < D_PARTICLE_BUCKET_SIZE)
								{
									ndInt8 isUnique = 1;
									ndInt32* const neighborg = pair[particle0].m_neighborg;
									for (ndInt32 k = neigborCount - 1; k >= 0; --k)
									{
										isUnique = isUnique & (neighborg[k] != particle1);
									}
									//ndAssert(isUnique);
					
									neighborg[neigborCount] = particle1;
									distance[particle0].m_dist[neigborCount] = dist;
									pairCount[particle0] = neigborCount + isUnique;
								}
							}
					
							{
								ndSpinLock lock(locks[particle1]);
								ndInt8 neigborCount = pairCount[particle1];
								if (neigborCount < D_PARTICLE_BUCKET_SIZE)
								{
									ndInt8 isUnique = 1;
									ndInt32* const neighborg = pair[particle1].m_neighborg;
									for (ndInt32 k = neigborCount - 1; k >= 0; --k)
									{
										isUnique = isUnique & (neighborg[k] != particle0);
									}
									//ndAssert(isUnique);

									neighborg[neigborCount] = particle0;
									distance[particle1].m_dist[neigborCount] = dist;
									pairCount[particle1] = neigborCount + isUnique;
								}
							}
						}
					}
				}
			}
		};

		const ndInt32 scansCount = gridScans.GetCount() - 1;
		for (ndInt32 i = threadIndex; i < scansCount; i += threadCount)
		{
			const ndInt32 start = gridScans[i];
			const ndInt32 count = gridScans[i + 1] - start;
			ProccessCell(start, count);
		}
	});

	auto AddPairs_new = ndMakeObject::ndFunction([this, &data](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(AddPairs);
		const ndArray<ndGridHash>& hashGridMap = data.m_hashGridMap;
		const ndArray<ndInt32>& gridScans = data.m_gridScans;
		const ndFloat32 diameter = ndFloat32(1.5f) * ndFloat32(2.0f) * GetParticleRadius();
		const ndFloat32 diameter2 = diameter * diameter;
		const ndInt32 windowsTest = data.WorldToGrid(ndVector(data.m_worlToGridOrigin + diameter)) + 1;

		ndArray<ndSpinLock>& locks = data.m_locks;
		ndArray<ndInt8>& pairCount = data.m_pairCount;
		ndArray<ndParticlePair>& pair = data.m_pairs;
		ndArray<ndParticleKernelDistance>& distance = data.m_kernelDistance;

		auto ProccessCell = [this, &data, &hashGridMap, &pair, &pairCount, &locks, &distance, windowsTest, diameter2](ndInt32 start, ndInt32 count)
		{
			const ndInt32 count0 = count - 1;
			for (ndInt32 i = 0; i < count0; ++i)
			{
				const ndGridHash hash0 = hashGridMap[start + i];
				const ndInt32 particle0 = ndInt32(hash0.m_particleIndex);
				const ndInt32 x0 = data.WorldToGrid(m_posit[particle0]);
				const bool homeGridTest0 = (hash0.m_cellType == ndGridHash::m_homeGrid);
				for (ndInt32 j = i + 1; j < count; ++j)
				{
					const ndGridHash hash1 = hashGridMap[start + j];
					const ndInt32 particle1 = ndInt32(hash1.m_particleIndex);
					ndAssert(particle0 != particle1);
					const ndInt32 x1 = data.WorldToGrid(m_posit[particle1]);
					const ndInt32 sweeptTest = ((x1 - x0) >= windowsTest);
					if (sweeptTest)
					{
						break;
					}
					ndAssert(particle0 != particle1);
					const ndVector p1p0(m_posit[particle0] - m_posit[particle1]);
					const ndFloat32 dist2(p1p0.DotProduct(p1p0).GetScalar());
					if (dist2 < diameter2)
					{
						const bool homeGridTest1 = (hash1.m_cellType == ndGridHash::m_homeGrid);
						if (homeGridTest0 && homeGridTest1)
						{
							ndInt8 neigborCount0 = pairCount[particle0];
							const ndFloat32 dist = ndSqrt(ndMax(dist2, ndFloat32(1.0e-8f)));
							if (neigborCount0 < D_PARTICLE_BUCKET_SIZE)
							{
								pair[particle0].m_neighborg[neigborCount0] = particle1;
								distance[particle0].m_dist[neigborCount0] = dist;
								pairCount[particle0] = neigborCount0 + 1;
							}
		
							ndInt8 neigborCount1 = pairCount[particle1];
							if (neigborCount1 < D_PARTICLE_BUCKET_SIZE)
							{
								pair[particle1].m_neighborg[neigborCount1] = particle0;
								distance[particle1].m_dist[neigborCount1] = dist;
								pairCount[particle1] = neigborCount1 + 1;
							}
		
						}
						else if (homeGridTest0)
						{
							ndAssert(!homeGridTest1);
							ndInt8 neigborCount0 = pairCount[particle0];
							const ndFloat32 dist = ndSqrt(ndMax(dist2, ndFloat32(1.0e-8f)));
							if (neigborCount0 < D_PARTICLE_BUCKET_SIZE)
							{
								pair[particle0].m_neighborg[neigborCount0] = particle1;
								distance[particle0].m_dist[neigborCount0] = dist;
								pairCount[particle0] = neigborCount0 + 1;
							}
						}
						else if (homeGridTest1)
						{
							ndAssert(!homeGridTest0);
							ndInt8 neigborCount1 = pairCount[particle1];
							const ndFloat32 dist = ndSqrt(ndMax(dist2, ndFloat32(1.0e-8f)));
							if (neigborCount1 < D_PARTICLE_BUCKET_SIZE)
							{
								pair[particle1].m_neighborg[neigborCount1] = particle0;
								distance[particle1].m_dist[neigborCount1] = dist;
								pairCount[particle1] = neigborCount1 + 1;
							}
						}
					}
				}
			}
		};

		const ndInt32 scansCount = gridScans.GetCount() - 1;
		for (ndInt32 i = threadIndex; i < scansCount; i += threadCount)
		{
			const ndInt32 start = gridScans[i];
			const ndInt32 count = gridScans[i + 1] - start;
			ProccessCell(start, count);
		}
	});

//#ifdef _DEBUG 
#if 0
	ndTree<ndInt32, ndInt32> filter;
	for (ndInt32 i = 0; i < data.m_hashGridMap.GetCount(); ++i)
	{
		if (data.m_hashGridMap[i].m_cellType == ndGridHash::m_homeGrid)
		{
			ndAssert(filter.Insert(ndInt32 (data.m_hashGridMap[i].m_particleIndex)));
		}
	}
#endif	

	threadPool->ParallelExecute(AddPairs);

	//threadPool->ParallelExecute(AddPairs_new);
}

void ndBodySphFluid::CalculateParticlesDensity(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	ndWorkingBuffers& data = *m_workingBuffers;
	data.m_density.SetCount(m_posit.GetCount());
	data.m_invDensity.SetCount(m_posit.GetCount());

	auto CalculateDensity = ndMakeObject::ndFunction([this, &data](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(CalculateDensity);
		const ndArray<ndVector>& posit = m_posit;

		const ndFloat32 h = ndFloat32(1.5f) * ndFloat32(2.0f) * GetParticleRadius();
		const ndFloat32 h2 = h * h;
		const ndFloat32 kernelMagicConst = ndFloat32(315.0f) / (ndFloat32(64.0f) * ndPi * ndPow(h, ndFloat32 (9.0f)));
		const ndFloat32 kernelConst = m_mass * kernelMagicConst;
		const ndFloat32 selfDensity = kernelConst * h2 * h2 * h2;

		const ndStartEnd startEnd(posit.GetCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			const ndInt32 count = data.m_pairCount[i];
			const ndParticleKernelDistance& distance = data.m_kernelDistance[i];
			ndFloat32 density = selfDensity;
			for (ndInt32 j = 0; j < count; ++j)
			{
				const ndFloat32 d = distance.m_dist[j];
				const ndFloat32 dist2 = h2 - d * d;
				ndAssert(dist2 > ndFloat32(0.0f));
				const ndFloat32 dist6 = dist2 * dist2 * dist2;
				density += kernelConst * dist6;
			}
			ndAssert(density > ndFloat32(0.0f));
			data.m_density[i] = density;
			data.m_invDensity[i] = ndFloat32(1.0f) / density;
		}
	});

	threadPool->ParallelExecute(CalculateDensity);
}

void ndBodySphFluid::CalculateAccelerations(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	ndWorkingBuffers& data = *m_workingBuffers;
	data.m_accel.SetCount(m_posit.GetCount());

	auto CalculateAcceleration = ndMakeObject::ndFunction([this, &data](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(CalculateAcceleration);
		const ndVector epsilon2 (ndFloat32(1.0e-12f));

		const ndArray<ndVector>& veloc = m_veloc;
		const ndArray<ndVector>& posit = m_posit;
		const ndFloat32* const density = &data.m_density[0];
		const ndFloat32* const invDensity = &data.m_invDensity[0];

		const ndFloat32 h = ndFloat32(1.5f) * ndFloat32(2.0f) * GetParticleRadius();
		//const ndFloat32 u = m_viscosity;
		const ndVector kernelConst(m_mass * ndFloat32(45.0f) / (ndPi * ndPow(h, ndFloat32 (6.0f))));

		const ndFloat32 viscosity = m_viscosity;
		const ndFloat32 restDensity = m_restDensity;
		const ndFloat32 gasConstant = m_gasConstant;

		const ndVector gravity(m_gravity);
		const ndStartEnd startEnd(posit.GetCount(), threadIndex, threadCount);
		for (ndInt32 i0 = startEnd.m_start; i0 < startEnd.m_end; ++i0)
		{
			const ndVector p0(posit[i0]);
			const ndVector v0(veloc[i0]);

			const ndInt32 count = data.m_pairCount[i0];
			const ndParticlePair& pairs = data.m_pairs[i0];
			ndParticleKernelDistance& distance = data.m_kernelDistance[i0];
			const ndFloat32 pressureI0 = density[i0] - restDensity;

			ndVector forceAcc(ndVector::m_zero);
			for (ndInt32 j = 0; j < count; ++j)
			{
				const ndInt32 i1 = pairs.m_neighborg[j];
				const ndVector p10(posit[i1] - p0);
				const ndVector dot(p10.DotProduct(p10) + epsilon2);
				const ndVector unitDir(p10 * dot.InvSqrt());

				ndAssert(unitDir.m_w == ndFloat32(0.0f));

				// kernel distance
				const ndFloat32 dist = distance.m_dist[j];
				const ndFloat32 kernelDist = h - dist;
				ndAssert(kernelDist >= ndFloat32(0.0f));

				// calculate pressure
				const ndFloat32 kernelDist2 = kernelDist * kernelDist;
				const ndFloat32 pressureI1 = density[i1] - restDensity;
				const ndVector force(gasConstant * kernelDist2 * invDensity[i1] * (pressureI0 + pressureI1));
				forceAcc += force * unitDir;

				// calculate viscosity acceleration
				const ndVector v01(veloc[i1] - v0);
				forceAcc += v01 * ndVector(kernelDist * viscosity * invDensity[j]);
			}
			const ndVector accel(gravity + ndVector(invDensity[i0]) * kernelConst * forceAcc);
			data.m_accel[i0] = accel;
		}
	});

	threadPool->ParallelExecute(CalculateAcceleration);
}

void ndBodySphFluid::IntegrateParticles(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	ndWorkingBuffers& data = *m_workingBuffers;
	auto IntegrateParticles = ndMakeObject::ndFunction([this, &data](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(IntegrateParticles);
		const ndArray<ndVector>& accel = data.m_accel;
		ndArray<ndVector>& veloc = m_veloc;
		ndArray<ndVector>& posit = m_posit;

		//const ndVector timestep (ndFloat32 (0.003f));
		//const ndVector timestep(m_timestep);
		//const ndVector timestep(m_timestep * 0.5f);
		const ndVector timestep(m_timestep * 0.25f);

		const ndStartEnd startEnd(posit.GetCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			veloc[i] = veloc[i] + accel[i] * timestep;
			posit[i] = posit[i] + veloc[i] * timestep;
			if (posit[i].m_y <= 1.0f)
			{
				posit[i].m_y = 1.0f;
				veloc[i].m_y = 0.0f;
			}
		}
	});

	threadPool->ParallelExecute(IntegrateParticles);
}

void ndBodySphFluid::CaculateAabb(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	class ndBox
	{
		public:
		ndBox()
			:m_min(ndFloat32(1.0e10f))
			,m_max(ndFloat32(-1.0e10f))
		{
		}
		ndVector m_min;
		ndVector m_max;
	};

	ndBox boxes[D_MAX_THREADS_COUNT];
	auto CalculateAabb = ndMakeObject::ndFunction([this, &boxes](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(CalculateAabb);
		ndBox box;
		const ndArray<ndVector>& posit = m_posit;
		const ndStartEnd startEnd(posit.GetCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			box.m_min = box.m_min.GetMin(posit[i]);
			box.m_max = box.m_max.GetMax(posit[i]);
		}
		boxes[threadIndex] = box;
	});

	threadPool->ParallelExecute(CalculateAabb);

	ndBox box;
	const ndInt32 threadCount = threadPool->GetThreadCount();
	for (ndInt32 i = 0; i < threadCount; ++i)
	{
		box.m_min = box.m_min.GetMin(boxes[i].m_min);
		box.m_max = box.m_max.GetMax(boxes[i].m_max);
	}

	ndWorkingBuffers& data = *m_workingBuffers;
	//const ndFloat32 diameter = ndFloat32(2.0f) * GetParticleRadius();
	const ndFloat32 diameter = ndFloat32(1.5f) * ndFloat32(2.0f) * GetParticleRadius();
	const ndFloat32 gridSize = diameter * D_GRID_SIZE_SCALER;

	data.m_hashGridSize = gridSize;
	data.m_hashInvGridSize = ndFloat32(1.0f) / gridSize;

	const ndVector grid(data.m_hashGridSize);
	const ndVector invGrid(data.m_hashInvGridSize);

	// add one grid padding to the aabb
	box.m_min -= grid;
	box.m_max += (grid + grid);

	// quantize the aabb to integers of the gird size
	box.m_min = grid * (box.m_min * invGrid).Floor();
	box.m_max = grid * (box.m_max * invGrid).Floor();

	// make sure the w component is zero.
	m_box0 = box.m_min & ndVector::m_triplexMask;
	m_box1 = box.m_max & ndVector::m_triplexMask;
	data.SetWorldToGridMapping(gridSize, m_box1, m_box0);
}

void ndBodySphFluid::CreateGrids(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	class ndGridNeighborInfo
	{
		public:
		ndGridNeighborInfo()
		{
			//ndGridHash stepsCode;
			m_neighborDirs[0][0] = ndGridHash(0, 0);
			m_neighborDirs[0][1] = ndGridHash(0, 0);
			m_neighborDirs[0][2] = ndGridHash(0, 0);
			m_neighborDirs[0][3] = ndGridHash(0, 0);

			m_counter[0] = 1;
			m_isPadd[0][0] = 0;
			m_isPadd[0][1] = 1;
			m_isPadd[0][2] = 1;
			m_isPadd[0][3] = 1;

			//ndGridHash stepsCode_y;
			m_neighborDirs[1][0] = ndGridHash(0, 0);
			m_neighborDirs[1][1] = ndGridHash(1, 0);
			m_neighborDirs[1][2] = ndGridHash(0, 0);
			m_neighborDirs[1][3] = ndGridHash(0, 0);

			m_counter[1] = 2;
			m_isPadd[1][0] = 0;
			m_isPadd[1][1] = 0;
			m_isPadd[1][2] = 1;
			m_isPadd[1][3] = 1;

			//ndGridHash stepsCode_z;
			m_neighborDirs[2][0] = ndGridHash(0, 0);
			m_neighborDirs[2][1] = ndGridHash(0, 1);
			m_neighborDirs[2][2] = ndGridHash(0, 0);
			m_neighborDirs[2][3] = ndGridHash(0, 0);

			m_counter[2] = 2;
			m_isPadd[2][0] = 0;
			m_isPadd[2][1] = 0;
			m_isPadd[2][2] = 1;
			m_isPadd[2][3] = 1;

			//ndGridHash stepsCode_yz;
			m_neighborDirs[3][0] = ndGridHash(0, 0);
			m_neighborDirs[3][1] = ndGridHash(1, 0);
			m_neighborDirs[3][2] = ndGridHash(0, 1);
			m_neighborDirs[3][3] = ndGridHash(1, 1);

			m_counter[3] = 4;
			m_isPadd[3][0] = 0;
			m_isPadd[3][1] = 0;
			m_isPadd[3][2] = 0;
			m_isPadd[3][3] = 0;
		}

		ndGridHash m_neighborDirs[4][4];
		ndInt8 m_isPadd[4][4];
		ndInt8 m_counter[4];
	};
	
	ndGridNeighborInfo neiborghood;
	ndWorkingBuffers& data = *m_workingBuffers;
	
	auto CountGrids = ndMakeObject::ndFunction([this, &data, &neiborghood](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(CountGrids);
		const ndVector origin(m_box0);
		const ndVector invGridSize(data.m_hashInvGridSize);
		//const ndVector particleBox(ndFloat32(1.5f) * GetParticleRadius());
		const ndVector particleBox(ndFloat32(0.5f) * data.m_hashGridSize);
		
		const ndVector* const posit = &m_posit[0];
		ndInt32* const scans = &data.m_gridScans[0];

		const ndStartEnd startEnd(m_posit.GetCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			const ndVector gridPosit(posit[i] - origin);
			const ndVector p0(gridPosit - particleBox);
			const ndVector p1(gridPosit + particleBox);
			const ndGridHash box0Hash(p0 * invGridSize, i);
			const ndGridHash box1Hash(p1 * invGridSize, i);
			const ndGridHash codeHash(box1Hash.m_gridHash - box0Hash.m_gridHash);
			
#ifdef D_USE_YZ_PLANE_BUCKETS
			ndAssert(codeHash.m_y <= 1);
			ndAssert(codeHash.m_z <= 1);
			const ndUnsigned32 code = ndUnsigned32(codeHash.m_z * 2 + codeHash.m_y);
#else
			ndAssert(codeHash.m_x <= 1);
			ndAssert(codeHash.m_z <= 1);
			const ndUnsigned32 code = ndUnsigned32(codeHash.m_z * 2 + codeHash.m_x);
#endif
			scans[i] = neiborghood.m_counter[code];
		}
	});

	auto CreateGrids = ndMakeObject::ndFunction([this, &data, &neiborghood](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(CreateGrids);
		const ndVector origin(m_box0);
		ndGridHash* const dst = &data.m_hashGridMap[0];
		const ndInt32* const scans = &data.m_gridScans[0];
		const ndVector* const posit = &m_posit[0];
		const ndVector invGridSize(data.m_hashInvGridSize);
		//const ndVector particleBox(ndFloat32(1.5f)* GetParticleRadius());
		const ndVector particleBox(ndFloat32(0.5f)* data.m_hashGridSize);

		const ndStartEnd startEnd(m_posit.GetCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			const ndVector gridPosit(posit[i] - origin);
			const ndVector gridPosit0(gridPosit - particleBox);
			const ndVector gridPosit1(gridPosit + particleBox);
			const ndGridHash centerHash(gridPosit* invGridSize, i);
			const ndGridHash box0Hash(gridPosit0 * invGridSize, i);
			const ndGridHash box1Hash(gridPosit1 * invGridSize, i);
			const ndGridHash codeHash(box1Hash.m_gridHash - box0Hash.m_gridHash);

			const ndInt32 base = scans[i];
			const ndInt32 count = scans[i + 1] - base;

#ifdef D_USE_YZ_PLANE_BUCKETS
			ndAssert(codeHash.m_y <= 1);
			ndAssert(codeHash.m_z <= 1);
			const ndInt32 code = ndInt32(codeHash.m_z * 2 + codeHash.m_y);
#else
			ndAssert(codeHash.m_x <= 1);
			ndAssert(codeHash.m_z <= 1);
			const ndInt32 code = ndInt32(codeHash.m_z * 2 + codeHash.m_x);
#endif
			const ndGridHash* const neigborgh = &neiborghood.m_neighborDirs[code][0];
			ndAssert(count == neiborghood.m_counter[code]);
			for (ndInt32 j = 0; j < count; ++ j)
			{
				ndGridHash quadrand(box0Hash);
				quadrand.m_gridHash += neigborgh[j].m_gridHash;
				quadrand.m_cellType = ndGridHash::ndGridType(quadrand.m_gridHash == centerHash.m_gridHash);
				ndAssert(quadrand.m_cellType == ((quadrand.m_gridHash == centerHash.m_gridHash) ? ndGridHash::m_homeGrid : ndGridHash::m_adjacentGrid));
				dst[base + j] = quadrand;
			}
		}
	});

	data.m_gridScans.SetCount(m_posit.GetCount() + 1);
	data.m_gridScans[m_posit.GetCount()] = 0;
	threadPool->ParallelExecute(CountGrids);
	
	ndInt32 gridCount = 0;
	const ndInt32 itemsCount = data.m_gridScans.GetCount() & (-8);
	for (ndInt32 i = 0; i < itemsCount; i += 8)
	{
		for (ndInt32 j = 0; j < 8; ++j)
		{
			ndInt32 count = data.m_gridScans[i + j];
			data.m_gridScans[i + j] = gridCount;
			gridCount += count;
		}
	}
	for (ndInt32 j = itemsCount; j < data.m_gridScans.GetCount(); ++j)
	{
		ndInt32 count = data.m_gridScans[j];
		data.m_gridScans[j] = gridCount;
		gridCount += count;
	}
	
	data.m_hashGridMap.SetCount(gridCount);
	threadPool->ParallelExecute(CreateGrids);
	data.m_hashGridMapScratchBuffer.SetCount(gridCount);

	//ndAssert(TraceHashes());
}

bool ndBodySphFluid::TraceHashes() const
{
#if 0
	ndWorkingBuffers& data = *m_workingBuffers;
	ndGridHash* xxxx = &data.m_hashGridMap[0];
	for (ndInt32 i = 0; i < data.m_hashGridMap.GetCount(); i++)
	{
		ndTrace(("id(%d)\tx(%d)\tz(%d)\n", xxxx[i].m_particleIndex, xxxx[i].m_x, xxxx[i].m_z));
	}
#endif

	return true;
}

void ndBodySphFluid::Update(const ndScene* const scene, ndFloat32 timestep)
{
	if (TaskState() == ndBackgroundTask::m_taskCompleted)
	{
		if (m_posit.GetCount())
		{
			m_timestep = timestep;
			((ndScene*)scene)->SendBackgroundTask(this);
			if (!m_updateInBackground)
			{
				Sync();
			}
		}
	}
}

void ndBodySphFluid::Execute(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	ndAssert(sizeof(ndGridHash) == sizeof(ndUnsigned64));

	CaculateAabb(threadPool);
	CreateGrids(threadPool);
	SortGrids(threadPool);
	CalculateScans(threadPool);
	BuildBuckets(threadPool);
	CalculateParticlesDensity(threadPool);
	CalculateAccelerations(threadPool);
	IntegrateParticles(threadPool);
}

#else

class ndBodySphFluid::ndGridHash
{
	public:
	enum ndGridType
	{
		m_adjacentGrid = 0,
		m_homeGrid = 1,
	};

	ndGridHash()
	{
	}

	ndGridHash(ndUnsigned64 gridHash)
		:m_gridHash(gridHash)
	{
	}

#ifdef D_USE_YZ_PLANE_BUCKETS
	ndGridHash(ndInt32 y, ndInt32 z)
	{
		m_gridHash = 0;
		m_y = ndUnsigned64(y);
		m_z = ndUnsigned64(z);
	}

	ndGridHash(const ndVector& grid, ndInt32 particleIndex)
	{
		ndAssert(grid.m_y >= ndFloat32(0.0f));
		ndAssert(grid.m_z >= ndFloat32(0.0f));
		ndAssert(grid.m_y < ndFloat32(1 << (D_SPH_HASH_BITS * 2)));
		ndAssert(grid.m_z < ndFloat32(1 << (D_SPH_HASH_BITS * 2)));

		ndVector hash(grid.GetInt());

		m_gridHash = 0;
		m_y = ndUnsigned64(hash.m_iy);
		m_z = ndUnsigned64(hash.m_iz);

		m_cellType = m_adjacentGrid;
		m_particleIndex = ndUnsigned64(particleIndex);
	}
#else
	ndGridHash(ndInt32 x, ndInt32 z)
	{
		m_gridHash = 0;
		m_x = ndUnsigned64(x);
		m_z = ndUnsigned64(z);
	}

	ndGridHash(const ndVector& grid, ndInt32 particleIndex)
	{
		ndAssert(grid.m_x >= ndFloat32(0.0f));
		ndAssert(grid.m_z >= ndFloat32(0.0f));
		ndAssert(grid.m_x < ndFloat32(1 << (D_SPH_HASH_BITS * 2)));
		ndAssert(grid.m_z < ndFloat32(1 << (D_SPH_HASH_BITS * 2)));

		ndVector hash(grid.GetInt());

		m_gridHash = 0;
		m_x = ndUnsigned64(hash.m_ix);
		m_z = ndUnsigned64(hash.m_iz);

		m_cellType = m_adjacentGrid;
		m_particleIndex = ndUnsigned64(particleIndex);
	}
#endif

	union
	{
		struct
		{
#ifdef D_USE_YZ_PLANE_BUCKETS
			ndUnsigned64 m_y : D_SPH_HASH_BITS * 2;
			ndUnsigned64 m_z : D_SPH_HASH_BITS * 2;
#else
			ndUnsigned64 m_x : D_SPH_HASH_BITS * 2;
			ndUnsigned64 m_z : D_SPH_HASH_BITS * 2;
#endif
			ndUnsigned64 m_particleIndex : 23;
			ndUnsigned64 m_cellType : 1;
		};
		struct
		{
#ifdef D_USE_YZ_PLANE_BUCKETS
			ndUnsigned64 m_yLow : D_SPH_HASH_BITS;
			ndUnsigned64 m_yHigh : D_SPH_HASH_BITS;
			ndUnsigned64 m_zLow : D_SPH_HASH_BITS;
			ndUnsigned64 m_zHigh : D_SPH_HASH_BITS;
#else
			ndUnsigned64 m_xLow : D_SPH_HASH_BITS;
			ndUnsigned64 m_xHigh : D_SPH_HASH_BITS;
			ndUnsigned64 m_zLow : D_SPH_HASH_BITS;
			ndUnsigned64 m_zHigh : D_SPH_HASH_BITS;
#endif
		};
		ndUnsigned64 m_gridHash : D_SPH_HASH_BITS * 2 * 2;
	};
};

class ndBodySphFluid::ndParticlePair
{
	public:
	ndInt32 m_neighborg[D_PARTICLE_BUCKET_SIZE];
};

class ndBodySphFluid::ndParticleKernelDistance
{
	public:
	ndFloat32 m_dist[D_PARTICLE_BUCKET_SIZE];
};

class ndBodySphFluid::ndWorkingBuffers
{
#define D_SPH_GRID_X_RESOLUTION 4

	public:
	ndWorkingBuffers()
		:m_accel(D_SPH_BUFFER_GRANULARITY)
		, m_locks(D_SPH_BUFFER_GRANULARITY)
		, m_pairCount(D_SPH_BUFFER_GRANULARITY)
		, m_gridScans(D_SPH_BUFFER_GRANULARITY)
		, m_density(D_SPH_BUFFER_GRANULARITY)
		, m_invDensity(D_SPH_BUFFER_GRANULARITY)
		, m_pairs(D_SPH_BUFFER_GRANULARITY)
		, m_hashGridMap(D_SPH_BUFFER_GRANULARITY)
		, m_hashGridMapScratchBuffer(D_SPH_BUFFER_GRANULARITY)
		, m_kernelDistance(D_SPH_BUFFER_GRANULARITY)
		, m_worlToGridOrigin(ndFloat32(1.0f))
		, m_worlToGridScale(ndFloat32(1.0f))
		, m_hashGridSize(ndFloat32(0.0f))
		, m_hashInvGridSize(ndFloat32(0.0f))
		, m_particleDiameter(ndFloat32(0.0f))
	{
		for (ndInt32 i = 0; i < D_MAX_THREADS_COUNT; ++i)
		{
			m_partialsGridScans[i].Resize(D_SPH_BUFFER_GRANULARITY);
		}
	}

	~ndWorkingBuffers()
	{
	}

	void SetWorldToGridMapping(ndFloat32 gridSize, const ndVector& maxP, const ndVector& minP)
	{
#ifdef D_USE_YZ_PLANE_BUCKETS
		m_worlToGridOrigin = minP.m_x;
		ndFloat32 gridCount = ndFloor((maxP.m_x - minP.m_x) / gridSize + ndFloat32(1.0f));
		m_worlToGridScale = ndFloat32(1 << D_SPH_GRID_X_RESOLUTION) * gridCount / (maxP.m_x - minP.m_x);
#else
		m_worlToGridOrigin = minP.m_y;
		ndFloat32 gridCount = ndFloor((maxP.m_y - minP.m_y) / gridSize + ndFloat32(1.0f));
		m_worlToGridScale = ndFloat32(1 << D_SPH_GRID_X_RESOLUTION) * (ndFloat32)gridCount / (maxP.m_y - minP.m_y);
#endif
	}

	ndInt32 WorldToGrid(const ndVector& point) const
	{
#ifdef D_USE_YZ_PLANE_BUCKETS
		ndInt32 val = ndInt32((point.m_x - m_worlToGridOrigin) * m_worlToGridScale);
#else
		ndInt32 val = ndInt32((point.m_y - m_worlToGridOrigin) * m_worlToGridScale);
#endif
		ndAssert(val >= 0);
		return val;
	}

	ndArray<ndVector> m_accel;
	ndArray<ndSpinLock> m_locks;
	ndArray<ndInt8> m_pairCount;
	ndArray<ndInt32> m_gridScans;
	ndArray<ndFloat32> m_density;
	ndArray<ndFloat32> m_invDensity;
	ndArray<ndParticlePair> m_pairs;
	ndArray<ndGridHash> m_hashGridMap;
	ndArray<ndGridHash> m_hashGridMapScratchBuffer;
	ndArray<ndParticleKernelDistance> m_kernelDistance;
	ndArray<ndInt32> m_partialsGridScans[D_MAX_THREADS_COUNT];
	ndFloat32 m_worlToGridOrigin;
	ndFloat32 m_worlToGridScale;
	ndFloat32 m_hashGridSize;
	ndFloat32 m_hashInvGridSize;
	ndFloat32 m_particleDiameter;
};

ndBodySphFluid::ndBodySphFluid()
	:ndBodyParticleSet()
	,m_workingBuffers(new ndWorkingBuffers)
	,m_mass(ndFloat32(1.0f))
	,m_viscosity(ndFloat32(1.05f))
	,m_restDensity(ndFloat32(1000.0f))
	,m_gasConstant(ndFloat32(1.0f))
{
	SetRestDensity(m_restDensity);
}

ndBodySphFluid::~ndBodySphFluid()
{
	delete m_workingBuffers;
}

void ndBodySphFluid::SortBuckets(ndThreadPool* const threadPool)
{
	D_TRACKTIME();

	class ndKey_low
	{
		public:
		ndKey_low(void* const context)
			:m_fluid((ndBodySphFluid*)context)
			, m_data(*m_fluid->m_workingBuffers)
			, m_point(m_fluid->GetPositions())
		{
		}

		ndInt32 GetKey(const ndGridHash& cell) const
		{
			ndInt32 index = ndInt32(cell.m_particleIndex);
			ndUnsigned32 key = ndUnsigned32(m_data.WorldToGrid(m_point[index]));
			return ndInt32(key & 0xff);
		}

		ndBodySphFluid* m_fluid;
		ndWorkingBuffers& m_data;
		const ndArray<ndVector>& m_point;
	};

	class ndKey_middle
	{
		public:
		ndKey_middle(void* const context)
			:m_fluid((ndBodySphFluid*)context)
			, m_data(*m_fluid->m_workingBuffers)
			, m_point(m_fluid->GetPositions())
		{
		}

		ndInt32 GetKey(const ndGridHash& cell) const
		{
			ndInt32 index = ndInt32(cell.m_particleIndex);
			ndUnsigned32 key = ndUnsigned32(m_data.WorldToGrid(m_point[index]));
			return ndInt32((key >> 8) & 0xff);
		}

		ndBodySphFluid* m_fluid;
		ndWorkingBuffers& m_data;
		const ndArray<ndVector>& m_point;
	};

	class ndKey_high
	{
		public:
		ndKey_high(void* const context)
			:m_fluid((ndBodySphFluid*)context)
			, m_data(*m_fluid->m_workingBuffers)
			, m_point(m_fluid->GetPositions())
		{
		}

		ndInt32 GetKey(const ndGridHash& cell) const
		{
			ndInt32 index = ndInt32(cell.m_particleIndex);
			ndUnsigned32 key = ndUnsigned32(m_data.WorldToGrid(m_point[index]));
			return ndInt32((key >> 16) & 0xff);
		}

		ndBodySphFluid* m_fluid;
		ndWorkingBuffers& m_data;
		const ndArray<ndVector>& m_point;
	};

	ndWorkingBuffers& data = *m_workingBuffers;
	const ndInt32 keySize = data.WorldToGrid(m_box1);

	ndCountingSort<ndGridHash, ndKey_low, 8>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, nullptr, this);
	if (keySize >= 256)
	{
		ndCountingSort<ndGridHash, ndKey_middle, 8>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, nullptr, this);
	}
	if (keySize >= (256 * 256))
	{
		ndCountingSort<ndGridHash, ndKey_high, 8>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, nullptr, this);
	}

#ifdef _DEBUG
	const ndArray<ndVector>& point = GetPositions();
	for (ndInt32 i = 1; i < data.m_hashGridMap.GetCount(); ++i)
	{
		ndGridHash cell0(data.m_hashGridMap[i - 1]);
		ndGridHash cell1(data.m_hashGridMap[i + 0]);
		const ndVector p0(point[ndInt32(cell0.m_particleIndex)]);
		const ndVector p1(point[ndInt32(cell1.m_particleIndex)]);
		ndInt32 key0 = data.WorldToGrid(p0);
		ndInt32 key1 = data.WorldToGrid(p1);
		ndAssert(key0 <= key1);
	}
#endif
}

void ndBodySphFluid::SortCellBuckects(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
#ifdef D_USE_YZ_PLANE_BUCKETS
	class ndKey_ylow
	{
		public:
		ndKey_ylow(void* const) {}
		ndInt32 GetKey(const ndGridHash& cell) const
		{
			return ndInt32(cell.m_yLow);
		}
	};

	class ndKey_yhigh
	{
		public:
		ndKey_yhigh(void* const) {}
		ndInt32 GetKey(const ndGridHash& cell) const
		{
			return ndInt32(cell.m_yHigh);
		}
	};
#else
	class ndKey_xlow
	{
		public:
		ndKey_xlow(void* const) {}
		ndInt32 GetKey(const ndGridHash& cell) const
		{
			return ndInt32(cell.m_xLow);
		}
	};

	class ndKey_xhigh
	{
		public:
		ndKey_xhigh(void* const) {}
		ndInt32 GetKey(const ndGridHash& cell) const
		{
			return ndInt32(cell.m_xHigh);
		}
	};

#endif

	class ndKey_zlow
	{
		public:
		ndKey_zlow(void* const) {}
		ndInt32 GetKey(const ndGridHash& cell) const
		{
			return ndInt32(cell.m_zLow);
		}
	};

	class ndKey_zhigh
	{
		public:
		ndKey_zhigh(void* const) {}
		ndInt32 GetKey(const ndGridHash& cell) const
		{
			return ndInt32(cell.m_zHigh);
		}
	};

	ndWorkingBuffers& data = *m_workingBuffers;
	const ndVector boxSize((m_box1 - m_box0).Scale(data.m_hashInvGridSize).GetInt());

#ifdef D_USE_YZ_PLANE_BUCKETS
	ndCountingSort<ndGridHash, ndKey_ylow, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, nullptr, nullptr);
	if (boxSize.m_iy > (1 << D_SPH_HASH_BITS))
	{
		ndCountingSort<ndGridHash, ndKey_yhigh, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, nullptr, nullptr);
	}
#else
	ndCountingSort<ndGridHash, ndKey_xlow, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, nullptr, nullptr);
	if (boxSize.m_ix > (1 << D_SPH_HASH_BITS))
	{
		ndCountingSort<ndGridHash, ndKey_xhigh, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, nullptr, nullptr);
	}
#endif

	ndCountingSort<ndGridHash, ndKey_zlow, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, nullptr, nullptr);
	if (boxSize.m_iz > (1 << D_SPH_HASH_BITS))
	{
		ndCountingSort<ndGridHash, ndKey_zhigh, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, nullptr, nullptr);
	}

#ifdef _DEBUG
	for (ndInt32 i = 1; i < data.m_hashGridMap.GetCount(); ++i)
	{
		ndGridHash cell0(data.m_hashGridMap[i - 1]);
		ndGridHash cell1(data.m_hashGridMap[i + 0]);
#ifdef D_USE_YZ_PLANE_BUCKETS
		ndUnsigned64 key0 = (cell0.m_z << (D_SPH_HASH_BITS * 2)) + cell0.m_y;
		ndUnsigned64 key1 = (cell1.m_z << (D_SPH_HASH_BITS * 2)) + cell1.m_y;
#else
		ndUnsigned64 key0 = (cell0.m_z << (D_SPH_HASH_BITS * 2)) + cell0.m_x;
		ndUnsigned64 key1 = (cell1.m_z << (D_SPH_HASH_BITS * 2)) + cell1.m_x;
#endif
		ndAssert(key0 <= key1);
	}
#endif
}

void ndBodySphFluid::CalculateScans(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	ndWorkingBuffers& data = *m_workingBuffers;
	ndInt32 sums[D_MAX_THREADS_COUNT + 1];
	ndInt32 scans[D_MAX_THREADS_COUNT + 1];

	auto CountGridScans = ndMakeObject::ndFunction([&data, &scans](ndInt32 threadIndex, ndInt32)
	{
		D_TRACKTIME_NAMED(CountGridScans);
		const ndGridHash* const hashGridMap = &data.m_hashGridMap[0];

		const ndInt32 start = scans[threadIndex];
		const ndInt32 end = scans[threadIndex + 1];
		ndArray<ndInt32>& gridScans = data.m_partialsGridScans[threadIndex];
		ndUnsigned64 gridHash0 = hashGridMap[start].m_gridHash;

		ndInt32 count = 0;
		gridScans.SetCount(0);
		for (ndInt32 i = start; i < end; ++i)
		{
			ndUnsigned64 gridHash = hashGridMap[i].m_gridHash;
			if (gridHash != gridHash0)
			{
				gridScans.PushBack(count);
				count = 0;
				gridHash0 = gridHash;
			}
			count++;
		}
		gridScans.PushBack(count);
	});

	auto CalculateScans = ndMakeObject::ndFunction([&data, &scans, &sums](ndInt32 threadIndex, ndInt32)
	{
		D_TRACKTIME_NAMED(CalculateScans);
		ndArray<ndInt32>& gridScans = data.m_gridScans;
		const ndArray<ndInt32>& partialScan = data.m_partialsGridScans[threadIndex];
		const ndInt32 base = sums[threadIndex];
		ndInt32 sum = scans[threadIndex];
		for (ndInt32 i = 0; i < partialScan.GetCount(); ++i)
		{
			gridScans[base + i] = sum;
			sum += partialScan[i];
		}
	});

	memset(scans, 0, sizeof(scans));
	const ndInt32 threadCount = threadPool->GetThreadCount();

	ndInt32 particleCount = ndInt32(data.m_hashGridMap.GetCount());

	ndInt32 acc0 = 0;
	ndInt32 stride = particleCount / threadCount;
	const ndGridHash* const hashGridMap = &data.m_hashGridMap[0];
	for (ndInt32 threadIndex = 0; threadIndex < threadCount; threadIndex++)
	{
		scans[threadIndex] = acc0;
		acc0 += stride;
		while (acc0 < particleCount && (hashGridMap[acc0].m_gridHash == hashGridMap[acc0 - 1].m_gridHash))
		{
			acc0++;
		}
	}
	scans[threadCount] = particleCount;
	threadPool->ParallelExecute(CountGridScans);

	ndInt32 scansCount = 0;

	for (ndInt32 i = 0; i < threadCount; ++i)
	{
		sums[i] = scansCount;
		scansCount += ndInt32(data.m_partialsGridScans[i].GetCount());
	}
	sums[threadCount] = scansCount;

	data.m_gridScans.SetCount(scansCount + 1);
	threadPool->ParallelExecute(CalculateScans);

	data.m_gridScans[scansCount] = scans[threadCount];
}

void ndBodySphFluid::SortGrids(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	SortBuckets(threadPool);
	SortCellBuckects(threadPool);

#ifdef _DEBUG
	ndWorkingBuffers& data = *m_workingBuffers;
	for (ndInt32 i = 0; i < (data.m_hashGridMap.GetCount() - 1); ++i)
	{
		const ndGridHash& entry0 = data.m_hashGridMap[i + 0];
		const ndGridHash& entry1 = data.m_hashGridMap[i + 1];
		ndUnsigned64 gridHashA = entry0.m_gridHash;
		ndUnsigned64 gridHashB = entry1.m_gridHash;
		ndAssert(gridHashA <= gridHashB);
	}
#endif
}

//void ndBodySphFluid::BuildBuckets(ndThreadPool* const threadPool)
void ndBodySphFluid::BuildBuckets(ndThreadPool* const)
{
    //#ifdef _DEBUG
#if 0
    D_TRACKTIME();
    ndWorkingBuffers& data = *m_workingBuffers;
    ndInt32 countReset = ndInt32(data.m_locks.GetCount());
    data.m_pairs.SetCount(m_posit.GetCount());
    data.m_locks.SetCount(m_posit.GetCount());
    data.m_pairCount.SetCount(m_posit.GetCount());
    data.m_kernelDistance.SetCount(m_posit.GetCount());
    for (ndInt32 i = countReset; i < data.m_locks.GetCount(); ++i)
    {
        data.m_locks[i].Unlock();
    }
    for (ndInt32 i = 0; i < data.m_pairCount.GetCount(); ++i)
    {
        data.m_pairCount[i] = 0;
    }
    
    auto AddPairs = ndMakeObject::ndFunction([this, &data](ndInt32 threadIndex, ndInt32 threadCount)
                                             {
        D_TRACKTIME_NAMED(AddPairs);
        const ndArray<ndGridHash>& hashGridMap = data.m_hashGridMap;
        const ndArray<ndInt32>& gridScans = data.m_gridScans;
        //const ndFloat32 diameter = ndFloat32(1.5f) * ndFloat32(2.0f) * GetParticleRadius();
        const ndFloat32 diameter = data.m_particleDiameter;
        const ndFloat32 diameter2 = diameter * diameter;
        const ndInt32 windowsTest = data.WorldToGrid(ndVector(data.m_worlToGridOrigin + diameter)) + 1;
        
        ndArray<ndSpinLock>& locks = data.m_locks;
        ndArray<ndInt8>& pairCount = data.m_pairCount;
        ndArray<ndParticlePair>& pair = data.m_pairs;
        ndArray<ndParticleKernelDistance>& distance = data.m_kernelDistance;
        
        auto ProccessCell = [this, &data, &hashGridMap, &pair, &pairCount, &locks, &distance, windowsTest, diameter2](ndInt32 start, ndInt32 count)
        {
            const ndInt32 count0 = count - 1;
            for (ndInt32 i = 0; i < count0; ++i)
            {
                const ndGridHash hash0 = hashGridMap[start + i];
                const ndInt32 particle0 = ndInt32(hash0.m_particleIndex);
                const ndInt32 x0 = data.WorldToGrid(m_posit[particle0]);
                const bool homeGridTest0 = (hash0.m_cellType == ndGridHash::m_homeGrid);
                for (ndInt32 j = i + 1; j < count; ++j)
                {
                    const ndGridHash hash1 = hashGridMap[start + j];
                    const ndInt32 particle1 = ndInt32(hash1.m_particleIndex);
                    ndAssert(particle0 != particle1);
                    const ndInt32 x1 = data.WorldToGrid(m_posit[particle1]);
                    const ndInt32 sweeptTest = ((x1 - x0) >= windowsTest);
                    if (sweeptTest)
                    {
                        break;
                    }
                    ndAssert(particle0 != particle1);
                    const bool homeGridTest1 = (hash1.m_cellType == ndGridHash::m_homeGrid);
                    const ndInt32 test = homeGridTest0 | homeGridTest1;
                    if (test)
                    {
                        const ndVector p1p0(m_posit[particle0] - m_posit[particle1]);
                        const ndFloat32 dist2(p1p0.DotProduct(p1p0).GetScalar());
                        if (dist2 <= diameter2)
                        {
                            const ndFloat32 dist = ndSqrt(ndMax(dist2, ndFloat32(1.0e-8f)));
                            {
                                ndSpinLock lock(locks[particle0]);
                                ndInt8 neigborCount = pairCount[particle0];
                                if (neigborCount < D_PARTICLE_BUCKET_SIZE)
                                {
                                    ndInt8 isUnique = 1;
                                    ndInt32* const neighborg = pair[particle0].m_neighborg;
                                    for (ndInt32 k = neigborCount - 1; k >= 0; --k)
                                    {
                                        isUnique = isUnique & (neighborg[k] != particle1);
                                    }
                                    //ndAssert(isUnique);
                                    
                                    neighborg[neigborCount] = particle1;
                                    distance[particle0].m_dist[neigborCount] = dist;
                                    pairCount[particle0] = neigborCount + isUnique;
                                }
                            }
                            
                            {
                                ndSpinLock lock(locks[particle1]);
                                ndInt8 neigborCount = pairCount[particle1];
                                if (neigborCount < D_PARTICLE_BUCKET_SIZE)
                                {
                                    ndInt8 isUnique = 1;
                                    ndInt32* const neighborg = pair[particle1].m_neighborg;
                                    for (ndInt32 k = neigborCount - 1; k >= 0; --k)
                                    {
                                        isUnique = isUnique & (neighborg[k] != particle0);
                                    }
                                    //ndAssert(isUnique);
                                    
                                    neighborg[neigborCount] = particle0;
                                    distance[particle1].m_dist[neigborCount] = dist;
                                    pairCount[particle1] = neigborCount + isUnique;
                                }
                            }
                        }
                    }
                }
            }
        };
        
        const ndInt32 scansCount = ndInt32(gridScans.GetCount()) - 1;
        for (ndInt32 i = threadIndex; i < scansCount; i += threadCount)
        {
            const ndInt32 start = gridScans[i];
            const ndInt32 count = gridScans[i + 1] - start;
            ProccessCell(start, count);
        }
    });
    
    auto AddPairs_new = ndMakeObject::ndFunction([this, &data](ndInt32 threadIndex, ndInt32 threadCount)
                                                 {
        D_TRACKTIME_NAMED(AddPairs);
        const ndArray<ndGridHash>& hashGridMap = data.m_hashGridMap;
        const ndArray<ndInt32>& gridScans = data.m_gridScans;
        const ndFloat32 diameter = data.m_particleDiameter;
        const ndFloat32 diameter2 = diameter * diameter;
        const ndInt32 windowsTest = data.WorldToGrid(ndVector(data.m_worlToGridOrigin + diameter)) + 1;
        
        ndArray<ndSpinLock>& locks = data.m_locks;
        ndArray<ndInt8>& pairCount = data.m_pairCount;
        ndArray<ndParticlePair>& pair = data.m_pairs;
        ndArray<ndParticleKernelDistance>& distance = data.m_kernelDistance;
        
        auto ProccessCell = [this, &data, &hashGridMap, &pair, &pairCount, &locks, &distance, windowsTest, diameter2](ndInt32 start, ndInt32 count)
        {
            const ndInt32 count0 = count - 1;
            for (ndInt32 i = 0; i < count0; ++i)
            {
                const ndGridHash hash0 = hashGridMap[start + i];
                const ndInt32 particle0 = ndInt32(hash0.m_particleIndex);
                const ndInt32 x0 = data.WorldToGrid(m_posit[particle0]);
                const bool homeGridTest0 = (hash0.m_cellType == ndGridHash::m_homeGrid);
                for (ndInt32 j = i + 1; j < count; ++j)
                {
                    const ndGridHash hash1 = hashGridMap[start + j];
                    const ndInt32 particle1 = ndInt32(hash1.m_particleIndex);
                    ndAssert(particle0 != particle1);
                    const ndInt32 x1 = data.WorldToGrid(m_posit[particle1]);
                    const ndInt32 sweeptTest = ((x1 - x0) >= windowsTest);
                    if (sweeptTest)
                    {
                        break;
                    }
                    ndAssert(particle0 != particle1);
                    const ndVector p1p0(m_posit[particle0] - m_posit[particle1]);
                    const ndFloat32 dist2(p1p0.DotProduct(p1p0).GetScalar());
                    if (dist2 < diameter2)
                    {
                        const bool homeGridTest1 = (hash1.m_cellType == ndGridHash::m_homeGrid);
                        if (homeGridTest0 && homeGridTest1)
                        {
                            ndInt8 neigborCount0 = pairCount[particle0];
                            const ndFloat32 dist = ndSqrt(ndMax(dist2, ndFloat32(1.0e-8f)));
                            if (neigborCount0 < D_PARTICLE_BUCKET_SIZE)
                            {
                                pair[particle0].m_neighborg[neigborCount0] = particle1;
                                distance[particle0].m_dist[neigborCount0] = dist;
                                pairCount[particle0] = neigborCount0 + 1;
                            }
                            
                            ndInt8 neigborCount1 = pairCount[particle1];
                            if (neigborCount1 < D_PARTICLE_BUCKET_SIZE)
                            {
                                pair[particle1].m_neighborg[neigborCount1] = particle0;
                                distance[particle1].m_dist[neigborCount1] = dist;
                                pairCount[particle1] = neigborCount1 + 1;
                            }
                            
                        }
                        else if (homeGridTest0)
                        {
                            ndAssert(!homeGridTest1);
                            ndInt8 neigborCount0 = pairCount[particle0];
                            const ndFloat32 dist = ndSqrt(ndMax(dist2, ndFloat32(1.0e-8f)));
                            if (neigborCount0 < D_PARTICLE_BUCKET_SIZE)
                            {
                                pair[particle0].m_neighborg[neigborCount0] = particle1;
                                distance[particle0].m_dist[neigborCount0] = dist;
                                pairCount[particle0] = neigborCount0 + 1;
                            }
                        }
                        else if (homeGridTest1)
                        {
                            ndAssert(!homeGridTest0);
                            ndInt8 neigborCount1 = pairCount[particle1];
                            const ndFloat32 dist = ndSqrt(ndMax(dist2, ndFloat32(1.0e-8f)));
                            if (neigborCount1 < D_PARTICLE_BUCKET_SIZE)
                            {
                                pair[particle1].m_neighborg[neigborCount1] = particle0;
                                distance[particle1].m_dist[neigborCount1] = dist;
                                pairCount[particle1] = neigborCount1 + 1;
                            }
                        }
                    }
                }
            }
        };
        
        const ndInt32 scansCount = ndInt32(gridScans.GetCount()) - 1;
        for (ndInt32 i = threadIndex; i < scansCount; i += threadCount)
        {
            const ndInt32 start = gridScans[i];
            const ndInt32 count = gridScans[i + 1] - start;
            ProccessCell(start, count);
        }
    });
    
    ndTree<ndInt32, ndInt32> filter;
    for (ndInt32 i = 0; i < data.m_hashGridMap.GetCount(); ++i)
    {
        if (data.m_hashGridMap[i].m_cellType == ndGridHash::m_homeGrid)
        {
            ndAssert(filter.Insert(ndInt32(data.m_hashGridMap[i].m_particleIndex)));
        }
    }
    
    
    threadPool->ParallelExecute(AddPairs);
    //threadPool->ParallelExecute(AddPairs_new);
#endif    
}

void ndBodySphFluid::CalculateParticlesDensity(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	ndWorkingBuffers& data = *m_workingBuffers;
	data.m_density.SetCount(m_posit.GetCount());
	data.m_invDensity.SetCount(m_posit.GetCount());

	auto CalculateDensity = ndMakeObject::ndFunction([this, &data](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(CalculateDensity);
		const ndArray<ndVector>& posit = m_posit;

		const ndFloat32 h = data.m_particleDiameter;
		const ndFloat32 h2 = h * h;
		const ndFloat32 kernelConst = ndFloat32(315.0f) / (ndFloat32(64.0f) * ndPi * ndPow(h, ndFloat32 (9.0f)));
		const ndFloat32 kernelMassConst = m_mass * kernelConst;
		//const ndFloat32 selfDensity = kernelConst * h2 * h2 * h2;
		const ndFloat32 selfVolume = h2 * h2 * h2;

		const ndStartEnd startEnd(ndInt32(posit.GetCount()), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			const ndInt32 count = data.m_pairCount[i];
			const ndParticleKernelDistance& distance = data.m_kernelDistance[i];
			//ndFloat32 density = selfDensity;
			ndFloat32 volume = selfVolume;
			for (ndInt32 j = 0; j < count; ++j)
			{
				const ndFloat32 dist = distance.m_dist[j];
				const ndFloat32 dist2 = h2 - dist * dist;
				ndAssert(dist2 >= ndFloat32(0.0f));
				const ndFloat32 dist6 = dist2 * dist2 * dist2;
				//density += kernelConst * dist6;
				volume += dist6;
			}
			//density = kernelConst * density;
			ndFloat32 density = kernelMassConst * volume;
			data.m_density[i] = density;
			data.m_invDensity[i] = ndFloat32(1.0f) / density;
		}
	});

	threadPool->ParallelExecute(CalculateDensity);
}

void ndBodySphFluid::CalculateAccelerations(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	ndWorkingBuffers& data = *m_workingBuffers;
	data.m_accel.SetCount(m_posit.GetCount());

	auto CalculateAcceleration = ndMakeObject::ndFunction([this, &data](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(CalculateAcceleration);
		const ndVector epsilon2(ndFloat32(1.0e-12f));

		const ndArray<ndVector>& veloc = m_veloc;
		const ndArray<ndVector>& posit = m_posit;
		const ndFloat32* const density = &data.m_density[0];
		const ndFloat32* const invDensity = &data.m_invDensity[0];

		//const ndFloat32 u = m_viscosity;
		const ndFloat32 h = data.m_particleDiameter;
		const ndVector kernelConst(ndFloat32(45.0f) / (ndPi * ndPow(h, ndFloat32 (6.0f))));

		//const ndFloat32 viscosity = m_viscosity;
		const ndFloat32 restDensity = m_restDensity;
		const ndFloat32 gasConstant = m_gasConstant;

		//const ndVector gravity(m_gravity);
		const ndVector gravity(ndVector::m_zero);
		const ndStartEnd startEnd(ndInt32(posit.GetCount()), threadIndex, threadCount);
		for (ndInt32 i0 = startEnd.m_start; i0 < startEnd.m_end; ++i0)
		{
			const ndVector p0(posit[i0]);
			const ndVector v0(veloc[i0]);
			ndVector forceAcc(ndVector::m_zero);
			
			const ndParticlePair& pairs = data.m_pairs[i0];
			ndParticleKernelDistance& distance = data.m_kernelDistance[i0];
			const ndFloat32 pressureI0 = gasConstant * (density[i0] - restDensity);

			const ndInt32 count = data.m_pairCount[i0];
			for (ndInt32 j = 0; j < count; ++j)
			{
				const ndInt32 i1 = pairs.m_neighborg[j];
				const ndVector p10(p0 - posit[i1]);
				//const ndVector p10(posit[i1] - p0);
				const ndVector dot(p10.DotProduct(p10) + epsilon2);
				const ndVector unitDir(p10 * dot.InvSqrt());
			
				ndAssert(p10.m_w == ndFloat32(0.0f));
				ndAssert(ndAbs(ndSqrt (dot.GetScalar()) - distance.m_dist[j]) < ndFloat32(1.0e-4f));
			
				// kernel distance
				const ndFloat32 dist = h - distance.m_dist[j];
				ndAssert(dist >= ndFloat32(0.0f));
				const ndFloat32 kernelValue = dist * dist;
			
				// calculate pressure
				const ndFloat32 pressureI1 = gasConstant * (density[i1] - restDensity);
				const ndFloat32 averagePressure = ndFloat32 (0.5f) * invDensity[i1] * (pressureI1 + pressureI0);
				const ndVector forcePresure(m_mass * averagePressure * kernelValue);

				//// calculate viscosity acceleration
				//const ndVector v01(veloc[i1] - v0);
				//forceAcc += v01 * ndVector(kernelDist * viscosity * invDensity[j]);

				const ndVector force(forcePresure * unitDir);
				forceAcc += force;
			}

			const ndVector accel(gravity + forceAcc);
			//const ndVector accel(gravity + ndVector(invDensity[i0]) * forceAcc);
			data.m_accel[i0] = accel;
		}
	});

	threadPool->ParallelExecute(CalculateAcceleration);
}

void ndBodySphFluid::IntegrateParticles(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	ndWorkingBuffers& data = *m_workingBuffers;
	auto IntegrateParticles = ndMakeObject::ndFunction([this, &data](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(IntegrateParticles);
		const ndArray<ndVector>& accel = data.m_accel;
		ndArray<ndVector>& veloc = m_veloc;
		ndArray<ndVector>& posit = m_posit;

		//const ndVector timestep (ndFloat32 (0.003f));
		//const ndVector timestep(m_timestep);
		//const ndVector timestep(m_timestep * 0.5f);
		const ndVector timestep(m_timestep * 0.25f);

		const ndStartEnd startEnd(ndInt32(posit.GetCount()), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			veloc[i] = veloc[i] + accel[i] * timestep;
			posit[i] = posit[i] + veloc[i] * timestep;
			if (posit[i].m_y <= 1.0f)
			{
				posit[i].m_y = 1.0f;
				veloc[i].m_y = 0.0f;
			}
		}
	});

	threadPool->ParallelExecute(IntegrateParticles);
}

void ndBodySphFluid::CaculateAabb(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	class ndBox
	{
		public:
		ndBox()
			:m_min(ndFloat32(1.0e10f))
			, m_max(ndFloat32(-1.0e10f))
		{
		}
		ndVector m_min;
		ndVector m_max;
	};

	ndBox boxes[D_MAX_THREADS_COUNT];
	auto CalculateAabb = ndMakeObject::ndFunction([this, &boxes](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(CalculateAabb);
		ndBox box;
		const ndArray<ndVector>& posit = m_posit;
		const ndStartEnd startEnd(ndInt32(posit.GetCount()), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			box.m_min = box.m_min.GetMin(posit[i]);
			box.m_max = box.m_max.GetMax(posit[i]);
		}
		boxes[threadIndex] = box;
	});

	threadPool->ParallelExecute(CalculateAabb);

	ndBox box;
	const ndInt32 threadCount = threadPool->GetThreadCount();
	for (ndInt32 i = 0; i < threadCount; ++i)
	{
		box.m_min = box.m_min.GetMin(boxes[i].m_min);
		box.m_max = box.m_max.GetMax(boxes[i].m_max);
	}

	ndWorkingBuffers& data = *m_workingBuffers;
	const ndFloat32 diameter = ndFloat32(2.0f) * GetParticleRadius();
	const ndFloat32 gridSize = ndFloat32(4.0f) * diameter * D_GRID_SIZE_SCALER;

	data.m_hashGridSize = gridSize;
	data.m_particleDiameter = diameter;
	data.m_hashInvGridSize = ndFloat32(1.0f) / gridSize;

	const ndVector grid(data.m_hashGridSize);
	const ndVector invGrid(data.m_hashInvGridSize);

	// add one grid padding to the aabb
	box.m_min -= grid;
	box.m_max += (grid + grid);

	// quantize the aabb to integers of the gird size
	box.m_min = grid * (box.m_min * invGrid).Floor();
	box.m_max = grid * (box.m_max * invGrid).Floor();

	// make sure the w component is zero.
	m_box0 = box.m_min & ndVector::m_triplexMask;
	m_box1 = box.m_max & ndVector::m_triplexMask;
	data.SetWorldToGridMapping(gridSize, m_box1, m_box0);
}

void ndBodySphFluid::CreateGrids(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	class ndGridNeighborInfo
	{
		public:
		ndGridNeighborInfo()
		{
			//ndGridHash stepsCode;
			m_neighborDirs[0][0] = ndGridHash(0, 0);
			m_neighborDirs[0][1] = ndGridHash(0, 0);
			m_neighborDirs[0][2] = ndGridHash(0, 0);
			m_neighborDirs[0][3] = ndGridHash(0, 0);

			m_counter[0] = 1;
			m_isPadd[0][0] = 0;
			m_isPadd[0][1] = 1;
			m_isPadd[0][2] = 1;
			m_isPadd[0][3] = 1;

			//ndGridHash stepsCode_y;
			m_neighborDirs[1][0] = ndGridHash(0, 0);
			m_neighborDirs[1][1] = ndGridHash(1, 0);
			m_neighborDirs[1][2] = ndGridHash(0, 0);
			m_neighborDirs[1][3] = ndGridHash(0, 0);

			m_counter[1] = 2;
			m_isPadd[1][0] = 0;
			m_isPadd[1][1] = 0;
			m_isPadd[1][2] = 1;
			m_isPadd[1][3] = 1;

			//ndGridHash stepsCode_z;
			m_neighborDirs[2][0] = ndGridHash(0, 0);
			m_neighborDirs[2][1] = ndGridHash(0, 1);
			m_neighborDirs[2][2] = ndGridHash(0, 0);
			m_neighborDirs[2][3] = ndGridHash(0, 0);

			m_counter[2] = 2;
			m_isPadd[2][0] = 0;
			m_isPadd[2][1] = 0;
			m_isPadd[2][2] = 1;
			m_isPadd[2][3] = 1;

			//ndGridHash stepsCode_yz;
			m_neighborDirs[3][0] = ndGridHash(0, 0);
			m_neighborDirs[3][1] = ndGridHash(1, 0);
			m_neighborDirs[3][2] = ndGridHash(0, 1);
			m_neighborDirs[3][3] = ndGridHash(1, 1);

			m_counter[3] = 4;
			m_isPadd[3][0] = 0;
			m_isPadd[3][1] = 0;
			m_isPadd[3][2] = 0;
			m_isPadd[3][3] = 0;
		}

		ndGridHash m_neighborDirs[4][4];
		ndInt8 m_isPadd[4][4];
		ndInt8 m_counter[4];
	};

	ndGridNeighborInfo neiborghood;
	ndWorkingBuffers& data = *m_workingBuffers;

	auto CountGrids = ndMakeObject::ndFunction([this, &data, &neiborghood](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(CountGrids);
		const ndVector origin(m_box0);
		const ndVector invGridSize(data.m_hashInvGridSize);
		const ndVector particleBox(data.m_particleDiameter);

		const ndVector* const posit = &m_posit[0];
		ndInt32* const scans = &data.m_gridScans[0];

		const ndStartEnd startEnd(ndInt32(m_posit.GetCount()), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			const ndVector gridPosit(posit[i] - origin);
			const ndVector p0(gridPosit - particleBox);
			const ndVector p1(gridPosit + particleBox);
			const ndGridHash box0Hash(p0 * invGridSize, i);
			const ndGridHash box1Hash(p1 * invGridSize, i);
			const ndGridHash codeHash(box1Hash.m_gridHash - box0Hash.m_gridHash);

#ifdef D_USE_YZ_PLANE_BUCKETS
			ndAssert(codeHash.m_y <= 1);
			ndAssert(codeHash.m_z <= 1);
			const ndUnsigned32 code = ndUnsigned32(codeHash.m_z * 2 + codeHash.m_y);
#else
			ndAssert(codeHash.m_x <= 1);
			ndAssert(codeHash.m_z <= 1);
			const ndUnsigned32 code = ndUnsigned32(codeHash.m_z * 2 + codeHash.m_x);
#endif
			scans[i] = neiborghood.m_counter[code];
		}
	});

	auto CreateGrids = ndMakeObject::ndFunction([this, &data, &neiborghood](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(CreateGrids);
		const ndVector origin(m_box0);
		ndGridHash* const dst = &data.m_hashGridMap[0];
		const ndInt32* const scans = &data.m_gridScans[0];
		const ndVector* const posit = &m_posit[0];
		const ndVector invGridSize(data.m_hashInvGridSize);
		const ndVector particleBox(data.m_particleDiameter);

		const ndStartEnd startEnd(ndInt32(m_posit.GetCount()), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			const ndVector gridPosit(posit[i] - origin);
			const ndVector gridPosit0(gridPosit - particleBox);
			const ndVector gridPosit1(gridPosit + particleBox);
			const ndGridHash centerHash(gridPosit * invGridSize, i);
			const ndGridHash box0Hash(gridPosit0 * invGridSize, i);
			const ndGridHash box1Hash(gridPosit1 * invGridSize, i);
			const ndGridHash codeHash(box1Hash.m_gridHash - box0Hash.m_gridHash);

			const ndInt32 base = scans[i];
			const ndInt32 count = scans[i + 1] - base;

#ifdef D_USE_YZ_PLANE_BUCKETS
			ndAssert(codeHash.m_y <= 1);
			ndAssert(codeHash.m_z <= 1);
			const ndInt32 code = ndInt32(codeHash.m_z * 2 + codeHash.m_y);
#else
			ndAssert(codeHash.m_x <= 1);
			ndAssert(codeHash.m_z <= 1);
			const ndInt32 code = ndInt32(codeHash.m_z * 2 + codeHash.m_x);
#endif
			const ndGridHash* const neigborgh = &neiborghood.m_neighborDirs[code][0];
			ndAssert(count == neiborghood.m_counter[code]);
			for (ndInt32 j = 0; j < count; ++j)
			{
				ndGridHash quadrand(box0Hash);
				quadrand.m_gridHash += neigborgh[j].m_gridHash;
				quadrand.m_cellType = ndGridHash::ndGridType(quadrand.m_gridHash == centerHash.m_gridHash);
				ndAssert(quadrand.m_cellType == ((quadrand.m_gridHash == centerHash.m_gridHash) ? ndGridHash::m_homeGrid : ndGridHash::m_adjacentGrid));
				dst[base + j] = quadrand;
			}
		}
	});

	data.m_gridScans.SetCount(m_posit.GetCount() + 1);
	data.m_gridScans[m_posit.GetCount()] = 0;
	threadPool->ParallelExecute(CountGrids);

	ndInt32 gridCount = 0;
	const ndInt32 itemsCount = ndInt32 (data.m_gridScans.GetCount()) & (-8);
	for (ndInt32 i = 0; i < itemsCount; i += 8)
	{
		for (ndInt32 j = 0; j < 8; ++j)
		{
			ndInt32 count = data.m_gridScans[i + j];
			data.m_gridScans[i + j] = gridCount;
			gridCount += count;
		}
	}
	for (ndInt32 j = itemsCount; j < data.m_gridScans.GetCount(); ++j)
	{
		ndInt32 count = data.m_gridScans[j];
		data.m_gridScans[j] = gridCount;
		gridCount += count;
	}

	data.m_hashGridMap.SetCount(gridCount);
	threadPool->ParallelExecute(CreateGrids);
	data.m_hashGridMapScratchBuffer.SetCount(gridCount);

	//ndAssert(TraceHashes());
}

bool ndBodySphFluid::TraceHashes() const
{
#if 0
	ndWorkingBuffers& data = *m_workingBuffers;
	ndGridHash* xxxx = &data.m_hashGridMap[0];
	for (ndInt32 i = 0; i < data.m_hashGridMap.GetCount(); i++)
	{
		ndTrace(("id(%d)\tx(%d)\tz(%d)\n", xxxx[i].m_particleIndex, xxxx[i].m_x, xxxx[i].m_z));
	}
#endif

	return true;
}

void ndBodySphFluid::Update(const ndScene* const scene, ndFloat32 timestep)
{
	if (TaskState() == ndBackgroundTask::m_taskCompleted)
	{
		if (m_posit.GetCount())
		{
			m_timestep = timestep;
			((ndScene*)scene)->SendBackgroundTask(this);
			if (!m_updateInBackground)
			{
				Sync();
			}
		}
	}
}

void ndBodySphFluid::Execute(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	ndAssert(sizeof(ndGridHash) == sizeof(ndUnsigned64));

	CaculateAabb(threadPool);
	CreateGrids(threadPool);
	SortGrids(threadPool);
	CalculateScans(threadPool);
	BuildBuckets(threadPool);
	CalculateParticlesDensity(threadPool);
	CalculateAccelerations(threadPool);
	IntegrateParticles(threadPool);
}

#endif

#endif
