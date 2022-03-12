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
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndBodySphFluid.h"

#define D_USE_SPH_SMALL_HASH

#ifdef D_USE_SPH_SMALL_HASH
	#define D_SPH_HASH_BITS	7
#else
	#define D_SPH_HASH_BITS	10
#endif

class ndBodySphFluid::ndGridHash
{
	public:
	ndGridHash()
	{
	}

	ndGridHash(ndUnsigned64 gridHash)
		:m_gridHash(gridHash)
	{
	}

	ndGridHash(ndInt32 y, ndInt32 z)
	{
		m_gridHash = 0;
		m_y = y;
		m_z = z;
	}

	ndGridHash(const ndVector& grid, ndInt32 particleIndex)
	{
		dAssert(grid.m_y >= ndFloat32(0.0f));
		dAssert(grid.m_z >= ndFloat32(0.0f));
		dAssert(grid.m_y < ndFloat32(1 << (D_SPH_HASH_BITS * 2)));
		dAssert(grid.m_z < ndFloat32(1 << (D_SPH_HASH_BITS * 2)));

		ndVector hash(grid.GetInt());

		m_gridHash = 0;
		m_y = hash.m_iy;
		m_z = hash.m_iz;

		m_cellIsPadd = 1;
		m_cellType = ndAdjacentGrid;
		m_particleIndex = particleIndex;
	}

#ifdef D_USE_SPH_SMALL_HASH
	union
	{
		struct
		{
			ndUnsigned64 m_y : D_SPH_HASH_BITS * 2;
			ndUnsigned64 m_z : D_SPH_HASH_BITS * 2;
			ndUnsigned64 m_particleIndex : 20;
			ndUnsigned64 m_cellType : 1;
			ndUnsigned64 m_cellIsPadd : 1;
		};
		struct
		{
			ndUnsigned64 m_yLow : D_SPH_HASH_BITS;
			ndUnsigned64 m_yHigh : D_SPH_HASH_BITS;
			ndUnsigned64 m_zLow : D_SPH_HASH_BITS;
			ndUnsigned64 m_zHigh : D_SPH_HASH_BITS;
		};
		ndUnsigned64 m_gridHash : D_SPH_HASH_BITS * 2 * 2;
	};
#else
	union
	{
		struct
		{
			ndUnsigned64 m_y : D_SPH_HASH_BITS * 2;
			ndUnsigned64 m_z : D_SPH_HASH_BITS * 2;
		};
		struct
		{
			ndUnsigned64 m_yLow : D_SPH_HASH_BITS;
			ndUnsigned64 m_yHigh : D_SPH_HASH_BITS;
			ndUnsigned64 m_zLow : D_SPH_HASH_BITS;
			ndUnsigned64 m_zHigh : D_SPH_HASH_BITS;
		};
		ndUnsigned64 m_gridHash;
	};
	ndInt32 m_particleIndex;
	ndInt8 m_cellType;
	ndInt8 m_cellIsPadd;
#endif
};

class ndBodySphFluid::ndParticlePair
{
	public:
	ndInt32 m_neighborg[32];
};

class ndBodySphFluid::ndParticleKernelDistance
{
	public:
	ndFloat32 m_dist[32];
};

class ndBodySphFluid::ndWorkingData
{
	#define D_SPH_GRID_X_RESOLUTION 4

	public:
	ndWorkingData()
		:m_accel(256)
		,m_locks(256)
		,m_pairCount(256)
		,m_gridScans(256)
		,m_density(256)
		,m_invDensity(256)
		,m_pairs(256)
		,m_hashGridMap(256)
		,m_hashGridMapScratchBuffer(256)
		,m_kernelDistance(256)
		,m_worlToGridOrigin(ndFloat32 (1.0f))
		,m_worlToGridScale(ndFloat32(1.0f))
	{
		for (ndInt32 i = 0; i < D_MAX_THREADS_COUNT; ++i)
		{
			m_partialsGridScans[i].Resize(256);
		}
	}

	void Clear()
	{
		m_accel.Resize(256);
		m_locks.Resize(256);
		m_pairs.Resize(256);
		m_density.Resize(256);
		m_pairCount.Resize(256);
		m_gridScans.Resize(256);
		m_invDensity.Resize(256);
		m_hashGridMap.Resize(256);
		m_kernelDistance.Resize(256);
		m_hashGridMapScratchBuffer.Resize(256);
		for (ndInt32 i = 0; i < D_MAX_THREADS_COUNT; ++i)
		{
			m_partialsGridScans[i].Resize(256);
		}
	}

	void SetWorldToGridMapping(ndInt32 gridCount, ndFloat32 xMax, ndFloat32 xMin)
	{
		m_worlToGridOrigin = xMin;
		m_worlToGridScale = ndFloat32(1<< D_SPH_GRID_X_RESOLUTION) * gridCount / (xMax - xMin);
	}

	ndInt32 WorldToGrid(ndFloat32 x) const
	{
		return ndInt32 ((x - m_worlToGridOrigin) * m_worlToGridScale);
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
};

ndBodySphFluid::ndBodySphFluid()
	:ndBodyParticleSet()
	,m_mass(ndFloat32(0.02f))
	,m_viscosity(ndFloat32 (1.05f))
	,m_restDensity(ndFloat32(1000.0f))
	,m_gasConstant(ndFloat32(1.0f))
{
}

ndBodySphFluid::ndBodySphFluid(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndBodyParticleSet(desc)
	,m_mass(ndFloat32(0.02f))
	,m_viscosity(ndFloat32(1.0f))
	,m_restDensity(ndFloat32(1000.0f))
	,m_gasConstant(ndFloat32(1.0f))
{
	// nothing was saved
	dAssert(0);
}

ndBodySphFluid::~ndBodySphFluid()
{
	ndWorkingData& data = WorkingData();
	data.Clear();
}

//void ndBodySphFluid::Save(const dLoadSaveBase::dSaveDescriptor& desc) const
void ndBodySphFluid::Save(const ndLoadSaveBase::ndSaveDescriptor&) const
{
	dAssert(0);
	//nd::TiXmlElement* const paramNode = CreateRootElement(rootNode, "ndBodySphFluid", nodeid);
	//ndBodyParticleSet::Save(paramNode, assetPath, nodeid, shapesCache);
}

ndBodySphFluid::ndWorkingData& ndBodySphFluid::WorkingData()
{
	static ndWorkingData workingBuffers;
	return workingBuffers;
}

void ndBodySphFluid::SortXdimension(ndThreadPool* const threadPool)
{
	D_TRACKTIME();

	#define XRESOLUTION	ndFloat32(1<<23)
	class ndKey_low
	{
		public:
		ndKey_low(void* const context)
			:m_fluid((ndBodySphFluid*)context)
			,m_data(m_fluid->WorkingData())
			,m_point(m_fluid->GetPositions())
		{
		}

		ndInt32 GetKey(const ndGridHash& cell) const
		{
			ndUnsigned32 key = m_data.WorldToGrid(m_point[cell.m_particleIndex].m_x);
			return key & 0xff;
		}

		ndBodySphFluid* m_fluid;
		ndWorkingData& m_data;
		const ndArray<ndVector>& m_point;
	};

	class ndKey_middle
	{
		public:
		ndKey_middle(void* const context)
			:m_fluid((ndBodySphFluid*)context)
			,m_data(m_fluid->WorkingData())
			,m_point(m_fluid->GetPositions())
		{
		}

		ndInt32 GetKey(const ndGridHash& cell) const
		{
			ndUnsigned32 key = m_data.WorldToGrid(m_point[cell.m_particleIndex].m_x);
			return (key >> 8) & 0xff;
		}

		ndBodySphFluid* m_fluid;
		ndWorkingData& m_data;
		const ndArray<ndVector>& m_point;
	};

	class ndKey_high
	{
		public:
		ndKey_high(void* const context)
			:m_fluid((ndBodySphFluid*)context)
			,m_data(m_fluid->WorkingData())
			,m_point(m_fluid->GetPositions())
		{
		}

		ndInt32 GetKey(const ndGridHash& cell) const
		{
			ndUnsigned32 key = m_data.WorldToGrid(m_point[cell.m_particleIndex].m_x);
			return (key >> 16) & 0xff;
		}

		ndBodySphFluid* m_fluid;
		ndWorkingData& m_data;
		const ndArray<ndVector>& m_point;
	};

	ndWorkingData& data = WorkingData();

	ndCountingSort<ndGridHash, ndKey_low, 8>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, this);
	const ndInt32 keySize = data.WorldToGrid(m_box1.m_x);
	if (keySize >= 256)
	{
		ndCountingSort<ndGridHash, ndKey_middle, 8>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, this);
		if (keySize >= (256 * 256))
		{
			ndCountingSort<ndGridHash, ndKey_high, 8>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, this);
		}
	}
}

void ndBodySphFluid::SortCellBuckects(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	class ndKey_ylow
	{
		public:
		ndKey_ylow(void* const) {}
		ndInt32 GetKey(const ndGridHash& cell) const
		{
			return cell.m_yLow;
		}
	};

	class ndKey_zlow
	{
		public:
		ndKey_zlow(void* const) {}
		ndInt32 GetKey(const ndGridHash& cell) const
		{
			return cell.m_zLow;
		}
	};

	class ndKey_yhigh
	{
		public:
		ndKey_yhigh(void* const) {}
		ndInt32 GetKey(const ndGridHash& cell) const
		{
			return cell.m_yHigh;
		}
	};

	class ndKey_zhigh
	{
		public:
		ndKey_zhigh(void* const) {}
		ndInt32 GetKey(const ndGridHash& cell) const
		{
			return cell.m_zHigh;
		}
	};

	ndWorkingData& data = WorkingData();

	ndVector boxSize((m_box1 - m_box0).Scale(ndFloat32(1.0f) / GetSphGridSize()).GetInt());
	ndCountingSort<ndGridHash, ndKey_ylow, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer);
	if (boxSize.m_iy > (1 << D_SPH_HASH_BITS))
	{
		ndCountingSort<ndGridHash, ndKey_yhigh, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer);
	}
	
	ndCountingSort<ndGridHash, ndKey_zlow, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer);
	if (boxSize.m_iz > (1 << D_SPH_HASH_BITS))
	{
		ndCountingSort<ndGridHash, ndKey_zhigh, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer);
	}
}

void ndBodySphFluid::CalculateScans(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	ndWorkingData& data = WorkingData();
	ndInt32 sums[D_MAX_THREADS_COUNT + 1];
	ndInt32 scans[D_MAX_THREADS_COUNT + 1];

	auto CountGridScans = ndMakeObject::ndFunction([this, &data, &scans](ndInt32 threadIndex, ndInt32)
	{
		D_TRACKTIME();
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

	auto CalculateScans = ndMakeObject::ndFunction([this, &data, &scans, &sums](ndInt32 threadIndex, ndInt32)
	{
		D_TRACKTIME();
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
	SortXdimension(threadPool);
	SortCellBuckects(threadPool);

	#ifdef _DEBUG
	ndWorkingData& data = WorkingData();
	for (ndInt32 i = 0; i < (data.m_hashGridMap.GetCount() - 1); ++i)
	{
		const ndGridHash& entry0 = data.m_hashGridMap[i + 0];
		const ndGridHash& entry1 = data.m_hashGridMap[i + 1];
		ndUnsigned64 gridHashA = entry0.m_gridHash;
		ndUnsigned64 gridHashB = entry1.m_gridHash;
		dAssert(gridHashA <= gridHashB);
	}
	#endif
}

void ndBodySphFluid::BuildPairs(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	ndWorkingData& data = WorkingData();
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
		D_TRACKTIME();
		const ndArray<ndGridHash>& hashGridMap = data.m_hashGridMap;
		const ndArray<ndInt32>& gridScans = data.m_gridScans;
		const ndFloat32 diameter = GetSphGridSize();
		const ndFloat32 diameter2 = diameter * diameter;
		const ndInt32 windowsTest = data.WorldToGrid(data.m_worlToGridOrigin + diameter) + 1;

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
				const ndInt32 homeGridTest0 = (hash0.m_cellType == ndHomeGrid);
				const ndInt32 particle0 = hash0.m_particleIndex;
				const ndInt32 x0 = data.WorldToGrid(m_posit[particle0].m_x);
				for (ndInt32 j = i + 1; j < count; ++j)
				{
					const ndGridHash hash1 = hashGridMap[start + j];
					const ndInt32 particle1 = hash1.m_particleIndex;
					dAssert(particle0 != particle1);
					const ndInt32 x1 = data.WorldToGrid(m_posit[particle1].m_x);
					dAssert((x1 - x0) > ndFloat32(-1.0e-3f));
					const ndInt32 sweeptTest = ((x1 - x0) >= windowsTest);
					if (sweeptTest)
					{
						break;
					}
					dAssert(particle0 != particle1);
					const ndInt32 homeGridTest1 = (hash1.m_cellType == ndHomeGrid);
					const ndInt32 test = homeGridTest0 | homeGridTest1;
					if (test)
					{
						dAssert(particle0 != particle1);

						const ndVector p1p0(m_posit[particle0] - m_posit[particle1]);
						const ndFloat32 dist2(p1p0.DotProduct(p1p0).GetScalar());
						if (dist2 < diameter2)
						{
							dAssert(dist2 >= ndFloat32(0.0f));
							const ndFloat32 dist = ndSqrt(dist2);
							{
								ndSpinLock lock(locks[particle0]);
								ndInt8 neigborCount = pairCount[particle0];
								if (neigborCount < 32)
								{
									ndInt8 isUnique = 1;
									ndInt32* const neighborg = pair[particle0].m_neighborg;
									for (ndInt32 k = neigborCount - 1; k >= 0; --k)
									{
										isUnique = isUnique & (neighborg[k] != particle1);
									}

									neighborg[neigborCount] = particle1;
									distance[particle0].m_dist[neigborCount] = dist;
									pairCount[particle0] = neigborCount + isUnique;
								}
							}

							{
								ndSpinLock lock(locks[particle1]);
								ndInt8 neigborCount = pairCount[particle1];
								if (neigborCount < 32)
								{
									ndInt8 isUnique = 1;
									ndInt32* const neighborg = pair[particle1].m_neighborg;
									for (ndInt32 k = neigborCount - 1; k >= 0; --k)
									{
										isUnique = isUnique & (neighborg[k] != particle0);
									}
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

		// even step is nor good because the bashes tend to be clustered
		// a better way is to sort, bu that takes about 1 ms,
		// we interleaving bashes and has the randomizing 
		// effect that balance the work load on the thread.

		const ndInt32 scansCount = gridScans.GetCount() - 1;
		for (ndInt32 i = threadIndex; i < scansCount; i += threadCount)
		{
			const ndInt32 start = gridScans[i];
			const ndInt32 count = gridScans[i + 1] - start;
			ProccessCell(start, count);
		}
	});
	
	threadPool->ParallelExecute(AddPairs);
}

void ndBodySphFluid::CalculateParticlesDensity(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	ndWorkingData& data = WorkingData();
	data.m_density.SetCount(m_posit.GetCount());
	data.m_invDensity.SetCount(m_posit.GetCount());

	auto CalculateDensity = ndMakeObject::ndFunction([this, &data](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		const ndArray<ndVector>& posit = m_posit;

		const ndFloat32 h = GetSphGridSize();
		const ndFloat32 h2 = h * h;
		const ndFloat32 kernelMagicConst = ndFloat32(315.0f) / (ndFloat32(64.0f) * ndPi * ndPow(h, 9));
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
				dAssert(dist2 > ndFloat32(0.0f));
				const ndFloat32 dist6 = dist2 * dist2 * dist2;
				density += kernelConst * dist6;
			}
			dAssert(density > ndFloat32(0.0f));
			data.m_density[i] = density;
			data.m_invDensity[i] = ndFloat32(1.0f) / density;
		}
	});

	threadPool->ParallelExecute(CalculateDensity);
}

void ndBodySphFluid::CalculateAccelerations(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	ndWorkingData& data = WorkingData();
	data.m_accel.SetCount(m_posit.GetCount());

	auto CalculateAcceleration = ndMakeObject::ndFunction([this, &data](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		const ndVector epsilon2 (ndFloat32(1.0e-12f));

		const ndArray<ndVector>& veloc = m_veloc;
		const ndArray<ndVector>& posit = m_posit;
		const ndFloat32* const density = &data.m_density[0];
		const ndFloat32* const invDensity = &data.m_invDensity[0];

		const ndFloat32 h = GetSphGridSize();
		const ndFloat32 u = m_viscosity;
		const ndVector kernelConst(m_mass * ndFloat32(45.0f) / (ndPi * ndPow(h, 6)));

		const ndFloat32 viscosity = m_viscosity;
		const ndFloat32 restDensity = m_restDensity;
		const ndFloat32 gasConstant = ndFloat32(0.5f) * m_gasConstant;

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

				dAssert(unitDir.m_w == ndFloat32(0.0f));

				// kernel distance
				const ndFloat32 dist = distance.m_dist[j];
				const ndFloat32 kernelDist = h - dist;
				dAssert(kernelDist >= ndFloat32(0.0f));

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
	ndWorkingData& data = WorkingData();
	auto IntegrateParticles = ndMakeObject::ndFunction([this, &data](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		const ndArray<ndVector>& accel = data.m_accel;
		ndArray<ndVector>& veloc = m_veloc;
		ndArray<ndVector>& posit = m_posit;

		//const ndVector timestep(m_timestep);
		//ndVector halfTime(timestep * ndVector::m_half);
		const ndVector timestep (ndFloat32 (0.003f));

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
			, m_max(ndFloat32(-1.0e10f))
		{
		}
		ndVector m_min;
		ndVector m_max;
	};

	ndBox boxes[D_MAX_THREADS_COUNT];
	auto CalculateAabb = ndMakeObject::ndFunction([this, &boxes](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
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

	const ndFloat32 gridSize = GetSphGridSize();

	ndVector grid(gridSize);
	ndVector invGrid(ndFloat32(1.0f) / gridSize);

	// add one grid padding to the aabb
	box.m_min -= grid;
	box.m_max += (grid + grid);

	// quantize the aabb to integers of the gird size
	box.m_min = grid * (box.m_min * invGrid).Floor();
	box.m_max = grid * (box.m_max * invGrid).Floor();

	// make sure the w component is zero.
	m_box0 = box.m_min & ndVector::m_triplexMask;
	m_box1 = box.m_max & ndVector::m_triplexMask;

	ndWorkingData& data = WorkingData();
	ndInt32 numberOfGrid = ndInt32((box.m_max.m_x - box.m_min.m_x) * invGrid.m_x + ndFloat32(1.0f));
	data.SetWorldToGridMapping(numberOfGrid, m_box1.m_x, m_box0.m_x);
}

void ndBodySphFluid::CreateGrids(ndThreadPool* const threadPool)
{
	//#define D_USE_PARALLEL_CLASSIFY
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

			ndGridHash stepsCode_y;
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
	ndWorkingData& data = WorkingData();
	ndInt32 scans[D_MAX_THREADS_COUNT][2];

	auto CreateGrids = ndMakeObject::ndFunction([this, &data, &neiborghood, &scans](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		const ndVector origin(m_box0);
		const ndFloat32 gridSize = GetSphGridSize();
		ndGridHash* const dst = &data.m_hashGridMapScratchBuffer[0];
		
		// the 0.99 factor is to make sure the box 
		// fits in not more than two adjacent grids.
		const ndVector box(gridSize * ndFloat32(0.5f * 0.99f));
		const ndVector invGridSize(ndFloat32(1.0f) / gridSize);
		const ndVector* const posit = &m_posit[0];

		#ifdef D_USE_PARALLEL_CLASSIFY
		ndInt32* const scan = &scans[threadIndex][0];
		scan[0] = 0;
		scan[1] = 0;
		#endif

		const ndStartEnd startEnd(m_posit.GetCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			const ndVector r(posit[i] - origin);
			const ndVector p(r * invGridSize);
			const ndGridHash hashKey(p, i);

			const ndVector p0((r - box) * invGridSize);
			const ndVector p1((r + box) * invGridSize);
			ndGridHash box0Hash(p0, i);
			const ndGridHash box1Hash(p1, i);
			const ndGridHash codeHash(box1Hash.m_gridHash - box0Hash.m_gridHash);

			dAssert(codeHash.m_y <= 1);
			dAssert(codeHash.m_z <= 1);
			const ndUnsigned32 code = ndUnsigned32(codeHash.m_z * 2 + codeHash.m_y);

			#ifdef D_USE_PARALLEL_CLASSIFY
			scan[0] += neiborghood.m_counter[code];
			scan[1] += (4 - neiborghood.m_counter[code]);
			#endif

			const ndInt8* const padding = &neiborghood.m_isPadd[code][0];
			const ndGridHash* const neigborgh = &neiborghood.m_neighborDirs[code][0];
			for (ndInt32 j = 0; j < 4; j++)
			{
				ndGridHash quadrand(box0Hash);
				quadrand.m_cellIsPadd = padding[j];
				quadrand.m_gridHash += neigborgh[j].m_gridHash;
				quadrand.m_cellType = ndGridType(quadrand.m_gridHash == hashKey.m_gridHash);
				dAssert(quadrand.m_cellIsPadd <= 1);
				dAssert(quadrand.m_cellType == ((quadrand.m_gridHash == hashKey.m_gridHash) ? ndHomeGrid : ndAdjacentGrid));
				dst[i * 4 + j] = quadrand;
			}
		}
	});

	auto CompactGrids = ndMakeObject::ndFunction([this, &data, &scans](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		ndArray<ndGridHash>& dst = data.m_hashGridMap;
		const ndGridHash* const src = &data.m_hashGridMapScratchBuffer[0];
		ndInt32* const scan = &scans[threadIndex][0];

		const ndStartEnd startEnd(dst.GetCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			const ndGridHash grid(src[i]);
			const ndInt32 key = grid.m_cellIsPadd;
			const ndInt32 index = scan[key];
			dst[index] = grid;
			scan[key] = index + 1;
		}
	});

	dAssert(sizeof(ndGridHash) <= 16);
	
	data.m_hashGridMap.SetCount(m_posit.GetCount() * 4);
	data.m_hashGridMapScratchBuffer.SetCount(m_posit.GetCount() * 4);
	threadPool->ParallelExecute(CreateGrids);

	#ifdef D_USE_PARALLEL_CLASSIFY
		ndInt32 sum = 0;
		const ndInt32 threadCount = threadPool->GetThreadCount();
		for (ndInt32 i = 0; i < 2; ++i)
		{
			for (ndInt32 j = 0; j < threadCount; ++j)
			{
				ndInt32 partialSum = scans[j][i];
				scans[j][i] = sum;
				sum += partialSum;
			}
		}

		// there is a bug here. need to debug it
		ndInt32 gridCount = scans[0][1] - scans[0][0];
		threadPool->ParallelExecute(CompactGrids);

	#else

		ndInt32 gridCount = 0;
		{
			D_TRACKTIME();
			// this seems to be very cache friendly and beating radix sort hand down.
			for (ndInt32 i = 0; i < data.m_hashGridMapScratchBuffer.GetCount(); ++i)
			{
				const ndGridHash cell(data.m_hashGridMapScratchBuffer[i]);
				data.m_hashGridMap[gridCount] = cell;
				gridCount += (1 - ndInt32(cell.m_cellIsPadd));
			}
		}
	#endif
	data.m_hashGridMap.SetCount(gridCount);
}

void ndBodySphFluid::Execute(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	CaculateAabb(threadPool);
	CreateGrids(threadPool);
	SortGrids(threadPool);
	CalculateScans(threadPool);
	BuildPairs(threadPool);
	CalculateParticlesDensity(threadPool);
	CalculateAccelerations(threadPool);
	IntegrateParticles(threadPool);
}

void ndBodySphFluid::Update(const ndWorld* const world, ndFloat32 timestep)
{
	if (taskState() == ndBackgroundTask::m_taskCompleted)
	{
		m_timestep = timestep;
		ndScene* const scene = world->GetScene();
		scene->SendBackgroundTask(this);
		if (!m_updateInBackground)
		{
			Sync();
		}
	}
}
