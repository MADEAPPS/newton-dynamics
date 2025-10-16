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
#include "ndBodySphFluid_New.h"

#ifdef D_USE_NEW_FLUID

#define D_SPH_HASH_BITS				7
#define D_SPH_BUFFER_GRANULARITY	4096	

#define D_PARTICLE_BUCKET_SIZE		32
//#define D_GRID_SIZE_SCALER		(2.0f)
#define D_GRID_SIZE_SCALER		(4.0f)

class ndBodySphFluid::ndGridHash
{
	public:
	enum ndGridType
	{
		m_adajacent = 0,
		m_homeGrid = 1,
	};

	ndGridHash()
	{
	}

	ndGridHash(ndUnsigned64 gridHash)
		:m_gridHash(gridHash)
	{
	}

	ndGridHash(ndInt32 x, ndInt32 y, ndInt32 z, ndInt32 particleIndex)
	{
		m_gridHash = 0;
		m_x = ndUnsigned64(x);
		m_y = ndUnsigned64(y);
		m_z = ndUnsigned64(z);
		m_particleIndex = ndUnsigned64(particleIndex);
	}

	ndGridHash(const ndVector& grid, ndInt32 particleIndex)
	{
		ndAssert(grid.m_x >= ndFloat32(0.0f));
		ndAssert(grid.m_y >= ndFloat32(0.0f));
		ndAssert(grid.m_z >= ndFloat32(0.0f));
		ndAssert(grid.m_x < ndFloat32(1 << (D_SPH_HASH_BITS * 2)));
		ndAssert(grid.m_y < ndFloat32(1 << (D_SPH_HASH_BITS * 2)));
		ndAssert(grid.m_z < ndFloat32(1 << (D_SPH_HASH_BITS * 2)));
	
		ndVector hash(grid.GetInt());
	
		m_gridHash = 0;
		m_x = ndUnsigned64(hash.m_ix);
		m_y = ndUnsigned64(hash.m_iy);
		m_z = ndUnsigned64(hash.m_iz);
		m_particleIndex = ndUnsigned64(particleIndex);
	}

	union
	{
		struct
		{
			ndUnsigned64 m_x				: D_SPH_HASH_BITS * 2;
			ndUnsigned64 m_y				: D_SPH_HASH_BITS * 2;
			ndUnsigned64 m_z				: D_SPH_HASH_BITS * 2;
			ndUnsigned64 m_particleIndex	: 64 - 3 * 2 * D_SPH_HASH_BITS - 1;
			ndUnsigned64 m_cellType			: 1;
		};
		struct
		{
			ndUnsigned64 m_xLow		: D_SPH_HASH_BITS;
			ndUnsigned64 m_xHigh	: D_SPH_HASH_BITS;
			ndUnsigned64 m_yLow		: D_SPH_HASH_BITS;
			ndUnsigned64 m_yHigh	: D_SPH_HASH_BITS;
			ndUnsigned64 m_zLow		: D_SPH_HASH_BITS;
			ndUnsigned64 m_zHigh	: D_SPH_HASH_BITS;
		};
		ndUnsigned64 m_gridHash		: 3 * 2 * D_SPH_HASH_BITS;
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
	public:
	ndWorkingBuffers()
		:m_accel(D_SPH_BUFFER_GRANULARITY)
		,m_pairCount(D_SPH_BUFFER_GRANULARITY)
		,m_gridScans(D_SPH_BUFFER_GRANULARITY)
		,m_density(D_SPH_BUFFER_GRANULARITY)
		,m_invDensity(D_SPH_BUFFER_GRANULARITY)
		,m_pairs(D_SPH_BUFFER_GRANULARITY)
		,m_hashGridMap(D_SPH_BUFFER_GRANULARITY)
		,m_hashGridMapScratchBuffer(D_SPH_BUFFER_GRANULARITY)
		,m_hashGridSize(ndFloat32 (0.0f))
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

	ndArray<ndVector> m_accel;
	ndArray<ndInt8> m_pairCount;
	ndArray<ndInt32> m_gridScans;
	ndArray<ndFloat32> m_density;
	ndArray<ndFloat32> m_invDensity;
	ndArray<ndParticlePair> m_pairs;
	ndArray<ndGridHash> m_hashGridMap;
	ndArray<ndGridHash> m_hashGridMapScratchBuffer;
	ndArray<ndParticleKernelDistance> m_kernelDistance;
	ndArray<ndInt32> m_partialsGridScans[D_MAX_THREADS_COUNT];
	ndFloat32 m_hashGridSize;
	ndFloat32 m_hashInvGridSize;
};

ndBodySphFluid::ndBodySphFluid()
	:ndBodyParticleSet()
	,m_workingBuffers(new ndWorkingBuffers)
	,m_mass(ndFloat32(0.02f))
	,m_viscosity(ndFloat32 (1.05f))
	,m_restDensity(ndFloat32(1000.0f))
	,m_gasConstant(ndFloat32(1.0f))
{
}

ndBodySphFluid::~ndBodySphFluid()
{
	delete m_workingBuffers;
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
	
	ndInt32 acc0 = 0;
	ndInt32 cellsCount = data.m_hashGridMap.GetCount();
	ndInt32 stride = cellsCount / threadCount;
	const ndGridHash* const hashGridMap = &data.m_hashGridMap[0];
	for (ndInt32 threadIndex = 0; threadIndex < threadCount; threadIndex++)
	{
		scans[threadIndex] = acc0;
		acc0 += stride;
		while (acc0 < cellsCount && (hashGridMap[acc0].m_gridHash == hashGridMap[acc0 - 1].m_gridHash))
		{
			acc0++;
		}
	}
	scans[threadCount] = cellsCount;
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
	data.m_hashGridMapScratchBuffer.SetCount(data.m_hashGridMap.GetCount());

	ndCountingSort<ndGridHash, ndKey_xlow, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, nullptr, nullptr);
	if (boxSize.m_ix > (1 << D_SPH_HASH_BITS))
	{
		ndCountingSort<ndGridHash, ndKey_xhigh, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, nullptr, nullptr);
	}

	ndCountingSort<ndGridHash, ndKey_ylow, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, nullptr, nullptr);
	if (boxSize.m_iy > (1 << D_SPH_HASH_BITS))
	{
		ndCountingSort<ndGridHash, ndKey_yhigh, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, nullptr, nullptr);
	}

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
		ndUnsigned64 key0 = cell0.m_gridHash;
		ndUnsigned64 key1 = cell1.m_gridHash;
		ndAssert(key0 <= key1);
	}
#endif
}

void ndBodySphFluid::BuildBuckets(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	ndWorkingBuffers& data = *m_workingBuffers;
	data.m_pairs.SetCount(m_posit.GetCount());

	data.m_pairCount.SetCount(m_posit.GetCount());
	data.m_kernelDistance.SetCount(m_posit.GetCount());
	for (ndInt32 i = 0; i < data.m_pairCount.GetCount(); ++i)
	{
		data.m_pairCount[i] = 0;
	}

	auto BuildBuckets = ndMakeObject::ndFunction([this, &data](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(AddPairs);
		const ndArray<ndInt32>& gridScans = data.m_gridScans;
		const ndArray<ndGridHash>& hashGridMap = data.m_hashGridMap;
		
		const ndFloat32 diameter = ndFloat32 (2.0f) * GetParticleRadius();
		const ndFloat32 diameter2 = diameter * diameter;
		ndArray<ndInt8>& pairCount = data.m_pairCount;
		ndArray<ndParticlePair>& pair = data.m_pairs;
		ndArray<ndParticleKernelDistance>& distance = data.m_kernelDistance;
		
		auto ProccessCellInSameGrid = [this, &data, &hashGridMap, &pairCount, &pair, &distance, diameter2](ndInt32 start, ndInt32 count)
		{
			for (ndInt32 i = count - 1; i > 0; --i)
			{
				const ndGridHash& grid0(hashGridMap[start + i]);
				ndInt32 particle0 = ndInt32(grid0.m_particleIndex);
				for (ndInt32 j = i - 1; j >= 0; --j)
				{
					const ndGridHash& grid1(hashGridMap[start + j]);
					ndInt32 particle1 = ndInt32(grid1.m_particleIndex);

					const ndVector p1p0(m_posit[particle0] - m_posit[particle1]);

					ndFloat32 radios2 = p1p0.DotProduct(p1p0).GetScalar();
					if (radios2 < diameter2)
					{
						radios2 = ndMax(radios2, ndFloat32(1.0e-8f));
						ndFloat32 dist = ndSqrt(radios2);
						ndInt8 index0 = pairCount[particle0];
						if (index0 < D_PARTICLE_BUCKET_SIZE)
						{
							distance[particle0].m_dist[index0] = dist;
							pair[particle0].m_neighborg[index0] = ndInt32(particle1);
							pairCount[particle0] = index0 + 1;
						}

						ndInt8 index1 = pairCount[particle1];
						if (index1 < D_PARTICLE_BUCKET_SIZE)
						{
							distance[particle1].m_dist[index1] = dist;
							pair[particle1].m_neighborg[index1] = ndInt32(particle0);
							pairCount[particle1] = index1 + 1;
						}
					}
				}
			}
		};

		const ndStartEnd startEnd(gridScans.GetCount() - 1, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			const ndInt32 start = gridScans[i + 0];
			const ndInt32 count = gridScans[i + 1] - start;
			ProccessCellInSameGrid(start, count);
		}
	});
	
	threadPool->ParallelExecute(BuildBuckets);
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
	
		const ndFloat32 h = ndFloat32 (2.0f) * GetParticleRadius();
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
	
		const ndFloat32 h = ndFloat32 (2.0f) * GetParticleRadius();
		//const ndFloat32 u = m_viscosity;
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
	const ndFloat32 diameter = ndFloat32 (2.0f) * GetParticleRadius();
	const ndFloat32 gridSize =  diameter * D_GRID_SIZE_SCALER;

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
}

void ndBodySphFluid::CreateGrids(ndThreadPool* const threadPool)
{
	D_TRACKTIME();
	ndWorkingBuffers& data = *m_workingBuffers;

	auto CountGrids = ndMakeObject::ndFunction([this, &data](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(CountGrids);
		const ndVector origin(m_box0);
		const ndVector invGridSize(data.m_hashInvGridSize);
		const ndVector particleBox(ndFloat32(1.5f) * GetParticleRadius());
		const ndVector* const posit = &m_posit[0];
		ndInt32* const scans = &data.m_gridScans[0];

		const ndStartEnd startEnd(m_posit.GetCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			const ndVector gridPosit(posit[i] - origin);
			const ndVector gridPosit0(gridPosit - particleBox);
			const ndVector gridPosit1(gridPosit + particleBox);

			//const ndGridHash hash(gridPosit * invGridSize, i);
			const ndGridHash box0Hash(gridPosit0 * invGridSize, i);
			const ndGridHash box1Hash(gridPosit1 * invGridSize, i);
			
			ndInt32 dx = ndInt32(box1Hash.m_x - box0Hash.m_x + 1);
			ndInt32 dy = ndInt32(box1Hash.m_y - box0Hash.m_y + 1);
			ndInt32 dz = ndInt32(box1Hash.m_z - box0Hash.m_z + 1);
			ndAssert(dx >= 1);
			ndAssert(dy >= 1);
			ndAssert(dz >= 1);
			ndAssert(dx <= 2);
			ndAssert(dy <= 2);
			ndAssert(dz <= 2);
			ndInt32 count = dz * dy * dx;
			scans[i] = count;
		}
	});

	auto CreateGrids = ndMakeObject::ndFunction([this, &data](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(CreateGrids);
		const ndVector origin(m_box0);
		const ndVector invGridSize(data.m_hashInvGridSize);
		const ndVector particleBox(ndFloat32(1.5f) * GetParticleRadius());
		const ndVector* const posit = &m_posit[0];
		ndInt32* const scans = &data.m_gridScans[0];
		ndGridHash* const dst = &data.m_hashGridMap[0];

		const ndStartEnd startEnd(m_posit.GetCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			const ndVector gridPosit(posit[i] - origin);
			const ndVector gridPosit0(gridPosit - particleBox);
			const ndVector gridPosit1(gridPosit + particleBox);
			const ndGridHash centerHash(gridPosit * invGridSize, i);
			const ndGridHash box0Hash(gridPosit0 * invGridSize, i);
			const ndGridHash box1Hash(gridPosit1 * invGridSize, i);

			ndInt32 base = scans[i];
			for (ndInt32 z = ndInt32(box0Hash.m_z); z <= ndInt32(box1Hash.m_z); z++)
			{
				for (ndInt32 y = ndInt32(box0Hash.m_y); y <= ndInt32(box1Hash.m_y); y++)
				{
					for (ndInt32 x = ndInt32(box0Hash.m_x); x <= ndInt32(box1Hash.m_x); x++)
					{
						ndGridHash hash(x, y, z, i);
						hash.m_cellType = (hash.m_gridHash == centerHash.m_gridHash) ? ndGridHash::m_homeGrid : ndGridHash::m_adajacent;
						dst[base] = hash;
						base++;
					}
				}
			}
			ndAssert(base == scans[i + 1]);
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
	//data.m_hashGridMapScratchBuffer.SetCount(gridCount);
	threadPool->ParallelExecute(CreateGrids);
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