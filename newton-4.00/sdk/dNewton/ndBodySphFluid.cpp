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

#ifdef _DEBUG
	//#define D_DEBUG_SOLVER
#endif

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

#ifdef D_DEBUG_SOLVER
	class ndPair
	{
		public:
		ndPair() {}
		ndPair(ndInt32 m0, ndInt32 m1)
			:m_m0(m0)
			,m_m1(m1)
		{
		}

		bool operator< (const ndPair& other) const
		{
			ndUnsigned64 key0 = (ndUnsigned64(m_m1) << 32) + m_m0;
			ndUnsigned64 key1 = (ndUnsigned64(other.m_m1) << 32) + other.m_m0;
			return key0 < key1;
		}

		bool operator> (const ndPair& other) const
		{
			ndUnsigned64 key0 = (ndUnsigned64(m_m1) << 32) + m_m0;
			ndUnsigned64 key1 = (ndUnsigned64(other.m_m1) << 32) + other.m_m0;
			return key0 > key1;
		}

		ndInt32 m_m0;
		ndInt32 m_m1;
	};
#endif
};

ndBodySphFluid::ndBodySphFluid()
	:ndBodyParticleSet()
	,ndBackgroundJob()
	,m_mass(ndFloat32(0.02f))
	,m_viscosity(ndFloat32 (1.05f))
	,m_restDensity(ndFloat32(1000.0f))
	,m_gasConstant(ndFloat32(1.0f))
{
}

ndBodySphFluid::ndBodySphFluid(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndBodyParticleSet(desc)
	,ndBackgroundJob()
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

ndBodySphFluid::ndWorkingData& ndBodySphFluid::WorkingData()
{
	static ndWorkingData workingBuffers;
	return workingBuffers;
}

//void ndBodySphFluid::Save(const dLoadSaveBase::dSaveDescriptor& desc) const
void ndBodySphFluid::Save(const ndLoadSaveBase::ndSaveDescriptor&) const
{
	dAssert(0);
	//nd::TiXmlElement* const paramNode = CreateRootElement(rootNode, "ndBodySphFluid", nodeid);
	//ndBodyParticleSet::Save(paramNode, assetPath, nodeid, shapesCache);
}

void ndBodySphFluid::CaculateAabb()
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

	class ndContext
	{
		public:
		ndBox m_boxes[D_MAX_THREADS_COUNT];
		const ndBodySphFluid* m_fluid;
	};

	class ndCalculateAabb : public ndThreadPoolJob_old
	{
		virtual void Execute()
		{
			D_TRACKTIME();
			ndContext* const context = (ndContext*)GetContext();
			const ndBodySphFluid* const fluid = context->m_fluid;
			const ndArray<ndVector>& posit = fluid->m_posit;

			ndBox box;
			const ndStartEnd startEnd(posit.GetCount(), GetThreadId(), GetThreadCount());
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				box.m_min = box.m_min.GetMin(posit[i]);
				box.m_max = box.m_max.GetMax(posit[i]);
			}
			context->m_boxes[GetThreadId()] = box;
		}
	};

	ndContext context;
	context.m_fluid = this;
	ndThreadBackgroundWorker* const threadPool = GetThreadPool();
	threadPool->SubmitJobs<ndCalculateAabb>(&context);

	ndBox box;
	const ndInt32 threadCount = threadPool->GetThreadCount();
	for (ndInt32 i = 0; i < threadCount; ++i)
	{
		box.m_min = box.m_min.GetMin(context.m_boxes[i].m_min);
		box.m_max = box.m_max.GetMax(context.m_boxes[i].m_max);
	}
	
	const ndFloat32 gridSize = GetSphGridSize();
	
	ndVector grid(gridSize);
	ndVector invGrid(ndFloat32 (1.0f) / gridSize);
	
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

void ndBodySphFluid::SortXdimension()
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
	ndThreadBackgroundWorker* const threadPool = GetThreadPool();

	ndCountingSortOld<ndGridHash, ndKey_low, 8>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, this);
	const ndInt32 keySize = data.WorldToGrid(m_box1.m_x);
	if (keySize >= 256)
	{
		ndCountingSortOld<ndGridHash, ndKey_middle, 8>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, this);
		if (keySize >= (256 * 256))
		{
			ndCountingSortOld<ndGridHash, ndKey_high, 8>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer, this);
		}
	}
}

void ndBodySphFluid::SortCellBuckects()
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
	ndThreadPool* const threadPool = GetThreadPool();

	ndVector boxSize((m_box1 - m_box0).Scale(ndFloat32(1.0f) / GetSphGridSize()).GetInt());
	ndCountingSortOld<ndGridHash, ndKey_ylow, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer);
	if (boxSize.m_iy > (1 << D_SPH_HASH_BITS))
	{
		ndCountingSortOld<ndGridHash, ndKey_yhigh, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer);
	}
	
	ndCountingSortOld<ndGridHash, ndKey_zlow, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer);
	if (boxSize.m_iz > (1 << D_SPH_HASH_BITS))
	{
		ndCountingSortOld<ndGridHash, ndKey_zhigh, D_SPH_HASH_BITS>(*threadPool, data.m_hashGridMap, data.m_hashGridMapScratchBuffer);
	}
}

void ndBodySphFluid::CalculateScans()
{
	D_TRACKTIME();
	class ndContext
	{
		public:
		ndContext(ndBodySphFluid* const fluid)
			:m_fluid(fluid)
		{
			memset(m_scan, 0, sizeof(m_scan));

			const ndInt32 threadCount = m_fluid->GetThreadPool()->GetThreadCount();

			ndWorkingData& data = m_fluid->WorkingData();
			ndInt32 particleCount = data.m_hashGridMap.GetCount();

			ndInt32 acc0 = 0;
			ndInt32 stride = particleCount / threadCount;
			const ndGridHash* const hashGridMap = &data.m_hashGridMap[0];
			for (ndInt32 threadIndex = 0; threadIndex < threadCount; threadIndex++)
			{
				m_scan[threadIndex] = acc0;
				acc0 += stride;
				while (acc0 < particleCount && (hashGridMap[acc0].m_gridHash == hashGridMap[acc0 - 1].m_gridHash))
				{
					acc0++;
				}
			}
			m_scan[threadCount] = particleCount;
		}

		ndBodySphFluid* m_fluid;
		ndInt32 m_sums[D_MAX_THREADS_COUNT + 1];
		ndInt32 m_scan[D_MAX_THREADS_COUNT + 1];
	};

	class ndCountGridScans : public ndThreadPoolJob_old
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndContext* const context = (ndContext*)GetContext();
		
			const ndInt32 threadIndex = GetThreadId();
			ndBodySphFluid* const fluid = context->m_fluid;
			ndWorkingData& data = fluid->WorkingData();

			const ndGridHash* const hashGridMap = &data.m_hashGridMap[0];
				
			const ndInt32 start = context->m_scan[threadIndex];
			const ndInt32 end = context->m_scan[threadIndex + 1];
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
		}
	};

	class ndCalculateScans : public ndThreadPoolJob_old
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndContext* const context = (ndContext*)GetContext();
			const ndInt32 threadIndex = GetThreadId();
			ndBodySphFluid* const fluid = context->m_fluid;
			ndWorkingData& data = fluid->WorkingData();

			ndArray<ndInt32>& gridScans = data.m_gridScans;
			const ndArray<ndInt32>& partialScan = data.m_partialsGridScans[threadIndex];
			const ndInt32 base = context->m_sums[threadIndex];
			ndInt32 sum = context->m_scan[threadIndex];
			for (ndInt32 i = 0; i < partialScan.GetCount(); ++i)
			{
				gridScans[base + i] = sum;
				sum += partialScan[i];
			}
		}
	};

	ndThreadPool* const threadPool = GetThreadPool();
	ndContext context(this);
	threadPool->SubmitJobs<ndCountGridScans>(&context);

	ndInt32 scansCount = 0;
	ndWorkingData& data = WorkingData();
	const ndInt32 threadCount = threadPool->GetThreadCount();
	for (ndInt32 i = 0; i < threadCount; ++i)
	{
		context.m_sums[i] = scansCount;
		scansCount += data.m_partialsGridScans[i].GetCount();
	}
	context.m_sums[threadCount] = scansCount;
	data.m_gridScans.SetCount(scansCount + 1);
	threadPool->SubmitJobs<ndCalculateScans>(&context);
	data.m_gridScans[scansCount] = context.m_scan[threadCount];
}

void ndBodySphFluid::CreateGrids()
{
	//#define D_USE_PARALLEL_CLASSIFY
	D_TRACKTIME();
	class ndGridCounters
	{
		public:
		ndInt32 m_scan[D_MAX_THREADS_COUNT][2];
	};

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

	class ndContext
	{
		public:
		ndContext(ndBodySphFluid* const fluid)
			:m_scan()
			,m_neighbors()
			,m_fluid(fluid)
		{
		}

		ndGridCounters m_scan;
		ndGridNeighborInfo m_neighbors;
		ndBodySphFluid* m_fluid;
	};

	class ndCreateGrids: public ndThreadPoolJob_old
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndContext* const context = (ndContext*)GetContext();
			ndBodySphFluid* const fluid = context->m_fluid;
			ndWorkingData& data = fluid->WorkingData();
			ndGridHash* const dst = &data.m_hashGridMapScratchBuffer[0];

			const ndFloat32 gridSize = fluid->GetSphGridSize();
			const ndVector origin(fluid->m_box0);
			// the 0.99 factor is to make sure the box 
			// fits in not more than two adjacent grids.
			const ndVector box(gridSize * ndFloat32(0.5f * 0.99f));
			const ndVector invGridSize(ndFloat32(1.0f) / gridSize);
			const ndVector* const posit = &fluid->m_posit[0];
			
			const ndGridNeighborInfo& gridNeighborInfo = context->m_neighbors;

			#ifdef D_USE_PARALLEL_CLASSIFY
			ndInt32* const scan = &context->m_scan.m_scan[GetThreadId()][0];
			scan[0] = 0;
			scan[1] = 0;
			#endif

			const ndStartEnd startEnd(fluid->m_posit.GetCount(), GetThreadId(), GetThreadCount());
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
				scan[0] += gridNeighborInfo.m_counter[code];
				scan[1] += (4 - gridNeighborInfo.m_counter[code]);
				#endif

				const ndInt8* const padding = &gridNeighborInfo.m_isPadd[code][0];
				const ndGridHash* const neigborgh = &gridNeighborInfo.m_neighborDirs[code][0];
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
		}
	};

	class ndCompactGrids : public ndThreadPoolJob_old
	{
		public:
		void Execute()
		{
			D_TRACKTIME();
			ndContext* const context = (ndContext*)GetContext();
			ndBodySphFluid* const fluid = context->m_fluid;
			ndWorkingData& data = fluid->WorkingData();
			
			ndArray<ndGridHash>& dst = data.m_hashGridMap;
			const ndGridHash* const src = &data.m_hashGridMapScratchBuffer[0];
			
			const ndInt32 threadID = GetThreadId();
			const ndInt32 threadCount = GetThreadCount();

			ndInt32* const scan = &context->m_scan.m_scan[threadID][0];

			const ndStartEnd startEnd(dst.GetCount(), threadID, threadCount);
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				const ndGridHash grid(src[i]);
				const ndInt32 key = grid.m_cellIsPadd;
				const ndInt32 index = scan[key];
				dst[index] = grid;
				scan[key] = index + 1;
			}
		}
	};

	dAssert (sizeof(ndGridHash) <= 16);
	ndThreadPool* const threadPool = GetThreadPool();

	ndContext context(this);
	ndWorkingData& data = WorkingData();
	data.m_hashGridMap.SetCount(m_posit.GetCount() * 4);
	data.m_hashGridMapScratchBuffer.SetCount(m_posit.GetCount() * 4);
	threadPool->SubmitJobs<ndCreateGrids>(&context);

#ifdef D_USE_PARALLEL_CLASSIFY
	ndInt32 sum = 0;
	const ndInt32 threadCount = scene->GetThreadCount();
	for (ndInt32 i = 0; i < 2; ++i)
	{
		for (ndInt32 j = 0; j < threadCount; ++j)
		{
			ndInt32 partialSum = context.m_scan.m_scan[j][i];
			context.m_scan.m_scan[j][i] = sum;
			sum += partialSum;
		}
	}

	ndInt32 gridCount = context.m_scan.m_scan[0][1] - context.m_scan.m_scan[0][0];
	scene->SubmitJobs<ndCompactGrids>(&context);
#else
	ndInt32 gridCount = 0;
	{
		D_TRACKTIME();
		// this seems to be very cache friendly and beating radix sort hand down.
		for (ndInt32 i = 0; i < data.m_hashGridMapScratchBuffer.GetCount(); ++i)
		{
			const ndGridHash cell(data.m_hashGridMapScratchBuffer[i]);
			data.m_hashGridMap[gridCount] = cell;
			gridCount += (1 - ndInt32 (cell.m_cellIsPadd));
		}
	}
#endif
	data.m_hashGridMap.SetCount(gridCount);
}

void ndBodySphFluid::SortGrids()
{
	D_TRACKTIME();
	SortXdimension();
	SortCellBuckects();

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

void ndBodySphFluid::BuildPairs()
{
	D_TRACKTIME();
	class ndAddPairs : public ndThreadPoolJob_old
	{
		public:
		class ndPairInfo
		{
			public:
			ndPairInfo(ndBodySphFluid* const fluid)
				:m_data(fluid->WorkingData())
				,m_locks(m_data.m_locks)
				,m_posit(fluid->m_posit)
				,m_pairCount(m_data.m_pairCount)
				,m_pair(m_data.m_pairs)
				,m_distance(m_data.m_kernelDistance)
				,m_diameter(fluid->GetSphGridSize())
				,m_diameter2(m_diameter * m_diameter)
				,m_windosTest(m_data.WorldToGrid(m_data.m_worlToGridOrigin + m_diameter) + 1)
			{
			}

			ndWorkingData& m_data;
			ndArray<ndSpinLock>& m_locks;
			const ndArray<ndVector>& m_posit;
			ndArray<ndInt8>& m_pairCount;
			ndArray<ndParticlePair>& m_pair;
			ndArray<ndParticleKernelDistance>& m_distance;
			ndFloat32 m_diameter;
			ndFloat32 m_diameter2;
			ndInt32 m_windosTest;
		};

		void AddPair(ndPairInfo& info, ndInt32 particle0, ndInt32 particle1)
		{
			ndVector p1p0(info.m_posit[particle0] - info.m_posit[particle1]);
			ndFloat32 dist2(p1p0.DotProduct(p1p0).GetScalar());
			if (dist2 < info.m_diameter2)
			{
				dAssert(dist2 >= ndFloat32(0.0f));
				ndFloat32 dist = ndSqrt(dist2);
				{
					ndSpinLock lock(info.m_locks[particle0]);
					ndInt8 count = info.m_pairCount[particle0];
					if (count < 32)
					{
						ndInt8 isUnique = 1;
						ndInt32* const neighborg = info.m_pair[particle0].m_neighborg;
						for (ndInt32 i = count - 1; i >= 0; --i)
						{
							isUnique = isUnique & (neighborg[i] != particle1);
						}

						neighborg[count] = particle1;
						info.m_distance[particle0].m_dist[count] = dist;
						info.m_pairCount[particle0] = count + isUnique;
					}
				}
			
				{
					ndSpinLock lock(info.m_locks[particle1]);
					ndInt8 count = info.m_pairCount[particle1];
					if (count < 32)
					{
						ndInt8 isUnique = 1;
						ndInt32* const neighborg = info.m_pair[particle1].m_neighborg;
						for (ndInt32 i = count - 1; i >= 0; --i)
						{
							isUnique = isUnique & (neighborg[i] != particle0);
						}
						neighborg[count] = particle0;
						info.m_distance[particle1].m_dist[count] = dist;
						info.m_pairCount[particle1] = count + isUnique;
					}
				}
			}
		}

		void proccessCell(const ndArray<ndGridHash>& hashGridMap, ndPairInfo& info, const ndInt32 start, const ndInt32 count)
		{
			//D_TRACKTIME();
			const ndInt32 count0 = count - 1;
			for (ndInt32 i = 0; i < count0; ++i)
			{
				const ndGridHash hash0 = hashGridMap[start + i];
				const ndInt32 homeGridTest0 = (hash0.m_cellType == ndHomeGrid);
				const ndInt32 particle0 = hash0.m_particleIndex;
				const ndInt32 x0 = info.m_data.WorldToGrid(info.m_posit[particle0].m_x);
				for (ndInt32 j = i + 1; j < count; ++j)
				{
					const ndGridHash hash1 = hashGridMap[start + j];
					const ndInt32 particle1 = hash1.m_particleIndex;
					dAssert(particle0 != particle1);
					const ndInt32 x1 = info.m_data.WorldToGrid(info.m_posit[particle1].m_x);
					dAssert((x1 - x0) > ndFloat32 (-1.0e-3f));
					const ndInt32 sweeptTest = ((x1 - x0) >= info.m_windosTest);
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
						AddPair(info, particle0, particle1);
					}
				}
			}
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			ndBodySphFluid* const fluid = (ndBodySphFluid*)GetContext();
			ndWorkingData& data = fluid->WorkingData();
			
			const ndArray<ndGridHash>& hashGridMap = data.m_hashGridMap;
			const ndArray<ndInt32>& gridScans = data.m_gridScans;

			ndPairInfo info(fluid);
						
			const ndStartEnd startEnd(gridScans.GetCount() - 1, GetThreadId(), GetThreadCount());
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				const ndInt32 start = gridScans[i];
				const ndInt32 count = gridScans[i + 1] - start;
				proccessCell(hashGridMap, info, start, count);
			}
		}
	};

	//class ndRemoveDuplicates : public ndThreadPoolJob
	//{
	//	public:
	//	virtual void Execute()
	//	{
	//		D_TRACKTIME();
	//		ndScene* const scene = (ndScene*)GetThreadPool();
	//		ndWorld* const world = scene->GetWorld();
	//		ndBodySphFluid* const fluid = (ndBodySphFluid*)GetContext();
	//		ndWorkingData& data = fluid->WorkingData();
	//
	//		ndArray<ndParticlePair>& pairs = data.m_pairs;
	//		ndArray<ndInt8>& pairsCount = data.m_pairCount;
	//
	//		const ndStartEnd startEnd(pairsCount.GetCount() - 1, GetThreadId(), world->GetThreadCount());
	//		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
	//		{
	//			ndInt32 count = pairsCount[i];
	//			ndParticlePair& pair = pairs[i];
	//			for (ndInt32 j = count - 1; j >= 1; --j)
	//			{
	//				ndInt32 m = pair.m_neighborg[j];
	//				for (ndInt32 k = j - 1; k >= 0; --k)
	//				{
	//					if (m == pair.m_neighborg[k])
	//					{
	//						--count;
	//						pair.m_neighborg[j] = pair.m_neighborg[count];
	//						break;
	//					}
	//				}
	//			}
	//			pairsCount[i] = ndInt8 (count);
	//		}
	//	}
	//};

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

	ndThreadBackgroundWorker* const threadPool = GetThreadPool();
	threadPool->SubmitJobs<ndAddPairs>(this);

#ifdef D_DEBUG_SOLVER

	ndTree <ndInt32, ndWorkingData::ndPair> pairfilter0;
	ndArray<ndInt8>& pairsCount = data.m_pairCount;
	ndArray<ndParticlePair>& pairs = data.m_pairs;
	ndArray<ndVector> testPosit;
	for (ndInt32 i = 0; i < m_posit.GetCount(); i++)
	{
		ndVector p(m_posit[i]);
		p.m_w = ndFloat32(i);
		testPosit.PushBack(p);
	
		ndInt32 count = pairsCount[i];
		ndParticlePair& pair = pairs[i];
		for (ndInt32 j = 0; j < count; ++j)
		{
			pairfilter0.Insert(ndWorkingData::ndPair(i, pair.m_neighborg[j]));
		}
	}

	class ComparePosit
	{
		public:
		ndInt32 Compare(const ndVector& p0, const ndVector& p1, void* const) const
		{
			if (p0.m_x < p1.m_x)
			{
				return -1;
			}
			if (p0.m_x > p1.m_x)
			{
				return 1;
			}
			return 0;
		}
	};
	ndSort<ndVector, ComparePosit>(&testPosit[0], m_posit.GetCount());

	ndTree<ndInt32, ndWorkingData::ndPair> pairfilter;
	ndFloat32 h = GetSphGridSize();
	ndFloat32 h2 = h * h;
	for (ndInt32 i = 0; i < testPosit.GetCount() - 1; i++)
	{
		ndFloat32 x0 = testPosit[i].m_x;
		ndInt32 i0 = ndInt32(testPosit[i].m_w);
		for (ndInt32 j = i + 1; j < testPosit.GetCount(); j++)
		{
			ndFloat32 x1 = testPosit[j].m_x;
			if (x1 >= (x0 + h))
			{
				break;
			}
			ndVector dist2(ndVector::m_triplexMask & (testPosit[i] - testPosit[j]));
			ndFloat32 d2 = dist2.DotProduct(dist2).GetScalar();
			if (d2 < h2)
			{
				ndInt32 i1 = ndInt32(testPosit[j].m_w);
				pairfilter.Insert(0, ndWorkingData::ndPair(i0, i1));
				pairfilter.Insert(0, ndWorkingData::ndPair(i1, i0));
			}
		}
	}
	dAssert(pairfilter.GetCount() == pairfilter0.GetCount());
#endif
}

void ndBodySphFluid::CalculateParticlesDensity()
{
	D_TRACKTIME();
	class CalculateDensity : public ndThreadPoolJob_old
	{
		virtual void Execute()
		{
			D_TRACKTIME();
			ndBodySphFluid* const fluid = (ndBodySphFluid*)GetContext();
			const ndArray<ndVector>& posit = fluid->m_posit;
	
			ndWorkingData& data = fluid->WorkingData();
			const ndFloat32 h = fluid->GetSphGridSize();
			const ndFloat32 h2 = h * h;
			const ndFloat32 kernelMagicConst = ndFloat32(315.0f) / (ndFloat32(64.0f) * ndPi * ndPow(h, 9));
			const ndFloat32 kernelConst = fluid->m_mass * kernelMagicConst;
			const ndFloat32 selfDensity = kernelConst * h2 * h2 * h2;

			const ndStartEnd startEnd(posit.GetCount(), GetThreadId(), GetThreadCount());
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
				data.m_invDensity[i] = ndFloat32 (1.0f) / density;
			}
		}
	};

	ndWorkingData& data = WorkingData();
	data.m_density.SetCount(m_posit.GetCount());
	data.m_invDensity.SetCount(m_posit.GetCount());
	ndThreadBackgroundWorker* const threadPool = GetThreadPool();
	threadPool->SubmitJobs<CalculateDensity>(this);
}

void ndBodySphFluid::CalculateAccelerations()
{
	D_TRACKTIME();
	class ndCalculateAcceleration : public ndThreadPoolJob_old
	{
		inline ndVector Normalize(const ndVector& dir) const 
		{
			const ndVector dot (dir.DotProduct(dir) + m_epsilon2);
			const ndVector normal(dir * dot.InvSqrt());
			return normal;
		}

		virtual void Execute()
		{
			D_TRACKTIME();

			m_epsilon2 = ndVector(ndFloat32(1.0e-12f));
			ndBodySphFluid* const fluid = (ndBodySphFluid*)GetContext();

			ndWorkingData& data = fluid->WorkingData();
			const ndArray<ndVector>& veloc = fluid->m_veloc;
			const ndArray<ndVector>& posit = fluid->m_posit;
			const ndFloat32* const density = &data.m_density[0];
			const ndFloat32* const invDensity = &data.m_invDensity[0];

			const ndFloat32 h = fluid->GetSphGridSize();
			const ndFloat32 u = fluid->m_viscosity;
			const ndVector kernelConst(fluid->m_mass * ndFloat32(45.0f) / (ndPi * ndPow(h, 6)));

			const ndFloat32 viscosity = fluid->m_viscosity;
			const ndFloat32 restDensity = fluid->m_restDensity;
			const ndFloat32 gasConstant = ndFloat32 (0.5f) * fluid->m_gasConstant;

			const ndVector gravity(fluid->m_gravity);
			const ndStartEnd startEnd(posit.GetCount(), GetThreadId(), GetThreadCount());
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
					const ndVector dir(Normalize(p10));
					dAssert(dir.m_w == ndFloat32(0.0f));
				
					// kernel distance
					const ndFloat32 dist = distance.m_dist[j];
					const ndFloat32 kernelDist = h - dist;
					dAssert(kernelDist >= ndFloat32(0.0f));
				
					// calculate pressure
					const ndFloat32 kernelDist2 = kernelDist * kernelDist;
					const ndFloat32 pressureI1 = density[i1] - restDensity;
					const ndVector force(gasConstant * kernelDist2 * invDensity[i1] * (pressureI0 + pressureI1));
					forceAcc += force * dir;
				
					// calculate viscosity acceleration
					const ndVector v01(veloc[i1] - v0);
					forceAcc += v01 * ndVector(kernelDist * viscosity * invDensity[j]);
				}
				const ndVector accel(gravity + ndVector(invDensity[i0]) * kernelConst * forceAcc);
				data.m_accel[i0] = accel;
			}
		}

		ndVector m_epsilon2;
	};

	ndWorkingData& data = WorkingData();
	data.m_accel.SetCount(m_posit.GetCount());
	ndThreadBackgroundWorker* const threadPool = GetThreadPool();
	threadPool->SubmitJobs<ndCalculateAcceleration>(this);
}

void ndBodySphFluid::IntegrateParticles()
{
	D_TRACKTIME();
	class ndContext
	{
		public:
		ndContext(ndBodySphFluid* const fluid, ndFloat32 timestep)
			:m_fluid(fluid)
			,m_timestep(timestep)
		{
			m_timestep = 0.003f;
		}

		ndBodySphFluid* m_fluid;
		ndFloat32 m_timestep;
	};

	class ndIntegrateParticles : public ndThreadPoolJob_old
	{
		virtual void Execute()
		{
			D_TRACKTIME();
			const ndContext& context = *((ndContext*)GetContext());
			ndBodySphFluid* const fluid = context.m_fluid;

			ndWorkingData& data = fluid->WorkingData();
			const ndArray<ndVector>& accel = data.m_accel;
			ndArray<ndVector>& veloc = fluid->m_veloc;
			ndArray<ndVector>& posit = fluid->m_posit;

			ndVector timestep(context.m_timestep);
			ndVector halfTime(timestep * ndVector::m_half);
			const ndStartEnd startEnd(posit.GetCount(), GetThreadId(), GetThreadCount());
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
		}
	};

	ndContext context(this, m_timestep);
	ndThreadBackgroundWorker* const threadPool = GetThreadPool();
	threadPool->SubmitJobs<ndIntegrateParticles>(&context);
}

void ndBodySphFluid::Execute()
{
	// do the scene management 
	D_TRACKTIME();
	CaculateAabb();
	CreateGrids();
	SortGrids();
	CalculateScans();
	BuildPairs();
	CalculateParticlesDensity();
	CalculateAccelerations();
	IntegrateParticles();
}

void ndBodySphFluid::Update(const ndWorld* const world, ndFloat32 timestep)
{
	m_timestep = timestep;
	if (JobState() == ndBackgroundJob::m_jobCompleted)
	{
		ndScene* const scene = world->GetScene();
		scene->SendBackgroundJob(this);
	}
}