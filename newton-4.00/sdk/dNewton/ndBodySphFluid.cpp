/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndBodySphFluid.h"


/*
{
dFloat32 xxx[6][6][6];
for (dInt32 i = 0; i < 6 * 6 * 6; i++)
{
dFloat32* yyy = &xxx[0][0][0];
yyy[i] = 1.0f;
}
for (dInt32 i = 0; i < uniqueCount; i++)
{
dInt32 x = m_hashGridMap[i].m_x;
dInt32 y = m_hashGridMap[i].m_y;
dInt32 z = m_hashGridMap[i].m_z;

xxx[z][y][x] = 0.0f;
}

dIsoSurfaceOld isoSurcase;
isoSurcase.GenerateSurface(&xxx[0][0][0], 0.5f, 5, 5, 5, gridSize, gridSize, gridSize);
cellCount *= 1;
}
*/


ndBodySphFluid::ndBodySphFluid()
	:ndBodyParticleSet()
	,m_box0(dFloat32(-1e10f))
	,m_box1(dFloat32(1e10f))
	,m_hashGridMap(1024)
	,m_hashGridMapScratchBuffer(1024)
{
}

ndBodySphFluid::ndBodySphFluid(const nd::TiXmlNode* const xmlNode, const dTree<const ndShape*, dUnsigned32>& shapesCache)
	:ndBodyParticleSet(xmlNode->FirstChild("ndBodyKinematic"), shapesCache)
	,m_box0(dFloat32(-1e10f))
	,m_box1(dFloat32(1e10f))
	,m_hashGridMap()
	,m_hashGridMapScratchBuffer()
{
	// nothing was saved
	dAssert(0);
}

ndBodySphFluid::~ndBodySphFluid()
{
}

void ndBodySphFluid::Save(nd::TiXmlElement* const rootNode, const char* const assetPath, dInt32 nodeid, const dTree<dUnsigned32, const ndShape*>& shapesCache) const
{
	dAssert(0);
	nd::TiXmlElement* const paramNode = CreateRootElement(rootNode, "ndBodySphFluid", nodeid);
	ndBodyParticleSet::Save(paramNode, assetPath, nodeid, shapesCache);
}

void ndBodySphFluid::AddParticle(const dFloat32 mass, const dVector& position, const dVector& velocity)
{
	dVector point(position);
	point.m_w = dFloat32(1.0f);
	m_posit.PushBack(point);
}

void ndBodySphFluid::CaculateAABB(const ndWorld* const world, dVector& boxP0, dVector& boxP1) const
{
	D_TRACKTIME();
	dVector box0(dFloat32(1e20f));
	dVector box1(dFloat32(-1e20f));
	for (dInt32 i = m_posit.GetCount() - 1; i >= 0; i--)
	{
		box0 = box0.GetMin(m_posit[i]);
		box1 = box1.GetMax(m_posit[i]);
	}
	boxP0 = box0;
	boxP1 = box1;
}

void ndBodySphFluid::Update(const ndWorld* const world, dFloat32 timestep)
{
	dVector boxP0;
	dVector boxP1;
	CaculateAABB(world, boxP0, boxP1);
	const dFloat32 gridSize = CalculateGridSize();
	m_box0 = boxP0 - dVector(gridSize);
	m_box1 = boxP1 + dVector(gridSize);

	CreateGrids(world);
	SortGrids(world);
}

void ndBodySphFluid::CreateGrids(const ndWorld* const world)
{
	D_TRACKTIME();
	class ndCreateGrids: public ndScene::ndBaseJob
	{
		public:
		class ndContext
		{
			public:
			ndBodySphFluid* m_fluid;
			//dAtomic<dInt32> m_iterator;
		};

		//#define D_SCRATCH_BUFFER_SIZE (1024 * 4)

		void AddCell(dInt32 count, const ndGridHash& origin, const ndGridHash& cell, const ndGridHash* const neigborgh, dArray<ndGridHash>& buffer)
		{
			#ifdef _DEBUG 
			int xxxxx = 0;
			#endif
			for (dInt32 j = 0; j < count; j++)
			{
				ndGridHash quadrand(cell);
				quadrand.m_gridHash += neigborgh[j].m_gridHash;
				quadrand.m_cellType = (quadrand.m_gridHash == origin.m_gridHash) ? ndHomeGrid : ndAdjacentGrid;
				
				//m_scratchBuffer[m_scratchBufferCount] = quadrand;
				//m_scratchBufferCount++;
				buffer.PushBack(quadrand);
				#ifdef _DEBUG 
				xxxxx += quadrand.m_cellType ? 1 : 0;
				#endif
			}
			dAssert(xxxxx == 1);
		}

		virtual void Execute()
		{
			D_TRACKTIME();

			dVector m_neighborkDirs[8];

			ndBodySphFluid* const fluid = ((ndContext*)m_context)->m_fluid;
			const dFloat32 radius = fluid->m_radius;

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 particleCount = fluid->m_posit.GetCount();

			const dInt32 step = particleCount / threadCount;
			const dInt32 start = threadIndex * step;
			const dInt32 count = ((threadIndex + 1) < threadCount) ? step : particleCount - start;

			const dFloat32 gridSize = fluid->CalculateGridSize();

			const dVector origin(fluid->m_box0);
			const dVector invGridSize(dFloat32(1.0f) / gridSize);
			const dVector* const posit = &fluid->m_posit[0];

			const dVector box0(-radius, -radius, -radius, dFloat32(0.0f));
			const dVector box1(radius, radius, radius, dFloat32(0.0f));

			ndGridHash stepsCode_xyz[8];
			stepsCode_xyz[0] = ndGridHash(0, 0, 0);
			stepsCode_xyz[1] = ndGridHash(1, 0, 0);
			stepsCode_xyz[2] = ndGridHash(0, 1, 0);
			stepsCode_xyz[3] = ndGridHash(1, 1, 0);
			stepsCode_xyz[4] = ndGridHash(0, 0, 1);
			stepsCode_xyz[5] = ndGridHash(1, 0, 1);
			stepsCode_xyz[6] = ndGridHash(0, 1, 1);
			stepsCode_xyz[7] = ndGridHash(1, 1, 1);

			ndGridHash stepsCode_xy[4];
			stepsCode_xy[0] = ndGridHash(0, 0, 0);
			stepsCode_xy[1] = ndGridHash(1, 0, 0);
			stepsCode_xy[2] = ndGridHash(0, 1, 0);
			stepsCode_xy[3] = ndGridHash(1, 1, 0);

			ndGridHash stepsCode_yz[4];
			stepsCode_yz[0] = ndGridHash(0, 0, 0);
			stepsCode_yz[1] = ndGridHash(0, 1, 0);
			stepsCode_yz[2] = ndGridHash(0, 0, 1);
			stepsCode_yz[3] = ndGridHash(0, 1, 1);

			ndGridHash stepsCode_xz[4];
			stepsCode_xz[0] = ndGridHash(0, 0, 0);
			stepsCode_xz[1] = ndGridHash(1, 0, 0);
			stepsCode_xz[2] = ndGridHash(0, 0, 1);
			stepsCode_xz[3] = ndGridHash(1, 0, 1);

			ndGridHash stepsCode_x[2];
			stepsCode_x[0] = ndGridHash(0, 0, 0);
			stepsCode_x[1] = ndGridHash(1, 0, 0);

			ndGridHash stepsCode_y[2];
			stepsCode_y[0] = ndGridHash(0, 0, 0);
			stepsCode_y[1] = ndGridHash(0, 1, 0);

			ndGridHash stepsCode_z[2];
			stepsCode_z[0] = ndGridHash(0, 0, 0);
			stepsCode_z[1] = ndGridHash(0, 0, 1);

			dArray<ndGridHash>& bufferOut = fluid->m_hashGridMapThreadBuffers[threadIndex];
			bufferOut.SetCount(0);
			for (dInt32 i = 0; i < count; i++)
			{
				dVector r(posit[start + i] - origin);
				dVector p(r * invGridSize);
				ndGridHash hashKey(p, i);

				fluid->m_upperDigisIsValid[0] |= hashKey.m_xHigh;
				fluid->m_upperDigisIsValid[1] |= hashKey.m_yHigh;
				fluid->m_upperDigisIsValid[2] |= hashKey.m_zHigh;
				dVector p0((r + box0) * invGridSize);
				dVector p1((r + box1) * invGridSize);

				ndGridHash box0Hash(p0, i);
				ndGridHash box1Hash(p1, i);
				ndGridHash codeHash(box1Hash.m_gridHash - box0Hash.m_gridHash);

				dAssert(codeHash.m_x <= 1);
				dAssert(codeHash.m_y <= 1);
				dAssert(codeHash.m_z <= 1);
				dUnsigned32 code = dUnsigned32(codeHash.m_z * 4 + codeHash.m_y * 2 + codeHash.m_x);

				switch (code)
				{
					case 0:
					{
						box0Hash.m_cellType = ndHomeGrid;
						bufferOut.PushBack(box0Hash);
						break;
					}

					case 1:
					{
						// this grid goes across all cell.
						AddCell(2, hashKey, box0Hash, stepsCode_x, bufferOut);
						break;
					}

					case 2:
					{
						// this grid goes across all cell.
						AddCell(2, hashKey, box0Hash, stepsCode_y, bufferOut);
						break;
					}

					case 4:
					{
						// this grid goes across all cell.
						AddCell(2, hashKey, box0Hash, stepsCode_z, bufferOut);
						break;
					}


					case 3:
					{
						// this grid goes across all cell.
						AddCell(4, hashKey, box0Hash, stepsCode_xy, bufferOut);
						break;
					}

					case 5:
					{
						// this grid goes across all cell.
						AddCell(4, hashKey, box0Hash, stepsCode_xz, bufferOut);
						break;
					}

					case 6:
					{
						// this grid goes across all cell.
						AddCell(4, hashKey, box0Hash, stepsCode_yz, bufferOut);
						break;
					}

					case 7:
					{
						// this grid goes across all cell.
						AddCell(8, hashKey, box0Hash, stepsCode_xyz, bufferOut);
						break;
					}

					default:
						dAssert(0);
				}
			}
		}
	};

	ndScene* const scene = world->GetScene();
	m_upperDigisIsValid[0] = 0;
	m_upperDigisIsValid[1] = 0;
	m_upperDigisIsValid[2] = 0;

	ndCreateGrids::ndContext context;
	context.m_fluid = this;
	scene->SubmitJobs<ndCreateGrids>(&context);

	m_hashGridMap.SetCount(0);
	for (dInt32 i = 0; i < world->GetThreadCount(); i++)
	{
		dInt32 size = m_hashGridMap.GetCount();
		const dArray<ndGridHash>& source = m_hashGridMapThreadBuffers[i];
		m_hashGridMap.SetCount(size + source.GetCount());
		memcpy(&m_hashGridMap[size], &source[0], source.GetCount() * sizeof(ndGridHash));
	}
}

void ndBodySphFluid::SortSingleThreaded(const ndWorld* const world)
{
	const dInt32 count = m_hashGridMap.GetCount();

	dInt32 histograms[6][1 << D_RADIX_DIGIT_SIZE];
	memset(histograms, 0, sizeof(histograms));

	ndGridHash* srcArray = &m_hashGridMap[0];
	for (dInt32 i = 0; i < count; i++)
	{
		const ndGridHash& entry = srcArray[i];
		
		const dInt32 xlow = entry.m_xLow;
		histograms[0][xlow] = histograms[0][xlow] + 1;
	
		const dInt32 xHigh = entry.m_xHigh;
		histograms[1][xHigh] = histograms[1][xHigh] + 1;
	
		const dInt32 ylow = entry.m_yLow;
		histograms[2][ylow] = histograms[2][ylow] + 1;
	
		const dInt32 yHigh = entry.m_yHigh;
		histograms[3][yHigh] = histograms[3][yHigh] + 1;
	
		const dInt32 zlow = entry.m_zLow;
		histograms[4][zlow] = histograms[4][zlow] + 1;
	
		const dInt32 zHigh = entry.m_zHigh;
		histograms[5][zHigh] = histograms[5][zHigh] + 1;
	}
	
	dInt32 acc[6];
	memset(acc, 0, sizeof(acc));
	for (dInt32 i = 0; i < (1 << D_RADIX_DIGIT_SIZE); i++)
	{
		for (dInt32 j = 0; j < 6; j++)
		{
			const dInt32 n = histograms[j][i];
			histograms[j][i] = acc[j];
			acc[j] += n;
		}
	}

	dInt32 shiftbits = 0;
	dUnsigned64 mask = ~dUnsigned64(dInt64(-1 << D_RADIX_DIGIT_SIZE));
	ndGridHash* dstArray = &m_hashGridMapScratchBuffer[0];
	for (dInt32 radix = 0; radix < 3; radix++)
	{
		dInt32* const scan0 = &histograms[radix * 2][0];
		for (dInt32 i = 0; i < count; i++)
		{
			const ndGridHash& entry = srcArray[i];
			const dInt32 key = dUnsigned32((entry.m_gridHash & mask) >> shiftbits);
			const dInt32 index = scan0[key];
			dstArray[index] = entry;
			scan0[key] = index + 1;
		}
		mask <<= D_RADIX_DIGIT_SIZE;
		shiftbits += D_RADIX_DIGIT_SIZE;
		dSwap(dstArray, srcArray);
	
		if (m_upperDigisIsValid[radix])
		{
			dInt32* const scan1 = &histograms[radix * 2 + 1][0];
			for (dInt32 i = 0; i < count; i++)
			{
				const ndGridHash& entry = dstArray[i];
				const dInt32 key = dUnsigned32((entry.m_gridHash & mask) >> shiftbits);
				const dInt32 index = scan1[key];
				srcArray[index] = entry;
				scan1[key] = index + 1;
			}
			dSwap(dstArray, srcArray);
		}
		mask <<= D_RADIX_DIGIT_SIZE;
		shiftbits += D_RADIX_DIGIT_SIZE;
	}

	if (srcArray != &m_hashGridMap[0])
	{
		m_hashGridMap.Swap(m_hashGridMapScratchBuffer);
	}
	dAssert(srcArray == &m_hashGridMap[0]);
}

void ndBodySphFluid::AddCounters(const ndWorld* const world, ndContext& context) const
{
	D_TRACKTIME();

	dInt32 acc = 0;
	for (dInt32 i = 0; i < sizeof(context.m_scan) / sizeof(dInt32); i++)
	{
		dInt32 sum = context.m_scan[i];
		context.m_scan[i] = acc;
		acc += sum;
	}

	dInt32 accTemp[1 << D_RADIX_DIGIT_SIZE];
	memset(accTemp, 0, sizeof(accTemp));

	const dInt32 count = sizeof(context.m_scan) / sizeof(dInt32);
	const dInt32 threadCount = world->GetThreadCount();
	for (dInt32 threadId = 0; threadId < threadCount; threadId++)
	{
		for (dInt32 i = 0; i < count; i++)
		{
			dInt32 a = context.m_histogram[threadId][i];
			context.m_histogram[threadId][i] = accTemp[i] + context.m_scan[i];
			accTemp[i] += a;
		}
	}
}

void ndBodySphFluid::SortParallel(const ndWorld* const world)
{
	D_TRACKTIME();
	class ndBodySphFluidCountDigits : public ndScene::ndBaseJob
	{
		virtual void Execute()
		{
			D_TRACKTIME();

			ndWorld* const world = m_owner->GetWorld();
			ndContext* const context = (ndContext*)m_context;
			ndBodySphFluid* const fluid = context->m_fluid;
			const dInt32 threadId = GetThreadId();
			const dInt32 threadCount = world->GetThreadCount();
			
			const dInt32 count = fluid->m_hashGridMap.GetCount();
			const dInt32 size = count / threadCount;
			const dInt32 start = threadId * size;
			const dInt32 batchSize = (threadId == threadCount - 1) ? count - start : size;

			ndGridHash* const hashArray = &fluid->m_hashGridMap[0];
			dInt32* const histogram = context->m_histogram[threadId];

			memset(histogram, 0, sizeof(context->m_scan));
			dInt32 shiftbits = context->m_pass * D_RADIX_DIGIT_SIZE;
			dUnsigned64 mask = ~dUnsigned64(dInt64(-1 << D_RADIX_DIGIT_SIZE));
			mask = mask << shiftbits;

			for (dInt32 i = 0; i < batchSize; i++)
			{
				const ndGridHash& entry = hashArray[i + start];
				const dInt32 key = dUnsigned32((entry.m_gridHash & mask) >> shiftbits);
				histogram[key] += 1;
			}
		}
	};

	class ndBodySphFluidAddPartialSum : public ndScene::ndBaseJob
	{
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndContext* const context = (ndContext*)m_context;
			const dInt32 threadId = GetThreadId();
			const dInt32 threadCount = world->GetThreadCount();

			const dInt32 count = sizeof (context->m_scan) / sizeof (context->m_scan[0]);

			const dInt32 size = count / threadCount;
			const dInt32 start = threadId * size;
			const dInt32 batchSize = (threadId == threadCount - 1) ? count - start : size;

			dInt32* const scan = context->m_scan;
			for (dInt32 i = 0; i < batchSize; i++)
			{
				dInt32 acc = 0;
				for (dInt32 j = 0; j < threadCount; j++)
				{
					acc += context->m_histogram[j][i + start];
				}
				scan[i + start] = acc;
			}
		}
	};

	class ndBodySphFluidReorderBuckets: public ndScene::ndBaseJob
	{
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndContext* const context = (ndContext*)m_context;
			ndBodySphFluid* const fluid = context->m_fluid;
			const dInt32 threadId = GetThreadId();
			const dInt32 threadCount = world->GetThreadCount();

			const dInt32 count = fluid->m_hashGridMap.GetCount();
			const dInt32 size = count / threadCount;
			const dInt32 start = threadId * size;
			const dInt32 batchSize = (threadId == threadCount - 1) ? count - start : size;

			ndGridHash* const srcArray = &fluid->m_hashGridMap[0];
			ndGridHash* const dstArray = &fluid->m_hashGridMapScratchBuffer[0];

			dInt32 shiftbits = context->m_pass * D_RADIX_DIGIT_SIZE;
			dUnsigned64 mask = ~dUnsigned64(dInt64(-1 << D_RADIX_DIGIT_SIZE));
			mask = mask << shiftbits;
			dInt32* const histogram = context->m_histogram[threadId];
			for (dInt32 i = 0; i < batchSize; i++)
			{
				const ndGridHash& entry = srcArray[i + start];
				const dInt32 key = dUnsigned32((entry.m_gridHash & mask) >> shiftbits);
				const dInt32 index = histogram[key];
				dstArray[index] = entry;
				histogram[key] = index + 1;
			}
		}
	};

	ndContext context;
	context.m_fluid = this;
	ndScene* const scene = world->GetScene();
	for (dInt32 pass = 0; pass < 6; pass++)
	{
		if (!(pass & 1) || m_upperDigisIsValid[pass >> 1])
		{
			D_TRACKTIME();
			context.m_pass = pass;
			scene->SubmitJobs<ndBodySphFluidCountDigits>(&context);
			scene->SubmitJobs<ndBodySphFluidAddPartialSum>(&context);

			AddCounters(world, context);

			scene->SubmitJobs<ndBodySphFluidReorderBuckets>(&context);
			m_hashGridMap.Swap(m_hashGridMapScratchBuffer);
		}
	}	
}

void ndBodySphFluid::SortGrids(const ndWorld* const world)
{
	D_TRACKTIME();
	const dInt32 threadCount = world->GetThreadCount();
	m_hashGridMapScratchBuffer.SetCount(m_hashGridMap.GetCount());
	if (threadCount <= 1)
	{
		dAssert(threadCount == 1);
		SortSingleThreaded(world);
	}
	else
	{
		SortParallel(world);
	}

#ifdef _DEBUG
	for (dInt32 i = 0; i < (m_hashGridMap.GetCount() - 1); i++)
	{
		const ndGridHash& entry0 = m_hashGridMap[i + 0];
		const ndGridHash& entry1 = m_hashGridMap[i + 1];
		dUnsigned64 gridHashA = entry0.m_gridHash;
		dUnsigned64 gridHashB = entry1.m_gridHash;
		dAssert(gridHashA <= gridHashB);
	}
#endif
}

D_NEWTON_API void ndBodySphFluid::GenerateIsoSurface(const ndWorld* const world)
{
return;
#if 0
	D_TRACKTIME();
	dVector boxP0;
	dVector boxP1;
	CaculateAABB(world, boxP0, boxP1);

	dFloat32 gridSize = m_radius * dFloat32(2.0f);
	const dVector invGridSize (dFloat32 (1.0f) / gridSize);

	dVector padd(dFloat32(2.0f) * gridSize);
	boxP0 -= padd & dVector::m_triplexMask;
	boxP1 += padd & dVector::m_triplexMask;

	m_hashGridMap.SetCount(m_posit.GetCount());
	m_hashGridMapScratchBuffer.SetCount(m_posit.GetCount());
	const dVector* const posit = &m_posit[0];
	
	for (dInt32 i = m_posit.GetCount() - 1; i >= 0; i--)
	{
		dAssert(0);
		//dVector r(posit[i] - boxP0);
		//dVector p(r * invGridSize);
		//ndGridHash hashKey(p, i, ndHomeGrid);
		//m_hashGridMap[i] = hashKey;
	}

	ndContext context;
	context.m_fluid = this;

	SortSingleThreaded(world);
	dInt32 uniqueCount = 0;
	for (dInt32 i = 0; i < m_hashGridMap.GetCount();)
	{
		dUnsigned64 key0 = m_hashGridMap[i].m_gridHash;
		m_hashGridMap[uniqueCount].m_gridHash = m_hashGridMap[i].m_gridHash;
		uniqueCount++;
		for (i ++; (i < m_hashGridMap.GetCount()) && (key0 == m_hashGridMap[i].m_gridHash); i++);
	}

	dAssert(0);
	//ndGridHash hashBox0(dVector::m_zero, 0, ndHomeGrid);
	//ndGridHash hashBox1((boxP1 - boxP0) * invGridSize, 0, ndHomeGrid);
	//
	//dUnsigned64 cellCount = (hashBox1.m_z - hashBox0.m_z) * (hashBox1.m_y - hashBox0.m_y) * (hashBox1.m_x - hashBox0.m_x);
	//
	////if (cellCount <= 128)
	//if (cellCount <= 256)
	//{
	//	dAssert((hashBox1.m_z - hashBox0.m_z) > 1);
	//	dAssert((hashBox1.m_y - hashBox0.m_y) > 1);
	//	dAssert((hashBox1.m_x - hashBox0.m_x) > 1);
	//
	//	const int x_ = 6;
	//	const int y_ = 6;
	//	const int z_ = 20;
	//	dFloat32 xxx[z_][y_][x_];
	//	memset(xxx, 0, sizeof(xxx));
	//	for (dInt32 i = 0; i < uniqueCount; i++)
	//	{
	//		dInt32 x = m_hashGridMap[i].m_x;
	//		dInt32 y = m_hashGridMap[i].m_y;
	//		dInt32 z = m_hashGridMap[i].m_z;
	//		xxx[z][y][x] = 1.0f;
	//	}
	//	
	//	dInt32 gridCountX = dInt32(hashBox1.m_x - hashBox0.m_x) + 32;
	//	dInt32 gridCountY = dInt32(hashBox1.m_y - hashBox0.m_y) + 32;
	//	dInt32 gridCountZ = dInt32(hashBox1.m_z - hashBox0.m_z) + 32;
	//	m_isoSurcase.Begin(boxP0, dFloat32(0.5f), gridSize, gridCountX, gridCountY, gridCountZ);
	//	
	//	dIsoSurface::dIsoCell cell;
	//	for (dInt32 z = 0; z < z_-1; z++)
	//	{
	//		cell.m_z = z;
	//		for (dInt32 y = 0; y < y_-1; y++)
	//		{
	//			cell.m_y = y;
	//			for (dInt32 x = 0; x < x_-1; x++)
	//			{
	//				cell.m_x = x;
	//				cell.m_isoValues[0][0][0] = xxx[z + 0][y + 0][x + 0];
	//				cell.m_isoValues[0][0][1] = xxx[z + 0][y + 0][x + 1];
	//				cell.m_isoValues[0][1][0] = xxx[z + 0][y + 1][x + 0];
	//				cell.m_isoValues[0][1][1] = xxx[z + 0][y + 1][x + 1];
	//				cell.m_isoValues[1][0][0] = xxx[z + 1][y + 0][x + 0];
	//				cell.m_isoValues[1][0][1] = xxx[z + 1][y + 0][x + 1];
	//				cell.m_isoValues[1][1][0] = xxx[z + 1][y + 1][x + 0];
	//				cell.m_isoValues[1][1][1] = xxx[z + 1][y + 1][x + 1];
	//				m_isoSurcase.ProcessCell(cell);
	//			}
	//		}
	//	}
	//	m_isoSurcase.End();
	//}
#endif
}