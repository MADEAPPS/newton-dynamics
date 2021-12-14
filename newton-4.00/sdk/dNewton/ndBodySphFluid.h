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

#ifndef __ND_BODY_SPH_FLUID_H__
#define __ND_BODY_SPH_FLUID_H__

#include "ndNewtonStdafx.h"
#include "ndBodyParticleSet.h"

#define D_RADIX_DIGIT_SIZE	10
//#define D_GRID_SIZE_FACTOR	ndFloat32(4.0f)

D_MSV_NEWTON_ALIGN_32
class ndBodySphFluid: public ndBodyParticleSet
{
	public:
	D_NEWTON_API ndBodySphFluid();
	D_NEWTON_API ndBodySphFluid(const ndLoadSaveBase::dLoadDescriptor& desc);
	D_NEWTON_API virtual ~ndBodySphFluid ();

	virtual ndBodySphFluid* GetAsBodySphFluid();
	D_NEWTON_API virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	D_NEWTON_API virtual void AddParticle(const ndFloat32 mass, const ndVector& position, const ndVector& velocity);

	D_NEWTON_API virtual void GenerateIsoSurface(const ndWorld* const world);

	const ndIsoSurface& GetIsoSurface() const;

	protected:
	D_NEWTON_API virtual void Update(const ndWorld* const world, ndFloat32 timestep);
	virtual bool RayCast(ndRayCastNotify& callback, const ndFastRay& ray, const ndFloat32 maxT) const;

	private:
	enum ndGridType
	{
		ndAdjacentGrid = 0,
		ndHomeGrid = 1,
	};

	class ndGridHash
	{
		public:
		ndGridHash()
		{
		}

		ndGridHash(ndUnsigned64 gridHash)
			:m_gridHash(gridHash)
		{
		}

		ndGridHash(ndInt32 x, ndInt32 y, ndInt32 z)
		{
			m_gridHash = 0;
			m_x = x;
			m_y = y;
			m_z = z;
		}

		ndGridHash(const ndVector& grid, ndInt32 particelIndex)
		{
			dAssert(grid.m_x >= ndFloat32 (0.0f));
			dAssert(grid.m_y >= ndFloat32 (0.0f));
			dAssert(grid.m_z >= ndFloat32 (0.0f));
			dAssert(grid.m_x < ndFloat32 (1<<(D_RADIX_DIGIT_SIZE * 2)));
			dAssert(grid.m_y < ndFloat32 (1<<(D_RADIX_DIGIT_SIZE * 2)));
			dAssert(grid.m_z < ndFloat32 (1<<(D_RADIX_DIGIT_SIZE * 2)));

			ndVector hash(grid.GetInt());

			m_gridHash = 0;
			m_x = hash.m_ix;
			m_y = hash.m_iy;
			m_z = hash.m_iz;

			m_cellType = ndAdjacentGrid;
			m_particleIndex = particelIndex;
		}

		union
		{
			struct
			{
				ndUnsigned64 m_x : D_RADIX_DIGIT_SIZE * 2;
				ndUnsigned64 m_y : D_RADIX_DIGIT_SIZE * 2;
				ndUnsigned64 m_z : D_RADIX_DIGIT_SIZE * 2;
			};
			struct
			{
				ndUnsigned64 m_xLow  : D_RADIX_DIGIT_SIZE;
				ndUnsigned64 m_xHigh : D_RADIX_DIGIT_SIZE;
				ndUnsigned64 m_yLow  : D_RADIX_DIGIT_SIZE;
				ndUnsigned64 m_yHigh : D_RADIX_DIGIT_SIZE;
				ndUnsigned64 m_zLow  : D_RADIX_DIGIT_SIZE;
				ndUnsigned64 m_zHigh : D_RADIX_DIGIT_SIZE;
			};

			ndUnsigned64 m_gridHash;
		};

		ndInt32 m_particleIndex;
		ndGridType m_cellType;
	};

	class ndParticlePair
	{
		public:
		ndInt32 m_m0;
		ndInt32 m_m1;
	};

	class ndContext
	{
		public:
		ndBodySphFluid* m_fluid;
		ndInt32 m_pass;
		ndInt32 m_scan[1 << D_RADIX_DIGIT_SIZE];
		ndInt32 m_histogram[D_MAX_THREADS_COUNT][1 << D_RADIX_DIGIT_SIZE];
	};

	void SortGrids(const ndWorld* const world);
	void BuildPairs(const ndWorld* const world);
	void CreateGrids(const ndWorld* const world);
	void CalculateAccelerations(const ndWorld* const world);
	void AddCounters(const ndWorld* const world, ndContext& context) const;
	void CaculateAABB(const ndWorld* const world, ndVector& boxP0, ndVector& boxP1) const;

	void SortByCenterType();
	void SortSingleThreaded();
	void SortParallel(const ndWorld* const world);
	void CalculateScans(const ndWorld* const world);
	void CalculateScansDebug(ndArray<ndInt32>& gridScans);
	ndFloat32 CalculateGridSize() const;

	ndVector m_box0;
	ndVector m_box1;
	ndArray<ndGridHash> m_hashGridMap;
	ndArray<ndParticlePair> m_particlesPairs;
	ndArray<ndGridHash> m_hashGridMapScratchBuffer;
	ndArray<ndInt32> m_gridScans[D_MAX_THREADS_COUNT];
	ndInt32 m_upperDigisIsValid[3];
	ndIsoSurface m_isoSurcase;
} D_GCC_NEWTON_ALIGN_32 ;

inline bool ndBodySphFluid::RayCast(ndRayCastNotify&, const ndFastRay&, const ndFloat32) const
{
	return false;
}

inline ndBodySphFluid* ndBodySphFluid::GetAsBodySphFluid()
{ 
	return this; 
}

inline const ndIsoSurface& ndBodySphFluid::GetIsoSurface() const
{
	return m_isoSurcase;
}

inline ndFloat32 ndBodySphFluid::CalculateGridSize() const
{
	//return m_radius * ndFloat32(2.0f) * ndFloat32(1.125f);
	return m_radius * (ndFloat32(2.0f) * ndFloat32(2.0f));
}

#endif 


