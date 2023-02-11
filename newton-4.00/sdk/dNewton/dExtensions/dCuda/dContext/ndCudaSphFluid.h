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

#ifndef __ND_CUDA_SPH_FLUID_H__
#define __ND_CUDA_SPH_FLUID_H__

#include "ndCudaUtils.h"
#include "ndCudaStdafx.h"
#include "ndCudaVector.h"
#include "ndCudaDevice.h"
#include "ndCudaDeviceBuffer.h"

class ndCudaContext;
class ndBodySphFluid;

#define D_SPH_CUDA_HASH_BITS 8


class ndSphFluidInitInfo
{
	public:
	ndCudaContext* m_context;
	ndBodySphFluid* m_owner;
	float m_gridSize;
};

class ndSphFluidAabb
{
	public:
	ndCudaVector m_min;
	ndCudaVector m_max;
};

class ndCudaSphFluid
{
	public:
	D_CUDA_OPERATOR_NEW_AND_DELETE;

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

		__device__ __host__ ndGridHash(uint64_t gridHash)
			:m_gridHash(gridHash)
		{
		}

		ndGridHash(int x, int z)
		{
			m_gridHash = 0;
			m_x = uint64_t (x);
			m_z = uint64_t (z);
		}

		__device__ __host__ ndGridHash(const ndCudaVector& grid, int particleIndex)
		{
			//ndCudaVector hash(grid.GetInt());
			unsigned hash_x = unsigned(floor(grid.x));
			unsigned hash_z = unsigned(floor(grid.z));

			m_gridHash = 0;
			m_x = uint64_t (hash_x);
			m_z = uint64_t (hash_z);
			m_cellType = ndAdjacentGrid;
			m_particleIndex = uint64_t (particleIndex);
		}

		union
		{
			struct
			{
				uint64_t  m_x : D_SPH_CUDA_HASH_BITS * 2;
				uint64_t  m_z : D_SPH_CUDA_HASH_BITS * 2;
				uint64_t  m_particleIndex : 23;
				uint64_t  m_cellType : 1;
			};
			struct
			{
				uint64_t  m_xLow : D_SPH_CUDA_HASH_BITS;
				uint64_t  m_xHigh : D_SPH_CUDA_HASH_BITS;
				uint64_t  m_zLow : D_SPH_CUDA_HASH_BITS;
				uint64_t  m_zHigh : D_SPH_CUDA_HASH_BITS;
			};
			uint64_t  m_gridHash : D_SPH_CUDA_HASH_BITS * 2 * 2;
		};
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
		unsigned char m_isPadd[4][4];
		unsigned char m_counter[4];
	};

	class Image: public ndSphFluidInitInfo
	{
		public:

		enum ndError
		{
			m_noError,
			m_gridsOverFlow,
		};

		Image(const ndSphFluidInitInfo& info);
		~Image();
		void Init(ndCudaSphFluid& fluid);

		ndKernelParams m_param;
		ndAssessor<int> m_gridScans;
		ndAssessor<ndCudaVector> m_points;
		ndAssessor<ndGridHash> m_hashGridMap;
		ndAssessor<ndGridHash> m_hashGridMapTemp;
		ndAssessor<ndSphFluidAabb> m_pointsAabb;

		ndAssessor<ndGridHash> m_sortHashGridMap0;
		ndAssessor<ndGridHash> m_sortHashGridMap1;

		ndSphFluidAabb m_aabb;
		ndGridNeighborInfo m_neighborgInfo;

		int* m_errorCode;
		int m_gridSizeX;
		int m_gridSizeY;
		int m_gridSizeZ;
		int m_activeHashGridMapSize;
	
		ndError m_error;
		cudaStream_t m_childStream;
	};

	D_CUDA_API ndCudaSphFluid(const ndSphFluidInitInfo& info);
	D_CUDA_API ~ndCudaSphFluid();

	ndBodySphFluid* GetOwner();

	D_CUDA_API void MemCpy(const float* const src, int strideInItems, int items);
	D_CUDA_API void MemCpy(const double* const src, int strideInItems, int items);

	D_CUDA_API void GetPositions(float* const dst, int strideInItems, int items);
	D_CUDA_API void GetPositions(double* const dst, int strideInItems, int items);

	D_CUDA_API void Update(float timestep);

	void InitBuffers();
	void HandleErrors();
	void CaculateAabb();
	void CreateGrids();
	void SortGrids();

	bool TraceHashes();

	Image m_imageCpu;
	Image* m_imageGpu;

	ndErrorCode m_errorCode;
	ndCudaDeviceBuffer<ndCudaVector> m_points;
	ndCudaDeviceBuffer<ndGridHash> m_hashGridMap;
	ndCudaDeviceBuffer<ndGridHash> m_hashGridMapTemp;
	ndCudaDeviceBuffer<ndSphFluidAabb> m_pointsAabb;
};

inline ndBodySphFluid* ndCudaSphFluid::GetOwner()
{
	return m_imageCpu.m_owner;
}
#endif