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

#ifndef __ND_CUDA_CONTEXT_H__
#define __ND_CUDA_CONTEXT_H__

#include <cuda.h>
#include <cuda_runtime.h>
#include <ndNewtonStdafx.h>
#include <device_launch_parameters.h>

#include "cuHostBuffer.h"
#include "ndBodyBuffer.h"
#include "cuSolverTypes.h"
#include "cuDeviceBuffer.h"

class ndCudaDevice
{
	public:
	ndCudaDevice();
	~ndCudaDevice();

	ndInt32 m_valid;
	struct cudaDeviceProp m_prop;
};

class ndGpuInfo
{
	public:
	cuBoundingBox m_worldBox;
	int m_cellBodyCount;
};

class ndCudaContext : public ndClassAlloc, public ndCudaDevice
{
	public: 
	ndCudaContext();
	~ndCudaContext();
	static ndCudaContext* CreateContext();

	void SwapBuffers();
	
	ndGpuInfo* m_sceneInfo;
	cuDeviceBuffer<int> m_scan;
	ndArray<cuBodyProxy> m_bodyBufferCpu;
	cuDeviceBuffer<cuAabbGridHash> m_gridHash;
	cuDeviceBuffer<cuBodyProxy> m_bodyBufferGpu;
	cuDeviceBuffer<cuBoundingBox> m_boundingBoxGpu;
	cuHostBuffer<cuSpatialVector> m_transformBufferCpu0;
	cuHostBuffer<cuSpatialVector> m_transformBufferCpu1;
	cuDeviceBuffer<cuSpatialVector> m_transformBufferGpu;

	cudaStream_t m_stream0;
	//cudaStream_t m_stream1;
};

#endif