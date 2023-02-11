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

#ifndef __ND_CUDA_DEVICE_H__
#define __ND_CUDA_DEVICE_H__

#include "ndCudaStdafx.h"

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

class ndCudaDevice
{
	public:
	ndCudaDevice();
	~ndCudaDevice();

	void* operator new (size_t size);
	void operator delete (void* ptr);

	void SyncDevice() const;
	int GetComputeUnits() const;

	cudaDeviceProp m_prop;
	cudaEvent_t m_startTimer;
	cudaEvent_t m_stopTimer;
	cudaEvent_t m_syncEvent;
	cudaStream_t m_childStream;

	int* m_statusMemory;
	int m_computeUnits;
	int m_workGroupSize;
	int m_timerFrames;
	float m_timeAcc;
	cudaError_t m_lastError;
};

class ndKernelParams
{
	public:
	ndKernelParams() {}
	ndKernelParams(const ndCudaDevice* const device, int workGroupSize, int itemCount);

	__device__ __host__ ndKernelParams(const ndKernelParams& params, int workGroupSize, int itemCount)
		:m_itemCount(itemCount)
		,m_workGroupSize(workGroupSize)
		,m_deviceComputeUnits(params.m_deviceComputeUnits)
	{
		int computeUnitsBashCount = (itemCount + m_workGroupSize - 1) / m_workGroupSize;
		m_blocksPerKernel = (computeUnitsBashCount + m_deviceComputeUnits - 1) / m_deviceComputeUnits;
		m_kernelCount = (itemCount + m_blocksPerKernel * m_workGroupSize - 1) / (m_blocksPerKernel * m_workGroupSize);
	}

	int m_itemCount;
	int m_kernelCount;
	int m_workGroupSize;
	int m_blocksPerKernel;
	int m_deviceComputeUnits;
};

class ndErrorCode
{
	public:
	ndErrorCode();
	ndErrorCode(ndCudaDevice* const context);
	~ndErrorCode();

	int __device__ __host__ Get() const
	{
		return m_baseAdress[m_offset];
	}

	void __device__ __host__ Set(int x)
	{
		m_baseAdress[m_offset] = x;
	}

	int* Pointer() const
	{
		return &m_baseAdress[m_offset];
	}

	private:
	int* m_baseAdress;
	int m_offset;
};


#endif