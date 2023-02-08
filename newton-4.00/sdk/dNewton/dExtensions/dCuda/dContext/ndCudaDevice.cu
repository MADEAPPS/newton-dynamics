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

#include "ndCudaStdafx.h"
#include "ndCudaUtils.h"
#include "ndCudaDevice.h"

#define D_STATUS_ERROR_SIZE	512

ndErrorCode::ndErrorCode()
	:m_baseAdress(nullptr)
	,m_offset(0)
{
}

ndErrorCode::ndErrorCode(ndCudaDevice* const context)
	:m_baseAdress(context->m_statusMemory)
	,m_offset(0)
{
	for (int i = 0; i < D_STATUS_ERROR_SIZE; i++)
	{
		if (m_baseAdress[i] == -1)
		{
			m_offset = i;
			m_baseAdress[m_offset] = 0;
			return;
		}
	}
	ndAssert(0);
}

ndErrorCode::~ndErrorCode()
{
	m_baseAdress[m_offset] = -1;
}


ndCudaDevice::ndCudaDevice()
{
	m_lastError = cudaGetDeviceProperties(&m_prop, 0);
	ndAssert(m_lastError == cudaSuccess);
	
	m_lastError = cudaSetDevice(0);
	ndAssert(m_lastError == cudaSuccess);
	
	cuTrace(("gpu: %s\n", m_prop.name));
	cuTrace(("compute capability: %d.%d\n", m_prop.major, m_prop.minor));
	
	cuTrace(("warp size: %d\n", m_prop.warpSize));
	cuTrace(("multiprocessors: %d\n", m_prop.multiProcessorCount));
	cuTrace(("threads per blocks %d\n", m_prop.maxThreadsPerBlock));
	cuTrace(("blocks per multiprocessors %d\n", m_prop.maxBlocksPerMultiProcessor));
	cuTrace(("memory bus with: %d bits\n", m_prop.memoryBusWidth));
	cuTrace(("memory: (mbytes) %d\n", m_prop.totalGlobalMem / (1024 * 1024)));

	m_workGroupSize = std::min(m_prop.maxThreadsPerBlock, 512);
	m_computeUnits = std::min(4 * m_prop.multiProcessorCount, 512);

	m_lastError = cudaMallocManaged(&m_statusMemory, D_STATUS_ERROR_SIZE * sizeof(int));
	ndAssert(m_lastError == cudaSuccess);
	memset(m_statusMemory, -1, D_STATUS_ERROR_SIZE * sizeof(int));
}

ndCudaDevice::~ndCudaDevice()
{
	m_lastError = cudaFree(m_statusMemory);
	ndAssert(m_lastError == cudaSuccess);

	m_lastError = cudaDeviceReset();
	ndAssert(m_lastError == cudaSuccess);
}

void* ndCudaDevice::operator new (size_t size)
{											
	return ndCudaMalloc(size);
}											

void ndCudaDevice::operator delete (void* ptr)
{											
	ndCudaFree(ptr);
}											

int ndCudaDevice::GetComputeUnits() const
{
	return m_computeUnits;
}

ndKernelParams::ndKernelParams(const ndCudaDevice* const device, int workGroupSize, int itemCount)
	:m_itemCount(itemCount)
	,m_workGroupSize(workGroupSize)
{
	ndAssert(workGroupSize);
	ndAssert(!(workGroupSize & (workGroupSize - 1)));
	int deviceComputeUnits = device->GetComputeUnits();
	int computeUnitsBashCount = (itemCount + m_workGroupSize - 1) / m_workGroupSize;

	m_blocksPerKernel = (computeUnitsBashCount + deviceComputeUnits - 1) / deviceComputeUnits;
	m_kernelCount = (itemCount + m_blocksPerKernel * m_workGroupSize - 1) / (m_blocksPerKernel * m_workGroupSize);
	ndAssert(m_kernelCount <= deviceComputeUnits);
}
