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
#include "ndCudaDevice.h"
#include "ndCudaContext.h"
#include "ndCudaSphFluid.h"

#define D_MAX_LOCAL_SIZE 512

__global__ void ndFluidInitTranspose(const ndKernelParams params, const ndAssessor<ndCudaVector> input, ndSphFluidPosit::ndPointAssessor output)
{
	int blockId = blockIdx.x;
	int threadId = threadIdx.x;
	int blockSride = blockDim.x;

	int base = blockSride * params.m_blocksPerKernel * blockId;
	for (int i = 0; i < params.m_blocksPerKernel; ++i)
	{
		int index = base + threadId;
		if (index < input.m_size)
		{
			ndCudaVector point(input[index]);
			output.m_x[index] = point.x;
			output.m_y[index] = point.y;
			output.m_z[index] = point.z;
		}

		base += blockSride;
	}
};

__global__ void ndFluidGetPositions(const ndKernelParams params, ndAssessor<ndCudaVector> output, const ndSphFluidPosit::ndPointAssessor input)
{
	int blockId = blockIdx.x;
	int threadId = threadIdx.x;
	int blockSride = blockDim.x;

	int base = blockSride * params.m_blocksPerKernel * blockId;
	for (int i = 0; i < params.m_blocksPerKernel; ++i)
	{
		int index = base + threadId;
		if (index < output.m_size)
		{
			ndCudaVector point(input.m_x[index], input.m_y[index], input.m_z[index], 1.0f);
			output[index] = point;
		}
		base += blockSride;
	}
};

__global__ void ndCalculateBlockAabb(const ndKernelParams params, const ndSphFluidPosit::ndPointAssessor input, ndAssessor<ndSphFluidAabb> output)
{
	__shared__  float box_x0[D_MAX_LOCAL_SIZE];
	__shared__  float box_y0[D_MAX_LOCAL_SIZE];
	__shared__  float box_z0[D_MAX_LOCAL_SIZE];
	__shared__  float box_x1[D_MAX_LOCAL_SIZE];
	__shared__  float box_y1[D_MAX_LOCAL_SIZE];
	__shared__  float box_z1[D_MAX_LOCAL_SIZE];

	int blockId = blockIdx.x;
	int threadId = threadIdx.x;
	int blockSride = blockDim.x;

	float xMin = 1.0e15f;
	float yMin = 1.0e15f;
	float zMin = 1.0e15f;
	float xMax = -1.0e15f;
	float yMax = -1.0e15f;
	float zMax = -1.0e15f;

	int base = blockSride * params.m_blocksPerKernel * blockId;
	for (int i = 0; i < params.m_blocksPerKernel; ++i)
	{
		int index = base + threadId;

		float x = index < output.m_size ? input.m_x[index] : input.m_x[0];
		float y = index < output.m_size ? input.m_y[index] : input.m_y[0];
		float z = index < output.m_size ? input.m_z[index] : input.m_z[0];

		xMin = x < xMin ? x : xMin;
		yMin = y < yMin ? y : yMin;
		zMin = z < zMin ? z : zMin;

		xMax = x > xMax ? x : xMax;
		yMax = y > yMax ? y : yMax;
		zMax = z > zMax ? z : zMax;

		base += blockSride;
	}

	box_x0[threadId] = xMin;
	box_y0[threadId] = yMin;
	box_z0[threadId] = zMin;

	box_x1[threadId] = xMax;
	box_y1[threadId] = yMax;
	box_z1[threadId] = zMax;

	for (int i = params.m_workGroupSize / 2; i > 0; i = i >> 1)
	{
		if (threadId < i)
		{
			float x0 = box_x0[threadId];
			float y0 = box_y0[threadId];
			float z0 = box_z0[threadId];
			float x1 = box_x0[i + threadId];
			float y1 = box_y0[i + threadId];
			float z1 = box_z0[i + threadId];
			box_x0[threadId] = x0 < x1 ? x0 : x1;
			box_y0[threadId] = y0 < y1 ? y0 : y1;
			box_z0[threadId] = z0 < z1 ? z0 : z1;
	
			x0 = box_x1[threadId];
			y0 = box_y1[threadId];
			z0 = box_z1[threadId];
			x1 = box_x1[i + threadId];
			y1 = box_y1[i + threadId];
			z1 = box_z1[i + threadId];
			box_x1[threadId] = x0 > x1 ? x0 : x1;
			box_y1[threadId] = y0 > y1 ? y0 : y1;
			box_z1[threadId] = z0 > z1 ? z0 : z1;
		}
		__syncthreads();
	}
	
	if (threadId == 0)
	{
		output[blockId].m_min = ndCudaVector(box_x0[0], box_y0[0], box_z0[0], 0.0f);
		output[blockId].m_max = ndCudaVector(box_x1[0], box_y1[0], box_z1[0], 0.0f);
	}
}

__global__ void ndCalculateAabb(const ndKernelParams params, ndAssessor<ndSphFluidAabb> output, float gridSize)
{
	__shared__  float box_x0[D_MAX_LOCAL_SIZE];
	__shared__  float box_y0[D_MAX_LOCAL_SIZE];
	__shared__  float box_z0[D_MAX_LOCAL_SIZE];
	__shared__  float box_x1[D_MAX_LOCAL_SIZE];
	__shared__  float box_y1[D_MAX_LOCAL_SIZE];
	__shared__  float box_z1[D_MAX_LOCAL_SIZE];

	int threadId = threadIdx.x;
	int blockSride = blockDim.x;

	if (threadId < params.m_kernelCount)
	{
		box_x0[threadId] = output[threadId].m_min.x;
		box_y0[threadId] = output[threadId].m_min.y;
		box_z0[threadId] = output[threadId].m_min.z;
		box_x1[threadId] = output[threadId].m_max.x;
		box_y1[threadId] = output[threadId].m_max.y;
		box_z1[threadId] = output[threadId].m_max.z;
	}
	else
	{
		box_x0[threadId] = output[0].m_min.x;
		box_y0[threadId] = output[0].m_min.y;
		box_z0[threadId] = output[0].m_min.z;
		box_x1[threadId] = output[0].m_max.x;
		box_y1[threadId] = output[0].m_max.y;
		box_z1[threadId] = output[0].m_max.z;
	}

	for (int i = blockSride / 2; i > 0; i = i >> 1)
	{
		if (threadId < i)
		{
			float x0 = box_x0[threadId];
			float y0 = box_y0[threadId];
			float z0 = box_z0[threadId];
			float x1 = box_x0[i + threadId];
			float y1 = box_y0[i + threadId];
			float z1 = box_z0[i + threadId];
			box_x0[threadId] = x0 < x1 ? x0 : x1;
			box_y0[threadId] = y0 < y1 ? y0 : y1;
			box_z0[threadId] = z0 < z1 ? z0 : z1;

			x0 = box_x1[threadId];
			y0 = box_y1[threadId];
			z0 = box_z1[threadId];
			x1 = box_x1[i + threadId];
			y1 = box_y1[i + threadId];
			z1 = box_z1[i + threadId];
			box_x1[threadId] = x0 > x1 ? x0 : x1;
			box_y1[threadId] = y0 > y1 ? y0 : y1;
			box_z1[threadId] = z0 > z1 ? z0 : z1;
		}
		__syncthreads();
	}

	if (threadId == 0)
	{
		ndSphFluidAabb box;

		box.m_min = ndCudaVector(box_x0[0], box_y0[0], box_z0[0], 0.0f);
		box.m_max = ndCudaVector(box_x1[0], box_y1[0], box_z1[0], 0.0f);
		//output[0].m_min = ndCudaVector(box_x0[0], box_y0[0], box_z0[0], 0.0f);
		//output[0].m_max = ndCudaVector(box_x1[0], box_y1[0], box_z1[0], 0.0f);

		//const ndFloat32 gridSize = GetSphGridSize();
		//
		//ndVector grid(gridSize);
		//ndVector invGrid(ndFloat32(1.0f) / gridSize);
		ndCudaVector grid(gridSize);
		ndCudaVector invGrid(1.0f / gridSize);
		
		//// add one grid padding to the aabb
		box.m_min = box.m_min - grid;
		box.m_max = box.m_max + grid + grid;
		
		// quantize the aabb to integers of the gird size
		box.m_min = grid * (box.m_min * invGrid).Floor();
		box.m_max = grid * (box.m_max * invGrid).Floor();
		
		box.m_min.w = 0.0f;
		box.m_max.w = 0.0f;

		// make sure the w component is zero.
		//m_box0 = box.m_min & ndVector::m_triplexMask;
		//m_box1 = box.m_max & ndVector::m_triplexMask;
		output[0] = box;

		//ndWorkingBuffers& data = *m_workingBuffers;
		//ndInt32 numberOfGrid = ndInt32((box.m_max.m_x - box.m_min.m_x) * invGrid.m_x + ndFloat32(1.0f));
		//data.SetWorldToGridMapping(numberOfGrid, m_box1.m_x, m_box0.m_x);
	}

}

//ndCudaSphFliud::ndCudaSphFliud(ndCudaContext* const context, ndBodySphFluid* const owner)
ndCudaSphFliud::ndCudaSphFliud(const ndSphFluidInitInfo& info)
	//:m_owner(owner)
	//,m_context(context)
	:m_info(info)
	,m_points()
	,m_aabb()
	,m_workingPoint()
{
}

ndCudaSphFliud::~ndCudaSphFliud()
{
}

void ndCudaSphFliud::MemCpy(const double* const src, int strideInItems, int items)
{
	ndAssert(0);
}

void GetPositions(double* const dst, int strideInItems, int items)
{
	ndAssert(0);
}

void ndCudaSphFliud::MemCpy(const float* const src, int strideInItems, int items)
{
	m_points.SetCount(items);

	if (strideInItems == sizeof(ndCudaVector) / sizeof(float))
	{
		const ndCudaVector* const srcPtr = (ndCudaVector*)src;
		m_points.ReadData(srcPtr, items);
	}
	else
	{
		ndAssert(0);
	}

	InitBuffers();
}

void ndCudaSphFliud::GetPositions(float* const dst, int strideInItems, int items)
{
	ndAssessor<ndCudaVector> output(m_points);
	ndCudaContext* const context = m_info.m_context;
	const ndSphFluidPosit::ndPointAssessor input(m_workingPoint);
	
	const ndKernelParams params(context->m_device, context->m_device->m_workGroupSize, m_points.GetCount());

	if (strideInItems == sizeof(ndCudaVector) / sizeof(float))
	{
		ndFluidGetPositions << <params.m_kernelCount, params.m_workGroupSize, 0 >> > (params, output, input);
		ndCudaVector* const dstPtr = (ndCudaVector*)dst;
		m_points.WriteData(dstPtr, items);
	}
	else
	{
		ndAssert(0);
	}
}

void ndCudaSphFliud::InitBuffers()
{
	ndCudaContext* const context = m_info.m_context;
	const ndKernelParams params(context->m_device, context->m_device->m_workGroupSize, m_points.GetCount());

	m_aabb.SetCount(params.m_kernelCount + 32);
	m_workingPoint.m_x.SetCount(params.m_itemCount);
	m_workingPoint.m_y.SetCount(params.m_itemCount);
	m_workingPoint.m_z.SetCount(params.m_itemCount);

	const ndAssessor<ndCudaVector> input(m_points);
	ndSphFluidPosit::ndPointAssessor output(m_workingPoint);
	ndFluidInitTranspose<<<params.m_kernelCount, params.m_workGroupSize, 0>>>(params, input, output);
}

void ndCudaSphFliud::Update(float timestep)
{
	CaculateAabb();
}

void ndCudaSphFliud::CaculateAabb()
{
	ndAssessor<ndSphFluidAabb> aabb(m_aabb);
	ndCudaContext* const context = m_info.m_context;
	const ndSphFluidPosit::ndPointAssessor input(m_workingPoint);
	const ndKernelParams params(context->m_device, context->m_device->m_workGroupSize, m_points.GetCount());

	int power = 1;
	while (power < params.m_kernelCount)
	{
		power *= 2;
	}
	ndCalculateBlockAabb << <params.m_kernelCount, params.m_workGroupSize, 0 >> > (params, input, aabb);
	ndCalculateAabb << <1, power, 0 >> > (params, aabb, m_info.m_gridSize);
}
													
