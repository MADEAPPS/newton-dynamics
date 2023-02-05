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

__global__ void ndFluidInitTranspose(const ndKernelParams params, const ndAssessor<ndCudaVector> input, ndSphFliudPoint::ndPointAssessor output)
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

__global__ void ndFluidGetPositions(const ndKernelParams params, ndAssessor<ndCudaVector> output, const ndSphFliudPoint::ndPointAssessor input)
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


__global__ void ndCalculateAabb(const ndSphFliudPoint::ndPointAssessor input)
{
//	D_TRACKTIME_NAMED(CalculateAabb);
//	ndBox box;
//	const ndArray<ndVector>& posit = m_posit;
//	const ndStartEnd startEnd(posit.GetCount(), threadIndex, threadCount);
//	for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
//	{
//		box.m_min = box.m_min.GetMin(posit[i]);
//		box.m_max = box.m_max.GetMax(posit[i]);
//	}
//	boxes[threadIndex] = box;

	int threadId = threadIdx.x;
	int blockSride = blockDim.x;
	int blockId = blockIdx.x;
	int index = threadId + blockSride * blockId;
	if (index < input.m_x.m_size)
	{
		ndCudaVector point(input.m_x[threadId], input.m_y[threadId], input.m_z[threadId], 1.0f);
	}

}

ndCudaSphFliud::ndCudaSphFliud(ndCudaContext* const context, ndBodySphFluid* const owner)
	:m_owner(owner)
	,m_context(context)
	,m_points()
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
	const ndSphFliudPoint::ndPointAssessor input(m_workingPoint);
	const ndKernelParams params(m_context->m_device, m_context->m_device->m_workGroupSize, m_points.GetCount());

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
	const ndKernelParams params(m_context->m_device, m_context->m_device->m_workGroupSize, m_points.GetCount());

	//m_aabb.SetCount(groups);
	//m_aabb.SetCount(params.m_kernelCount + 32);
	m_workingPoint.m_x.SetCount(params.m_itemCount);
	m_workingPoint.m_y.SetCount(params.m_itemCount);
	m_workingPoint.m_z.SetCount(params.m_itemCount);

	const ndAssessor<ndCudaVector> input(m_points);
	ndSphFliudPoint::ndPointAssessor output(m_workingPoint);
	ndFluidInitTranspose<<<params.m_kernelCount, params.m_workGroupSize, 0>>>(params, input, output);
}

void ndCudaSphFliud::Update(float timestep)
{
	CaculateAabb();
}

void ndCudaSphFliud::CaculateAabb()
{
	//D_TRACKTIME();
	//class ndBox
	//{
	//	public:
	//	ndBox()
	//		:m_min(ndFloat32(1.0e10f))
	//		, m_max(ndFloat32(-1.0e10f))
	//	{
	//	}
	//	ndVector m_min;
	//	ndVector m_max;
	//};
	//
	//ndBox boxes[D_MAX_THREADS_COUNT];
	//auto CalculateAabb = ndMakeObject::ndFunction([this, &boxes](ndInt32 threadIndex, ndInt32 threadCount)
	//{
	//	D_TRACKTIME_NAMED(CalculateAabb);
	//	ndBox box;
	//	const ndArray<ndVector>& posit = m_posit;
	//	const ndStartEnd startEnd(posit.GetCount(), threadIndex, threadCount);
	//	for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
	//	{
	//		box.m_min = box.m_min.GetMin(posit[i]);
	//		box.m_max = box.m_max.GetMax(posit[i]);
	//	}
	//	boxes[threadIndex] = box;
	//});
	//
	//threadPool->ParallelExecute(CalculateAabb);
	//
	//ndBox box;
	//const ndInt32 threadCount = threadPool->GetThreadCount();
	//for (ndInt32 i = 0; i < threadCount; ++i)
	//{
	//	box.m_min = box.m_min.GetMin(boxes[i].m_min);
	//	box.m_max = box.m_max.GetMax(boxes[i].m_max);
	//}
	//
	//const ndFloat32 gridSize = GetSphGridSize();
	//
	//ndVector grid(gridSize);
	//ndVector invGrid(ndFloat32(1.0f) / gridSize);
	//
	//// add one grid padding to the aabb
	//box.m_min -= grid;
	//box.m_max += (grid + grid);
	//
	//// quantize the aabb to integers of the gird size
	//box.m_min = grid * (box.m_min * invGrid).Floor();
	//box.m_max = grid * (box.m_max * invGrid).Floor();
	//
	//// make sure the w component is zero.
	//m_box0 = box.m_min & ndVector::m_triplexMask;
	//m_box1 = box.m_max & ndVector::m_triplexMask;
	//
	//ndWorkingBuffers& data = *m_workingBuffers;
	//ndInt32 numberOfGrid = ndInt32((box.m_max.m_x - box.m_min.m_x) * invGrid.m_x + ndFloat32(1.0f));
	//data.SetWorldToGridMapping(numberOfGrid, m_box1.m_x, m_box0.m_x);

	ndSphFliudPoint::ndPointAssessor output(m_workingPoint);
}

