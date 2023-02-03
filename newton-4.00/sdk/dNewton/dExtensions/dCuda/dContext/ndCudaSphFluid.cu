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
	if (strideInItems == sizeof(ndCudaVector) / sizeof(float))
	{
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
	int workGroupSize = m_context->m_device->m_workGroupSize;
	int groups = (m_points.GetCount() + workGroupSize - 1) / workGroupSize;
	int size = workGroupSize * groups;
	m_workingPoint.m_x.SetCount(size);
	m_workingPoint.m_y.SetCount(size);
	m_workingPoint.m_z.SetCount(size);
}

void ndCudaSphFliud::Update(float timestep)
{

}
