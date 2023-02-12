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
#include "ndCudaSort.cuh"
#include "ndCudaDevice.h"
#include "ndCudaContext.h"
#include "ndCudaPrefixScan.cuh"
#include "ndCudaContextImplement.h"

#if 0
__global__ void CudaIntegrateUnconstrainedBodiesInternal(ndCudaSceneInfo& info, float timestep)
{
	int index = threadIdx.x + blockDim.x * blockIdx.x;
	const unsigned maxCount = info.m_bodyArray.m_size - 1;
	ndCudaBodyProxy* bodyArray = info.m_bodyArray.m_array;
	if (index < maxCount)
	{
		ndCudaBodyProxy& body = bodyArray[index];
		const ndCudaMatrix3x3 matrix(body.m_rotation.GetMatrix3x3());
		body.AddDampingAcceleration(matrix);
		body.IntegrateExternalForce(matrix, timestep);
	}
}

__global__ void CudaIntegrateBodiesInternal(ndCudaSceneInfo& info, float timestep)
{
	int index = threadIdx.x + blockDim.x * blockIdx.x;
	const unsigned maxCount = info.m_bodyArray.m_size - 1;
	ndCudaBodyProxy* bodyArray = info.m_bodyArray.m_array;
	if (index < maxCount)
	{
		ndCudaBodyProxy& body = bodyArray[index];
		body.IntegrateVelocity(timestep);
		//printf("%f %f %f %f\n", body.m_rotation.x, body.m_rotation.y, body.m_rotation.z, body.m_rotation.w);
	}
};

__global__ void CudaIntegrateUnconstrainedBodies(ndCudaSceneInfo& info, float timestep)
{
	const unsigned threads = info.m_bodyArray.m_size - 1;
	if (info.m_frameIsValid && threads)
	{
		const unsigned blocks = (threads + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
		CudaIntegrateUnconstrainedBodiesInternal << <blocks, D_THREADS_PER_BLOCK, 0 >> > (info, timestep);
	}
}


__global__ void CudaIntegrateBodies(ndCudaSceneInfo& info, float timestep)
{
	const unsigned threads = info.m_bodyArray.m_size - 1;
	if (info.m_frameIsValid && threads)
	{
		const unsigned blocks = (threads + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
		CudaIntegrateBodiesInternal << <blocks, D_THREADS_PER_BLOCK, 0 >> > (info, timestep);
	}
}

void ndCudaContextImplement::IntegrateBodies(float timestep)
{
	ndCudaSceneInfo* const infoGpu = m_sceneInfoGpu;
	CudaIntegrateBodies << <1, 1, 0, m_solverComputeStream >> > (*infoGpu, timestep);
}

void ndCudaContextImplement::IntegrateUnconstrainedBodies(float timestep)
{
	ndCudaSceneInfo* const infoGpu = m_sceneInfoGpu;
	CudaIntegrateUnconstrainedBodies <<<1, 1, 0, m_solverComputeStream >>> (*infoGpu, timestep);
}
#endif