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
#include "ndCudaDeviceBuffer.h"

class ndCudaContext;
class ndBodySphFluid;

class ndCudaSphFliud
{
	public:
	class ndPoint
	{
		public:
		ndCudaDeviceBuffer<float> m_x;
		ndCudaDeviceBuffer<float> m_y;
		ndCudaDeviceBuffer<float> m_z;
	};

	D_CUDA_OPERATOR_NEW_AND_DELETE;

	D_CUDA_API ndCudaSphFliud(ndCudaContext* const context, ndBodySphFluid* const owner);
	D_CUDA_API ~ndCudaSphFliud();

	D_CUDA_API void MemCpy(const float* const src, int strideInItems, int items);
	D_CUDA_API void MemCpy(const double* const src, int strideInItems, int items);

	D_CUDA_API void GetPositions(float* const dst, int strideInItems, int items);
	D_CUDA_API void GetPositions(double* const dst, int strideInItems, int items);

	D_CUDA_API void Update(float timestep);

	void InitBuffers();

	ndBodySphFluid* m_owner;
	ndCudaContext* m_context;
	ndCudaDeviceBuffer<ndCudaVector> m_points;

	ndPoint m_workingPoint;
	
};


#endif