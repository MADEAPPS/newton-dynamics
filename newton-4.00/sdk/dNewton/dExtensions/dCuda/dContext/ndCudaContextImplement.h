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

#ifndef __ND_CUDA_CONTEXT_IMPLEMENT_H__
#define __ND_CUDA_CONTEXT_IMPLEMENT_H__

#include "ndCudaStdafx.h"
#include "ndCudaUtils.h"
#include "ndCudaDevice.h"
#include "ndCudaSceneInfo.h"
#include "ndCudaBodyProxy.h"
#include "ndCudaHostBuffer.h"
#include "ndCudaDeviceBuffer.h"
#include "ndCudaBodyAabbCell.h"


#define D_THREADS_PER_BLOCK_BITS	8
#define D_THREADS_PER_BLOCK			(1<<D_THREADS_PER_BLOCK_BITS)

class ndCudaDevice;

class ndCudaContextImplement
{
	public: 
	ndCudaContextImplement();
	~ndCudaContextImplement();

	void Begin();
	void ResizeBuffers(int size);
	void LoadBodyData(const ndCudaBodyProxy* const src, int size);

	void SwapBuffers();
	void InitBodyArray();
	void ValidateContextBuffers();
	ndCudaSpatialVector* GetTransformBuffer0();
	ndCudaSpatialVector* GetTransformBuffer1();
	
	ndCudaSceneInfo* m_sceneInfoGpu;
	ndCudaSceneInfo* m_sceneInfoCpu;
	ndCudaDeviceBuffer<unsigned> m_histogram;
	ndCudaDeviceBuffer<ndCudaBodyProxy> m_bodyBufferGpu;
	ndCudaDeviceBuffer<ndCudaBodyAabbCell> m_bodyAabbCell;
	ndCudaDeviceBuffer<ndCudaBodyAabbCell> m_bodyAabbCellScrath;
	ndCudaDeviceBuffer<ndCudaBoundingBox> m_boundingBoxGpu;
	ndCudaHostBuffer<ndCudaSpatialVector> m_transformBufferCpu0;
	ndCudaHostBuffer<ndCudaSpatialVector> m_transformBufferCpu1;
	ndCudaDeviceBuffer<ndCudaSpatialVector> m_transformBufferGpu0;
	ndCudaDeviceBuffer<ndCudaSpatialVector> m_transformBufferGpu1;
	
	cudaStream_t m_solverMemCpyStream;
	cudaStream_t m_solverComputeStream;
	unsigned m_frameCounter;
};

#endif