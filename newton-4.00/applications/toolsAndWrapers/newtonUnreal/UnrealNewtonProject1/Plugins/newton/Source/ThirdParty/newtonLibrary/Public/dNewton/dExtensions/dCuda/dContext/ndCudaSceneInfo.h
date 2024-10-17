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

#ifndef __ND_CUDA_SCENE_INFO_H__
#define __ND_CUDA_SCENE_INFO_H__

#include <cuda.h>
#include <vector_types.h>
#include <cuda_runtime.h>
#include "ndCudaTypes.h"


#if 0
class ndCudaSceneInfo
{
	public:
	ndCudaSceneInfo()
		:m_worldBox()
		,m_histogram()
		,m_bodyArray()
		,m_sceneGraph()
		,m_bodyAabbCell()
		,m_bodyAabbCellScratch()
		,m_transformBuffer0()
		,m_transformBuffer1()
		,m_frameCount(0)
		,m_frameIsValid(0)
		,m_startFrameTime(0)
		,m_frameTimeInNanosecunds(0)
	{
	}

	ndCudaBoundingBox m_worldBox;
	ndCudaBuffer<unsigned> m_histogram;
	ndCudaBuffer<ndCudaBodyProxy> m_bodyArray;
	ndCudaBuffer<ndCudaSceneNode> m_sceneGraph;
	ndCudaBuffer<ndCudaBodyAabbCell> m_bodyAabbCell;
	ndCudaBuffer<ndCudaBodyAabbCell> m_bodyAabbCellScratch;
	ndCudaBuffer<ndCudaSpatialVector> m_transformBuffer0;
	ndCudaBuffer<ndCudaSpatialVector> m_transformBuffer1;
	
	unsigned m_frameCount;
	unsigned m_frameIsValid;
	long long m_startFrameTime;
	long long m_frameTimeInNanosecunds;
};

inline void __device__ cuInvalidateFrame(ndCudaSceneInfo& info, const char* functionName, unsigned lineNumber)
{
	m_device->SyncDevice();
	info.m_frameIsValid = 0;
	printf("skipping frame %d  function %s  line %d\n", info.m_frameCount, functionName, lineNumber);
}
#endif

#endif