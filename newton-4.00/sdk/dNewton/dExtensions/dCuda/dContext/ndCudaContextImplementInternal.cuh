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

#ifndef __ND_CUDA_CONTEXT_IMPLEMENT_INTERNAL_H__
#define __ND_CUDA_CONTEXT_IMPLEMENT_INTERNAL_H__

//#include "ndCudaStdafx.h"
//#include "ndCudaUtils.h"
//#include "ndCudaDevice.h"
//#include "ndCudaSceneInfo.h"
//#include "ndCudaBodyProxy.h"
//#include "ndCudaHostBuffer.h"
//#include "ndCudaDeviceBuffer.h"

#include <cuda.h>

#if 0
class ndCudaSceneInfo;

__global__ void ndCudaMergeAabbInternal(ndCudaSceneInfo& info);
__global__ void ndCudaCountAabbInternal(ndCudaSceneInfo& info);
__global__ void ndCudaInitBodyArrayInternal(ndCudaSceneInfo& info);
__global__ void ndCudaGenerateGridsInternal(const ndCudaSceneInfo& info);
__global__ void ndCudaCalculateBodyPairsCountInternal(ndCudaSceneInfo& info);
__global__ void ndCudaGetBodyTransformsInternal(const ndCudaSceneInfo& info);


__global__ void ndCudaBeginFrame(ndCudaSceneInfo& info);
__global__ void ndCudaInitTransforms(ndCudaSceneInfo& info);
__global__ void ndCudaInitBodyArray(ndCudaSceneInfo& info);
__global__ void ndCudaGenerateGrids(ndCudaSceneInfo& info);
__global__ void ndCudaGetBodyTransforms(ndCudaSceneInfo& info);
__global__ void ndCudaGenerateSceneGraph(ndCudaSceneInfo& info);
__global__ void ndCudaCalculateBodyPairsCount(ndCudaSceneInfo& info);
__global__ void ndCudaEndFrame(ndCudaSceneInfo& info, int frameCount);

#endif

#endif