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

#ifndef __CU_SOLVER_TYPES_H__
#define __CU_SOLVER_TYPES_H__

#include <cuda.h>
#include <vector_types.h>
#include <cuda_runtime.h>
#include <ndNewtonStdafx.h>
#include "cuVector.h"
#include "cuDeviceBuffer.h"

class cuBodyProxy;
class cuBodyAabbCell;

class cuSpatialVector
{
	public:
	cuVector m_linear;
	cuVector m_angular;
};

class cuBoundingBox
{
	public:
	cuVector m_min;
	cuVector m_max;
};

template <class T>
class cuBuffer
{
	public:
	cuBuffer()
		:m_array(nullptr)
		,m_size(0)
		,m_capacity(0)
	{
	}

	cuBuffer(const cuDeviceBuffer<T>& src)
		:m_array((T*)&src[0])
		,m_size(src.GetCount())
		,m_capacity(src.GetCapacity())
	{
	}

	T* m_array;
	int m_size;
	int m_capacity;
};

class cuSceneInfo
{
	public:
	cuSceneInfo()
		:m_worldBox()
		,m_scan()
		,m_histogram()
		,m_bodyArray()
		,m_bodyAabbCell()
		,m_bodyAabbArray()
		,m_bodyAabbCellScrath()
		,m_transformBuffer0()
		,m_transformBuffer1()
		,m_frameIsValid(0)
		,m_frameCount(0)
	{
		m_hasUpperByteHash.x = 0;
		m_hasUpperByteHash.y = 0;
		m_hasUpperByteHash.z = 0;
		m_hasUpperByteHash.w = 0;
	}

	cuBoundingBox m_worldBox;
	cuBuffer<int> m_scan;
	cuBuffer<int> m_histogram;
	cuBuffer<cuBodyProxy> m_bodyArray;
	cuBuffer<cuBodyAabbCell> m_bodyAabbCell;
	cuBuffer<cuBoundingBox> m_bodyAabbArray;
	cuBuffer<cuBodyAabbCell> m_bodyAabbCellScrath;
	cuBuffer<cuSpatialVector> m_transformBuffer0;
	cuBuffer<cuSpatialVector> m_transformBuffer1;
	
	int4 m_hasUpperByteHash;
	int m_frameIsValid;
	int m_frameCount;
};

#endif