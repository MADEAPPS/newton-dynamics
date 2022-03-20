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

#ifndef __ND_BODY_BUFFER_H__
#define __ND_BODY_BUFFER_H__

#include <cuda.h>
#include <cuda_runtime.h>
#include <ndNewtonStdafx.h>

#include "ndDeviceBuffer.h"

class ndBodyProxi
{
	public:
	float4 m_rotation;
};

class ndBodyBuffer: public ndDeviceBuffer<ndBodyProxi>
{
	public:
	ndBodyBuffer();
	~ndBodyBuffer();

	//ndDeviceBuffer<ndCudaJacobian> m_transform;
	//ndDeviceBuffer<ndCudaJacobian> m_veloc;
	//ndDeviceBuffer<ndCudaJacobian> m_accel;
	ndArray<ndBodyProxi> m_dataView;
};


inline ndBodyBuffer::ndBodyBuffer()
	:ndDeviceBuffer<ndBodyProxi>()
{
}

inline ndBodyBuffer::~ndBodyBuffer()
{
}
#endif