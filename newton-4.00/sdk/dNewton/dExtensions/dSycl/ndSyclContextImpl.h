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

#ifndef __ND_SYCL_CONTEXT_IMPL_H__
#define __ND_SYCL_CONTEXT_IMPL_H__

#include <ndSyclStdafx.h>

class ndSyclContextImpl
{
	public: 
	D_SYCL_OPERATOR_NEW_AND_DELETE

	ndSyclContextImpl(sycl::device device);
	~ndSyclContextImpl();
	
	const char* GetStringId() const;

	//D_SYCL_API void Begin();
	//D_SYCL_API void End();
	//D_SYCL_API double GetGPUTime() const;
	//
	//D_SYCL_API void ResizeBuffers(int size);
	//D_SYCL_API void LoadBodyData(const ndCudaBodyProxy* const src, int size);
	//
	//D_SYCL_API void InitBodyArray();
	//D_SYCL_API void UpdateTransform();
	//D_SYCL_API void ValidateContextBuffers();
	//
	//D_SYCL_API ndCudaSpatialVector* GetTransformBuffer();
	//
	//D_SYCL_API void IntegrateBodies(float timestep);
	//D_SYCL_API void IntegrateUnconstrainedBodies(float timestep);

	sycl::device m_device;
	sycl::queue m_queue;
	char m_deviceName[64];

};

#endif