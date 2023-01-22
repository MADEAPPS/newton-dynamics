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

	void Begin();
	//void End();
	//double GetGPUTime() const;
	//
	//void ResizeBuffers(int size);
	//void LoadBodyData(const ndCudaBodyProxy* const src, int size);
	//
	//void InitBodyArray();
	//void UpdateTransform();
	//void ValidateContextBuffers();
	//
	//ndCudaSpatialVector* GetTransformBuffer();
	//
	//void IntegrateBodies(float timestep);
	//void IntegrateUnconstrainedBodies(float timestep);

	sycl::device m_device;
	sycl::queue m_queue;
	int m_computeUnits;
	char m_deviceName[64];

	sycl::buffer<unsigned> m_sortPrefixBuffer;
};

#endif