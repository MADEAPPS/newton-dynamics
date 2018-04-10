/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
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

#ifndef _DG_BODY_SOA_H_
#define _DG_BODY_SOA_H_
#include "dgNewtonCpuStdafx.h"
#include "dgAvxMath.h"


template<class T>
class dgGlobalArray: public dgArray<T>
{
	public:
	dgGlobalArray(dgMemoryAllocator* const allocator, dgInt32 aligmentInBytes = DG_MEMORY_GRANULARITY)
		:dgArray<T>(allocator, aligmentInBytes)
		, m_ptr(NULL)
	{
	}

	void Reserve(dgInt32 count)
	{
		ResizeIfNecessary(count);
		dgArray<T>& me = *this;
		m_ptr = &me[0];
	}

	DG_INLINE T& operator[] (dgInt32 i)
	{
		return m_ptr[i];
	}

	DG_INLINE const T& operator[] (dgInt32 i) const
	{
		return m_ptr[i];
	}

	T* m_ptr;
};


class dgAvxBody
{
	public:
	dgAvxBody (dgMemoryAllocator* const allocator);
	void Reserve (dgInt32 count);
	
	
	void GetVeloc(dgInt32 index, dgDynamicBody** const bodyArray);
	void GetOmega(dgInt32 index, dgDynamicBody** const bodyArray);
	void GetMatrix(dgInt32 index, dgDynamicBody** const bodyArray);
	void GetInvMassMatrix(dgInt32 index, dgDynamicBody** const bodyArray);
	void GetDampingCoef(dgInt32 index, dgDynamicBody** const bodyArray, float timestep);

	void ApplyDampingAndCalculateInvInertia(dgInt32 index);

	dgGlobalArray<dgAvxVector3> m_veloc;
	dgGlobalArray<dgAvxVector3> m_omega;
	dgGlobalArray<dgAvxVector3> m_localInvInertia;
	dgGlobalArray<dgAvxFloat> m_invMass;
	dgGlobalArray<dgAvxFloat> m_linearDamp;
	dgGlobalArray<dgAvxVector3> m_angularDamp;
	dgGlobalArray<dgAvxFloat> m_weigh;
	dgGlobalArray<dgAvxFloat> m_invWeigh;
	dgGlobalArray<dgAvxMatrix3x3> m_rotation;
	dgGlobalArray<dgAvxMatrix3x3> m_invInertia;
	dgInt32 m_count;
};
#endif

