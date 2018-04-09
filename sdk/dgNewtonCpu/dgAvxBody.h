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

class dgAvxBody
{
	public:
	dgAvxBody (dgMemoryAllocator* const allocator);
	void Reserve (dgInt32 count);

	void GetMatrix(dgInt32 index, dgDynamicBody** const bodyArray);
	void GetVeloc(dgInt32 index, dgDynamicBody** const bodyArray);
	void GetOmega(dgInt32 index, dgDynamicBody** const bodyArray);
	void GetDampingCoef(dgInt32 index, dgDynamicBody** const bodyArray, float timestep);


	DG_INLINE void AddDampingAcceleration(dgInt32 index, const dgAvxFloat& timestep)
	{
		m_veloc.Set(index, m_veloc.Get(index).Scale(timestep));

		//dgVector omegaDamp(damp & dgVector::m_triplexMask);
		//dgVector omega(m_matrix.UnrotateVector(m_omega) * omegaDamp);
		//m_omega = m_matrix.RotateVector(omega);
		dgAvxVector3_local damp(m_angularDamp.Get(index));
		dgAvxVector3_local omega(m_omega.Get(index));
		dgAvxMatrix3x3_local matrix(m_rotation.Get(index));
		m_omega.Set(index, matrix.RotateVector(matrix.UnrotateVector(omega) * damp));
	}


	
	dgAvxMatrix3x3 m_rotation;
	dgAvxVector3 m_veloc;
	dgAvxVector3 m_omega;
	dgAvxScalar m_weight;
	dgAvxScalar m_invWeigh;
	dgAvxScalar m_linearDamp;
	dgAvxVector3 m_angularDamp;

	dgInt32 m_count;
};




#endif

