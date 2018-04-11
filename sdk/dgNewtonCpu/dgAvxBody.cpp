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

#include "dgNewtonCpuStdafx.h"
#include "dgAvxBody.h"

dgAvxBody::dgAvxBody(dgMemoryAllocator* const allocator)
	:m_veloc(allocator)
	,m_omega(allocator)
	,m_localInvInertia(allocator)
	,m_invMass(allocator)
	,m_weigh(allocator)
	,m_invWeigh(allocator)
	,m_linearDamp(allocator)
	,m_angularDamp(allocator)
	,m_rotation(allocator)
	,m_invInertia(allocator)
	,m_count(0)
{
}

void dgAvxBody::Reserve (dgInt32 count)
{
	m_count = ((count + 7) & -7) >> 3;
	m_veloc.Reserve(m_count);
	m_omega.Reserve(m_count);
	m_localInvInertia.Reserve(m_count);
	m_invMass.Reserve(m_count);
	m_weigh.Reserve(m_count);
	m_invWeigh.Reserve(m_count);
	m_angularDamp.Reserve(m_count);
	m_linearDamp.Reserve(m_count);
	m_rotation.Reserve(m_count);
	m_invInertia.Reserve(m_count);
}

void dgAvxBody::GetVeloc(dgInt32 index, dgDynamicBody** const bodyArray)
{
	m_veloc[index] = dgAvxVector3(bodyArray[0]->GetVelocity(), bodyArray[1]->GetVelocity(), bodyArray[2]->GetVelocity(), bodyArray[3]->GetVelocity(),
								  bodyArray[4]->GetVelocity(), bodyArray[5]->GetVelocity(), bodyArray[6]->GetVelocity(), bodyArray[7]->GetVelocity());
}


void dgAvxBody::GetOmega(dgInt32 index, dgDynamicBody** const bodyArray)
{
	m_omega[index] = dgAvxVector3(bodyArray[0]->GetOmega(), bodyArray[1]->GetOmega(), bodyArray[2]->GetOmega(), bodyArray[3]->GetOmega(),
								  bodyArray[4]->GetOmega(), bodyArray[5]->GetOmega(), bodyArray[6]->GetOmega(), bodyArray[7]->GetOmega());
}

void dgAvxBody::GetMatrix(dgInt32 index, dgDynamicBody** const bodyArray)
{
	m_rotation[index] = dgAvxMatrix3x3(bodyArray[0]->GetMatrix(), 
									   bodyArray[1]->GetMatrix(),
									   bodyArray[2]->GetMatrix(),
									   bodyArray[3]->GetMatrix(),
									   bodyArray[4]->GetMatrix(),
									   bodyArray[5]->GetMatrix(),
									   bodyArray[6]->GetMatrix(),
									   bodyArray[7]->GetMatrix());
}

void dgAvxBody::GetInvMassMatrix(dgInt32 index, dgDynamicBody** const bodyArray)
{
	dgAvxFloat v0(bodyArray[0]->GetInvMass(), bodyArray[4]->GetInvMass());
	dgAvxFloat v1(bodyArray[1]->GetInvMass(), bodyArray[5]->GetInvMass());
	dgAvxFloat v2(bodyArray[2]->GetInvMass(), bodyArray[6]->GetInvMass());
	dgAvxFloat v3(bodyArray[3]->GetInvMass(), bodyArray[7]->GetInvMass());
	dgAvxFloat::Transpose4x8(v0, v1, v2, v3);
	m_invMass[index] = v3;
	m_localInvInertia[index] = dgAvxVector3(v0, v1, v2);
}

void dgAvxBody::GetDampingCoef(dgInt32 index, dgDynamicBody** const bodyArray, float timestep)
{
	dgAvxFloat v0(bodyArray[0]->GetDampCoeffcient(timestep), bodyArray[4]->GetDampCoeffcient(timestep));
	dgAvxFloat v1(bodyArray[1]->GetDampCoeffcient(timestep), bodyArray[5]->GetDampCoeffcient(timestep));
	dgAvxFloat v2(bodyArray[2]->GetDampCoeffcient(timestep), bodyArray[6]->GetDampCoeffcient(timestep));
	dgAvxFloat v3(bodyArray[3]->GetDampCoeffcient(timestep), bodyArray[7]->GetDampCoeffcient(timestep));
	dgAvxFloat::Transpose4x8(v0, v1, v2, v3);
	m_linearDamp[index] = v3;
	m_angularDamp[index] = dgAvxVector3(v0, v1, v2);
}



