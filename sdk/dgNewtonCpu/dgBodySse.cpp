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

#include "dgNewtonPluginStdafx.h"
#include "dgBodySse.h"

dgBodySse::dgBodySse(dgMemoryAllocator* const allocator)
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
	,m_veloc0(allocator)
	,m_internalForces(allocator)
	,m_count(0)
{
}

void dgBodySse::Reserve (dgInt32 count)
{
	m_count = ((count + 3) & -3) >> 2;
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
	m_veloc0.Reserve(m_count);
	m_internalForces.Reserve(m_count);
}

void dgBodySse::GetVeloc(dgInt32 index, dgDynamicBody** const bodyArray)
{
	m_veloc[index] = dgVector3Sse(bodyArray[0]->GetVelocity(), bodyArray[1]->GetVelocity(), bodyArray[2]->GetVelocity(), bodyArray[3]->GetVelocity());
}


void dgBodySse::GetOmega(dgInt32 index, dgDynamicBody** const bodyArray)
{
	m_omega[index] = dgVector3Sse(bodyArray[0]->GetOmega(), bodyArray[1]->GetOmega(), bodyArray[2]->GetOmega(), bodyArray[3]->GetOmega());
}

void dgBodySse::GetMatrix(dgInt32 index, dgDynamicBody** const bodyArray)
{
	m_rotation[index] = dgMatrix3x3Sse(bodyArray[0]->GetMatrix(), 
									   bodyArray[1]->GetMatrix(),
									   bodyArray[2]->GetMatrix(),
									   bodyArray[3]->GetMatrix());
}

void dgBodySse::GetInvMassMatrix(dgInt32 index, dgDynamicBody** const bodyArray)
{
	dgFloatSse v0(bodyArray[0]->GetInvMass());
	dgFloatSse v1(bodyArray[1]->GetInvMass());
	dgFloatSse v2(bodyArray[2]->GetInvMass());
	dgFloatSse v3(bodyArray[3]->GetInvMass());
	dgFloatSse::Transpose4x4(v0, v1, v2, v3);
	m_invMass[index] = v3;
	m_localInvInertia[index] = dgVector3Sse(v0, v1, v2);
}

void dgBodySse::GetDampingCoef(dgInt32 index, dgDynamicBody** const bodyArray, float timestep)
{
	dgFloatSse v0(bodyArray[0]->GetDampCoeffcient(timestep));
	dgFloatSse v1(bodyArray[1]->GetDampCoeffcient(timestep));
	dgFloatSse v2(bodyArray[2]->GetDampCoeffcient(timestep));
	dgFloatSse v3(bodyArray[3]->GetDampCoeffcient(timestep));
	dgFloatSse::Transpose4x4(v0, v1, v2, v3);
	m_linearDamp[index] = v3;
	m_angularDamp[index] = dgVector3Sse(v0, v1, v2);
}



