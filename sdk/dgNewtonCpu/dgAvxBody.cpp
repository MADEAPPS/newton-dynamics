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
	:m_rotation(allocator)
	,m_veloc(allocator)
	,m_omega(allocator)
	,m_weight(allocator)
	,m_invWeigh(allocator)
	,m_linearDamp(allocator)
	,m_angularDamp(allocator)
	,m_count(0)
{
}

void dgAvxBody::Reserve (dgInt32 count)
{
	m_count = ((count + 7) & -7) >> 3;
	m_weight.Reserve(m_count);
	m_invWeigh.Reserve(m_count);
	m_veloc.Reserve(m_count);
	m_omega.Reserve(m_count);
	m_angularDamp.Reserve(m_count);
	m_linearDamp.Reserve(m_count);
	m_rotation.Reserve(m_count);
}

void dgAvxBody::GetVeloc(dgInt32 index, dgDynamicBody** const bodyArray)
{
	m_veloc.InitVector3(index, bodyArray[0]->GetVelocity(), bodyArray[1]->GetVelocity(), bodyArray[2]->GetVelocity(), bodyArray[3]->GetVelocity(),
							   bodyArray[4]->GetVelocity(), bodyArray[5]->GetVelocity(), bodyArray[6]->GetVelocity(), bodyArray[7]->GetVelocity());
}


void dgAvxBody::GetOmega(dgInt32 index, dgDynamicBody** const bodyArray)
{
	m_omega.InitVector3(index, bodyArray[0]->GetOmega(), bodyArray[1]->GetOmega(), bodyArray[2]->GetOmega(), bodyArray[3]->GetOmega(),
						       bodyArray[4]->GetOmega(), bodyArray[5]->GetOmega(), bodyArray[6]->GetOmega(), bodyArray[7]->GetOmega());
}

void dgAvxBody::GetMatrix(dgInt32 index, dgDynamicBody** const bodyArray)
{
	const dgMatrix& matrix0 = bodyArray[0]->GetMatrix();
	const dgMatrix& matrix1 = bodyArray[1]->GetMatrix();
	const dgMatrix& matrix2 = bodyArray[2]->GetMatrix();
	const dgMatrix& matrix3 = bodyArray[3]->GetMatrix();
	const dgMatrix& matrix4 = bodyArray[4]->GetMatrix();
	const dgMatrix& matrix5 = bodyArray[5]->GetMatrix();
	const dgMatrix& matrix6 = bodyArray[6]->GetMatrix();
	const dgMatrix& matrix7 = bodyArray[7]->GetMatrix();

	m_rotation.m_front.InitVector3(index, matrix0[0], matrix1[0], matrix2[0], matrix3[0], matrix4[0], matrix5[0], matrix6[0], matrix7[0]);
	m_rotation.m_up.InitVector3(index, matrix0[1], matrix1[1], matrix2[1], matrix3[1], matrix4[1], matrix5[1], matrix6[1], matrix7[1]);
	m_rotation.m_right.InitVector3(index, matrix0[2], matrix1[2], matrix2[2], matrix3[2], matrix4[2], matrix5[2], matrix6[2], matrix7[2]);
}


void dgAvxBody::GetDampingCoef(dgInt32 index, dgDynamicBody** const bodyArray, float timestep)
{
	dgAvxFloat damp0(bodyArray[0]->GetDampCoeffcient(timestep), bodyArray[4]->GetDampCoeffcient(timestep));
	dgAvxFloat damp1(bodyArray[1]->GetDampCoeffcient(timestep), bodyArray[5]->GetDampCoeffcient(timestep));
	dgAvxFloat damp2(bodyArray[2]->GetDampCoeffcient(timestep), bodyArray[6]->GetDampCoeffcient(timestep));
	dgAvxFloat damp3(bodyArray[3]->GetDampCoeffcient(timestep), bodyArray[7]->GetDampCoeffcient(timestep));
	dgAvxFloat::Transpose4x8(damp0, damp1, damp2, damp3);
	m_angularDamp.m_x[index] = damp0;
	m_angularDamp.m_y[index] = damp1;
	m_angularDamp.m_z[index] = damp2;
	m_linearDamp.m_val[index] = damp3;
}