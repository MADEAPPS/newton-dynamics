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

