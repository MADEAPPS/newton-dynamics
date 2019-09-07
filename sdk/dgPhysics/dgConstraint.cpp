/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#include "dgPhysicsStdafx.h"
#include "dgBody.h"
#include "dgWorld.h"
#include "dgConstraint.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


void* dgConstraint::GetUserData () const
{
	return m_userData;
}

void dgConstraint::SetUserData (void *userData)
{
	m_userData = userData;
}


void dgConstraint::InitPointParam (dgPointParam& param, dgFloat32 stiffness, const dgVector& p0Global, const dgVector& p1Global) const
{
	dgAssert (m_body0);
	dgAssert (m_body1);
	param.m_stiffness = stiffness; 

	param.m_posit0 = p0Global;
	param.m_posit1 = p1Global;

	param.m_r0 = (p0Global - m_body0->m_globalCentreOfMass) & dgVector::m_triplexMask;
	param.m_r1 = (p1Global - m_body1->m_globalCentreOfMass) & dgVector::m_triplexMask;
}

void dgConstraint::InitInfo (dgConstraintInfo* const info) const
{
	info->m_attachBody_0 = GetBody0();
	dgAssert (info->m_attachBody_0);
	dgWorld* const world = info->m_attachBody_0->GetWorld();
	if (info->m_attachBody_0  == (dgBody*)world->GetSentinelBody()) {
		info->m_attachBody_0  = NULL;
	}

	info->m_attachBody_1 = GetBody1();
	if (info->m_attachBody_1  == (dgBody*)world->GetSentinelBody()) {
		info->m_attachBody_1  = NULL;
	}

	info->m_attachMatrix_0 = dgGetIdentityMatrix();
	info->m_attachMatrix_1 = dgGetIdentityMatrix();
	
	info->m_discriptionType[0] = 0;

}


