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
#include "ndCollisionStdafx.h"
#include "ndBody.h"
#include "ndContact.h"
#include "ndConstraint.h"
#include "ndBodyKinematic.h"

#if 0
#include "dgBody.h"
#include "dgWorld.h"
#include "ndConstraint.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


void* ndConstraint::GetUserData () const
{
	return m_userData;
}

void ndConstraint::SetUserData (void *userData)
{
	m_userData = userData;
}

dFloat32 ndConstraint::GetMassScaleBody0() const
{
	return dFloat32(1.0f);
}

dFloat32 ndConstraint::GetMassScaleBody1() const
{
	return dFloat32(1.0f);
}

void ndConstraint::InitPointParam (dgPointParam& param, dFloat32 stiffness, const dVector& p0Global, const dVector& p1Global) const
{
	dAssert (m_body0);
	dAssert (m_body1);
	param.m_defualtDiagonalRegularizer = stiffness; 

	param.m_posit0 = p0Global;
	param.m_posit1 = p1Global;

	param.m_r0 = (p0Global - m_body0->m_globalCentreOfMass) & dVector::m_triplexMask;
	param.m_r1 = (p1Global - m_body1->m_globalCentreOfMass) & dVector::m_triplexMask;
}

void ndConstraint::InitInfo (dgConstraintInfo* const info) const
{
	info->m_attachBody_0 = GetBody0();
	dAssert (info->m_attachBody_0);
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
#endif

ndConstraint::ndConstraint()
	//:m_body0(body0)
	//,m_body1(body1)
	//,m_userData(nullptr)
	//,m_link0(nullptr)
	//,m_link1(nullptr)
	//,m_updaFeedbackCallback(nullptr)
	//,m_clusterLRU(-1)
	//,m_index(0)
	//,m_impulseLru(0)
	//,m_dynamicsLru(0)
	//,m_maxDOF(6)
	//,m_constId(m_unknownConstraint)
	//,m_solverModel(2)
	//,m_enableCollision(false)
	//,m_isActive(false)
	//,m_isBilateral(false)
	//,m_graphTagged(false)
	//,m_isInSkeleton(false)
	//,m_isInSkeletonLoop(false)
{
}

void ndConstraint::InitPointParam(dgPointParam& param, dFloat32 stiffness, const dVector& p0Global, const dVector& p1Global) const
{
	//ndBody* const body0 = GetBody0();
	//ndBody* const body1 = GetBody1();
	ndBodyKinematic* const body0 = GetKinematicBody0();
	ndBodyKinematic* const body1 = GetKinematicBody1();
	dAssert(body0);
	dAssert(body1);
	param.m_defualtDiagonalRegularizer = stiffness;

	param.m_posit0 = p0Global;
	param.m_posit1 = p1Global;

	param.m_r0 = (p0Global - body0->m_globalCentreOfMass) & dVector::m_triplexMask;
	param.m_r1 = (p1Global - body1->m_globalCentreOfMass) & dVector::m_triplexMask;
}

