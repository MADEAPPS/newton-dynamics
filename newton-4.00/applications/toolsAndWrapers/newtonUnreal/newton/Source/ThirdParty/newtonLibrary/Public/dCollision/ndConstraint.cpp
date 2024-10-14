/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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


#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndBody.h"
#include "ndContact.h"
#include "ndConstraint.h"
#include "ndBodyKinematic.h"

void ndForceImpactPair::Clear()
{
	m_force = ndFloat32(ndFloat32(0.0f));
	m_impact = ndFloat32(ndFloat32(0.0f));
	for (ndInt32 i = 0; i < ndInt32(sizeof(m_initialGuess) / sizeof(m_initialGuess[0])); ++i)
	{
		m_initialGuess[i] = ndFloat32(ndFloat32(0.0f));
	}
}

void ndForceImpactPair::Push(ndFloat32 val)
{
	for (ndInt32 i = 1; i < ndInt32(sizeof(m_initialGuess) / sizeof(m_initialGuess[0])); ++i)
	{
		m_initialGuess[i - 1] = m_initialGuess[i];
	}
	m_initialGuess[sizeof(m_initialGuess) / sizeof(m_initialGuess[0]) - 1] = val;
}

ndFloat32 ndForceImpactPair::GetInitialGuess() const
{
	//return 100.0f;
	ndFloat32 smallest = ndFloat32(1.0e15f);
	ndFloat32 value = ndFloat32(ndFloat32(0.0f));
	for (ndInt32 i = 0; i < ndInt32(sizeof(m_initialGuess) / sizeof(m_initialGuess[0])); ++i)
	{
		ndFloat32 mag = ndAbs(m_initialGuess[i]);
		if (mag < smallest)
		{
			smallest = mag;
			value = m_initialGuess[i];
		}
	}
	return value;
}


ndConstraint::ndConstraint()
	:ndContainersFreeListAlloc<ndConstraint>()
	,m_forceBody0(ndVector::m_zero)
	,m_torqueBody0(ndVector::m_zero)
	,m_forceBody1(ndVector::m_zero)
	,m_torqueBody1(ndVector::m_zero)
	,m_body0(nullptr)
	,m_body1(nullptr)
	,m_rowCount(0)
	,m_rowStart(0)
	,m_maxDof(0)
	,m_active(1)
	,m_fence0(0)
	,m_fence1(0)
	,m_resting(0)
	,m_isInSkeletonLoop(0)
{
}

void ndConstraint::InitPointParam(ndPointParam& param, const ndVector& p0Global, const ndVector& p1Global) const
{
	ndBodyKinematic* const body0 = GetBody0();
	ndBodyKinematic* const body1 = GetBody1();
	ndAssert(body0);
	ndAssert(body1);

	param.m_posit0 = p0Global;
	param.m_posit1 = p1Global;

	param.m_r0 = (p0Global - body0->m_globalCentreOfMass) & ndVector::m_triplexMask;
	param.m_r1 = (p1Global - body1->m_globalCentreOfMass) & ndVector::m_triplexMask;
}

