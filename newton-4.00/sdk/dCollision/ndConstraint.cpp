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


#include "dCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndBody.h"
#include "ndContact.h"
#include "ndConstraint.h"
#include "ndBodyKinematic.h"

ndConstraint::ndConstraint()
	:m_preconditioner0(dFloat32 (1.0f))
	,m_preconditioner1(dFloat32(1.0f))
	,m_rowCount(0)
	,m_rowStart(0)
	,m_jointFeebackForce(false)
	,m_isInSkeletonLoop(false)
{
}

void ndConstraint::InitPointParam(dgPointParam& param, dFloat32 stiffness, const dVector& p0Global, const dVector& p1Global) const
{
	ndBodyKinematic* const body0 = GetBody0();
	ndBodyKinematic* const body1 = GetBody1();
	dAssert(body0);
	dAssert(body1);
	param.m_defaultDiagonalRegularizer = stiffness;

	param.m_posit0 = p0Global;
	param.m_posit1 = p1Global;

	param.m_r0 = (p0Global - body0->m_globalCentreOfMass) & dVector::m_triplexMask;
	param.m_r1 = (p1Global - body1->m_globalCentreOfMass) & dVector::m_triplexMask;
}

