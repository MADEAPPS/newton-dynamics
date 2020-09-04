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

#ifndef __D_CONSTRAINT_H__
#define __D_CONSTRAINT_H__

#include "ndCollisionStdafx.h"

class ndBody;
class ndContact;

D_MSV_NEWTON_ALIGN_32
class ndConstraint
{
	public:
	virtual ~ndConstraint();

	ndConstraint* GetAsConstraint() { return this; }
	ndContact* GetAsContact() {return nullptr;}

	protected:
	ndConstraint();
	
	//ntBody* m_body0;
	//ntBody* m_body1;
	//void* m_userData;
} D_GCC_NEWTON_ALIGN_32 ;

#if 0

D_INLINE ndConstraint::~ndConstraint()
{
}

D_INLINE ConstraintsForceFeeback ndConstraint::GetUpdateFeedbackFunction ()
{
	return m_updaFeedbackCallback;
}

D_INLINE void ndConstraint::SetUpdateFeedbackFunction (ConstraintsForceFeeback function)
{
	m_updaFeedbackCallback = function;
}

D_INLINE bool ndConstraint::IsBilateral() const
{
	return m_isBilateral ? true : false;
}

D_INLINE bool ndConstraint::IsSkeleton () const
{
	return m_isInSkeleton ? true : false;
}

D_INLINE bool ndConstraint::IsSkeletonLoop () const
{
	return m_isInSkeletonLoop ? true : false;
}

D_INLINE bool ndConstraint::IsCollidable () const
{
	return m_enableCollision ? true : false;
}

D_INLINE void ndConstraint::SetCollidable (bool state)
{
	m_enableCollision = dgUnsigned32 (state);
}

D_INLINE dgUnsigned32 ndConstraint::GetId () const
{
	return m_constId;
}

D_INLINE ntBody* ndConstraint::GetBody0 () const
{
	return m_body0;
}

D_INLINE ntBody* ndConstraint::GetBody1 () const
{
	return m_body1;
}

//D_INLINE void ndConstraint::SetBodies (ntBody* const body0, ntBody* const body1)
//{
//	m_body0 = body0;
//	m_body1 = body1;
//}

D_INLINE dgBodyMasterListRow::dgListNode* ndConstraint::GetLink0()	const
{
	return m_link0;
}
D_INLINE dgBodyMasterListRow::dgListNode* ndConstraint::GetLink1()	const
{
	return m_link1;
}


D_INLINE dgFloat32 ndConstraint::GetStiffness() const
{
	return dgFloat32 (1.0f);
}

D_INLINE void ndConstraint::SetStiffness(dgFloat32 stiffness)
{
}

D_INLINE dgInt32 ndConstraint::GetSolverModel() const
{
	return m_solverModel;
}

D_INLINE void ndConstraint::SetSolverModel(dgInt32 model)
{
	m_solverModel = dgClamp(model, 0, 2);
}

D_INLINE void ndConstraint::ResetMaxDOF()
{
}

D_INLINE void ndConstraint::SetImpulseContactSpeed(dgFloat32 speed)
{
}

D_INLINE dgFloat32 ndConstraint::GetImpulseContactSpeed() const
{
	return dgFloat32 (0.0f);
}

D_INLINE dgInt32 ndConstraint::GetMaxDOF() const
{
	return dgInt32 (m_maxDOF);
}

D_INLINE bool ndConstraint::IsActive() const
{
	return m_isActive ? true : false;
}

D_INLINE void ndConstraint::SetIndex (dgInt32 index)
{
	m_index = index;
}

D_INLINE void ndConstraint::GetInfo(dgConstraintInfo* const info) const
{
	dgAssert(0);
}
#endif


D_INLINE ndConstraint::~ndConstraint()
{
}

#endif 

