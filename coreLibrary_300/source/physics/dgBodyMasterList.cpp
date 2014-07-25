/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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
#include "dgDynamicBody.h"
#include "dgBodyMasterList.h"


dgBodyMasterListRow::dgBodyMasterListRow ()
	:dgList<dgBodyMasterListCell>(NULL)
{
	m_body = NULL;
}

dgBodyMasterListRow::~dgBodyMasterListRow()
{
	dgAssert (GetCount() == 0);
}


dgBodyMasterListRow::dgListNode* dgBodyMasterListRow::AddContactJoint (dgConstraint* const joint, dgBody* const body)
{
	dgListNode* const node = Addtop();
	node->GetInfo().m_joint = joint;
	node->GetInfo().m_bodyNode = body;
	return node;
}


dgBodyMasterListRow::dgListNode* dgBodyMasterListRow::AddBilateralJoint (dgConstraint* const joint, dgBody* const body)
{
	dgListNode* const node = Append();
	node->GetInfo().m_joint = joint;
	node->GetInfo().m_bodyNode = body;
	return node;
}

void dgBodyMasterListRow::RemoveAllJoints ()
{
	dgWorld* const world = m_body->GetWorld();

	for (dgListNode* node = GetFirst(); node; ) {
		dgConstraint* const constraint = node->GetInfo().m_joint;
		node = node->GetNext();
		world->DestroyConstraint (constraint);
	}
}

void dgBodyMasterListRow::SortList()
{
	for (dgListNode* node = GetFirst(); node; ) { 

		dgListNode* const entry = node;
		node = node->GetNext();
		dgListNode* prev = entry->GetPrev();
		for (; prev; prev = prev->GetPrev()) {
			if (prev < entry) {
				break;
			}
		}

		if (!prev) {
			RotateToBegin (entry);
		} else {
			InsertAfter (prev, entry);
		}
	}
}


dgBodyMasterList::dgBodyMasterList (dgMemoryAllocator* const allocator)
	:dgList<dgBodyMasterListRow>(allocator)
	,m_disableBodies(allocator)
	,m_deformableCount(0)
	,m_constraintCount (0)
{
}


dgBodyMasterList::~dgBodyMasterList(void)
{
}


void dgBodyMasterList::AddBody (dgBody* const body)
{
	dgListNode* const node = Append();
	body->m_masterNode = node;
	node->GetInfo().SetAllocator (body->GetWorld()->GetAllocator());
	node->GetInfo().m_body = body;

	if (GetFirst() != node) {
		InsertAfter (GetFirst(), node);
	}

	if (body->IsRTTIType(dgBody::m_deformableBodyRTTI)) {
		m_deformableCount ++;
	}
}

void dgBodyMasterList::RemoveBody (dgBody* const body)
{
	if (body->IsRTTIType(dgBody::m_deformableBodyRTTI)) {
		m_deformableCount --;
	}
	dgAssert (m_deformableCount >= 0);

	dgListNode* const node = body->m_masterNode;
	dgAssert (node);
	
	node->GetInfo().RemoveAllJoints();
	dgAssert (node->GetInfo().GetCount() == 0);

	Remove (node);
	body->m_masterNode = NULL;
}


dgBodyMasterListRow::dgListNode* dgBodyMasterList::FindConstraintLink (const dgBody* const body0, const dgBody* const body1) const
{
	dgAssert (body0);
	dgAssert (body1);
	dgAssert (body0->m_masterNode);

	for (dgBodyMasterListRow::dgListNode* node = body0->m_masterNode->GetInfo().GetFirst(); node; node = node->GetNext()) {
		if (node->GetInfo().m_bodyNode == body1) {
			return node;
		}
	}

	return NULL;
}

dgBodyMasterListRow::dgListNode* dgBodyMasterList::FindConstraintLinkNext (const dgBodyMasterListRow::dgListNode* const me, const dgBody* const body) const
{
	dgAssert (me);
	dgAssert (body);
	for (dgBodyMasterListRow::dgListNode* node = me->GetNext(); node; node = node->GetNext()) {
		if (node->GetInfo().m_bodyNode == body) {
			return node;
		}
	}

	return NULL;
}


void dgBodyMasterList::AttachConstraint(dgConstraint* const constraint,	dgBody* const body0, dgBody* const srcbody1)
{
	dgAssert (body0);
	dgBody* body1 = srcbody1;
	if (!body1) {
		body1 = body0->GetWorld()->GetSentinelBody();
	}
	dgAssert (body1);

	constraint->m_body0 = body0;
	constraint->m_body1 = body1;

	if (constraint->GetId() != dgConstraint::m_contactConstraint) {
		body0->m_equilibrium = body0->GetInvMass().m_w ? false : true;
		body1->m_equilibrium = body1->GetInvMass().m_w ? false : true;
		constraint->m_link0 = body0->m_masterNode->GetInfo().AddBilateralJoint (constraint, body1);
		constraint->m_link1 = body1->m_masterNode->GetInfo().AddBilateralJoint (constraint, body0);
	} else {
		constraint->m_link0 = body0->m_masterNode->GetInfo().AddContactJoint (constraint, body1);
		constraint->m_link1 = body1->m_masterNode->GetInfo().AddContactJoint (constraint, body0);
	}
	m_constraintCount = m_constraintCount + 1;
}


void dgBodyMasterList::RemoveConstraint (dgConstraint* const constraint)
{
	m_constraintCount = m_constraintCount - 1;
	dgAssert (((dgInt32)m_constraintCount) >= 0);

	dgBody* const body0 = constraint->m_body0;
	dgBody* const body1 = constraint->m_body1;
	dgAssert (body0);
	dgAssert (body1);
	dgAssert (body0 == constraint->m_link1->GetInfo().m_bodyNode);
	dgAssert (body1 == constraint->m_link0->GetInfo().m_bodyNode);

	body0->m_masterNode->GetInfo().Remove(constraint->m_link0);
	body1->m_masterNode->GetInfo().Remove(constraint->m_link1);

    if (constraint->GetId() != dgConstraint::m_contactConstraint) {
	    if (body0->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
		    dgDynamicBody* const dynBody0 = (dgDynamicBody*) body0;
		    dynBody0->m_prevExternalForce = dgVector (dgFloat32 (0.0f));
		    dynBody0->m_prevExternalTorque = dgVector (dgFloat32 (0.0f));
	    }

	    if (body1->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
		    dgDynamicBody* const dynBody1 = (dgDynamicBody*) body1;
		    dynBody1->m_prevExternalForce = dgVector (dgFloat32 (0.0f));
		    dynBody1->m_prevExternalTorque = dgVector (dgFloat32 (0.0f));
	    }

	    if (constraint->m_maxDOF) {
		    body0->m_equilibrium = body0->GetInvMass().m_w ? false : true;
		    body1->m_equilibrium = body1->GetInvMass().m_w ? false : true;
	    }
    }
}


DG_INLINE dgUnsigned32 dgBodyMasterList::MakeSortMask(const dgBody* const body) const
{
//	return body->m_uniqueID | ((body->GetInvMass().m_w > 0.0f) << 30);
	dgUnsigned32 val0 = body->IsRTTIType(dgBody::m_dynamicBodyRTTI) ? (body->GetInvMass().m_w > 0.0f) << 30 : 0;
	dgUnsigned32 val1 = body->IsRTTIType(dgBody::m_kinematicBodyRTTI) ? 1<<29 : 0;
	dgUnsigned32 val2 = body->IsRTTIType(dgBody::m_deformableBodyRTTI) ? 1<<28 : 0;
	return body->m_uniqueID | val0 | val1 | val2;
}


void dgBodyMasterList::SortMasterList()
{
	GetFirst()->GetInfo().SortList();

	for (dgListNode* node = GetFirst()->GetNext(); node; ) { 
		node->GetInfo().SortList();
		dgBody* const body1 = node->GetInfo().GetBody();
		
		dgAssert (GetFirst() != node);

		body1->InvalidateCache ();

		dgInt32 key1 = MakeSortMask (body1);
		dgListNode* const entry = node;
		node = node->GetNext();
		dgListNode* prev = entry->GetPrev();
		for (; prev != GetFirst(); prev = prev->GetPrev()) {
			dgBody* const body0 = prev->GetInfo().GetBody();

			dgInt32 key0 = MakeSortMask (body0);
			if (key0 < key1) {
				break;
			}
		}

		if (!prev) {
			dgAssert (entry == GetFirst());
			RotateToBegin (entry);
		} else {
			InsertAfter (prev, entry);
		}
	}

}
