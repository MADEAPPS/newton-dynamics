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


dgInt32 dgBodyMasterListRow::m_contactCountReversal[] = {1, 0, 2};

dgBodyMasterListRow::dgBodyMasterListRow ()
	:dgList<dgBodyMasterListCell>(NULL)
	,m_body (NULL)
	,m_contactCount(0)
{
}

dgBodyMasterListRow::~dgBodyMasterListRow()
{
	dgAssert (GetCount() == 0);
}

dgBodyMasterListRow::dgListNode* dgBodyMasterListRow::AddContactJoint (dgConstraint* const joint, dgBody* const body)
{
	dgThreadHiveScopeLock lock (body->m_world, &m_body->m_criticalSectionLock, false);

	body->m_world->GlobalLock(false);
	dgListNode* const node = Addtop();
	body->m_world->GlobalUnlock();

#ifdef _DEBUG
	for (dgListNode* ptr = GetFirst()->GetNext(); ptr && (ptr->GetInfo().m_joint->GetId() == dgConstraint::m_contactConstraint); ptr = ptr->GetNext()) { 
		dgAssert (ptr->GetInfo().m_bodyNode->m_uniqueID != body->m_uniqueID);
	}
#endif

	node->GetInfo().m_joint = joint;
	node->GetInfo().m_bodyNode = body;

	m_contactCount ++;
	if (m_contactCount > 1) {
		dgInt32 key = body->GetUniqueID();
		dgListNode* ptr0 = node; 
		for (dgListNode* ptr = node->GetNext(); ptr && (ptr->GetInfo().m_joint->GetId() == dgConstraint::m_contactConstraint); ptr = ptr->GetNext()) { 
			dgInt32 key1 = ptr->GetInfo().m_bodyNode->GetUniqueID();
			if (key1 > key) {
				break;
			}
			ptr0 = ptr;
		}
		if (ptr0 != node) {
			InsertAfter (ptr0, node);
		}
	}

#ifdef _DEBUG
	for (dgListNode* ptr = GetFirst(); ptr && ptr->GetNext() && (ptr->GetNext()->GetInfo().m_joint->GetId() == dgConstraint::m_contactConstraint); ptr = ptr->GetNext()) { 
		dgAssert (ptr->GetInfo().m_bodyNode->m_uniqueID < ptr->GetNext()->GetInfo().m_bodyNode->m_uniqueID);
	}
#endif

	SetAcceleratedSearch();
	return node;
}


dgBodyMasterListRow::dgListNode* dgBodyMasterListRow::AddBilateralJoint (dgConstraint* const joint, dgBody* const body)
{
	dgThreadHiveScopeLock lock (body->m_world, &m_body->m_criticalSectionLock, false);

	body->m_world->GlobalLock(false);
	dgListNode* const node = Append();
	body->m_world->GlobalUnlock();
	
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

void dgBodyMasterListRow::RemoveContactJoint (dgListNode* const link)
{
	dgThreadHiveScopeLock lock (m_body->m_world, &m_body->m_criticalSectionLock, false);
	
	m_body->m_world->GlobalLock(false);
	Remove(link);
	m_body->m_world->GlobalUnlock();
	
	m_contactCount --;
	SetAcceleratedSearch();
}

void dgBodyMasterListRow::RemoveBilateralJoint (dgListNode* const link)
{
	dgThreadHiveScopeLock lock (m_body->m_world, &m_body->m_criticalSectionLock, false);
	
	m_body->m_world->GlobalLock(false);
	Remove(link);
	m_body->m_world->GlobalUnlock();
}


dgContact* dgBodyMasterListRow::FindContactJoint (const dgBody* const otherBody) const
{
	dgThreadHiveScopeLock lock (m_body->m_world, &m_body->m_criticalSectionLock, false);
	if (m_body->m_masterNode->GetInfo().m_contactCount) {
		dgInt32 key = otherBody->m_uniqueID;

		dgBodyMasterListRow::dgListNode* link = NULL;
		if (key < m_acceleratedSearch[0]->GetInfo().m_bodyNode->m_uniqueID) {
			link = (key < m_acceleratedSearch[1]->GetInfo().m_bodyNode->m_uniqueID) ? GetFirst() : m_acceleratedSearch[1];
		} else {
			link = (key < m_acceleratedSearch[2]->GetInfo().m_bodyNode->m_uniqueID) ? m_acceleratedSearch[0] : m_acceleratedSearch[2];
		}

		for (; link && (key >= link->GetInfo().m_bodyNode->m_uniqueID); link = link->GetNext()) {
			dgConstraint* const constraint = link->GetInfo().m_joint;
			if (constraint->GetId() != dgConstraint::m_contactConstraint) {
				return NULL;
			}
			if (link->GetInfo().m_bodyNode == otherBody) {
				return (dgContact*)constraint;
			}
		}
	}
	return NULL;
}


dgBilateralConstraint* dgBodyMasterListRow::FindBilateralJoint (const dgBody* const otherBody) const
{
	dgThreadHiveScopeLock lock (m_body->m_world, &m_body->m_criticalSectionLock, false);
	for (dgBodyMasterListRow::dgListNode* link = GetLast(); link; link = link->GetPrev()) {
		dgConstraint* const constraint = link->GetInfo().m_joint;
		if (constraint->GetId() == dgConstraint::m_contactConstraint) {
			return NULL;
		}
		if (link->GetInfo().m_bodyNode == otherBody) {
			dgAssert (constraint->IsBilateral());
			return (dgBilateralConstraint*) constraint;
		}
	}
	return NULL;
}


void dgBodyMasterListRow::SetAcceleratedSearch()
{
	if (GetFirst()) {
		dgInt32 index = 0;
		dgInt32 dx = 2 * m_contactCount;
		dgInt32 dy = 2 * sizeof (m_acceleratedSearch) / sizeof (m_acceleratedSearch[0]);
		dgInt32 acc = dy - m_contactCount;
		dgListNode* ptr = GetFirst();
		for (dgInt32 i = 0; i < m_contactCount; i ++) { 
			dgAssert (ptr->GetInfo().m_joint->GetId() ==  dgConstraint::m_contactConstraint);
			if (acc > 0) {
				dgInt32 j = m_contactCountReversal[index];
				m_acceleratedSearch[j] = ptr;
				index++;
				dgAssert(index <= m_contactCount);
				acc -= dx;
			}
			acc += dy;
			ptr = ptr->GetNext();
		}

		dgAssert ((index <= 2) || ((m_acceleratedSearch[0]->GetInfo().m_bodyNode->m_uniqueID > m_acceleratedSearch[1]->GetInfo().m_bodyNode->m_uniqueID) && 
								   (m_acceleratedSearch[0]->GetInfo().m_bodyNode->m_uniqueID < m_acceleratedSearch[2]->GetInfo().m_bodyNode->m_uniqueID)));

		for (dgInt32 i = index; i < dgInt32 (sizeof (m_acceleratedSearch) / sizeof (m_acceleratedSearch[0])); i++) {
			dgInt32 j = m_contactCountReversal[i];
			m_acceleratedSearch[j] = GetFirst();
		}
	}
}


void dgBodyMasterListRow::SortList()
{
	dgThreadHiveScopeLock lock (m_body->m_world, &m_body->m_criticalSectionLock, false);
	if (GetFirst()) {
		dgAssert (GetFirst()->GetInfo().m_joint->GetId() != dgConstraint::m_contactConstraint);
		dgListNode* nextKeyNode = NULL;
		for (dgListNode* keyNode = GetFirst()->GetNext(); keyNode; keyNode = nextKeyNode) { 
			nextKeyNode = keyNode->GetNext();
			dgAssert (keyNode->GetInfo().m_joint->GetId() != dgConstraint::m_contactConstraint);
			dgInt32 key = keyNode->GetInfo().m_bodyNode->GetUniqueID();
	
			dgListNode* ptr0 = NULL;
			for (dgListNode* ptr = GetFirst(); ptr != keyNode; ptr = ptr->GetNext()) {
				dgInt32 key1 = ptr->GetInfo().m_bodyNode->GetUniqueID();
				if (key1 > key) {
					break;
				}
				ptr0 = ptr;
			}
			dgAssert (ptr0 != keyNode);
			if (!ptr0) {
				RotateToBegin (keyNode);
			} else {
				InsertAfter(ptr0, keyNode);
			}
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
	node->GetInfo().SetBody(body);

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


dgContact* dgBodyMasterList::FindContactJoint (const dgBody* body0, const dgBody* body1) const
{
	dgInt32 count0 = body0->m_masterNode->GetInfo().GetCount();
	dgInt32 count1 = body1->m_masterNode->GetInfo().GetCount();
	if (count0 > count1) {
		dgSwap(body0, body1);
	}
	dgContact* const contact = body0->m_masterNode->GetInfo().FindContactJoint (body1);
	dgAssert (contact || !FindConstraintLink (body0, body1) || FindConstraintLink (body0, body1)->GetInfo().m_joint->IsBilateral());
	return contact;
}


dgBilateralConstraint* dgBodyMasterList::FindBilateralJoint (const dgBody* body0, const dgBody* body1) const
{
	return body0->m_masterNode->GetInfo().FindBilateralJoint (body1);
}

/*
dgBodyMasterListRow::dgListNode* dgBodyMasterList::FindConstraintLinkNext (const dgBodyMasterListRow::dgListNode* const me, const dgBody* const body) const
{
	dgAssert (0);
	dgAssert (me);
	dgAssert (body);
	for (dgBodyMasterListRow::dgListNode* node = me->GetNext(); node; node = node->GetNext()) {
		if (node->GetInfo().m_bodyNode == body) {
			return node;
		}
	}
	return NULL;
}
*/

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
	dgAtomicExchangeAndAdd((dgInt32*) &m_constraintCount, 1);
}


void dgBodyMasterList::RemoveConstraint (dgConstraint* const constraint)
{
	dgAtomicExchangeAndAdd((dgInt32*) &m_constraintCount, -1);
	dgAssert (((dgInt32)m_constraintCount) >= 0);

	dgBody* const body0 = constraint->m_body0;
	dgBody* const body1 = constraint->m_body1;
	dgAssert (body0);
	dgAssert (body1);
	dgAssert (body0 == constraint->m_link1->GetInfo().m_bodyNode);
	dgAssert (body1 == constraint->m_link0->GetInfo().m_bodyNode);

	dgBodyMasterListRow& row0 = body0->m_masterNode->GetInfo();
	dgBodyMasterListRow& row1 = body1->m_masterNode->GetInfo();

	if (body0->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
		dgDynamicBody* const dynBody0 = (dgDynamicBody*)body0;
		dynBody0->m_prevExternalForce = dgVector(dgFloat32(0.0f));
		dynBody0->m_prevExternalTorque = dgVector(dgFloat32(0.0f));
	}

	if (body1->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
		dgDynamicBody* const dynBody1 = (dgDynamicBody*)body1;
		dynBody1->m_prevExternalForce = dgVector(dgFloat32(0.0f));
		dynBody1->m_prevExternalTorque = dgVector(dgFloat32(0.0f));
	}

	body0->m_equilibrium = body0->GetInvMass().m_w ? false : true;
	body1->m_equilibrium = body1->GetInvMass().m_w ? false : true;
	if (constraint->GetId() == dgConstraint::m_contactConstraint) {
		row0.RemoveContactJoint(constraint->m_link0);
		row1.RemoveContactJoint(constraint->m_link1);
	} else {
		row0.RemoveBilateralJoint(constraint->m_link0);
		row1.RemoveBilateralJoint(constraint->m_link1);
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
