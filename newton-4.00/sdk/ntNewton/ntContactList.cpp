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

#include "ntStdafx.h"
#include "ntBody.h"
#include "ntContact.h"
#include "ntContactList.h"


void ntContactList::DeleteAllContacts()
{
	ntContact** const contacts = &(*this)[0];
	for (dInt32 i = GetCount() - 1; i >= 0; i--)
	{
		ntContact* const contact = contacts[i];
		contact->DetachFromBodies();
		delete contact;
	}

	m_index.store(0);
	Clear();
	Resize(256);
}

void ntContactList::Reset()
{
	m_activeCount = GetCount();
	m_index.store(GetCount());
	m_extraContacts.Clear();
	SetCount(GetCapacity());
}

void ntContactList::Update()
{
	D_TRACKTIME();
	const dInt32 extraCount = m_extraContacts.GetCount();
	if (extraCount)
	{
		dInt32 index = GetCapacity();
		SetCount(index + extraCount);
		memcpy(&(*this)[index], &m_extraContacts[0], extraCount * sizeof(ntContact*));
		m_extraContacts.Clear();
		m_extraContacts.Resize(256);
	}
	else
	{
		dAssert(m_index.load() < GetCapacity());
		SetCount(m_index.load());
	}

	ntContact** const contacts = &(*this)[0];
	for (dInt32 i = GetCount() - 1; i >= m_activeCount ; i--)
	{
		contacts[i]->AttachToBodies();
	}
}

void ntContactList::PushBack(ntContact* const contact)
{
	dInt32 index = m_index.fetch_add(1);
	if (index < GetCapacity())
	{
		(*this)[index] = contact;
	}
	else
	{
		dScopeSpinLock lock(m_lock);
		ntContact* contactPtr = contact;
		m_extraContacts.PushBack(contactPtr);
	}
}

ntContact* ntContactFreeList::GetContact(ntBody* const body0, ntBody* const body1)
{
	dInt32 index = m_index.fetch_add(-1) - 1;
	if (index >= 0)
	{
		dAssert(0);
		return nullptr;
	}
	else
	{
		return new ntContact(body0, body1);
	}
};

void ntContactFreeList::RemoveContact(ntContact* const contact)
{
	dAssert(0);
}


