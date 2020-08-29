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

#ifndef __D_CONTACT_CACHE_H__
#define __D_CONTACT_CACHE_H__

#include "ntStdafx.h"

class ntContact;

class ntContactFreeList: public dArray<ntContact*>
{
	public:
	ntContactFreeList()
		:dArray<ntContact*>()
		,m_index(0)
	{
	}

	~ntContactFreeList()
	{
	}

	void Clear()
	{
		m_index.store(GetCount());
	}

	D_NEWTON_API ntContact* GetContact(ntBody* const body0, ntBody* const body1);
	D_NEWTON_API void RemoveContact(ntContact* const contact);

	std::atomic<dInt32> m_index;
};

class ntContactList: public dArray<ntContact*>
{
	public:
	ntContactList()
		:dArray<ntContact*>()
		,m_extraContacts()
		,m_index(0)
		,m_lock()
		,m_activeCount(0)
	{
	}

	~ntContactList()
	{
	}

	D_NEWTON_API void Reset();
	D_NEWTON_API void Update();
	D_NEWTON_API void PushBack(ntContact* const contact);
	D_NEWTON_API void DeleteAllContacts();
	
	dArray<ntContact*> m_extraContacts;
	std::atomic<dInt32> m_index;
	dSpinLock m_lock;
	std::atomic<dInt32> m_activeCount;
};


#endif