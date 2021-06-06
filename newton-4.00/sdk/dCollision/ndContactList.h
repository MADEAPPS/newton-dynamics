/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#include "ndCollisionStdafx.h"
#include "ndContact.h"


class ndConstraintArray: public dArray<ndConstraint*>
{
	public: 
	ndConstraintArray()
		:dArray<ndConstraint*>(1024)
	{
	}
};

class ndContactList: public dList<ndContact, dContainersFreeListAlloc<ndContact>>
{
	public:
	ndContactList()
		:dList<ndContact, dContainersFreeListAlloc<ndContact>>()
	{
	}

	~ndContactList()
	{
	}

	D_COLLISION_API void DeleteAllContacts();
	D_COLLISION_API void DeleteContact(ndContact* const contact);
	D_COLLISION_API ndContact* CreateContact(ndBodyKinematic* const body0, ndBodyKinematic* const body1);
};


#endif