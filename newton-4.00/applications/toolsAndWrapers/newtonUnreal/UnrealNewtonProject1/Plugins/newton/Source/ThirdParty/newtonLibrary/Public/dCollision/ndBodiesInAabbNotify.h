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

#ifndef __ND_BODIES_IN_AABB_NOTIFY_H__
#define __ND_BODIES_IN_AABB_NOTIFY_H__

#include "ndCollisionStdafx.h"


D_MSV_NEWTON_ALIGN_32
class ndBodiesInAabbNotify : public ndClassAlloc
{
	public: 
	ndBodiesInAabbNotify()
		:m_bodyArray()
	{
	}

	virtual ~ndBodiesInAabbNotify()
	{
	}

	virtual void Reset()
	{
		m_bodyArray.SetCount(0);
	}

	virtual void OnOverlap(const ndBody* const body)
	{
		m_bodyArray.PushBack(body);
	}

	ndArray<const ndBody*> m_bodyArray;
} D_GCC_NEWTON_ALIGN_32;


#endif
