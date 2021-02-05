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

#ifndef __D_CONTACT_OPTIONS_H__
#define __D_CONTACT_OPTIONS_H__

#include "ndCollisionStdafx.h"

enum ndContactOptions
{
	m_collisionEnable = 1 << 0,
	m_friction0Enable = 1 << 1,
	m_friction1Enable = 1 << 2,
	m_isSoftContact = 1 << 3,
	m_override0Accel = 1 << 4,
	m_override1Accel = 1 << 5,
	m_override0Friction = 1 << 6,
	m_override1Friction = 1 << 7,
	m_overrideNormalAccel = 1 << 8,
	m_resetSkeletonSelfCollision = 1 << 9,
	m_resetSkeletonIntraCollision = 1 << 10,
};

#endif 

