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

#ifndef __D_BODY_PLAYER_CAPSULE_H__
#define __D_BODY_PLAYER_CAPSULE_H__

#include "ndCollisionStdafx.h"
#include "ndBodyKinematic.h"

D_MSV_NEWTON_ALIGN_32
class ndBodyPlayerCapsule : public ndBodyKinematic
{
	public:
	D_COLLISION_API ndBodyPlayerCapsule();
	D_COLLISION_API virtual ~ndBodyPlayerCapsule();

	ndBodyPlayerCapsule* ndBodyPlayerCapsule::GetAsBodyPlayerCapsule();

	virtual void SetCollisionShape(const ndShapeInstance& shapeInstance);

} D_GCC_NEWTON_ALIGN_32;

inline ndBodyPlayerCapsule* ndBodyPlayerCapsule::GetAsBodyPlayerCapsule()
{ 
	return this; 
}

inline void ndBodyPlayerCapsule::SetCollisionShape(const ndShapeInstance& shapeInstance)
{
	// ignore the changing collision shape;
}

#endif