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

#ifndef _DG_DEFORMABLE_BODY_H_
#define _DG_DEFORMABLE_BODY_H_

#include "dgPhysicsStdafx.h"

#if 0
#include "dgBody.h"


class dgDeformableBody: public dgBody
{
	public:
	dgDeformableBody();
	dgDeformableBody(dgWorld* const world, const dgTree<const dgCollision*, dgInt32>* const collisionNode, dgDeserialize serializeCallback, void* const userData);
	virtual ~dgDeformableBody();

	virtual void Serialize (const dgTree<dgInt32, const dgCollision*>* const collisionNode, dgSerialize serializeCallback, void* const userData);

	protected:
	virtual bool IsDeformable() const;
	virtual bool IsInEquilibrium  () const;

	virtual void ApplyExtenalForces (dgFloat32 timestep, dgInt32 threadIndex);
	virtual void SetVelocity (const dgVector& velocity);

	virtual void SetMatrix(const dgMatrix& matrix);
	virtual void SetMassMatrix (dgFloat32 mass, dgFloat32 Ix, dgFloat32 Iy, dgFloat32 Iz);

	
};


inline bool dgDeformableBody::IsDeformable() const
{
	return true;
}

#endif
#endif 

