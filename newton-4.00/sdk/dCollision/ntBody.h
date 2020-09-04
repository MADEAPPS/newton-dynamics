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

#ifndef _D_BODY_H_
#define _D_BODY_H_

#include "ndCollisionStdafx.h"
#include "ntShapeInstance.h"

class ntContact;
class ntBodyNotify;
class ndBodyDynamic;
class ntBodyKinematic;
class ntRayCastNotify;
class ntBilateralJoint;

D_MSV_NEWTON_ALIGN_32
class ntBody: public dClassAlloc
{
	public:
	ND_COLLISION_API ntBody();
	ND_COLLISION_API virtual ~ntBody();

	virtual ntBody* GetAsBody() { return this;}
	virtual ndBodyDynamic* GetAsBodyDynamic() { return nullptr; }
	virtual ntBodyKinematic* GetAsBodyKinematic() { return nullptr; }

	dUnsigned32 GetId() const;
	virtual dFloat32 GetInvMass() const { return dFloat32(0.0f); }
	virtual dFloat32 RayCast(ntRayCastNotify& callback, const dFastRayTest& ray, const dFloat32 maxT) const = 0;

	ND_COLLISION_API void SetCentreOfMass(const dVector& com);
	ND_COLLISION_API void SetNotifyCallback(ntBodyNotify* const notify);
	ND_COLLISION_API ntBodyNotify* GetNotifyCallback(ntBodyNotify* const notify) const;
	ND_COLLISION_API dVector GetOmega() const;
	ND_COLLISION_API void SetOmega(const dVector& veloc);
	ND_COLLISION_API dVector GetVelocity() const;
	ND_COLLISION_API void SetVelocity(const dVector& veloc);
	ND_COLLISION_API dMatrix GetMatrix() const;
	ND_COLLISION_API void SetMatrix(const dMatrix& matrix);

	protected:
	virtual void AttachContact(ntContact* const contact) {}
	virtual void DetachContact(ntContact* const contact) {}
	virtual ntContact* FindContact(const ntBody* const otherBody) const { return nullptr; }

	dMatrix m_matrix;
	dMatrix m_invWorldInertiaMatrix;
	dVector m_veloc;
	dVector m_omega;
	dVector m_localCentreOfMass;
	dVector m_globalCentreOfMass;
	dVector m_minAABB;
	dVector m_maxAABB;
	dQuaternion m_rotation;
	ntBodyNotify* m_notifyCallback;

	union
	{
		dUnsigned32 m_flags;
		struct
		{
			dUnsigned32 m_equilibrium : 1;
			//dUnsigned32 m_freeze : 1;
			//dUnsigned32 m_resting : 1;
			//dUnsigned32 m_sleeping : 1;
			//dUnsigned32 m_autoSleep : 1;
			//dUnsigned32 m_inCallback : 1;
			//dUnsigned32 m_jointSet : 1;
			//dUnsigned32 m_collidable : 1;
			//dUnsigned32 m_equilibrium : 1;
			//dUnsigned32 m_spawnnedFromCallback : 1;
			//dUnsigned32 m_continueCollisionMode : 1;
			dUnsigned32 m_collideWithLinkedBodies : 1;
			//dUnsigned32 m_transformIsDirty : 1;
			//dUnsigned32 m_gyroTorqueOn : 1;
			//dUnsigned32 m_isdead : 1;
		};
	};

	dUnsigned32 m_uniqueID;
	static dUnsigned32 m_uniqueIDCount;
	friend class ntBroadPhase;
} D_GCC_NEWTON_ALIGN_32;


inline dUnsigned32 ntBody::GetId() const
{
	return m_uniqueID;
}

//inline dList<ntBody*>::dListNode* ntBody::GetBroadPhaseNode() const
//{
//	return m_broadPhaseNode;
//}
//


#endif 

