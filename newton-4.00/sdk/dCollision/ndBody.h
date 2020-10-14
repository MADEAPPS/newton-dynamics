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
#include "ndShapeInstance.h"

class ndContact;
class ndBodyNotify;
class ndBodyDynamic;
class ndBodyKinematic;
class ndRayCastNotify;
class ndBodyTriggerVolume;
class ndJointBilateralConstraint;

D_MSV_NEWTON_ALIGN_32
class ndBody: public dClassAlloc
{
	public:
	D_COLLISION_API ndBody();
	D_COLLISION_API virtual ~ndBody();

	virtual ndBody* GetAsBody() { return this;}
	virtual ndBodyDynamic* GetAsBodyDynamic() { return nullptr; }
	virtual ndBodyKinematic* GetAsBodyKinematic() { return nullptr; }
	virtual ndBodyKinematic* GetAsBodyPlayerCapsule() { return nullptr; }
	virtual ndBodyTriggerVolume* GetAsBodyTriggerVolume() { return nullptr; }

	dUnsigned32 GetId() const;
	virtual const dFloat32 GetInvMass() const { return dFloat32(0.0f); }
	virtual dFloat32 RayCast(ndRayCastNotify& callback, const dFastRayTest& ray, const dFloat32 maxT) const = 0;

	const dVector& GetCentreOfMass() const;
	D_COLLISION_API void SetCentreOfMass(const dVector& com);

	ndBodyNotify* GetNotifyCallback() const;
	D_COLLISION_API void SetNotifyCallback(ndBodyNotify* const notify);
	D_COLLISION_API dVector GetOmega() const;
	D_COLLISION_API void SetOmega(const dVector& veloc);
	D_COLLISION_API dVector GetVelocity() const;
	D_COLLISION_API void SetVelocity(const dVector& veloc);
	D_COLLISION_API dMatrix GetMatrix() const;
	D_COLLISION_API void SetMatrix(const dMatrix& matrix);
	D_COLLISION_API dQuaternion GetRotation() const;

	void GetAABB(dVector& p0, dVector& p1) const;

	protected:
	virtual void AttachContact(ndContact* const contact) {}
	virtual void DetachContact(ndContact* const contact) {}
	virtual ndContact* FindContact(const ndBody* const otherBody) const { return nullptr; }

	dMatrix m_matrix;
	dVector m_veloc;
	dVector m_omega;
	dVector m_localCentreOfMass;
	dVector m_globalCentreOfMass;
	dVector m_minAABB;
	dVector m_maxAABB;
	dQuaternion m_rotation;
	ndBodyNotify* m_notifyCallback;

	union
	{
		dUnsigned32 m_flags;
		struct
		{
			dUnsigned32 m_resting : 1;
			dUnsigned32 m_autoSleep : 1;
			dUnsigned32 m_equilibrium : 1;
			dUnsigned32 m_islandSleep : 1;
			dUnsigned32 m_gyroTorqueOn : 1;
			dUnsigned32 m_skeletonMark : 1;
			dUnsigned32 m_transformIsDirty : 1;
			dUnsigned32 m_bodyIsConstrained : 1;
			dUnsigned32 m_collideWithLinkedBodies : 1;

			//dUnsigned32 m_freeze : 1;
			//dUnsigned32 m_sleeping : 1;
			//dUnsigned32 m_inCallback : 1;
			//dUnsigned32 m_jointSet : 1;
			//dUnsigned32 m_collidable : 1;
			//dUnsigned32 m_spawnnedFromCallback : 1;
			//dUnsigned32 m_continueCollisionMode : 1;
			//dUnsigned32 m_isdead : 1;
		};
	};

	dUnsigned32 m_uniqueID;
	static dUnsigned32 m_uniqueIDCount;

	friend class ndScene;
	friend class ndConstraint;
} D_GCC_NEWTON_ALIGN_32;


inline dUnsigned32 ndBody::GetId() const
{
	return m_uniqueID;
}

inline ndBodyNotify* ndBody::GetNotifyCallback() const
{
	return m_notifyCallback;
}

inline dVector ndBody::GetVelocity() const
{
	return m_veloc;
}

inline dVector ndBody::GetOmega() const
{
	return m_omega;
}

inline void ndBody::GetAABB(dVector& p0, dVector& p1) const
{
	p0 = m_minAABB;
	p1 = m_maxAABB;
}

inline const dVector& ndBody::GetCentreOfMass() const
{
	return m_localCentreOfMass;
}


#endif 

