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

#include "ntNewtonStdafx.h"
#include "ntShapeInstance.h"

class ntWorld;
class ntContact;
class ntBodyNotify;
class ntBodyDynamic;
class ntBodyKinematic;
class ntRayCastNotify;
class ntBilateralJoint;
class ntBroadPhaseBodyNode;
class ntBroadPhaseAggregate;

D_MSV_NEWTON_ALIGN_32
class ntBody: public dClassAlloc
{
	public:
	D_NEWTON_API ntBody();
	D_NEWTON_API virtual ~ntBody();

	dUnsigned32 GetID() const { return m_uniqueID; }

	virtual ntBody* GetAsBody() { return this;}
	virtual ntBodyDynamic* GetAsBodyDynamic() { return nullptr; }
	virtual ntBodyKinematic* GetAsBodyKinematic() { return nullptr; }


	D_NEWTON_API void SetCentreOfMass(const dVector& com);

	D_NEWTON_API void SetNotifyCallback(ntBodyNotify* const notify);
	D_NEWTON_API ntBodyNotify* GetNotifyCallback(ntBodyNotify* const notify) const;

	D_NEWTON_API dInt32 GetId() const;
	D_NEWTON_API ntWorld* GetWorld() const;

	D_NEWTON_API dVector GetOmega() const;
	D_NEWTON_API void SetOmega(const dVector& veloc);

	D_NEWTON_API dVector GetVelocity() const;
	D_NEWTON_API void SetVelocity(const dVector& veloc);

	D_NEWTON_API dMatrix GetMatrix() const;
	D_NEWTON_API void SetMatrix(const dMatrix& matrix);

	virtual dFloat32 GetInvMass() const {return dFloat32 (0.0f);}

	D_NEWTON_API virtual void AttachContact(ntContact* const contact) {}
	D_NEWTON_API virtual void DetachContact(ntContact* const contact) {}
	virtual ntContact* FindContact(const ntBody* const otherBody) const {return nullptr;}

	virtual dFloat32 RayCast(ntRayCastNotify& callback, const dFastRayTest& ray, const dFloat32 maxT) const = 0;

	protected:
	dList<ntBody*>::dListNode* GetNewtonNode() const;
	void SetWorldNode(ntWorld* const world, dList<ntBody*>::dListNode* const node);

	dMatrix m_matrix;
	dMatrix m_invWorldInertiaMatrix;
	dVector m_veloc;
	dVector m_omega;
	dVector m_localCentreOfMass;
	dVector m_globalCentreOfMass;
	dVector m_minAABB;
	dVector m_maxAABB;
	dQuaternion m_rotation;

	ntWorld* m_world;
	ntBodyNotify* m_notifyCallback;
	dList<ntBody*>::dListNode* m_worldNode;

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

	friend class ntWorld;
} D_GCC_NEWTON_ALIGN_32;


#endif 

