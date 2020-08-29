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

#include "ntStdafx.h"
#include "ntShapeInstance.h"

class ntWorld;
class ntContact;
class ntDynamicBody;
class ntBilateralJoint;
class ntBroadPhaseBodyNode;
class ntBroadPhaseAggregate;

D_MSC_VECTOR_ALIGNMENT
class ntBodyNotify: public dClassAlloc
{
	public:  
	ntBodyNotify()
		:dClassAlloc()
		,m_body(nullptr)
	{
	}

	virtual ~ntBodyNotify()
	{
	}

	virtual void OnApplyExternalForce(dInt32 threadIndex, dFloat32 timestep)
	{
	}

	virtual void OnTranform(dInt32 threadIndex, const dMatrix& matrix)
	{
	}

	protected:
	ntBody* m_body;

	friend class ntBody;

} D_GCC_VECTOR_ALIGNMENT;

D_MSC_VECTOR_ALIGNMENT
class ntBody: public dClassAlloc
{
	public:
	D_NEWTON_API ntBody();
	D_NEWTON_API virtual ~ntBody();

	dUnsigned32 GetID() const { return m_uniqueID; }
	virtual ntBody* GetAsBody() { return this;}
	virtual ntDynamicBody* GetAsDynamicBody() { return nullptr; }

	D_NEWTON_API const ntShapeInstance& GetCollisionShape() const;
	D_NEWTON_API void SetCollisionShape(const ntShapeInstance& shapeInstance);

	D_NEWTON_API void SetCentreOfMass(const dVector& com);

	D_NEWTON_API void SetNotifyCallback(ntBodyNotify* const notify);
	D_NEWTON_API ntBodyNotify* GetNotifyCallback(ntBodyNotify* const notify) const;

	D_NEWTON_API dInt32 GetId() const;
	D_NEWTON_API ntWorld* GetNewton() const;

	D_NEWTON_API dVector GetOmega() const;
	D_NEWTON_API void SetOmega(const dVector& veloc);

	D_NEWTON_API dVector GetVelocity() const;
	D_NEWTON_API void SetVelocity(const dVector& veloc);

	D_NEWTON_API dMatrix GetMatrix() const;
	D_NEWTON_API void SetMatrix(const dMatrix& matrix);

	protected:
	dList<ntBody*>::dListNode* GetNewtonNode() const;
	void SetNewtonNode(ntWorld* const newton, dList<ntBody*>::dListNode* const node);

	void UpdateCollisionMatrix();

	ntBroadPhaseBodyNode* GetBroadPhaseNode() const;
	void SetBroadPhaseNode(ntBroadPhaseBodyNode* const node);

	ntBroadPhaseAggregate* GetBroadPhaseAggregate() const;
	void SetBroadPhaseAggregate(ntBroadPhaseAggregate* const node);

	dMatrix m_matrix;
	dMatrix m_invWorldInertiaMatrix;
	ntShapeInstance m_shapeInstance;
		
	dVector m_veloc;
	dVector m_omega;
	dVector m_localCentreOfMass;
	dVector m_globalCentreOfMass;
	dVector m_minAABB;
	dVector m_maxAABB;
	dQuaternion m_rotation;

	dArray<ntBilateralJoint*> m_jointArray;
	dTree<ntContact*, dUnsigned32> m_contactList;
	ntBodyNotify* m_notifyCallback;
	ntWorld* m_newton;
	ntBroadPhaseBodyNode* m_broadPhaseNode;
	dList<ntBody*>::dListNode* m_newtonNode;
	ntBroadPhaseAggregate* m_broadPhaseAggregateNode;

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
			//dUnsigned32 m_collideWithLinkedBodies : 1;
			//dUnsigned32 m_transformIsDirty : 1;
			//dUnsigned32 m_gyroTorqueOn : 1;
			//dUnsigned32 m_isdead : 1;
		};
	};

	dUnsigned32 m_uniqueID;

	static dUnsigned32 m_uniqueIDCount;

	friend class ntWorld;
	friend class ntBroadPhase;
	friend class ntBroadPhaseMixed;
	friend class ntBroadPhaseBodyNode;
} D_GCC_VECTOR_ALIGNMENT;


#endif 

