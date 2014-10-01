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

#ifndef _DG_BODY_H_
#define _DG_BODY_H_

#include "dgPhysicsStdafx.h"
#include "dgBroadPhase.h"
#include "dgBodyMasterList.h"

class dgLink;
class dgBody;  
class dgWorld;  
class dgCollision;
class dgBroadPhaseNode;
class dgCollisionInstance;


#define DG_MINIMUM_MASS		dgFloat32(1.0e-3f)
#define DG_INFINITE_MASS	dgFloat32(1.0e15f)

#define OverlapTest(body0,body1) dgOverlapTest ((body0)->m_minAABB, (body0)->m_maxAABB, (body1)->m_minAABB, (body1)->m_maxAABB)

//DG_MSC_VECTOR_ALIGMENT
DG_MSC_VECTOR_ALIGMENT
class dgBody  
{
	public:
	typedef void (dgApi *OnBodyDestroy) (dgBody& me);
	typedef void (dgApi *OnApplyExtForceAndTorque) (dgBody& me, dgFloat32 timestep, dgInt32 threadIndex);
	typedef void (dgApi *OnMatrixUpdateCallback) (const dgBody& me, const dgMatrix& matrix, dgInt32 threadIndex);

	enum dgRTTI
	{
		m_baseBodyRTTI = 1<<0,
		m_dynamicBodyRTTI = 1<<1,
		m_kinematicBodyRTTI = 1<<2,
		m_deformableBodyRTTI = 1<<3,
	};

	enum dgType
	{
		m_dynamicBody = 0,
		m_kinematicBody,
		m_deformableBody,
	};


	DG_CLASS_ALLOCATOR(allocator)


	dgBody();
	dgBody (dgWorld* const world, const dgTree<const dgCollision*, dgInt32>* const collisionNode, dgDeserialize serializeCallback, void* const userData);
	virtual ~dgBody();

	dgType GetType () const;
	dgInt32 IsRTTIType (dgUnsigned32 rtti) const;

	void* GetUserData() const;
	void SetUserData (void* const userData);
	dgWorld* GetWorld() const;


	const dgMatrix& GetMatrix() const;
	const dgQuaternion& GetRotation() const;
	const dgVector& GetPosition() const;
	

	void SetCentreOfMass (const dgVector& com);
	const dgVector& GetCentreOfMass () const;

	void GetAABB (dgVector &p0, dgVector &p1) const;	

	const dgVector& GetOmega() const;
	const dgVector& GetVelocity() const;
	const dgVector& GetNetForce() const;
	const dgVector& GetNetTorque() const;

	dgVector GetVelocityAtPoint (const dgVector& point) const;

	void SetOmega (const dgVector& omega);
	void SetVelocity (const dgVector& velocity);

	dgUnsigned32 GetGroupID () const;
	virtual void SetGroupID (dgUnsigned32 id);
	dgInt32 GetUniqueID () const;

	bool GetContinueCollisionMode () const;
	void SetContinueCollisionMode (bool mode);
	bool GetCollisionWithLinkedBodies () const;
	void SetCollisionWithLinkedBodies (bool state);

	void Freeze ();
	void Unfreeze ();
	bool GetFreeze () const;
	void SetFreeze (bool state);

	bool GetSleepState () const;
	void SetSleepState (bool state);

	bool GetAutoSleep () const;
	void SetAutoSleep (bool state);

	dgCollisionInstance* GetCollision () const;
	dgBodyMasterList::dgListNode* GetMasterList() const;

	OnBodyDestroy GetDestructorCallback () const;
	void SetDestructorCallback (OnBodyDestroy destructor);
	OnMatrixUpdateCallback GetMatrixUpdateCallback () const;
	void SetMatrixUpdateCallback (OnMatrixUpdateCallback callback);

	dgVector GetMass() const;
	dgVector GetInvMass() const;
	const dgMatrix& GetInertiaMatrix () const;

	virtual dgMatrix CalculateInertiaMatrix () const;
	virtual dgMatrix CalculateInvInertiaMatrix () const;

	bool IsCollidable() const;
	virtual void SetCollidable (bool state) = 0;
	virtual bool IsInEquilibrium () const = 0;

	virtual const dgVector& GetForce() const = 0;
	virtual const dgVector& GetTorque() const = 0;
	virtual void AddForce (const dgVector& force) = 0;
	virtual void AddTorque (const dgVector& torque) = 0;
	virtual void SetForce (const dgVector& force) = 0;
	virtual void SetTorque (const dgVector& torque) = 0;

	virtual dgFloat32 GetLinearDamping () const = 0;
	virtual dgVector GetAngularDamping () const = 0;
	virtual void SetLinearDamping (dgFloat32 linearDamp) = 0;
	virtual void SetAngularDamping (const dgVector& angularDamp) = 0;
	virtual void AddDampingAcceleration() = 0;

	virtual void SetMassMatrix (dgFloat32 mass, dgFloat32 Ix, dgFloat32 Iy, dgFloat32 Iz);
	virtual void SetMassProperties (dgFloat32 mass, const dgCollisionInstance* const collision);

	virtual dgVector PredictLinearVelocity(dgFloat32 timestep) const = 0;
	virtual dgVector PredictAngularVelocity(dgFloat32 timestep) const = 0;


	virtual void AddImpulse (const dgVector& pointVeloc, const dgVector& pointPosit);
	virtual void ApplyImpulsePair (const dgVector& linearImpulse, const dgVector& angularImpulse);
	virtual void ApplyImpulsesAtPoint (dgInt32 count, dgInt32 strideInBytes, const dgFloat32* const impulseArray, const dgFloat32* const pointArray);
	
	virtual void InvalidateCache();
	
    virtual void SetMatrix(const dgMatrix& matrix);
    virtual void SetMatrixIgnoreSleep(const dgMatrix& matrix);
	virtual void IntegrateVelocity (dgFloat32 timestep);
	virtual void AttachCollision (dgCollisionInstance* const collision);
    
	virtual void ApplyExtenalForces (dgFloat32 timestep, dgInt32 threadIndex) = 0;		
	virtual OnApplyExtForceAndTorque GetExtForceAndTorqueCallback () const = 0;
	virtual void SetExtForceAndTorqueCallback (OnApplyExtForceAndTorque callback) = 0;

	virtual dgFloat32 RayCast (const dgLineBox& line, OnRayCastAction filter, OnRayPrecastAction preFilter, void* const userData, dgFloat32 minT) const;
	virtual dgFloat32 ConvexRayCast (const dgFastRayTest& ray, const dgCollisionInstance* const convexShape, const dgVector& shapeMinBox, const dgVector& shapeMaxBox, const dgMatrix& origin, const dgVector& shapeVeloc, OnRayCastAction filter, OnRayPrecastAction preFilter, void* const userData, dgFloat32 minT, dgInt32 threadId) const;

	
	virtual void Serialize (const dgTree<dgInt32, const dgCollision*>* const collisionNode, dgSerialize serializeCallback, void* const userData);
	
	virtual dgConstraint* GetFirstJoint() const;
	virtual dgConstraint* GetNextJoint(dgConstraint* const joint) const;
	virtual dgConstraint* GetFirstContact() const;
	virtual dgConstraint* GetNextContact(dgConstraint* const joint) const;
	virtual dgVector CalculateInverseDynamicForce (const dgVector& desiredVeloc, dgFloat32 timestep) const;

    void SetMatrixOriginAndRotation(const dgMatrix& matrix);

	protected:
	void UpdateWorlCollisionMatrix() const;
	void UpdateMatrix (dgFloat32 timestep, dgInt32 threadIndex);
	void UpdateCollisionMatrix (dgFloat32 timestep, dgInt32 threadIndex);

		
	// member variables:
	protected:
	void CalcInvInertiaMatrix ();
	dgVector GetApparentMass() const;
	void SetAparentMassMatrix (const dgVector& massMatrix);


	dgMatrix m_invWorldInertiaMatrix;
	dgMatrix m_matrix;
	dgQuaternion m_rotation;
	dgVector m_mass;
	dgVector m_invMass;
	dgVector m_veloc;
	dgVector m_omega;
	dgVector m_minAABB;
	dgVector m_maxAABB;
	dgVector m_netForce;
	dgVector m_netTorque;
	dgVector m_localCentreOfMass;	
	dgVector m_globalCentreOfMass;	
	dgVector m_aparentMass;
	dgInt32 m_index;
	dgInt32 m_uniqueID;
	dgInt32 m_bodyGroupId;
	dgInt32 m_rtti;
	dgInt32 m_type;
	dgUnsigned32 m_dynamicsLru;	
	dgUnsigned32 m_genericLRUMark;
	
	dgThread::dgCriticalSection m_criticalSectionLock;
	union 
	{
		dgUnsigned32 m_flags;
		struct {
			dgUnsigned32 m_freeze					: 1;
			dgUnsigned32 m_sleeping					: 1;
			dgUnsigned32 m_autoSleep				: 1;
			dgUnsigned32 m_equilibrium				: 1;
			dgUnsigned32 m_continueCollisionMode    : 1;
			dgUnsigned32 m_spawnnedFromCallback		: 1;
			dgUnsigned32 m_collideWithLinkedBodies  : 1;
			dgUnsigned32 m_inCallback				: 1;
			dgUnsigned32 m_collidable				: 1;
			dgUnsigned32 m_resting					: 1;
			dgUnsigned32 m_active					: 1;
		};
	};

	void* m_userData;
	dgWorld* m_world;
	dgCollisionInstance* m_collision;
	dgBroadPhase::dgNode* m_collisionCell;
	dgBodyMasterList::dgListNode* m_masterNode;
	
	OnBodyDestroy m_destructor;
	OnMatrixUpdateCallback m_matrixUpdate;

	friend class dgWorld;
	friend class dgContact;
	friend class dgConstraint;
	friend class dgBroadPhase;
	friend class dgAmpInstance;
	friend class dgCollisionBVH;
	friend class dgBroadPhaseNode;
	friend class dgBodyMasterList;
	friend class dgCollisionScene;
	friend class dgCollisionConvex;
	friend class dgCollisionCompound;
	friend class dgCollisionUserMesh;
	friend class dgWorldDynamicUpdate;
	friend class dgBilateralConstraint;
	friend class dgCollisionConvexPolygon;
	friend class dgCollidingPairCollector;

} DG_GCC_VECTOR_ALIGMENT;

// *****************************************************************************
// 
//	 Implementation	
// 
// *****************************************************************************
DG_INLINE const dgMatrix& dgBody::GetInertiaMatrix () const 
{
	return m_invWorldInertiaMatrix;
}



DG_INLINE dgVector dgBody::GetInvMass() const
{
	return m_invMass;
}

DG_INLINE dgVector dgBody::GetMass() const
{
	return m_mass;
}


DG_INLINE dgBody::dgType dgBody::GetType () const
{
	return dgType (m_type);
}

DG_INLINE dgInt32 dgBody::IsRTTIType (dgUnsigned32 rtti) const
{
	return rtti & m_rtti;
}

DG_INLINE void dgBody::SetUserData(void* const userData)
{
	m_userData = userData;
}

DG_INLINE void* dgBody::GetUserData() const
{
	return m_userData;
}

DG_INLINE dgWorld* dgBody::GetWorld() const
{
	return m_world;
}


DG_INLINE dgUnsigned32 dgBody::GetGroupID () const
{
	return dgUnsigned32 (m_bodyGroupId);
}

DG_INLINE void dgBody::SetGroupID (dgUnsigned32 id)
{
	m_bodyGroupId = dgInt32 (id);
}



DG_INLINE dgBodyMasterList::dgListNode* dgBody::GetMasterList() const
{
	return m_masterNode;
}

DG_INLINE void dgBody::GetAABB (dgVector &p0, dgVector &p1) const
{
	p0.m_x = m_minAABB.m_x;
	p0.m_y = m_minAABB.m_y;
	p0.m_z = m_minAABB.m_z;
	p1.m_x = m_maxAABB.m_x;
	p1.m_y = m_maxAABB.m_y;
	p1.m_z = m_maxAABB.m_z;
}

DG_INLINE const dgVector& dgBody::GetOmega() const
{
	return m_omega;
}

DG_INLINE const dgVector& dgBody::GetVelocity() const
{
	return m_veloc; 
}

DG_INLINE void dgBody::SetOmega (const dgVector& omega)
{
	m_omega = omega;
	m_equilibrium = false;
}

DG_INLINE dgVector dgBody::GetVelocityAtPoint (const dgVector& point) const
{
	return m_veloc + m_omega * (point - m_globalCentreOfMass);
}

DG_INLINE void dgBody::SetVelocity (const dgVector& velocity)
{
	m_veloc = velocity;
	m_equilibrium = false;
}

DG_INLINE const dgMatrix& dgBody::GetMatrix() const
{
	return m_matrix;
}

DG_INLINE const dgVector& dgBody::GetPosition() const
{
	return m_matrix.m_posit;
}

DG_INLINE const dgQuaternion& dgBody::GetRotation() const
{
	return m_rotation;
}

DG_INLINE const dgVector& dgBody::GetCentreOfMass () const
{
	return m_localCentreOfMass;
}

DG_INLINE void dgBody::SetCentreOfMass (const dgVector& com)
{
	m_localCentreOfMass.m_x = com.m_x;
	m_localCentreOfMass.m_y = com.m_y;
	m_localCentreOfMass.m_z = com.m_z;
	m_localCentreOfMass.m_w = dgFloat32 (1.0f);
	m_globalCentreOfMass = m_matrix.TransformVector (m_localCentreOfMass);
}


DG_INLINE dgInt32 dgBody::GetUniqueID () const 
{
	return m_uniqueID;
}


DG_INLINE void dgBody::SetDestructorCallback (OnBodyDestroy destructor)
{
	m_destructor = destructor;
}

DG_INLINE dgBody::OnBodyDestroy dgBody::GetDestructorCallback () const
{
	return m_destructor;
}


DG_INLINE void dgBody::SetMatrixUpdateCallback (OnMatrixUpdateCallback callback)
{
	m_matrixUpdate = callback;
}

DG_INLINE dgBody::OnMatrixUpdateCallback dgBody::GetMatrixUpdateCallback () const
{
	return m_matrixUpdate;
}



DG_INLINE dgCollisionInstance* dgBody::GetCollision () const
{
	return m_collision;
}

DG_INLINE void dgBody::SetContinueCollisionMode (bool mode)
{
	m_continueCollisionMode = dgUnsigned32 (mode);
}

DG_INLINE bool dgBody::GetContinueCollisionMode () const
{
	return m_continueCollisionMode;
}

DG_INLINE void dgBody::SetCollisionWithLinkedBodies (bool state)
{
	m_collideWithLinkedBodies = dgUnsigned32 (state);
}

DG_INLINE bool dgBody::GetCollisionWithLinkedBodies () const
{
	return m_collideWithLinkedBodies;
}

DG_INLINE bool dgBody::GetFreeze () const
{
	return m_freeze;
}

DG_INLINE void dgBody::SetAutoSleep (bool state)
{
	m_autoSleep = dgUnsigned32 (state);
	if (m_autoSleep == 0) {
		m_sleeping = false;
	}
}

DG_INLINE bool dgBody::GetAutoSleep () const
{
	return m_autoSleep;
}

DG_INLINE bool dgBody::GetSleepState () const
{
	return m_sleeping;
}

DG_INLINE void dgBody::SetSleepState (bool state)
{
	m_sleeping = state;
	m_equilibrium = state;
}


DG_INLINE bool dgBody::IsCollidable() const
{
	return m_collidable;
}

DG_INLINE dgVector dgBody::GetApparentMass() const
{
	return m_aparentMass;
}


DG_INLINE void dgBody::SetMatrixOriginAndRotation(const dgMatrix& matrix)
{
	m_matrix = matrix;

#ifdef _DEBUG
	for (int i = 0; i < 4; i ++) {
		for (int j = 0; j < 4; j ++) {
			dgAssert (dgCheckFloat(m_matrix[i][j]));
		}
	}

	int j0 = 1;
	int j1 = 2;
	for (dgInt32 i = 0; i < 3; i ++) {
		dgAssert (m_matrix[i][3] == 0.0f);
		dgFloat32 val = m_matrix[i] % m_matrix[i];
		dgAssert (dgAbsf (val - 1.0f) < 1.0e-4f);
		dgVector tmp (m_matrix[j0] * m_matrix[j1]);
		val = tmp % m_matrix[i];
		dgAssert (dgAbsf (val - 1.0f) < 1.0e-4f);
		j0 = j1;
		j1 = i;
	}
#endif

	m_rotation = dgQuaternion (m_matrix);
	m_globalCentreOfMass = m_matrix.TransformVector (m_localCentreOfMass);
}


DG_INLINE const dgVector& dgBody::GetNetForce() const
{
	return m_netForce; 
}

DG_INLINE const dgVector& dgBody::GetNetTorque() const
{
	return m_netTorque;
}


DG_INLINE void dgBody::CalcInvInertiaMatrix ()
{
	dgAssert (m_invWorldInertiaMatrix[0][3] == dgFloat32 (0.0f));
	dgAssert (m_invWorldInertiaMatrix[1][3] == dgFloat32 (0.0f));
	dgAssert (m_invWorldInertiaMatrix[2][3] == dgFloat32 (0.0f));
	dgAssert (m_invWorldInertiaMatrix[3][3] == dgFloat32 (1.0f));

	m_invWorldInertiaMatrix = CalculateInvInertiaMatrix ();

	dgAssert (m_invWorldInertiaMatrix[0][3] == dgFloat32 (0.0f));
	dgAssert (m_invWorldInertiaMatrix[1][3] == dgFloat32 (0.0f));
	dgAssert (m_invWorldInertiaMatrix[2][3] == dgFloat32 (0.0f));
	dgAssert (m_invWorldInertiaMatrix[3][3] == dgFloat32 (1.0f));
}


#endif 

