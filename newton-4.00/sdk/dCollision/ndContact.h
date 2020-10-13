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

#ifndef __D_CONTACT_H__
#define __D_CONTACT_H__

#include "ndCollisionStdafx.h"
#include "ndConstraint.h"
#include "ndContactNotify.h"
#include "ndContactSolver.h"

class ndBodyKinematic;
class ndShapeInstance;

#define D_MAX_CONTATCS					128
#define D_CONSTRAINT_MAX_ROWS			(3 * 16)
#define D_RESTING_CONTACT_PENETRATION	(D_PENETRATION_TOL + dFloat32 (1.0f / 1024.0f))
#define D_DIAGONAL_PRECONDITIONER		dFloat32 (25.0f)

D_MSV_NEWTON_ALIGN_32
class ndContactPoint
{
	public:
	dVector m_point;
	dVector m_normal;
	const ndBodyKinematic* m_body0;
	const ndBodyKinematic* m_body1;
	const ndShapeInstance* m_shapeInstance0;
	const ndShapeInstance* m_shapeInstance1;
	//dInt64 m_shapeId0;
	//dInt64 m_shapeId1;
	dFloat32 m_penetration;
} D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class ndContactMaterial: public ndContactPoint
{
	public:
	//D_MSV_NEWTON_ALIGN_32
	//class ndUserContactPoint
	//{
	//	public:
	//	dVector m_point;
	//	dVector m_normal;
	//	dUnsigned64 m_shapeId0;
	//	dUnsigned64 m_shapeId1;
	//	dFloat32 m_penetration;
	//	dUnsigned32 m_unused[3];
	//} D_GCC_NEWTON_ALIGN_32;

	//typedef bool (dgApi *OnAABBOverlap) (dgContact& contactJoint, dFloat32 timestep, dInt32 threadIndex);
	//typedef void (dgApi *OnContactCallback) (dgContact& contactJoint, dFloat32 timestep, dInt32 threadIndex);
	//typedef bool (dgApi *OnCompoundCollisionPrefilter) (dgContact& contactJoint, dFloat32 timestep, const dgBody* bodyA, const void* collisionNodeA, const dgBody* bodyB, const void* collisionNodeB, dInt32 threadIndex);
	//typedef bool (dgApi *OnContactGeneration) (const dgContactMaterial& material, const dgBody& body0, const dgCollisionInstance* collisionIntance0, const dgBody& body1, const dgCollisionInstance* collisionIntance1, dgUserContactPoint* const contacts, dInt32 maxCount, dInt32 threadIndex);

	ndContactMaterial()
		:m_dir0(dVector::m_zero)
		,m_dir1(dVector::m_zero)
		,m_material()
	{
		m_dir0_Force.Clear();
		m_dir1_Force.Clear();
		m_normal_Force.Clear();
	}
	//void* GetUserData() const;
	//void SetUserData(void* const userData);
	//void SetAsSoftContact(dFloat32 regularizer);
	//void SetCollisionGenerationCallback(OnContactGeneration contactGeneration);
	//void SetCollisionCallback(OnAABBOverlap abbOvelap, OnContactCallback callback);
	//void SetCompoundCollisionCallback(OnCompoundCollisionPrefilter abbCompounndOvelap);

	dVector m_dir0;
	dVector m_dir1;
	ndForceImpactPair m_normal_Force;
	ndForceImpactPair m_dir0_Force;
	ndForceImpactPair m_dir1_Force;
	ndMaterial m_material;
} D_GCC_NEWTON_ALIGN_32;

class ndContactPointList: public dList<ndContactMaterial, dContainersFreeListAlloc<ndContactPoint>>
{
	public:
};

D_MSV_NEWTON_ALIGN_32 
class ndContact
	:public ndConstraint
	,public dContainersFreeListAlloc<ndContact*>
{
	public:
	D_COLLISION_API ndContact();
	D_COLLISION_API virtual ~ndContact();

	D_COLLISION_API virtual ndBodyKinematic* GetBody0() const;
	D_COLLISION_API virtual ndBodyKinematic* GetBody1() const;

	D_COLLISION_API void AttachToBodies();
	D_COLLISION_API void DetachFromBodies();

	ndContact* GetAsContact() { return this; }
	dFloat32 GetPruningTolerance() const;

	virtual const dUnsigned32 GetRowsCount() const;
	virtual void JacobianDerivative(ndConstraintDescritor& desc);
	virtual void JointAccelerations(ndJointAccelerationDecriptor* const desc);

	const ndContactPointList& GetContactPoints() const;

	bool IsActive() const;
	bool IsSkeletonSelftCollision() const;
	bool IsSkeletonIntraCollision() const;
	
	private:
	void SetBodies(ndBodyKinematic* const body0, ndBodyKinematic* const body1);
	void CalculatePointDerivative(dInt32 index, ndConstraintDescritor& desc, const dVector& dir, const dgPointParam& param) const;
	void JacobianContactDerivative(ndConstraintDescritor& desc, const ndContactMaterial& contact, dInt32 normalIndex, dInt32& frictionIndex);

	dVector m_positAcc;
	dQuaternion m_rotationAcc;
	dVector m_separatingVector;
	ndContactPointList m_contacPointsList;
	ndBodyKinematic* m_body0;
	ndBodyKinematic* m_body1;
	dList<ndContact, dContainersFreeListAlloc<ndContact>>::dListNode* m_linkNode;
	dFloat32 m_timeOfImpact;
	dFloat32 m_separationDistance;
	dFloat32 m_contactPruningTolereance;
	dUnsigned32 m_maxDOF;
	dUnsigned32 m_sceneLru;
	dUnsigned32 m_active : 1;
	dUnsigned32 m_isTrigger : 1;
	dUnsigned32 m_isAttached : 1;
	dUnsigned32 m_killContact : 1;
	dUnsigned32 m_skeletonIntraCollision : 1;
	dUnsigned32 m_skeletonSelftCollision : 1;
	static dVector m_initialSeparatingVector;

	friend class ndScene;
	friend class ndContactList;
	friend class ndBodyKinematic;
	friend class ndContactSolver;
} D_GCC_NEWTON_ALIGN_32 ;

#if 0
DG_INLINE void dgContactMaterial::SetCollisionCallback (OnAABBOverlap aabbOverlap, OnContactCallback contact) 
{
	m_aabbOverlap = aabbOverlap;
	m_processContactPoint = contact;
}

DG_INLINE void dgContactMaterial::SetCollisionGenerationCallback (OnContactGeneration contactGeneration)
{
	m_contactGeneration = contactGeneration;
}

DG_INLINE void dgContactMaterial::SetCompoundCollisionCallback (OnCompoundCollisionPrefilter aabbOverlap)
{
	m_compoundAABBOverlap = aabbOverlap;
}

DG_INLINE void* dgContactMaterial::GetUserData () const
{
	return m_userData;
}

DG_INLINE void dgContactMaterial::SetUserData (void* const userData)
{
	m_userData = userData;
}

DG_INLINE void dgContactMaterial::SetAsSoftContact(dFloat32 regularizer)
{
	dAssert(regularizer >= dFloat32 (0.0f));
	dAssert(regularizer <= dFloat32 (1.0f));
	// re purpose some of the variable to store parameter for soft contact
	m_flags |= m_isSoftContact;
	m_skinThickness = regularizer;
}

DG_INLINE const dgContactMaterial* ndContact::GetMaterial() const
{
	return m_material;
}

DG_INLINE bool ndContact::IsDeformable() const 
{
	return false;
}

DG_INLINE void ndContact::SetDestructorCallback (OnConstraintDestroy destructor)
{
}

DG_INLINE void ndContact::SetTimeOfImpact(dFloat32 timetoImpact)
{
	m_timeOfImpact = timetoImpact;
}

DG_INLINE dFloat32 ndContact::GetTimeOfImpact() const
{
	return m_timeOfImpact;
}

DG_INLINE dFloat32 ndContact::GetClosestDistance() const
{
    return m_closestDistance;
}

DG_INLINE void ndContact::ResetMaxDOF()
{
	m_maxDOF = 0;
}

DG_INLINE void ndContact::SetPruningTolerance(dFloat32 tolerance)
{
	m_contactPruningTolereance = dAbs (tolerance);
}

DG_INLINE void ndContact::ResetSkeletonIntraCollision()
{
	m_skeletonIntraCollision = 0;
}


DG_INLINE void ndContact::ResetSkeletonSelftCollision()
{
	m_skeletonSelftCollision = 0;
}


DG_INLINE dFloat32 ndContact::GetImpulseContactSpeed() const
{
	return m_impulseSpeed;
}

DG_INLINE void ndContact::SetImpulseContactSpeed(dFloat32 speed)
{
	m_impulseSpeed = speed;
}
#endif

D_INLINE dFloat32 ndContact::GetPruningTolerance() const
{
	return m_contactPruningTolereance;
}

inline const dUnsigned32 ndContact::GetRowsCount() const
{
	return m_maxDOF;
}

inline ndBodyKinematic* ndContact::GetBody0() const
{
	return m_body0;
}

inline ndBodyKinematic* ndContact::GetBody1() const
{
	return m_body1;
}

inline const ndContactPointList& ndContact::GetContactPoints() const
{
	return m_contacPointsList;
}

inline bool ndContact::IsActive() const
{
	return m_active ? true : false;
}

inline bool ndContact::IsSkeletonSelftCollision() const
{
	return m_skeletonSelftCollision ? true : false;;
}

inline bool ndContact::IsSkeletonIntraCollision() const
{
	return m_skeletonIntraCollision ? true : false;
}

#endif 

