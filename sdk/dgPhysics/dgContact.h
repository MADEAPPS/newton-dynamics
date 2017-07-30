/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __DGCONTACT_H__
#define __DGCONTACT_H__

#include "dgConstraint.h"
#include "dgContactSolver.h"

class dgBody;
class dgWorld;
class dgContact; 
class dgContactPoint; 
class dgContactMaterial;
class dgPolygonMeshDesc;
class dgCollisionInstance;


#define DG_MAX_CONTATCS					128
#define DG_RESTING_CONTACT_PENETRATION	(DG_PENETRATION_TOL + dgFloat32 (1.0f / 1024.0f))

class dgActiveContacts: public dgList<dgContact*>
{
	public:
	dgActiveContacts (dgMemoryAllocator* const allocator)
		:dgList<dgContact*>(allocator)
	{
	}
};


DG_MSC_VECTOR_ALIGMENT
class dgCollisionParamProxy
{	
	public:
	dgCollisionParamProxy(dgContact* const contact, dgContactPoint* const contactBuffer, dgInt32 threadIndex, bool ccdMode, bool intersectionTestOnly)
		:m_normal(dgVector::m_zero)
		,m_closestPointBody0(dgVector::m_zero)
		,m_closestPointBody1(dgVector::m_zero)
		,m_contactJoint(contact)
		,m_contacts(contactBuffer)
		,m_polyMeshData(NULL)		
		,m_threadIndex(threadIndex)
		,m_continueCollision(ccdMode)
		,m_intersectionTestOnly(intersectionTestOnly)
	{
	}

	dgVector m_normal;
	dgVector m_closestPointBody0;
	dgVector m_closestPointBody1;
	dgContact* m_contactJoint;
	dgBody* m_body0;
	dgBody* m_body1;
	dgCollisionInstance* m_instance0;
	dgCollisionInstance* m_instance1;
	dgContactPoint* m_contacts;
	dgPolygonMeshDesc* m_polyMeshData;
	
	dgFloat32 m_timestep;
	dgFloat32 m_skinThickness;
	dgInt32 m_threadIndex;
	dgInt32 m_maxContacts;
	bool m_continueCollision;
	bool m_intersectionTestOnly;

}DG_GCC_VECTOR_ALIGMENT;


DG_MSC_VECTOR_ALIGMENT
class dgContactPoint
{
	public:
	dgVector m_point;
	dgVector m_normal;
	const dgBody* m_body0;
	const dgBody* m_body1;
	const dgCollisionInstance* m_collision0;
	const dgCollisionInstance* m_collision1;
	dgInt64 m_shapeId0;
	dgInt64 m_shapeId1;
	dgFloat32 m_penetration;
}DG_GCC_VECTOR_ALIGMENT;


DG_MSC_VECTOR_ALIGMENT 
class dgContactMaterial: public dgContactPoint
{
	public:
	enum {
		m_collisionEnable = 1<<0,
		m_friction0Enable = 1<<1,
		m_friction1Enable = 1<<2,
		m_override0Accel  = 1<<3,
		m_override1Accel  = 1<<4,
		m_override0Friction = 1<<5,
		m_override1Friction = 1<<6,
		m_overrideNormalAccel = 1<<7,
	};

	DG_MSC_VECTOR_ALIGMENT 
	class dgUserContactPoint
	{
		public:
		dgVector m_point;
		dgVector m_normal;
		dgUnsigned64 m_shapeId0;
		dgUnsigned64 m_shapeId1;
		dgFloat32 m_penetration;
		dgUnsigned32 m_unused[3];
	} DG_GCC_VECTOR_ALIGMENT;


	typedef void (dgApi *OnContactCallback) (dgContact& contactJoint, dgFloat32 timestep, dgInt32 threadIndex);
	typedef bool (dgApi *OnAABBOverlap) (const dgContactMaterial& material, const dgBody& body0, const dgBody& body1, dgInt32 threadIndex);
	typedef bool (dgApi *OnCompoundCollisionPrefilter) (const dgContactMaterial& material, const dgBody* bodyA, const void* collisionNodeA, const dgBody* bodyB, const void* collisionNodeB, dgInt32 threadIndex);
	typedef bool (dgApi *OnContactGeneration) (const dgContactMaterial& material, const dgBody& body0, const dgCollisionInstance* collisionIntance0, const dgBody& body1, const dgCollisionInstance* collisionIntance1, dgUserContactPoint* const contacts, dgInt32 maxCount, dgInt32 threadIndex);

	dgContactMaterial();
	void* GetUserData () const; 
	void SetUserData (void* const userData); 
	void SetCollisionGenerationCallback (OnContactGeneration contactGeneration); 
	void SetCollisionCallback (OnAABBOverlap abbOvelap, OnContactCallback callback); 
	void SetCompoundCollisionCallback (OnCompoundCollisionPrefilter abbCompounndOvelap); 

	dgVector m_dir0;
	dgVector m_dir1;
	dgForceImpactPair m_normal_Force;
	dgForceImpactPair m_dir0_Force;
	dgForceImpactPair m_dir1_Force;
	dgFloat32 m_softness;
	dgFloat32 m_restitution;
	dgFloat32 m_staticFriction0;
	dgFloat32 m_staticFriction1;
	dgFloat32 m_dynamicFriction0;
	dgFloat32 m_dynamicFriction1;
	dgFloat32 m_skinThickness;
	dgInt32 m_flags;

	private:
	void *m_userData;
	OnAABBOverlap m_aabbOverlap;
	OnContactCallback m_processContactPoint;
	OnContactGeneration m_contactGeneration;
	OnCompoundCollisionPrefilter m_compoundAABBOverlap;

	friend class dgWorld;
	friend class dgBroadPhase;
	friend class dgCollisionScene;
	friend class dgCollisionCompound;
	friend class dgWorldDynamicUpdate;
	friend class dgSolverWorlkerThreads;
	friend class dgCollidingPairCollector;
	friend class dgBroadPhaseMaterialCallbackWorkerThread;
	
}DG_GCC_VECTOR_ALIGMENT;



DG_MSC_VECTOR_ALIGMENT 
class dgContact: public dgConstraint, public dgList<dgContactMaterial>
{
	public:
    dgFloat32 GetClosestDistance() const;
	dgFloat32 GetTimeOfImpact() const;
	void SetTimeOfImpact(dgFloat32 timetoImpact);
	const dgContactMaterial* GetMaterial() const;

	protected:
	dgContact(dgContact* const clone);
	dgContact(dgWorld* const world, const dgContactMaterial* const material);
	virtual ~dgContact();

	DG_CLASS_ALLOCATOR(allocator)

	virtual void ResetMaxDOF();
	virtual void GetInfo (dgConstraintInfo* const info) const;
	virtual dgUnsigned32 JacobianDerivative (dgContraintDescritor& params); 
	virtual void JointAccelerations (dgJointAccelerationDecriptor* const params); 
	
	virtual bool IsDeformable() const ;
	virtual void SetDestructorCallback (OnConstraintDestroy destructor);

	void JacobianContactDerivative (dgContraintDescritor& params, const dgContactMaterial& contact, dgInt32 normalIndex, dgInt32& frictionIndex); 
	void CalculatePointDerivative (dgInt32 index, dgContraintDescritor& desc, const dgVector& dir, const dgPointParam& param) const;

	void AppendToActiveList();
	void SwapBodies();

	dgVector m_positAcc;
	dgQuaternion m_rotationAcc;
	dgVector m_separtingVector;
	dgFloat32 m_closestDistance;
	dgFloat32 m_separationDistance;
	dgFloat32 m_timeOfImpact;
	dgWorld* m_world;
	const dgContactMaterial* m_material;
	dgActiveContacts::dgListNode* m_contactNode;
	dgUnsigned32 m_broadphaseLru;
	dgUnsigned32 m_isNewContact				: 1;

    friend class dgBody;
	friend class dgWorld;
	friend class dgBroadPhase;
	friend class dgContactSolver;
	friend class dgActiveContacts;
	friend class dgCollisionScene;
	friend class dgCollisionConvex;
	friend class dgCollisionCompound;
	friend class dgWorldDynamicUpdate;
	friend class dgSolverWorlkerThreads;
	friend class dgCollisionConvexPolygon;
	friend class dgCollidingPairCollector;
}DG_GCC_VECTOR_ALIGMENT;

inline void dgContactMaterial::SetCollisionCallback (OnAABBOverlap aabbOverlap, OnContactCallback contact) 
{
	m_aabbOverlap = aabbOverlap;
	m_processContactPoint = contact;
}

inline void dgContactMaterial::SetCollisionGenerationCallback (OnContactGeneration contactGeneration)
{
	m_contactGeneration = contactGeneration;
}

inline void dgContactMaterial::SetCompoundCollisionCallback (OnCompoundCollisionPrefilter aabbOverlap)
{
	m_compoundAABBOverlap = aabbOverlap;
}

inline void* dgContactMaterial::GetUserData () const
{
	return m_userData;
}

inline void dgContactMaterial::SetUserData (void* const userData)
{
	m_userData = userData;
}


inline bool dgContact::IsDeformable() const 
{
	return false;
}

inline void dgContact::SetDestructorCallback (OnConstraintDestroy destructor)
{
}

inline const dgContactMaterial* dgContact::GetMaterial() const
{
	return m_material;
}

inline void dgContact::SetTimeOfImpact(dgFloat32 timetoImpact)
{
	m_timeOfImpact = timetoImpact;
}

inline dgFloat32 dgContact::GetTimeOfImpact() const
{
	return m_timeOfImpact;
}

inline dgFloat32 dgContact::GetClosestDistance() const
{
    return m_closestDistance;
}

inline void dgContact::ResetMaxDOF()
{
	m_maxDOF = 0;
}


#endif 

