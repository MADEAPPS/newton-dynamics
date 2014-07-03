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

#ifndef __AFX_BROADPHASE_H_
#define __AFX_BROADPHASE_H_

#include "dgPhysicsStdafx.h"


class dgBody;
class dgWorld;
class dgContact;
class dgCollision;
class dgCollisionInstance;
class dgBroadphaseSyncDescriptor;

typedef dgInt32 (dgApi *OnBodiesInAABB) (dgBody* body, void* const userData);
typedef dgUnsigned32 (dgApi *OnRayPrecastAction) (const dgBody* const body, const dgCollisionInstance* const collision, void* const userData);
typedef dgFloat32 (dgApi *OnRayCastAction) (const dgBody* const body, const dgCollisionInstance* const collision, const dgVector& contact, const dgVector& normal, dgInt64 collisionID, void* const userData, dgFloat32 intersetParam);


#define DG_CACHE_DIST_TOL			dgFloat32 (1.0e-3f)

DG_MSC_VECTOR_ALIGMENT
struct dgLineBox
{
	dgVector m_l0;
	dgVector m_l1;
	dgVector m_boxL0;
	dgVector m_boxL1;
} DG_GCC_VECTOR_ALIGMENT;


class dgConvexCastReturnInfo
{
	public:
	dgFloat32 m_point[4];					// collision point in global space
	dgFloat32 m_normal[4];					// surface normal at collision point in global space
	//dgFloat32 m_normalOnHitPoint[4];		// surface normal at the surface of the hit body, 
											// is the same as the normal calculate by a raycast passing by the hit point in the direction of the cast
	dgInt64  m_contaID;	                // collision ID at contact point
	const dgBody* m_hitBody;				// body hit at contact point
	dgFloat32 m_penetration;                // contact penetration at collision point
};


class dgBroadPhase
{
	public:
	DG_CLASS_ALLOCATOR(allocator);

	enum dgType
	{
		m_generic = 0,
		m_persistent,
	};

	class dgNode;
	class dgSpliteInfo;

	dgBroadPhase(dgWorld* const world);
	virtual ~dgBroadPhase();

    dgUnsigned32 GetLRU () const;
	void GetWorldSize (dgVector& p0, dgVector& p1) const;
	void RayCast (const dgVector& p0, const dgVector& p1, OnRayCastAction filter, OnRayPrecastAction prefilter, void* const userData) const;
	void ConvexRayCast (dgCollisionInstance* const shape, const dgMatrix& matrx, const dgVector& p1, OnRayCastAction filter, OnRayPrecastAction prefilter, void* const userData, dgInt32 threadId) const;

	dgInt32 ConvexCast (dgCollisionInstance* const shape, const dgMatrix& p0, const dgVector& p1, dgFloat32& timetoImpact, OnRayPrecastAction prefilter, void* const userData, dgConvexCastReturnInfo* const info, dgInt32 maxContacts, dgInt32 threadIndex) const;
	void ForEachBodyInAABB (const dgVector& q0, const dgVector& q1, OnBodiesInAABB callback, void* const userData) const;

	dgInt32 GetBroadPhaseType () const;
	void SelectBroadPhaseType (dgInt32 algorthmType);

	void ResetEntropy ();

	protected:
	class dgFitnessList: public dgList <dgNode*>
	{
		public:
		dgFitnessList (dgMemoryAllocator* const allocator);
		dgFloat64 TotalCost () const;
	};

	void Add (dgBody* const body);
	void Remove (dgBody* const body);
	void InvalidateCache ();
	void UpdateContacts (dgFloat32 timestep);
	void AddInternallyGeneratedBody(dgBody* const body);
	void UpdateBodyBroadphase(dgBody* const body, dgInt32 threadIndex);

	void ImproveFitness();
	void ImproveNodeFitness (dgNode* const node);
	dgNode* InsertNode (dgNode* const node);
	dgFloat32 CalculateSurfaceArea (const dgNode* const node0, const dgNode* const node1, dgVector& minBox, dgVector& maxBox) const;

	void AddPair (dgBody* const body0, dgBody* const body1, const dgVector& timestep2, dgInt32 threadID);

	static void ForceAndToqueKernel (void* const descriptor, void* const worldContext, dgInt32 threadID);
	static void CollidingPairsKernel (void* const descriptor, void* const worldContext, dgInt32 threadID);
	static void UpdateContactsKernel (void* const descriptor, void* const worldContext, dgInt32 threadID);
//	static void UpdateSoftBodyForcesKernel (void* const descriptor, void* const worldContext, dgInt32 threadID);
	static void AddGeneratedBodyesContactsKernel (void* const descriptor, void* const worldContext, dgInt32 threadID);
	
	void UpdateContactsBroadPhaseEnd ();
	void ApplyForceAndtorque (dgBroadphaseSyncDescriptor* const desctiptor, dgInt32 threadID);
	void ApplyDeformableForceAndtorque (dgBroadphaseSyncDescriptor* const desctiptor, dgInt32 threadID);
	void CalculatePairContacts (dgBroadphaseSyncDescriptor* const descriptor, dgInt32 threadID);
//	void UpdateSoftBodyForcesKernel (dgBroadphaseSyncDescriptor* const descriptor, dgInt32 threadID);
	
	dgNode* BuildTopDown (dgNode** const leafArray, dgInt32 firstBox, dgInt32 lastBox, dgFitnessList::dgListNode** const nextNode);

	void FindCollidingPairsGeneric (dgBroadphaseSyncDescriptor* const desctiptor, dgInt32 threadID);
	void FindCollidingPairsPersistent (dgBroadphaseSyncDescriptor* const desctiptor, dgInt32 threadID);
	void SubmitPairsPersistent (dgNode* const body, dgNode* const node, const dgVector& timeStepBound, dgInt32 threadID);

	void FindGeneratedBodiesCollidingPairs (dgBroadphaseSyncDescriptor* const desctiptor, dgInt32 threadID);

	void KinematicBodyActivation (dgContact* const contatJoint) const;

	bool TestOverlaping (const dgBody* const body0, const dgBody* const body1) const;
	dgFloat64 CalculateEmptropy ();

	dgWorld* m_world;
	dgNode* m_rootNode;
	dgFloat64 m_treeEntropy;
	dgUnsigned32 m_lru;
	dgFitnessList m_fitness;
	dgList<dgBody*> m_generatedBodies;
	dgType m_broadPhaseType;
	dgThread::dgCriticalSection m_contacJointLock;
	dgThread::dgCriticalSection m_criticalSectionLock;
	bool m_recursiveChunks;

	static dgVector m_conservativeRotAngle;
	friend class dgBody;
	friend class dgWorld;
	friend class dgWorldDynamicUpdate;
	friend class dgCollisionCompoundFractured;
};
#endif
