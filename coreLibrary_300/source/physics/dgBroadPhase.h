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
#include "dgBody.h"
#include "dgBodyMasterList.h"

class dgBody;
class dgWorld;
class dgContact;
class dgCollision;
class dgCollisionInstance;


#define DG_CACHE_DIST_TOL				dgFloat32 (1.0e-3f)
#define DG_BROADPHASE_MAX_STACK_DEPTH	256

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

DG_MSC_VECTOR_ALIGMENT
class dgBroadPhaseNode
{
	public:
	DG_CLASS_ALLOCATOR(allocator)

	dgBroadPhaseNode()
		:m_minBox(dgFloat32(-1.0e15f))
		,m_maxBox(dgFloat32(1.0e15f))
		,m_surfaceArea(dgFloat32(1.0e20f))
		,m_body(NULL)
		,m_left(NULL)
		,m_right(NULL)
		,m_parent(NULL)
		,m_fitnessNode(NULL)
	{
	}

	dgBroadPhaseNode(dgBody* const body)
		:m_minBox(body->m_minAABB)
		,m_maxBox(body->m_maxAABB)
		,m_body(body)
		,m_left(NULL)
		,m_right(NULL)
		,m_parent(NULL)
		,m_fitnessNode(NULL)
	{
		SetAABB(body->m_minAABB, body->m_maxAABB);
		m_body->SetBroadPhase (this);
	}

	dgBroadPhaseNode(dgBroadPhaseNode* const sibling, dgBroadPhaseNode* const myNode)
		:m_body(NULL)
		,m_left(sibling)
		,m_right(myNode)
		,m_parent(sibling->m_parent)
		,m_fitnessNode(NULL)
	{
		if (m_parent) {
			if (m_parent->m_left == sibling) {
				m_parent->m_left = this;
			} else {
				dgAssert(m_parent->m_right == sibling);
				m_parent->m_right = this;
			}
		}
		sibling->m_parent = this;
		myNode->m_parent = this;

		dgBroadPhaseNode* const left = m_left;
		dgBroadPhaseNode* const right = m_right;

		m_minBox = left->m_minBox.GetMin(right->m_minBox);
		m_maxBox = left->m_maxBox.GetMax(right->m_maxBox);
		dgVector side0(m_maxBox - m_minBox);
		m_surfaceArea = side0.DotProduct4(side0.ShiftTripleRight()).m_x;
	}

	dgBroadPhaseNode(dgBroadPhaseNode* const parent, const dgVector& minBox, const dgVector& maxBox)
		:m_minBox(minBox)
		,m_maxBox(maxBox)
		,m_body(NULL)
		,m_left(NULL)
		,m_right(NULL)
		,m_parent(parent)
		,m_fitnessNode(NULL)
	{
	}

	~dgBroadPhaseNode()
	{
		if (m_body) {
			dgAssert(!m_left);
			dgAssert(!m_right);
			dgAssert(m_body->GetBroadPhase() == this);
			m_body->SetBroadPhase(NULL);
		}
		else {
			if (m_left) {
				delete m_left;
			}
			if (m_right) {
				delete m_right;
			}
		}
	}

	void SetAABB(const dgVector& minBox, const dgVector& maxBox)
	{
		dgAssert(minBox.m_x <= maxBox.m_x);
		dgAssert(minBox.m_y <= maxBox.m_y);
		dgAssert(minBox.m_z <= maxBox.m_z);

		dgVector p0(minBox.CompProduct4(m_broadPhaseScale));
		dgVector p1(maxBox.CompProduct4(m_broadPhaseScale) + dgVector::m_one);

		m_minBox = p0.Floor().CompProduct4(m_broadInvPhaseScale);
		m_maxBox = p1.Floor().CompProduct4(m_broadInvPhaseScale);

		dgAssert(m_minBox.m_w == dgFloat32(0.0f));
		dgAssert(m_maxBox.m_w == dgFloat32(0.0f));

		dgVector side0(m_maxBox - m_minBox);
		m_surfaceArea = side0.DotProduct4(side0.ShiftTripleRight()).m_x;
	}

	dgVector m_minBox;
	dgVector m_maxBox;
	dgFloat32 m_surfaceArea;
	dgBody* m_body;
	dgBroadPhaseNode* m_left;
	dgBroadPhaseNode* m_right;
	dgBroadPhaseNode* m_parent;
	dgList<dgBroadPhaseNode*>::dgListNode* m_fitnessNode;
	static dgVector m_broadPhaseScale;
	static dgVector m_broadInvPhaseScale;

	friend class dgBody;
	friend class dgBroadPhaseDefault;
	friend class dgFitnessList;
} DG_GCC_VECTOR_ALIGMENT;


class dgBroadPhase
{
	protected:
	class dgSpliteInfo;
	class dgBroadphaseSyncDescriptor
	{
		public:
		dgBroadphaseSyncDescriptor(dgFloat32 timestep, dgWorld* const world)
			:m_world(world)
			, m_newBodiesNodes(NULL)
			, m_timestep(timestep)
			, m_pairsAtomicCounter(0)
		{
		}

		dgWorld* m_world;
		dgList<dgBody*>::dgListNode* m_newBodiesNodes;
		dgFloat32 m_timestep;
		dgInt32 m_pairsAtomicCounter;
	};
	
	class dgFitnessList: public dgList <dgBroadPhaseNode*>
	{
		public:
		dgFitnessList(dgMemoryAllocator* const allocator)
			:dgList <dgBroadPhaseNode*>(allocator)
		{
		}

		dgFloat64 TotalCost() const
		{
			dgFloat64 cost = dgFloat32(0.0f);
			for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
				dgBroadPhaseNode* const box = node->GetInfo();
				cost += box->m_surfaceArea;
			}
			return cost;
		}
	};

	public:
	enum dgContactCode
	{
		m_close,
		m_persist,
		m_separated,
	};

	class dgPair
	{
    	public:
		dgContact* m_contact;
		dgContactPoint* m_contactBuffer;
		dgFloat32 m_timeOfImpact;
		dgInt32 m_contactCount : 16;
		dgInt32 m_isDeformable : 1;
		dgInt32 m_cacheIsValid : 1;
	};

	dgBroadPhase(dgWorld* const world);
	virtual ~dgBroadPhase();

	DG_INLINE dgUnsigned32 GetLRU() const
	{
		return m_lru;
	}

	DG_INLINE dgFloat32 CalculateSurfaceArea(const dgBroadPhaseNode* const node0, const dgBroadPhaseNode* const node1, dgVector& minBox, dgVector& maxBox) const
	{
		minBox = node0->m_minBox.GetMin(node1->m_minBox);
		maxBox = node0->m_maxBox.GetMax(node1->m_maxBox);
		dgVector side0(maxBox - minBox);
		return side0.DotProduct4(side0.ShiftTripleRight()).GetScalar();
	}

	virtual dgInt32 GetType() const = 0;
	
	virtual void Add(dgBody* const body) = 0;
	virtual void Remove(dgBody* const body) = 0;
	virtual void ResetEntropy() = 0;
	virtual void InvalidateCache() = 0;
	virtual void UpdateFitness() = 0;

	virtual void CheckStaticDynamic(dgBody* const body, dgFloat32 mass) = 0;
	virtual void ForEachBodyInAABB (const dgVector& minBox, const dgVector& maxBox, OnBodiesInAABB callback, void* const userData) const = 0;
	virtual void RayCast (const dgVector& p0, const dgVector& p1, OnRayCastAction filter, OnRayPrecastAction prefilter, void* const userData) const = 0;
	virtual void ConvexRayCast (dgCollisionInstance* const shape, const dgMatrix& matrix, const dgVector& target, OnRayCastAction filter, OnRayPrecastAction prefilter, void* const userData, dgInt32 threadId) const = 0;
	virtual dgInt32 ConvexCast (dgCollisionInstance* const shape, const dgMatrix& matrix, const dgVector& target, dgFloat32& timeToImpact, OnRayPrecastAction prefilter, void* const userData, dgConvexCastReturnInfo* const info, dgInt32 maxContacts, dgInt32 threadIndex) const = 0;

	virtual void ScanForContactJoints(dgBroadphaseSyncDescriptor& syncPoints) = 0;
	virtual void FindCollidingPairs (dgBroadphaseSyncDescriptor* const descriptor, dgBodyMasterList::dgListNode* node, dgInt32 threadID) = 0;

	void UpdateBody(dgBody* const body, dgInt32 threadIndex);
	void AddInternallyGeneratedBody(dgBody* const body)
	{
		m_generatedBodies.Append(body);
	}
	
	void UpdateContacts(dgFloat32 timestep);
	void CollisionChange (dgBody* const body, dgCollisionInstance* const collisionSrc);

	protected:
	bool DoNeedUpdate(dgBodyMasterList::dgListNode* const node) const;
	dgFloat64 CalculateEntropy (dgFitnessList& fitness, dgBroadPhaseNode** const root);
	dgBroadPhaseNode* InsertNode (dgBroadPhaseNode* const root, dgBroadPhaseNode* const node);

	void RotateLeft(dgBroadPhaseNode* const node, dgBroadPhaseNode** const root);
	void RotateRight(dgBroadPhaseNode* const node, dgBroadPhaseNode** const root);
	void ImproveNodeFitness(dgBroadPhaseNode* const node, dgBroadPhaseNode** const root);
	void ImproveFitness(dgFitnessList& fitness, dgFloat64& oldEntropy, dgBroadPhaseNode** const root);

	bool ValidateContactCache(dgContact* const contact, dgFloat32 timestep) const;
    void CalculatePairContacts (dgPair* const pair, dgFloat32 timestep, dgInt32 threadID);
    void AddPair (dgContact* const contact, dgFloat32 timestep, dgInt32 threadIndex);
	void AddPair (dgBody* const body0, dgBody* const body1, dgFloat32 timestep, dgInt32 threadID);	
	bool TestOverlaping (const dgBody* const body0, const dgBody* const body1, dgFloat32 timestep) const;

	void ForEachBodyInAABB (const dgBroadPhaseNode** stackPool, dgInt32 stack, const dgVector& minBox, const dgVector& maxBox, OnBodiesInAABB callback, void* const userData) const;
	void RayCast (const dgBroadPhaseNode** stackPool, dgFloat32* const distance, dgInt32 stack, const dgVector& l0, const dgVector& l1, dgFastRayTest& ray, OnRayCastAction filter, OnRayPrecastAction prefilter, void* const userData) const;

	void ConvexRayCast (const dgBroadPhaseNode** stackPool, dgFloat32* const distance, dgInt32 stack, const dgVector& velocA, dgFastRayTest& ray,  
					 	dgCollisionInstance* const shape, const dgMatrix& matrix, const dgVector& target, OnRayCastAction filter, OnRayPrecastAction prefilter, void* const userData, dgInt32 threadId) const;

	dgInt32 ConvexCast (const dgBroadPhaseNode** stackPool, dgFloat32* const distance, dgInt32 stack, const dgVector& velocA, const dgVector& velocB, dgFastRayTest& ray,  
						dgCollisionInstance* const shape, const dgMatrix& matrix, const dgVector& target, dgFloat32& timeToImpact, OnRayPrecastAction prefilter, void* const userData, dgConvexCastReturnInfo* const info, dgInt32 maxContacts, dgInt32 threadIndex) const;

	void ApplyForceAndtorque (dgBroadphaseSyncDescriptor* const descriptor, dgBodyMasterList::dgListNode* node, dgInt32 threadID);

	dgBroadPhaseNode* BuildTopDown(dgBroadPhaseNode** const leafArray, dgInt32 firstBox, dgInt32 lastBox, dgFitnessList::dgListNode** const nextNode);
	dgBroadPhaseNode* BuildTopDownBig(dgBroadPhaseNode** const leafArray, dgInt32 firstBox, dgInt32 lastBox, dgFitnessList::dgListNode** const nextNode);

	void UpdateContactsBroadPhaseEnd ();
	void KinematicBodyActivation (dgContact* const contatJoint) const;
	void SubmitPairs (dgBroadPhaseNode* const body, dgBroadPhaseNode* const node, dgFloat32 timestep, dgInt32 threadID);
	void FindGeneratedBodiesCollidingPairs (dgBroadphaseSyncDescriptor* const descriptor, dgInt32 threadID);
	
	static void CollidingPairsKernel(void* const descriptor, void* const worldContext, dgInt32 threadID);
	static void ForceAndToqueKernel(void* const descriptor, void* const worldContext, dgInt32 threadID);
	static void AddGeneratedBodiesContactsKernel (void* const descriptor, void* const worldContext, dgInt32 threadID);
	static dgInt32 CompareNodes(const dgBroadPhaseNode* const nodeA, const dgBroadPhaseNode* const nodeB, void* const notUsed);

	dgWorld* m_world;
	dgBroadPhaseNode* m_rootNode;
	dgList<dgBody*> m_generatedBodies;
	dgUnsigned32 m_lru;
	dgThread::dgCriticalSection m_contacJointLock;
	dgThread::dgCriticalSection m_criticalSectionLock;
	bool m_recursiveChunks;

	static dgVector m_velocTol;
	static dgVector m_linearContactError2;
	static dgVector m_angularContactError2;

	friend class dgBody;
	friend class dgWorld;
	friend class dgWorldDynamicUpdate;
	friend class dgCollisionCompoundFractured;
};


#endif
