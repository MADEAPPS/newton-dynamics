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

#ifndef __D_BROADPHASE_H__
#define __D_BROADPHASE_H__

#include "dNewtonStdafx.h"
#include "dBroadPhaseNode.h"
//#include "dgBody.h"
//#include "dgBodyMasterList.h"

//class dBody;
class dNewton;
//class dContact;
//class dCollision;
//class dDynamicBody;
//class dCollisionInstance;
//class dBroadPhaseAggregate;

#if 0
//#define D_CACHE_DIST_TOL				dFloat32 (1.0e-3f)
//#define D_BROADPHASE_MAX_STACK_DEPTH	256

//class dgConvexCastReturnInfo
//{
//	public:
//	dFloat32 m_point[4];					// collision point in global space
//	dFloat32 m_normal[4];					// surface normal at collision point in global space
//	//dFloat32 m_normalOnHitPoint[4];		// surface normal at the surface of the hit body, 
//											// is the same as the normal calculate by a raycast passing by the hit point in the direction of the cast
//	dgInt64  m_contaID;	                // collision ID at contact point
//	const dgBody* m_hitBody;				// body hit at contact point
//	dFloat32 m_penetration;                // contact penetration at collision point
//};

//D_MSC_VECTOR_ALIGNMENT
//class dBroadPhaseNode
//{
//	public:
//	dBroadPhaseNode(dBroadPhaseNode* const parent)
//		:m_minBox(dFloat32(-1.0e15f))
//		,m_maxBox(dFloat32(1.0e15f))
//		,m_parent(parent)
//		,m_surfaceArea(dFloat32(1.0e20f))
//		,m_criticalSectionLock(0)
//	{
//	}
//
//	virtual ~dBroadPhaseNode()
//	{
//	}
//
//	virtual bool IsSegregatedRoot() const
//	{
//		return false;
//	}
//
//	virtual bool IsLeafNode() const
//	{
//		return false;
//	}
//
//	virtual bool IsAggregate() const
//	{
//		return false;
//	}
//
//	void SetAABB(const dVector& minBox, const dVector& maxBox)
//	{
//		dAssert(minBox.m_x <= maxBox.m_x);
//		dAssert(minBox.m_y <= maxBox.m_y);
//		dAssert(minBox.m_z <= maxBox.m_z);
//
//		dVector p0(minBox * m_broadPhaseScale);
//		dVector p1(maxBox * m_broadPhaseScale + dVector::m_one);
//
//		m_minBox = p0.Floor() * m_broadInvPhaseScale;
//		m_maxBox = p1.Floor() * m_broadInvPhaseScale;
//
//		dAssert(m_minBox.m_w == dFloat32(0.0f));
//		dAssert(m_maxBox.m_w == dFloat32(0.0f));
//
//		dVector side0(m_maxBox - m_minBox);
//		m_surfaceArea = side0.DotProduct(side0.ShiftTripleRight()).m_x;
//	}
//
//	virtual dgBody* GetBody() const
//	{
//		return nullptr;
//	}
//
//	virtual dBroadPhaseNode* GetLeft() const
//	{
//		return nullptr;
//	}
//
//	virtual dBroadPhaseNode* GetRight() const
//	{
//		return nullptr;
//	}
//
//	dVector m_minBox;
//	dVector m_maxBox;
//	dBroadPhaseNode* m_parent;
//	dFloat32 m_surfaceArea;
//	dInt32 m_criticalSectionLock;
//
//	static dVector m_broadPhaseScale;
//	static dVector m_broadInvPhaseScale;
//} D_GCC_VECTOR_ALIGNMENT;

//class dBroadPhaseBodyNode: public dBroadPhaseNode
//{
//	public:
//	dBroadPhaseBodyNode(dgBody* const body)
//		:dBroadPhaseNode(nullptr)
//		,m_body(body)
//		,m_updateNode(nullptr)
//	{
//		SetAABB(body->m_minAABB, body->m_maxAABB);
//		m_body->SetBroadPhase(this);
//	}
//
//	virtual ~dBroadPhaseBodyNode()
//	{
//		m_body->SetBroadPhase(nullptr);
//	}
//
//	virtual bool IsLeafNode() const
//	{
//		return true;
//	}
//
//	virtual dgBody* GetBody() const
//	{
//		return m_body;
//	}
//
//	dgBody* m_body;
//	dList<dBroadPhaseNode*>::dListNode* m_updateNode;
//};

//class dBroadPhaseTreeNode: public dBroadPhaseNode
//{
//	public:
//	dBroadPhaseTreeNode()
//		:dBroadPhaseNode(nullptr)
//		,m_left(nullptr)
//		,m_right(nullptr)
//		,m_fitnessNode(nullptr)
//	{
//	}
//
//	dBroadPhaseTreeNode(dBroadPhaseNode* const sibling, dBroadPhaseNode* const myNode)
//		:dBroadPhaseNode(sibling->m_parent)
//		,m_left(sibling)
//		,m_right(myNode)
//		,m_fitnessNode(nullptr)
//	{
//		if (m_parent) {
//			dBroadPhaseTreeNode* const myParent = (dBroadPhaseTreeNode*)m_parent;
//			if (myParent->m_left == sibling) {
//				myParent->m_left = this;
//			} else {
//				dAssert(myParent->m_right == sibling);
//				myParent->m_right = this;
//			}
//		}
//
//		sibling->m_parent = this;
//		myNode->m_parent = this;
//
//		dBroadPhaseNode* const left = m_left;
//		dBroadPhaseNode* const right = m_right;
//
//		m_minBox = left->m_minBox.GetMin(right->m_minBox);
//		m_maxBox = left->m_maxBox.GetMax(right->m_maxBox);
//		dVector side0(m_maxBox - m_minBox);
//		m_surfaceArea = side0.DotProduct(side0.ShiftTripleRight()).m_x;
//	}
//
//	virtual ~dBroadPhaseTreeNode()
//	{
//		if (m_left) {
//			delete m_left;
//		}
//		if (m_right) {
//			delete m_right;
//		}
//	}
//	
//	virtual dBroadPhaseNode* GetLeft() const
//	{
//		return m_left;
//	}
//
//	virtual dBroadPhaseNode* GetRight() const
//	{
//		return m_right;
//	}
//
//	dBroadPhaseNode* m_left;
//	dBroadPhaseNode* m_right;
//	dList<dBroadPhaseTreeNode*>::dListNode* m_fitnessNode;
//} D_GCC_VECTOR_ALIGNMENT;

//#define D_CONTACT_CACHE_LINE_SIZE 4

D_MSC_VECTOR_ALIGNMENT
class dBroadPhase: public dClassAlloc
{
	protected:
	//class CacheEntryTag
	//{
	//	public:
	//	D_INLINE CacheEntryTag() {}
	//	D_INLINE CacheEntryTag(dUnsigned32 tag0, dUnsigned32 tag1)
	//		:m_tagLow(dMin(tag0, tag1))
	//		,m_tagHigh(dMax(tag0, tag1))
	//	{
	//	}
	//
	//	D_INLINE dUnsigned32 GetHash() const
	//	{
	//		return m_tagHigh * 31415821u + m_tagLow;
	//	}
	//
	//	union
	//	{
	//		dUnsigned64 m_tag;
	//		struct 
	//		{
	//			dUnsigned32 m_tagLow;
	//			dUnsigned32 m_tagHigh;
	//		};
	//	};
	//};
	//
	//class dContactCacheLine
	//{
	//	public:
	//	D_INLINE dContactCacheLine()
	//	{
	//	}
	//
	//	dInt32 m_count;
	//	dUnsigned32 m_key;
	//	dContact* m_contact[D_CONTACT_CACHE_LINE_SIZE];
	//	dInt32 m_hashKey[D_CONTACT_CACHE_LINE_SIZE];
	//	CacheEntryTag m_tags[D_CONTACT_CACHE_LINE_SIZE];
	//};
	//
	//class dContactCache: public dArray<dContactCacheLine>
	//{
	//	public:
	//	dContactCache (dgMemoryAllocator* const allocator)
	//		:dArray<dContactCacheLine>(allocator)
	//		,m_count(1<<10)
	//	{
	//		Init();
	//	}
	//
	//	void Flush()
	//	{
	//		Clear();
	//		Init();
	//	}
	//
	//	void Init()
	//	{
	//		m_count = 1 << 10;
	//		ResizeIfNecessary(m_count);
	//		dContactCacheLine* const cache = &(*this)[0];
	//		memset(cache, 0, m_count * sizeof(dContactCacheLine));
	//	}
	//
	//	D_INLINE dContact* FindContactJoint(const dgBody* const body0, const dgBody* const body1) const
	//	{
	//		CacheEntryTag tag(body0->m_uniqueID, body1->m_uniqueID);
	//		dUnsigned32 hash = tag.GetHash();
	//
	//		dInt32 entry = hash & (m_count - 1);
	//
	//		const dContactCacheLine& cacheLine = (*this)[entry];
	//		for (dInt32 i = 0; i < cacheLine.m_count; i++) {
	//			if (cacheLine.m_tags[i].m_tag == tag.m_tag) {
	//				return cacheLine.m_contact[i];
	//			}
	//		}
	//		return nullptr;
	//	}
	//
	//	D_INLINE bool AddContactJoint(dContact* const joint)
	//	{
	//		// note this function is not thread safe
	//		CacheEntryTag tag(joint->GetBody0()->m_uniqueID, joint->GetBody1()->m_uniqueID);
	//		dUnsigned32 hash = tag.GetHash();
	//
	//		dInt32 entry = hash & (m_count - 1);
	//		dContactCacheLine* cacheLine = &(*this)[entry];
	//
	//		for (dInt32 i = cacheLine->m_count - 1; i >= 0; i--) {
	//			if (cacheLine->m_tags[i].m_tag == tag.m_tag) {
	//				return false;
	//			}
	//		}
	//
	//		while (cacheLine->m_count == 4) {
	//			Rehash();
	//			entry = hash & (m_count - 1);
	//			cacheLine = &(*this)[entry];
	//		}
	//		if (cacheLine->m_count == 0) {
	//			cacheLine->m_key = hash;
	//		}
	//
	//		const dInt32 index = cacheLine->m_count;
	//		cacheLine->m_count++;
	//		cacheLine->m_tags[index] = tag;
	//		cacheLine->m_hashKey[index] = hash;
	//		cacheLine->m_contact[index] = joint;
	//		return true;
	//	}
	//
	//	D_INLINE void RemoveContactJoint(dContact* const joint)
	//	{
	//		CacheEntryTag tag(joint->GetBody0()->m_uniqueID, joint->GetBody1()->m_uniqueID);
	//		dUnsigned32 hash = tag.GetHash();
	//
	//		dInt32 entry = hash & (m_count - 1);
	//		dContactCacheLine* const cacheLine = &(*this)[entry];
	//		for (dInt32 i = cacheLine->m_count - 1; i >= 0 ; i--) {
	//			if (cacheLine->m_tags[i].m_tag == tag.m_tag) {
	//				cacheLine->m_count--;
	//				const dInt32 index = cacheLine->m_count;
	//				cacheLine->m_tags[i] = cacheLine->m_tags[index];
	//				cacheLine->m_hashKey[i] = cacheLine->m_hashKey[index];
	//				cacheLine->m_contact[i] = cacheLine->m_contact[index];
	//				break;
	//			}
	//		}
	//	}
	//
	//	private:
	//	void Rehash()
	//	{
	//		const dInt32 newCount = m_count * 2;
	//		ResizeIfNecessary(newCount);
	//		dContactCacheLine* const cache0 = &(*this)[0];
	//		dContactCacheLine* const cache1 = &cache0[m_count];
	//
	//		const dInt32 mask = newCount - 1;
	//		for (dInt32 i = 0; i < m_count; i++) {
	//			dContactCacheLine* const src = &cache0[i];
	//			dContactCacheLine* const dst = &cache1[i];
	//			dst->m_count = 0;
	//			for (dInt32 j = src->m_count - 1; j >= 0; j--) {
	//				dInt32 entry = src->m_hashKey[j] & mask;
	//				if (entry >= m_count) {
	//					const dInt32 dstIndex = dst->m_count;
	//					dst->m_count++;
	//					dst->m_tags[dstIndex] = src->m_tags[j];
	//					dst->m_hashKey[dstIndex] = src->m_hashKey[j];
	//					dst->m_contact[dstIndex] = src->m_contact[j];
	//
	//					src->m_count--;
	//					const dInt32 srcIndex = src->m_count;
	//					src->m_tags[j] = src->m_tags[srcIndex];
	//					src->m_hashKey[j] = src->m_hashKey[srcIndex];
	//					src->m_contact[j] = src->m_contact[srcIndex];
	//				}
	//			}
	//		}
	//		m_count = newCount;
	//	}
	//
	//	dInt32 m_count;
	//};

	//class dSpliteInfo;
	//class dBroadphaseSyncDescriptor
	//{
	//	public:
	//	dBroadphaseSyncDescriptor(dFloat32 timestep, dgWorld* const world)
	//		:m_world(world)
	//		,m_timestep(timestep)
	//		,m_atomicIndex(0)
	//		,m_contactStart(0)
	//		,m_atomicDynamicsCount(0)
	//		,m_atomicPendingBodiesCount(0)
	//		,m_fullScan(false)
	//	{
	//	}
	//
	//	dgWorld* m_world;
	//	dFloat32 m_timestep;
	//	dInt32 m_atomicIndex;
	//	dInt32 m_contactStart;
	//	dInt32 m_atomicDynamicsCount;
	//	dInt32 m_atomicPendingBodiesCount;
	//	bool m_fullScan;
	//};
	
	//class dFitnessList: public dList <dBroadPhaseTreeNode*>
	//{
	//	public:
	//	dFitnessList(dgMemoryAllocator* const allocator)
	//		:dList <dBroadPhaseTreeNode*>(allocator)
	//		,m_index(0)
	//		,m_prevCost(dFloat32 (0.0f))
	//	{
	//	}
	//
	//	dFloat64 TotalCost() const
	//	{
	//		dFloat64 cost = dFloat32(0.0f);
	//		for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
	//			dBroadPhaseNode* const box = node->GetInfo();
	//			cost += box->m_surfaceArea;
	//		}
	//		return cost;
	//	}
	//
	//	dInt32 m_index;
	//	dFloat64 m_prevCost;
	//};

	public:
	//enum dContactCode
	//{
	//	m_close,
	//	m_persist,
	//	m_separated,
	//};
	//
	//class dgPair
	//{
    //	public:
	//	dContact* m_contact;
	//	dContactPoint* m_contactBuffer;
	//	dFloat32 m_timestep;
	//	dInt32 m_contactCount : 16;
	//	dInt32 m_cacheIsValid : 1;
	//	dInt32 m_flipContacts : 1;
	//};

	dBroadPhase(dNewton* const world);
	virtual ~dBroadPhase();

	//D_INLINE dUnsigned32 GetLRU() const
	//{
	//	return m_lru;
	//}
	//
	//D_INLINE dFloat32 CalculateSurfaceArea(const dBroadPhaseNode* const node0, const dBroadPhaseNode* const node1, dVector& minBox, dVector& maxBox) const
	//{
	//	minBox = node0->m_minBox.GetMin(node1->m_minBox);
	//	maxBox = node0->m_maxBox.GetMax(node1->m_maxBox);
	//	dVector side0(maxBox - minBox);
	//	return side0.DotProduct(side0.ShiftTripleRight()).GetScalar();
	//}
	//
	//dgWorld* GetWorld() const { return m_world;}
	//
	//virtual dInt32 GetType() const = 0;
	//
	//virtual void Add(dgBody* const body) = 0;
	//virtual void Remove(dgBody* const body) = 0;
	//
	//virtual void ResetEntropy() = 0;
	//virtual void UpdateFitness() = 0;
	//virtual void InvalidateCache() = 0;
	//virtual dBroadPhaseAggregate* CreateAggregate() = 0;
	//virtual void DestroyAggregate(dBroadPhaseAggregate* const aggregate) = 0;
	//
	//virtual void CheckStaticDynamic(dgBody* const body, dFloat32 mass) = 0;
	//virtual void ForEachBodyInAABB (const dVector& minBox, const dVector& maxBox, OnBodiesInAABB callback, void* const userData) const = 0;
	//virtual void RayCast (const dVector& p0, const dVector& p1, OnRayCastAction filter, OnRayPrecastAction prefilter, void* const userData) const = 0;
	//virtual dInt32 Collide(dgCollisionInstance* const shape, const dgMatrix& matrix, OnRayPrecastAction prefilter, void* const userData, dgConvexCastReturnInfo* const info, dInt32 maxContacts, dInt32 threadIndex) const = 0;
	//virtual dInt32 ConvexCast (dgCollisionInstance* const shape, const dgMatrix& matrix, const dVector& target, dFloat32* const param, OnRayPrecastAction prefilter, void* const userData, dgConvexCastReturnInfo* const info, dInt32 maxContacts, dInt32 threadIndex) const = 0;
	//virtual void FindCollidingPairs (dBroadphaseSyncDescriptor* const descriptor, dList<dBroadPhaseNode*>::dListNode* const node, dInt32 threadID) = 0;
	//
	//void UpdateBody(dgBody* const body, dInt32 threadIndex);
	//void AddInternallyGeneratedBody(dgBody* const body)
	//{
	//	m_generatedBodies.Append(body);
	//}
	//
	//void UpdateContacts(dFloat32 timestep);
	//void CollisionChange (dgBody* const body, dgCollisionInstance* const collisionSrc);
	//
	//void MoveNodes (dBroadPhase* const dest);
	//
	//protected:
	//virtual void LinkAggregate (dBroadPhaseAggregate* const aggregate) = 0; 
	//virtual void UnlinkAggregate (dBroadPhaseAggregate* const aggregate) = 0; 
	//
	//bool DoNeedUpdate(dgBodyMasterList::dListNode* const node) const;
	//dFloat64 CalculateEntropy (dFitnessList& fitness, dBroadPhaseNode** const root);
	//dBroadPhaseTreeNode* InsertNode (dBroadPhaseNode* const root, dBroadPhaseNode* const node);
	//
	//void RotateLeft(dBroadPhaseTreeNode* const node, dBroadPhaseNode** const root);
	//void RotateRight(dBroadPhaseTreeNode* const node, dBroadPhaseNode** const root);
	//void ImproveNodeFitness(dBroadPhaseTreeNode* const node, dBroadPhaseNode** const root);
	//void ImproveFitness(dFitnessList& fitness, dFloat64& oldEntropy, dBroadPhaseNode** const root);
	//
	//void CalculatePairContacts (dgPair* const pair, dInt32 threadID);
	//void AddPair (dContact* const contact, dFloat32 timestep, dInt32 threadIndex);
	//void AddPair (dgBody* const body0, dgBody* const body1, dFloat32 timestep, dInt32 threadID);	
	//
	//bool TestOverlaping(const dgBody* const body0, const dgBody* const body1, dFloat32 timestep) const;
	//
	//void ForEachBodyInAABB (const dBroadPhaseNode** stackPool, dInt32 stack, const dVector& minBox, const dVector& maxBox, OnBodiesInAABB callback, void* const userData) const;
	//void RayCast (const dBroadPhaseNode** stackPool, dFloat32* const distance, dInt32 stack, const dVector& l0, const dVector& l1, dgFastRayTest& ray, OnRayCastAction filter, OnRayPrecastAction prefilter, void* const userData) const;
	//
	//dInt32 ConvexCast (const dBroadPhaseNode** stackPool, dFloat32* const distance, dInt32 stack, const dVector& velocA, const dVector& velocB, dgFastRayTest& ray,  
	//					dgCollisionInstance* const shape, const dgMatrix& matrix, const dVector& target, dFloat32* const param, OnRayPrecastAction prefilter, void* const userData, dgConvexCastReturnInfo* const info, dInt32 maxContacts, dInt32 threadIndex) const;
	//
	//dInt32 Collide(const dBroadPhaseNode** stackPool, dInt32* const overlap, dInt32 stack, const dVector& p0, const dVector& p1, 
	//	            dgCollisionInstance* const shape, const dgMatrix& matrix, OnRayPrecastAction prefilter, void* const userData, dgConvexCastReturnInfo* const info, dInt32 maxContacts, dInt32 threadIndex) const;
	//
	//void SleepingState (dBroadphaseSyncDescriptor* const descriptor, dgBodyMasterList::dListNode* node, dInt32 threadID);
	//void ApplyForceAndtorque (dBroadphaseSyncDescriptor* const descriptor, dgBodyMasterList::dListNode* node, dInt32 threadID);
	//
	//void UpdateAggregateEntropy (dBroadphaseSyncDescriptor* const descriptor, dList<dBroadPhaseAggregate*>::dListNode* node, dInt32 threadID);
	//
	//dBroadPhaseNode* BuildTopDown(dBroadPhaseNode** const leafArray, dInt32 firstBox, dInt32 lastBox, dFitnessList::dListNode** const nextNode);
	//dBroadPhaseNode* BuildTopDownBig(dBroadPhaseNode** const leafArray, dInt32 firstBox, dInt32 lastBox, dFitnessList::dListNode** const nextNode);
	//
	//void KinematicBodyActivation (dContact* const contatJoint) const;
	//
	//void FindGeneratedBodiesCollidingPairs (dBroadphaseSyncDescriptor* const descriptor, dInt32 threadID);
	//void UpdateSoftBodyContacts(dBroadphaseSyncDescriptor* const descriptor, dFloat32 timeStep, dInt32 threadID);
	//void UpdateRigidBodyContacts (dBroadphaseSyncDescriptor* const descriptor, dFloat32 timeStep, dInt32 threadID);
	//void SubmitPairs (dBroadPhaseNode* const body, dBroadPhaseNode* const node, dFloat32 timestep, dInt32 threaCount, dInt32 threadID);
	//
	//bool SanityCheck() const;
	//void AttachNewContact(dInt32 startCount);
	//void DeleteDeadContact(dFloat32 timestep);
	//
	//D_INLINE bool ValidateContactCache(dContact* const contact, const dVector& timestep) const;
	//	
	//static void SleepingStateKernel(void* const descriptor, void* const worldContext, dInt32 threadID);
	//static void ForceAndToqueKernel(void* const descriptor, void* const worldContext, dInt32 threadID);
	//static void CollidingPairsKernel(void* const descriptor, void* const worldContext, dInt32 threadID);
	//static void UpdateAggregateEntropyKernel(void* const descriptor, void* const worldContext, dInt32 threadID);
	//static void AddGeneratedBodiesContactsKernel(void* const descriptor, void* const worldContext, dInt32 threadID);
	//static void UpdateRigidBodyContactKernel(void* const descriptor, void* const worldContext, dInt32 threadID);
	//static void UpdateSoftBodyContactKernel(void* const descriptor, void* const worldContext, dInt32 threadID);
	//static dInt32 CompareNodes(const dBroadPhaseNode* const nodeA, const dBroadPhaseNode* const nodeB, void* const notUsed);
	//
	//class dgPendingCollisionSoftBodies
	//{
	//	public:
	//	dgBody* m_body0;
	//	dgBody* m_body1;
	//};

	dNewton* m_newton;

	//dBroadPhaseNode* m_rootNode;
	//dList<dgBody*> m_generatedBodies;
	//dList<dBroadPhaseNode*> m_updateList;
	//dList<dBroadPhaseAggregate*> m_aggregateList;
	//dUnsigned32 m_lru;
	//dContactCache m_contactCache;
	//dArray<dgPendingCollisionSoftBodies> m_pendingSoftBodyCollisions;
	//dInt32 m_pendingSoftBodyPairsCount;
	//dInt32 m_criticalSectionLock;
	//
	//static dVector m_velocTol;
	//static dVector m_linearContactError2;
	//static dVector m_angularContactError2;
	//
	//friend class dgBody;
	//friend class dgWorld;
	//friend class dgDeadBodies;
	//friend class dgWorldDynamicUpdate;
	//friend class dBroadPhaseAggregate;
	//friend class dgCollisionCompoundFractured;
} D_GCC_VECTOR_ALIGNMENT;
#endif

D_MSC_VECTOR_ALIGNMENT
class dBroadPhase: public dClassAlloc
{
	public:
	D_NEWTON_API virtual ~dBroadPhase();

	virtual void AddBody(dBody* const body) = 0;
	D_NEWTON_API virtual void RemoveBody(dBody* const body);

	D_NEWTON_API virtual void Update(dFloat32 timestep);

	protected:
	D_NEWTON_API dBroadPhase(dNewton* const world);

	dNewton* m_newton;
} D_GCC_VECTOR_ALIGNMENT;

#endif
