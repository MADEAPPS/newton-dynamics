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

#include "ndCollisionStdafx.h"
#include "ntContactList.h"
#include "ntSceneNode.h"

#define D_BROADPHASE_MAX_STACK_DEPTH	256

class ntPair;
class ntContact;
class ntRayCastNotify;
class ntContactNotify;
class ntBilateralJoint;

D_MSV_NEWTON_ALIGN_32
class ntScene
	:public dClassAlloc
	,public dSyncMutex
	,public dThread
	,public dThreadPool
{
	public: 
	class ntBaseJob: public dThreadPoolJob
	{
		public:
		dAtomic<int>* m_it;
		ntScene* m_owner;
		dFloat32 m_timestep;
	};

	protected:
	class ntBodyList: public dList<ntBodyKinematic*>
	{
	};

	class ntSpliteInfo;
	class ntFitnessList: public dList <ntSceneTreeNode*, dContainersFreeListAlloc<ntSceneTreeNode*>>
	{
		public:
		ntFitnessList();
		dFloat64 TotalCost() const;
		
		dFloat64 m_prevCost;
		dInt32 m_index;
	};

	public:
	ND_COLLISION_API virtual ~ntScene();

	dInt32 GetThreadCount() const;

	ND_COLLISION_API virtual bool AddBody(ntBodyKinematic* const body);
	ND_COLLISION_API virtual bool RemoveBody(ntBodyKinematic* const body);

	ND_COLLISION_API virtual void Cleanup() = 0;
	ND_COLLISION_API void Update(dFloat32 timestep);

	ND_COLLISION_API ntContactNotify* GetContactNotify() const;
	ND_COLLISION_API void SetContactNotify(ntContactNotify* const notify);

	private:
	void AttachNewContact();
	void UpdateAabb(dFloat32 timestep);
	void FindCollidingPairs(dFloat32 timestep);
	void CalculateContacts(dFloat32 timestep);
	bool ValidateContactCache(ntContact* const contact, const dVector& timestep) const;
	dFloat32 CalculateSurfaceArea(const ntSceneNode* const node0, const ntSceneNode* const node1, dVector& minBox, dVector& maxBox) const;

	virtual void BalanceBroadPhase() = 0;
	virtual void UpdateAabb(dInt32 threadIndex, dFloat32 timestep, ntBodyKinematic* const body);
	virtual void FindCollidinPairs(dInt32 threadIndex, dFloat32 timestep, ntBodyKinematic* const body) = 0;
	virtual void CalculateContacts(dInt32 threadIndex, dFloat32 timestep, ntContact* const contact);
	void CalculateJointContacts(dInt32 threadIndex, dFloat32 timestep, ntContact* const contact);

	void RotateLeft(ntSceneTreeNode* const node, ntSceneNode** const root);
	void RotateRight(ntSceneTreeNode* const node, ntSceneNode** const root);
	dFloat64 ReduceEntropy(ntFitnessList& fitness, ntSceneNode** const root);
	void ImproveNodeFitness(ntSceneTreeNode* const node, ntSceneNode** const root);
	static dInt32 CompareNodes(const ntSceneNode* const nodeA, const ntSceneNode* const nodeB, void* const);
	ntSceneNode* BuildTopDown(ntSceneNode** const leafArray, dInt32 firstBox, dInt32 lastBox, ntFitnessList::dListNode** const nextNode);
	ntSceneNode* BuildTopDownBig(ntSceneNode** const leafArray, dInt32 firstBox, dInt32 lastBox, ntFitnessList::dListNode** const nextNode);

	template <class T>
	void SubmitJobs(dFloat32 timestep);

	protected:
	ntScene();
	virtual void ThreadFunction();
	void BuildBodyArray();
	void InternalUpdate(dFloat32 timestep);

	ND_COLLISION_API ntSceneTreeNode* InsertNode(ntSceneNode* const root, ntSceneNode* const node);
	void UpdateFitness(ntFitnessList& fitness, dFloat64& oldEntropy, ntSceneNode** const root);

	//void CalculatePairContacts(dInt32 threadIndex, ntPair* const pair) const;
	ntContact* FindContactJoint(ntBodyKinematic* const body0, ntBodyKinematic* const body1) const;
	ntBilateralJoint* FindBilateralJoint(ntBody* const body0, ntBody* const body1) const;

	void AddPair(ntBodyKinematic* const body0, ntBodyKinematic* const body1, const dFloat32 timestep);
	bool TestOverlaping(const ntBodyKinematic* const body0, const ntBodyKinematic* const body1, dFloat32 timestep) const;
	void SubmitPairs(ntSceneNode* const leaftNode, ntSceneNode* const node, dFloat32 timestep);

	ND_COLLISION_API virtual dFloat32 RayCast(ntRayCastNotify& callback, const dVector& p0, const dVector& p1) const = 0;
	dFloat32 RayCast(ntRayCastNotify& callback, const ntSceneNode** stackPool, dFloat32* const distance, dInt32 stack, const dFastRayTest& ray) const;
	
	ntBodyList m_bodyList;
	ntContactList m_contactList;
	dArray<ntBodyKinematic*> m_tmpBodyArray;
	dArray<ntContact*> m_activeContacts;
	ntSceneNode* m_rootNode;
	ntContactNotify* m_contactNotifyCallback;
	dUnsigned32 m_lru;
	dFloat32 m_timestep;
	bool m_fullScan;

	static dVector m_velocTol;
	static dVector m_linearContactError2;
	static dVector m_angularContactError2;

	friend class ntRayCastNotify;
} D_GCC_NEWTON_ALIGN_32 ;

inline dInt32 ntScene::GetThreadCount() const
{
	const dThreadPool& pool = *this;
	return pool.GetCount();
}

template <class T>
void ntScene::SubmitJobs(dFloat32 timestep)
{
	dAtomic<dInt32> it(0);
	T extJob[D_MAX_THREADS_COUNT];
	dThreadPoolJob* extJobPtr[D_MAX_THREADS_COUNT];

	const dInt32 threadCount = GetThreadCount();
	for (int i = 0; i < threadCount; i++)
	{
		extJob[i].m_it = &it;
		extJob[i].m_owner = this;
		extJob[i].m_timestep = timestep;
		extJobPtr[i] = &extJob[i];
	}
	//DispatchJobs(extJobPtr);
	ExecuteJobs(extJobPtr);
}

D_INLINE dFloat32 ntScene::CalculateSurfaceArea(const ntSceneNode* const node0, const ntSceneNode* const node1, dVector& minBox, dVector& maxBox) const
{
	minBox = node0->m_minBox.GetMin(node1->m_minBox);
	maxBox = node0->m_maxBox.GetMax(node1->m_maxBox);
	dVector side0(maxBox - minBox);
	return side0.DotProduct(side0.ShiftTripleRight()).GetScalar();
}

#endif
