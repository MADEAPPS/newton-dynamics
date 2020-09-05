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
#include "ndContactList.h"
#include "ndSceneNode.h"

#define D_BROADPHASE_MAX_STACK_DEPTH	256

class ndContact;
class ndRayCastNotify;
class ndContactNotify;
class ndBilateralJoint;

D_MSV_NEWTON_ALIGN_32
class ndScene
	:public dClassAlloc
	,public dSyncMutex
	,public dThread
	,public dThreadPool
{
	public: 
	class ndBaseJob: public dThreadPoolJob
	{
		public:
		dAtomic<int>* m_it;
		ndScene* m_owner;
		dFloat32 m_timestep;
	};

	protected:
	class ndBodyList: public dList<ndBodyKinematic*>
	{
	};

	class ndSpliteInfo;
	class ndFitnessList: public dList <ndSceneTreeNode*, dContainersFreeListAlloc<ndSceneTreeNode*>>
	{
		public:
		ndFitnessList();
		dFloat64 TotalCost() const;
		
		dFloat64 m_prevCost;
		dInt32 m_index;
	};

	public:
	ND_COLLISION_API virtual ~ndScene();

	dInt32 GetThreadCount() const;
	const dArray<ndBodyKinematic*>& GetWorkingBodyArray() const;

	template <class T>
	void SubmitJobs(dFloat32 timestep);

	ND_COLLISION_API virtual bool AddBody(ndBodyKinematic* const body);
	ND_COLLISION_API virtual bool RemoveBody(ndBodyKinematic* const body);

	ND_COLLISION_API virtual void Cleanup() = 0;
	ND_COLLISION_API void Update(dFloat32 timestep);

	ND_COLLISION_API ndContactNotify* GetContactNotify() const;
	ND_COLLISION_API void SetContactNotify(ndContactNotify* const notify);

	private:
	bool ValidateContactCache(ndContact* const contact, const dVector& timestep) const;
	dFloat32 CalculateSurfaceArea(const ndSceneNode* const node0, const ndSceneNode* const node1, dVector& minBox, dVector& maxBox) const;
	virtual void UpdateAabb(dInt32 threadIndex, dFloat32 timestep, ndBodyKinematic* const body);
	virtual void FindCollidinPairs(dInt32 threadIndex, dFloat32 timestep, ndBodyKinematic* const body) = 0;
	virtual void CalculateContacts(dInt32 threadIndex, dFloat32 timestep, ndContact* const contact);
	void CalculateJointContacts(dInt32 threadIndex, dFloat32 timestep, ndContact* const contact);

	void RotateLeft(ndSceneTreeNode* const node, ndSceneNode** const root);
	void RotateRight(ndSceneTreeNode* const node, ndSceneNode** const root);
	dFloat64 ReduceEntropy(ndFitnessList& fitness, ndSceneNode** const root);
	void ImproveNodeFitness(ndSceneTreeNode* const node, ndSceneNode** const root);
	static dInt32 CompareNodes(const ndSceneNode* const nodeA, const ndSceneNode* const nodeB, void* const);
	ndSceneNode* BuildTopDown(ndSceneNode** const leafArray, dInt32 firstBox, dInt32 lastBox, ndFitnessList::dListNode** const nextNode);
	ndSceneNode* BuildTopDownBig(ndSceneNode** const leafArray, dInt32 firstBox, dInt32 lastBox, ndFitnessList::dListNode** const nextNode);

	void CollisionOnlyUpdate();

	protected:
	ndScene();
	
	void BuildBodyArray();
	void AttachNewContact();
	void UpdateAabb(dFloat32 timestep);
	void TransformUpdate(dFloat32 timestep);
	void CalculateContacts(dFloat32 timestep);
	void FindCollidingPairs(dFloat32 timestep);

	virtual void ThreadFunction();
	virtual void BalanceBroadPhase() = 0;

	ND_COLLISION_API ndSceneTreeNode* InsertNode(ndSceneNode* const root, ndSceneNode* const node);
	void UpdateFitness(ndFitnessList& fitness, dFloat64& oldEntropy, ndSceneNode** const root);

	ndContact* FindContactJoint(ndBodyKinematic* const body0, ndBodyKinematic* const body1) const;
	ndBilateralJoint* FindBilateralJoint(ndBody* const body0, ndBody* const body1) const;

	void AddPair(ndBodyKinematic* const body0, ndBodyKinematic* const body1, const dFloat32 timestep);
	bool TestOverlaping(const ndBodyKinematic* const body0, const ndBodyKinematic* const body1, dFloat32 timestep) const;
	void SubmitPairs(ndSceneNode* const leaftNode, ndSceneNode* const node, dFloat32 timestep);

	ND_COLLISION_API virtual dFloat32 RayCast(ndRayCastNotify& callback, const dVector& p0, const dVector& p1) const = 0;
	dFloat32 RayCast(ndRayCastNotify& callback, const ndSceneNode** stackPool, dFloat32* const distance, dInt32 stack, const dFastRayTest& ray) const;
	
	ndBodyList m_bodyList;
	ndContactList m_contactList;
	dArray<ndBodyKinematic*> m_tmpBodyArray;
	dArray<ndContact*> m_activeContacts;
	ndSceneNode* m_rootNode;
	ndContactNotify* m_contactNotifyCallback;
	dUnsigned32 m_lru;
	dFloat32 m_timestep;
	bool m_fullScan;

	static dVector m_velocTol;
	static dVector m_linearContactError2;
	static dVector m_angularContactError2;

	friend class ndRayCastNotify;
} D_GCC_NEWTON_ALIGN_32 ;

inline dInt32 ndScene::GetThreadCount() const
{
	const dThreadPool& pool = *this;
	return pool.GetCount();
}

inline const dArray<ndBodyKinematic*>& ndScene::GetWorkingBodyArray() const
{
	return m_tmpBodyArray;
}

template <class T>
void ndScene::SubmitJobs(dFloat32 timestep)
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
	ExecuteJobs(extJobPtr);
}

D_INLINE dFloat32 ndScene::CalculateSurfaceArea(const ndSceneNode* const node0, const ndSceneNode* const node1, dVector& minBox, dVector& maxBox) const
{
	minBox = node0->m_minBox.GetMin(node1->m_minBox);
	maxBox = node0->m_maxBox.GetMax(node1->m_maxBox);
	dVector side0(maxBox - minBox);
	return side0.DotProduct(side0.ShiftTripleRight()).GetScalar();
}

#endif
