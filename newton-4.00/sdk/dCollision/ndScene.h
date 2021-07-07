/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __D_SCENE_H__
#define __D_SCENE_H__

#include "ndCollisionStdafx.h"
#include "ndBodyList.h"
#include "ndSceneNode.h"
#include "ndContactList.h"

#define D_SCENE_MAX_STACK_DEPTH	256
#define D_PRUNE_CONTACT_TOLERANCE		dFloat32 (5.0e-2f)

class ndWorld;
class ndScene;
class ndContact;
class ndRayCastNotify;
class ndContactNotify;
class ndConvexCastNotify;
class ndJointBilateralConstraint;

D_MSV_NEWTON_ALIGN_32
class ndSceneTreeNotiFy
{
	public:
	ndSceneTreeNotiFy()
	{
	}

	virtual ~ndSceneTreeNotiFy()
	{
	}

	virtual void OnDebugNode(const ndSceneNode* const node) = 0;

} D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class ndScene
	:public dClassAlloc
	,public dThreadPool
{
	public: 
	class ndBaseJob: public dThreadPoolJob
	{
		public:
		ndScene* m_owner;
		dFloat32 m_timestep;
		void* m_context;
	};

	protected:
	class ndSpliteInfo;
	class ndFitnessList: public dList <ndSceneTreeNode*, dContainersFreeListAlloc<ndSceneTreeNode*>>
	{
		public:
		ndFitnessList();
		dFloat64 TotalCost() const;

		void AddNode(ndSceneTreeNode* const node);
		void RemoveNode(ndSceneTreeNode* const node);
		
		dFloat64 m_currentCost;
		dNode* m_currentNode;
		dInt32 m_index;
	};

	public:
	D_COLLISION_API virtual ~ndScene();

	void Sync();

	dInt32 GetThreadCount() const;
	virtual ndWorld* GetWorld() const;
	const ndBodyList& GetBodyList() const;

	ndConstraintArray& GetActiveContactArray();
	const ndConstraintArray& GetActiveContactArray() const;

	dArray<ndBodyKinematic*>& GetActiveBodyArray();
	const dArray<ndBodyKinematic*>& GetActiveBodyArray() const;

	template <class T>
	void SubmitJobs(void* const context = nullptr);

	dFloat32 GetTimestep() const;
	void SetTimestep(dFloat32 timestep);

	D_COLLISION_API virtual bool AddBody(ndBodyKinematic* const body);
	D_COLLISION_API virtual bool RemoveBody(ndBodyKinematic* const body);

	D_COLLISION_API virtual void Cleanup() = 0;
	D_COLLISION_API void Update(dFloat32 timestep);

	D_COLLISION_API ndContactNotify* GetContactNotify() const;
	D_COLLISION_API void SetContactNotify(ndContactNotify* const notify);

	virtual void DebugScene(ndSceneTreeNotiFy* const notify) = 0;

	private:
	bool ValidateContactCache(ndContact* const contact, const dVector& timestep) const;
	dFloat32 CalculateSurfaceArea(const ndSceneNode* const node0, const ndSceneNode* const node1, dVector& minBox, dVector& maxBox) const;

	virtual void FindCollidinPairs(dInt32 threadIndex, ndBodyKinematic* const body, bool oneWay) = 0;

	D_COLLISION_API virtual void UpdateAabb(dInt32 threadIndex, ndBodyKinematic* const body);
	D_COLLISION_API virtual void UpdateTransformNotify(dInt32 threadIndex, ndBodyKinematic* const body);
	D_COLLISION_API virtual void CalculateContacts(dInt32 threadIndex, ndContact* const contact); 

	void CalculateJointContacts(dInt32 threadIndex, ndContact* const contact);
	void ProcessContacts(dInt32 threadIndex, dInt32 contactCount, ndContactSolver* const contactSolver);

	void RotateLeft(ndSceneTreeNode* const node, ndSceneNode** const root);
	void RotateRight(ndSceneTreeNode* const node, ndSceneNode** const root);
	dFloat64 ReduceEntropy(ndFitnessList& fitness, ndSceneNode** const root);
	void ImproveNodeFitness(ndSceneTreeNode* const node, ndSceneNode** const root);
	static dInt32 CompareNodes(const ndSceneNode* const nodeA, const ndSceneNode* const nodeB, void* const);
	ndSceneNode* BuildTopDown(ndSceneNode** const leafArray, dInt32 firstBox, dInt32 lastBox, ndFitnessList::dNode** const nextNode);
	ndSceneNode* BuildTopDownBig(ndSceneNode** const leafArray, dInt32 firstBox, dInt32 lastBox, ndFitnessList::dNode** const nextNode);

	const ndContactList& GetContactList() const;

	D_COLLISION_API void CollisionOnlyUpdate();

	protected:
	D_COLLISION_API ndScene();
	
	D_COLLISION_API void UpdateAabb();
	D_COLLISION_API void BuildBodyArray();
	D_COLLISION_API void UpdateTransform();
	D_COLLISION_API void CalculateContacts();
	D_COLLISION_API void DeleteDeadContact();
	D_COLLISION_API void FindCollidingPairs();

	D_COLLISION_API virtual void ThreadFunction();
	virtual void BalanceScene() = 0;

	D_COLLISION_API ndSceneTreeNode* InsertNode(ndSceneNode* const root, ndSceneNode* const node);
	void UpdateFitness(ndFitnessList& fitness, dFloat64& oldEntropy, ndSceneNode** const root);

	ndContact* FindContactJoint(ndBodyKinematic* const body0, ndBodyKinematic* const body1) const;
	ndJointBilateralConstraint* FindBilateralJoint(ndBodyKinematic* const body0, ndBodyKinematic* const body1) const;

	void AddPair(ndBodyKinematic* const body0, ndBodyKinematic* const body1);
	bool TestOverlaping(const ndBodyKinematic* const body0, const ndBodyKinematic* const body1) const;
	void SubmitPairs(ndSceneNode* const leaftNode, ndSceneNode* const node);

	D_COLLISION_API void BuildContactArray();
	D_COLLISION_API virtual dFloat32 RayCast(ndRayCastNotify& callback, const dVector& globalOrigin, const dVector& globalDest) const = 0;
	D_COLLISION_API virtual dFloat32 ConvexCast(ndConvexCastNotify& callback, const ndShapeInstance& convexShape, const dMatrix& globalOrigin, const dVector& globalDest) const = 0;

	dFloat32 RayCast(ndRayCastNotify& callback, const ndSceneNode** stackPool, dFloat32* const distance, dInt32 stack, const dFastRayTest& ray) const;
	dFloat32 ConvexCast(ndConvexCastNotify& callback, const ndSceneNode** stackPool, dFloat32* const distance, dInt32 stack, const dFastRayTest& ray, const ndShapeInstance& convexShape, const dMatrix& globalOrigin, const dVector& globalDest, const dVector& velocA, const dVector& velocB) const;
	
	ndBodyList m_bodyList;
	ndContactList m_contactList;
	dArray<ndBodyKinematic*> m_activeBodyArray;
	ndConstraintArray m_activeConstraintArray;
	dSpinLock m_contactLock;
	ndSceneNode* m_rootNode;
	ndContactNotify* m_contactNotifyCallback;
	dFloat32 m_timestep;
	dUnsigned32 m_sleepBodies;
	dUnsigned32 m_lru;
	bool m_fullScan;

	static dVector m_velocTol;
	static dVector m_linearContactError2;
	static dVector m_angularContactError2;

	friend class ndWorld;
	friend class ndRayCastNotify;
	friend class ndConvexCastNotify;
	friend class ndSkeletonContainer;
} D_GCC_NEWTON_ALIGN_32 ;

inline void ndScene::Sync()
{
	dThreadPool::Sync();
}

inline ndWorld* ndScene::GetWorld() const
{
	return nullptr;
}

inline dInt32 ndScene::GetThreadCount() const
{
	const dThreadPool& pool = *this;
	return pool.GetCount();
}

inline const ndBodyList& ndScene::GetBodyList() const
{
	return m_bodyList;
}

inline ndConstraintArray& ndScene::GetActiveContactArray()
{
	return m_activeConstraintArray;
}

inline const ndConstraintArray& ndScene::GetActiveContactArray() const
{
	return m_activeConstraintArray;
}

inline dArray<ndBodyKinematic*>& ndScene::GetActiveBodyArray()
{
	return m_activeBodyArray;
}

inline const dArray<ndBodyKinematic*>& ndScene::GetActiveBodyArray() const
{
	return m_activeBodyArray;
}

inline const ndContactList& ndScene::GetContactList() const
{
	return m_contactList;
}

template <class T>
void ndScene::SubmitJobs(void* const context)
{
	T extJob[D_MAX_THREADS_COUNT];
	dThreadPoolJob* extJobPtr[D_MAX_THREADS_COUNT];

	const dInt32 threadCount = GetThreadCount();
	for (dInt32 i = 0; i < threadCount; i++)
	{
		extJob[i].m_owner = this;
		extJob[i].m_context = context;
		extJob[i].m_timestep = m_timestep;
		extJobPtr[i] = &extJob[i];
	}
	ExecuteJobs(extJobPtr);
}

inline dFloat32 ndScene::GetTimestep() const
{
	return m_timestep;
}

inline void ndScene::SetTimestep(dFloat32 timestep)
{
	m_timestep = timestep;
}

D_INLINE dFloat32 ndScene::CalculateSurfaceArea(const ndSceneNode* const node0, const ndSceneNode* const node1, dVector& minBox, dVector& maxBox) const
{
	minBox = node0->m_minBox.GetMin(node1->m_minBox);
	maxBox = node0->m_maxBox.GetMax(node1->m_maxBox);
	dVector side0(maxBox - minBox);
	return side0.DotProduct(side0.ShiftTripleRight()).GetScalar();
}

#endif
