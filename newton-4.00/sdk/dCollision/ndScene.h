/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_SCENE_H__
#define __ND_SCENE_H__

#include "ndCollisionStdafx.h"
#include "ndBvhNode.h"
#include "ndListView.h"
#include "ndContactArray.h"

#define D_SCENE_MAX_STACK_DEPTH		256

class ndWorld;
class ndScene;
class ndContact;
class ndRayCastNotify;
class ndContactNotify;
class ndConvexCastNotify;
class ndBodiesInAabbNotify;
class ndJointBilateralConstraint;

D_MSV_NEWTON_ALIGN_32
class ndSceneTreeNotiFy : public ndClassAlloc
{
	public:
	ndSceneTreeNotiFy()
	{
	}

	virtual ~ndSceneTreeNotiFy()
	{
	}

	virtual void OnDebugNode(const ndBvhNode* const node) = 0;

} D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class ndScene : public ndThreadPool
{
	protected:
	class ndContactPairs
	{
		public:
		ndContactPairs(ndUnsigned32 body0, ndUnsigned32 body1)
			:m_body0(ndMin(body0, body1))
			,m_body1(ndMax(body0, body1))
		{
		}

		ndUnsigned32 m_body0;
		ndUnsigned32 m_body1;
	};

	public:
	D_COLLISION_API virtual ~ndScene();
	D_COLLISION_API virtual bool AddBody(ndBodyKinematic* const body);
	D_COLLISION_API virtual bool RemoveBody(ndBodyKinematic* const body);

	D_COLLISION_API virtual void Begin();
	D_COLLISION_API virtual void End();
	D_COLLISION_API virtual void Sync();
	D_COLLISION_API virtual bool IsGPU() const;
	D_COLLISION_API virtual bool IsValid() const;
	D_COLLISION_API virtual double GetGPUTime() const;

	D_COLLISION_API virtual void Cleanup();
	D_COLLISION_API void Update(ndFloat32 timestep);

	D_COLLISION_API ndContactNotify* GetContactNotify() const;
	D_COLLISION_API void SetContactNotify(ndContactNotify* const notify);

	D_COLLISION_API virtual void DebugScene(ndSceneTreeNotiFy* const notify);

	D_COLLISION_API virtual void BodiesInAabb(ndBodiesInAabbNotify& callback) const;
	D_COLLISION_API virtual bool RayCast(ndRayCastNotify& callback, const ndVector& globalOrigin, const ndVector& globalDest) const;
	D_COLLISION_API virtual bool ConvexCast(ndConvexCastNotify& callback, const ndShapeInstance& convexShape, const ndMatrix& globalOrigin, const ndVector& globalDest) const;

	D_COLLISION_API void SendBackgroundTask(ndBackgroundTask* const job);

	ndInt32 GetThreadCount() const;

	virtual ndWorld* GetWorld() const;
	const ndBodyList& GetBodyList() const;

	ndArray<ndBodyKinematic*>& GetActiveBodyArray();
	const ndArray<ndBodyKinematic*>& GetActiveBodyArray() const;

	ndArray<ndConstraint*>& GetActiveContactArray();
	const ndArray<ndConstraint*>& GetActiveContactArray() const;

	ndArray<ndUnsigned8>& GetScratchBuffer();

	ndFloat32 GetTimestep() const;
	void SetTimestep(ndFloat32 timestep);
	ndBodyKinematic* GetSentinelBody() const;

	protected:
	D_COLLISION_API ndScene();
	D_COLLISION_API ndScene(const ndScene& src);
	bool ValidateContactCache(ndContact* const contact, const ndVector& timestep) const;

	const ndContactArray& GetContactArray() const;
	void FindCollidingPairs(ndBodyKinematic* const body, ndInt32 threadId);
	void FindCollidingPairsForward(ndBodyKinematic* const body, ndInt32 threadId);
	void FindCollidingPairsBackward(ndBodyKinematic* const body, ndInt32 threadId);
	void AddPair(ndBodyKinematic* const body0, ndBodyKinematic* const body1, ndInt32 threadId);
	void SubmitPairs(ndBvhLeafNode* const bodyNode, ndBvhNode* const node, ndInt32 threadId);

	void CalculateJointContacts(ndInt32 threadIndex, ndContact* const contact);
	void ProcessContacts(ndInt32 threadIndex, ndInt32 contactCount, ndContactSolver* const contactSolver);

	void BodiesInAabb(ndBodiesInAabbNotify& callback, const ndBvhNode** stackPool, ndInt32 stack) const;
	ndJointBilateralConstraint* FindBilateralJoint(ndBodyKinematic* const body0, ndBodyKinematic* const body1) const;
	bool RayCast(ndRayCastNotify& callback, const ndBvhNode** stackPool, ndFloat32* const distance, ndInt32 stack, const ndFastRay& ray) const;
	bool ConvexCast(ndConvexCastNotify& callback, const ndBvhNode** stackPool, ndFloat32* const distance, ndInt32 stack, const ndFastRay& ray, const ndShapeInstance& convexShape, const ndMatrix& globalOrigin, const ndVector& globalDest) const;

	// call from sub steps update
	D_COLLISION_API virtual void ApplyExtForce();
	D_COLLISION_API virtual void BalanceScene();
	D_COLLISION_API virtual void InitBodyArray();
	D_COLLISION_API virtual void UpdateSpecial();
	D_COLLISION_API virtual void UpdateBodyList();
	D_COLLISION_API virtual void UpdateTransform();
	D_COLLISION_API virtual void CalculateContacts();
	D_COLLISION_API virtual void FindCollidingPairs();
	
	D_COLLISION_API virtual void ThreadFunction();

	D_COLLISION_API virtual void CollisionOnlyUpdate();
	D_COLLISION_API virtual void CalculateContacts(ndInt32 threadIndex, ndContact* const contact);
	D_COLLISION_API virtual void UpdateTransformNotify(ndInt32 threadIndex, ndBodyKinematic* const body);

	ndBodyList m_bodyList;
	ndContactArray m_contactArray;
	ndBvhSceneManager m_bvhSceneManager;
	ndArray<ndUnsigned8> m_scratchBuffer;
	ndArray<ndBodyKinematic*> m_sceneBodyArray;
	ndArray<ndConstraint*> m_activeConstraintArray;
	ndList<ndBodyKinematic*> m_specialUpdateList;
	ndThreadBackgroundWorker m_backgroundThread;
	ndArray<ndContactPairs> m_newPairs;
	ndArray<ndContactPairs> m_partialNewPairs[D_MAX_THREADS_COUNT];
	

	ndSpinLock m_lock;
	ndBvhNode* m_rootNode;
	ndBodyKinematic* m_sentinelBody;
	ndContactNotify* m_contactNotifyCallback;
	
	ndFloat32 m_timestep;
	ndUnsigned32 m_lru;
	ndUnsigned32 m_frameNumber;
	ndUnsigned32 m_subStepNumber;
	ndUnsigned32 m_forceBalanceSceneCounter;

	static ndVector m_velocTol;
	static ndVector m_linearContactError2;
	static ndVector m_angularContactError2;

	friend class ndWorld;
	friend class ndBodyKinematic;
	friend class ndRayCastNotify;
	friend class ndConvexCastNotify;
	friend class ndSkeletonContainer;
} D_GCC_NEWTON_ALIGN_32 ;

inline bool ndScene::IsValid() const
{
	return true;
}

inline bool ndScene::IsGPU() const
{
	return false;
}

inline double ndScene::GetGPUTime() const
{
	return 0.0;
}

inline ndWorld* ndScene::GetWorld() const
{
	return nullptr;
}

inline ndInt32 ndScene::GetThreadCount() const
{
	const ndThreadPool& pool = *this;
	return pool.GetThreadCount();
}

inline ndArray<ndUnsigned8>& ndScene::GetScratchBuffer()
{
	return m_scratchBuffer;
}

inline const ndBodyList& ndScene::GetBodyList() const
{
	return m_bodyList;
}

inline ndArray<ndConstraint*>& ndScene::GetActiveContactArray()
{
	return m_activeConstraintArray;
}

inline const ndContactArray& ndScene::GetContactArray() const
{
	return m_contactArray;
}

inline const ndArray<ndConstraint*>& ndScene::GetActiveContactArray() const
{
	return m_activeConstraintArray;
}

inline ndArray<ndBodyKinematic*>& ndScene::GetActiveBodyArray()
{
	return m_bodyList.GetView();
}

inline const ndArray<ndBodyKinematic*>& ndScene::GetActiveBodyArray() const
{
	return m_bodyList.GetView();
}

inline ndFloat32 ndScene::GetTimestep() const
{
	return m_timestep;
}

inline void ndScene::SetTimestep(ndFloat32 timestep)
{
	m_timestep = timestep;
}

inline ndBodyKinematic* ndScene::GetSentinelBody() const
{
	return m_sentinelBody;
}

#endif
