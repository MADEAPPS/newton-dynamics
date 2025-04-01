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
#include "ndBodyListView.h"
#include "ndContactArray.h"
#include "ndPolygonMeshDesc.h"

#define D_SCENE_MAX_STACK_DEPTH		256

class ndWorld;
class ndScene;
class ndContact;
class ndRayCastNotify;
class ndContactNotify;
class ndConvexCastNotify;
class ndBodiesInAabbNotify;
class ndJointBilateralConstraint;

D_MSV_NEWTON_CLASS_ALIGN_32
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
} D_GCC_NEWTON_CLASS_ALIGN_32;

D_MSV_NEWTON_CLASS_ALIGN_32
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
	D_COLLISION_API bool ValidateScene();

	D_COLLISION_API virtual bool AddBody(const ndSharedPtr<ndBody>& body);
	D_COLLISION_API virtual bool RemoveBody(const ndSharedPtr<ndBody>& body);

	D_COLLISION_API ndSharedPtr<ndBody> GetBody(ndBody* const body) const;

	D_COLLISION_API virtual void Begin();
	D_COLLISION_API virtual void End();
	D_COLLISION_API virtual void Sync();
	D_COLLISION_API virtual bool IsHighPerformanceCompute() const;
	D_COLLISION_API virtual bool IsValid() const;

	D_COLLISION_API virtual void Cleanup();
	D_COLLISION_API virtual void PrepareCleanup();

	D_COLLISION_API ndContactNotify* GetContactNotify() const;
	D_COLLISION_API void SetContactNotify(ndContactNotify* const notify);

	D_COLLISION_API virtual void DebugScene(ndSceneTreeNotiFy* const notify);

	D_COLLISION_API virtual void BodiesInAabb(ndBodiesInAabbNotify& callback, const ndVector& minBox, const ndVector& maxBox) const;
	D_COLLISION_API virtual bool RayCast(ndRayCastNotify& callback, const ndVector& globalOrigin, const ndVector& globalDest) const;
	D_COLLISION_API virtual bool ConvexCast(ndConvexCastNotify& callback, const ndShapeInstance& convexShape, const ndMatrix& globalOrigin, const ndVector& globalDest) const;

	D_COLLISION_API void SendBackgroundTask(ndBackgroundTask* const job);

	D_COLLISION_API ndInt32 GetThreadCount() const;

	D_COLLISION_API virtual ndWorld* GetWorld() const;
	D_COLLISION_API const ndBodyListView& GetBodyList() const;
	D_COLLISION_API const ndBodyList& GetParticleList() const;

	D_COLLISION_API ndArray<ndBodyKinematic*>& GetActiveBodyArray();
	D_COLLISION_API const ndArray<ndBodyKinematic*>& GetActiveBodyArray() const;

	D_COLLISION_API ndArray<ndConstraint*>& GetActiveContactArray();
	D_COLLISION_API const ndArray<ndConstraint*>& GetActiveContactArray() const;

	D_COLLISION_API ndFloat32 GetTimestep() const;
	D_COLLISION_API void SetTimestep(ndFloat32 timestep);
	D_COLLISION_API ndBodyKinematic* GetSentinelBody() const;
	D_COLLISION_API ndArray<ndUnsigned8>& GetScratchBuffer();

	protected:
	D_COLLISION_API ndScene();
	D_COLLISION_API ndScene(const ndScene& src);
	bool ValidateContactCache(ndContact* const contact, const ndVector& timestep) const;
	
	const ndContactArray& GetContactArray() const;
	void FindCollidingPairs(ndBodyKinematic* const body, ndInt32 threadId);
	void FindCollidingPairsForward(ndBodyKinematic* const body, ndInt32 threadId);
	void FindCollidingPairsBackward(ndBodyKinematic* const body, ndInt32 threadId);
	void AddPair(ndBodyKinematic* const body0, ndBodyKinematic* const body1, ndInt32 threadId);
	void SubmitPairs(ndBvhLeafNode* const bodyNode, ndBvhNode* const node, bool forward, ndInt32 threadId);

	void CalculateJointContacts(ndInt32 threadIndex, ndContact* const contact);
	void ProcessContacts(ndInt32 threadIndex, ndInt32 contactCount, ndContactSolver* const contactSolver);

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
	D_COLLISION_API virtual void CreateNewContacts();
	D_COLLISION_API virtual void CalculateContacts();
	D_COLLISION_API virtual void FindCollidingPairs();
	D_COLLISION_API virtual void DeleteDeadContacts();

	D_COLLISION_API virtual void CalculateContacts(ndInt32 threadIndex, ndContact* const contact);
	D_COLLISION_API virtual void UpdateTransformNotify(ndInt32 threadIndex, ndBodyKinematic* const body);

	D_COLLISION_API virtual void ParticleUpdate(ndFloat32 timestep);
	D_COLLISION_API virtual bool AddParticle(const ndSharedPtr<ndBody>& particle);
	D_COLLISION_API virtual bool RemoveParticle(const ndSharedPtr<ndBody>& particle);

	ndBodyListView m_bodyList;
	ndBodyList m_particleSetList;
	ndContactArray m_contactArray;
	ndBvhSceneManager m_bvhSceneManager;
	ndArray<ndUnsigned8> m_scratchBuffer;
	ndArray<ndBodyKinematic*> m_sceneBodyArray;
	ndArray<ndConstraint*> m_activeConstraintArray;
	ndSpecialList<ndBodyKinematic> m_specialUpdateList;
	ndArray<ndContactPairs> m_newPairs;
	ndArray<ndContactPairs> m_partialNewPairs[D_MAX_THREADS_COUNT];
	ndPolygonMeshDesc::ndStaticMeshFaceQuery m_staticMeshQuery[D_MAX_THREADS_COUNT];
	ndPolygonMeshDesc::ndProceduralStaticMeshFaceQuery m_proceduralStaticMeshQuery[D_MAX_THREADS_COUNT];

	ndSpinLock m_lock;
	ndBvhNode* m_rootNode;
	ndBodyKinematic* m_sentinelBody;
	ndContactNotify* m_contactNotifyCallback;
	ndThreadBackgroundWorker* m_backgroundThread;
	
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
	friend class ndPolygonMeshDesc;
	friend class ndConvexCastNotify;
	friend class ndSkeletonContainer;
} D_GCC_NEWTON_CLASS_ALIGN_32 ;


#endif
