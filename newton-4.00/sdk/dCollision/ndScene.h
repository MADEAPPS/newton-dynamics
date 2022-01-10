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

#ifndef __ND_SCENE_H__
#define __ND_SCENE_H__

#include "ndCollisionStdafx.h"
#include "ndBodyList.h"
#include "ndSceneNode.h"
#include "ndContactArray.h"

#define D_SCENE_MAX_STACK_DEPTH		256
#define D_PRUNE_CONTACT_TOLERANCE	ndFloat32 (5.0e-2f)

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

	virtual void OnDebugNode(const ndSceneNode* const node) = 0;

} D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class ndScene : public ndThreadPool
{
	public: 
	class ndBaseJob: public ndThreadPoolJob
	{
		public:
		ndScene* m_owner;
		ndFloat32 m_timestep;
		void* m_context;
	};

	protected:
	class ndSpliteInfo;
	class ndFitnessList: public ndList <ndSceneTreeNode*, ndContainersFreeListAlloc<ndSceneTreeNode*>>
	{
		public:
		ndFitnessList();
		ndFloat64 TotalCost() const;

		void AddNode(ndSceneTreeNode* const node);
		void RemoveNode(ndSceneTreeNode* const node);
		
		ndFloat64 m_currentCost;
		ndNode* m_currentNode;
		ndInt32 m_index;
	};

	public:
	D_COLLISION_API virtual ~ndScene();

	void Sync();

	ndInt32 GetThreadCount() const;
	virtual ndWorld* GetWorld() const;
	const ndBodyList& GetBodyList() const;

	ndArray<ndConstraint*>& GetActiveContactArray();
	const ndArray<ndConstraint*>& GetActiveContactArray() const;

	ndArray<ndBodyKinematic*>& GetActiveBodyArray();
	const ndArray<ndBodyKinematic*>& GetActiveBodyArray() const;

	template <class T> 
	void SubmitJobs(void* const context = nullptr);

	template <class T, ndInt32 bits, class dEvaluateKey, class dKey = ndUnsigned32>
	void CountingSort(T* const array, T* const scratchBuffer, ndInt32 elements, ndInt32 digitLocation);

	ndFloat32 GetTimestep() const;
	void SetTimestep(ndFloat32 timestep);

	ndBodyKinematic* GetSentinelBody() const;

	D_COLLISION_API virtual bool AddBody(ndBodyKinematic* const body);
	D_COLLISION_API virtual bool RemoveBody(ndBodyKinematic* const body);

	D_COLLISION_API virtual void Cleanup();
	D_COLLISION_API void Update(ndFloat32 timestep);

	D_COLLISION_API ndContactNotify* GetContactNotify() const;
	D_COLLISION_API void SetContactNotify(ndContactNotify* const notify);

	D_COLLISION_API virtual void DebugScene(ndSceneTreeNotiFy* const notify);

	D_COLLISION_API virtual void BodiesInAabb(ndBodiesInAabbNotify& callback) const;
	D_COLLISION_API virtual bool RayCast(ndRayCastNotify& callback, const ndVector& globalOrigin, const ndVector& globalDest) const;
	D_COLLISION_API virtual bool ConvexCast(ndConvexCastNotify& callback, const ndShapeInstance& convexShape, const ndMatrix& globalOrigin, const ndVector& globalDest) const;

	private:
	bool ValidateContactCache(ndContact* const contact, const ndVector& timestep) const;
	ndFloat32 CalculateSurfaceArea(const ndSceneNode* const node0, const ndSceneNode* const node1, ndVector& minBox, ndVector& maxBox) const;

	D_COLLISION_API virtual void FindCollidingPairs(ndBodyKinematic* const body);
	D_COLLISION_API virtual void FindCollidingPairsForward(ndBodyKinematic* const body);
	D_COLLISION_API virtual void FindCollidingPairsBackward(ndBodyKinematic* const body);
	void AddNode(ndSceneNode* const newNode);
	void RemoveNode(ndSceneNode* const newNode);

	D_COLLISION_API virtual void UpdateAabb(ndInt32 threadIndex, ndBodyKinematic* const body);
	D_COLLISION_API virtual void UpdateTransformNotify(ndInt32 threadIndex, ndBodyKinematic* const body);
	D_COLLISION_API virtual void CalculateContacts(ndInt32 threadIndex, ndContact* const contact);

	void CalculateJointContacts(ndInt32 threadIndex, ndContact* const contact);
	void ProcessContacts(ndInt32 threadIndex, ndInt32 contactCount, ndContactSolver* const contactSolver);

	void RotateLeft(ndSceneTreeNode* const node, ndSceneNode** const root);
	void RotateRight(ndSceneTreeNode* const node, ndSceneNode** const root);
	ndFloat64 ReduceEntropy(ndFitnessList& fitness, ndSceneNode** const root);
	void ImproveNodeFitness(ndSceneTreeNode* const node, ndSceneNode** const root);
	ndSceneNode* BuildTopDown(ndSceneNode** const leafArray, ndInt32 firstBox, ndInt32 lastBox, ndFitnessList::ndNode** const nextNode);
	ndSceneNode* BuildTopDownBig(ndSceneNode** const leafArray, ndInt32 firstBox, ndInt32 lastBox, ndFitnessList::ndNode** const nextNode);

	D_COLLISION_API void CollisionOnlyUpdate();
	const ndContactArray& GetContactArray() const;

	ndSceneTreeNode* InsertNode(ndSceneNode* const root, ndSceneNode* const node);
	ndContact* FindContactJoint(ndBodyKinematic* const body0, ndBodyKinematic* const body1) const;
	ndJointBilateralConstraint* FindBilateralJoint(ndBodyKinematic* const body0, ndBodyKinematic* const body1) const;

	void UpdateFitness(ndFitnessList& fitness, ndFloat64& oldEntropy, ndSceneNode** const root);
	void AddPair(ndBodyKinematic* const body0, ndBodyKinematic* const body1);
	bool TestOverlaping(const ndBodyKinematic* const body0, const ndBodyKinematic* const body1) const;
	void SubmitPairs(ndSceneNode* const leaftNode, ndSceneNode* const node);

	void BodiesInAabb(ndBodiesInAabbNotify& callback, const ndSceneNode** stackPool, ndInt32 stack) const;
	bool RayCast(ndRayCastNotify& callback, const ndSceneNode** stackPool, ndFloat32* const distance, ndInt32 stack, const ndFastRay& ray) const;
	bool ConvexCast(ndConvexCastNotify& callback, const ndSceneNode** stackPool, ndFloat32* const distance, ndInt32 stack, const ndFastRay& ray, const ndShapeInstance& convexShape, const ndMatrix& globalOrigin, const ndVector& globalDest) const;

	protected:
	D_COLLISION_API ndScene();
	
	D_COLLISION_API void InitBodyArray();
	D_COLLISION_API void UpdateSpecial();
	D_COLLISION_API void UpdateTransform();
	D_COLLISION_API void CalculateContacts();
	D_COLLISION_API void FindCollidingPairs();
	D_COLLISION_API virtual void BalanceScene();
	D_COLLISION_API virtual void ThreadFunction();
	D_COLLISION_API void SendBackgroundJob(ndBackgroundJob* const job);
	

	class ndBodyListRun
	{
		public:
		ndBodyList::ndNode* m_begin;
		ndInt32 m_count;
		ndInt32 m_start;
	};
	
	ndBodyList m_bodyList;
	ndContactArray m_contactArray;

	ndArray<void*> m_scratchBuffer;
	ndArray<ndBodyKinematic*> m_sceneBodyArray;
	ndArray<ndBodyKinematic*> m_activeBodyArray;
	ndArray<ndConstraint*> m_activeConstraintArray;
	ndList<ndBodyKinematic*> m_specialUpdateList;
	ndThreadBackgroundWorker m_backgroundThread;
	ndSpinLock m_lock;
	ndBodyListRun m_bodyListRuns[D_MAX_THREADS_COUNT];
	ndSceneNode* m_rootNode;
	ndBodyKinematic* m_sentinelBody;
	ndContactNotify* m_contactNotifyCallback;
	ndFloat64 m_treeEntropy;
	ndFitnessList m_fitness;
	ndFloat32 m_timestep;
	ndUnsigned32 m_lru;
	ndUnsigned8 m_bodyListChanged;
	ndUnsigned8 m_currentThreadsMem;

	static ndVector m_velocTol;
	static ndVector m_linearContactError2;
	static ndVector m_angularContactError2;

	friend class ndWorld;
	friend class ndRayCastNotify;
	friend class ndConvexCastNotify;
	friend class ndSkeletonContainer;
} D_GCC_NEWTON_ALIGN_32 ;

inline void ndScene::Sync()
{
	ndThreadPool::Sync();
}

inline ndWorld* ndScene::GetWorld() const
{
	return nullptr;
}

inline ndInt32 ndScene::GetThreadCount() const
{
	const ndThreadPool& pool = *this;
	return pool.GetCount();
}

inline const ndBodyList& ndScene::GetBodyList() const
{
	return m_bodyList;
}

inline ndArray<ndConstraint*>& ndScene::GetActiveContactArray()
{
	return m_activeConstraintArray;
}

inline const ndArray<ndConstraint*>& ndScene::GetActiveContactArray() const
{
	return m_activeConstraintArray;
}

inline ndArray<ndBodyKinematic*>& ndScene::GetActiveBodyArray()
{
	return m_activeBodyArray;
}

inline const ndArray<ndBodyKinematic*>& ndScene::GetActiveBodyArray() const
{
	return m_activeBodyArray;
}

inline const ndContactArray& ndScene::GetContactArray() const
{
	return m_contactArray;
}

template <class T>
void ndScene::SubmitJobs(void* const context)
{
	T extJob[D_MAX_THREADS_COUNT];
	ndThreadPoolJob* extJobPtr[D_MAX_THREADS_COUNT];

	const ndInt32 threadCount = GetThreadCount();
	for (ndInt32 i = 0; i < threadCount; i++)
	{
		extJob[i].m_owner = this;
		extJob[i].m_context = context;
		extJob[i].m_timestep = m_timestep;
		extJobPtr[i] = &extJob[i];
	}
	ExecuteJobs(extJobPtr);
}

inline ndFloat32 ndScene::GetTimestep() const
{
	return m_timestep;
}

inline void ndScene::SetTimestep(ndFloat32 timestep)
{
	m_timestep = timestep;
}

inline ndFloat32 ndScene::CalculateSurfaceArea(const ndSceneNode* const node0, const ndSceneNode* const node1, ndVector& minBox, ndVector& maxBox) const
{
	minBox = node0->m_minBox.GetMin(node1->m_minBox);
	maxBox = node0->m_maxBox.GetMax(node1->m_maxBox);
	ndVector side0(maxBox - minBox);
	return side0.DotProduct(side0.ShiftTripleRight()).GetScalar();
}

template <class T, ndInt32 bits, class dEvaluateKey, class dKey>
void ndScene::CountingSort(T* const array, T* const scratchBuffer, ndInt32 elementsCount, ndInt32 digitLocation)
{
	D_TRACKTIME();
	class ndInfo
	{
		public:
		T* m_sourceBuffer;
		T* m_scratchBuffer;
		ndInt32 m_elementCount;
		ndInt32 m_digitNumber;
		ndInt32 m_digitScan[D_MAX_THREADS_COUNT][1 << bits];
	};

	class ndScanDigit : public ndBaseJob
	{
		public:
		ndScanDigit()
		{
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			ndInfo& info = *((ndInfo*)m_context);
			const ndInt32 threadIndex = GetThreadId();
			const ndInt32 threadCount = m_owner->GetThreadCount();
			const ndInt32 stride = info.m_elementCount / threadCount;
			const ndInt32 start = threadIndex * stride;
			const ndInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : info.m_elementCount - start;
			
			const ndInt32 digitCount = 1 << bits;
			const dKey digitMask = digitCount - 1;
			const ndInt32 shiftBits = info.m_digitNumber * bits;

			ndInt32* const digitBuffer = &info.m_digitScan[threadIndex][0];
			memset(digitBuffer, 0, digitCount * sizeof(ndInt32));
						
			const dEvaluateKey evaluator;
			for (ndInt32 i = 0; i < blockSize; i++)
			{
				const T data(info.m_sourceBuffer[i + start]);
				info.m_scratchBuffer[i + start] = data;

				const dKey key = evaluator.GetKey(data);
				const ndInt32 entry = ndInt32 ((key >> shiftBits) & digitMask);
				digitBuffer[entry] ++;
			}
		}
	};

	class ndSortBuffer : public ndBaseJob
	{
		public:
		ndSortBuffer()
		{
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			ndInfo& info = *((ndInfo*)m_context);
			const ndInt32 threadIndex = GetThreadId();
			const ndInt32 threadCount = m_owner->GetThreadCount();
			const ndInt32 stride = info.m_elementCount / threadCount;
			const ndInt32 start = threadIndex * stride;
			const ndInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : info.m_elementCount - start;

			const ndInt32 digitCount = 1 << bits;
			const dKey digitMask = digitCount - 1;
			const ndInt32 shiftBits = info.m_digitNumber * bits;
			ndInt32* const digitBuffer = &info.m_digitScan[threadIndex][0];

			const dEvaluateKey evaluator;
			for (ndInt32 i = 0; i < blockSize; i++)
			{
				const T data(info.m_scratchBuffer[i + start]);
				const dKey key = evaluator.GetKey(data);
				const ndInt32 digitEntry = ndInt32((key >> shiftBits) & digitMask);
				const ndInt32 dstIndex = digitBuffer[digitEntry];
				info.m_sourceBuffer[dstIndex] = data;
				digitBuffer[digitEntry] ++;
			}
		}
	};

	ndInfo info;
	info.m_sourceBuffer = array;
	info.m_scratchBuffer = scratchBuffer;
	info.m_elementCount = elementsCount;
	info.m_digitNumber = digitLocation;

	const ndInt32 threadCount = GetThreadCount();
	SubmitJobs<ndScanDigit>(&info);

	ndInt32 sum = 0;
	const ndInt32 scanSize = 1 << bits;
	for (ndInt32 j = 0; j < scanSize; j++)
	{
		for (ndInt32 i = 0; i < threadCount; i++)
		{
			const ndInt32 count = info.m_digitScan[i][j];
			info.m_digitScan[i][j] = sum;
			sum += count;
		}
	}

	SubmitJobs<ndSortBuffer>(&info);

	#ifdef _DEBUG
		const ndInt32 digitCount = 1 << bits;
		const dKey digitMask = digitCount - 1;
		const ndInt32 shiftBits = info.m_digitNumber * bits;

		const dEvaluateKey evaluator;
		for (ndInt32 i = elementsCount - 1; i; i--)
		{
			const dKey key0 = evaluator.GetKey(array[i - 1]);
			const dKey key1 = evaluator.GetKey(array[i - 0]);
			const ndInt32 digitEntry0 = ndInt32((key0 >> shiftBits) & digitMask);
			const ndInt32 digitEntry1 = ndInt32((key1 >> shiftBits) & digitMask);
			dAssert(digitEntry0 <= digitEntry1);
		}
	#endif
}

inline ndBodyKinematic* ndScene::GetSentinelBody() const
{
	return m_sentinelBody;
}

#endif
