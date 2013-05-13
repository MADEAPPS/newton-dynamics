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

#ifndef _DG_WORLD_H_
#define _DG_WORLD_H_


#include "dgBody.h"
#include "dgContact.h"
#include "dgCollision.h"
#include "dgBroadPhase.h"
#include "dgCollisionScene.h"
#include "dgBodyMasterList.h"
#include "dgWorldDynamicUpdate.h"


#define DG_REDUCE_CONTACT_TOLERANCE			dgFloat32 (1.0e-2f)
#define DG_PRUNE_CONTACT_TOLERANCE			dgFloat32 (1.0e-2f)

#define DG_SLEEP_ENTRIES					8
#define DG_MAX_DESTROYED_BODIES_BY_FORCE	8

class dgBody;
class dgDynamicBody;
class dgKinematicBody;
class dgCollisionPoint;
class dgUserConstraint;
class dgBallConstraint;
class dgOpenclInstance;
class dgHingeConstraint;
class dgUserMeshCreation;
class dgSlidingConstraint;
class dgCollisionInstance;
class dgUpVectorConstraint;
class dgUniversalConstraint;
class dgCorkscrewConstraint;
class dgCollisionDeformableMesh;


class dgBodyCollisionList: public dgTree<const dgCollision*, dgUnsigned32>
{
	public:
	dgBodyCollisionList (dgMemoryAllocator* const allocator)
		:dgTree<const dgCollision*, dgUnsigned32>(allocator)
	{
	}
};

class dgBodyMaterialList: public dgTree<dgContactMaterial, dgUnsigned32>
{
	public:
	dgBodyMaterialList (dgMemoryAllocator* const allocator)
		:dgTree<dgContactMaterial, dgUnsigned32>(allocator)
	{
	}
};

class dgCollisionDeformableMeshList: public dgList<const dgCollisionDeformableMesh*>
{
	public:
	dgCollisionDeformableMeshList (dgMemoryAllocator* const allocator)
		:dgList<const dgCollisionDeformableMesh*>(allocator)
		,m_dictionary(allocator)
	{
	}

	void AddShape(dgCollisionDeformableMesh* const shape)
	{
		dgListNode* const node = Append (shape);
		m_dictionary.Insert (node, shape);
	}

	void RemoveShape(dgCollisionDeformableMesh*  const shape)
	{
		dgTree <dgListNode*, const dgCollisionDeformableMesh*>::dgTreeNode* const node = m_dictionary.Find (shape);
		dgAssert (node);
		Remove (node->GetInfo());
		m_dictionary.Remove (node);
	}

	private:
	dgTree <dgListNode*, const dgCollisionDeformableMesh*> m_dictionary;
};

class dgCollisionParamProxy;

enum dgPerformanceCounters
{
	m_worldTicks = 0,

	m_collisionTicks,
	m_broadPhaceTicks,
	m_narrowPhaseTicks,

	m_dynamicsTicks,
	m_dynamicsBuildSpanningTreeTicks,
	m_dynamicsSolveSpanningTreeTicks,

	m_forceCallback,

	m_preUpdataLister,
	m_postUpdataLister,

	m_counterSize
};

class dgWorld;

typedef dgUnsigned32 (dgApi *OnIslandUpdate) (const dgWorld* const world, void* island, dgInt32 bodyCount);
typedef void (dgApi *OnBodyDestructionByExeciveForce) (const dgBody* const body, const dgContact* joint);
typedef void (dgApi *OnListenerUpdateCallback) (const dgWorld* const world, void* const listenerUserData, dgFloat32 timestep);
typedef void (dgApi *OnListenerDestroyCallback) (const dgWorld* const world, void* const listenerUserData);

typedef void (dgApi *OnBodySerialize) (dgBody& me, dgSerialize funt, void* const serilalizeObject);
typedef void (dgApi *OnBodyDeserialize) (dgBody& me, dgDeserialize funt, void* const serilalizeObject);




class dgSolverSleepTherfesholds
{
	public:
	dgFloat32 m_maxAccel;
	dgFloat32 m_maxAlpha;
	dgFloat32 m_maxVeloc;
	dgFloat32 m_maxOmega;
	dgInt32 m_steps;
};

DG_MSC_VECTOR_ALIGMENT
class dgWorld:
	public dgBodyMasterList,
	public dgBodyMaterialList,
	public dgBodyCollisionList,
	public dgCollisionDeformableMeshList,
	public dgActiveContacts, 
	public dgCollidingPairCollector,
	public dgWorldDynamicUpdate,
	public dgMutexThread,
	public dgAsyncThread,
	public dgThreadHive
{
	public:
	class dgDetroyBodyByForce
	{	
		public:
		dgDetroyBodyByForce()
			:m_count(0)
		{
		}
		dgInt32 m_count;
		dgFloat32 m_force[DG_MAX_DESTROYED_BODIES_BY_FORCE];
		const dgBody* m_bodies[DG_MAX_DESTROYED_BODIES_BY_FORCE];
		const dgContact* m_joint[DG_MAX_DESTROYED_BODIES_BY_FORCE];
	};

	class dgListener
	{
		public: 
		
		~dgListener()
		{
			if (m_onListenerDestroy) {
				m_onListenerDestroy(m_world, m_userData);
			}
		}

		char m_name[32];
		dgWorld* m_world;
		void* m_userData;
		OnListenerUpdateCallback m_onListenerUpdate;
		OnListenerDestroyCallback m_onListenerDestroy;
	};

	class dgListenerList: public dgList <dgListener>
	{
		public: 
		dgListenerList (dgMemoryAllocator* const allocator)
			:dgList <dgListener>(allocator)
		{
		}

		~dgListenerList()
		{
		}
	};

	DG_CLASS_ALLOCATOR(allocator)

	dgWorld(dgMemoryAllocator* const allocator);
	~dgWorld();

	dgBroadPhase* GetBroadPhase() const;

	void SetSolverMode (dgInt32 mode);
	void SetFrictionMode (dgInt32 mode);

	dgInt32 EnumerateCPUHardwareModes() const;
	dgInt32 EnumerateHardwareModes() const;
	dgInt32 GetCurrentHardwareMode() const;
	void SetCurrentHardwareMode(dgInt32 deviceIndex);
	void GetHardwareVendorString (dgInt32 deviceIndex, char* const description, dgInt32 maxlength) const;


	void EnableThreadOnSingleIsland(dgInt32 mode);
	dgInt32 GetThreadOnSingleIsland() const;

	void FlushCache();
	
	void* GetUserData() const;
	void SetUserData (void* const userData);

	void Update (dgFloat32 timestep);
	void UpdateAsync (dgFloat32 timestep);
	
	void StepDynamics (dgFloat32 timestep);
	
	dgInt32 Collide (const dgCollisionInstance* const collisionA, const dgMatrix& matrixA, 
					 const dgCollisionInstance* const collisionB, const dgMatrix& matrixB, 
					 dgTriplex* const points, dgTriplex* const normals, dgFloat32* const penetration, 
					 dgInt64* const attibuteA, dgInt64* const attibuteB, dgInt32 maxContacts, dgInt32 threadIndex);

	dgInt32 CollideContinue (const dgCollisionInstance* const collisionA, const dgMatrix& matrixA, const dgVector& velocA, const dgVector& omegaA, 
							 const dgCollisionInstance* const collisionB, const dgMatrix& matrixB, const dgVector& velocB, const dgVector& omegaB, 
		                     dgFloat32& timeStep, dgTriplex* const points, dgTriplex* const normals, dgFloat32* const penetration, 
							 dgInt64* const attibuteA, dgInt64* const attibuteB, dgInt32 maxContacts, dgInt32 threadIndex);

	dgInt32 CollideContinueSimd (const dgCollisionInstance* const collisionA, const dgMatrix& matrixA, const dgVector& velocA, const dgVector& omegaA, 
								const dgCollisionInstance* const collisionB, const dgMatrix& matrixB, const dgVector& velocB, const dgVector& omegaB, 
								dgFloat32& timeStep, dgTriplex* const points, dgTriplex* const normals, dgFloat32* const penetration, 
								dgInt64* const attibuteA, dgInt64* const attibuteB, dgInt32 maxContacts, dgInt32 threadIndex);


	dgInt32 ClosestPoint (dgTriplex& point, const dgCollisionInstance* const collision, const dgMatrix& matrix, dgTriplex& contact, dgTriplex& normal, dgInt32 threadIndex);
	dgInt32 ClosestPoint (const dgCollisionInstance* const collisionA, const dgMatrix& matrixA, 
						  const dgCollisionInstance* const collisionB, const dgMatrix& matrixB, 
						  dgTriplex& contactA, dgTriplex& contactB, dgTriplex& normalAB, dgInt32 threadIndex);


	void SetFrictionThreshold (dgFloat32 acceletion);


	dgBody* GetIslandBody (const void* const island, dgInt32 index) const;


	void* GetListenerUserData (void* const lintener) const;
	void* FindPreListener (const char* const nameid) const;
	void* FindPostListener (const char* const nameid) const;
	void* AddPreListener (const char* const nameid, void* const userData, OnListenerUpdateCallback updateCallback, OnListenerDestroyCallback destroyCallback);
	void* AddPostListener (const char* const nameid, void* const userData, OnListenerUpdateCallback updateCallback, OnListenerDestroyCallback destroyCallback);


	void SetIslandUpdateCallback (OnIslandUpdate callback); 
	void SetLeavingWorldCallback (OnLeavingWorldAction callback); 
	void SetBodyDestructionByExeciveForce (OnBodyDestructionByExeciveForce callback); 

	void InitBody (dgBody* const body, dgCollisionInstance* const collision, const dgMatrix& matrix);
	dgDynamicBody* CreateDynamicBody (dgCollisionInstance* const collision, const dgMatrix& matrix);
	dgKinematicBody* CreateKinematicBody (dgCollisionInstance* const collision, const dgMatrix& matrix);
	dgBody* CreateDeformableBody (dgCollisionInstance* const collision, const dgMatrix& matrix);
	void DestroyBody(dgBody* const body);
	void DestroyAllBodies ();

//	void AddToBreakQueue (const dgContact* const contactJoint, dgBody* const body, dgFloat32 maxForce);

    // modify the velocity and angular velocity of a body in such a way 
	// that the velocity of pointPosit is increase by pointDeltaVeloc 
	// pointVeloc and pointPosit are in world space
//	void AddBodyImpulse (dgBody* body, const dgVector& pointDeltaVeloc, const dgVector& pointPosit);
//	void ApplyImpulseArray (dgBody* body, dgInt32 count, dgInt32 strideInBytes, const dgFloat32* const impulseArray, const dgFloat32* const pointArray);

	// apply the transform matrix to the body and recurse trough all bodies attached to this body with a 
	// bilateral joint contact joint are ignored.
	void BodySetMatrix (dgBody* const body, const dgMatrix& matrix);
	
	dgInt32 GetBodiesCount() const;
	dgInt32 GetConstraintsCount() const;

    dgCollisionInstance* CreateInstance (const dgCollision* const child, dgInt32 shapeID, const dgMatrix& offsetMatrix);

	dgCollisionInstance* CreateNull ();
	dgCollisionInstance* CreateSphere (dgFloat32 radiusdg, dgInt32 shapeID, const dgMatrix& offsetMatrix = dgGetIdentityMatrix());
	dgCollisionInstance* CreateCone (dgFloat32 radius, dgFloat32 height, dgInt32 shapeID, const dgMatrix& offsetMatrix = dgGetIdentityMatrix());
	dgCollisionInstance* CreateCapsule (dgFloat32 radius, dgFloat32 height, dgInt32 shapeID, const dgMatrix& offsetMatrix = dgGetIdentityMatrix());
	dgCollisionInstance* CreateCylinder (dgFloat32 radius, dgFloat32 height, dgInt32 shapeID, const dgMatrix& offsetMatrix = dgGetIdentityMatrix());
	dgCollisionInstance* CreateBox (dgFloat32 dx, dgFloat32 dy, dgFloat32 dz, dgInt32 shapeID, const dgMatrix& offsetMatrix = dgGetIdentityMatrix());
	dgCollisionInstance* CreateConvexHull (dgInt32 count, const dgFloat32* const points, dgInt32 strideInBytes, dgFloat32 thickness, dgInt32 shapeID, const dgMatrix& offsetMatrix = dgGetIdentityMatrix());
	dgCollisionInstance* CreateTaperedCapsule (dgFloat32 radio0, dgFloat32 radio1, dgFloat32 height, dgInt32 shapeID, const dgMatrix& offsetMatrix = dgGetIdentityMatrix());
	dgCollisionInstance* CreateTaperedCylinder (dgFloat32 radio0, dgFloat32 radio1, dgFloat32 height, dgInt32 shapeID, const dgMatrix& offsetMatrix = dgGetIdentityMatrix());
	dgCollisionInstance* CreateChamferCylinder (dgFloat32 radius, dgFloat32 height, dgInt32 shapeID, const dgMatrix& offsetMatrix = dgGetIdentityMatrix());
	dgCollisionInstance* CreateCollisionCompound ();

	
	dgCollisionInstance* CreateDeformableMesh (dgMeshEffect* const mesh, dgInt32 shapeID);
	dgCollisionInstance* CreateClothPatchMesh (dgMeshEffect* const mesh, dgInt32 shapeID, const dgClothPatchMaterial& structuralMaterial, const dgClothPatchMaterial& bendMaterial);


//	dgCollision* CreateCollisionCompoundBreakable (dgInt32 count, const dgMeshEffect* const solidArray[], const dgInt32* const idArray,  const dgFloat32* const densities, const dgInt32* const internalFaceMaterial, dgInt32 debriID, dgFloat32 gap);


	dgCollisionInstance* CreateBVH ();	
	dgCollisionInstance* CreateStaticUserMesh (const dgVector& boxP0, const dgVector& boxP1, const dgUserMeshCreation& data);
	dgCollisionInstance* CreateHeightFieldCollision(dgInt32 width, dgInt32 height, dgInt32 contructionMode, 
										 const dgFloat32* const elevationMap, const dgInt8* const atributeMap, dgFloat32 horizontalScale);
	dgCollisionInstance* CreateScene ();	

	static void OnSerializeToFile (void* const userData, const void* const buffer, size_t size);
	static void OnBodySerializeToFile (dgBody& body, dgSerialize serializeCallback, void* const userData);

	void SerializeToFile (const char* const fileName) const;
	void SerializeBodyArray (dgBody** const array, dgInt32 count, OnBodySerialize bodyCallback, dgSerialize serializeCallback, void* const userData) const;
	void DeserializeBodyArray (OnBodyDeserialize bodyCallback, dgDeserialize deserializeCallback, void* const userData);

	void SerializeCollision (dgCollisionInstance* const shape, dgSerialize deserialization, void* const userData) const;
	dgCollisionInstance* CreateCollisionFromSerialization (dgDeserialize deserialization, void* const userData);
	void ReleaseCollision(const dgCollision* const collision);
	
	dgUpVectorConstraint* CreateUpVectorConstraint (const dgVector& pin, dgBody *body);
	
	dgBallConstraint* CreateBallConstraint (const dgVector& pivot, dgBody* const body0, dgBody *refBody = NULL);
	dgHingeConstraint* CreateHingeConstraint (const dgVector& pivot, const dgVector& pin, dgBody* const body0, dgBody *refBody = NULL);
	dgSlidingConstraint* CreateSlidingConstraint (const dgVector& pivot, const dgVector& pin, dgBody* const body0, dgBody *refBody = NULL);
	dgCorkscrewConstraint* CreateCorkscrewConstraint (const dgVector& pivot, const dgVector& pin, dgBody* const body0, dgBody *refBody = NULL);
	dgUniversalConstraint* CreateUniversalConstraint (const dgVector& pivot, const dgVector& pin0, const dgVector& pin1, dgBody* const body0, dgBody *body1 = NULL);


	void DestroyConstraint (dgConstraint* constraint);
	dgUnsigned32 CreateBodyGroupID();
	void RemoveAllGroupID();

	dgUnsigned32 GetDefualtBodyGroupID() const;
	dgContactMaterial* GetMaterial (dgUnsigned32 bodyGroupId0, dgUnsigned32 bodyGroupId1) const;

	dgContactMaterial* GetFirstMaterial () const;
	dgContactMaterial* GetNextMaterial (dgContactMaterial* material) const;

	OnGetPerformanceCountCallback GetPerformaceFuntion()const ;
	void SetPerfomanceCounter(OnGetPerformanceCountCallback callback);
	

	void SetThreadsCount (dgInt32 count);
	dgUnsigned32 GetPerfomanceTicks (dgUnsigned32 entry) const;
	dgUnsigned32 GetThreadPerfomanceTicks (dgUnsigned32 threadIndex) const;

	//Parallel Job dispatcher for user related stuff
	void ExecuteUserJob (dgWorkerThreadTaskCallback userJobKernel, void* const userJobKernelContext);

	dgBody* GetSentinelBody() const;
	dgMemoryAllocator* GetAllocator() const;

	void Sync ();
	
	private:
	
	void CalculateContacts (dgCollidingPairCollector::dgPair* const pair, dgFloat32 timestep, bool ccdMode, dgInt32 threadIndex);
	void CalculateContactsSimd (dgCollidingPairCollector::dgPair* const pair, dgFloat32 timestep, bool ccdMode, dgInt32 threadIndex);

	dgInt32 PruneContacts (dgInt32 count, dgContactPoint* const contact, dgInt32 maxCount = (DG_CONSTRAINT_MAX_ROWS / 3)) const;
	dgInt32 ReduceContacts (dgInt32 count, dgContactPoint* const contact, dgInt32 maxCount, dgFloat32 tol, dgInt32 arrayIsSorted = 0) const;
	
//	dgInt32 CalculateHullToHullContacts (dgCollisionParamProxy& proxy) const;
//	dgInt32 CalculateHullToHullContactsSimd (dgCollisionParamProxy& proxy) const;
//	void PopulateContacts (dgContact* const contact, dgCollidingPairCollector::dgPair* const pair, dgFloat32 timestep, dgInt32 threadIndex);	
//	void SceneContactsSimd (const dgCollisionScene::dgProxy& sceneProxy, dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const;
//	dgInt32 CalculateConicConvexToHullContacts (dgCollisionParamProxy& proxy) const;
//	dgInt32 CalculateConvexToConvexContinuesContacts (dgFloat32& timestep, dgBody* body1, dgBody* body2, dgContactPoint contactOut[]) const; 
//	dgInt32 CalculateConvexToConvexContinuesContacts (dgCollisionParamProxy& proxy) const; 
//	dgInt32 CalculateConvexToConvexContinuesContacts (dgCollisionParamProxy& proxy) const; 
//	dgInt32 CalculateConvexToConvexContacts (dgFloat32& timestep, dgBody* conv1, dgBody* conv2, dgFloat32 penetrationPadding, dgContactPoint* const contact) const;
//	dgInt32 CalculateConvexToConvexContactsSimd (dgFloat32& timestep, dgBody* conv1, dgBody* conv2, dgFloat32 penetrationPadding, dgContactPoint* const contact) const;
//	dgInt32 CalculateConvexToNonConvexContacts (dgFloat32& timestep, dgBody* conv, dgBody* nConv, dgContactPoint* const contact, dgInt32 maxContacts) const;
//	dgInt32 CalculatecConvexConvexCastContacts (dgCollisionParamProxy& proxy, bool& algorithmSuccessful) const;
	
	dgInt32 ValidateContactCache (dgContact* const contact) const;
	dgInt32 ValidateContactCacheSimd (dgContact* const contact) const;

	dgInt32 CalculateConvexPolygonToHullContactsDescrete (dgCollisionParamProxy& proxy) const;

	dgInt32 CalculatePolySoupToHullContactsDescrete (dgCollisionParamProxy& proxy) const;
	dgInt32 CalculatePolySoupToHullContactsDescreteSimd (dgCollisionParamProxy& proxy) const;

	dgInt32 CalculateConvexToNonConvexContactsContinue (dgCollisionParamProxy& proxy) const;
	dgInt32 CalculateConvexToNonConvexContactsContinueSimd (dgCollisionParamProxy& proxy) const;
	
	dgInt32 CalculateConvexToConvexContacts (dgCollisionParamProxy& proxy) const;
	dgInt32 CalculateConvexToConvexContactsSimd (dgCollisionParamProxy& proxy) const;

	dgInt32 CalculateConvexToNonConvexContacts (dgCollisionParamProxy& proxy) const;
	dgInt32 CalculateConvexToNonConvexContactsSimd (dgCollisionParamProxy& proxy) const;

	//dgInt32 FilterPolygonDuplicateContacts (dgInt32 count, dgContactPoint* const contact) const;
	
	void PopulateContacts (dgCollidingPairCollector::dgPair* const pair, dgFloat32 timestep, dgInt32 threadIndex);	
	void ProcessContacts (dgCollidingPairCollector::dgPair* const pair, dgFloat32 timestep, dgInt32 threadIndex);
	void ProcessDeformableContacts (dgCollidingPairCollector::dgPair* const pair, dgFloat32 timestep, dgInt32 threadIndex);
	void ProcessCachedContacts (dgContact* const contact, dgFloat32 timestep, dgInt32 threadIndex) const;

	void ConvexContacts (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const;
	void ConvexContactsSimd (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const;
	void CompoundContacts (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const;
	void CompoundContactsSimd (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const;
	void DeformableContacts (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const;
	void DeformableContactsSimd (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const;

	void SceneContacts (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const;
	void SceneContactsSimd (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const;

	void SceneChildContacts (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const;


	dgFloat32 CalculateTimeToImpact (dgContact* const contactJoint, dgFloat32 timestep, dgInt32 threadIndex, dgVector& p, dgVector& q, dgVector& normal) const;
	dgInt32 ClosestPoint (dgCollisionParamProxy& proxy) const;
	dgInt32 ClosestCompoundPoint (dgBody* const compoundConvexA, dgBody* const collisionB, dgTriplex& contactA, dgTriplex& contactB, dgTriplex& normalAB, dgInt32 threadIndex) const;

	bool AreBodyConnectedByJoints (dgBody* const origin, dgBody* const target);
	
	void AddSentinelBody();
	void InitConvexCollision ();
	static dgUnsigned32 dgApi GetPerformanceCount ();

	virtual void Execute (dgInt32 threadID);
	virtual void TickCallback (dgInt32 threadID);


	dgUnsigned32 m_dynamicsLru;
	dgUnsigned32 m_inUpdate;
	dgUnsigned32 m_solverMode;
	dgUnsigned32 m_frictionMode;
	dgUnsigned32 m_bodyGroupID;
	dgUnsigned32 m_defualtBodyGroupID;
	dgUnsigned32 m_bodiesUniqueID;
	dgUnsigned32 m_useParallelSolver;
	dgUnsigned32 m_genericLRUMark;

	dgFloat32 m_freezeAccel2;
	dgFloat32 m_freezeAlpha2;
	dgFloat32 m_freezeSpeed2;
	dgFloat32 m_freezeOmega2;
	dgFloat32 m_frictiomTheshold;
	dgFloat32 m_savetimestep;

	dgSolverSleepTherfesholds m_sleepTable[DG_SLEEP_ENTRIES];
	bool m_collisionSwapPriority[m_nullCollision][m_nullCollision];
	
	dgOpenclInstance* m_openCL;
	dgBroadPhase* m_broadPhase; 
	dgDynamicBody* m_sentionelBody;
	dgCollisionInstance* m_pointCollision;
	
	void* m_userData;
	dgMemoryAllocator* m_allocator;
	dgCpuClass m_cpu;
	dgInt32 m_hardwaredIndex;
	OnIslandUpdate m_islandUpdate;
	OnLeavingWorldAction m_leavingWorldNotify;
	OnGetPerformanceCountCallback m_getPerformanceCount;
	OnBodyDestructionByExeciveForce m_destroyBodyByExeciveForce;

	dgDetroyBodyByForce m_destroyeddBodiesPool;

	
	dgUnsigned32 m_perfomanceCounters[m_counterSize];	
	dgUnsigned32 m_perfomanceCountersBack[m_counterSize];	

	dgListenerList m_preListener;
	dgListenerList m_postListener;
	dgTree<void*, unsigned> m_perInstanceData;
	dgArray<dgUnsigned8> m_islandMemory; 
	dgArray<dgUnsigned8> m_bodiesMemory; 
	dgArray<dgUnsigned8> m_jointsMemory; 
	dgArray<dgUnsigned8> m_pairMemoryBuffer;
	dgArray<dgUnsigned8> m_internalForcesMemory;  
	dgArray<dgUnsigned8> m_solverMatrixMemory;  
	
	
	friend class dgBody;
	friend class dgDeformableBody;
	friend class dgActiveContacts;

	friend class dgUserConstraint;
	friend class dgBodyMasterList;
	friend class dgJacobianMemory;
	friend class dgCollisionScene;
	friend class dgCollisionConvex;
	friend class dgCollisionCompound;
	friend class dgWorldDynamicUpdate;
	friend class dgParallelSolverClear;	
	friend class dgParallelSolverSolve;
	friend class dgBroadPhase;
	friend class dgCollisionHeightField;
	friend class dgSolverWorlkerThreads;
	friend class dgCollisionConvexPolygon;
	friend class dgCollidingPairCollector;
	friend class dgCollisionDeformableMesh;
	friend class dgParallelSolverUpdateForce;
	friend class dgParallelSolverUpdateVeloc;
	friend class dgParallelSolverBodyInertia;
	friend class dgParallelSolverInitFeedbackUpdate;
	friend class dgBroadPhaseApplyExternalForce;
	friend class dgParallelSolverCalculateForces;
	friend class dgParallelSolverJointAcceleration;
	friend class dgParallelSolverBuildJacobianRows;
	friend class dgParallelSolverInitInternalForces;
	friend class dgParallelSolverBuildJacobianMatrix;
	
	friend class dgBroadPhaseMaterialCallbackWorkerThread;
	friend class dgBroadPhaseCalculateContactsWorkerThread;
} DG_GCC_VECTOR_ALIGMENT ;


inline dgMemoryAllocator* dgWorld::GetAllocator() const
{
	return m_allocator;
}

inline dgBroadPhase* dgWorld::GetBroadPhase() const
{
	return m_broadPhase;
}

/*
inline void dgWorld::AddToBreakQueue (const dgContact* const contactJoint, dgBody* const body, dgFloat32 maxForce)
{
	if (m_destroyeddBodiesPool.m_count < DG_MAX_DESTROYED_BODIES_BY_FORCE) {
		if (body->m_isInDestructionArrayLRU != body->m_dynamicsLru) {
			body->m_isInDestructionArrayLRU = body->m_dynamicsLru;
			m_destroyeddBodiesPool.m_force[m_destroyeddBodiesPool.m_count] = maxForce;
			m_destroyeddBodiesPool.m_bodies[m_destroyeddBodiesPool.m_count] = body;
			m_destroyeddBodiesPool.m_joint[m_destroyeddBodiesPool.m_count] = contactJoint;
			m_destroyeddBodiesPool.m_count ++;
		} else {
			for (dgInt32 i = 0; i < m_destroyeddBodiesPool.m_count; i ++) {
				if (m_destroyeddBodiesPool.m_bodies[i] == body) {
					if (maxForce > m_destroyeddBodiesPool.m_force[i]) {
						m_destroyeddBodiesPool.m_force[i] = maxForce;
						m_destroyeddBodiesPool.m_joint[i] = contactJoint;
					}
				}
			}
		}
	}
}
*/




#endif 

