/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
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

#include "dgPhysicsStdafx.h"
#include "dgWorld.h"

#include "dgDynamicBody.h"
#include "dgKinematicBody.h"
#include "dgCollisionBox.h"
#include "dgKinematicBody.h"
#include "dgCollisionNull.h"
#include "dgCollisionCone.h"
#include "dgCollisionScene.h"
#include "dgCollisionSphere.h"
#include "dgCollisionCapsule.h"
#include "dgBroadPhaseDefault.h"
#include "dgCollisionInstance.h"
#include "dgCollisionCompound.h"
#include "dgWorldDynamicUpdate.h"
#include "dgCollisionConvexHull.h"
#include "dgBroadPhasePersistent.h"
#include "dgCollisionChamferCylinder.h"

#include "dgUserConstraint.h"
#include "dgBallConstraint.h"
#include "dgHingeConstraint.h"
#include "dgSkeletonContainer.h"
#include "dgSlidingConstraint.h"
#include "dgUpVectorConstraint.h"
#include "dgUniversalConstraint.h"
#include "dgCorkscrewConstraint.h"


#define DG_SOLVER_CONVERGENCE_COUNT			4
#define DG_DEFAULT_SOLVER_ITERATION_COUNT	4

#define DG_INITIAL_BODIES_SIZE		(1024 * 4)
#define DG_INITIAL_JOINTS_SIZE		(1024 * 4)
#define DG_INITIAL_JACOBIAN_SIZE	(1024 * 16)



/*
static  char *xxx[10] = {"bbbbbbbbbb",
						 "baaaaaaaab",
						 "babbbbbaab",
					 	 "babaaabaab",
						 "baaababaab",
						 "bbbaaabaab",
						 "bbbbaabaab",
						 "babbbbbaab",
						 "baaaaaaaab",
						 "bbbbbbbbbb"};

struct myPath: public dgPathFinder<dgInt32, float> 
{
	dgInt32 goalx;
	dgInt32 goaly;
	myPath ()
		:dgPathFinder<dgInt32, float>(1024, 10)
	{
		for (int i = 0; i < 10; i ++) {
			strcpy (&m_map[i][0], xxx[i]);
		}
	}

	const dgPathNode<dgInt32, float>* CalCulatePath (dgInt32 source, dgInt32 goal)
	{
		goalx = goal % 10;
		goaly = goal / 10;
		return dgPathFinder<dgInt32, float>::CalCulatePath (source, goal) ;
	}


	float GetCostFromParent(const dgPathNode<dgInt32, float>& node) const
	{
		dgInt32 x;
		dgInt32 y;
		dgInt32 x0;
		dgInt32 y0;
		dgInt32 x1;
		dgInt32 y1;
		dgInt32 id;

		id = node.GetId();
		x = id % 10;
		y = id / 10;

		const dgPathNode<dgInt32, float>* parent = node.GetParent();
		id = parent->GetId(); 
		x0 = id % 10;
		y0 = id / 10;

		const dgPathNode<dgInt32, float>* grandParent = parent->GetParent();
		x1 = 2 * x0 - x;
		y1 = 2 * y0 - y;
		if (grandParent) {
			id = grandParent->GetId(); 
			x1 = id % 10;
			y1 = id / 10;
		}

		dgInt32 dx0;
		dgInt32 dy0;
		dgInt32 dx1;
		dgInt32 dy1;
		float penalty;

		dx0 = x0 - x;
		dy0 = y0 - y;
		dx1 = x1 - x0;
		dy1 = y1 - y0;
		penalty = 0.0f;
		if (dx1 * dy0 - dx0 * dy1)	{
			penalty = dgFloat32(1.0f);
		}

		static dgInt32 xxxx;
		if (!xxxx){
			xxxx = 1;
			penalty = 9.1f;
		}

		return (xxx[y][x] == 'a') ? (dgFloat32(1.0f) + penalty): 50.0f;
	}

	float GetEstimatedCostToGoal(dgInt32 id) const
	{
		dgInt32 x;
		dgInt32 y;

		x = id % 10 - goalx;
		y = id / 10 - goaly;
		return dgSqrt ((float)(x * x + y * y));
	}

	dgInt32 EnumerateChildren(dgInt32 parent, dgInt32 array[]) const
	{
		dgInt32 x;
		dgInt32 y;

		x = parent % 10;
		y = parent / 10;

		array[0]	= (y - 1) * 10 + x;
		array[1]	= (y - 0) * 10 + x - 1;
		array[2]	= (y + 1) * 10 + x;
		array[3]	= (y + 0) * 10 + x + 1;
		return 4;
	}

	char m_map[20][20];
};


void xxxxx()
{
	myPath path; 
	const dgPathNode<dgInt32, float>* firtNode;
	for (firtNode = path.CalCulatePath (5 * 10 + 3, 4 * 10 + 8); firtNode; firtNode= firtNode->GetNext()) {
		dgInt32 id;
		dgInt32 x;
		dgInt32 y;

		id = firtNode->GetId();
		x = id % 10;
		y = id / 10;
		path.m_map[y][x] = '_';
	}
}
*/

/*
dgWorld::dgAsyncUpdate::dgAsyncUpdate ()
	:dgAsyncUpdateThread("Newton AsynUpdate", 1)
{
}

void dgWorld::dgAsyncUpdate::Update ()
{
}
*/

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define DG_MUTEX_THREAD_ID	0
#define DG_ASYNC_THREAD_ID	1

dgWorld::dgWorld(dgMemoryAllocator* const allocator, dgInt32 stackSize)
	:dgBodyMasterList(allocator)
	,dgBodyMaterialList(allocator)
	,dgBodyCollisionList(allocator)
//	,dgDeformableBodiesUpdate(allocator)
	,dgSkeletonList(allocator)
	,dgActiveContacts(allocator) 
	,dgWorldDynamicUpdate()
	,dgMutexThread("newtonSyncThread", DG_MUTEX_THREAD_ID, stackSize)
	,dgAsyncThread("newtonAsyncThread", DG_ASYNC_THREAD_ID)
	,dgWorldThreadPool(allocator)
	,dgDeadBodies(allocator)
	,dgDeadJoints(allocator)
	,m_broadPhase(NULL)
	,m_sentinelBody(NULL)
	,m_pointCollision(NULL)
	,m_preListener(allocator)
	,m_postListener(allocator)
	,m_perInstanceData(allocator)
	,m_bodiesMemory (DG_INITIAL_BODIES_SIZE, allocator, 64)
	,m_jointsMemory (DG_INITIAL_JOINTS_SIZE, allocator, 64)
	,m_solverJacobiansMemory (DG_INITIAL_JACOBIAN_SIZE, allocator, 64)
	,m_solverForceAccumulatorMemory (DG_INITIAL_BODIES_SIZE, allocator, 64)
	,m_clusterMemory (DG_INITIAL_BODIES_SIZE, allocator, 64)
{
	dgMutexThread* const mutexThread = this;
	SetMatertThread (mutexThread);

	m_savetimestep = dgFloat32 (0.0f);
	m_allocator = allocator;
	m_clusterUpdate = NULL;
	m_getDebugTime = NULL;

	m_onCollisionInstanceDestruction = NULL;
	m_onCollisionInstanceCopyConstrutor = NULL;

	m_serializedJointCallback = NULL;	
	m_deserializedJointCallback = NULL;	

	m_inUpdate = 0;
	m_bodyGroupID = 0;
	
	m_defualtBodyGroupID = CreateBodyGroupID();
	m_genericLRUMark = 0;
	m_delayDelateLock = 0;
	m_clusterLRU = 0;

	m_useParallelSolver = 0;

	m_solverMode = DG_DEFAULT_SOLVER_ITERATION_COUNT;
	m_dynamicsLru = 0;
	m_numberOfSubsteps = 1;
		
	m_bodiesUniqueID = 0;
	m_frictiomTheshold = dgFloat32 (0.25f);

	m_userData = NULL;
	m_clusterUpdate = NULL;

	m_freezeAccel2 = DG_FREEZE_MAG2;
	m_freezeAlpha2 = DG_FREEZE_MAG2;
	m_freezeSpeed2 = DG_FREEZE_MAG2 * dgFloat32 (0.1f);
	m_freezeOmega2 = DG_FREEZE_MAG2 * dgFloat32 (0.1f);

	m_contactTolerance = DG_PRUNE_CONTACT_TOLERANCE;
	m_solverConvergeQuality = DG_SOLVER_CONVERGENCE_COUNT;

	dgInt32 steps = 1;
	dgFloat32 freezeAccel2 = m_freezeAccel2;
	dgFloat32 freezeAlpha2 = m_freezeAlpha2;
	dgFloat32 freezeSpeed2 = m_freezeSpeed2;
	dgFloat32 freezeOmega2 = m_freezeOmega2;
	for (dgInt32 i = 0; i < DG_SLEEP_ENTRIES; i ++) {
		m_sleepTable[i].m_maxAccel = freezeAccel2;
		m_sleepTable[i].m_maxAlpha = freezeAlpha2;
		m_sleepTable[i].m_maxVeloc = freezeSpeed2;
		m_sleepTable[i].m_maxOmega = freezeOmega2;
		m_sleepTable[i].m_steps = steps;
		steps += 7;
		freezeAccel2 *= dgFloat32 (1.5f);
		freezeAlpha2 *= dgFloat32 (1.4f);
		freezeSpeed2 *= dgFloat32 (1.5f);
		freezeOmega2 *= dgFloat32 (1.5f);
	}

	steps += 300;
	m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxAccel *= dgFloat32 (100.0f);
	m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxAlpha *= dgFloat32 (100.0f);
	m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxVeloc = 0.25f;
	m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxOmega = 0.1f;
	m_sleepTable[DG_SLEEP_ENTRIES - 1].m_steps = steps;

	m_hardwaredIndex = 0;
	SetThreadsCount (0);

	m_broadPhase = new (allocator) dgBroadPhaseDefault(this);
	//m_broadPhase = new (allocator) dgBroadPhasePersistent(this);

	//m_pointCollision = new (m_allocator) dgCollisionPoint(m_allocator);
	dgCollision* const pointCollison = new (m_allocator) dgCollisionPoint(m_allocator);
	m_pointCollision = CreateInstance(pointCollison, 0, dgGetIdentityMatrix());
	pointCollison->Release();

	AddSentinelBody();
}

dgWorld::~dgWorld()
{	
	Sync();
	dgAsyncThread::Terminate();
	dgMutexThread::Terminate();

	dgSkeletonList::Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		delete iter.GetNode()->GetInfo();
	}

	m_preListener.RemoveAll();
	m_postListener.RemoveAll();

	DestroyAllBodies();
	RemoveAllGroupID();
	m_pointCollision->Release();
	DestroyBody (m_sentinelBody);

	delete m_broadPhase;
}

void dgWorld::SetThreadsCount (dgInt32 count)
{
	dgThreadHive::SetThreadsCount(count);
}

dgUnsigned32 dgWorld::GetPerformanceCount ()
{
	return 0;
}

void dgWorld::AddSentinelBody()
{
	dgCollision* const collision = new  (m_allocator) dgCollisionNull (m_allocator, 0x4352fe67);
	dgCollisionInstance* const instance = CreateInstance(collision, 0, dgGetIdentityMatrix());
	collision->Release();
	m_sentinelBody = CreateDynamicBody(instance, dgGetIdentityMatrix());
	instance->Release();
}

dgBody* dgWorld::GetSentinelBody() const
{
	return m_sentinelBody;
}


void dgWorld::SetSolverMode (dgInt32 mode)
{
	m_solverMode = dgUnsigned32 (dgMax (1, mode));
}


void dgWorld::SetSolverConvergenceQuality (dgInt32 mode)
{
	m_solverConvergeQuality = mode ? DG_SOLVER_CONVERGENCE_COUNT : 2 * DG_SOLVER_CONVERGENCE_COUNT;
}

dgInt32 dgWorld::EnumerateHardwareModes() const
{
	dgInt32 count = 1;
	return count;
}

void dgWorld::GetHardwareVendorString (dgInt32 deviceIndex, char* const description, dgInt32 maxlength) const
{
	deviceIndex = dgClamp(deviceIndex, 0, EnumerateHardwareModes() - 1);
	if (deviceIndex == 0) {
		sprintf (description, "newton cpu");
	}
}

void dgWorld::SetCurrentHardwareMode(dgInt32 deviceIndex)
{
	m_hardwaredIndex = dgClamp(deviceIndex, 0, EnumerateHardwareModes() - 1);
}


dgFloat32 dgWorld::GetContactMergeTolerance() const
{
	return m_contactTolerance;
}

void dgWorld::SetContactMergeTolerance(dgFloat32 tolerenace)
{
	m_contactTolerance = dgMax (tolerenace, dgFloat32 (1.e-3f));
}


dgInt32 dgWorld::GetCurrentHardwareMode() const
{
	return m_hardwaredIndex;
}

void dgWorld::EnableThreadOnSingleIsland(dgInt32 mode)
{
	m_useParallelSolver = mode ? 1 : 0;
}

dgInt32 dgWorld::GetThreadOnSingleIsland() const
{
	return m_useParallelSolver ? 1 : 0;
}


void dgWorld::SetFrictionThreshold (dgFloat32 acceleration)
{
	m_frictiomTheshold = dgMax (dgFloat32(1.0e-2f), acceleration);
}


void dgWorld::RemoveAllGroupID()
{
	while (dgBodyMaterialList::GetCount()) {
		dgBodyMaterialList::Remove (dgBodyMaterialList::GetRoot());
	}
	m_bodyGroupID = 0;
	m_defualtBodyGroupID = CreateBodyGroupID();
}

void dgWorld::DestroyAllBodies ()
{
	dgBodyMasterList& me = *this;

	Sync ();

	while (m_disableBodies.GetRoot()) {
		dgBody* const body = m_disableBodies.GetRoot()->GetKey();
		BodyEnableSimulation(body);
	}

	dgAssert (dgBodyMasterList::GetFirst()->GetInfo().GetBody() == m_sentinelBody);
	for (dgBodyMasterList::dgListNode* node = me.GetFirst()->GetNext(); node; ) {
		dgBody* const body = node->GetInfo().GetBody();
		node = node->GetNext();
		DestroyBody (body);
	}

	dgAssert (me.GetFirst()->GetInfo().GetCount() == 0);
	dgAssert (dgBodyCollisionList::GetCount() == 0);
}


void dgWorld::InitBody (dgBody* const body, dgCollisionInstance* const collision, const dgMatrix& matrix)
{
	dgAssert (collision);

	m_bodiesUniqueID ++;
	body->m_world = this;

	body->m_spawnnedFromCallback = dgUnsigned32 (m_inUpdate ? true : false);
	body->m_uniqueID = dgInt32 (m_bodiesUniqueID);

	dgBodyMasterList::AddBody(body);

	body->SetCentreOfMass (dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (1.0f))); 
	body->SetLinearDamping (dgFloat32 (0.1045f)) ;
	body->SetAngularDamping (dgVector (dgFloat32 (0.1045f), dgFloat32 (0.1045f), dgFloat32 (0.1045f), dgFloat32 (0.0f)));

	body->AttachCollision(collision);
	body->m_bodyGroupId = dgInt32 (m_defualtBodyGroupID);

	dgMatrix inertia (dgGetIdentityMatrix());
	inertia[0][0] = DG_INFINITE_MASS;
	inertia[1][1] = DG_INFINITE_MASS;
	inertia[2][2] = DG_INFINITE_MASS;
	body->SetMassMatrix (DG_INFINITE_MASS * dgFloat32 (2.0f), inertia);
	body->SetMatrix (matrix);
	if (!body->GetCollision()->IsType (dgCollision::dgCollisionNull_RTTI)) {
		m_broadPhase->Add (body);
	}
}

void dgWorld::BodyEnableSimulation (dgBody* const body)
{
	if (!body->m_masterNode) {
		m_disableBodies.Remove(body);
		dgBodyMasterList::AddBody(body);
		body->SetMassMatrix(body->m_mass.m_w, body->CalculateLocalInertiaMatrix());
		m_broadPhase->Add (body);
		dgAssert (body->m_masterNode);
	}
}

void dgWorld::BodyDisableSimulation(dgBody* const body)
{
	if (body->m_masterNode) {
		m_broadPhase->Remove(body);
		dgBodyMasterList::RemoveBody(body);
		m_disableBodies.Insert(0, body);
		dgAssert (!body->m_masterNode);
	}
}

bool dgWorld::GetBodyEnableDisableSimulationState (dgBody* const body) const
{
	return body->m_masterNode ? true : false;
}

dgDynamicBody* dgWorld::CreateDynamicBody(dgCollisionInstance* const collision, const dgMatrix& matrix)
{
	dgDynamicBody* const body = new (m_allocator) dgDynamicBody();
	dgAssert (dgInt32 (sizeof (dgBody) & 0xf) == 0);
	dgAssert ((dgUnsigned64 (body) & 0xf) == 0);

	InitBody (body, collision, matrix);
	return body;
}

dgKinematicBody* dgWorld::CreateKinematicBody (dgCollisionInstance* const collision, const dgMatrix& matrix)
{
	dgKinematicBody* const body = new (m_allocator) dgKinematicBody();
	dgAssert (dgInt32 (sizeof (dgBody) & 0xf) == 0);
	dgAssert ((dgUnsigned64 (body) & 0xf) == 0);

	InitBody (body, collision, matrix);
	return body;
}


void dgWorld::DestroyBody(dgBody* const body)
{
	for (dgListenerList::dgListNode* node = m_postListener.GetLast(); node; node = node->GetPrev()) {
		dgListener& listener = node->GetInfo();
		if (listener.m_onBodyDestroy) {
			listener.m_onBodyDestroy (this, node, body);
		}
	}

	for (dgListenerList::dgListNode* node = m_preListener.GetLast(); node; node = node->GetPrev()) {
		dgListener& listener = node->GetInfo();
		if (listener.m_onBodyDestroy) {
			listener.m_onBodyDestroy (this, node, body);
		}
	}


	if (body->m_destructor) {
		body->m_destructor (*body);
	}
	
	if (m_disableBodies.Find(body)) {
		m_disableBodies.Remove(body);
	} else {
		m_broadPhase->Remove (body);
		dgBodyMasterList::RemoveBody (body);
	}

	dgAssert (body->m_collision);
	body->m_collision->Release();
	delete body;
}


void dgWorld::DestroyConstraint(dgConstraint* const constraint)
{
	RemoveConstraint (constraint);
	delete constraint;
}

void dgWorld::ExecuteUserJob (dgWorkerThreadTaskCallback userJobKernel, void* const userJobKernelContext)
{
	QueueJob (userJobKernel, this, userJobKernelContext);
}


void dgWorld::SetUserData (void* const userData)
{
	m_userData = userData;
}

void* dgWorld::GetUserData() const
{
	return m_userData;
}


void dgWorld::SetIslandUpdateCallback (OnClusterUpdate callback)
{
	m_clusterUpdate = callback;
}


void* dgWorld::AddPreListener (const char* const nameid, void* const userData, OnListenerUpdateCallback updateCallback, OnListenerDestroyCallback destroyCallback)
{
	dgListenerList::dgListNode* const node = m_preListener.Append();
	dgListener& listener = node->GetInfo();
	strncpy (listener.m_name, nameid, sizeof (listener.m_name));
	listener.m_world = this;
	listener.m_userData = userData;
	listener.m_onListenerUpdate = updateCallback;
	listener.m_onListenerDestroy = destroyCallback;
	return node;
}


void* dgWorld::AddPostListener (const char* const nameid, void* const userData, OnListenerUpdateCallback updateCallback, OnListenerDestroyCallback destroyCallback)
{
	dgListenerList::dgListNode* const node = m_postListener.Append();
	dgListener& listener = node->GetInfo();
	strncpy (listener.m_name, nameid, sizeof (listener.m_name));
	listener.m_world = this;
	listener.m_userData = userData;
	listener.m_onListenerUpdate = updateCallback;
	listener.m_onListenerDestroy = destroyCallback;
	return node;
}

void* dgWorld::GetListenerUserData (void* const listenerNode) const
{
	dgListener& listener = ((dgListenerList::dgListNode*) listenerNode)->GetInfo();
	return listener.m_userData;
}

void dgWorld::SetListenerBodyDestroyCallback (void* const listenerNode, OnListenerBodyDestroyCallback callback)
{
	dgListener& listener = ((dgListenerList::dgListNode*) listenerNode)->GetInfo();
	listener.m_onBodyDestroy = callback;
}

dgWorld::OnListenerBodyDestroyCallback dgWorld::GetListenerBodyDestroyCallback (void* const listenerNode) const
{
	dgListener& listener = ((dgListenerList::dgListNode*) listenerNode)->GetInfo();
	return listener.m_onBodyDestroy;
}


void* dgWorld::FindPreListener (const char* const nameid) const
{
	for (dgListenerList::dgListNode* node = m_preListener.GetFirst(); node; node = node->GetNext()) {
		dgListener& listener = node->GetInfo();
		if (!strcmp (nameid, listener.m_name)) {
			return node;
		}
	}
	return NULL;
}

void* dgWorld::FindPostListener (const char* const nameid) const 
{
	for (dgListenerList::dgListNode* node = m_postListener.GetFirst(); node; node = node->GetNext()) {
		dgListener& listener = node->GetInfo();
		if (!strcmp (nameid, listener.m_name)) {
			return node;
		}
	}
	return NULL;
}



dgBallConstraint* dgWorld::CreateBallConstraint (
	const dgVector& pivot, 
	dgBody* const body0, 
	dgBody* const body1)
{

	dgAssert (body0);
	dgAssert (body0 != body1);
	dgBallConstraint* const constraint = new (m_allocator) dgBallConstraint;

	AttachConstraint (constraint, body0, body1);
	constraint->SetPivotPoint (pivot);
	return constraint;
}


dgHingeConstraint* dgWorld::CreateHingeConstraint (
	const dgVector& pivot, 
	const dgVector& pinDir, 
	dgBody* const body0, 
	dgBody* const body1)
{
	dgAssert (body0);
	dgAssert (body0 != body1);
	dgHingeConstraint* const constraint = new (m_allocator) dgHingeConstraint;

	AttachConstraint (constraint, body0, body1);
	constraint->SetPivotAndPinDir (pivot, pinDir);
	return constraint;
}


dgUpVectorConstraint* dgWorld::CreateUpVectorConstraint (const dgVector& pin, dgBody *body)
{
	dgAssert (body);
	dgUpVectorConstraint* const constraint = new (m_allocator) dgUpVectorConstraint;

	AttachConstraint (constraint, body, NULL);
	constraint->InitPinDir (pin);
	return constraint;
}



dgSlidingConstraint* dgWorld::CreateSlidingConstraint (
	const dgVector& pivot, 
	const dgVector& pinDir, 
	dgBody* const body0, 
	dgBody* const body1)
{
	dgAssert (body0);
	dgAssert (body0 != body1);
	dgSlidingConstraint* const constraint = new (m_allocator) dgSlidingConstraint;

	AttachConstraint (constraint, body0, body1);
	constraint->SetPivotAndPinDir (pivot, pinDir);
	return constraint;
}


dgCorkscrewConstraint* dgWorld::CreateCorkscrewConstraint (
	const dgVector& pivot, 
	const dgVector& pinDir, 
	dgBody* const body0, 
	dgBody* const body1)
{
	dgAssert (body0);
	dgAssert (body0 != body1);
	dgCorkscrewConstraint* const constraint = new (m_allocator) dgCorkscrewConstraint;

	AttachConstraint (constraint, body0, body1);
	constraint->SetPivotAndPinDir (pivot, pinDir);
	return constraint;
}


dgUniversalConstraint* dgWorld::CreateUniversalConstraint (
	const dgVector& pivot, 
	const dgVector& pin0, 
	const dgVector& pin1, 
	dgBody* const body0, 
	dgBody* const body1)
{
	dgAssert (body0);
	dgAssert (body0 != body1);
	dgUniversalConstraint* const constraint = new (m_allocator) dgUniversalConstraint;

	AttachConstraint (constraint, body0, body1);
	constraint->SetPivotAndPinDir(pivot, pin0, pin1);
	return constraint;
}


/*
dgInt32 dgWorld::GetActiveBodiesCount() const
{
	return m_activeBodiesCount;
}
*/

dgInt32 dgWorld::GetBodiesCount() const
{
	const dgBodyMasterList& list = *this;
	return list.GetCount() - 1;
}

dgInt32 dgWorld::GetConstraintsCount() const
{
	const dgBodyMasterList& list = *this;
	return dgInt32 (list.m_constraintCount);
}


void dgWorld::BodySetMatrix (dgBody* const body, const dgMatrix& matrix)
{
	#define DG_RECURSIVE_SIZE	1024

	dgBody* queue[DG_RECURSIVE_SIZE];

	dgInt32 index = 1;
	queue[0] = body;
	m_genericLRUMark ++;
	body->m_genericLRUMark = m_genericLRUMark;
	dgMatrix relMatrix (body->GetMatrix().Inverse() * matrix);
	while (index) {
		index --;
		dgBody* body = queue[index];
		dgAssert (body != m_sentinelBody);

		// why should I do this? I do no remember the reason
		//m_broadPhase->Remove (body);
		//m_broadPhase->Add (body);

		dgMatrix matrix (body->GetMatrix() * relMatrix);
		body->SetVelocity (dgVector (dgFloat32 (0.0f)));    
		body->SetOmega (dgVector (dgFloat32 (0.0f)));    
		body->SetMatrix (matrix);

		for (dgBodyMasterListRow::dgListNode* jointNode = body->m_masterNode->GetInfo().GetFirst(); jointNode; jointNode = jointNode->GetNext()) {
			dgBodyMasterListCell& cell = jointNode->GetInfo();
			body = cell.m_bodyNode;
			if (body != m_sentinelBody) {
				if (body->m_genericLRUMark != m_genericLRUMark) {
					dgConstraint* constraint;
					constraint = cell.m_joint;
					if (constraint->GetId() != dgConstraint::m_contactConstraint) {
						body->m_genericLRUMark = m_genericLRUMark;
						queue[index] = body;
						index ++;
						dgAssert (index < DG_RECURSIVE_SIZE);
					}
				}
			}
		}
	}
}


bool dgWorld::AreBodyConnectedByJoints (dgBody* const originSrc, dgBody* const targetSrc)
{
	#define DG_QEUEU_SIZE	1024
	dgBody* queue[DG_QEUEU_SIZE];

	m_genericLRUMark ++;

	dgBody* origin1 = originSrc;
	dgBody* target1 = targetSrc;
	if (origin1->GetInvMass().m_w == dgFloat32 (0.0f)) {
		dgSwap (origin1, target1);
	}

	dgAssert (origin1->GetInvMass().m_w != dgFloat32 (0.0f));
	dgBody* const origin = origin1;
	dgBody* const target = target1;

	dgInt32 end = 1;
	dgInt32 start = 0;
	queue[0] = origin;
	origin->m_genericLRUMark = m_genericLRUMark;

	while (start != end) {
		dgBody* const origin = queue[start];
		start ++;
		start &= (DG_QEUEU_SIZE - 1);
	
		for (dgBodyMasterListRow::dgListNode* jointNode = origin->m_masterNode->GetInfo().GetFirst(); jointNode; jointNode = jointNode->GetNext()) {
			dgBodyMasterListCell& cell = jointNode->GetInfo();

			dgBody* const body = cell.m_bodyNode;
			if (body->m_genericLRUMark != m_genericLRUMark) {
				dgConstraint* const constraint = cell.m_joint;
				if (constraint->GetId() != dgConstraint::m_contactConstraint) {
					if (body == target) {
						return true;
					}
					body->m_genericLRUMark = m_genericLRUMark;
					queue[end] = body;
					end ++;
					end &= (DG_QEUEU_SIZE - 1);
				}
			}
		}
	}
	return false;
}

void dgWorld::FlushCache()
{
	// delete all contacts
	dgActiveContacts& contactList = *this;
	for (dgActiveContacts::dgListNode* contactNode = contactList.GetFirst(); contactNode; ) {
		dgContact* contact;
		contact = contactNode->GetInfo();
		contactNode = contactNode->GetNext();
		DestroyConstraint (contact);
	}

	// clean up memory in bradPhase
	m_broadPhase->InvalidateCache ();

	// sort body list
	SortMasterList();
}


void dgWorld::StepDynamics (dgFloat32 timestep)
{

//static int xxx ;
//xxx ++;
//dgTrace (("%d\n", xxx));
//if (xxx >= 2000)
//xxx *=1;

	//xxxxx();
	//SerializeToFile ("xxx.bin");

	dTimeTrackerEvent(__FUNCTION__);
	dgAssert (m_inUpdate == 0);
	dgAssert (GetThreadCount() >= 1);

	m_inUpdate ++;

	m_broadPhase->UpdateContacts (timestep);
	UpdateDynamics (timestep);

	if (m_postListener.GetCount()) {
		dTimeTrackerEvent("postListeners");
		for (dgListenerList::dgListNode* node = m_postListener.GetFirst(); node; node = node->GetNext()) {
			dgListener& listener = node->GetInfo();
			listener.m_onListenerUpdate (this, listener.m_userData, timestep);
		}
	}

	m_inUpdate --;
}


void dgWorldThreadPool::OnBeginWorkerThread (dgInt32 threadId)
{

}

void dgWorldThreadPool::OnEndWorkerThread (dgInt32 threadId)
{

}

void dgWorld::Execute (dgInt32 threadID)
{
	if (threadID == DG_MUTEX_THREAD_ID) {
		dgMutexThread::Execute (threadID);
	} else {
		dgAsyncThread::Execute (threadID);
	}
}

void dgWorld::Sync ()
{
	while (dgMutexThread::IsBusy()) {
		dgThreadYield();
	}
}


void dgWorld::RunStep ()
{
	dgUnsigned64 timeAcc = m_getDebugTime ? m_getDebugTime() : 0;
	dgFloat32 step = m_savetimestep / m_numberOfSubsteps;
	for (dgUnsigned32 i = 0; i < m_numberOfSubsteps; i ++) {
		dgInterlockedExchange(&m_delayDelateLock, 1);
		StepDynamics (step);
		dgInterlockedExchange(&m_delayDelateLock, 0);

		dgDeadBodies& bodyList = *this;
		dgDeadJoints& jointList = *this;

		jointList.DestroyJoints (*this);
		bodyList.DestroyBodies (*this);
	}
	m_lastExecutionTime = m_getDebugTime ? dgFloat32 (m_getDebugTime() - timeAcc) * dgFloat32 (1.0e-6f): 0;
}

void dgWorld::TickCallback (dgInt32 threadID)
{
	if (threadID == DG_MUTEX_THREAD_ID) {
		RunStep ();
	} else {
		Update (m_savetimestep);
	}
}


void dgWorld::Update (dgFloat32 timestep)
{
	m_savetimestep = timestep;
	#ifdef DG_USE_THREAD_EMULATION
		dgFloatExceptions exception;
		dgSetPrecisionDouble precision;
		RunStep ();
	#else 
		// runs the update in a separate thread and wait until the update is completed before it returns.
		// this will run well on single core systems, since the two thread are mutually exclusive 
		dgMutexThread::Tick();
	#endif
}


void dgWorld::UpdateAsync (dgFloat32 timestep)
{
	Sync ();
	m_savetimestep = timestep;
	#ifdef DG_USE_THREAD_EMULATION
		dgFloatExceptions exception;
		dgSetPrecisionDouble precision;
		RunStep ();
	#else 
		// execute one update, but do not wait for the update to finish, instead return immediately to the caller
		dgAsyncThread::Tick();
	#endif
}


void dgWorld::SerializeToFile (const char* const fileName, OnBodySerialize bodyCallback, void* const userData) const
{
	FILE* const file = fopen (fileName, "wb");
	if (file) {
		dgBody** const array = new dgBody*[GetBodiesCount()];

		dgInt32 count = 0;
		const dgBodyMasterList& me = *this;
		for (dgBodyMasterList::dgListNode* node = me.GetFirst()->GetNext(); node; node = node->GetNext()) {
			const dgBodyMasterListRow& graphNode = node->GetInfo();
			array[count] = graphNode.GetBody();	
			count ++;
			dgAssert (count <= GetBodiesCount());
		}
		SerializeBodyArray (array, count, bodyCallback ? bodyCallback : OnBodySerializeToFile, userData, OnSerializeToFile, file);
		SerializeJointArray (array, count, OnSerializeToFile, file);

		delete[] array;
		fclose (file);
	}
}

void dgWorld::DeserializeFromFile (const char* const fileName, OnBodyDeserialize bodyCallback, void* const userData)
{
	FILE* const file = fopen (fileName, "rb");
	if (file) {
		dgTree<dgBody*, dgInt32> bodyMap (GetAllocator());
		DeserializeBodyArray (bodyMap, bodyCallback ? bodyCallback : OnBodyDeserializeFromFile, userData, OnDeserializeFromFile, file);
		DeserializeJointArray (bodyMap, OnDeserializeFromFile, file);
		fclose (file);
	}
}



void dgWorld::OnSerializeToFile (void* const fileHandle, const void* const buffer, size_t size)
{
	dgAssert ((size & 0x03) == 0);
	fwrite (buffer, size, 1, (FILE*) fileHandle);
}

void dgWorld::OnDeserializeFromFile (void* const fileHandle, void* const buffer, size_t size)
{
	dgAssert ((size & 0x03) == 0);
	fread (buffer, size, 1, (FILE*) fileHandle);
}

void dgWorld::OnBodySerializeToFile (dgBody& body, void* const userData, dgSerialize serializeCallback, void* const fileHandle)
{
	const char* const bodyIndentification = "NewtonGravityBody\0\0\0\0";
	int size = (dgInt32 (strlen (bodyIndentification)) + 3) & -4;
	serializeCallback (fileHandle, &size, sizeof (size));
	serializeCallback (fileHandle, bodyIndentification, size);
}

void dgWorld::OnBodyDeserializeFromFile (dgBody& body, void* const userData, dgDeserialize serializeCallback, void* const fileHandle)
{

}


void dgWorld::SetGetTimeInMicrosenconds (OnGetTimeInMicrosenconds callback)
{
	m_getDebugTime = callback;
}

void dgWorld::SetCollisionInstanceConstructorDestructor (OnCollisionInstanceDuplicate constructor, OnCollisionInstanceDestroy destructor)
{
	m_onCollisionInstanceDestruction = destructor;
	m_onCollisionInstanceCopyConstrutor = constructor;
}


void dgWorld::SerializeBodyArray (dgBody** const array, dgInt32 count, OnBodySerialize bodyCallback, void* const userData, dgSerialize serializeCallback, void* const fileHandle) const
{
	dgSerializeMarker (serializeCallback, fileHandle);

	// serialize all collisions
	dgInt32 uniqueShapes = 0;
	dgTree<dgInt32, const dgCollision*> shapeMap(GetAllocator());
	for (dgInt32 i = 0; i < count; i ++) {
		dgBody* const body = array[i];
		dgAssert (body->m_world == this);
		dgCollisionInstance* const instance = body->GetCollision();
		const dgCollision* const collision = instance->GetChildShape();
		dgTree<dgInt32, const dgCollision*>::dgTreeNode* const shapeNode = shapeMap.Insert(uniqueShapes, collision);
		if (shapeNode) {
			uniqueShapes ++;
		}
	}

	serializeCallback(fileHandle, &uniqueShapes, sizeof (uniqueShapes));	
	dgTree<dgInt32, const dgCollision*>::Iterator iter (shapeMap);
	for (iter.Begin(); iter; iter ++) {
		dgInt32 id = iter.GetNode()->GetInfo();
		const dgCollision* const collision = iter.GetKey();
		dgCollisionInstance instance (this, collision, 0, dgMatrix (dgGetIdentityMatrix()));
		serializeCallback(fileHandle, &id, sizeof (id));	
		instance.Serialize(serializeCallback, fileHandle);
		dgSerializeMarker(serializeCallback, fileHandle);
	}

	serializeCallback(fileHandle, &count, sizeof (count));	
	for (dgInt32 i = 0; i < count; i ++) {
		dgBody* const body = array[i];

		dgInt32 bodyType = body->GetType();
		serializeCallback(fileHandle, &bodyType, sizeof (bodyType));	

		// serialize the body
		body->Serialize(shapeMap, serializeCallback, fileHandle);

		// serialize body custom data
		bodyCallback (*body, userData, serializeCallback, fileHandle);

		dgSerializeMarker(serializeCallback, fileHandle);
	}
}


void dgWorld::DeserializeBodyArray (dgTree<dgBody*, dgInt32>&bodyMap, OnBodyDeserialize bodyCallback, void* const userData, dgDeserialize deserialization, void* const fileHandle)
{
	dgInt32 revision = dgDeserializeMarker (deserialization, fileHandle);

	dgTree<const dgCollision*, dgInt32> shapeMap(GetAllocator());

	dgInt32 uniqueShapes;
	deserialization(fileHandle, &uniqueShapes, sizeof (uniqueShapes));	
	for (dgInt32 i = 0; i < uniqueShapes; i ++) {
		dgInt32 id;

		deserialization(fileHandle, &id, sizeof (id));	
		dgCollisionInstance instance (this, deserialization, fileHandle, revision);
		dgDeserializeMarker (deserialization, fileHandle);

		const dgCollision* const shape = instance.GetChildShape();
		shapeMap.Insert(shape, id);
		shape->AddRef();
	}

	dgInt32 bodyCount;
	deserialization (fileHandle, &bodyCount, sizeof (bodyCount));	
	for (dgInt32 i = 0; i < bodyCount; i ++) {
		dgInt32 bodyType;
		deserialization(fileHandle, &bodyType, sizeof (bodyType));	
		dgBody* body = NULL; 

		switch (bodyType)
		{
			case dgBody::m_dynamicBody:
			{
				body = new (m_allocator) dgDynamicBody(this, &shapeMap, deserialization, fileHandle, revision);
				break;
			}
			case dgBody::m_kinematicBody:
			{
				body = new (m_allocator) dgKinematicBody(this, &shapeMap, deserialization, fileHandle, revision);
				break;
			}
		}

		dgAssert (body);
		m_bodiesUniqueID ++;
		body->m_freeze = false;
		body->m_sleeping = false;
		body->m_equilibrium = false;
		body->m_spawnnedFromCallback = false;
		body->m_uniqueID = dgInt32 (m_bodiesUniqueID);

		dgBodyMasterList::AddBody(body);
		body->SetMatrix (body->GetMatrix());
		m_broadPhase->Add (body);
		if (body->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
			dgDynamicBody* const dynBody = (dgDynamicBody*)body;
			dynBody->SetMassMatrix (dynBody->m_mass.m_w, dynBody->CalculateLocalInertiaMatrix());
		}

		// load user related data 
		bodyCallback (*body, userData, deserialization, fileHandle);

		bodyMap.Insert(body, i);

		// sync to next body
		dgDeserializeMarker (deserialization, fileHandle);
	}

	dgTree<const dgCollision*, dgInt32>::Iterator iter (shapeMap);
	for (iter.Begin(); iter; iter ++) {
		const dgCollision* const collision = iter.GetNode()->GetInfo();
		collision->Release();
	}
}

void dgWorld::SetJointSerializationCallbacks (OnJointSerializationCallback serializeJoint, OnJointDeserializationCallback deserializeJoint)
{
	m_serializedJointCallback = serializeJoint;
	m_deserializedJointCallback = deserializeJoint;
}

void dgWorld::GetJointSerializationCallbacks (OnJointSerializationCallback* const serializeJoint, OnJointDeserializationCallback* const deserializeJoint) const
{
	*serializeJoint = m_serializedJointCallback;
	*deserializeJoint = m_deserializedJointCallback;
}


void dgWorld::SerializeJointArray (dgBody** const bodyArray, dgInt32 bodyCount, dgSerialize serializeCallback, void* const userData) const
{
	dgInt32 count = 0;
	const dgBodyMasterList* me = this;
	for (dgBodyMasterList::dgListNode* node = me->GetFirst(); node; node = node->GetNext()) {
		const dgBodyMasterListRow& info = node->GetInfo();
		for (dgBodyMasterListRow::dgListNode *jointNode = info.GetFirst(); jointNode; jointNode = jointNode->GetNext()) {
			const dgBodyMasterListCell& cell = jointNode->GetInfo();

			dgConstraint* const joint = cell.m_joint;
			count += joint->IsBilateral() ? 1 : 0;
		}
	}

	dgTree<int, dgBody*> bodyMap (GetAllocator());
	for (dgInt32 i = 0; i < bodyCount; i ++) {
		bodyMap.Insert (i, bodyArray[i]);
	}

	count /= 2;
	dgSerializeMarker (serializeCallback, userData);
	serializeCallback(userData, &count, sizeof (count));	

	dgTree<int, dgConstraint*> map (GetAllocator());
	for (dgBodyMasterList::dgListNode* node = me->GetFirst(); node; node = node->GetNext()) {
		dgBodyMasterListRow& info = node->GetInfo();
		for (dgBodyMasterListRow::dgListNode *jointNode = info.GetFirst(); jointNode; jointNode = jointNode->GetNext()) {
			const dgBodyMasterListCell& cell = jointNode->GetInfo();
			dgConstraint* const joint = cell.m_joint;
			if (joint->IsBilateral()) {
				if (!map.Find(joint)) {
					map.Insert (0, joint);
					dgInt32 body0; 
					dgInt32 body1; 
					dgAssert (joint->GetBody0());
					dgAssert (joint->GetBody1());
					body0 = (joint->GetBody0() != m_sentinelBody) ? bodyMap.Find (joint->GetBody0())->GetInfo() : -1;
					body1 = (joint->GetBody1() != m_sentinelBody) ? bodyMap.Find (joint->GetBody1())->GetInfo() : -1;

					serializeCallback(userData, &body0, sizeof (body0));
					serializeCallback(userData, &body1, sizeof (body1));

					dgBilateralConstraint* const bilateralJoint = (dgBilateralConstraint*) joint;
					bilateralJoint->Serialize (serializeCallback, userData);

					dgSerializeMarker(serializeCallback, userData);
				}
			}
		}
	}

	dgSerializeMarker(serializeCallback, userData);
}

void dgWorld::DeserializeJointArray (const dgTree<dgBody*, dgInt32>&bodyMap, dgDeserialize serializeCallback, void* const userData)
{
	dgInt32 count = 0;

	dgDeserializeMarker (serializeCallback, userData);
	serializeCallback(userData, &count, sizeof (count));	

	for (dgInt32 i = 0; i < count; i ++) {
		if (m_deserializedJointCallback) {
			dgInt32 bodyIndex0; 
			dgInt32 bodyIndex1; 

			serializeCallback(userData, &bodyIndex0, sizeof (bodyIndex0));
			serializeCallback(userData, &bodyIndex1, sizeof (bodyIndex1));

			dgBody* const body0 = (bodyIndex0 != -1) ? bodyMap.Find (bodyIndex0)->GetInfo() : NULL;
			dgBody* const body1 = (bodyIndex1 != -1) ? bodyMap.Find (bodyIndex1)->GetInfo() : NULL;
			m_deserializedJointCallback (body0, body1, serializeCallback, userData);
		}
		dgDeserializeMarker(serializeCallback, userData);
	}

	dgDeserializeMarker(serializeCallback, userData);
}


dgInt32 dgWorld::GetBroadPhaseType() const
{
	return dgInt32 (m_broadPhase->GetType());
}

void dgWorld::SetBroadPhaseType(dgInt32 type)
{
	if (type != GetBroadPhaseType()) {
		dgBroadPhase* newBroadPhase = NULL;
		
		switch (type)
		{
			case m_persistentBroadphase:
				newBroadPhase = new (m_allocator) dgBroadPhasePersistent(this);
				break;

			case m_defaultBroadphase:
			default:
				newBroadPhase = new (m_allocator) dgBroadPhaseDefault(this);
				break;
		}

		m_broadPhase->MoveNodes(newBroadPhase);
		delete m_broadPhase;
		m_broadPhase = newBroadPhase;
	}
}


dgSkeletonContainer* dgWorld::CreateNewtonSkeletonContainer (dgBody* const rootBone)
{
	dgAssert (rootBone);
	dgSkeletonList* const list = this;
	if (dgSkeletonContainer::m_uniqueID > 1014 * 16) {
		dgList<dgSkeletonContainer*> saveList (GetAllocator());
		dgSkeletonList::Iterator iter (*list);
		for (iter.Begin(); iter; iter ++) {
			saveList.Append(iter.GetNode()->GetInfo());
		}
		list->RemoveAll();

		dgInt32 index = DG_SKELETON_BASEW_UNIQUE_ID;
		for (dgList<dgSkeletonContainer*>::dgListNode* ptr = saveList.GetFirst(); ptr; ptr ++) {
			dgSkeletonContainer* const skeleton = ptr->GetInfo();
			skeleton->m_id = index;
			list->Insert (skeleton, skeleton->GetId());
			index ++;
		}
		dgSkeletonContainer::ResetUniqueId(index);
	}

	dgAssert (rootBone->GetType() == dgBody::m_dynamicBody);
	dgSkeletonContainer* const container = new (m_allocator) dgSkeletonContainer(this, (dgDynamicBody*)rootBone);
	
	list->Insert (container, container->GetId());
	return container;
}

void dgWorld::DestroySkeletonContainer (dgSkeletonContainer* const container)
{
	dgSkeletonList* const list = this;
	list->Remove (container->GetId());
	delete container;
}


dgBroadPhaseAggregate* dgWorld::CreateAggreGate() const
{
	return m_broadPhase->CreateAggregate();
}

void dgWorld::DestroyAggregate(dgBroadPhaseAggregate* const aggregate) const
{
	m_broadPhase->DestroyAggregate((dgBroadPhaseAggregate*) aggregate);
}


void dgDeadJoints::DestroyJoint(dgConstraint* const joint)
{
	dgSpinLock (&m_lock, true);

	dgWorld& me = *((dgWorld*)this);
	if (me.m_delayDelateLock) {
		// the engine is busy in the previous update, deferred the deletion
		Insert (joint, joint);
	} else {
		me.DestroyConstraint (joint);
	}

	dgSpinUnlock(&m_lock);
}


void dgDeadJoints::DestroyJoints(dgWorld& world)
{
	dgSpinLock (&m_lock, true);
	Iterator iter (*this);
	for (iter.Begin(); iter; iter++) {
		dgTreeNode* const node = iter.GetNode();
		dgConstraint* const joint = node->GetInfo();
		world.DestroyConstraint (joint);
	}
	RemoveAll ();
	dgSpinUnlock(&m_lock);
}


void dgDeadBodies::DestroyBody(dgBody* const body)
{
	dgAssert (0);
	dgSpinLock (&m_lock, true);

	dgWorld& me = *((dgWorld*)this);
	if (me.m_delayDelateLock) {
		// the engine is busy in the previous update, deferred the deletion
		Insert (body, body);
	} else {
		me.DestroyBody(body);
	}
	dgSpinUnlock(&m_lock);
}


void dgDeadBodies::DestroyBodies(dgWorld& world)
{
	dgSpinLock (&m_lock, true);

	Iterator iter (*this);
	for (iter.Begin(); iter; iter++) {
		dgTreeNode* const node = iter.GetNode();
		dgBody* const body = node->GetInfo();
		world.DestroyBody(body);
	}
	RemoveAll ();
	dgSpinUnlock(&m_lock);
}
