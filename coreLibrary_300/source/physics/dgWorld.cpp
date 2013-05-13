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

#include "dgPhysicsStdafx.h"
#include "dgWorld.h"

#include "dgDynamicBody.h"
#include "dgKinematicBody.h"
#include "dgCollisionBox.h"
#include "dgKinematicBody.h"
#include "dgCollisionNull.h"
#include "dgCollisionCone.h"
#include "dgMinkowskiConv.h"
#include "dgDeformableBody.h"
#include "dgCollisionScene.h"
#include "dgCollisionSphere.h"
#include "dgCollisionCapsule.h"
#include "dgCollisionCylinder.h"
#include "dgCollisionInstance.h"
#include "dgCollisionCompound.h"
#include "dgWorldDynamicUpdate.h"
#include "dgCollisionConvexHull.h"
#include "dgCollisionChamferCylinder.h"

#include "dgUserConstraint.h"
#include "dgBallConstraint.h"
#include "dgHingeConstraint.h"
#include "dgSlidingConstraint.h"
#include "dgUpVectorConstraint.h"
#include "dgUniversalConstraint.h"
#include "dgCorkscrewConstraint.h"


#ifdef _NEWTON_OPENCL
#include "dgOpenclInstance.h"
#endif


#define DG_INITIAL_ISLAND_SIZE		(1024 * 4)
#define DG_INITIAL_BODIES_SIZE		(1024 * 4)
#define DG_INITIAL_JOINTS_SIZE		(1024 * 4)
#define DG_INITIAL_JACOBIAN_SIZE	(1024 * 16)
#define DG_INITIAL_CONTACT_SIZE		(1024 * 32)


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

dgWorld::dgWorld(dgMemoryAllocator* const allocator)
	:dgBodyMasterList(allocator)
	,dgBodyMaterialList(allocator)
	,dgBodyCollisionList(allocator)
	,dgCollisionDeformableMeshList(allocator)
	,dgActiveContacts(allocator) 
	,dgCollidingPairCollector()
	,dgWorldDynamicUpdate()
	,dgMutexThread("dgMutexThread", DG_MUTEX_THREAD_ID)
	,dgAsyncThread("dgAsyncThread", DG_ASYNC_THREAD_ID)
	,dgThreadHive(allocator)
	,m_openCL(NULL)
	,m_broadPhase(NULL)
	,m_preListener(allocator)
	,m_postListener(allocator)
	,m_perInstanceData(allocator)
	,m_islandMemory (DG_INITIAL_ISLAND_SIZE, allocator, 64)
	,m_bodiesMemory (DG_INITIAL_BODIES_SIZE, allocator, 64)
	,m_jointsMemory (DG_INITIAL_JOINTS_SIZE, allocator, 64)
	,m_pairMemoryBuffer (DG_INITIAL_CONTACT_SIZE, allocator, 64)
	,m_internalForcesMemory (DG_INITIAL_BODIES_SIZE, allocator, 64)
	,m_solverMatrixMemory (DG_INITIAL_JACOBIAN_SIZE, allocator, 64)
{
	dgMutexThread* const mutexThread = this;
	SetMatertThread (mutexThread);

	m_allocator = allocator;
	m_islandUpdate = NULL;
	m_leavingWorldNotify = NULL;
	m_getPerformanceCount = NULL;
	m_destroyBodyByExeciveForce = NULL;

	m_inUpdate = 0;
	m_bodyGroupID = 0;
	
	m_defualtBodyGroupID = CreateBodyGroupID();
	m_genericLRUMark = 0;

	m_useParallelSolver = 0;

	//m_solverMode = 0;
	m_solverMode = 1;
	m_frictionMode = 0;
	m_dynamicsLru = 0;
		
	m_bodiesUniqueID = 0;
	m_frictiomTheshold = dgFloat32 (0.25f);

	m_userData = NULL;
	m_islandUpdate = NULL;
	m_leavingWorldNotify = NULL;
	m_destroyBodyByExeciveForce = NULL;

	m_freezeAccel2 = DG_FREEZE_MAG2;
	m_freezeAlpha2 = DG_FREEZE_MAG2;
	m_freezeSpeed2 = DG_FREEZE_MAG2 * dgFloat32 (0.1f);
	m_freezeOmega2 = DG_FREEZE_MAG2 * dgFloat32 (0.1f);

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
	m_cpu = dgNoSimdPresent;
//	SetHardwareMode (0);

	SetThreadsCount (0);

	//dgBroadPhase::Init ();
	m_broadPhase = new (allocator) dgBroadPhase(this);
	dgCollidingPairCollector::Init ();
	
	//m_pointCollision = new (m_allocator) dgCollisionPoint(m_allocator);
	dgCollision* const pointCollison = new (m_allocator) dgCollisionPoint(m_allocator);
	m_pointCollision = CreateInstance(pointCollison, 0, dgGetIdentityMatrix());
	pointCollison->Release();

	AddSentinelBody();
	SetPerfomanceCounter(NULL);


	// set all entry to be no swap order
	memset (m_collisionSwapPriority, false, sizeof (m_collisionSwapPriority));
	for (dgInt32 i = m_sphereCollision; i < m_boxCollision; i ++) {
		for (dgInt32 j = m_sphereCollision; j < i; j ++) {
			m_collisionSwapPriority[i][j] = true;
		}
	}
	for (dgInt32 i = m_boxCollision; i < m_nullCollision; i ++) {
		for (dgInt32 j = m_sphereCollision; j < m_boxCollision; j ++) {
			m_collisionSwapPriority[i][j] = true;
		}
	}

	#ifdef _NEWTON_OPENCL
	//m_openCL = dgOpencl::GetOpenCL(GetAllocator());
	m_openCL = new (GetAllocator()) dgOpenclInstance(this);
	#endif
}

dgWorld::~dgWorld()
{	
	Sync();
	dgAsyncThread::Terminate();
	dgMutexThread::Terminate();

	#ifdef _NEWTON_OPENCL
	if (m_openCL) {
		delete m_openCL;
	}
	#endif


	DestroyAllBodies();
	RemoveAllGroupID();
	m_pointCollision->Release();
	DestroyBody (m_sentionelBody);


	delete m_broadPhase;
}

void dgWorld::SetThreadsCount (dgInt32 count)
{
	dgThreadHive::SetThreadsCount(count);
	dgThreadHive::SetPerfomanceCounter(m_getPerformanceCount);
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
	m_sentionelBody = CreateDynamicBody(instance, dgGetIdentityMatrix());
	instance->Release();
	dgCollidingPairCollector::m_sentinel = m_sentionelBody;
}

dgBody* dgWorld::GetSentinelBody() const
{
	return m_sentionelBody;
}


void dgWorld::SetSolverMode (dgInt32 mode)
{
	m_solverMode = dgUnsigned32 (dgMax (0, mode));
}

void dgWorld::SetFrictionMode (dgInt32 mode)
{
	m_frictionMode = dgUnsigned32 (mode);
}
/*
void dgWorld::SetHardwareMode (dgInt32 mode)
{
	m_cpu = dgNoSimdPresent;
	if (mode) {
		m_cpu = dgGetCpuType ();
	}
}

dgInt32 dgWorld::GetHardwareMode(char* const description)
{
	dgInt32 mode = 0;

	if (m_cpu == dgNoSimdPresent){
		mode = 0;
		if (description) {
			sprintf (description, "x87");
		}
	} else if (m_cpu == dgSimdPresent) {
		mode = 1;
		if (description) {
			sprintf (description, "simd");
		}
	} else {
		mode = 2;
		if (description) {
			sprintf (description, "avx");
		}
	}
	return mode;
}
*/

dgInt32 dgWorld::EnumerateCPUHardwareModes() const
{
	dgInt32 count = 1;
	switch (dgGetCpuType ())
	{
		case dgNoSimdPresent:
			count = 1;
			break;

		case dgSimdPresent:
			count = 2;
			break;
	}

	return count;
}

dgInt32 dgWorld::EnumerateHardwareModes() const
{
	dgInt32 count = EnumerateCPUHardwareModes();
	#ifdef _NEWTON_OPENCL
		if (m_openCL) {
			count += m_openCL->GetPlatformsCount();
		}
	#endif
	return count;
}

void dgWorld::GetHardwareVendorString (dgInt32 deviceIndex, char* const description, dgInt32 maxlength) const
{
	deviceIndex = dgClamp(deviceIndex, 0, EnumerateHardwareModes() - 1);
	dgInt32 cpuModes = EnumerateCPUHardwareModes();
	if (deviceIndex < cpuModes) {
		switch (deviceIndex) 
		{
			case 0:
				sprintf (description, "x87");
				break;

			case 1:
				sprintf (description, "simd");
				break;

			case 2:
				sprintf (description, "avx");
				break;

		}

	} else if (m_openCL) {
		#ifdef _NEWTON_OPENCL
			m_openCL->GetVendorString (deviceIndex - cpuModes, description, maxlength);
		#endif
	}
}

void dgWorld::SetCurrentHardwareMode(dgInt32 deviceIndex)
{
	#ifdef _NEWTON_OPENCL
		if (m_openCL) {
			m_openCL->CleanUp();
		}
	#endif
	m_hardwaredIndex = dgClamp(deviceIndex, 0, EnumerateHardwareModes() - 1);

	dgInt32 cpuModes = EnumerateCPUHardwareModes();
	if (m_hardwaredIndex < cpuModes) {
		switch (m_hardwaredIndex) 
		{
			case 0:
				m_cpu = dgNoSimdPresent;
				break;

			case 1:
				m_cpu = dgSimdPresent;
				break;
		}

	} else if (m_openCL) {
		#ifdef _NEWTON_OPENCL
			m_cpu = dgSimdPresent;
			m_openCL->SelectPlaform (m_hardwaredIndex - cpuModes);
		#endif
	}
}


dgInt32 dgWorld::GetCurrentHardwareMode() const
{
//	return m_cpu;
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
	dgAssert (dgBodyMasterList::GetFirst()->GetInfo().GetBody() == m_sentionelBody);
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

	body->SetMassMatrix (DG_INFINITE_MASS * dgFloat32 (2.0f), DG_INFINITE_MASS, DG_INFINITE_MASS, DG_INFINITE_MASS);
	body->SetMatrix (matrix);
	m_broadPhase->Add (body);
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


dgBody* dgWorld::CreateDeformableBody(dgCollisionInstance* const collision, const dgMatrix& matrix)
{
	dgBody* const body = new (m_allocator) dgDeformableBody();
	dgAssert (dgInt32 (sizeof (dgBody) & 0xf) == 0);
	dgAssert ((dgUnsigned64 (body) & 0xf) == 0);

	InitBody (body, collision, matrix);
	return body;
}


void dgWorld::DestroyBody(dgBody* const body)
{
	if (body->m_destructor) {
		body->m_destructor (*body);
	}

	m_broadPhase->Remove (body);
	dgBodyMasterList::RemoveBody (body);

	dgAssert (body->m_collision);
	body->m_collision->Release();
	delete body;
}


void dgWorld::DestroyConstraint(dgConstraint* const constraint)
{
	RemoveConstraint (constraint);
	delete constraint;
}

OnGetPerformanceCountCallback dgWorld::GetPerformaceFuntion() const
{
	return m_getPerformanceCount;
};

void dgWorld::SetPerfomanceCounter(OnGetPerformanceCountCallback callback)
{
	dgThreadHive::SetPerfomanceCounter (callback);

	if (!callback) {
		callback = GetPerformanceCount;
	}
	m_getPerformanceCount = callback;
	memset (m_perfomanceCounters, 0, sizeof (m_perfomanceCounters));
	memset (m_perfomanceCountersBack, 0, sizeof (m_perfomanceCountersBack));
}


dgUnsigned32 dgWorld::GetPerfomanceTicks (dgUnsigned32 entry) const
{
	entry = dgClamp(dgUnsigned32 (entry), dgUnsigned32 (0), dgUnsigned32 (m_counterSize - 1));
	return m_perfomanceCountersBack[entry];
}

dgUnsigned32 dgWorld::GetThreadPerfomanceTicks (dgUnsigned32 threadIndex) const
{
	return dgThreadHive::GetPerfomanceTicks (threadIndex);
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


void dgWorld::SetIslandUpdateCallback (OnIslandUpdate callback)
{
	m_islandUpdate = callback;
}


void dgWorld::SetLeavingWorldCallback (OnLeavingWorldAction callback) 
{
	m_leavingWorldNotify = callback;
}


void dgWorld::SetBodyDestructionByExeciveForce (OnBodyDestructionByExeciveForce callback) 
{
	m_destroyBodyByExeciveForce = callback;
}


void* dgWorld::AddPreListener (const char* const nameid, void* const userData, OnListenerUpdateCallback updateCallback, OnListenerDestroyCallback destroyCallback)
{
	dgListenerList::dgListNode* const node = m_preListener.Append();
	dgListener& listener = node->GetInfo();
	strncpy (listener.m_name, nameid, sizeof (listener.m_name));
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
	dgBallConstraint *constraint;
	
	dgAssert (body0);
	dgAssert (body0 != body1);
//	constraint = dgBallConstraint::Create(this);
	constraint = new (m_allocator) dgBallConstraint;

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
	dgHingeConstraint *constraint;

	dgAssert (body0);
	dgAssert (body0 != body1);
//	constraint = dgHingeConstraint::Create(this);
	constraint = new (m_allocator) dgHingeConstraint;

	AttachConstraint (constraint, body0, body1);
	constraint->SetPivotAndPinDir (pivot, pinDir);
	return constraint;
}


dgUpVectorConstraint* dgWorld::CreateUpVectorConstraint (const dgVector& pin, dgBody *body)
{
	dgUpVectorConstraint *constraint;
	
	dgAssert (body);
//	constraint = dgUpVectorConstraint::Create(this);
	constraint = new (m_allocator) dgUpVectorConstraint;

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
	dgSlidingConstraint *constraint;

	dgAssert (body0);
	dgAssert (body0 != body1);
//	constraint = dgSlidingConstraint::Create(this);
	constraint = new (m_allocator) dgSlidingConstraint;

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
	dgCorkscrewConstraint *constraint;

	dgAssert (body0);
	dgAssert (body0 != body1);
//	constraint = dgCorkscrewConstraint::Create(this);
	constraint = new (m_allocator) dgCorkscrewConstraint;

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
	dgUniversalConstraint *constraint;

	dgAssert (body0);
	dgAssert (body0 != body1);
//	constraint = dgUniversalConstraint::Create(this);
	constraint = new (m_allocator) dgUniversalConstraint;

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


/*
dgLink* dgWorld::FindConstraintLink (const dgBody* const body0, const dgBody* const body1) const
{
	dgAssert (0);

	dgLink *ptr;
	dgLink *link;

	dgAssert (body0);
	if (!body0) {
		dgAssert (0);
		Swap (body0, body1);
	}

	if (body0) {
		link = body0->m_firstConstraintLink;
		if (link) {
			ptr = link;
			do {
				dgAssert (ptr->m_body == body0);
				if (ptr->m_twin->m_body == body1) {
					return ptr;
				}
			
				ptr = ptr->m_twin->m_next;
			} while (ptr != link);
		}
	}

	return NULL;
}
*/
/*
dgConstraint* dgWorld::GetConstraint (const dgLink* constraintLink) const
{
	return constraintLink->m_constraint;
}
*/



void dgWorld::BodySetMatrix (dgBody* const body, const dgMatrix& matrix)
{

	#define DG_RECURSIVE_SIZE	1024
	dgInt32 index;
	dgBody* queue[DG_RECURSIVE_SIZE];


	index = 1;
	queue[0] = body;
	m_genericLRUMark ++;
	body->m_genericLRUMark = m_genericLRUMark;
	dgMatrix relMatrix (body->GetMatrix().Inverse() * matrix);
	while (index) {
		dgBody* body;
		
		index --;
		body = queue[index];
		dgAssert (body != m_sentionelBody);

		m_broadPhase->Remove (body);
		m_broadPhase->Add (body);

		dgMatrix matrix (body->GetMatrix() * relMatrix);
		body->SetVelocity (dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)));    
		body->SetOmega (dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)));    
		body->SetMatrix (matrix);

		for (dgBodyMasterListRow::dgListNode* jointNode = body->m_masterNode->GetInfo().GetFirst(); jointNode; jointNode = jointNode->GetNext()) {
			dgBodyMasterListCell& cell = jointNode->GetInfo();
			body = cell.m_bodyNode;
			if (body != m_sentionelBody) {
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


/*
void dgWorld::AddBodyImpulse (dgBody* body, const dgVector& pointVeloc, const dgVector& pointPosit)
{
	if (body->m_invMass.m_w > dgFloat32 (0.0f)) {
//		UnfreezeBody (body);
		body->AddImpulse (pointVeloc, pointPosit);
	}
}

void dgWorld::ApplyImpulseArray (dgBody* body, dgInt32 count, dgInt32 strideInBytes, const dgFloat32* const impulseArray, const dgFloat32* const pointArray)
{
	if (body->m_invMass.m_w > dgFloat32 (0.0f)) {
		//		UnfreezeBody (body);
		body->ApplyImpulseArray (count, strideInBytes, impulseArray, pointArray);
	}
}


dgInt32 dgWorld::GetBodyArray (dgBody* root, dgBody** array, dgInt32 maxSize) const
{
	dgAssert (0);
	return 0;

	dgInt32 i;
	dgInt32 count;
	dgLink* ptr;
	dgLink* link;
	dgBody* body;
	dgConstraint* constraint;
	dgLink* stack[2048];
	const dgInt64 mask = dgInt64 (65536) * dgInt64 (65536) * dgInt64 (65536) * dgInt64 (4096);

	count = 0;
	link = root->m_firstConstraintLink; 
	if (link) {
		root->m_lru |= mask;
		stack[0] = link;
		i = 1;
		while (i) {
			i --;
			link = stack[i];
			ptr = link;
			do {
				body = ptr->m_twin->m_body;
				if (body) {
					dgAssert (body);
					if (~body->m_lru & mask) {
						constraint = ptr->m_constraint;
						if (constraint->IsBilateral()) {
							if (count <	maxSize) {
								array[count] = body; 
								count ++;
							}
							stack[i] = ptr->m_twin;
							i ++;
						}
						body->m_lru |= mask;
					}
				}

				ptr = ptr->m_twin->m_next;
			} while (ptr != link);
		}

		link = root->m_firstConstraintLink; 
		root->m_lru &= ~ mask;
		stack[0] = link;
		i = 1;
		while (i) {
			i --;
			link = stack[i];
			ptr = link;
			do {
				body = ptr->m_twin->m_body;
				if (body) {
					dgAssert (body);
					if (body->m_lru & mask) {
						constraint = ptr->m_constraint;
						if (constraint->IsBilateral()) {
							stack[i] = ptr->m_twin;
							i ++;
						}
						body->m_lru &= ~ mask;
					}
				}
				ptr = ptr->m_twin->m_next;
			} while (ptr != link);
		}
	}

	return count;
}


dgInt32 dgWorld::GetConstraintArray (dgConstraint* root, dgConstraint** constraintArray, dgInt32 maxSize) const
{
dgAssert (0);
return 0;

	dgInt32 i;
	dgInt32 stack;
	dgInt32 count;
	dgLink* ptr;
	dgLink* link;
	dgBody* body;
	dgConstraint* constraint;
	dgConstraint* constraintPtr;
	dgConstraint* stackPool[2048];
	const dgInt64 mask = dgInt64 (65536) * dgInt64 (65536) * dgInt64 (65536) * dgInt64 (4096);

	count = 0;
	stack = 1;
	stackPool[0] = root;
	root->m_lru |= mask;

	while (stack && (count < maxSize)) {
		stack --;
		constraint = stackPool[stack];
		constraintArray[count] = constraint;
		count ++;

		body = constraint->m_body0;
		if (body && (body->m_invMass.m_w != dgFloat32 (0.0f))) {
			link = body->m_firstConstraintLink; 
			dgAssert (link);
			ptr = link;
			do {
				constraintPtr = ptr->m_constraint;
				if (!(constraintPtr->m_lru & mask)) {
					constraintPtr->m_lru |= mask;
					stackPool[stack] = constraintPtr;
					stack ++;
				}
			
				ptr = ptr->m_twin->m_next;
			} while (ptr != link); 
		}

		body = constraint->m_body1;
		if (body && (body->m_invMass.m_w != dgFloat32 (0.0f))) {
			link = body->m_firstConstraintLink; 
			dgAssert (link);
			ptr = link;
			do {
				constraintPtr = ptr->m_constraint;
				if (!(constraintPtr->m_lru & mask)) {
					constraintPtr->m_lru |= mask;
					stackPool[stack] = constraintPtr;
					stack ++;
				}

				ptr = ptr->m_twin->m_next;
			} while (ptr != link); 
		}
	} 

	for (i = 0; i < stack; i ++) {
		constraint = stackPool[i];
		constraint->m_lru &= ~mask;
	}

	for (i = 0; i < count; i ++) {
		constraint = constraintArray[i];
		constraint->m_lru &= ~mask;

		#ifdef _DEBUG
		for (stack = i + 1; stack < count; stack ++) {                                                                 
			dgAssert (constraint != constraintArray[stack]);
		}
		#endif                             
	}

	return count;

}
*/

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
	//timestep = 1.0f/ 60.0f;
	//timestep = 1.0f/ 120.0f;
	//timestep = 1.0f/ 180.0f;
	//timestep = 1.0f/ 240.0f;
	//timestep = 1.0f/ 300.0f;
	//timestep = 1.0f/ 600.0f;
	//timestep = 1.0f/ 1000.0f;

	//m_cpu = dgNoSimdPresent;
	//m_cpu = dgSimdPresent;
	//m_solverMode = 1;

	//static int xxx ;
	//xxx ++;
	//if (xxx >= 2000)
	//xxx *=1;

	//xxxxx();

	//static int xxx;
	//dgTrace (("pass %d\n", xxx));
	//xxx ++;

//m_cpu = dgSimdPresent;
//m_cpu = dgNoSimdPresent;
#ifdef _POSIX_VER
	//m_cpu = dgNoSimdPresent;
#endif

	dgAssert (m_inUpdate == 0);
//SerializeToFile ("xxx.bin");

	m_destroyeddBodiesPool.m_count = 0;

	dgThreadHive::ClearTimers();
	memset (m_perfomanceCounters, 0, sizeof (m_perfomanceCounters));
	dgUnsigned32 ticks = m_getPerformanceCount();

	m_inUpdate ++;
	dgAssert (GetThreadCount() >= 1);

	if (m_cpu != dgNoSimdPresent) {

		simd_env rounding = dgSimd::get_ctrl();
		dgSimd::set_FZ_mode();

m_cpu = dgNoSimdPresent;
		m_broadPhase->UpdateContacts (timestep);
m_cpu = dgSimdPresent;
		UpdateDynamics (timestep);

		dgSimd::set_ctrl (rounding);

	} else {
		m_broadPhase->UpdateContacts (timestep);
		UpdateDynamics (timestep);
	}
	m_inUpdate --;

	if (m_destroyBodyByExeciveForce) {
		for (dgInt32 i = 0; i < m_destroyeddBodiesPool.m_count; i ++) {
			m_destroyBodyByExeciveForce (m_destroyeddBodiesPool.m_bodies[i], m_destroyeddBodiesPool.m_joint[i]);
		}
	}

	if (m_postListener.GetCount()) {
		dgUnsigned32 ticks = m_getPerformanceCount();
		for (dgListenerList::dgListNode* node = m_postListener.GetFirst(); node; node = node->GetNext()) {
			dgListener& listener = node->GetInfo();
			listener.m_onListenerUpdate (this, listener.m_userData, timestep);
		}
		m_perfomanceCounters[m_postUpdataListerTicks] = m_getPerformanceCount() - ticks;
	}

	m_perfomanceCounters[m_worldTicks] = m_getPerformanceCount() - ticks;
}



void dgWorld::Execute (dgInt32 threadID)
{
	// some graphics engines set this to 32 bit floats, and some newton algorithm 
	// make sure the round mode is set to double, even when using single precision,
//	#if (defined (_WIN_32_VER) || defined (_WIN_64_VER))
//		#ifndef __USE_DOUBLE_PRECISION__
//			dgUnsigned32 controlWorld = dgControlFP (0xffffffff, 0);
//			dgControlFP (_PC_53, _MCW_PC);
//		#endif
//	#endif

	if (threadID == DG_MUTEX_THREAD_ID) {
		dgMutexThread::Execute (threadID);
	} else {
		dgAsyncThread::Execute (threadID);
	}

//	#if (defined (_WIN_32_VER) || defined (_WIN_64_VER))
//		#ifndef __USE_DOUBLE_PRECISION__
//			dgControlFP (controlWorld, _MCW_PC);
//		#endif
//	#endif
}


void dgWorld::TickCallback (dgInt32 threadID)
{
	if (threadID == DG_MUTEX_THREAD_ID) {
		StepDynamics (m_savetimestep);
		memcpy (m_perfomanceCountersBack, m_perfomanceCounters, sizeof (m_perfomanceCounters));
	} else {
		Update (m_savetimestep);
	}
}


void dgWorld::Sync ()
{
	while (dgMutexThread::IsBusy()) {
		dgThreadYield();
	}
}


void dgWorld::Update (dgFloat32 timestep)
{
	m_savetimestep = timestep;

	#ifndef DG_USE_THREAD_EMULATION
		// runs the update in a separate thread but wait until the update is completed before it returns.
		// this will run well on single core systems, since the two thread are mutually exclusive 
		dgMutexThread::Tick();
	#else 
		dgFloatExceptions exceptions;
		StepDynamics (m_savetimestep);
		memcpy (m_perfomanceCountersBack, m_perfomanceCounters, sizeof (m_perfomanceCounters));
	#endif
}


void dgWorld::UpdateAsync (dgFloat32 timestep)
{
	m_savetimestep = timestep;

	#ifndef DG_USE_THREAD_EMULATION
		// execute one update, but do not wait for the update to finish, instead return immediately to the caller
		dgAsyncThread::Tick();
	#else 
		dgFloatExceptions exceptions;
		StepDynamics (m_savetimestep);
		memcpy (m_perfomanceCountersBack, m_perfomanceCounters, sizeof (m_perfomanceCounters));
	#endif
}


void dgWorld::SerializeToFile (const char* const fileName) const
{
	FILE* const file = fopen (fileName, "wb");

	dgBody** const array = new dgBody*[GetBodiesCount()];

	dgInt32 count = 0;
	const dgBodyMasterList& me = *this;
	for (dgBodyMasterList::dgListNode* node = me.GetFirst()->GetNext(); node; node = node->GetNext()) {
		const dgBodyMasterListRow& graphNode = node->GetInfo();
		array[count] = graphNode.GetBody();	
		count ++;
		dgAssert (count <= GetBodiesCount());
	}
	SerializeBodyArray (array, count, OnBodySerializeToFile, OnSerializeToFile, file);

	delete[] array;
	fclose (file);
}


void dgWorld::SerializeBodyArray (dgBody** const array, dgInt32 count, OnBodySerialize bodyCallback, dgSerialize serializeCallback, void* const userData) const
{
	dgSerializeMarker (serializeCallback, userData);

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

	serializeCallback(userData, &uniqueShapes, sizeof (uniqueShapes));	
	dgTree<dgInt32, const dgCollision*>::Iterator iter (shapeMap);
	for (iter.Begin(); iter; iter ++) {
		dgInt32 id = iter.GetNode()->GetInfo();
		const dgCollision* const collision = iter.GetKey();
		dgCollisionInstance instance (this, collision, 0, dgMatrix (dgGetIdentityMatrix()));
		serializeCallback(userData, &id, sizeof (id));	
		instance.Serialize(serializeCallback, userData);
		dgSerializeMarker(serializeCallback, userData);
	}

	serializeCallback(userData, &count, sizeof (count));	
	for (dgInt32 i = 0; i < count; i ++) {
		dgBody* const body = array[i];

		dgInt32 bodyType = body->GetType();
		serializeCallback(userData, &bodyType, sizeof (bodyType));	

		// serialize the body
		body->Serialize(&shapeMap, serializeCallback, userData);

		// serialize body custom data
		bodyCallback (*body, serializeCallback, userData);
		

		dgSerializeMarker(serializeCallback, userData);
	}
}


void dgWorld::OnSerializeToFile (void* const fileHandle, const void* const buffer, size_t size)
{
	dgAssert ((size & 0x03) == 0);
	fwrite (buffer, size, 1, (FILE*) fileHandle);
}


void dgWorld::OnBodySerializeToFile (dgBody& body, dgSerialize serializeCallback, void* const fileHandle)
{
	const char* const bodyIndentification = "NewtonGravityBody\0\0\0\0";
	int size = (dgInt32 (strlen (bodyIndentification)) + 3) & -4;
	serializeCallback (fileHandle, &size, sizeof (size));
	serializeCallback (fileHandle, bodyIndentification, size);
}




void dgWorld::DeserializeBodyArray (OnBodyDeserialize bodyCallback, dgDeserialize deserialization, void* const userData)
{
	dgDeserializeMarker (deserialization, userData);

	dgTree<const dgCollision*, dgInt32> shapeMap(GetAllocator());

	dgInt32 uniqueShapes;
	deserialization(userData, &uniqueShapes, sizeof (uniqueShapes));	
	for (dgInt32 i = 0; i < uniqueShapes; i ++) {
		dgInt32 id;

		deserialization(userData, &id, sizeof (id));	
		dgCollisionInstance instance (this, deserialization, userData);
		dgDeserializeMarker (deserialization, userData);

		const dgCollision* const shape = instance.GetChildShape();
		shapeMap.Insert(shape, id);
		shape->AddRef();
	}

	dgInt32 bodyCount;
	deserialization (userData, &bodyCount, sizeof (bodyCount));	
	for (dgInt32 i = 0; i < bodyCount; i ++) {
		dgInt32 bodyType;
		deserialization(userData, &bodyType, sizeof (bodyType));	
		dgBody* body = NULL; 

		switch (bodyType)
		{
			case dgBody::m_dynamicBody:
			{
				body = new (m_allocator) dgDynamicBody(this, &shapeMap, deserialization, userData);
				break;
			}
			case dgBody::m_kinamticBody:
			{
				body = new (m_allocator) dgKinematicBody(this, &shapeMap, deserialization, userData);
				break;
			}

			case dgBody::m_deformableBody:
			{
				dgAssert (0);
				break;
			}
		}

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
			dynBody->SetMassMatrix (dynBody->m_mass.m_w, dynBody->m_mass.m_x, dynBody->m_mass.m_y, dynBody->m_mass.m_z);
		}

		// load user related data 
		bodyCallback (*body, deserialization, userData);

		// sync to next body
		dgDeserializeMarker (deserialization, userData);

	}

	dgTree<const dgCollision*, dgInt32>::Iterator iter (shapeMap);
	for (iter.Begin(); iter; iter ++) {
		const dgCollision* const collision = iter.GetNode()->GetInfo();
		collision->Release();
	}

}


