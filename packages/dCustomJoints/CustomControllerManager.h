/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// CustomControllerManager.h: interface for the CustomControllerManager class.
//
//////////////////////////////////////////////////////////////////////

#ifndef D_CUSTOM_CONTROLLER_MANAGER_H_
#define D_CUSTOM_CONTROLLER_MANAGER_H_

#include "CustomJointLibraryStdAfx.h"
#include "CustomList.h"
#include "dMathDefines.h"
#include "dVector.h"
#include "dMatrix.h"
#include "dQuaternion.h"
#include "CustomAlloc.h"


class CustomControllerConvexCastPreFilter
{	
	public:
	NEWTON_API CustomControllerConvexCastPreFilter (const NewtonBody* const me)
		:m_bodiesToSkipCount(1)
	{
		m_bodiesToSkip[0] = me;
	}

	NEWTON_API static unsigned Prefilter(const NewtonBody* const body, const NewtonCollision* const myCollision, void* const userData)
	{
		CustomControllerConvexCastPreFilter* const filter = (CustomControllerConvexCastPreFilter*) userData;
		const NewtonCollision* const collision = NewtonBodyGetCollision(body);
		if (NewtonCollisionGetMode(collision)) {
			for (int i = 0; i < filter->m_bodiesToSkipCount; i ++) {
				if (body == filter->m_bodiesToSkip[i]) {
					return 0;
				}
			}
			return 1;
		}
		return 0;
	}

	const NewtonBody* m_bodiesToSkip[16];
	int m_bodiesToSkipCount;
};


class CustomControllerConvexRayFilter: public CustomControllerConvexCastPreFilter
{	
	public:
	NEWTON_API CustomControllerConvexRayFilter (const NewtonBody* const me)
		:CustomControllerConvexCastPreFilter(me)
		,m_hitBody(NULL)
		,m_shapeHit(NULL)
		,m_collisionID(NULL) 
		,m_intersectParam(1.2f)
	{
	}

	NEWTON_API static dFloat Filter (const NewtonBody* const body, const NewtonCollision* const shapeHit, const dFloat* const hitContact, const dFloat* const hitNormal, dLong collisionID, void* const userData, dFloat intersectParam)
	{
		CustomControllerConvexRayFilter* const filter = (CustomControllerConvexRayFilter*) userData;
		dAssert (body != filter->m_bodiesToSkip[0]);
		if (intersectParam < filter->m_intersectParam) {
			filter->m_hitBody = body;	
			filter->m_shapeHit = shapeHit;
			filter->m_collisionID = collisionID;
			filter->m_intersectParam = intersectParam;
			filter->m_hitContact = dVector(hitContact[0], hitContact[1], hitContact[2], 0.0f); 
			filter->m_hitNormal = dVector(hitNormal[0], hitNormal[1], hitNormal[2], 0.0f); 
		}
		return intersectParam;
	}
	
	dVector m_hitContact; 
	dVector m_hitNormal;
	const NewtonBody* m_hitBody;
	const NewtonCollision* m_shapeHit;
	dLong m_collisionID; 
	dFloat m_intersectParam;
};


class CustomControllerBase
{
	public:
	CustomControllerBase ()
		:m_userData(NULL)
		,m_body(NULL)
		,m_manager(NULL)
	{
	}

	virtual ~CustomControllerBase ()
	{
	}

	void* GetUserData () const 
	{
		return m_userData;
	}

	void SetUserData (void* const userData)
	{
		m_userData = userData;
	}

	NewtonBody* GetBody() const 
	{
		return m_body;
	}

	void* GetManager() const 
	{
		return m_manager;
	}

	virtual void PreUpdate (dFloat timestep, int threadIndex) = 0;
	virtual void PostUpdate (dFloat timestep, int threadIndex) = 0;

	void* m_userData;
	NewtonBody* m_body;
	void* m_manager;
};

template<class CONTROLLER_BASE>
class CustomControllerManager: public CustomList<CONTROLLER_BASE>
{
	public:
	CustomControllerManager(NewtonWorld* const world, const char* const managerName);
	~CustomControllerManager();

	NewtonWorld* GetWorld() const 
	{
		return m_world;
	}

	dFloat GetTimeStep() const 
	{
		return m_curTimestep;
	}

	virtual CONTROLLER_BASE* CreateController ();
	virtual void DestroyController (CONTROLLER_BASE* const controller);

    virtual void Debug () const;
	virtual void PreUpdate(dFloat timestep);
	virtual void PostUpdate(dFloat timestep);

	private:
	void DestroyAllController ();

	static void Destroy (const NewtonWorld* const world, void* const listenerUserData);
	static void PreUpdate (const NewtonWorld* const world, void* const listenerUserData, dFloat timestep);
	static void PostUpdate (const NewtonWorld* const world, void* const listenerUserData, dFloat timestep);

	static void PreUpdateKernel (NewtonWorld* const world, void* const context, int threadIndex);
	static void PostUpdateKernel (NewtonWorld* const world, void* const context, int threadIndex);

	protected:
	NewtonWorld* m_world;
	dFloat m_curTimestep;
};


template<class CONTROLLER_BASE>
CustomControllerManager<CONTROLLER_BASE>::CustomControllerManager(NewtonWorld* const world, const char* const managerName)
	:m_world(world)
	,m_curTimestep(0.0f)
{
	NewtonWorldAddPreListener (world, managerName, this, PreUpdate, NULL);
	NewtonWorldAddPostListener (world, managerName, this, PostUpdate, Destroy);
}

template<class CONTROLLER_BASE>
CustomControllerManager<CONTROLLER_BASE>::~CustomControllerManager()
{
}

template<class CONTROLLER_BASE>
void CustomControllerManager<CONTROLLER_BASE>::DestroyAllController ()
{
	while (GetCount()) {
		DestroyController (&GetLast()->GetInfo());
	}
}

template<class CONTROLLER_BASE>
void CustomControllerManager<CONTROLLER_BASE>::Debug () const
{
}

template<class CONTROLLER_BASE>
void CustomControllerManager<CONTROLLER_BASE>::PreUpdate(dFloat timestep)
{
	//	NewtonWorld* const world = GetWorld();
	for (CustomListNode* node = GetFirst(); node; node = node->GetNext()) {
		NewtonDispachThreadJob(m_world, PreUpdateKernel, &node->GetInfo());
	}
	NewtonSyncThreadJobs(m_world);
}

template<class CONTROLLER_BASE>
void CustomControllerManager<CONTROLLER_BASE>::PostUpdate(dFloat timestep)
{
	for (CustomListNode* node = GetFirst(); node; node = node->GetNext()) {
		NewtonDispachThreadJob(m_world, PostUpdateKernel, &node->GetInfo());
	}
	NewtonSyncThreadJobs(m_world);
}


template<class CONTROLLER_BASE>
void CustomControllerManager<CONTROLLER_BASE>::PreUpdate (const NewtonWorld* const world, void* const listenerUserData, dFloat timestep)
{
	CustomControllerManager* const me = (CustomControllerManager*) listenerUserData;
	me->m_curTimestep = timestep;
	me->PreUpdate(timestep);
}

template<class CONTROLLER_BASE>
void CustomControllerManager<CONTROLLER_BASE>::PostUpdate (const NewtonWorld* const world, void* const listenerUserData, dFloat timestep)
{
	CustomControllerManager* const me = (CustomControllerManager*) listenerUserData;
	me->m_curTimestep = timestep;
	me->PostUpdate(timestep);
}

template<class CONTROLLER_BASE>
void CustomControllerManager<CONTROLLER_BASE>::Destroy (const NewtonWorld* const world, void* const listenerUserData)
{
	CustomControllerManager* const me = (CustomControllerManager*) listenerUserData;
	me->DestroyAllController ();
	delete me;
}


template<class CONTROLLER_BASE>
void CustomControllerManager<CONTROLLER_BASE>::PreUpdateKernel (NewtonWorld* const world, void* const context, int threadIndex)
{
	CustomControllerBase* const controller = (CustomControllerBase*) context;
	controller->PreUpdate(((CustomControllerManager*)controller->m_manager)->GetTimeStep(), threadIndex);
}

template<class CONTROLLER_BASE>
void CustomControllerManager<CONTROLLER_BASE>::PostUpdateKernel (NewtonWorld* const world, void* const context, int threadIndex)
{
	class CustomControllerBase* const controller = (CustomControllerBase*) context;
	controller->PostUpdate(((CustomControllerManager*)controller->m_manager)->GetTimeStep(), threadIndex);
}



template<class CONTROLLER_BASE>
CONTROLLER_BASE* CustomControllerManager<CONTROLLER_BASE>::CreateController ()
{
	CONTROLLER_BASE* const controller = &Append()->GetInfo();

	controller->m_manager = this;
	return controller;
}

template<class CONTROLLER_BASE>
void CustomControllerManager<CONTROLLER_BASE>::DestroyController (CONTROLLER_BASE* const controller)
{
	dAssert (FindNodeFromInfo (*controller));
	CustomListNode* const node = GetNodeFromInfo (*controller);
	Remove (node);
}

#endif 

