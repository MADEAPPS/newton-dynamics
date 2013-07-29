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



#define CUSTOM_CONTROLLER_GLUE(ControllerClass)													\
		public:																					\
		ControllerClass() {}																	\
		virtual ~ControllerClass() {}															\
		NEWTON_API virtual NewtonBody* GetBody() const = 0;										\
		NEWTON_API virtual void SetBody(NewtonBody* const body) = 0;							\
		NEWTON_API virtual CustomControllerManager<ControllerClass>* GetManager() const = 0;	\
		private:


class CustomControllerBase: public CustomAlloc
{
	public:
	dFloat GetTimeStep() const {return m_curTimestep;}
	NewtonWorld* GetWorld() const {return m_world;}
	
	virtual void Debug () const = 0;

	protected:
	NEWTON_API CustomControllerBase(NewtonWorld* const world, const char* const managerName);
	NEWTON_API virtual ~CustomControllerBase();

	virtual void PreUpdate (dFloat timestep) = 0;
	virtual void PostUpdate (dFloat timestep) = 0;

	private:
	static void PreUpdate (const NewtonWorld* const world, void* const listenerUserData, dFloat timestep);
	static void PostUpdate (const NewtonWorld* const world, void* const listenerUserData, dFloat timestep);
	static void Destroy (const NewtonWorld* const world, void* const listenerUserData);
	
	dFloat m_curTimestep;
	NewtonWorld* m_world;
};

template<class CONTROLLER_BASE>
class CustomControllerManager: public CustomControllerBase  
{
	public:
	class CustomController: public CustomAlloc, public CONTROLLER_BASE   
	{
		public:
		CustomController();
		virtual ~CustomController();
		
		virtual NewtonBody* GetBody() const;
		virtual void SetBody(NewtonBody* const body);
		virtual CustomControllerManager* GetManager() const; 
		
		virtual void PreUpdate(dFloat timestep, int threadIndex);
		virtual void PostUpdate(dFloat timestep, int threadIndex);

		private:
		NewtonBody* m_body;
		CustomController* m_prev;
		CustomController* m_next;
		CustomControllerManager* m_manager; 
		friend class CustomControllerManager;
	};

	CustomControllerManager(NewtonWorld* const world, const char* const managerName);
	virtual ~CustomControllerManager();

	virtual CustomController* CreateController ();
	virtual void DestroyController (CustomController* const controller);

	CustomController* GetFirstController() const;
	CustomController* GetNextController(CustomController* const node) const;

	protected:
	virtual void PreUpdate (dFloat timestep);
	virtual void PostUpdate (dFloat timestep);
	static void PreUpdateKernel (NewtonWorld* const world, void* const context, int threadIndex);
	static void PostUpdateKernel (NewtonWorld* const world, void* const context, int threadIndex);

	CustomController* m_controllerList;
};



template<class CONTROLLER_BASE>
CustomControllerManager<CONTROLLER_BASE>::CustomController::CustomController()
	:m_body(NULL)
	,m_manager(NULL)
{
}

template<class CONTROLLER_BASE>
CustomControllerManager<CONTROLLER_BASE>::CustomController::~CustomController()
{
}

template<class CONTROLLER_BASE>
CustomControllerManager<CONTROLLER_BASE>* CustomControllerManager<CONTROLLER_BASE>::CustomController::GetManager() const 
{
	return m_manager;
}

template<class CONTROLLER_BASE>
NewtonBody* CustomControllerManager<CONTROLLER_BASE>::CustomController::GetBody() const 
{
	return m_body;
}

template<class CONTROLLER_BASE>
void CustomControllerManager<CONTROLLER_BASE>::CustomController::SetBody(NewtonBody* const body)
{
	m_body = body;
}

template<class CONTROLLER_BASE>
void CustomControllerManager<CONTROLLER_BASE>::CustomController::PreUpdate(dFloat timestep, int threadIndex)
{
	CONTROLLER_BASE::PreUpdate(timestep, threadIndex);
}

template<class CONTROLLER_BASE>
void CustomControllerManager<CONTROLLER_BASE>::CustomController::PostUpdate(dFloat timestep, int threadIndex)
{
	CONTROLLER_BASE::PostUpdate(timestep, threadIndex);
}




template<class CONTROLLER_BASE>
CustomControllerManager<CONTROLLER_BASE>::CustomControllerManager(NewtonWorld* const world, const char* const managerName)
	:CustomControllerBase(world, managerName)
	,m_controllerList(NULL)
{
}


template<class CONTROLLER_BASE>
CustomControllerManager<CONTROLLER_BASE>::~CustomControllerManager()
{
	while (m_controllerList) {
		DestroyController (m_controllerList);
	}
}

template<class CONTROLLER_BASE>
void CustomControllerManager<CONTROLLER_BASE>::PreUpdateKernel (NewtonWorld* const world, void* const context, int threadIndex)
{
	CustomController* const controller = (CustomController*) context;
	controller->PreUpdate(controller->m_manager->GetTimeStep(), threadIndex);
}

template<class CONTROLLER_BASE>
void CustomControllerManager<CONTROLLER_BASE>::PostUpdateKernel (NewtonWorld* const world, void* const context, int threadIndex)
{
	CustomController* const controller = (CustomController*) context;
	controller->PostUpdate(controller->m_manager->GetTimeStep(), threadIndex);
}


template<class CONTROLLER_BASE>
void CustomControllerManager<CONTROLLER_BASE>::PreUpdate (dFloat timestep)
{
	NewtonWorld* const world = GetWorld();
	for (CustomController* controller = m_controllerList; controller; controller = controller->m_next) {
		NewtonDispachThreadJob(world, PreUpdateKernel, controller);
	}
	NewtonSyncThreadJobs(world);
}

template<class CONTROLLER_BASE>
void CustomControllerManager<CONTROLLER_BASE>::PostUpdate (dFloat timestep)
{
	NewtonWorld* const world = GetWorld();
	for (CustomController* controller = m_controllerList; controller; controller = controller->m_next) {
		NewtonDispachThreadJob(world, PostUpdateKernel, controller);
	}
	NewtonSyncThreadJobs(world);

}


template<class CONTROLLER_BASE>
typename CustomControllerManager<CONTROLLER_BASE>::CustomController* CustomControllerManager<CONTROLLER_BASE>::CreateController ()
{
	CustomController* const controller = new CustomController();

	controller->m_prev = NULL;
	controller->m_next = m_controllerList;
	m_controllerList = controller;

	controller->m_manager = this;
	return controller;
}

template<class CONTROLLER_BASE>
void CustomControllerManager<CONTROLLER_BASE>::DestroyController (CustomController* const controller) 
{
	if (controller == m_controllerList) {
		m_controllerList = controller->m_next;
	}

	if (controller->m_body) {
		NewtonDestroyBody (GetWorld(), controller->m_body);
	}

	if (controller->m_next) {
		controller->m_next->m_prev = controller->m_prev;
	}
	if (controller->m_prev) {
		controller->m_prev->m_next = controller->m_next;
	}
	controller->m_next = NULL;
	controller->m_prev = NULL;
	delete controller;
}

template<class CONTROLLER_BASE>
typename CustomControllerManager<CONTROLLER_BASE>::CustomController* CustomControllerManager<CONTROLLER_BASE>::GetFirstController() const 
{
	return m_controllerList;
}

template<class CONTROLLER_BASE>
typename CustomControllerManager<CONTROLLER_BASE>::CustomController* CustomControllerManager<CONTROLLER_BASE>::GetNextController(CustomController* const controller) const 
{
	return controller->m_next;
}









class CustomControllerBase____
{
	protected:
	virtual ~CustomControllerBase____();

	virtual void PreUpdate (dFloat timestep);
	virtual void PostUpdate (dFloat timestep);
};


template<class CUSTOM_CONTOLLER>
class CustomControllerManager____: public CustomAlloc
{
	public:
	
	template<class CUSTOM_CONTOLLER>
	class CustomController: public CustomControllerBase____
	{
		public:
		CustomController();
		~CustomController();

		CustomControllerManager____<CUSTOM_CONTOLLER>* GetManager() const;

		CustomControllerManager____<CUSTOM_CONTOLLER>* m_manager;

		friend class CustomControllerManager____<CUSTOM_CONTOLLER>;
	};

	class CustonControllerList: public CustomList<CustomController<CUSTOM_CONTOLLER> > 
	{
	};

	CustomControllerManager____(NewtonWorld* const world, const char* const managerName);
	virtual ~CustomControllerManager____();

	dFloat GetTimeStep() const 
	{
		return m_curTimestep;
	}

//	virtual CustomController* CreateController ();
//	virtual void DestroyController (CustomController* const controller);

	private:
	static void Destroy (const NewtonWorld* const world, void* const listenerUserData);
	static void PreUpdate (const NewtonWorld* const world, void* const listenerUserData, dFloat timestep);
	static void PostUpdate (const NewtonWorld* const world, void* const listenerUserData, dFloat timestep);
	static void PreUpdateKernel (NewtonWorld* const world, void* const context, int threadIndex);
	static void PostUpdateKernel (NewtonWorld* const world, void* const context, int threadIndex);

	protected:
	virtual void PreUpdate (dFloat timestep);
	virtual void PostUpdate (dFloat timestep);

//	virtual void PreUpdate (dFloat timestep);
//	virtual void PostUpdate (dFloat timestep);

	dFloat m_curTimestep;
	NewtonWorld* m_world;
	CustonControllerList m_controllerList;
};


template<class CUSTOM_CONTOLLER>
CustomControllerManager____<CUSTOM_CONTOLLER>::CustomControllerManager____(NewtonWorld* const world, const char* const managerName)
	:CustomAlloc()
	,m_curTimestep(0.0f)
	,m_world(world)
{
	NewtonWorldAddPreListener (world, managerName, this, PreUpdate, NULL);
	NewtonWorldAddPostListener (world, managerName, this, PostUpdate, Destroy);
}

template<class CUSTOM_CONTOLLER>
CustomControllerManager____<CUSTOM_CONTOLLER>::~CustomControllerManager____()
{
//	dAssert (0);
}

template<class CUSTOM_CONTOLLER>
void CustomControllerManager____<CUSTOM_CONTOLLER>::PreUpdate (const NewtonWorld* const world, void* const listenerUserData, dFloat timestep)
{
	CustomControllerManager____* const me = (CustomControllerManager____*) listenerUserData;
	dAssert (me->m_world == world);
	me->m_curTimestep = timestep;
	me->PreUpdate(timestep);
}

template<class CUSTOM_CONTOLLER>
void CustomControllerManager____<CUSTOM_CONTOLLER>::PostUpdate (const NewtonWorld* const world, void* const listenerUserData, dFloat timestep)
{
	CustomControllerManager____* const me = (CustomControllerManager____*) listenerUserData;
	dAssert (me->m_world == world);
	me->m_curTimestep = timestep;
	me->PostUpdate(timestep);
}


template<class CUSTOM_CONTOLLER>
void CustomControllerManager____<CUSTOM_CONTOLLER>::Destroy (const NewtonWorld* const world, void* const listenerUserData)
{
	CustomControllerManager____* const me = (CustomControllerManager____*) listenerUserData;
	delete me;
}

template<class CUSTOM_CONTOLLER>
void CustomControllerManager____<CUSTOM_CONTOLLER>::PreUpdate (dFloat timestep)
{
	for (CustonControllerList::CustomListNode* controllerNode = m_controllerList.GetFirst(); controllerNode; controllerNode = controllerNode->GetNext()) {
		dAssert (0);
		NewtonDispachThreadJob(m_world, PreUpdateKernel, &controllerNode->GetInfo());
	}
	NewtonSyncThreadJobs(m_world);
}

template<class CUSTOM_CONTOLLER>
void CustomControllerManager____<CUSTOM_CONTOLLER>::PostUpdate (dFloat timestep)
{
	dAssert (0);
	for (CustonControllerList::CustomListNode* controllerNode = m_controllerList.GetFirst(); controllerNode; controllerNode = controllerNode->GetNext()) {
		NewtonDispachThreadJob(m_world, PreUpdateKernel, &controllerNode->GetInfo());
	}
	NewtonSyncThreadJobs(m_world);
}

template<class CUSTOM_CONTOLLER>
void CustomControllerManager____<CUSTOM_CONTOLLER>::PreUpdateKernel (NewtonWorld* const world, void* const context, int threadIndex)
{
	dAssert (0);
	CustomController<CUSTOM_CONTOLLER>* const controller = (CustomController<CUSTOM_CONTOLLER>*) context;
//	controller->PreUpdate(controller->m_manager->GetTimeStep(), threadIndex);
	controller->PreUpdate(controller->m_manager->GetTimeStep());
}

template<class CUSTOM_CONTOLLER>
void CustomControllerManager____<CUSTOM_CONTOLLER>::PostUpdateKernel (NewtonWorld* const world, void* const context, int threadIndex)
{
	dAssert (0);
	CustomController<CUSTOM_CONTOLLER>* const controller = (CustomController<CUSTOM_CONTOLLER>*) context;
	//controller->PostUpdate(controller->m_manager->GetTimeStep(), threadIndex);
	controller->PostUpdate(controller->m_manager->GetTimeStep());
}


#endif 



