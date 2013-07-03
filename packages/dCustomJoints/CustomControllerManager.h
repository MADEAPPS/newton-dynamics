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


class CustomControllerFilterCastFilter
{	
	public:
	NEWTON_API CustomControllerFilterCastFilter (const NewtonBody* const me)
	{
		m_count = 1;
		m_filter[0] = me;
	}

	NEWTON_API static unsigned ConvexStaticCastPrefilter(const NewtonBody* const body, const NewtonCollision* const myCollision, void* const userData)
	{
		CustomControllerFilterCastFilter* const filter = (CustomControllerFilterCastFilter*) userData;
		const NewtonCollision* const collision = NewtonBodyGetCollision(body);
		if (NewtonCollisionGetMode(collision)) {
			for (int i = 0; i < filter->m_count; i ++) {
				if (body == filter->m_filter[i]) {
					return 0;
				}
			}
			return 1;
		}
		return 0;
	}

	int m_count;
	const NewtonBody* m_filter[32];
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
	
//	dRttiRootClassSupportDeclare(CustomControllerBase);
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
//	CustomControllerList m_controllerList;
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




#endif 

