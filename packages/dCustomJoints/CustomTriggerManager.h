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


// NewtonPlayerControllerManager.h: interface for the NewtonPlayerControllerManager class.
//
//////////////////////////////////////////////////////////////////////

#ifndef D_CUSTOM_TRIGGER_MANAGER_H_
#define D_CUSTOM_TRIGGER_MANAGER_H_

#include <CustomJointLibraryStdAfx.h>
#include <CustomControllerManager.h>



#define TRIGGER_PLUGIN_NAME				"__triggerManager__"

// a trigger is volume of space that is there to send a message to other objects when and object enter of leave the trigger region  
// they are not visible and do not collide with bodies, but the generate contacts
class CustomTriggerController: public CustomControllerBase
{
	public:
	CUSTOM_JOINTS_API CustomTriggerController();
	CUSTOM_JOINTS_API ~CustomTriggerController();

	CUSTOM_JOINTS_API void Init (NewtonCollision* const convexShape, const dMatrix& matrix, void* const userData);
	CUSTOM_JOINTS_API virtual void PreUpdate(dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void PostUpdate(dFloat timestep, int threadIndex);
	
	private:
	dTree<NewtonBody*,NewtonBody*> m_manifest;
	friend class CustomTriggerManager;
};

class CustomTriggerManager: public CustomControllerManager<CustomTriggerController> 
{
	public:
	enum TriggerEventType
	{
		m_inTrigger,
		m_enterTrigger,
		m_exitTrigger,
	};

	CUSTOM_JOINTS_API CustomTriggerManager (NewtonWorld* const world);
	CUSTOM_JOINTS_API virtual ~CustomTriggerManager();

	CUSTOM_JOINTS_API virtual void PreUpdate(dFloat timestep);
	virtual void PostUpdate(dFloat timestep)
	{
		// bypass the entire Post Update call by not calling the base class
	}

	CUSTOM_JOINTS_API virtual void OnDestroyBody (NewtonBody* const body); 

	virtual void Debug () const {};
	CUSTOM_JOINTS_API virtual CustomTriggerController* CreateTrigger (const dMatrix& matrix, NewtonCollision* const convexShape, void* const userData);

	CUSTOM_JOINTS_API virtual void EventCallback (const CustomTriggerController* const me, TriggerEventType eventType, NewtonBody* const guess) const = 0;

	private:
	void UpdateTrigger (CustomTriggerController* const controller);
	static void UpdateTrigger (NewtonWorld* const world, void* const context, int threadIndex);

	unsigned m_lock;
	friend class CustomTriggerController;
};


#endif 


