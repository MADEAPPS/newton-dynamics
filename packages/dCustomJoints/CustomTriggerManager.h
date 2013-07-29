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

#include "CustomJointLibraryStdAfx.h"
#include "CustomControllerManager.h"



#define TRIGGER_PLUGIN_NAME				"triggerManager"

// a trigger is volume of space that is there to send a message to other objects when and object enter of leave the trigger region  
// they are not visible and do not collide with bodies, but the generate contacts
class CustomTriggerController
{
//	CUSTOM_CONTROLLER_GLUE(CustomTriggerController);

	protected:
	class PassangerManifest
	{
		public:
		class Passenger
		{
			public:
			unsigned m_lru;
			NewtonBody* m_body;
		};

		NEWTON_API PassangerManifest ();
		NEWTON_API ~PassangerManifest ();

		NEWTON_API Passenger* Find (NewtonBody* const m_body);
		NEWTON_API Passenger* Insert (NewtonBody* const m_body);

		int m_count;
		int m_capacity;
		Passenger* m_passangerList;
	};

	public:
	NEWTON_API const void* GetUserData() const;
	NEWTON_API void SetUserData(void* const userData);
	NEWTON_API void Init (NewtonCollision* const convexShape, const dMatrix& matrix, void* const userData);
	NEWTON_API virtual void PreUpdate(dFloat timestep, int threadIndex);
	NEWTON_API virtual void PostUpdate(dFloat timestep, int threadIndex);
	
	private:
	void* m_userData;
	PassangerManifest m_manifest;
	friend class CustomTriggerManager;
};

class CustomTriggerManager: public CustomControllerManager____<CustomTriggerController> 
{
	public:
	enum TriggerEventType
	{
		m_enterTrigger,
		m_inTrigger,
		m_exitTrigger,
	};

	NEWTON_API CustomTriggerManager(NewtonWorld* const world);
	NEWTON_API virtual ~CustomTriggerManager();

	NEWTON_API virtual void PreUpdate(dFloat timestep);
	virtual void PostUpdate(dFloat timestep)
	{
		// bypass the entire Post Update call by not calling the base class
	}

	virtual void Debug () const {};
	NEWTON_API virtual CustomTriggerController* CreateTrigger (const dMatrix& matrix, NewtonCollision* const convexShape, void* const userData);

	NEWTON_API virtual void EventCallback (const CustomTriggerController* const me, TriggerEventType eventType, NewtonBody* const visitor) const = 0;

	unsigned m_lru;
};


#endif 

