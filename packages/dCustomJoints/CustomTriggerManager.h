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
	CUSTOM_CONTROLLER_GLUE(CustomTriggerController);

	public:
	void SetUserData(void* const userData);
	const void* GetUserData() const;
	
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

		PassangerManifest ();
		~PassangerManifest ();

		Passenger* Find (NewtonBody* const m_body);
		Passenger* Insert (NewtonBody* const m_body);

		int m_count;
		int m_capacity;
		Passenger* m_passangerList;
	};

	void Init (NewtonCollision* const convexShape, const dMatrix& matrix, void* const userData);

	virtual void PreUpdate(dFloat timestep, int threadIndex);
	virtual void PostUpdate(dFloat timestep, int threadIndex);
	
	private:
	void* m_userData;
	PassangerManifest m_manifest;
	friend class CustomTriggerManager;
};

class CustomTriggerManager: public CustomControllerManager<CustomTriggerController> 
{
	public:
	enum TriggerEventType
	{
		m_enterTrigger,
		m_inTrigger,
		m_exitTrigger,
	};

	CustomTriggerManager(NewtonWorld* const world);
	virtual ~CustomTriggerManager();

	virtual void PreUpdate(dFloat timestep);
	virtual void PostUpdate(dFloat timestep)
	{
		// bypass the entire Post Update call by not calling the base class
	}

	virtual void Debug () const {};
	virtual CustomTriggerController* CreateTrigger (const dMatrix& matrix, NewtonCollision* const convexShape, void* const userData);

	virtual void EventCallback (const CustomTriggerController* const me, TriggerEventType eventType, NewtonBody* const visitor) const = 0;

	unsigned m_lru;
};

inline void CustomTriggerController::SetUserData(void* const userData)
{
	m_userData = userData;
}

inline const void* CustomTriggerController::GetUserData() const
{
	return m_userData;
}


#endif 

