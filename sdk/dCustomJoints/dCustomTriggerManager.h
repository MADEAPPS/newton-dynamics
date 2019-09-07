/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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

#include "dCustomJointLibraryStdAfx.h"
#include "dCustomListener.h"

#define TRIGGER_PLUGIN_NAME				"__triggerManager__"
// a trigger is volume of space that is there to send a message 
// to other objects when and object enter of leave the trigger region  
// they are not visible and do not collide with bodies, but the generate contacts


class dCustomTriggerManager;

class dCustomTriggerController
{
	class dTriggerManifest: public dTree<unsigned, NewtonBody*>
	{
	};

	public:
	dCustomTriggerController()
		:m_manifest()
		,m_userData(NULL)
		,m_kinematicBody(NULL)
		,m_manager(NULL)
	{
	}

	~dCustomTriggerController()
	{
		NewtonDestroyBody(m_kinematicBody);
	}
	
	void* GetUserData() const {return m_userData;}
	void SetUserData(void* const data) {m_userData = data;}

	NewtonBody* GetBody() const {return m_kinematicBody; }
	dCustomTriggerManager* GetManager() const { return m_manager; }

	private:
	dTriggerManifest m_manifest;
	void* m_userData;
	NewtonBody* m_kinematicBody;
	dCustomTriggerManager* m_manager;

	friend class dCustomTriggerManager;
};

class dCustomTriggerManager: public dCustomParallelListener
{
	class dTriggerGuestPair
	{
		public:
		dCustomTriggerController* m_trigger;
		dCustomTriggerController::dTriggerManifest::dTreeNode* m_bodyNode;
	};

	public:
	CUSTOM_JOINTS_API dCustomTriggerManager (NewtonWorld* const world);
	CUSTOM_JOINTS_API virtual ~dCustomTriggerManager();
	
	CUSTOM_JOINTS_API virtual dCustomTriggerController* CreateTrigger (const dMatrix& matrix, NewtonCollision* const convexShape, void* const userData);
	CUSTOM_JOINTS_API virtual void DestroyTrigger (dCustomTriggerController* const trigger);

	virtual void OnEnter (const dCustomTriggerController* const me, NewtonBody* const guess) const {}
	virtual void OnExit (const dCustomTriggerController* const me, NewtonBody* const guess) const {}
	virtual void WhileIn (const dCustomTriggerController* const me, NewtonBody* const guess) const {}

	dList<dCustomTriggerController>& GetControllersList () {return m_triggerList;}
	const dList<dCustomTriggerController>& GetControllersList () const {return m_triggerList;}

	protected:
	CUSTOM_JOINTS_API virtual void OnDestroy();
	CUSTOM_JOINTS_API void PreUpdate(dFloat timestep, int threadID);
	CUSTOM_JOINTS_API virtual void OnDestroyBody (NewtonBody* const body); 

	virtual void OnDebug(dCustomJoint::dDebugDisplay* const debugContext, const dCustomTriggerController* const controller, const NewtonBody* const guess) const 
	{
	}

	CUSTOM_JOINTS_API virtual void PreUpdate(dFloat timestep);

	virtual void PostUpdate(dFloat timestep)
	{
		// bypass the entire Post Update call by not calling the base class
	}

	private:
	CUSTOM_JOINTS_API void OnDebug(dCustomJoint::dDebugDisplay* const debugContext);

	dList<dCustomTriggerController> m_triggerList;
	dArray<dTriggerGuestPair> m_pairCache;
	dFloat m_timestep;
	int m_cacheCount;
	unsigned m_lock;
	unsigned m_lru;
};


#endif 


