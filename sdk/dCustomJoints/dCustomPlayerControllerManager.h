/* Copyright (c) <2003-2016> <Newton Game Dynamics>
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

#ifndef D_CUSTOM_PLAYER_CONTROLLER_MANAGER_H_
#define D_CUSTOM_PLAYER_CONTROLLER_MANAGER_H_

#include "dCustomJointLibraryStdAfx.h"
#include "dCustomListener.h"


#define PLAYER_PLUGIN_NAME				"__playerManager__"
#define PLAYER_CONTROLLER_MAX_CONTACTS	32
#define PLAYER_MIN_RESTRAINING_DISTANCE	1.0e-2f

class dCustomPlayerControllerManager;

class dCustomPlayerController
{
	public:
	dCustomPlayerController ()
		:m_impulse(0.0f)
		,m_mass(0.0f)
		,m_invMass(0.0f)
		,m_userData(NULL)
		,m_kinematicBody(NULL)
		,m_manager(NULL)
	{
	}

	~dCustomPlayerController () 
	{
	}

	void* GetUserData () const {return m_userData;}
	NewtonBody* GetBody() {return m_kinematicBody;}
	void SetUserData(void* const userData) {m_userData = userData;}
	dCustomPlayerControllerManager* GetManager() const {return m_manager;}

	const dFloat GetMass() { return m_mass;}
	const dVector& GetImpulse() {return m_impulse;}
	void SetImpulse(const dVector& impulse) { m_impulse = impulse;}

	CUSTOM_JOINTS_API dVector GetVelocity() const;
	CUSTOM_JOINTS_API void SetVelocity(const dVector& veloc);

	private:
	void PreUpdate(dFloat timestep);
	void PostUpdate(dFloat timestep);

	void ResolveCollision();
	dFloat PredictTimestep(dFloat timestep);
	static unsigned dCustomPlayerController::PrefilterCallback(const NewtonBody* const body, const NewtonCollision* const collision, void* const userData);

	dVector m_impulse;
	dFloat m_mass;
	dFloat m_invMass;
	void* m_userData;
	NewtonBody* m_kinematicBody;
	dCustomPlayerControllerManager* m_manager;

	friend class dCustomPlayerControllerManager;
};

class dCustomPlayerControllerManager: public dCustomParallelListener
{
	public:
	CUSTOM_JOINTS_API dCustomPlayerControllerManager(NewtonWorld* const world);
	CUSTOM_JOINTS_API ~dCustomPlayerControllerManager();

	CUSTOM_JOINTS_API virtual void ApplyPlayerMove(dCustomPlayerController* const controller, dFloat timestep) = 0;
	CUSTOM_JOINTS_API virtual dCustomPlayerController* CreatePlayerController(const dMatrix& location, const dMatrix& localAxis, dFloat mass, dFloat radius, dFloat height);
	CUSTOM_JOINTS_API virtual int ProcessContacts(const dCustomPlayerController* const controller, NewtonWorldConvexCastReturnInfo* const contacts, int count) const;

	protected:
	CUSTOM_JOINTS_API virtual void PreUpdate(dFloat timestep, int threadID);
	CUSTOM_JOINTS_API virtual void PostUpdate(dFloat timestep, int threadID);

	dList<dCustomPlayerController> m_playerList;
};

#endif 

