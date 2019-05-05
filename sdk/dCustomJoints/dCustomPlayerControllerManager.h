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
		:m_localFrame(dGetIdentityMatrix())
		,m_impulse(0.0f)
		,m_mass(0.0f)
		,m_invMass(0.0f)
		,m_friction(1.0f)
		,m_headingAngle(0.0f)
		,m_forwardSpeed(0.0f)
		,m_lateralSpeed(0.0f)
		,m_userData(NULL)
		,m_kinematicBody(NULL)
		,m_manager(NULL)
	{
		//m_forwardSpeed = 1.0f;
		//m_lateralSpeed = 1.0f;
		//m_headingAngle = 0.0f * dDegreeToRad;
	}

	~dCustomPlayerController () 
	{
	}

	void* GetUserData () const {return m_userData;}
	NewtonBody* GetBody() {return m_kinematicBody;}
	void SetUserData(void* const userData) {m_userData = userData;}
	dCustomPlayerControllerManager* GetManager() const {return m_manager;}

	const dFloat GetMass() const { return m_mass;}

	dFloat GetFriction() const { return m_friction;}
	void SetFriction(dFloat friction) {m_friction = dClamp (friction, dFloat (0.0f), dFloat (3.0f));}

	const dVector& GetImpulse() { return m_impulse; }
	void SetImpulse(const dVector& impulse) { m_impulse = impulse;}

	dFloat GetForwardSpeed() const { return -m_forwardSpeed; }
	void SetForwardSpeed(dFloat speed) {m_forwardSpeed = -speed; }

	dFloat GetLateralSpeed() const { return -m_lateralSpeed; }
	void SetLateralSpeed(dFloat speed) { m_lateralSpeed = -speed; }

	dFloat GetHeadingAngle() const { return m_headingAngle; }
	void SetHeadingAngle(dFloat angle) {m_headingAngle = dClamp (angle, dFloat (-dPi), dFloat (dPi));}

	CUSTOM_JOINTS_API dVector GetVelocity() const;
	CUSTOM_JOINTS_API void SetVelocity(const dVector& veloc);

	private:
	void ResolveCollision();
	void PreUpdate(dFloat timestep);
	dFloat PredictTimestep(dFloat timestep);
	int ResolveInterpenetrations(int contactCount, NewtonWorldConvexCastReturnInfo* const contacts);
	dVector CalculateImpulse(int rows, const dFloat* const rhs, const dFloat* const low, const dFloat* const high, const int* const normalIndex, const dComplementaritySolver::dJacobian* const jt) const;
	
	static unsigned dCustomPlayerController::PrefilterCallback(const NewtonBody* const body, const NewtonCollision* const collision, void* const userData);

	dMatrix m_localFrame;
	dVector m_impulse;
	dFloat m_mass;
	dFloat m_invMass;
	dFloat m_friction;
	dFloat m_headingAngle;
	dFloat m_forwardSpeed;
	dFloat m_lateralSpeed;
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

	virtual dFloat ProccessContact(dCustomPlayerController* const controller, const dVector& position, const dVector& normal, const NewtonBody* const otherbody) const {return controller->GetFriction();}

	virtual void PostUpdate(dFloat timestep) {}
	protected:
	CUSTOM_JOINTS_API virtual void PreUpdate(dFloat timestep, int threadID);

	dList<dCustomPlayerController> m_playerList;
};

#endif 

