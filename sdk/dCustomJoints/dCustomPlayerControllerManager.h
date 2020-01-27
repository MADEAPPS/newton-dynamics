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

#ifndef D_CUSTOM_PLAYER_CONTROLLER_MANAGER_H_
#define D_CUSTOM_PLAYER_CONTROLLER_MANAGER_H_

#include "dCustomJointLibraryStdAfx.h"
#include "dCustomListener.h"


#define PLAYER_PLUGIN_NAME				"__playerManager__"
#define PLAYER_CONTROLLER_MAX_CONTACTS	32
#define PLAYER_MIN_RESTRAINING_DISTANCE	1.0e-2f

class dCustomPlayerControllerManager;

class dPlayerController
{
	class dContactSolver;
	class dImpulseSolver;

	public:
	CUSTOM_JOINTS_API dPlayerController();
	CUSTOM_JOINTS_API ~dPlayerController();

	void* GetUserData () const {return m_userData;}
	NewtonBody* GetBody() {return m_kinematicBody;}
	void SetUserData(void* const userData) {m_userData = userData;}
	dCustomPlayerControllerManager* GetManager() const {return m_manager;}

	CUSTOM_JOINTS_API void ToggleCrouch ();

	bool IsCrouched () const {return m_isCrouched;}
	bool IsAirBorn () const {return m_isAirbone;}
	bool IsOnFloor () const {return m_isOnFloor;}
	const dFloat GetMass() const { return m_mass;}

	const dVector& GetImpulse() { return m_impulse; }
	void SetImpulse(const dVector& impulse) { m_impulse = impulse;}

	dFloat GetForwardSpeed() const { return -m_forwardSpeed; }
	void SetForwardSpeed(dFloat speed) {m_forwardSpeed = -speed; }

	dFloat GetLateralSpeed() const { return -m_lateralSpeed; }
	void SetLateralSpeed(dFloat speed) { m_lateralSpeed = -speed; }

	dFloat GetHeadingAngle() const { return m_headingAngle; }
	void SetHeadingAngle(dFloat angle) {m_headingAngle = dClamp (angle, dFloat (-dPi), dFloat (dPi));}

	dMatrix GetFrame() const { return m_localFrame; }
	CUSTOM_JOINTS_API void SetFrame(const dMatrix& frame);

	CUSTOM_JOINTS_API dVector GetVelocity() const;
	CUSTOM_JOINTS_API void SetVelocity(const dVector& veloc);

	private:
	enum dCollisionState
	{
		m_colliding,
		m_freeMovement,
		m_deepPenetration,
	};

	void PreUpdate(dFloat timestep);
	void UpdatePlayerStatus(dContactSolver& contactSolver);
	void ResolveStep(dFloat timestep, dContactSolver& contactSolver);
	void ResolveCollision(dContactSolver& contactSolver, dFloat timestep);
	dFloat PredictTimestep(dFloat timestep, dContactSolver& contactSolver);
	void ResolveInterpenetrations(dContactSolver& contactSolver, dImpulseSolver& impulseSolver);
	dCollisionState TestPredictCollision(const dContactSolver& contactSolver, const dVector& veloc) const;

	dMatrix m_localFrame;
	dVector m_impulse;
	dFloat m_mass;
	dFloat m_invMass;
	dFloat m_headingAngle;
	dFloat m_forwardSpeed;
	dFloat m_lateralSpeed;
	dFloat m_stepHeight;
	dFloat m_contactPatch;
	dFloat m_height;
	dFloat m_weistScale;
	dFloat m_crouchScale;
	void* m_userData;
	NewtonBody* m_kinematicBody;
	dCustomPlayerControllerManager* m_manager;
	bool m_isAirbone;
	bool m_isOnFloor;
	bool m_isCrouched;
	friend class dCustomPlayerControllerManager;
};

class dCustomPlayerControllerManager: public dCustomParallelListener
{
	public:
	CUSTOM_JOINTS_API dCustomPlayerControllerManager(NewtonWorld* const world);
	CUSTOM_JOINTS_API ~dCustomPlayerControllerManager();

	CUSTOM_JOINTS_API virtual dPlayerController* CreateController(const dMatrix& location, const dMatrix& localAxis, dFloat mass, dFloat radius, dFloat height, dFloat stepHeight);
	CUSTOM_JOINTS_API virtual void DestroyController(dPlayerController* const player);

	virtual void ApplyMove(dPlayerController* const controller, dFloat timestep) = 0;
	virtual bool ProccessContact(dPlayerController* const controller, const dVector& position, const dVector& normal, const NewtonBody* const otherbody) const { return true; }
	virtual dFloat ContactFriction(dPlayerController* const controller, const dVector& position, const dVector& normal, int contactId, const NewtonBody* const otherbody) const {return 2.0f;}

	protected:
	virtual void PostUpdate(dFloat timestep) {}
	CUSTOM_JOINTS_API virtual void PreUpdate(dFloat timestep, int threadID);

	private:
	dList<dPlayerController> m_playerList;
	friend class dPlayerController;
};

#endif 

