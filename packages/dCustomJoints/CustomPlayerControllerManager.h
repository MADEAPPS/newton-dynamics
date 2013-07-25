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

#ifndef D_CUSTOM_PLAYER_CONTROLLER_MANAGER_H_
#define D_CUSTOM_PLAYER_CONTROLLER_MANAGER_H_

#include "CustomJointLibraryStdAfx.h"
#include "CustomControllerManager.h"


#define PLAYER_PLUGIN_NAME				"playerManager"
#define PLAYER_CONTROLLER_MAX_CONTACTS	32

class CustomPlayerController
{
	CUSTOM_CONTROLLER_GLUE(CustomPlayerController);
	
	public:
	dFloat GetHight() const 
	{
		return m_height;
	}

	const dVector GetUpDir () const
	{
		return m_upVector;	
	}

	void SetClimbSlope (dFloat slopeInRadians)
	{
		m_maxSlope = dCos (dAbs(slopeInRadians));
	}

	dFloat GetClimbSlope () const
	{
		return dAcos (m_maxSlope);
	}

	void SetRestrainingDistance (dFloat distance)
	{
		m_restrainingDistance = dAbs (distance);
	}

	dFloat GetRestrainingDistance ()
	{
		return m_restrainingDistance;
	}


	const dVector& GetGroundPlane() const
	{
		return m_groundPlane;
	}

	NEWTON_API void SetPlayerOrigin (dFloat originHigh);
	NEWTON_API void SetPlayerVelocity (dFloat forwardSpeed, dFloat lateralSpeed, dFloat verticalSpeed, dFloat headingAngle, const dVector& gravity, dFloat timestep);


	protected:
	NEWTON_API dVector CalculateDesiredOmega (dFloat headingAngle, dFloat timestep) const;
	NEWTON_API dVector CalculateDesiredVelocity (dFloat forwardSpeed, dFloat lateralSpeed, dFloat verticalSpeed, const dVector& gravity, dFloat timestep) const;

	NEWTON_API void Init(dFloat mass, dFloat outerRadius, dFloat innerRadius, dFloat height, dFloat stepHigh, const dMatrix& localAxis);
	NEWTON_API void Cleanup();

	NEWTON_API virtual void PreUpdate(dFloat timestep, int threadIndex);
	NEWTON_API virtual void PostUpdate(dFloat timestep, int threadIndex);
	
	private:
	void UpdateGroundPlane (dMatrix& matrix, const dMatrix& castMatrix, const dVector& target, int threadIndex);
	dFloat CalculateContactKinematics(const dVector& veloc, const NewtonWorldConvexCastReturnInfo* const contact) const;

	dVector m_upVector;
	dVector m_frontVector;
	dVector m_groundPlane;
	dVector m_groundVelocity;
	dFloat m_outerRadio;
	dFloat m_innerRadio;
	dFloat m_height;
	dFloat m_stairStep;
	dFloat m_maxSlope;
	dFloat m_sphereCastOrigin;
	dFloat m_restrainingDistance;
	bool m_isJumping;
	NewtonCollision* m_castingShape;
	NewtonCollision* m_supportShape;
	NewtonCollision* m_upperBodyShape;

	friend class CustomPlayerControllerManager;
};

class CustomPlayerControllerManager: public CustomControllerManager<CustomPlayerController> 
{
	public:
	NEWTON_API CustomPlayerControllerManager(NewtonWorld* const world);
	NEWTON_API virtual ~CustomPlayerControllerManager();

	virtual void Debug () const {};

	NEWTON_API virtual CustomController* CreatePlayer (dFloat mass, dFloat outerRadius, dFloat innerRadius, dFloat height, dFloat stairStep, const dMatrix& localAxis);
	NEWTON_API virtual void ApplyPlayerMove (CustomPlayerController* const controller, dFloat timestep) = 0; 

	// the client application can overload this function to customizer contacts
	NEWTON_API virtual int ProcessContacts (const CustomPlayerController* const controller, NewtonWorldConvexCastReturnInfo* const contacts, int count) const; 

	protected:
	virtual void DestroyController (CustomController* const controller);
};


#endif 

