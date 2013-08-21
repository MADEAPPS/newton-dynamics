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


#define PLAYER_PLUGIN_NAME				"__playerManager__"
#define PLAYER_CONTROLLER_MAX_CONTACTS	32
#define PLAYER_MIN_RESTRAINING_DISTANCE	1.0e-2f

class CustomPlayerController: public CustomControllerBase
{
	public:
	NEWTON_API CustomPlayerController();
	NEWTON_API ~CustomPlayerController();

	NEWTON_API void Init(dFloat mass, dFloat outerRadius, dFloat innerRadius, dFloat height, dFloat stepHigh, const dMatrix& localAxis);

	dFloat GetHigh() const 
	{
		return m_height;
	}

	const dVector GetUpDir () const
	{
		return m_upVector;	
	}
	const dVector& GetGroundPlane() const
	{
		return m_groundPlane;
	}

	void SetRestrainingDistance (dFloat distance)
	{
		m_restrainingDistance = dMax (dAbs (distance), PLAYER_MIN_RESTRAINING_DISTANCE);
	}

	void SetClimbSlope (dFloat slopeInRadians)
	{
		m_maxSlope = dCos (dAbs(slopeInRadians));
	}

	virtual void PreUpdate(dFloat timestep, int threadIndex)
	{
	}

	NEWTON_API void SetPlayerOrigin (dFloat originHigh);

	NEWTON_API dVector CalculateDesiredOmega (dFloat headingAngle, dFloat timestep) const;
	NEWTON_API dVector CalculateDesiredVelocity (dFloat forwardSpeed, dFloat lateralSpeed, dFloat verticalSpeed, const dVector& gravity, dFloat timestep) const;
	
	NEWTON_API virtual void PostUpdate(dFloat timestep, int threadIndex);
	NEWTON_API void SetPlayerVelocity (dFloat forwardSpeed, dFloat lateralSpeed, dFloat verticalSpeed, dFloat headingAngle, const dVector& gravity, dFloat timestep);

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
};


class CustomPlayerControllerManager: public CustomControllerManager<CustomPlayerController>
{
	public:
	NEWTON_API CustomPlayerControllerManager(NewtonWorld* const world);
	NEWTON_API ~CustomPlayerControllerManager();

	virtual void PreUpdate(dFloat timestep)
	{
	}

	NEWTON_API virtual void ApplyPlayerMove (CustomPlayerController* const controller, dFloat timestep) = 0; 

	NEWTON_API virtual CustomPlayerController* CreatePlayer (dFloat mass, dFloat outerRadius, dFloat innerRadius, dFloat height, dFloat stairStep, const dMatrix& localAxis);
	NEWTON_API virtual int ProcessContacts (const CustomPlayerController* const controller, NewtonWorldConvexCastReturnInfo* const contacts, int count) const; 
};

#endif 

