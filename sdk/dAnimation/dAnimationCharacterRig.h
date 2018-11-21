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


#ifndef __D_ANIMATION_CHARACTER_RIG_H__
#define __D_ANIMATION_CHARACTER_RIG_H__

#include "dAnimationStdAfx.h"


class dAnimationCharacterRig: public dCustomControllerBase
{
	public:
	dAnimationCharacterRig ();
//	dVehicleInterface* GetVehicle() {return m_vehicle;}
	
	void Finalize();
//	void ApplyExternalForces(dFloat timestep);

	protected:
	virtual void PreUpdate(dFloat timestep, int threadIndex);
	virtual void PostUpdate(dFloat timestep, int threadIndex);
	virtual void Debug(dCustomJoint::dDebugDisplay* const debugContext) const;
/*
	void Init(NewtonBody* const body, const dMatrix& localFrame, NewtonApplyForceAndTorque forceAndTorque, dFloat gravityMag);
	void Init(NewtonCollision* const chassisShape, dFloat mass, const dMatrix& localFrame, NewtonApplyForceAndTorque forceAndTorque, dFloat gravityMag);
	void Cleanup();
	
	void CalculateTireContacts(dFloat timestep);
	void CalculateSuspensionForces(dFloat timestep);

	static int OnAABBOverlap(const NewtonBody * const body, void* const me);
*/	

	friend class dAnimationCharacterRigManager;
};


#endif 

