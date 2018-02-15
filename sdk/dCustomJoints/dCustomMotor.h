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


// dCustomMotor.h: interface for the dCustomMotor class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CustomMotor_H__
#define _CustomMotor_H__

#include "dCustomJoint.h"

class dCustomMotor: public dCustomJoint
{
	public:
	CUSTOM_JOINTS_API dCustomMotor(int dof, NewtonBody* const child, NewtonBody* const parent);
	CUSTOM_JOINTS_API dCustomMotor(dFloat gearRatio, const dVector& childPin, const dVector& parentPin, NewtonBody* const child, NewtonBody* const parent);
	CUSTOM_JOINTS_API virtual ~dCustomMotor();

	protected:
	CUSTOM_JOINTS_API virtual void Deserialize (NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const; 
	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);

	dFloat m_motorSpeed;
	dFloat m_motorTorque;
	DECLARE_CUSTOM_JOINT(dCustomMotor, dCustomJoint)
};


#endif 

