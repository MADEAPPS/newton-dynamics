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



// CustomPlane3DOF.h: interface for the CustomPlane3DOF class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CUSTOM_PLANE_H_
#define _CUSTOM_PLANE_H_

#include "dCustomJoint.h"


class CustomPlane3DOF: public dCustomJoint  
{
	public:
	CUSTOM_JOINTS_API CustomPlane3DOF (const dVector& pivot, const dVector& normal, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~CustomPlane3DOF();

	protected:
	CUSTOM_JOINTS_API CustomPlane3DOF(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;

	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void GetInfo (NewtonJointRecord* const info) const;
	
	DECLARE_CUSTOM_JOINT(CustomPlane3DOF, dCustomJoint)
};


class CustomPlane5DOF : public dCustomJoint
{
	public:
	CUSTOM_JOINTS_API CustomPlane5DOF(const dVector& pivot, const dVector& normal, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~CustomPlane5DOF();

	protected:
	CUSTOM_JOINTS_API CustomPlane5DOF(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;

	CUSTOM_JOINTS_API virtual void SubmitConstraints(dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void GetInfo(NewtonJointRecord* const info) const;

	DECLARE_CUSTOM_JOINT(CustomPlane5DOF, dCustomJoint)
};


#endif 

