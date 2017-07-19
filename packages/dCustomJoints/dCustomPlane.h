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



// dCustomPlane3DOF.h: interface for the dCustomPlane3DOF class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CUSTOM_PLANE_H_
#define _CUSTOM_PLANE_H_

#include "dCustomJoint.h"


class dCustomPlane3DOF: public dCustomJoint  
{
	public:
	CUSTOM_JOINTS_API dCustomPlane3DOF (const dVector& pivot, const dVector& normal, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~dCustomPlane3DOF();

	protected:
	CUSTOM_JOINTS_API dCustomPlane3DOF(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;

	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	
	DECLARE_CUSTOM_JOINT(dCustomPlane3DOF, dCustomJoint)
};


class dCustomPlane5DOF : public dCustomJoint
{
	public:
	CUSTOM_JOINTS_API dCustomPlane5DOF(const dVector& pivot, const dVector& normal, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~dCustomPlane5DOF();

	protected:
	CUSTOM_JOINTS_API dCustomPlane5DOF(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;

	CUSTOM_JOINTS_API virtual void SubmitConstraints(dFloat timestep, int threadIndex);

	DECLARE_CUSTOM_JOINT(dCustomPlane5DOF, dCustomJoint)
};


#endif 

