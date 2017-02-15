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


// CustomGear.h: interface for the CustomGear class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CustomGear_H__
#define _CustomGear_H__

#include "CustomJoint.h"

// this joint is for used in conjunction with Hinge of other spherical joints
// is is usefully for establishing synchronization between the phase angle other the 
// relative angular velocity of two spinning disk according to the law of gears
// velErro = -(W0 * r0 + W1 *  r1)
// where w0 and W1 are the angular velocity
// r0 and r1 are the radius of the spinning disk
class CustomGear: public CustomJoint  
{
	public:
	CUSTOM_JOINTS_API CustomGear(int dof, NewtonBody* const child, NewtonBody* const parent);
	CUSTOM_JOINTS_API CustomGear(dFloat gearRatio, const dVector& childPin, const dVector& parentPin, NewtonBody* const child, NewtonBody* const parent);
	CUSTOM_JOINTS_API virtual ~CustomGear();

	protected:
	CUSTOM_JOINTS_API CustomGear (NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const; 

	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void GetInfo (NewtonJointRecord* const info) const;

	dFloat m_gearRatio;
	DECLARE_CUSTOM_JOINT(CustomGear, CustomJoint)
};

class CustomGearAndSlide: public CustomGear
{
	public:
	CUSTOM_JOINTS_API CustomGearAndSlide (dFloat gearRatio, dFloat slideRatio, const dVector& childPin, const dVector& parentPin, NewtonBody* const parenPin, NewtonBody* const parent);
	CUSTOM_JOINTS_API virtual ~CustomGearAndSlide();

	protected:
	CUSTOM_JOINTS_API CustomGearAndSlide (NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const; 

	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void GetInfo (NewtonJointRecord* const info) const;

	dFloat m_slideRatio;
	DECLARE_CUSTOM_JOINT(CustomGearAndSlide, CustomGear)
};


class CustomDifferentialGear: public CustomGear
{
	public:
	CUSTOM_JOINTS_API CustomDifferentialGear(dFloat gearRatio, const dVector& childPin, const dVector& parentPin, const dVector& referencePin, NewtonBody* const child, NewtonBody* const parent, NewtonBody* const parentReference);

	protected:
	CUSTOM_JOINTS_API CustomDifferentialGear (NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const; 
	CUSTOM_JOINTS_API virtual void SubmitConstraints(dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void GetInfo (NewtonJointRecord* const info) const;

	DECLARE_CUSTOM_JOINT(CustomDifferentialGear, CustomGear)

	dVector m_pintOnReference;
	NewtonBody* m_parentReference;
};




#endif 

