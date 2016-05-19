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


// CustomWormGear.h: interface for the CustomWormGear class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CustomWormGear_H_
#define _CustomWormGear_H_

#include "CustomJoint.h"

// this joint is for used in conjunction with Hinge of other spherical joints
// is is useful for establishing synchronization between the phase angle other the 
// relative angular velocity of two spinning disk according to the law of gears
// velErro = -(W0 * r0 + W1 * r1)
// where w0 and W1 are the angular velocity
// r0 and r1 are the radius of the spinning disk
class CustomRackAndPinion: public CustomJoint  
{
	public:
	CUSTOM_JOINTS_API CustomRackAndPinion(dFloat gearRatio, const dVector& rotationalPin, const dVector& linearPin, NewtonBody* rotationalBody, NewtonBody* linearBody);
	CUSTOM_JOINTS_API virtual ~CustomRackAndPinion();

	protected:
	CUSTOM_JOINTS_API CustomRackAndPinion (NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const; 

	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void GetInfo (NewtonJointRecord* const info) const;

	dFloat m_gearRatio;
	DECLARE_CUSTOM_JOINT(CustomRackAndPinion, CustomJoint)
};

#endif // !defined(AFX_CustomWormGear_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE_H)

