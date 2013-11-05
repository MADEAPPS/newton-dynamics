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



// CustomUserBlankJoint.h: interface for the CustomUserBlank class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CUSTOM_USER_BLANK_H_
#define _CUSTOM_USER_BLANK_H_

#include "CustomJoint.h"

class CustomUserBlank: public CustomJoint  
{
	public:
	CUSTOM_JOINTS_API CustomUserBlank(unsigned int maxDOF, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~CustomUserBlank();

	protected:
	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void GetInfo (NewtonJointRecord* const info) const;
};

#endif 

