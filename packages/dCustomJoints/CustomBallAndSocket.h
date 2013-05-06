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


// CustomBallAndSocket.h: interface for the CustomBallAndSocket class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CUSTOMBALLANDSOCKET_H_
#define _CUSTOMBALLANDSOCKET_H_

#include "CustomJoint.h"
class CustomBallAndSocket: public CustomJoint  
{
	public:
	dAddRtti(CustomJoint);
	CustomBallAndSocket(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent = NULL);
	virtual ~CustomBallAndSocket();

	protected:
	virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	virtual void GetInfo (NewtonJointRecord* const info) const;

	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;
};


// similar to the ball and socked 
// plus it has the ability to set joint limits
class CustomLimitBallAndSocket: public CustomBallAndSocket  
{
	public:
	dAddRtti(CustomJoint);
	CustomLimitBallAndSocket(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent = NULL);
	virtual ~CustomLimitBallAndSocket();

//	void SetChildOrientation (const dMatrix& matrix);

	void SetConeAngle (dFloat angle);
	void SetTwistAngle (dFloat minAngle, dFloat maxAngle);

	protected:
	virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	virtual void GetInfo (NewtonJointRecord* const info) const;

	dFloat m_minTwistAngle;
	dFloat m_maxTwistAngle;

	dFloat m_coneAngleCos;
	dFloat m_coneAngleSin;
	dFloat m_coneAngleHalfCos;
	dFloat m_coneAngleHalfSin;
};





#endif // !defined(AFX_CUSTOMBALLANDSOCKET_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE_H)

