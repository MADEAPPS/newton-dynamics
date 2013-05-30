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


// CustomUniversal.h: interface for the CustomUniversal class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CUSTOMUNIVERSAL_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE_H)
#define AFX_CUSTOMUNIVERSAL_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE_H

#include "CustomJoint.h"

class CustomUniversal: public CustomJoint  
{
	public:
	//dAddRtti(CustomJoint);

	CustomUniversal(const dMatrix& pinAndPivotFrame, NewtonBody* child, NewtonBody* parent = NULL);
	virtual ~CustomUniversal();

	void EnableLimit_0(bool state);
	void EnableLimit_1(bool state);
	void SetLimis_0(dFloat minAngle, dFloat maxAngle);
	void SetLimis_1(dFloat minAngle, dFloat maxAngle);

	void EnableMotor_0(bool state);
	void EnableMotor_1(bool state);

	protected:
	virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	virtual void GetInfo (NewtonJointRecord* const info) const;

	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;

	bool m_limit_0_On;
	bool m_limit_1_On;
	bool m_angularMotor_0_On;
	bool m_angularMotor_1_On;

	dFloat m_minAngle_0;
	dFloat m_maxAngle_0;
	dFloat m_angularDamp_0;
	dFloat m_angularAccel_0;

	dFloat m_minAngle_1;
	dFloat m_maxAngle_1;
	dFloat m_angularDamp_1;
	dFloat m_angularAccel_1;
};

#endif // !defined(AFX_CUSTOMUNIVERSAL_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE_H)

