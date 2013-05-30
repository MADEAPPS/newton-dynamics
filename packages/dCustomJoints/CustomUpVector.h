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


// CustomUpVector.h: interface for the CustomUpVector class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CUSTOMUPVECTOR_H__EAE1E36C_6FDF_4D86_B4EE_855E3D1046F4_H)
#define AFX_CUSTOMUPVECTOR_H__EAE1E36C_6FDF_4D86_B4EE_855E3D1046F4_H


#include "CustomJoint.h"

// This joint is useful to for implementing character controllers, and also precise object picking
class CustomUpVector: public CustomJoint
{
	public:
	//dAddRtti(CustomJoint);

	CustomUpVector(const dVector& pin, NewtonBody* child);
	virtual ~CustomUpVector();

	void SetPinDir (const dVector& pin);

	protected:
	virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	virtual void GetInfo (NewtonJointRecord* const info) const;


	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;
};

#endif // !defined(AFX_CUSTOMUPVECTOR_H__EAE1E36C_6FDF_4D86_B4EE_855E3D1046F4_H)

