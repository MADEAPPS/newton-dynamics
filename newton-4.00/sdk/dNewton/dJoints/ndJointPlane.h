/* Copyright (c) <2003-2022> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __ND_JOINT_PLANE_H__
#define __ND_JOINT_PLANE_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

D_MSV_NEWTON_CLASS_ALIGN_32
class ndJointPlane: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointPlane, ndJointBilateralConstraint)
	
	D_NEWTON_API ndJointPlane();
	D_NEWTON_API ndJointPlane (const ndVector& pivot, const ndVector& normal, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointPlane();

	void EnableControlRotation(bool state);
	bool GetEnableControlRotation() const;

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	
	bool m_enableControlRotation;
} D_GCC_NEWTON_CLASS_ALIGN_32;

inline void ndJointPlane::EnableControlRotation(bool state)
{ 
	m_enableControlRotation = state; 
}

inline bool ndJointPlane::GetEnableControlRotation() const
{ 
	return m_enableControlRotation; 
}

#endif 

