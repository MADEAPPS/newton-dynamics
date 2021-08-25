/* Copyright (c) <2003-2021> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __D_JOINT_PLANE_H__
#define __D_JOINT_PLANE_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndJointPlane: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointPlane);
	D_NEWTON_API ndJointPlane(const dLoadSaveBase::dLoadDescriptor& desc);
	D_NEWTON_API ndJointPlane (const dVector& pivot, const dVector& normal, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointPlane();

	void EnableControlRotation(bool state);
	bool GetEnableControlRotation() const;

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const dLoadSaveBase::dSaveDescriptor& desc) const;
	
	bool m_enableControlRotation;
};

inline void ndJointPlane::EnableControlRotation(bool state)
{ 
	m_enableControlRotation = state; 
}

inline bool ndJointPlane::GetEnableControlRotation() const
{ 
	return m_enableControlRotation; 
}

#endif 

