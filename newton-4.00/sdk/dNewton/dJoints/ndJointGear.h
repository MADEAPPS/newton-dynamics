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

#ifndef __D_JOINT_GEAR_H__
#define __D_JOINT_GEAR_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndJointGear: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointGear);
	D_COLLISION_API ndJointGear(const dLoadSaveBase::dLoadDescriptor& desc);
	D_NEWTON_API ndJointGear(dFloat32 gearRatio,
		const dVector& body0Pin, ndBodyKinematic* const body0,
		const dVector& body1Pin, ndBodyKinematic* const body1);
	D_NEWTON_API virtual ~ndJointGear();

	dFloat32 GetRatio() const;
	void SetRatio(dFloat32 ratio);

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const dLoadSaveBase::dSaveDescriptor& desc) const;

	dFloat32 m_gearRatio;
};

inline dFloat32 ndJointGear::GetRatio() const
{
	return m_gearRatio;
}

inline void ndJointGear::SetRatio(dFloat32 ratio)
{
	m_gearRatio = ratio;
}


#endif 

