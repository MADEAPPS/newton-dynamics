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

#ifndef __ND_JOINT_GEAR_H__
#define __ND_JOINT_GEAR_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndJointGear: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointGear);
	D_NEWTON_API ndJointGear(const ndLoadSaveBase::dLoadDescriptor& desc);
	D_NEWTON_API ndJointGear(ndFloat32 gearRatio,
		const ndVector& body0Pin, ndBodyKinematic* const body0,
		const ndVector& body1Pin, ndBodyKinematic* const body1);
	D_NEWTON_API virtual ~ndJointGear();

	ndFloat32 GetRatio() const;
	void SetRatio(ndFloat32 ratio);

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	ndFloat32 m_gearRatio;
};

inline ndFloat32 ndJointGear::GetRatio() const
{
	return m_gearRatio;
}

inline void ndJointGear::SetRatio(ndFloat32 ratio)
{
	m_gearRatio = ratio;
}


#endif 

