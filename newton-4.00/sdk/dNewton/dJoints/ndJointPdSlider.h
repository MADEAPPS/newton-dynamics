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

#ifndef __ND_JOINT_PD_SLIDER_H__
#define __ND_JOINT_PD_SLIDER_H__

#include "ndNewtonStdafx.h"
#include "ndJointSlider.h"

class ndJointPdSlider: public ndJointSlider
{
	public:
	D_CLASS_REFLECTION(ndJointPdSlider);
	D_NEWTON_API ndJointPdSlider(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API ndJointPdSlider(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointPdSlider();

	D_NEWTON_API ndFloat32 GetTarget() const;
	D_NEWTON_API void SetTarget(ndFloat32 target);

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	ndFloat32 n_targetPosit;
};

#endif 

