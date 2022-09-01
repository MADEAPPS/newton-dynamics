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

#ifndef __ND_JOINT_SLIDER_H__
#define __ND_JOINT_SLIDER_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndJointSlider: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointSlider);
	D_NEWTON_API ndJointSlider(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API ndJointSlider(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API ndJointSlider(const ndMatrix& pinAndPivotInChild, const ndMatrix& pinAndPivotInParent, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointSlider();

	D_NEWTON_API ndFloat32 GetSpeed() const;
	D_NEWTON_API ndFloat32 GetPosit() const;
	D_NEWTON_API ndFloat32 GetOffsetPosit() const;
	D_NEWTON_API void SetOffsetPosit(ndFloat32 offset);
	D_NEWTON_API bool GetLimitState() const;
	D_NEWTON_API void SetLimitState(bool state);
	D_NEWTON_API void SetLimits(ndFloat32 minLimit, ndFloat32 maxLimit);
	D_NEWTON_API void GetLimits(ndFloat32& minLimit, ndFloat32& maxLimit) const;
	D_NEWTON_API virtual void SetAsSpringDamper(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper);
	D_NEWTON_API void GetSpringDamper(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const;

	protected:
	D_NEWTON_API ndFloat32 PenetrationSpeed(ndFloat32 penetration) const;
	D_NEWTON_API void SubmitLimits(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);
	D_NEWTON_API void SubmitSpringDamper(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);
	
	D_NEWTON_API void ApplyBaseRows(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	ndFloat32 m_posit;
	ndFloat32 m_speed;
	ndFloat32 m_springK;
	ndFloat32 m_damperC;
	ndFloat32 m_minLimit;
	ndFloat32 m_maxLimit;
	ndFloat32 m_positOffset;
	ndFloat32 m_springDamperRegularizer;
	ndInt8 m_limitState;
};

#endif 

