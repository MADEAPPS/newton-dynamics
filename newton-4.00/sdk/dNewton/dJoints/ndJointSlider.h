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

#ifndef __ND_JOINT_SLIDER_H__
#define __ND_JOINT_SLIDER_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

#define D_SLIDER_PENETRATION_LIMIT			ndFloat32 (0.2f) 
#define D_SLIDER_PENETRATION_RECOVERY_SPEED ndFloat32 (0.1f) 

class ndJointSlider: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointSlider);
	D_NEWTON_API ndJointSlider(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API ndJointSlider(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointSlider();

	D_NEWTON_API virtual ndFloat32 GetSpeed() const;
	D_NEWTON_API virtual ndFloat32 GetPosit() const;
	D_NEWTON_API virtual ndFloat32 GetFriction() const;

	D_NEWTON_API virtual void SetFriction(ndFloat32 friction);
	D_NEWTON_API virtual void EnableLimits(bool state, ndFloat32 minLimit, ndFloat32 maxLimit);
	D_NEWTON_API virtual void SetAsSpringDamper(bool state, ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper);

	private:
	void SubmitConstraintLimits(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);
	void SubmitConstraintLimitSpringDamper(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	ndFloat32 m_posit;
	ndFloat32 m_speed;
	ndFloat32 m_springK;
	ndFloat32 m_damperC;
	ndFloat32 m_minLimit;
	ndFloat32 m_maxLimit;
	ndFloat32 m_friction;
	ndFloat32 m_springDamperRegularizer;

	bool m_hasLimits;
	bool m_isSpringDamper;
};

#endif 

