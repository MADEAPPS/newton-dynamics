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

#ifndef __ND_JOINT_SPHERICAL_H__
#define __ND_JOINT_SPHERICAL_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

#define D_MAX_SPHERICAL_RECOVERY_SPEED	ndFloat32 (0.25f)
#define D_MAX_SPHERICAL_PENETRATION		(ndFloat32 (4.0f) * ndDegreeToRad)
#define D_BALL_AND_SOCKED_MAX_ANGLE		(ndFloat32 (150.0f) * ndDegreeToRad)

class ndJointSpherical: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointSpherical);
	D_NEWTON_API ndJointSpherical(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API ndJointSpherical(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointSpherical();

	D_NEWTON_API virtual ndFloat32 GetConeLimit() const;
	D_NEWTON_API virtual void SetConeLimit(ndFloat32 maxConeAngle);
	D_NEWTON_API virtual void SetTwistLimits(ndFloat32 minAngle, ndFloat32 maxAngle);
	D_NEWTON_API virtual void GetTwistLimits(ndFloat32& minAngle, ndFloat32& maxAngle) const;
	D_NEWTON_API virtual void SetViscousFriction(ndFloat32 regularizer, ndFloat32 viscousFriction);

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;
	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& debugCallback) const;

	ndFloat32 PenetrationOmega(ndFloat32 penetartion) const;

	void SubmitFriction(ndConstraintDescritor& desc);
	void ApplyBaseRows(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);

	ndInt8 SubmitAngularAxis(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);
	ndInt8 SubmitTwistAngle(const ndVector& pin, ndFloat32 angle, ndConstraintDescritor& desc);
	ndInt8 SubmitLimits(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);
	ndInt8 SubmitAngularAxisCartesianApproximation(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);
	
	ndFloat32 m_maxConeAngle;
	ndFloat32 m_minTwistAngle;
	ndFloat32 m_maxTwistAngle;
	ndFloat32 m_viscousFriction;
	ndFloat32 m_viscousFrictionRegularizer;
};

#endif 

