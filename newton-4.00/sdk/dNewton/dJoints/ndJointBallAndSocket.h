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

#ifndef __ND_JOINT_BALLANDSOCKET_H__
#define __ND_JOINT_BALLANDSOCKET_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

#define D_BALL_AND_SOCKED_MAX_ANGLE	dFloat32 (120.0f * dDegreeToRad)
#define D_BALL_AND_SOCKED_PENETRATION_RECOVERY_SPEED dFloat32 (0.1f) 
#define D_BALL_AND_SOCKED_PENETRATION_LIMIT dFloat32 (10.0f * dDegreeToRad) 

class ndJointBallAndSocket: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointBallAndSocket);
	D_NEWTON_API ndJointBallAndSocket(const ndLoadSaveBase::dLoadDescriptor& desc);
	D_NEWTON_API ndJointBallAndSocket(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointBallAndSocket();

	D_NEWTON_API dFloat32 GetMaxConeAngle() const;
	D_NEWTON_API void SetConeLimit(dFloat32 maxConeAngle);
	D_NEWTON_API void SetConeFriction(dFloat32 regularizer, dFloat32 viscousFriction);

	D_NEWTON_API void SetTwistLimits(dFloat32 minAngle, dFloat32 maxAngle);
	D_NEWTON_API void GetTwistLimits(dFloat32& minAngle, dFloat32& maxAngle) const;
	D_NEWTON_API void SetTwistFriction(dFloat32 regularizer, dFloat32 viscousFriction);

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;
	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& debugCallback) const;

	void SubmitTwistAngle(const ndVector& pin, dFloat32 angle, ndConstraintDescritor& desc);
	void SubmitAngularAxis(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);
	void SubmitAngularAxisCartesianApproximation(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);

	void SubmitConeAngleOnlyRows(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);

	dFloat32 m_maxConeAngle;
	dFloat32 m_coneFriction;
	dFloat32 m_minTwistAngle;
	dFloat32 m_maxTwistAngle;
	dFloat32 m_twistFriction;
	dFloat32 m_coneFrictionRegularizer;
	dFloat32 m_twistFrictionRegularizer;
};

#endif 

