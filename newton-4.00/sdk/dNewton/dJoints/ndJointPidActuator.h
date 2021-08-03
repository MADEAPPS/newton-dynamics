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

#ifndef __D_JOINT_BALLANDSOCKET_ACTUATOR_H__
#define __D_JOINT_BALLANDSOCKET_ACTUATOR_H__

#include "ndNewtonStdafx.h"
#include "ndJointBallAndSocket.h"

class ndJointPidActuator: public ndJointBallAndSocket
{
	public:
	D_CLASS_RELECTION(ndJointPidActuator);

	D_NEWTON_API ndJointPidActuator(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointPidActuator();

	//D_NEWTON_API dFloat32 GetMaxConeAngle() const;
	//D_NEWTON_API void SetConeLimit(dFloat32 maxConeAngle);
	//D_NEWTON_API void SetConeFriction(dFloat32 regularizer, dFloat32 viscousFriction);
	//
	//D_NEWTON_API void SetTwistLimits(dFloat32 minAngle, dFloat32 maxAngle);
	//D_NEWTON_API void GetTwistLimits(dFloat32& minAngle, dFloat32& maxAngle) const;
	//D_NEWTON_API void SetTwistFriction(dFloat32 regularizer, dFloat32 viscousFriction);

	private:
	//void SubmitTwistAngle(const dVector& pin, dFloat32 angle, ndConstraintDescritor& desc);
	//void SubmitAngularAxis(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc);
	//void SubmitAngularAxisCartisianApproximation(const dMatrix& matrix07, const dMatrix& matrix1, ndConstraintDescritor& desc);

	protected:
	//D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	//D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& debugCallback) const;

	//dFloat32 m_maxConeAngle;
	//dFloat32 m_coneFriction;
	//dFloat32 m_minTwistAngle;
	//dFloat32 m_maxTwistAngle;
	//dFloat32 m_twistFriction;
	//dFloat32 m_coneFrictionRegularizer;
	//dFloat32 m_twistFrictionRegularizer;
};

#endif 

