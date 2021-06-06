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

#ifndef __D_JOINT_BALLANDSOCKET_H__
#define __D_JOINT_BALLANDSOCKET_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndJointBallAndSocket: public ndJointBilateralConstraint
{
	public:
	ND_CLASS_RELECTION(ndJointBallAndSocket);

	D_NEWTON_API ndJointBallAndSocket(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointBallAndSocket();

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);

#if 0
	D_NEWTON_API ndJointBallAndSocket(const dMatrix& pinAndPivotFrame0, const dMatrix& pinAndPivotFrame1, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API void EnableTwist(bool state);
	D_NEWTON_API void SetTwistLimits(dFloat32 minAngle, dFloat32 maxAngle);
	D_NEWTON_API void GetTwistLimits(dFloat32& minAngle, dFloat32& maxAngle) const;

	D_NEWTON_API void EnableCone(bool state);
	D_NEWTON_API dFloat32 GetConeLimits() const;
	D_NEWTON_API void SetConeLimits(dFloat32 maxAngle);

	D_NEWTON_API void SetTwistFriction (dFloat32 frictionTorque);
	D_NEWTON_API dFloat32 GetTwistFriction (dFloat32 frictionTorque) const;

	D_NEWTON_API void SetConeFriction(dFloat32 frictionTorque);
	D_NEWTON_API dFloat32 GetConeFriction(dFloat32 frictionTorque) const;

	D_NEWTON_API virtual void Debug(dDebugDisplay* const debugDisplay) const;
	
	protected:
	D_NEWTON_API virtual void Deserialize (NewtonDeserializeCallback callback, void* const userData);
	D_NEWTON_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const; 
	D_NEWTON_API virtual void SubmitConstraints (dFloat32 timestep, dInt32 threadIndex);

	void SubmitTwistAngle(const dVector& pin, dFloat32 pitchAngle, dFloat32 timestep);
	void SubmitAngularAxis(const dMatrix& matrix0, const dMatrix& matrix1, dFloat32 timestep);
	void SubmitAngularAxisCartisianApproximation(const dMatrix& matrix0, const dMatrix& matrix1, dFloat32 timestep);

	dAngularIntegration m_twistAngle;
	dFloat32 m_minTwistAngle;
	dFloat32 m_maxTwistAngle;
	dFloat32 m_maxConeAngle;
	dFloat32 m_coneFriction;
	dFloat32 m_twistFriction;
#endif
};

#endif 

