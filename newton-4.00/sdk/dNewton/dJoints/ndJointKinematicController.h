/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

// ndJointKinematicController.h: interface for the ndJointKinematicController class.
//
//////////////////////////////////////////////////////////////////////

#ifndef __D_JOINT_KINEMATIC_CONTROLLER_H__
#define __D_JOINT_KINEMATIC_CONTROLLER_H__


#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndJointKinematicController: public ndJointBilateralConstraint
{
	public:
	enum dControlModes
	{	
		m_linear,
		m_full6dof,
		m_linearAndTwist,
		m_linearAndCone,
		m_linearPlusAngularFriction, // this is pick mode from screen
	};
#if 0
	D_NEWTON_API ndJointKinematicController(ndBodyKinematic* const body, const dVector& attachmentPointInGlobalSpace, ndBodyKinematic* const referenceBody = nullptr);
	D_NEWTON_API ndJointKinematicController(ndBodyKinematic* const body, const dMatrix& attachmentMatrixInGlobalSpace, ndBodyKinematic* const referenceBody = nullptr);
	D_NEWTON_API virtual ~ndJointKinematicController();

	D_NEWTON_API dControlModes GetControlMode() const;
	D_NEWTON_API void SetControlMode(dControlModes mode);

	D_NEWTON_API void SetMaxLinearFriction(dFloat force); 
	D_NEWTON_API void SetMaxAngularFriction(dFloat torque); 
	D_NEWTON_API void SetAngularViscuosFrictionCoefficient(dFloat coefficient);
	D_NEWTON_API void SetMaxSpeed(dFloat speedInMetersPerSeconds); 
	D_NEWTON_API void SetMaxOmega(dFloat speedInRadiansPerSeconds); 

	D_NEWTON_API void SetTargetPosit (const dVector& posit); 
	D_NEWTON_API void SetTargetRotation (const dQuaternion& rotation); 
	D_NEWTON_API void SetTargetMatrix (const dMatrix& matrix); 

	D_NEWTON_API void ResetAutoSleep();

	D_NEWTON_API dMatrix GetBodyMatrix () const;
	D_NEWTON_API dMatrix GetTargetMatrix () const;
	D_NEWTON_API virtual void Debug(dDebugDisplay* const debugDisplay) const;

	protected:
	D_NEWTON_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	D_NEWTON_API virtual void Deserialize (NewtonDeserializeCallback callback, void* const userData); 
	D_NEWTON_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const;

	D_NEWTON_API void SubmitLinearConstraints (const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep);

	void CheckSleep() const;
	void Init(const dMatrix& matrix);
	
	dFloat m_maxLinearFriction;
	dFloat m_maxAngularFriction;
	dFloat m_angularFrictionCoefficient;
	dFloat m_maxSpeed;
	dFloat m_maxOmega;
	dControlModes m_controlMode;

	DECLARE_CUSTOM_JOINT(ndJointKinematicController, dCustomJoint)
#endif

	D_NEWTON_API ndJointKinematicController(ndBodyKinematic* const referenceBody, ndBodyKinematic* const body, const dVector& attachmentPointInGlobalSpace);
	D_NEWTON_API ndJointKinematicController(ndBodyKinematic* const referenceBody, ndBodyKinematic* const body, const dMatrix& attachmentMatrixInGlobalSpace);
	D_NEWTON_API virtual ~ndJointKinematicController();

	protected:
	void Init(const dMatrix& matrix);

	bool m_autoSleepState;

};

#endif 

