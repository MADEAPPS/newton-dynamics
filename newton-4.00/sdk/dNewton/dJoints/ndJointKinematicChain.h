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

#ifndef __D_JOINT_TWO_BODY_IK_H__
#define __D_JOINT_TWO_BODY_IK_H__

#include "ndNewtonStdafx.h"
#include "ndJointBallAndSocket.h"

class ndJointKinematicChain: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointKinematicChain);
	D_NEWTON_API ndJointKinematicChain(const dLoadSaveBase::dLoadDescriptor& desc);
	//D_NEWTON_API ndJointKinematicChain(const dMatrix& basisInGlobalSpace, const dVector& pivotInGlobalSpace, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API ndJointKinematicChain(const dMatrix& hipReference, const dMatrix& pinAndpivot, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointKinematicChain();

	D_NEWTON_API void SetTargetLocalMatrix(const dMatrix& matrix);
	D_NEWTON_API void SetTargetGlobalMatrix(const dMatrix& matrix);

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const dLoadSaveBase::dSaveDescriptor& desc) const;
	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& debugCallback) const;

	void SubmitLinearLimits(ndConstraintDescritor& desc);
	void SubmitAngularLimits(ndConstraintDescritor& desc);
	

	dMatrix m_localRotation;
	dMatrix m_coronalFrame;
	dMatrix m_coneRotation;
	dVector m_targetPosit;

	dFloat32 m_angle;
	dFloat32 m_minAngle;
	dFloat32 m_maxAngle;
	dFloat32 m_angularSpring;
	dFloat32 m_angularDamper;
	dFloat32 m_angularRegularizer;

	dFloat32 m_maxDist;
	dFloat32 m_linearSpring;
	dFloat32 m_linearDamper;
	dFloat32 m_linearRegularizer;
};

//inline const dMatrix& ndJointKinematicChain::GetReferenceMatrix() const
//{
//	return m_pivotFrame;
//}


#endif 

