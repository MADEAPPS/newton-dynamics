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

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndIkJointHinge.h"

ndIkJointHinge::ndIkJointHinge()
	:ndJointHinge()
	,ndJointBilateralConstraint::ndIkInterface()
	,m_maxTorque(D_IK_HINGE_MAX_TORQUE)
{
}

ndIkJointHinge::ndIkJointHinge(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointHinge(pinAndPivotFrame, child, parent)
	,ndJointBilateralConstraint::ndIkInterface()
	,m_maxTorque (D_IK_HINGE_MAX_TORQUE)
{
}

ndIkJointHinge::ndIkJointHinge(const ndMatrix& pinAndPivotInChild, const ndMatrix& pinAndPivotInParent, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointHinge(pinAndPivotInChild, pinAndPivotInParent, child, parent)
	,ndJointBilateralConstraint::ndIkInterface()
	,m_maxTorque(D_IK_HINGE_MAX_TORQUE)
{
}

ndIkJointHinge::~ndIkJointHinge()
{
}

ndFloat32 ndIkJointHinge::GetMaxTorque() const
{
	return m_maxTorque;
}

void ndIkJointHinge::SetMaxTorque(ndFloat32 maxTorque)
{
	m_maxTorque = ndAbs(maxTorque);
}

bool ndIkJointHinge::IsHolonomic(ndFloat32 timestep) const
{
	if (m_limitState)
	{
		if ((m_minLimit > (ndFloat32(-1.0f) * ndDegreeToRad)) && (m_maxLimit < (ndFloat32(1.0f) * ndDegreeToRad)))
		{
			ndAssert(0);
			return false;
		}
		else
		{
			const ndFloat32 angle = m_angle + m_omega * timestep;
			if (angle < m_minLimit)
			{
				return false;
			}
			else if (angle > m_maxLimit)
			{
				return false;
			}
		}
	}
	return true;
}

void ndIkJointHinge::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	ApplyBaseRows(desc, matrix0, matrix1);
	if (!m_ikMode)
	{
		const ndVector pin(matrix0.m_front);
		ndFloat32 accel = (pin * m_accel0.m_angular - pin * m_accel1.m_angular).AddHorizontal().GetScalar();
		AddAngularRowJacobian(desc, pin, 0.0f);
		SetMotorAcceleration(desc, accel);
		SetDiagonalRegularizer(desc, m_defualtRegularizer);
		if (m_maxTorque < D_IK_HINGE_MAX_TORQUE)
		{
			SetHighFriction(desc, m_maxTorque);
			SetLowerFriction(desc, -m_maxTorque);
		}
	}
	SubmitLimits(desc, matrix0, matrix1);
}


