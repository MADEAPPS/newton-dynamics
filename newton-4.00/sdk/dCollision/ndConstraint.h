/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __ND_CONSTRAINT_H__
#define __ND_CONSTRAINT_H__

#include "ndCollisionStdafx.h"

#define D_MAX_BOUND				D_LCP_MAX_VALUE
#define D_MIN_BOUND				(-D_LCP_MAX_VALUE)
#define D_INDEPENDENT_ROW		-1 
#define D_CONSTRAINT_MAX_ROWS	(3 * 16)
#define MIN_JOINT_PIN_LENGTH	dFloat32 (50.0f)

class ndBody;
class ndContact;
class ndConstraint;
class ndLeftHandSide;
class ndRightHandSide;
class ndBodyKinematic;
class ndJointBilateralConstraint;

D_MSV_NEWTON_ALIGN_32
class ndConstraintDebugCallback: public ndClassAlloc
{
	public:
	ndConstraintDebugCallback()
	{
		m_debugScale = dFloat32(1.0f);
	}

	virtual ~ndConstraintDebugCallback() 
	{
	}

	virtual void DrawLine(const ndVector& p0, const ndVector& p1, const ndVector& color, dFloat32 thickness = dFloat32 (1.0f)) = 0;

	virtual void SetScale(dFloat32 scale)
	{
		m_debugScale = scale;
	}

	virtual dFloat32 GetScale() const
	{
		return m_debugScale;
	}

	virtual void DrawFrame(const ndMatrix& matrix)
	{
		ndVector x(matrix.m_posit + matrix.RotateVector(ndVector(m_debugScale, dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f))));
		DrawLine(matrix.m_posit, x, ndVector(dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f)));

		ndVector y(matrix.m_posit + matrix.RotateVector(ndVector(dFloat32(0.0f), m_debugScale, dFloat32(0.0f), dFloat32(0.0f))));
		DrawLine(matrix.m_posit, y, ndVector(dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f)));

		ndVector z(matrix.m_posit + matrix.RotateVector(ndVector(dFloat32(0.0f), dFloat32(0.0f), m_debugScale, dFloat32(0.0f))));
		DrawLine(matrix.m_posit, z, ndVector(dFloat32(0.0f), dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f)));
	}

	virtual void DrawArrow(const ndMatrix& matrix, const ndVector& color, dFloat32 length)
	{
		ndVector p1(matrix.m_posit + matrix.RotateVector(ndVector(m_debugScale * length, dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f))));
		DrawLine(matrix.m_posit, p1, color);
	}
		
	dFloat32 m_debugScale;
} D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class dgPointParam
{
	public:
	ndVector m_r0;
	ndVector m_r1;
	ndVector m_posit0;
	ndVector m_posit1;
} D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class ndJacobian
{
	public:
	ndVector m_linear;
	ndVector m_angular;
} D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class ndJacobianPair
{
	public:
	ndJacobian m_jacobianM0;
	ndJacobian m_jacobianM1;
} D_GCC_NEWTON_ALIGN_32;

class ndForceImpactPair
{
	public:
	void Clear()
	{
		m_force = dFloat32(dFloat32(0.0f));
		m_impact = dFloat32(dFloat32(0.0f));
		for (dInt32 i = 0; i < dInt32(sizeof(m_initialGuess) / sizeof(m_initialGuess[0])); i++)
		{
			m_initialGuess[i] = dFloat32(dFloat32(0.0f));
		}
	}

	void Push(dFloat32 val)
	{
		for (dInt32 i = 1; i < dInt32(sizeof(m_initialGuess) / sizeof(m_initialGuess[0])); i++)
		{
			m_initialGuess[i - 1] = m_initialGuess[i];
		}
		m_initialGuess[sizeof(m_initialGuess) / sizeof(m_initialGuess[0]) - 1] = val;
	}

	dFloat32 GetInitialGuess() const
	{
		//return 100.0f;
		dFloat32 smallest = dFloat32(1.0e15f);
		dFloat32 value = dFloat32(dFloat32(0.0f));
		for (dInt32 i = 0; i < dInt32(sizeof(m_initialGuess) / sizeof(m_initialGuess[0])); i++)
		{
			dFloat32 mag = dAbs(m_initialGuess[i]);
			if (mag < smallest) 
			{
				smallest = mag;
				value = m_initialGuess[i];
			}
		}
		return value;
	}

	dFloat32 m_force;
	dFloat32 m_impact;
	dFloat32 m_initialGuess[4];
};

class ndJointAccelerationDecriptor
{
	public:
	dInt32 m_rowsCount;
	dFloat32 m_timestep;
	dFloat32 m_invTimestep;
	dFloat32 m_firstPassCoefFlag;
	ndRightHandSide* m_rightHandSide;
	const ndLeftHandSide* m_leftHandSide;
};

class ndBilateralBounds
{
	public:
	ndForceImpactPair* m_jointForce;
	dFloat32 m_low;
	dFloat32 m_upper;
	dInt32 m_normalIndex;
};

D_MSV_NEWTON_ALIGN_32
class ndConstraintDescritor
{
	public:
	ndJacobianPair m_jacobian[D_CONSTRAINT_MAX_ROWS];
	ndBilateralBounds m_forceBounds[D_CONSTRAINT_MAX_ROWS];
	dFloat32 m_jointAccel[D_CONSTRAINT_MAX_ROWS];
	dFloat32 m_restitution[D_CONSTRAINT_MAX_ROWS];
	dFloat32 m_penetration[D_CONSTRAINT_MAX_ROWS];
	dFloat32 m_diagonalRegularizer[D_CONSTRAINT_MAX_ROWS];
	dFloat32 m_penetrationStiffness[D_CONSTRAINT_MAX_ROWS];
	dFloat32 m_zeroRowAcceleration[D_CONSTRAINT_MAX_ROWS];
	dInt32 m_flags[D_CONSTRAINT_MAX_ROWS];
	dFloat32 m_timestep;
	dFloat32 m_invTimestep;
	dInt32 m_rowsCount;
} D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class ndLeftHandSide
{
	public:
	ndJacobianPair m_Jt;
	ndJacobianPair m_JMinv;
} D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class ndRightHandSide
{
	public:
	dFloat32 m_force;
	dFloat32 m_diagDamp;
	dFloat32 m_invJinvMJt;
	dFloat32 m_coordenateAccel;

	dFloat32 m_lowerBoundFrictionCoefficent;
	dFloat32 m_upperBoundFrictionCoefficent;
	dFloat32 m_deltaAccel;
	dFloat32 m_restitution;

	dFloat32 m_maxImpact;
	dFloat32 m_penetration;
	dFloat32 m_diagonalRegularizer;
	dFloat32 m_penetrationStiffness;

	ndForceImpactPair* m_jointFeebackForce;
	dInt32 m_normalForceIndex;
	dInt32 m_normalForceIndexFlat;
} D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class ndConstraint
{
	public:
	// add some reflexion to the classes
	D_CLASS_REFLECTION(ndConstraint);

	virtual ~ndConstraint();

	virtual ndContact* GetAsContact();
	virtual ndJointBilateralConstraint* GetAsBilateral();

	bool IsActive() const;
	void SetActive(bool state);
	virtual bool IsBilateral() const;

	virtual dUnsigned32 GetRowsCount() const = 0;
	virtual ndBodyKinematic* GetBody0() const;
	virtual ndBodyKinematic* GetBody1() const;
	virtual void JacobianDerivative(ndConstraintDescritor& desc) = 0;
	virtual void JointAccelerations(ndJointAccelerationDecriptor* const desc) = 0;
	
	virtual void DebugJoint(ndConstraintDebugCallback&) const;
	void InitPointParam(dgPointParam& param, const ndVector& p0Global, const ndVector& p1Global) const;

	dFloat32 m_preconditioner0;
	dFloat32 m_preconditioner1;
	dInt32 m_rowCount;
	dInt32 m_rowStart;
	dUnsigned8 m_active;
	dUnsigned8 m_fence0;
	dUnsigned8 m_fence1;
	dUnsigned8 m_resting;   // this should be idnetial to m_fence0, should be removed. 
	dUnsigned8 m_isInSkeletonLoop;
	
	protected:
	ndConstraint();
} D_GCC_NEWTON_ALIGN_32 ;

inline ndConstraint::~ndConstraint()
{
}

inline ndContact* ndConstraint::GetAsContact()
{ 
	return nullptr; 
}

inline ndJointBilateralConstraint* ndConstraint::GetAsBilateral()
{ 
	return nullptr; 
}

inline bool ndConstraint::IsActive() const
{ 
	return m_active ? true : false;
}

inline void ndConstraint::SetActive(bool state)
{ 
	m_active = state ? 1 : 0;
}

inline bool ndConstraint::IsBilateral() const
{ 
	return false; 
}

inline ndBodyKinematic* ndConstraint::GetBody0() const
{ 
	return nullptr; 
}

inline ndBodyKinematic* ndConstraint::GetBody1() const
{ 
	return nullptr; 
}

inline void ndConstraint::DebugJoint(ndConstraintDebugCallback&) const
{
}

#endif 

