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
#define MIN_JOINT_PIN_LENGTH	ndFloat32 (50.0f)

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
		m_debugScale = ndFloat32(1.0f);
	}

	virtual ~ndConstraintDebugCallback() 
	{
	}

	virtual void DrawLine(const ndVector& p0, const ndVector& p1, const ndVector& color, ndFloat32 thickness = ndFloat32 (1.0f)) = 0;

	virtual void SetScale(ndFloat32 scale)
	{
		m_debugScale = scale;
	}

	virtual ndFloat32 GetScale() const
	{
		return m_debugScale;
	}

	virtual void DrawFrame(const ndMatrix& matrix)
	{
		ndVector x(matrix.m_posit + matrix.RotateVector(ndVector(m_debugScale, ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f))));
		DrawLine(matrix.m_posit, x, ndVector(ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f)));

		ndVector y(matrix.m_posit + matrix.RotateVector(ndVector(ndFloat32(0.0f), m_debugScale, ndFloat32(0.0f), ndFloat32(0.0f))));
		DrawLine(matrix.m_posit, y, ndVector(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f)));

		ndVector z(matrix.m_posit + matrix.RotateVector(ndVector(ndFloat32(0.0f), ndFloat32(0.0f), m_debugScale, ndFloat32(0.0f))));
		DrawLine(matrix.m_posit, z, ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f)));
	}

	virtual void DrawArrow(const ndMatrix& matrix, const ndVector& color, ndFloat32 length)
	{
		ndVector p1(matrix.m_posit + matrix.RotateVector(ndVector(m_debugScale * length, ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f))));
		DrawLine(matrix.m_posit, p1, color);
	}
		
	ndFloat32 m_debugScale;
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
		m_force = ndFloat32(ndFloat32(0.0f));
		m_impact = ndFloat32(ndFloat32(0.0f));
		for (ndInt32 i = 0; i < ndInt32(sizeof(m_initialGuess) / sizeof(m_initialGuess[0])); i++)
		{
			m_initialGuess[i] = ndFloat32(ndFloat32(0.0f));
		}
	}

	void Push(ndFloat32 val)
	{
		for (ndInt32 i = 1; i < ndInt32(sizeof(m_initialGuess) / sizeof(m_initialGuess[0])); i++)
		{
			m_initialGuess[i - 1] = m_initialGuess[i];
		}
		m_initialGuess[sizeof(m_initialGuess) / sizeof(m_initialGuess[0]) - 1] = val;
	}

	ndFloat32 GetInitialGuess() const
	{
		//return 100.0f;
		ndFloat32 smallest = ndFloat32(1.0e15f);
		ndFloat32 value = ndFloat32(ndFloat32(0.0f));
		for (ndInt32 i = 0; i < ndInt32(sizeof(m_initialGuess) / sizeof(m_initialGuess[0])); i++)
		{
			ndFloat32 mag = dAbs(m_initialGuess[i]);
			if (mag < smallest) 
			{
				smallest = mag;
				value = m_initialGuess[i];
			}
		}
		return value;
	}

	ndFloat32 m_force;
	ndFloat32 m_impact;
	ndFloat32 m_initialGuess[4];
};

class ndJointAccelerationDecriptor
{
	public:
	ndInt32 m_rowsCount;
	ndFloat32 m_timestep;
	ndFloat32 m_invTimestep;
	ndFloat32 m_firstPassCoefFlag;
	ndRightHandSide* m_rightHandSide;
	const ndLeftHandSide* m_leftHandSide;
};

class ndBilateralBounds
{
	public:
	ndForceImpactPair* m_jointForce;
	ndFloat32 m_low;
	ndFloat32 m_upper;
	ndInt32 m_normalIndex;
};

D_MSV_NEWTON_ALIGN_32
class ndConstraintDescritor
{
	public:
	ndJacobianPair m_jacobian[D_CONSTRAINT_MAX_ROWS];
	ndBilateralBounds m_forceBounds[D_CONSTRAINT_MAX_ROWS];
	ndFloat32 m_jointAccel[D_CONSTRAINT_MAX_ROWS];
	ndFloat32 m_restitution[D_CONSTRAINT_MAX_ROWS];
	ndFloat32 m_penetration[D_CONSTRAINT_MAX_ROWS];
	ndFloat32 m_diagonalRegularizer[D_CONSTRAINT_MAX_ROWS];
	ndFloat32 m_penetrationStiffness[D_CONSTRAINT_MAX_ROWS];
	ndFloat32 m_zeroRowAcceleration[D_CONSTRAINT_MAX_ROWS];
	ndInt32 m_flags[D_CONSTRAINT_MAX_ROWS];
	ndFloat32 m_timestep;
	ndFloat32 m_invTimestep;
	ndInt32 m_rowsCount;
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
	ndFloat32 m_force;
	ndFloat32 m_diagDamp;
	ndFloat32 m_invJinvMJt;
	ndFloat32 m_coordenateAccel;

	ndFloat32 m_lowerBoundFrictionCoefficent;
	ndFloat32 m_upperBoundFrictionCoefficent;
	ndFloat32 m_deltaAccel;
	ndFloat32 m_restitution;

	ndFloat32 m_maxImpact;
	ndFloat32 m_penetration;
	ndFloat32 m_diagonalRegularizer;
	ndFloat32 m_penetrationStiffness;

	ndForceImpactPair* m_jointFeebackForce;
	ndInt32 m_normalForceIndex;
	ndInt32 m_normalForceIndexFlat;
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

	virtual ndUnsigned32 GetRowsCount() const = 0;
	virtual ndBodyKinematic* GetBody0() const;
	virtual ndBodyKinematic* GetBody1() const;
	virtual void JacobianDerivative(ndConstraintDescritor& desc) = 0;
	virtual void JointAccelerations(ndJointAccelerationDecriptor* const desc) = 0;
	
	virtual void DebugJoint(ndConstraintDebugCallback&) const;
	void InitPointParam(dgPointParam& param, const ndVector& p0Global, const ndVector& p1Global) const;

	ndFloat32 m_preconditioner0;
	ndFloat32 m_preconditioner1;
	ndInt32 m_rowCount;
	ndInt32 m_rowStart;
	ndUnsigned8 m_active;
	ndUnsigned8 m_fence0;
	ndUnsigned8 m_fence1;
	ndUnsigned8 m_resting;   // this should be identical to m_fence0, should be removed. 
	ndUnsigned8 m_isInSkeletonLoop;
	
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

