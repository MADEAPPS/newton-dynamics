/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "dStdAfxMath.h"
#include "dMathDefines.h"
#include "dVector.h"
#include "dMatrix.h"
#include "dQuaternion.h"


#ifndef __D_LINEAR_ALGEBRA_H__
#define __D_LINEAR_ALGEBRA_H__

class dSymmetricBiconjugateGradientSolve
{
	public:
	dSymmetricBiconjugateGradientSolve ();
	~dSymmetricBiconjugateGradientSolve ();	

	dFloat64 Solve (int size, dFloat64 tolerance, dFloat64* const x, const dFloat64* const b) const;

	protected:
	virtual void MatrixTimeVector (dFloat64* const out, const dFloat64* const v) const = 0;
	virtual bool InversePrecoditionerTimeVector (dFloat64* const out, const dFloat64* const v) const = 0;

	private:
	dFloat64 DotProduct (int size, const dFloat64* const b, const dFloat64* const c) const;
	void ScaleAdd (int size, dFloat64* const a, const dFloat64* const b, dFloat64 scale, const dFloat64* const c) const;
	void Sub (int size, dFloat64* const a, const dFloat64* const b, const dFloat64* const c) const;
};

class dComplemtaritySolver 
{
	public:
	class dBodyState;
	class dBilateralJoint;

	class dJacobian
	{
		public:
		dVector m_linear;
		dVector m_angular;
	};

	class dJacobianPair
	{
		public:
		dJacobian m_jacobian_IM0;
		dJacobian m_jacobian_IM1;
	};

	class dJacobianColum
	{
		public:
		dFloat m_force;
		dFloat m_diagDamp;
		dFloat m_deltaAccel;
		dFloat m_invDJMinvJt;
		dFloat m_coordenateAccel;
		dFloat m_jointLowFriction;
		dFloat m_jointHighFriction;
	};

	class dParamInfo
	{
		public:
		dJacobianPair m_jacobians[8];
		dFloat m_jointAccel[8];
		dFloat m_jointLowFriction[8];
		dFloat m_jointHighFriction[8];
		dFloat m_timestep;
		dFloat m_timestepInv;
		int m_count;
	};

	class dPointDerivativeParam
	{
		public:
		dVector m_r0;
		dVector m_posit0;
		dVector m_veloc0;
		dVector m_centripetal0;

		dVector m_r1;
		dVector m_posit1;
		dVector m_veloc1;
		dVector m_centripetal1;
	};

	class dJointAccelerationDecriptor
	{
		public:
		int m_rowsCount;
		dFloat m_timeStep;
		dFloat m_invTimeStep;
		dFloat m_firstPassCoefFlag;
		dJacobianPair* m_rowMatrix;
		dJacobianColum* m_colMatrix;
	};



	class dBodyState
	{
		public:
		dBodyState();
		virtual ~dBodyState() {}

		dFloat GetMass () const;
		void UpdateInertia();
		const dMatrix& GetMatrix () const;
		const dMatrix& GetLocalMatrix () const;
		const dVector& GetCenterOfMass () const;

		protected:
		virtual void IntegrateForce (dFloat timestep, const dVector& force, const dVector& torque);
		virtual void ApplyNetForceAndTorque (dFloat invTimestep, const dVector& veloc, const dVector& omega);

		dMatrix m_matrix;
		dMatrix m_localFrame;
		dMatrix m_inertia;
		dMatrix m_invInertia;

		dVector m_localInertia;
		dVector m_localInvInertia;

		dVector m_veloc;
		dVector m_omega;
		dVector m_externalForce;
		dVector m_externalTorque;
		dVector m_globalCentreOfMass;

		dFloat m_mass;
		dFloat m_invMass;
		int m_myIndex;

		friend class dBilateralJoint;
	};


	class dBilateralJoint
	{
		dBilateralJoint(){}
		virtual ~dBilateralJoint(){}

		virtual void Init (dBodyState* const state0, dBodyState* const state1);

		virtual void UpdateSolverForces (const dJacobianPair* const jacobians) const = 0; 
		virtual void JacobianDerivative (dParamInfo* const constraintParams) = 0; 
		virtual void JointAccelerations (dJointAccelerationDecriptor* const accelParam);

		void InitPointParam (dPointDerivativeParam& param, const dVector& pivot) const;
		void AddAngularRowJacobian (dParamInfo* const constraintParams, const dVector& dir, dFloat jointAngle);
		void AddLinearRowJacobian (dParamInfo* const constraintParams, const dVector& pivot, const dVector& dir);
		void AddAngularRowJacobian (dParamInfo* const constraintParams, const dVector& dir0, const dVector& dir1, dFloat ratio);
		void CalculatePointDerivative (dParamInfo* const constraintParams, const dVector& dir, const dPointDerivativeParam& param);

		dFloat m_motorAcceleration[8];
		dFloat m_jointFeebackForce[8];
		int m_rowIsMotor[8];
		dBodyState* m_state0;
		dBodyState* m_state1;
		int m_start;
		int m_count;

		friend class dBodyState;
	};

	public:
	dComplemtaritySolver();
	~dComplemtaritySolver() {};

	virtual int GetActiveJoints (dBilateralJoint** const jointArray, int bufferSize);
	int BuildJacobianMatrix (int jointCount, dBilateralJoint** const jointArray, dFloat timestep, dJacobianPair* const jacobianArray, dJacobianColum* const jacobianColumnArray);
	void CalculateReactionsForces(int jointCount, dBilateralJoint** const jointArray, dFloat timestep, dJacobianPair* const jacobianArray, dJacobianColum* const jacobianColumnArray);
};


#endif

