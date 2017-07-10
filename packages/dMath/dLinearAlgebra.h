/* Copyright (c) <2003-2016> <Newton Game Dynamics>
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

#ifdef _MSC_VER
	#pragma warning (disable: 4100) //unreferenced formal parameter
#endif


#define D_MAX_PARAM_INFO_SIZE		16 
#define D_MAX_PLACEMENT_CONTACTS	128


class dSpatialVector
{
	public:
	inline dSpatialVector()
	{
	}

	inline dSpatialVector(const dFloat a)
	{
		for (int i = 0; i < 6; i++) {
			m_d[i] = a;
		}
	}

	inline dSpatialVector(const dFloat* const ptr)
	{
		for (int i = 0; i < 6; i++) {
			m_d[i] = ptr[i];
		}
	}


	inline dSpatialVector(const dVector& low, const dVector& high)
	{
		m_d[0] = low[0];
		m_d[1] = low[1];
		m_d[2] = low[2];
		m_d[3] = high[0];
		m_d[4] = high[1];
		m_d[5] = high[2];
	}

	inline dFloat& operator[] (int i)
	{
		dAssert(i < 6);
		dAssert(i >= 0);
		return m_d[i];
	}

	inline const dFloat& operator[] (int i) const
	{
		dAssert(i < 6);
		dAssert(i >= 0);
		return m_d[i];
	}

	inline dSpatialVector operator+ (const dSpatialVector& A) const
	{
		dSpatialVector tmp;
		for (int i = 0; i < 6; i++) {
			tmp[i] = m_d[i] + A.m_d[i];
		}
		return tmp;
	}

	inline dSpatialVector CompProduct(const dSpatialVector& A) const
	{
		dSpatialVector tmp;
		for (int i = 0; i < 6; i++) {
			tmp[i] = m_d[i] * A.m_d[i];
		}
		return tmp;
	}

	inline dFloat DotProduct(const dSpatialVector& v) const
	{
		dFloat acc = dFloat (0.0f);
		for (int i = 0; i < 6; i++) {
			acc += m_d[i] * v.m_d[i];
		}
		return acc;
	}

	inline dSpatialVector Scale(dFloat s) const
	{
		dSpatialVector tmp;
		for (int i = 0; i < 6; i++) {
			tmp[i] = m_d[i] * s;
		}
		return tmp;
	}

	dFloat m_d[6];
};

class dSpatialMatrix
{
	public:
	inline dSpatialMatrix()
	{
	}

	inline dSpatialMatrix(dFloat val)
	{
		for (int i = 0; i < 6; i++) {
			m_rows[i] = dSpatialVector(val);
		}
	}

	inline dSpatialVector& operator[] (int i)
	{
		dAssert(i < 6);
		dAssert(i >= 0);
		return m_rows[i];
	}

	inline const dSpatialVector& operator[] (int i) const
	{
		dAssert(i < 6);
		dAssert(i >= 0);
		return m_rows[i];
	}

	inline dSpatialVector VectorTimeMatrix(const dSpatialVector& jacobian) const
	{
		dSpatialVector tmp(m_rows[0].Scale(jacobian[0]));
		for (int i = 1; i < 6; i++) {
			tmp = tmp + m_rows[i].Scale(jacobian[i]);
		}
		return tmp;
	}

	inline dSpatialVector VectorTimeMatrix(const dSpatialVector& jacobian, int dof) const
	{
		dSpatialVector tmp(0.0f);
		for (int i = 0; i < dof; i++) {
			tmp = tmp + m_rows[i].Scale(jacobian[i]);
		}
		return tmp;
	}

	inline dSpatialMatrix Inverse(int rows) const
	{
		dSpatialMatrix copy(*this);
		dSpatialMatrix inverse(0.0f);
		for (int i = 0; i < rows; i++) {
			inverse[i][i] = dFloat(1.0f);
		}

		for (int i = 0; i < rows; i++) {
			dFloat val = copy[i][i];
			dAssert(dAbs(val) > 1.0e-12f);
			dFloat den = 1.0f / val;

			copy[i] = copy[i].Scale(den);
			copy[i][i] = 1.0f;
			inverse[i] = inverse[i].Scale(den);

			for (int j = 0; j < i; j++) {
				dFloat pivot = -copy[j][i];
				copy[j] = copy[j] + copy[i].Scale(pivot);
				inverse[j] = inverse[j] + inverse[i].Scale(pivot);
			}

			for (int j = i + 1; j < rows; j++) {
				dFloat pivot = -copy[j][i];
				copy[j] = copy[j] + copy[i].Scale(pivot);
				inverse[j] = inverse[j] + inverse[i].Scale(pivot);
			}
		}
		return inverse;
	}

	dSpatialVector m_rows[6];
};

class dSymmetricBiconjugateGradientSolve
{
	public:
	dSymmetricBiconjugateGradientSolve () {}
	virtual ~dSymmetricBiconjugateGradientSolve () {}	

	virtual dFloat64 Solve (int size, dFloat64 tolerance, dFloat64* const x, const dFloat64* const b) const;

	protected:
	virtual void MatrixTimeVector (dFloat64* const out, const dFloat64* const v) const = 0;
	virtual bool InversePrecoditionerTimeVector (dFloat64* const out, const dFloat64* const v) const = 0;

	private:
	dFloat64 DotProduct (int size, const dFloat64* const b, const dFloat64* const c) const;
	void ScaleAdd (int size, dFloat64* const a, const dFloat64* const b, dFloat64 scale, const dFloat64* const c) const;
	void Sub (int size, dFloat64* const a, const dFloat64* const b, const dFloat64* const c) const;
};

class dComplentaritySolver 
{
	public:
	class dBodyState;
	class dBilateralJoint;


	class dContact
	{
		public:
		dContact()
			:m_point(0.0f)
			,m_normal(0.0f)
		{
		}

		dVector m_point;
		dVector m_normal;
	};

	class dJacobian
	{
		public:
		dJacobian ()
			:m_linear(0.0f)
			,m_angular(0.0f)
		{
		}

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
		dJacobianPair m_jacobians[D_MAX_PARAM_INFO_SIZE];
		dFloat m_jointAccel[D_MAX_PARAM_INFO_SIZE];
		dFloat m_jointLowFriction[D_MAX_PARAM_INFO_SIZE];
		dFloat m_jointHighFriction[D_MAX_PARAM_INFO_SIZE];
		dFloat m_timestep;
		dFloat m_timestepInv;
		int m_count;
	};

	class dPointDerivativeParam
	{
		public:
		dPointDerivativeParam()
			:m_r0(0.0f)
			,m_posit0(0.0f)
			,m_veloc0(0.0f)
			,m_centripetal0(0.0f)
			,m_r1(0.0f)
			,m_posit1(0.0f)
			,m_veloc1(0.0f)
			,m_centripetal1(0.0f)
		{
		}

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

	class dBilateralJoint
	{
		public:
		dBilateralJoint()
			:m_state0(NULL)
			,m_state1(NULL)
			,m_start(0)
			,m_count(0)
		{
		}
		virtual ~dBilateralJoint(){}

		virtual void Init (dBodyState* const state0, dBodyState* const state1);

		protected:
		virtual void JacobianDerivative (dParamInfo* const constraintParams) = 0; 
		virtual void UpdateSolverForces (const dJacobianPair* const jacobians) const = 0; 
		virtual void JointAccelerations (dJointAccelerationDecriptor* const accelParam);

		void InitPointParam (dPointDerivativeParam& param, const dVector& pivot) const;
		void AddAngularRowJacobian (dParamInfo* const constraintParams, const dVector& dir, dFloat jointAngle);
		void AddLinearRowJacobian (dParamInfo* const constraintParams, const dVector& pivot, const dVector& dir);
		void AddAngularRowJacobian (dParamInfo* const constraintParams, const dVector& dir0, const dVector& dir1, dFloat ratio);
		void CalculatePointDerivative (dParamInfo* const constraintParams, const dVector& dir, const dPointDerivativeParam& param);

		dFloat m_motorAcceleration[D_MAX_PARAM_INFO_SIZE];
		dFloat m_jointFeebackForce[D_MAX_PARAM_INFO_SIZE];
		int m_rowIsMotor[D_MAX_PARAM_INFO_SIZE];
		dBodyState* m_state0;
		dBodyState* m_state1;
		int m_start;
		int m_count;

		friend class dBodyState;
		friend class dComplentaritySolver;
	};

	class dFrictionLessContactJoint: public dBilateralJoint
	{
		public: 
		dFrictionLessContactJoint()
			:dBilateralJoint()
			,m_restitution(0.0f)
			,m_count (0)
		{}
		virtual ~dFrictionLessContactJoint(){}

		void SetContacts (int count, dContact* const contacts, dFloat restitution);

		protected:
		void UpdateSolverForces (const dJacobianPair* const jacobians) const {}

		static inline int CompareContact (const dContact* const contactA, const dContact* const contactB, void* dommy);
		int ReduceContacts (int count, dContact* const contacts, dFloat tol);
		void JacobianDerivative (dParamInfo* const constraintParams);
		void JointAccelerations (dJointAccelerationDecriptor* const params);

		dContact m_contacts[D_MAX_PARAM_INFO_SIZE];
		dFloat m_restitution;
		int m_count;
	};


	class dBodyState
	{
		public:
		dBodyState();
		virtual ~dBodyState() {}

		dFloat GetMass () const;
		void SetMass (dFloat mass);

		dFloat GetInvMass () const;

		void SetInertia (dFloat Ixx, dFloat Iyy, dFloat Izz);
		void GetInertia (dFloat& Ixx, dFloat& Iyy, dFloat& Izz) const;

		void SetVeloc (const dVector& veloc);
		void SetOmega (const dVector& omega);
		const dVector& GetOmega() const; 
		const dVector& GetVelocity() const; 

		void UpdateInertia();

		void SetMatrix (const dMatrix& matrix);
		const dMatrix& GetMatrix () const;

		void SetLocalMatrix (const dMatrix& matrix);
		const dMatrix& GetLocalMatrix () const;

		void SetForce (const dVector& force);
		void SetTorque (const dVector& torque);
		const dVector& GetForce () const;
		const dVector& GetTorque () const;

		const dVector& GetCenterOfMass () const;

		void IntegrateVelocity (dFloat timestep);

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
		friend class dComplentaritySolver;
	};



	public:
	dComplentaritySolver() {};
	virtual ~dComplentaritySolver() {};

	virtual int GetActiveJoints (dBilateralJoint** const jointArray, int bufferSize)
	{
		return 0;
	}

	virtual int BuildJacobianMatrix (int jointCount, dBilateralJoint** const jointArray, dFloat timestep, dJacobianPair* const jacobianArray, dJacobianColum* const jacobianColumnArray, int maxRowCount);
	virtual void CalculateReactionsForces (int bodyCount, dBodyState** const bodyArray, int jointCount, dBilateralJoint** const jointArray, dFloat timestep, dJacobianPair* const jacobianArray, dJacobianColum* const jacobianColumnArray);
};


#endif

