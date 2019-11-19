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


#ifndef __D_VEHICLE_DIFFERENTIAL_H__
#define __D_VEHICLE_DIFFERENTIAL_H__

#include "dStdafxVehicle.h"
#include "dVehicleNode.h"
#include "dVehicleLoopJoint.h"

class dVehicleMultiBody;

class dVehicleDifferential: public dVehicleNode, public dComplementaritySolver::dBilateralJoint
{
	enum dOperationMode
	{
		m_open,
		m_slipLocked,
		m_rightLocked,
		m_leftLocked,
	};

	class dTireAxleJoint: public dVehicleLoopJoint
	{
		public:
		dTireAxleJoint()
			:dVehicleLoopJoint()
			,m_diffSign(1.0f)
		{
			m_isActive = true;
		}

		private:
		int GetMaxDof() const { return 1; }
		void JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams);
		void UpdateSolverForces(const dComplementaritySolver::dJacobianPair* const jacobians) const { dAssert(0); }

		dFloat m_diffSign;
		friend class dVehicleDifferential;
	};

	public:
	DVEHICLE_API dVehicleDifferential(dVehicleMultiBody* const chassis, dFloat mass, dFloat radius, dVehicleNode* const leftNode, dVehicleNode* const rightNode, const dMatrix& axelMatrix = dGetIdentityMatrix());
	DVEHICLE_API virtual ~dVehicleDifferential();

	DVEHICLE_API int GetMode() const;
	DVEHICLE_API void SetMode(int mode);
	
	protected:
	void CalculateFreeDof();
	void ApplyExternalForce();
	void Integrate(dFloat timestep);
	int GetKinematicLoops(dVehicleLoopJoint** const jointArray);

	dComplementaritySolver::dBilateralJoint* GetJoint() {return this;}
	dVehicleDifferential* GetAsDifferential() const { return (dVehicleDifferential*)this; }
	
	const void Debug(dCustomJoint::dDebugDisplay* const debugContext) const;
	void JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams);
	void UpdateSolverForces(const dComplementaritySolver::dJacobianPair* const jacobians) const;

	dMatrix m_localAxis;
	dMatrix m_axelMatrix;
	dTireAxleJoint m_leftAxle;
	dTireAxleJoint m_rightAxle;
	dVehicleNode* m_leftNode;
	dVehicleNode* m_rightNode;
	dFloat m_diffOmega;
	dFloat m_shaftOmega;
	dFloat m_frictionLost;
	dOperationMode m_mode;

	friend class dVehicleMultiBody;
};


#endif 

