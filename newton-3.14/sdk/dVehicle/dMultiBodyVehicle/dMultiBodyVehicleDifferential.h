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

class dMultiBodyVehicle;

class dMultiBodyVehicleDifferential: public dVehicleNode, public dComplementaritySolver::dBilateralJoint
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
		friend class dMultiBodyVehicleDifferential;
	};

	public:
	DVEHICLE_API dMultiBodyVehicleDifferential(dMultiBodyVehicle* const chassis, dFloat mass, dFloat radius, dVehicleNode* const leftNode, dVehicleNode* const rightNode, const dMatrix& axelMatrix = dGetIdentityMatrix());
	DVEHICLE_API virtual ~dMultiBodyVehicleDifferential();

	DVEHICLE_API int GetMode() const;
	DVEHICLE_API void SetMode(int mode);
	
	protected:
	void ApplyExternalForce();
	void Integrate(dFloat timestep);
	void CalculateFreeDof(dFloat timestep);
	int GetKinematicLoops(dVehicleLoopJoint** const jointArray);

	dComplementaritySolver::dBilateralJoint* GetJoint() {return this;}
	dMultiBodyVehicleDifferential* GetAsDifferential() const { return (dMultiBodyVehicleDifferential*)this; }
	
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

	friend class dMultiBodyVehicle;
};


#endif 

