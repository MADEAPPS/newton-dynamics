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


#ifndef __D_VEHICLE_VIRTUAL_TIRE_H__
#define __D_VEHICLE_VIRTUAL__TIRE_H__

#include "dStdafxVehicle.h"
#include "dVehicleTireInterface.h"

class dVehicleVirtualTire: public dVehicleTireInterface
{
	class dTireJoint: public dComplementaritySolver::dBilateralJoint
	{
		public:
		void JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams);
		void UpdateSolverForces(const dComplementaritySolver::dJacobianPair* const jacobians) const{dAssert (0);}

		dVehicleVirtualTire* m_tire;
	};

	class dContact: public dKinematicLoopJoint
	{
		public:
		dContact();
		void JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams);
		void UpdateSolverForces(const dComplementaritySolver::dJacobianPair* const jacobians) const { dAssert(0); }
		void SetContact (const dMatrix& contact, dFloat penetration);
		int GetMaxDof() const { return 3;}

		dMatrix m_contact;
		dFloat m_penetration;
	};

	public:
	DVEHICLE_API dVehicleVirtualTire(dVehicleNode* const parent, const dMatrix& locationInGlobalSpace, const dTireInfo& info, const dMatrix& localFrame);
	DVEHICLE_API virtual ~dVehicleVirtualTire();

	DVEHICLE_API dMatrix GetLocalMatrix () const;
	DVEHICLE_API virtual dMatrix GetGlobalMatrix () const;
	DVEHICLE_API virtual NewtonCollision* GetCollisionShape() const;
	DVEHICLE_API virtual void SetSteeringAngle(dFloat steeringAngle);
	DVEHICLE_API void Debug(dCustomJoint::dDebugDisplay* const debugContext) const;

	protected:
	void ApplyExternalForce();
	void Integrate(dFloat timestep);
	dComplementaritySolver::dBilateralJoint* GetJoint();
	dMatrix GetHardpointMatrix (dFloat param) const;
	int GetKinematicLoops(dKinematicLoopJoint** const jointArray);
	void CalculateNodeAABB(const dMatrix& matrix, dVector& minP, dVector& maxP) const;
	void CalculateContacts(const dVehicleChassis::dCollectCollidingBodies& bodyArray, dFloat timestep);

	static void RenderDebugTire(void* userData, int vertexCount, const dFloat* const faceVertec, int id);

	dTireInfo m_info;
	dMatrix m_matrix;
	dMatrix m_bindingRotation;
	dTireJoint m_joint;
	dVehicleNode m_dynamicContactBodyNode;
	dContact m_contactsJoints[3];
	NewtonCollision* m_tireShape;
	dFloat m_omega;
	dFloat m_speed;
	dFloat m_position;
	dFloat m_tireLoad;
	dFloat m_tireAngle;
	dFloat m_steeringAngle;
	dFloat m_invSuspensionLength;

	friend class dVehicleChassis;
};


#endif 

