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


#ifndef __D_VEHICLE_TIRE_H__
#define __D_VEHICLE_TIRE_H__

#include "dStdafxVehicle.h"
#include "dVehicleNode.h"

class dVehicleChassis;
class dTireInfo
{
	public:
	enum dSuspensionType
	{
		m_offroad,
		m_confort,
		m_race,
		m_roller,
	};

	dTireInfo()
	{
		memset(this, 0, sizeof(dTireInfo));

		dFloat gravity = 9.8f;
		dFloat vehicleMass = 1000.0f;

		m_mass = 25.0f;
		m_radio = 0.5f;
		m_width = 0.15f;
		m_pivotOffset = 0.01f;
		m_steerRate = 0.5f * dPi;
		m_frictionCoefficient = 0.8f;
		m_maxSteeringAngle = 40.0f * dDegreeToRad;

		m_suspensionLength = 0.3f;
		m_dampingRatio = 15.0f * vehicleMass;
		m_springStiffness = dAbs(vehicleMass * 9.8f * 8.0f / m_suspensionLength);

		m_corneringStiffness = dAbs(vehicleMass * gravity * 1.0f);
		m_longitudinalStiffness = dAbs(vehicleMass * gravity * 1.0f);
	}

	dVector m_location;
	dFloat m_mass;
	dFloat m_radio;
	dFloat m_width;
	dFloat m_steerRate;
	dFloat m_pivotOffset;
	dFloat m_dampingRatio;
	dFloat m_springStiffness;
	dFloat m_suspensionLength;
	dFloat m_maxSteeringAngle;
	dFloat m_corneringStiffness;
	dFloat m_longitudinalStiffness;
	dFloat m_frictionCoefficient;
	//dFloat m_aligningMomentTrail;
	//int m_hasFender;
	//dSuspensionType m_suspentionType;
};

class dVehicleTire: public dVehicleNode
{
	public:
	DVEHICLE_API dVehicleTire(dVehicleChassis* const chassis, const dMatrix& locationInGlobalSpace, const dTireInfo& info);
	DVEHICLE_API virtual ~dVehicleTire();

	virtual dVehicleTire* GetAsTire() const { return (dVehicleTire*)this; }
	DVEHICLE_API dMatrix GetLocalMatrix () const;
	DVEHICLE_API virtual dMatrix GetGlobalMatrix () const;
//	DVEHICLE_API virtual NewtonCollision* GetCollisionShape() const;
//
//	DVEHICLE_API virtual dFloat GetSteeringAngle() const;
//	DVEHICLE_API virtual void SetSteeringAngle(dFloat steeringAngle);
//
//	DVEHICLE_API virtual dFloat GetBrakeTorque() const;
//	DVEHICLE_API virtual void SetBrakeTorque(dFloat brakeTorque);

	protected:
//	void ApplyExternalForce(dFloat timestep);
//	void Integrate(dFloat timestep);
//	dComplementaritySolver::dBilateralJoint* GetProxyJoint();
	dMatrix GetHardpointMatrix (dFloat param) const;
//	int GetKinematicLoops(dAnimIDRigKinematicLoopJoint** const jointArray);
//	void CalculateNodeAABB(const dMatrix& matrix, dVector& minP, dVector& maxP) const;
//	void CalculateContacts(const dVehicleChassis::dCollectCollidingBodies& bodyArray, dFloat timestep);

	private: 
	void ApplyExternalForce();
	const void Debug(dCustomJoint::dDebugDisplay* const debugContext) const;
	static void RenderDebugTire(void* userData, int vertexCount, const dFloat* const faceVertec, int id);

	dMatrix m_matrix;
	dMatrix m_bindingRotation;
//	dTireJoint m_proxyJoint;
//	dVehicleNode m_dynamicContactBodyNode;
//	dTireContact m_contactsJoints[3];
	dTireInfo m_info;
	NewtonCollision* m_tireShape;
	dFloat m_omega;
	dFloat m_speed;
	dFloat m_position;
	dFloat m_tireAngle;
//	dFloat m_brakeTorque;
	dFloat m_steeringAngle;
	dFloat m_invSuspensionLength;
	friend class dVehicleChassis;
};


#endif 

