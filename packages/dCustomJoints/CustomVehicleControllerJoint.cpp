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


// NewtonCustomJoint.cpp: implementation of the NewtonCustomJoint class.
//
//////////////////////////////////////////////////////////////////////
#include <CustomJointLibraryStdAfx.h>
#include <CustomVehicleControllerJoint.h>
#include <CustomVehicleControllerManager.h>
#include <CustomVehicleControllerComponent.h>
#include <CustomVehicleControllerBodyState.h>


CustomVehicleControllerJoint::CustomVehicleControllerJoint()
	:dComplemtaritySolver::dBilateralJoint()
{
}

CustomVehicleControllerJoint::~CustomVehicleControllerJoint()
{
}


void CustomVehicleControllerJoint::Init(CustomVehicleController* const controller, CustomVehicleControllerBodyState* const state0, CustomVehicleControllerBodyState* const state1)
{
	dComplemtaritySolver::dBilateralJoint::Init (state0, state1);
	m_controller = controller;
}


void CustomVehicleControllerJoint::InitPointParam (dComplemtaritySolver::dPointDerivativeParam& param, const dVector& pivot) const
{
	dComplemtaritySolver::dBilateralJoint::InitPointParam (param, pivot);
}


void CustomVehicleControllerJoint::CalculatePointDerivative (dComplemtaritySolver::dParamInfo* const constraintParams, const dVector& dir, const dComplemtaritySolver::dPointDerivativeParam& param)
{
	dComplemtaritySolver::dBilateralJoint::CalculatePointDerivative (constraintParams, dir, param);
}


void CustomVehicleControllerJoint::AddAngularRowJacobian (dComplemtaritySolver::dParamInfo* const constraintParams, const dVector& dir, dFloat jointAngle)
{
	dComplemtaritySolver::dBilateralJoint::AddAngularRowJacobian (constraintParams, dir, jointAngle);
}


void CustomVehicleControllerJoint::AddAngularRowJacobian (dComplemtaritySolver::dParamInfo* const constraintParams, const dVector& dir0, const dVector& dir1, dFloat accelerationRatio)
{
	dComplemtaritySolver::dBilateralJoint::AddAngularRowJacobian (constraintParams, dir0, dir1, accelerationRatio);
}

void CustomVehicleControllerJoint::AddLinearRowJacobian (dComplemtaritySolver::dParamInfo* const constraintParams, const dVector& pivot, const dVector& dir)
{
	dComplemtaritySolver::dBilateralJoint::AddLinearRowJacobian (constraintParams, pivot, dir);
}


void CustomVehicleControllerJoint::JointAccelerations (dComplemtaritySolver::dJointAccelerationDecriptor* const params)
{
	dComplemtaritySolver::dBilateralJoint::JointAccelerations (params);
}





void CustomVehicleControllerTireJoint::JacobianDerivative (dComplemtaritySolver::dParamInfo* const constraintParams)
{
	// Restrict the movement on the pivot point along all two orthonormal direction
	CustomVehicleControllerBodyStateTire* const tire = (CustomVehicleControllerBodyStateTire*) m_state1;
	CustomVehicleControllerBodyStateChassis* const chassis = (CustomVehicleControllerBodyStateChassis*) m_state0;
	dAssert (chassis == &m_controller->GetChassisState());
	
	// lateral force
	AddLinearRowJacobian (constraintParams, tire->m_matrix.m_posit, tire->m_matrix[0]);

	// longitudinal force
	AddLinearRowJacobian (constraintParams, tire->m_matrix.m_posit, tire->m_matrix[2]);

	// angular constraints	
	AddAngularRowJacobian (constraintParams, tire->m_matrix[1], 0.0f);
	AddAngularRowJacobian (constraintParams, tire->m_matrix[2], 0.0f);

	// dry rolling friction (for now contact, bu it should be a function of the tire angular velocity)
	int index = constraintParams->m_count;
	AddAngularRowJacobian (constraintParams, tire->m_matrix[0], 0.0f);
	constraintParams->m_jointLowFriction[index] = - chassis->m_dryRollingFrictionTorque;
	constraintParams->m_jointHighFriction[index] =  chassis->m_dryRollingFrictionTorque;

	// check if the brakes are applied
	if (tire->m_brakeTorque > 1.0e-3f) {
		// brake is on override rolling friction value
		constraintParams->m_jointLowFriction[index] = -tire->m_brakeTorque;
		constraintParams->m_jointHighFriction[index] = tire->m_brakeTorque;
	}

	// clear the input variable after there are res
	tire->m_brakeTorque = 0.0f;
}



void CustomVehicleControllerTireJoint::UpdateSolverForces (const dComplemtaritySolver::dJacobianPair* const jacobians) const
{
	CustomVehicleControllerBodyStateTire* const tire = (CustomVehicleControllerBodyStateTire*) m_state1;

	// get tire lateral force
	const int index = m_start;
	tire->m_lateralForce = jacobians[index].m_jacobian_IM1.m_linear.Scale(m_jointFeebackForce[0]); 

	// get tire longitudinal force
	tire->m_longitudinalForce = jacobians[index + 1].m_jacobian_IM1.m_linear.Scale(m_jointFeebackForce[1]); 
}






CustomVehicleControllerContactJoint::CustomVehicleControllerContactJoint ()
    :m_contactCount(0)
{
}


void CustomVehicleControllerContactJoint::JointAccelerations (dComplemtaritySolver::dJointAccelerationDecriptor* const accelParam)
{
	dComplemtaritySolver::dJacobianColum* const jacobianColElements = accelParam->m_colMatrix;
	dComplemtaritySolver::dJacobianPair* const jacobianMatrixElements = accelParam->m_rowMatrix;

	const dVector& bodyVeloc0 = m_state0->GetVelocity();
	const dVector& bodyOmega0 = m_state0->GetOmega();
	const dVector& bodyVeloc1 = m_state1->GetVelocity();
	const dVector& bodyOmega1 = m_state1->GetOmega();

	int count = accelParam->m_rowsCount;
	dFloat invTimestep = accelParam->m_invTimeStep;
	for (int k = 0; k < count; k ++) {
		dComplemtaritySolver::dJacobianColum* const col = &jacobianColElements[k];
		dComplemtaritySolver::dJacobianPair* const row = &jacobianMatrixElements[k];

		dVector relVeloc (row->m_jacobian_IM0.m_linear.CompProduct(bodyVeloc0) + row->m_jacobian_IM0.m_angular.CompProduct(bodyOmega0) +
						 row->m_jacobian_IM1.m_linear.CompProduct(bodyVeloc1) + row->m_jacobian_IM1.m_angular.CompProduct(bodyOmega1));

		dFloat vRel = relVeloc.m_x + relVeloc.m_y + relVeloc.m_z;
		dFloat aRel = col->m_deltaAccel;
		col->m_coordenateAccel = (aRel - vRel * invTimestep);
	}
}


void CustomVehicleControllerContactJoint::UpdateSolverForces (const dComplemtaritySolver::dJacobianPair* const jacobians) const
{
	CustomVehicleControllerBodyStateTire* const tire = (CustomVehicleControllerBodyStateTire*) m_state1;
	const dMatrix& tireMatrix = tire->m_matrix;

	const dVector longitudinalPin = tireMatrix[2];
	dVector radius (tireMatrix[1].Scale(-tire->m_radio));
	dVector hitBodyPointVelocity (0.0f, 0.0f, 0.0f, 0.0f);
	dVector contactRotationalVeloc (tire->m_omega * radius);
	dVector headingVeloc (tire->m_veloc - hitBodyPointVelocity);
	dFloat u = longitudinalPin % headingVeloc;

	dFloat Rw = longitudinalPin % contactRotationalVeloc;

	dFloat uMag = dAbs (u);
	dFloat wMag = dAbs (Rw);
	if (uMag > 1.0f) {
		if (wMag > (4.0f * uMag)) {
			wMag = 4.0f * uMag / tire->m_radio;
			tire->m_omega = tire->m_omega.Scale (wMag / dSqrt (tire->m_omega % tire->m_omega));
		}
	}

	// apply the forces to any body that is touching this contact
	//const int index = m_start;
	for (int i = 0; i < m_count; i ++) {
//		dAssert (0);
//		tire->m_lateralForce = jacobians[index].m_jacobian_IM1.m_linear.Scale(m_jointFeebackForce[0]); 
//		tire->m_longitudinalForce = jacobians[index + 1].m_jacobian_IM1.m_linear.Scale(m_jointFeebackForce[1]); 
	}
}


void CustomVehicleControllerContactJoint::JacobianDerivative (dComplemtaritySolver::dParamInfo* const constraintParams)
{
	CustomVehicleControllerBodyStateChassis& chassis = m_controller->m_chassisState;
	CustomVehicleControllerBodyStateTire* const tire = (CustomVehicleControllerBodyStateTire*) m_state1;
	const dMatrix& tireMatrix = tire->m_matrix;

	const dVector& upPin = chassis.m_matrix[1];
	dFloat tireLoad = tire->m_tireLoad % upPin;
	if (tireLoad > 0.01f) {
		dFloat restTireLoad = chassis.m_gravityMag * tire->m_restSprunMass;
		for (int i = 0; i < m_contactCount; i ++) {
			// rubber tire traction friction model
			dVector normal (m_contacts[i].m_normal);
			dVector lateralPin (tireMatrix[0]);
			dVector longitudinalPin (normal * lateralPin);
			dFloat pinMag2 = longitudinalPin % longitudinalPin;
			if (pinMag2 > 1.0e-3f) {

				longitudinalPin = longitudinalPin.Scale (1.0f / dSqrt(pinMag2));
				lateralPin = longitudinalPin * normal;

				dVector contactPoint (m_contacts[i].m_point);
				dVector hitBodyPointVelocity;
				NewtonBodyGetPointVelocity (m_contacts[i].m_hitBody, &contactPoint[0], &hitBodyPointVelocity[0]);
				hitBodyPointVelocity.m_w = 0.0f;

				dVector headingVeloc (tire->m_veloc + hitBodyPointVelocity);
				headingVeloc -= normal.Scale (headingVeloc % normal);

				dFloat v = lateralPin % headingVeloc;
				dFloat u = longitudinalPin % headingVeloc;

				dVector radius (contactPoint - tireMatrix[3]);
				dVector contactRotationalVeloc (tire->m_omega * radius);
				dFloat Rw = longitudinalPin % contactRotationalVeloc;

				dFloat uAbs = dAbs (u);
				dFloat vAbs = dAbs (v);
				dFloat wrAbs = dAbs (Rw);

				// calculate lateral slip angle
				dFloat sideSlipAngle = 1.0f;
				dFloat lateralSpeed = v;
				if (uAbs > 0.25f) {
					sideSlipAngle = dAtan2 (vAbs, uAbs);
					dAssert (sideSlipAngle >= 0.0f);
					dAssert (sideSlipAngle <= (3.141592f * 0.5f));

					// max sideSlip = tan(20.0f)
					if (sideSlipAngle > 0.364f) {
						lateralSpeed = v - 0.364f * uAbs * dSign (v);
					}
				}

				// calculate longitudinal slip ratio 
				dFloat longitudinalSlipRatio = 1.0f;
				if ((uAbs > 0.25f) || (wrAbs > 0.25f)) {
					longitudinalSlipRatio = dClamp((u + Rw) / u, -1.0f, 1.0f);
				}

				// get the normalize tire load
				dFloat normalizedTireLoad = dClamp (tireLoad / restTireLoad, 0.0f, 4.0f);

				// calculate longitudinal and lateral forces magnitude when no friction Limit (for now ignore camber angle effects)
				dFloat camberEffect = 0.0f;
				dFloat longitudinalStiffness = tire->m_longitudialStiffness * chassis.m_gravityMag;
				dFloat lateralStiffness = restTireLoad * tire->m_lateralStiffness * normalizedTireLoad;
				dFloat Teff = dTan (sideSlipAngle - camberEffect);

				dFloat Fy0 = lateralStiffness * Teff;
				dFloat Fx0 = longitudinalStiffness * longitudinalSlipRatio;

				// for now assume tire/road friction is 1.0
				dFloat contactGroundFriction = 1.5f;

				dFloat tireLoadFriction = contactGroundFriction * tireLoad;
				dFloat K = dSqrt (Fx0 * Fx0 + Fy0 * Fy0) / tireLoadFriction;
				dAssert (K >= 0.0f);

				// now use the friction curve approximation 
				// http://www.ricblues.nl/techniek/Technisch%20Specialist%2093430/6%20Remgedrag%20ABS%20weggedrag/Carsim%20-%20remsimulatieprogramma/Handleiding%20carsim.pdf
				// basically it replace the Pajecka equation with the with the two series expansions 
				// f = x - |x| * x / 3 + x * x * x / 27
				// m = x - |x| * x + x * x * x / 3 + x * x * x * x / 27
				dFloat tireForceCoef = dMin (K * (1.0f - K / 3.0f + K * K / 27.0f), 1.5f);

				dFloat nu = 1.0f;
				if (K < 2.0f * 3.141592f) {
					dFloat lateralToLongitudinalRatio = lateralStiffness / longitudinalStiffness;
					nu = 0.5f * (1.0f + lateralToLongitudinalRatio - (1.0f - lateralToLongitudinalRatio) * dCos (0.5f * K));
				}
			
				dFloat f0 = tireLoadFriction / dSqrt (longitudinalSlipRatio * longitudinalSlipRatio + nu * Teff * nu * Teff);
				dFloat lateralForce = dAbs (nu * Teff * tireForceCoef * f0);
				dFloat longitudinalForce = dAbs (longitudinalSlipRatio * tireForceCoef * f0);

				// ignore the tire alignment torque for now
				//dFloat k1 = dMin (K, 3.0f);
				//dFloat tireMomentCoef = k1 * (1.0f - k1 + k1 * k1 / 3.0f - k1 * k1 * k1 / 27.0f);
				//dFloat aligningMoment = nu * tire->m_aligningMomentTrail * Teff * tireMomentCoef * f0;
				//chassis.m_externalTorque += upPin.Scale (aligningMoment);

				// add a lateral force constraint row at the contact point
				int index = constraintParams->m_count;
				AddLinearRowJacobian (constraintParams, contactPoint, lateralPin);
				constraintParams->m_jointLowFriction[index] = -lateralForce;
				constraintParams->m_jointHighFriction[index] = lateralForce;
				constraintParams->m_jointAccel[index] = - 0.7f * lateralSpeed * constraintParams->m_timestepInv;
				index ++;

				// add a longitudinal force constraint row at the contact point
				dFloat longitudinalSpeed = u + Rw;
				AddLinearRowJacobian (constraintParams, contactPoint, longitudinalPin);
				constraintParams->m_jointLowFriction[index] = - longitudinalForce;
				constraintParams->m_jointHighFriction[index] = longitudinalForce;
				constraintParams->m_jointAccel[index] = - longitudinalSpeed * constraintParams->m_timestepInv;
				index ++;

				// normal force
				dComplemtaritySolver::dPointDerivativeParam pointData;
				InitPointParam (pointData, m_contacts[i].m_point);
				AddLinearRowJacobian (constraintParams, m_contacts[i].m_point, normal.Scale (-1.0f));
				constraintParams->m_jointLowFriction[index] = 0;
				dVector velocError (pointData.m_veloc0 - pointData.m_veloc1);
				dFloat restitution = 0.0f;
				dFloat relVelocErr = velocError % normal;
				if (relVelocErr > 0.0f) {
					relVelocErr *= (restitution + dFloat (1.0f));
				}
				constraintParams->m_jointAccel[index] = dMax (dFloat (-4.0f), relVelocErr) * constraintParams->m_timestepInv;
			}
		}
	}
}

/*

void CustomVehicleControllerEngineIdleJoint::UpdateSolverForces (const dComplemtaritySolver::dJacobianPair* const jacobians) const
{
}

void CustomVehicleControllerEngineIdleJoint::JacobianDerivative (dComplemtaritySolver::dParamInfo* const constraintParams)
{
	//CustomVehicleControllerComponentEngine* const engine = m_controller->GetEngine();
	CustomVehicleControllerBodyStateEngine* const engineState = (CustomVehicleControllerBodyStateEngine*) m_state0;

	const dVector& pin = engineState->m_matrix[0];

	int index = constraintParams->m_count;
	AddAngularRowJacobian (constraintParams, pin, 0.0f);
	dAssert (engineState == &m_controller->m_engineState);

	dFloat alpha = 0.35f * (engineState->m_omega % pin - m_omega) * constraintParams->m_timestepInv;

	m_rowIsMotor[index] = true;
	m_motorAcceleration[index] = - alpha;
	constraintParams->m_jointAccel[index] = 0.0f;
	constraintParams->m_jointLowFriction[index] = -m_friction;
	constraintParams->m_jointHighFriction[index] = m_friction;
}
*/

void CustomVehicleControllerEngineDifferencialJoint::UpdateSolverForces (const dComplemtaritySolver::dJacobianPair* const jacobians) const
{
}

void CustomVehicleControllerEngineDifferencialJoint::JacobianDerivative (dComplemtaritySolver::dParamInfo* const constraintParams)
{
	const CustomVehicleControllerBodyStateTire* const tire0 = (CustomVehicleControllerBodyStateTire*)m_state0;
	const CustomVehicleControllerBodyStateTire* const tire1 = (CustomVehicleControllerBodyStateTire*)m_state1;
	//const CustomVehicleControllerComponentEngine* const engineComponent = m_controller->GetEngine();

	dVector axis0 (tire0->m_matrix[0]);
	dVector axis1 (tire1->m_matrix[0].Scale (-1.0f));

	dFloat omega0 = axis0 % tire0->m_omega;  
	dFloat omega1 = axis1 % tire1->m_omega;  

	dFloat ratioAccel = - 0.5f * (omega0 + m_ratio * omega1) * constraintParams->m_timestepInv;
	AddAngularRowJacobian (constraintParams, axis0, axis1, ratioAccel);
}
