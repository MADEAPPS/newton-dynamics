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
#include <CustomVehicleControllerBodyState.h>


#define VEHICLE_VEL_DAMP				        dFloat(100.0f)
#define VEHICLE_POS_DAMP				        dFloat(1500.0f)
#define VEHICLE_MAX_FRICTION_BOUND	            dFloat(1.0e15f)
#define VEHICLE_MIN_FRICTION_BOUND			    -VEHICLE_MAX_FRICTION_BOUND


void VehicleJoint::Init(CustomVehicleController* const controller, CustomVehicleControllerBodyState* const state0, CustomVehicleControllerBodyState* const state1)
{
	m_start = 0;
	m_count = 0;
	memset (m_rowIsMotor, 0, sizeof (m_rowIsMotor));
	memset (m_motorAcceleration, 0, sizeof (m_motorAcceleration));
	memset (m_jointFeebackForce, 0, sizeof (m_jointFeebackForce));

	m_state0 = state0;
	m_state1 = state1;
	m_controller = controller;
}


void VehicleJoint::InitPointParam (PointDerivativeParam& param, const dVector& pivot) const
{
	dAssert (m_state0);
	dAssert (m_state1);

	param.m_posit0 = pivot;
	param.m_r0 = pivot - m_state0->m_globalCentreOfMass;
	param.m_veloc0 = m_state0->m_omega * param.m_r0;
	param.m_centripetal0 = m_state0->m_omega * param.m_veloc0;
	param.m_veloc0 += m_state0->m_veloc;

	param.m_posit1 = pivot;
	param.m_r1 = pivot - m_state1->m_globalCentreOfMass;
	param.m_veloc1 = m_state1->m_omega * param.m_r1;
	param.m_centripetal1 = m_state1->m_omega * param.m_veloc1;
	param.m_veloc1 += m_state1->m_veloc;
}


void VehicleJoint::CalculatePointDerivative (ParamInfo* const constraintParams, const dVector& dir, const PointDerivativeParam& param)
{
	int index = constraintParams->m_count;

	Jacobian &jacobian0 = constraintParams->m_jacobians[index].m_jacobian_IM0; 
	dVector r0CrossDir (param.m_r0 * dir);
	jacobian0.m_linear[0] = dir.m_x;
	jacobian0.m_linear[1] = dir.m_y;
	jacobian0.m_linear[2] = dir.m_z;
	jacobian0.m_linear[3] = dFloat (0.0f);
	jacobian0.m_angular[0] = r0CrossDir.m_x;
	jacobian0.m_angular[1] = r0CrossDir.m_y;
	jacobian0.m_angular[2] = r0CrossDir.m_z;
	jacobian0.m_angular[3] = 0.0f;

	Jacobian &jacobian1 = constraintParams->m_jacobians[index].m_jacobian_IM1; 
	dVector r1CrossDir (dir * param.m_r1);
	jacobian1.m_linear[0] = -dir.m_x;
	jacobian1.m_linear[1] = -dir.m_y;
	jacobian1.m_linear[2] = -dir.m_z;
	jacobian1.m_linear[3] = dFloat (0.0f);
	jacobian1.m_angular[0] = r1CrossDir.m_x;
	jacobian1.m_angular[1] = r1CrossDir.m_y;
	jacobian1.m_angular[2] = r1CrossDir.m_z;
	jacobian1.m_angular[3] = 0.0f;

	dVector velocError (param.m_veloc1 - param.m_veloc0);
	dVector positError (param.m_posit1 - param.m_posit0);
	dVector centrError (param.m_centripetal1 - param.m_centripetal0);

	dFloat relPosit = positError % dir;
	dFloat relVeloc = velocError % dir;
	dFloat relCentr = centrError % dir; 

	dFloat dt = constraintParams->m_timestep;
	dFloat ks = VEHICLE_POS_DAMP;
	dFloat kd = VEHICLE_VEL_DAMP;
	dFloat ksd = dt * ks;
	dFloat num = ks * relPosit + kd * relVeloc + ksd * relVeloc;
	dFloat den = dFloat (1.0f) + dt * kd + dt * ksd;
	dFloat accelError = num / den;

	m_rowIsMotor[index] = false;
	m_motorAcceleration[index] = 0.0f;
	constraintParams->m_jointAccel[index] = accelError + relCentr;
	constraintParams->m_jointLowFriction[index] = VEHICLE_MIN_FRICTION_BOUND;
	constraintParams->m_jointHighFriction[index] = VEHICLE_MAX_FRICTION_BOUND;
	constraintParams->m_count = index + 1;
}


void VehicleJoint::AddAngularRowJacobian (ParamInfo* const constraintParams, const dVector& dir, dFloat jointAngle)
{
	int index = constraintParams->m_count;
	Jacobian &jacobian0 = constraintParams->m_jacobians[index].m_jacobian_IM0; 

	jacobian0.m_linear[0] = 0.0f;
	jacobian0.m_linear[1] = 0.0f;
	jacobian0.m_linear[2] = 0.0f;
	jacobian0.m_linear[3] = 0.0f;
	jacobian0.m_angular[0] = dir.m_x;
	jacobian0.m_angular[1] = dir.m_y;
	jacobian0.m_angular[2] = dir.m_z;
	jacobian0.m_angular[3] = 0.0f;

	Jacobian &jacobian1 = constraintParams->m_jacobians[index].m_jacobian_IM1; 
	jacobian1.m_linear[0] = 0.0f;
	jacobian1.m_linear[1] = 0.0f;
	jacobian1.m_linear[2] = 0.0f;
	jacobian1.m_linear[3] = 0.0f;
	jacobian1.m_angular[0] = -dir.m_x;
	jacobian1.m_angular[1] = -dir.m_y;
	jacobian1.m_angular[2] = -dir.m_z;
	jacobian1.m_angular[3] = 0.0f;

	const dVector& omega0 = m_state0->m_omega;
	const dVector& omega1 = m_state1->m_omega;
	dFloat omegaError = (omega1 - omega0) % dir;


	//at =  [- ks (x2 - x1) - kd * (v2 - v1) - dt * ks * (v2 - v1)] / [1 + dt * kd + dt * dt * ks] 
	dFloat dt = constraintParams->m_timestep;
	dFloat ks = VEHICLE_POS_DAMP;
	dFloat kd = VEHICLE_VEL_DAMP;
	dFloat ksd = dt * ks;
	dFloat num = ks * jointAngle + kd * omegaError + ksd * omegaError;
	dFloat den = dFloat (1.0f) + dt * kd + dt * ksd;
	dFloat alphaError = num / den;

	m_rowIsMotor[index] = false;
	m_motorAcceleration[index] = 0.0f;
	constraintParams->m_jointAccel[index] = alphaError;
	constraintParams->m_jointLowFriction[index] = VEHICLE_MIN_FRICTION_BOUND;
	constraintParams->m_jointHighFriction[index] = VEHICLE_MAX_FRICTION_BOUND;
	constraintParams->m_count = index + 1;
}


void VehicleJoint::AddAngularRowJacobian (ParamInfo* const constraintParams, const dVector& dir0, const dVector& dir1, dFloat accelerationRatio)
{
	int index = constraintParams->m_count;
	Jacobian &jacobian0 = constraintParams->m_jacobians[index].m_jacobian_IM0; 

	jacobian0.m_linear[0] = 0.0f;
	jacobian0.m_linear[1] = 0.0f;
	jacobian0.m_linear[2] = 0.0f;
	jacobian0.m_linear[3] = 0.0f;
	jacobian0.m_angular[0] = dir0.m_x;
	jacobian0.m_angular[1] = dir0.m_y;
	jacobian0.m_angular[2] = dir0.m_z;
	jacobian0.m_angular[3] = 0.0f;

	Jacobian &jacobian1 = constraintParams->m_jacobians[index].m_jacobian_IM1; 
	jacobian1.m_linear[0] = 0.0f;
	jacobian1.m_linear[1] = 0.0f;
	jacobian1.m_linear[2] = 0.0f;
	jacobian1.m_linear[3] = 0.0f;
	jacobian1.m_angular[0] = dir1.m_x;
	jacobian1.m_angular[1] = dir1.m_y;
	jacobian1.m_angular[2] = dir1.m_z;
	jacobian1.m_angular[3] = 0.0f;

	m_rowIsMotor[index] = true;
	m_motorAcceleration[index] = accelerationRatio;
	constraintParams->m_jointAccel[index] = 0.0f;
	constraintParams->m_jointLowFriction[index] = VEHICLE_MIN_FRICTION_BOUND;
	constraintParams->m_jointHighFriction[index] = VEHICLE_MAX_FRICTION_BOUND;
	constraintParams->m_count = index + 1;
}

void VehicleJoint::AddLinearRowJacobian (ParamInfo* const constraintParams, const dVector& pivot, const dVector& dir)
{
	PointDerivativeParam pointData;
	InitPointParam (pointData, pivot);
	CalculatePointDerivative (constraintParams, dir, pointData); 
}


void VehicleJoint::JointAccelerations (JointAccelerationDecriptor* const params)
{
	JacobianColum* const jacobianColElements = params->m_colMatrix;
	JacobianPair* const jacobianRowElements = params->m_rowMatrix;

    const dVector& bodyVeloc0 = m_state0->m_veloc;
    const dVector& bodyOmega0 = m_state0->m_omega;
    const dVector& bodyVeloc1 = m_state1->m_veloc;
    const dVector& bodyOmega1 = m_state1->m_omega;

	dFloat timestep = params->m_timeStep;
    dFloat kd = VEHICLE_VEL_DAMP * dFloat (4.0f);
    dFloat ks = VEHICLE_POS_DAMP * dFloat (0.25f);
	for (int k = 0; k < params->m_rowsCount; k ++) {
		if (m_rowIsMotor[k]) {
			jacobianColElements[k].m_coordenateAccel = m_motorAcceleration[k] + jacobianColElements[k].m_deltaAccel;
		} else {
            const JacobianPair& Jt = jacobianRowElements[k];
			dVector relVeloc (Jt.m_jacobian_IM0.m_linear.CompProduct(bodyVeloc0) +
							  Jt.m_jacobian_IM0.m_angular.CompProduct(bodyOmega0) + 
						      Jt.m_jacobian_IM1.m_linear.CompProduct(bodyVeloc1) +
							  Jt.m_jacobian_IM1.m_angular.CompProduct(bodyOmega1));

            dFloat vRel = relVeloc.m_x + relVeloc.m_y + relVeloc.m_z;
            dFloat aRel = jacobianColElements[k].m_deltaAccel;
            dFloat ksd = timestep * ks;
            dFloat relPosit = 0.0f - vRel * timestep * params->m_firstPassCoefFlag;

            dFloat num = ks * relPosit - kd * vRel - ksd * vRel;
            dFloat den = dFloat (1.0f) + timestep * kd + timestep * ksd;
            dFloat aRelErr = num / den;
            jacobianColElements[k].m_coordenateAccel = aRelErr + aRel;
		}
	}
}


void EngineGearJoint::UpdateSolverForces (const JacobianPair* const jacobians) const
{
#if 0
	const CustomVehicleControllerBodyStateTire* const tire = (CustomVehicleControllerBodyStateTire*)m_state1;
	const CustomVehicleControllerBodyStateEngine* const engine = (CustomVehicleControllerBodyStateEngine*)m_state0;

	// get tire lateral force
	const int index = m_start;
	const dVector& tireAxis = tire->m_matrix[0];
	dVector torque (jacobians[index].m_jacobian_IM1.m_angular.Scale(m_jointFeebackForce[0])); 

	dFloat T = torque % tireAxis;
	dFloat w = tireAxis % tire->m_omega;  
	dTrace (("omega = %f, torque = %f\n", w, T));
#endif
}

void EngineGearJoint::JacobianDerivative (ParamInfo* const constraintParams)
{
	dAssert (0);
/*
	const CustomVehicleControllerBodyStateTire* const tire = (CustomVehicleControllerBodyStateTire*)m_state1;
	const CustomVehicleControllerBodyStateEngine* const engine = (CustomVehicleControllerBodyStateEngine*)m_state0;
	const EngineComponent* const engineComponent = m_controller->GetEngine();

	dFloat scale = (engineComponent->GetGear() != EngineComponent::GearBox::m_reverseGear) ? dFloat(1.0f) : dFloat(-1.0f);

	const dVector& tireAxis = tire->m_matrix[0];
	const dVector& engineAxis = engine->m_matrix[0];

	dFloat tireOmega = tireAxis % tire->m_omega;  
	dFloat engineOmega = engineAxis % engine->m_omega;  

//	dTrace (("tire = %f, engine = %f\n", tireOmega, engineOmega));
	dFloat ratio = -(tireOmega + engineOmega / m_powerTrainGain) * constraintParams->m_timestepInv;
	AddAngularRowJacobian (constraintParams, engineAxis, tireAxis.Scale (scale), ratio);
*/
}

void EngineIdleJoint::UpdateSolverForces (const JacobianPair* const jacobians) const
{
}

void EngineIdleJoint::JacobianDerivative (ParamInfo* const constraintParams)
{
	dAssert (0);
	/*

	EngineComponent* const engine = m_controller->GetEngine();
	CustomVehicleControllerBodyStateEngine* const engineState = (CustomVehicleControllerBodyStateEngine*) m_state0;
	
	const dVector& pin = engineState->m_matrix[0];

	int index = constraintParams->m_count;
	AddAngularRowJacobian (constraintParams, pin, 0.0f);
	dAssert (engineState == &m_controller->m_engineState);

	dFloat alpha = 0.35f * (engineState->m_omega % pin - engine->GetIdleRadianPerSeconds()) * constraintParams->m_timestepInv;

	m_rowIsMotor[index] = true;
	m_motorAcceleration[index] = - alpha;
	constraintParams->m_jointAccel[index] = 0.0f;
	constraintParams->m_jointLowFriction[index] = -m_friction;
	constraintParams->m_jointHighFriction[index] = m_friction;
*/
}


void TireJoint::JacobianDerivative (ParamInfo* const constraintParams)
{
	// Restrict the movement on the pivot point along all two orthonormal direction
	CustomVehicleControllerBodyStateTire* const tire = (CustomVehicleControllerBodyStateTire*) m_state1;

	// lateral force
	AddLinearRowJacobian (constraintParams, tire->m_matrix.m_posit, tire->m_matrix[0]);

	// longitudinal force
	AddLinearRowJacobian (constraintParams, tire->m_matrix.m_posit, tire->m_matrix[2]);

	if (tire->m_posit <= 1.0e-3f)  {
		//dAssert (0);
		// add the stop constraint here
		//AddLinearRowJacobian (params, centerInChassis, centerInTire, chassisPivotMatrix[1]);
		//params.m_jointLowFriction[params.m_rows - 1] = 0;
		//m_tire->m_suspensionTochassisImpulseIndex = params.m_rows - 1;
	}

	AddAngularRowJacobian (constraintParams, tire->m_matrix[1], 0.0f);
	AddAngularRowJacobian (constraintParams, tire->m_matrix[2], 0.0f);

	// check if the brakes are applied
	//dFloat breakResistance = dMax(tire->m_breakTorque, tire->m_engineTorqueResistance);
    dFloat breakResistance = tire->m_breakTorque;
	if (breakResistance > 1.0e-3f) {
		int index = constraintParams->m_count;
		AddAngularRowJacobian (constraintParams, tire->m_matrix[0], 0.0f);                    
		constraintParams->m_jointLowFriction[index] = -breakResistance;
		constraintParams->m_jointHighFriction[index] = breakResistance;
	}

	// clear the input variable after there are res
	tire->m_breakTorque = 0.0f;
}



void TireJoint::UpdateSolverForces (const JacobianPair* const jacobians) const
{
	CustomVehicleControllerBodyStateTire* const tire = (CustomVehicleControllerBodyStateTire*) m_state1;

	// get tire lateral force
	const int index = m_start;
	tire->m_lateralForce = jacobians[index].m_jacobian_IM1.m_linear.Scale(m_jointFeebackForce[0]); 

	// get tire longitudinal force
	tire->m_longitidinalForce = jacobians[index + 1].m_jacobian_IM1.m_linear.Scale(m_jointFeebackForce[1]); 
}


ContactJoint::ContactJoint ()
    :m_contactCount(0)
{
}


void ContactJoint::JacobianDerivative (ParamInfo* const constraintParams)
{
	dAssert (0);
	/*

	CustomVehicleControllerBodyStateTire* const tire = (CustomVehicleControllerBodyStateTire*) m_state1;
	const dMatrix& tireMatrix = tire->m_matrix;

	const InterpolationCurve& lateralSlipAngleCurve = m_controller->m_tireLateralSlipAngle;
	const InterpolationCurve& longitudinalSlipRationCurve = m_controller->m_tireLongitidialSlipRatio;
	for (int i = 0; i < m_contactCount; i ++) {
		// rubber tire traction friction model
		dVector normal (m_contacts[i].m_normal);
		dVector lateralPin (tireMatrix[0]);
		dVector longitudinalPin (normal * lateralPin);
		dFloat mag2 = longitudinalPin % longitudinalPin;
		if (mag2 > 1.0e-3f) {
			longitudinalPin = longitudinalPin.Scale (1.0f / dSqrt(mag2));
			lateralPin = longitudinalPin * normal;
	
			dVector contactPoint (m_contacts[i].m_point);
			dVector radius (contactPoint - tireMatrix[3]);
			dVector hitBodyPointVelocity (0.0f, 0.0f, 0.0f, 0.0f);

			dVector contactRotationalVeloc (tire->m_omega * radius);
			dVector headingVeloc (tire->m_veloc - hitBodyPointVelocity);
			headingVeloc = headingVeloc - normal.Scale (headingVeloc % normal);

			dFloat u = longitudinalPin % headingVeloc;
			dFloat Rw = longitudinalPin % contactRotationalVeloc;

			// calculate longitudinal slip ratio 
			dFloat longitudinalSlipRatio = 0.0f;
			if (dAbs (Rw) >= dAbs (u)) {
				// tire is accelerating
				// note: many books this is show u and rw as a scalar value but in reality u is a vector, the sign is already in the quantity
				if (dAbs (Rw) < 0.001f) {
					longitudinalSlipRatio = 1.0f;
				} else {
					longitudinalSlipRatio = (Rw + u) / Rw;
				}
			}  else {
				// tire is breaking
				// note: many books this is show u and rw as a scalar value but in reality u is a vector, the sign is already in the quantity
				if (dAbs (u) < 0.001f) {
					longitudinalSlipRatio = 1.0f;
				} else {
					longitudinalSlipRatio = (Rw + u) / u;
				}
			}


			// the SlipRatio must be between -1.0 and 1.0 
			longitudinalSlipRatio = dClamp(longitudinalSlipRatio, dFloat(-1.0f), dFloat(1.0f));

			// calculate lateral slip angle
			dFloat sideSlipAngle = 1.0f;
			if (dAbs (u) > 1.0f) {
				dFloat mag2 = headingVeloc % headingVeloc;
				dFloat vx = dAbs (headingVeloc % longitudinalPin);
				dFloat vy = dSqrt (dMax (mag2 - vx * vx, dFloat(0.1f)));
				sideSlipAngle = dAtan2 (vy, vx);
				dAssert (sideSlipAngle >= 0.0f);
				dAssert (sideSlipAngle <= (3.141592f * 0.5f));
			}

			// get the normalized lateral and longitudinal forces
			dAssert (sideSlipAngle >= 0.0f);
			dFloat normalizedLateralForce = lateralSlipAngleCurve.GetValue (sideSlipAngle);
			dAssert (normalizedLateralForce >= 0.0f);
			dAssert (normalizedLateralForce <= 1.0f);

			dFloat normalizedLongitudinalForce = longitudinalSlipRationCurve.GetValue (longitudinalSlipRatio);
			dAssert (normalizedLongitudinalForce >= 0.0f);
			dAssert (normalizedLongitudinalForce <= 1.0f);

			// apply circle of friction
			dFloat mag2 = normalizedLongitudinalForce * normalizedLongitudinalForce + normalizedLateralForce * normalizedLateralForce;
			if (mag2 > 1.0f) {
				// if tire fore is large that the circle of friction, 
				// longitudinal force is the dominant force, and the lateral force is project over the circle of friction
				normalizedLateralForce = dSqrt (1.0f - normalizedLongitudinalForce * normalizedLongitudinalForce);
				if (normalizedLateralForce < VEHICLE_SIDESLEP_NORMALIZED_FRICTION_AT_MAX_SLIP_ANGLE){
					// do not allow lateral friction to be zero
					normalizedLateralForce = VEHICLE_SIDESLEP_NORMALIZED_FRICTION_AT_MAX_SLIP_ANGLE;
				}
			}
		
			// for now make the material friction ate the tire contact 100%
			dFloat contactGroundFriction = 1.0f;

			// get tire load
			dFloat tireLoad = (tire->m_tireLoad % tireMatrix[1]) * contactGroundFriction * tire->m_adhesionCoefficient;

			// add a lateral force constraint row at the contact point
			int index = constraintParams->m_count;
			dFloat lateralSpeed = lateralPin % headingVeloc;
			AddLinearRowJacobian (constraintParams, contactPoint, lateralPin);
			constraintParams->m_jointLowFriction[index] = -tireLoad * normalizedLateralForce;
			constraintParams->m_jointHighFriction[index] = tireLoad * normalizedLateralForce;
			constraintParams->m_jointAccel[index] = - lateralSpeed * constraintParams->m_timestepInv;
			index ++;

			// add a longitudinal force constraint row at the contact point
			dVector contactVelocity = headingVeloc + contactRotationalVeloc;
			dFloat longitudinalSpeed = longitudinalPin % contactVelocity;
			AddLinearRowJacobian (constraintParams, contactPoint, longitudinalPin);
			constraintParams->m_jointLowFriction[index] = - tireLoad * normalizedLongitudinalForce;
			constraintParams->m_jointHighFriction[index] = tireLoad * normalizedLongitudinalForce;
			constraintParams->m_jointAccel[index] = - longitudinalSpeed * constraintParams->m_timestepInv;

			if (tire->m_posit <= 1.0e-3f)  {

				dAssert (0);
				// add the stop constraint here
				AddLinearRowJacobian (params, tire.m_contactPoint, tire.m_contactPoint, upPin);
				params.m_jointLowFriction[params.m_rows - 1] = 0;

				PointDerivativeParam pointData;
				InitPointParam (pointData, tire.m_contactPoint, tire.m_contactPoint);
				dVector velocError (pointData.m_veloc1 - pointData.m_veloc0);
				dFloat restitution = 0.01f;
				dFloat relVelocErr = velocError.dot(upPin);
				if (relVelocErr > 0.0f) {
					relVelocErr *= (restitution + dFloat (1.0f));
				params.m_jointAccel[params.m_rows - 1] = dgMax (dFloat (-4.0f), relVelocErr) * params.m_invTimestep;

			}
		}
	}
*/
}

void ContactJoint::UpdateSolverForces (const JacobianPair* const jacobians) const
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
}


void ContactJoint::JointAccelerations (JointAccelerationDecriptor* const accelParam)
{
	JacobianColum* const jacobianColElements = accelParam->m_colMatrix;
	JacobianPair* const jacobianMatrixElements = accelParam->m_rowMatrix;

	const dVector& bodyVeloc0 = m_state0->m_veloc;
	const dVector& bodyOmega0 = m_state0->m_omega;
	const dVector& bodyVeloc1 = m_state1->m_veloc;
	const dVector& bodyOmega1 = m_state1->m_omega;

	int count = accelParam->m_rowsCount;
	dFloat invTimestep = accelParam->m_invTimeStep;
	for (int k = 0; k < count; k ++) {
		JacobianColum* const col = &jacobianColElements[k];
		JacobianPair* const row = &jacobianMatrixElements[k];

		dVector relVeloc (row->m_jacobian_IM0.m_linear.CompProduct(bodyVeloc0) +
						  row->m_jacobian_IM0.m_angular.CompProduct(bodyOmega0) +
						  row->m_jacobian_IM1.m_linear.CompProduct(bodyVeloc1) +
						  row->m_jacobian_IM1.m_angular.CompProduct(bodyOmega1));

		dFloat vRel = relVeloc.m_x + relVeloc.m_y + relVeloc.m_z;
		dFloat aRel = col->m_deltaAccel;
		col->m_coordenateAccel = (aRel - vRel * invTimestep);
	}
}

