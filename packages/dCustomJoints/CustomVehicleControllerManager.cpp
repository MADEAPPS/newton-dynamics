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


// most the work for the tire model comes from this paper
// http://code.eng.buffalo.edu/dat/sites/tire/tire.html
// I do not really use their Simplified Tire Model Equations, 
// instead I use the explanation of the empirical tire model and use the slip and 
// side slip coefficients to determine friction limits from piecewise normalized tire force curves.
// I use these friction forces in a full analytical Lagrangian rigid body model of the vehicle. 
// Surprisingly the results are more realistic than I would expect.
//
// the only time the empirical tire model fall far from realistic behaviors 
// is when the tire liner velocity at the cent is too close to zero.
// but for this I case is handles with the constraints joints that keep the car stable.
// in fact the constraint joint effect is negligible so it can be left on during the entire simulation.
// when the car is moving a at any speed the constraint joint act a dry dolling friction.


// NewtonCustomJoint.cpp: implementation of the NewtonCustomJoint class.
//
//////////////////////////////////////////////////////////////////////
#include "CustomJointLibraryStdAfx.h"
#include "CustomJoint.h"
#include "CustomVehicleControllerManager.h"


#define VEHICLE_CONTROLLER_MAX_JOINTS			64
#define VEHICLE_CONTROLLER_MAX_JACOBIANS_PAIRS	(VEHICLE_CONTROLLER_MAX_JOINTS * 4)

#define VEHICLE_PSD_DAMP_TOL                    dFloat(1.0e-4f)
#define VEHICLE_VEL_DAMP				        dFloat(100.0f)
#define VEHICLE_POS_DAMP				        dFloat(1500.0f)
#define VEHICLE_MAX_FRICTION_BOUND	            dFloat(1.0e15f)
#define VEHICLE_MIN_FRICTION_BOUND			    -VEHICLE_MAX_FRICTION_BOUND

#define VEHICLE_SIDESLEP_NORMALIZED_FRICTION_AT_MAX_SLIP_ANGLE 0.25f



void CustomVehicleController::InterpolationCurve::InitalizeCurve (int points, const dFloat* const steps, const dFloat* const values)
{
	m_count = points;
	dAssert (points < int (sizeof(m_nodes)/sizeof (m_nodes[0])));
	memset (m_nodes, 0, sizeof (m_nodes));
	for (int i = 0; i < m_count; i ++) {
		m_nodes[i].m_param = steps[i];
		m_nodes[i].m_value = values[i];
	}
}

dFloat CustomVehicleController::InterpolationCurve::GetValue (dFloat param) const
{
	dFloat sign = (param >= 0.0f ) ? 1.0f : -1.0f;
	param = dAbs (param);
	dFloat interplatedValue = m_nodes[m_count - 1].m_value;
	for (int i = 1; i < m_count; i ++) {
		if (param < m_nodes[i].m_param) {
			dFloat df = m_nodes[i].m_value - m_nodes[i - 1].m_value;
			dFloat ds = m_nodes[i].m_param - m_nodes[i - 1].m_param;
			dFloat step = param - m_nodes[i - 1].m_param;

			interplatedValue = m_nodes[i - 1].m_value + df * step / ds;
			break;
		}
	}
	return interplatedValue * sign;
}



CustomVehicleController::EngineComponent::GearBox::GearBox(CustomVehicleController* const controller, dFloat reverseGearRatio, int gearCount, const dFloat* const gearBox)
	:m_gearsCount(gearCount + 2)
	,m_currentGear(NULL)
    ,m_controller(controller)
	,m_automatic(true)
{
	memset (m_gears, 0, sizeof (m_gears));

	dAssert (gearCount < (m_maxGears - 2));
	for (int i = 0; i < gearCount; i ++) {
        m_gears[i + m_firstGear] = new GearState (gearBox[i], 0.9f, 0.3f, GearID(i + m_firstGear)); 
	}

    for (int i = 0; i < gearCount - 1; i ++) {
        m_gears[i + m_firstGear]->m_next = m_gears[i + m_firstGear + 1]; 
    }
    

    for (int i = gearCount - 1; i; i --) {
        m_gears[i + m_firstGear]->m_prev = m_gears[i + m_firstGear - 1]; 
    }
 
    m_gears[m_reverseGear] = new ReverseGearState(reverseGearRatio);
    m_gears[m_newtralGear] = new NeutralGearState(m_gears[m_firstGear], m_gears[m_reverseGear]);

    m_gears[m_firstGear]->m_prev = m_gears[m_firstGear];
    m_gears[m_firstGear + gearCount - 1]->m_next = m_gears[m_firstGear + gearCount - 1];

    m_currentGear = m_gears[m_newtralGear];

    for (int i = 0; i < gearCount; i ++) {
        m_gears[i]->m_neutral = m_gears[m_newtralGear];
        m_gears[i]->m_reverse = m_gears[m_reverseGear];
    }
}

CustomVehicleController::EngineComponent::GearBox::~GearBox ()
{
	for (int i = 0; i < m_gearsCount; i ++) {
		delete m_gears[i]; 
	}
}


void CustomVehicleController::EngineComponent::GearBox::SetOptimalShiftLimits (dFloat minShift, dFloat maxShift)
{
	minShift = dMax (minShift - 0.05f, 0.10f);
	maxShift = dMin (maxShift + 0.05f, 0.95f);
	for (int i = m_firstGear; i < m_gearsCount; i ++) {
		GearState* const state = m_gears[i];
		state->m_shiftUp = maxShift;
		state->m_shiftDown = minShift;
	}
}

CustomVehicleController::EngineComponent::GearBox::GearState* CustomVehicleController::EngineComponent::GearBox::NeutralGearState::Update(CustomVehicleController* const vehicle)
{
    const EngineComponent* const engine = vehicle->GetEngine();
    dFloat param = engine->GetParam();

    if (param > dFloat (1.0e-3f)) {
        return m_next;
    } else if (param < dFloat(-1.0e-3f)) {
        return m_prev;
    }
    return this;
}

CustomVehicleController::EngineComponent::GearBox::GearState* CustomVehicleController::EngineComponent::GearBox::GearState::Update(CustomVehicleController* const vehicle)
{
    const EngineComponent* const engine = vehicle->GetEngine();

    dFloat param = engine->GetParam();
    dFloat speed = engine->GetSpeed();
    dFloat normalrpm = engine->GetRPM() / engine->GetTopRPM ();

    if ((normalrpm > m_shiftUp) && (speed > 1.0f)) {
        return m_next;
    } else if (normalrpm < m_shiftDown) {
        if ((dAbs (speed) < 0.5f) && (param < dFloat (1.0e-3f))) {
            return m_neutral;
        }

        if ((dAbs (speed) < 2.0f) && (param < dFloat (-1.0e-3f))) {
            return m_reverse;
        }

        if (param > dFloat (-1.0e-3f)) 

        dAssert (m_prev != m_neutral);
        dAssert (m_prev != m_reverse);
        return m_prev;
    } else if (param < 0.0f) {
        dAssert (0);
/*
        if (speed < 1.0f) {
            return m_reverse;
        }
*/
    }

    return this;
}



dFloat CustomVehicleController::EngineComponent::GearBox::GetGearRatio(int gear) const 
{
	dFloat ratio = m_gears[gear]->m_ratio;
	return (gear != m_reverseGear) ? ratio : -ratio;
}


void CustomVehicleController::EngineComponent::GearBox::SetGear(int gear) 
{
    for (int i = 0; i < m_gearsCount; i ++) {
        if (m_gears[i]->m_id == gear) {
            m_currentGear = m_gears[i];
            break;
        }
    }
}


void CustomVehicleController::EngineComponent::GearBox::Update(dFloat timestep)
{
	if (m_automatic) {
        m_currentGear = m_currentGear->Update(m_controller);
	}
}


CustomVehicleController::EngineComponent::EngineComponent (CustomVehicleController* const controller, GearBox* const gearBox, TireBodyState* const leftTire, TireBodyState* const righTire)
	:Component (controller)
	,m_gearBox(gearBox)
	,m_leftTire(controller->m_tireList.GetNodeFromInfo (*leftTire))
	,m_righTire(controller->m_tireList.GetNodeFromInfo (*righTire))
	,m_speedMPS(0.0f)
	,m_currentRPS(0.0f)
	,m_topSpeedMPS(0.0f)
	,m_fakeIdleInertia (0.05f)
	,m_engineInternalInertia (0.0f)
	,m_differentialGearRatio(1.0f)
	,m_engineOptimalRevPerSec(0.0f)
{
}

CustomVehicleController::EngineComponent::~EngineComponent()
{
	if (m_gearBox) {
		delete m_gearBox;
	}
}


void CustomVehicleController::EngineComponent::InitEngineTorqueCurve (dFloat vehicleSpeedKPH, dFloat idleTorque, dFloat idleTorqueRPM, dFloat peakTorque, dFloat peakTorqueRPM, dFloat peakHorsePower, dFloat peakHorsePowerRPM, dFloat redLineTorque, dFloat redLineTorqueRPM)
{
	dFloat rpm[5];
	dFloat torque[5];
	dFloat torqueAtPeakPower;

	const dFloat horsePowerToWatts = 745.7f;
	const dFloat rpmToRadiansPerSecunds = 0.105f;
	const dFloat poundFootToNewtonMeters = 1.356f;

	idleTorque *= poundFootToNewtonMeters;
	idleTorqueRPM *= rpmToRadiansPerSecunds;

	peakTorque *= poundFootToNewtonMeters;
	peakTorqueRPM *= rpmToRadiansPerSecunds;
	
	peakHorsePower *= horsePowerToWatts;
	peakHorsePowerRPM *= rpmToRadiansPerSecunds;
	torqueAtPeakPower = peakHorsePower / peakHorsePowerRPM;
	
	redLineTorque *= poundFootToNewtonMeters;
	redLineTorqueRPM *= rpmToRadiansPerSecunds;

	dAssert (idleTorqueRPM > 0.0f);
	dAssert (idleTorqueRPM < peakTorqueRPM);
	dAssert (peakTorqueRPM < peakHorsePowerRPM);
	dAssert (peakHorsePowerRPM < redLineTorqueRPM);

	dAssert (idleTorque > 0.0f);
	dAssert (idleTorque < peakTorque);
	dAssert (peakTorque > torqueAtPeakPower);
	dAssert (torqueAtPeakPower > redLineTorque);
	dAssert (redLineTorque > 0.0f);

	dAssert (peakTorque * peakTorqueRPM < peakHorsePower);

	rpm[0] = 0.0f;
	rpm[1] = idleTorqueRPM;
	rpm[2] = peakTorqueRPM;
	rpm[3] = peakHorsePowerRPM;
	rpm[4] = redLineTorqueRPM;

	torque[0] = idleTorque;
	torque[1] = idleTorque;
	torque[2] = peakTorque;
	torque[3] = torqueAtPeakPower;
	torque[4] = redLineTorque;

	m_engineInternalInertia = redLineTorque * 1.25f;
	m_torqueCurve.InitalizeCurve (sizeof (rpm)/sizeof (rpm[0]), rpm, torque);

	m_engineOptimalRevPerSec = peakHorsePowerRPM;
	SetTopSpeed (vehicleSpeedKPH * 0.278f);

	m_gearBox->SetOptimalShiftLimits (peakTorqueRPM / redLineTorqueRPM, peakHorsePowerRPM/ redLineTorqueRPM);

}


dFloat CustomVehicleController::EngineComponent::GetIdleFakeInertia() const
{
	return m_fakeIdleInertia;
}
void CustomVehicleController::EngineComponent::SetIdleFakeInertia(dFloat value)
{
	m_fakeIdleInertia = value;
}


dFloat CustomVehicleController::EngineComponent::GetTopSpeed () const
{
	return m_topSpeedMPS;
}

dFloat CustomVehicleController::EngineComponent::GetSpeed () const
{
	return m_speedMPS;
}

void CustomVehicleController::EngineComponent::SetGear (int gear)
{
	m_gearBox->SetGear(gear);
}

int CustomVehicleController::EngineComponent::GetGear () const
{
	return m_gearBox->GetGear();
}

dFloat CustomVehicleController::EngineComponent::GetRPM () const
{
	return m_currentRPS * 9.55f;
}

dFloat CustomVehicleController::EngineComponent::GetTopRPM () const
{
    return m_engineOptimalRevPerSec * 9.55f;
}

void CustomVehicleController::EngineComponent::SetTopSpeed (dFloat topSpeedMPS)
{
	dAssert (topSpeedMPS >= 0.0f);
	dAssert (topSpeedMPS < 100.0f);

	TireBodyState* const tire = &m_leftTire->GetInfo();
	m_topSpeedMPS = topSpeedMPS;

	// drive train geometrical relations
	// w = v * G0 * G1 / r
	// v = w * r / (G0 * G1)
	// G0 = m_differentialGearRatio
	// G1 = m_transmissionGearRatio
	// w = engine radians per seconds
	// r = tire radius in meters
	// v = vehicle top speed in meters per seconds

	// using the top gear and the optimal engine torque for the calculations
	dFloat topGearRatio = m_gearBox->GetGearRatio(m_gearBox->GetGearCount() - 1);

	// G0 = w * r / (G1 * v)
	m_differentialGearRatio = m_engineOptimalRevPerSec * tire->m_radio / (topGearRatio * m_topSpeedMPS);

	// calculate internal tire rolling resistance, (assume no transmission power lost)
	dFloat gain = m_differentialGearRatio * topGearRatio;
	dFloat tireTopOmega = m_topSpeedMPS / tire->m_radio;

	dFloat engineTorque = m_torqueCurve.GetValue(m_engineOptimalRevPerSec);
	dFloat tireTorque = 0.5f * engineTorque * gain;
	dFloat tireRollingResistance = tireTorque / (tireTopOmega * tireTopOmega);

	for (TireList::CustomListNode* node = m_controller->m_tireList.GetFirst(); node; node = node->GetNext()) {
		TireBodyState* const tire = &node->GetInfo();
		tire->m_idleRollingResistance = tireRollingResistance;
	}
}


dFloat CustomVehicleController::EngineComponent::CaculateEngineRPS (const TireBodyState* const tire, dFloat gearGain) const
{
//	if (gearGain > 1.0e-3f) {
//	} else if (gearGain < -1.0e-3f) {
//	}
//	return m_param * m_torqueCurve.m_nodes[m_torqueCurve.m_count-1].m_param;

	dFloat rps = -gearGain * tire->m_rotatonSpeed;
	if (rps < 0.0f) {
		rps = 0.0f;
	}
	return rps;
}


void CustomVehicleController::EngineComponent::Update (dFloat timestep)
{
	TireBodyState& leftTire = m_leftTire->GetInfo();
	TireBodyState& righTire = m_righTire->GetInfo();

	m_gearBox->Update (timestep);
	int gear = m_gearBox->GetGear();
	dFloat gearGain = m_gearBox->GetGearRatio(gear) * m_differentialGearRatio;

	dFloat leftTorque = 0.0f;
	dFloat rightTorque = 0.0f;
	
	if (gear == GearBox::m_newtralGear) {
		// vehicle in neutral fake some engine inertia
		dFloat rps = m_engineOptimalRevPerSec * m_param;
		m_currentRPS = m_currentRPS + (rps - m_currentRPS) * m_fakeIdleInertia * 60.0f * timestep;

		if (m_currentRPS < m_torqueCurve.m_nodes[1].m_param) {
			m_currentRPS = m_torqueCurve.m_nodes[1].m_param;
		}
	
	} else {
		dFloat leftRPS = CaculateEngineRPS (&leftTire, gearGain);
		dFloat rightRPS = CaculateEngineRPS (&righTire, gearGain);

		if (leftRPS < m_torqueCurve.m_nodes[1].m_param * 0.5f) {
			leftRPS = m_torqueCurve.m_nodes[1].m_param * 0.5f;
		}
		if (rightRPS < m_torqueCurve.m_nodes[1].m_param * 0.5f) {
			rightRPS = m_torqueCurve.m_nodes[1].m_param * 0.5f;
		}

		dFloat rps = (leftRPS + rightRPS) * 0.5f;
		m_currentRPS = m_currentRPS + (rps - m_currentRPS) * m_fakeIdleInertia * 60.0f * timestep;

		// when adding a differential torque distribution if control by the differential, for now add 50% to each tire
		leftTorque = gearGain * m_torqueCurve.GetValue(leftRPS * dFloat(1.0f/60.0f)) * m_param * 0.5f;
		rightTorque = gearGain * m_torqueCurve.GetValue(rightRPS * dFloat(1.0f/60.0f)) * m_param * 0.5f;

		if (gear != GearBox::m_reverseGear) {
			if (leftTorque < m_torqueCurve.m_nodes[1].m_value * 0.125f) {
				leftTorque = m_torqueCurve.m_nodes[1].m_value * 0.125f;
			}
			if (rightTorque < m_torqueCurve.m_nodes[1].m_value * 0.125f) {
				rightTorque = m_torqueCurve.m_nodes[1].m_value * 0.125f;
			}
		} else {
			if (leftTorque > -m_torqueCurve.m_nodes[1].m_value * 0.125f) {
				leftTorque = -m_torqueCurve.m_nodes[1].m_value * 0.125f;
			}
			if (rightTorque > -m_torqueCurve.m_nodes[1].m_value * 0.125f) {
				rightTorque = -m_torqueCurve.m_nodes[1].m_value * 0.125f;
			}
		}

		leftTorque += leftTire.m_idleRollingResistance * leftTire.m_rotatonSpeed * leftTire.m_rotatonSpeed * dSign (leftTire.m_rotatonSpeed);
		rightTorque += righTire.m_idleRollingResistance * righTire.m_rotatonSpeed * righTire.m_rotatonSpeed * dSign (righTire.m_rotatonSpeed);

		leftTire.m_engineTorqueResistance = dAbs (gearGain * m_engineInternalInertia);
		righTire.m_engineTorqueResistance = dAbs (gearGain * m_engineInternalInertia);
	}

	leftTire.m_engineTorque = leftTorque;
	righTire.m_engineTorque = rightTorque;

/*
if (tire.m_myIndex == 2){
    static FILE* xxx;
    if (!xxx) 
    {
        fopen_s (&xxx, "torque_rpm.csv", "wt");
        fprintf (xxx, "gear, tire_torque, tire_rps, eng_rps,\n");
    }

    if ((vehicle->xxxxxxxx > (1000 + 0)) && (vehicle->xxxxxxxx < (1000 + 750))) {

        dFloat tireRps___ = (tire.m_omega - vehicle->m_bodyState.m_omega).dot (tire.m_matrix[0]);
        int gear = m_transmission->GetGear();
        fprintf (xxx, "%d, %f, %f, %f,\n", gear * 100, xxxxxxxxxxx, tireRps, tireRps___);
        fflush (xxx);
    }
}
*/


	// set the vehicle speed
	const ChassisBodyState& chassis = m_controller->m_chassisState;
	dVector front (chassis.m_matrix.RotateVector(chassis.m_localFrame[0]));
	m_speedMPS = chassis.m_veloc % front;
}


CustomVehicleController::SteeringComponent::SteeringComponent (CustomVehicleController* const controller, dFloat maxAngleInRadians)
	:Component (controller)
	,m_maxAngle (dAbs (maxAngleInRadians))
{
}

void CustomVehicleController::SteeringComponent::AddSteeringTire (TireBodyState* const tireNode, dFloat sign)
{
	TireSignPair& pair = m_steeringTires.Append()->GetInfo();

	pair.m_sign = (sign >= 0.0f) ? 1.0f : -1.0f;
	pair.m_tireNode = m_controller->m_tireList.GetNodeFromInfo (*tireNode);
}


void CustomVehicleController::SteeringComponent::Update (dFloat timestep)
{
	for (CustomList<TireSignPair>::CustomListNode* node = m_steeringTires.GetFirst(); node; node = node->GetNext()) {
		TireSignPair& pair = node->GetInfo();
		TireBodyState& tire = pair.m_tireNode->GetInfo();
		tire.m_steeringAngle = m_maxAngle * m_param * pair.m_sign;
	}
}


CustomVehicleController::BrakeComponent::BrakeComponent (CustomVehicleController* const controller, dFloat maxBrakeTorque)
	:Component (controller)
	,m_maxBrakeTorque (dAbs (maxBrakeTorque))
{
}

void CustomVehicleController::BrakeComponent::AddBrakeTire (TireBodyState* const tire)
{
	m_brakeTires.Append(m_controller->m_tireList.GetNodeFromInfo (*tire));
}


void CustomVehicleController::BrakeComponent::Update (dFloat timestep)
{
	for (CustomList<TireList::CustomListNode*>::CustomListNode* node = m_brakeTires.GetFirst(); node; node = node->GetNext()) {
		TireBodyState& tire = node->GetInfo()->GetInfo();
		tire.m_breakTorque = dMax (tire.m_breakTorque, dAbs (m_maxBrakeTorque * m_param));
	}
}



void CustomVehicleController::VehicleJoint::Init(CustomVehicleController* const controller, BodyState* const state0, BodyState* const state1)
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


void CustomVehicleController::VehicleJoint::InitPointParam (PointDerivativeParam& param, const dVector& pivot) const
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


void CustomVehicleController::VehicleJoint::CalculatePointDerivative (ParamInfo* const constraintParams, const dVector& dir, const PointDerivativeParam& param)
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


void CustomVehicleController::VehicleJoint::CalculateAngularDerivative (ParamInfo* const constraintParams, const dVector& dir, dFloat jointAngle)
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
	//constraintParams->m_jointAccel[index] = 0;
	constraintParams->m_jointAccel[index] = alphaError;
	constraintParams->m_jointLowFriction[index] = VEHICLE_MIN_FRICTION_BOUND;
	constraintParams->m_jointHighFriction[index] = VEHICLE_MAX_FRICTION_BOUND;
	constraintParams->m_count = index + 1;
}



void CustomVehicleController::VehicleJoint::AddLinearRowJacobian (ParamInfo* const constraintParams, const dVector& pivot, const dVector& dir)
{
	PointDerivativeParam pointData;
	InitPointParam (pointData, pivot);
	CalculatePointDerivative (constraintParams, dir, pointData); 
}


void CustomVehicleController::VehicleJoint::JointAccelerations (JointAccelerationDecriptor* const params)
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
			dAssert (0);
			//jacobianColElements[k].m_coordenateAccel = m_motorAcceleration[k] + jacobianColElements[k].m_deltaAccel;
		} else {
			//jacobianColElements[k].m_coordenateAccel = jacobianColElements[k].m_deltaAccel;

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


void CustomVehicleController::TireJoint::JacobianDerivative (ParamInfo* const constraintParams)
{
	// Restrict the movement on the pivot point along all two orthonormal direction
	TireBodyState* const tire = (TireBodyState*) m_state1;

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

	// get a point along the pin axis at some reasonable large distance from the pivot
	CalculateAngularDerivative (constraintParams, tire->m_matrix[1], 0.0f);
	CalculateAngularDerivative (constraintParams, tire->m_matrix[2], 0.0f);

	// check if the brakes are applied
	dFloat breakResistance = dMax(tire->m_breakTorque, tire->m_engineTorqueResistance);
	if (breakResistance > 1.0e-3f) {
		int index = constraintParams->m_count;
		CalculateAngularDerivative (constraintParams, tire->m_matrix[0], 0.0f);                    
		constraintParams->m_jointLowFriction[index] = -breakResistance;
		constraintParams->m_jointHighFriction[index] = breakResistance;
	}

	// clear the input variable after there are res
	tire->m_breakTorque = 0.0f;
	tire->m_engineTorqueResistance = 0.0f;
}

void CustomVehicleController::TireJoint::UpdateSolverForces (const JacobianPair* const jacobians) const
{
	TireBodyState* const tire = (TireBodyState*) m_state1;

	// get tire lateral force
	int index = tire->m_chassisJoint.m_start;
	tire->m_lateralForce = jacobians[index].m_jacobian_IM0.m_linear.Scale(tire->m_chassisJoint.m_jointFeebackForce[0]); 

	// get tire longitudinal force
	tire->m_longitidinalForce = jacobians[index + 1].m_jacobian_IM0.m_linear.Scale(tire->m_chassisJoint.m_jointFeebackForce[1]); 
}

void CustomVehicleController::ContactJoint::JacobianDerivative (ParamInfo* const constraintParams)
{
	TireBodyState* const tire = (TireBodyState*) m_state1;
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
/*
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
*/
			}
		}
	}
}

void CustomVehicleController::ContactJoint::UpdateSolverForces (const JacobianPair* const jacobians) const
{
	TireBodyState* const tire = (TireBodyState*) m_state1;
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


void CustomVehicleController::ContactJoint::JointAccelerations (JointAccelerationDecriptor* const accelParam)
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


CustomVehicleController::BodyState::BodyState()
	:m_matrix(GetIdentityMatrix())
	,m_localFrame(GetZeroMatrix())
	,m_inertia(GetZeroMatrix())
	,m_invInertia(GetZeroMatrix())
	,m_localInertia (0.0f, 0.0f, 0.0f, 0.0f)
	,m_localInvInertia(0.0f, 0.0f, 0.0f, 0.0f)
	,m_veloc(0.0f, 0.0f, 0.0f, 0.0f)
	,m_omega(0.0f, 0.0f, 0.0f, 0.0f)
	,m_externalForce(0.0f, 0.0f, 0.0f, 0.0f)
	,m_externalTorque(0.0f, 0.0f, 0.0f, 0.0f)
	,m_globalCentreOfMass(0.0f, 0.0f, 0.0f, 0.0f)
	,m_mass(0.0f)
	,m_invMass(0.0f)
	,m_myIndex(0)
	,m_controller(NULL)
{
}

void CustomVehicleController::BodyState::Init(CustomVehicleController* const controller)
{
	m_controller = controller;
}

void CustomVehicleController::BodyState::UpdateInertia()
{
	dMatrix tmpMatrix (GetZeroMatrix());

	tmpMatrix[0] = m_localInertia.CompProduct (dVector (m_matrix[0][0], m_matrix[1][0], m_matrix[2][0], 0.0f));
	tmpMatrix[1] = m_localInertia.CompProduct (dVector (m_matrix[0][1], m_matrix[1][1], m_matrix[2][1], 0.0f));
	tmpMatrix[2] = m_localInertia.CompProduct (dVector (m_matrix[0][2], m_matrix[1][2], m_matrix[2][2], 0.0f));
	m_inertia = tmpMatrix * m_matrix;

	tmpMatrix[0] = m_localInvInertia.CompProduct (dVector (m_matrix[0][0], m_matrix[1][0], m_matrix[2][0], 0.0f));
	tmpMatrix[1] = m_localInvInertia.CompProduct (dVector (m_matrix[0][1], m_matrix[1][1], m_matrix[2][1], 0.0f));
	tmpMatrix[2] = m_localInvInertia.CompProduct (dVector (m_matrix[0][2], m_matrix[1][2], m_matrix[2][2], 0.0f));
	m_invInertia = tmpMatrix * m_matrix;
}

void CustomVehicleController::BodyState::IntegrateForce (dFloat timestep, const dVector& force, const dVector& torque)
{
	dVector accel (force.Scale (m_invMass));
	dVector alpha (m_invInertia.RotateVector(torque));
	m_veloc += accel.Scale (timestep);
	m_omega += alpha.Scale (timestep);
}

void CustomVehicleController::BodyState::CalculateAverageAcceleration (dFloat invTimestep, const dVector& veloc, const dVector& omega)
{
    dAssert (0);
}

void CustomVehicleController::TireBodyState::Init (CustomVehicleController* const controller, const TireCreationInfo& tireInfo)
{
	BodyState::Init (controller);
	NewtonBody* const body = m_controller->GetBody();

	const dMatrix& vehicleFrame = m_controller->m_chassisState.m_localFrame;

	// build a normalized size collision shape and scale to math the tire size, make it is also transparent to collision  
	NewtonCollisionSetScale (m_controller->m_tireCastShape, tireInfo.m_width, tireInfo.m_radio, tireInfo.m_radio);
	NewtonCollisionSetCollisonMode (m_controller->m_tireCastShape, 0);

	// calculate the location of the tie matrix
	m_localFrame = vehicleFrame * dYawMatrix(-3.141592f * 0.5f);
	m_localFrame.m_posit = tireInfo.m_location + m_localFrame.m_up.Scale (tireInfo.m_suspesionlenght);
	m_localFrame.m_posit.m_w = 1.0f;
	NewtonCollisionSetMatrix (m_controller->m_tireCastShape, &m_localFrame[0][0]);

	// now add a copy of this collision shape to the vehicle collision  
	NewtonCollision* const vehShape = NewtonBodyGetCollision(body);
	NewtonCompoundCollisionBeginAddRemove(vehShape);
	void* const tireShapeNode = NewtonCompoundCollisionAddSubCollision (vehShape, m_controller->m_tireCastShape);
	NewtonCompoundCollisionEndAddRemove (vehShape);	

	// restore the cast shape transform to identity
	dMatrix identMatrix (GetIdentityMatrix());
	NewtonCollisionSetCollisonMode (m_controller->m_tireCastShape, 1);
	NewtonCollisionSetMatrix (m_controller->m_tireCastShape, &identMatrix[0][0]);

	// get the collision shape
	m_shape = NewtonCompoundCollisionGetCollisionFromNode (vehShape, tireShapeNode);

	// initialize all constants
	m_userData = tireInfo.m_userData;
	m_mass = tireInfo.m_mass; 
	m_invMass = 1.0f / m_mass;
	m_radio = tireInfo.m_radio;
	m_width = tireInfo.m_width;
	m_localInertia[0] = m_mass * (0.50f * m_radio * m_radio);
	m_localInertia[1] = m_mass * (0.25f * m_radio * m_radio + (1.0f / 12.0f) * m_width * m_width);
	m_localInertia[2] = m_localInertia[1];
	m_localInertia[3] = 0.0f;
	m_localInvInertia[0] = 1.0f / m_localInertia[0];
	m_localInvInertia[1] = 1.0f / m_localInertia[1];
	m_localInvInertia[2] = 1.0f / m_localInertia[2];
	m_localInvInertia[3] = 0.0f;

	m_dampingRatio = tireInfo.m_dampingRatio;
	m_springStrength = tireInfo.m_springStrength;
	m_suspensionlenght = tireInfo.m_suspesionlenght;
	
	// initialize all local variables to default values
	m_breakTorque = 0.0f;
	m_engineTorque = 0.0f;
	m_rotatonSpeed = 0.0f;
	m_rotationAngle = 0.0f;
	m_steeringAngle = 0.0f;
	m_adhesionCoefficient = 2.0f;
	m_idleRollingResistance = 0.0f;
	m_engineTorqueResistance = 0.0f;
	m_tireLoad = dVector(0.0f, 0.0f, 0.0f, 0.0f);
	m_lateralForce = dVector(0.0f, 0.0f, 0.0f, 0.0f);
	m_longitidinalForce = dVector(0.0f, 0.0f, 0.0f, 0.0f);

	m_speed = 0.0f;
	m_posit = m_suspensionlenght;
	m_matrix = CalculateSteeringMatrix ();

	UpdateInertia();

	// initialize the joints that connect tghsi tire to other vehicle componets;
	m_chassisJoint.Init(m_controller, &m_controller->m_chassisState, this);
	m_contactJoint.Init(m_controller, &m_controller->m_staticWorld, this);
}


dFloat CustomVehicleController::TireBodyState::GetAdhesionCoefficient() const
{
	return m_adhesionCoefficient * 0.5f;
}
void CustomVehicleController::TireBodyState::SetAdhesionCoefficient(dFloat Coefficient)
{
	m_adhesionCoefficient = 2.0f * dClamp (Coefficient, dFloat(0.0f), dFloat(1.0f));
}



dMatrix CustomVehicleController::TireBodyState::CalculateSuspensionMatrix () const
{
	dMatrix matrix (m_localFrame);
	matrix.m_posit -= m_localFrame.m_up.Scale (m_posit);
	return matrix;
}

dMatrix CustomVehicleController::TireBodyState::CalculateSteeringMatrix () const
{
	return dYawMatrix(m_steeringAngle) * CalculateSuspensionMatrix ();
}

dMatrix CustomVehicleController::TireBodyState::CalculateMatrix () const
{
	return dPitchMatrix(m_rotationAngle) * CalculateSteeringMatrix ();
}

void CustomVehicleController::TireBodyState::Collide (CustomControllerConvexCastPreFilter& filter, dFloat timestepInv)
{
	NewtonBody* const body = m_controller->GetBody();
	NewtonWorld* const world = NewtonBodyGetWorld(body);
	const dMatrix& controllerMatrix = m_controller->m_chassisState.m_matrix;

	dFloat posit0 = m_posit;

	m_posit = 0.0f;
	m_speed = 0.0f;
	dMatrix localMatrix (CustomVehicleController::TireBodyState::CalculateSteeringMatrix ());
	m_posit = m_suspensionlenght;

	dFloat hitParam;
	dMatrix tireMatrix (localMatrix * controllerMatrix);
	dVector rayDestination (tireMatrix.TransformVector(localMatrix.m_up.Scale(-m_suspensionlenght)));   

	m_contactJoint.m_contactCount = 0;
	NewtonCollisionSetScale (m_controller->m_tireCastShape, m_width, m_radio, m_radio);
	m_contactJoint.m_contactCount = NewtonWorldConvexCast (world, &tireMatrix[0][0], &rayDestination[0], m_controller->m_tireCastShape, &hitParam, &filter, CustomControllerConvexCastPreFilter::Prefilter, m_contactJoint.m_contacts, sizeof (m_contactJoint.m_contacts) / sizeof (m_contactJoint.m_contacts[0]), 0);
	if (m_contactJoint.m_contactCount) {
		// this tire hit something generate contact point and normals, 
		// do not forget to filter bad contacts
		m_posit = hitParam * m_suspensionlenght;
		m_speed = (posit0 - m_posit) * timestepInv;
	}
	//dTrace (("%f ", m_posit));
}


void CustomVehicleController::TireBodyState::UpdateTransform()
{
	//dMatrix localMatrix (CustomVehicleController::TireBodyState::CalculateSteeringMatrix ());
	dMatrix localMatrix (CustomVehicleController::TireBodyState::CalculateMatrix());
	NewtonCollisionSetMatrix(m_shape, &localMatrix[0][0]);	
}


void CustomVehicleController::ChassisBodyState::Init (CustomVehicleController* const controller, const dMatrix& localframe)
{
    BodyState::Init (controller);
    NewtonBody* const body = m_controller->GetBody();

    m_localFrame = localframe;
    m_localFrame[3] = m_com + m_comOffset;
    m_localFrame[3][3] = 1.0f;

    NewtonBodySetCentreOfMass(body, &m_localFrame[3][0]);

    NewtonBodyGetMatrix(body, &m_matrix[0][0]);
    NewtonBodyGetMassMatrix(body, &m_mass, &m_localInertia[0], &m_localInertia[1], &m_localInertia[2]);

    NewtonBodyGetOmega(body, &m_omega[0]);
    NewtonBodyGetVelocity(body, &m_veloc[0]);

    m_invMass = 1.0f / m_mass;
    m_localInvInertia[0] = 1.0f / m_localInertia[0];
    m_localInvInertia[1] = 1.0f / m_localInertia[1];
    m_localInvInertia[2] = 1.0f / m_localInertia[2];

    UpdateInertia();
}



void CustomVehicleController::ChassisBodyState::UpdateDynamicInputs()
{
	NewtonBody* const body = m_controller->GetBody();

	NewtonBodyGetMatrix (body, &m_matrix[0][0]);
	NewtonBodyGetVelocity (body, &m_veloc[0]);
	NewtonBodyGetOmega (body, &m_omega[0]);

	NewtonBodyGetForceAcc(body, &m_externalForce[0]);
	NewtonBodyGetTorqueAcc(body, &m_externalTorque[0]);

	m_globalCentreOfMass = m_matrix.TransformVector(m_localFrame.m_posit);

	UpdateInertia();
}

void CustomVehicleController::ChassisBodyState::CalculateAverageAcceleration (dFloat invTimestep, const dVector& veloc, const dVector& omega)
{
    dVector accel = (m_veloc - veloc).Scale(invTimestep);
    dVector alpha = (m_omega - omega).Scale(invTimestep);

    m_externalForce = accel.Scale(m_mass);
    alpha = m_matrix.UnrotateVector(alpha);
    m_externalTorque = m_matrix.RotateVector(alpha.CompProduct(m_localInertia));
}


void CustomVehicleController::TireBodyState::UpdateDynamicInputs(dFloat timestep)
{
	ChassisBodyState& chassis = m_controller->m_chassisState;

	m_matrix = CalculateSteeringMatrix() * chassis.m_matrix;
	m_globalCentreOfMass = m_matrix.m_posit;
	UpdateInertia();

	// get the velocity state for this tire
	dVector com (m_matrix.m_posit - chassis.m_globalCentreOfMass);
	m_omega = chassis.m_omega + m_matrix[0].Scale (m_rotatonSpeed);
	m_veloc = chassis.m_veloc + chassis.m_omega * com + m_matrix[1].Scale (m_speed);

	// set the initial force on this tire
	m_externalForce = chassis.m_gravity.Scale (m_mass);

	// apply the engine torque on this tire
	dVector torque (m_matrix[0].Scale (m_engineTorque));
	chassis.m_externalTorque += torque;
	m_externalTorque = torque.Scale(-1.0f);

	// calculate force an torque generate by the suspension
	m_tireLoad = dVector(0.0f, 0.0f, 0.0f, 0.0f);
	m_lateralForce = dVector(0.0f, 0.0f, 0.0f, 0.0f);
	m_longitidinalForce = dVector(0.0f, 0.0f, 0.0f, 0.0f);
	if (m_contactJoint.m_contactCount) {
		dFloat distance = m_suspensionlenght - m_posit;
		dAssert (distance >= 0.0f);
		dAssert (distance <= m_suspensionlenght);
		if (distance <= 1.0e-3f) {
			// now calculate the tire load at the contact point, tire suspension distance also consider hard limit.
			//dAssert (0);
		}

		dFloat load = - NewtonCalculateSpringDamperAcceleration (timestep, m_springStrength, distance, m_dampingRatio, m_speed);
		if (load < 0.0f) {
			// tire can not pull the car, this is a symptom of bad suspension spring or damper
			load = 0.0f;
		} 

		// calculate the tire load 
		m_tireLoad = m_matrix[1].Scale(load * m_mass);

		// calculate tire force and torque spring and damper apply to the chassis
		//dVector force (m_matrix[1].Scale(m_tireLoad));
		dVector torque (com * m_tireLoad);
		chassis.m_externalForce += m_tireLoad;
		chassis.m_externalTorque += torque;

		// the spring apply the same force in the opposite direction to the tire
		m_externalForce -= m_tireLoad;
	}
}

void CustomVehicleController::TireBodyState::IntegrateForce (dFloat timestep, const dVector& force, const dVector& torque)
{
	BodyState::IntegrateForce (timestep, force, torque);
}

void CustomVehicleController::TireBodyState::CalculateAverageAcceleration (dFloat invTimestep, const dVector& veloc, const dVector& omega)
{
    dVector accel = (m_veloc - veloc).Scale(invTimestep);
    dVector alpha = (m_omega - omega).Scale(invTimestep);

    m_externalForce = accel.Scale(m_mass);
    alpha = m_matrix.UnrotateVector(alpha);
    m_externalTorque = m_matrix.RotateVector(alpha.CompProduct(m_localInertia));

/*
    dAssert (m_tireConstraint.m_state0 != this);
    dAssert (m_tireConstraint.m_state1 == this);
    const AppVehicleBodyState* const chassis = m_tireConstraint.m_state0;

    dVector upPin (chassis->m_matrix[1]);
    dVector lateralPin (m_matrix[0]);
    dVector longitudinalPin (crossProduct (lateralPin, upPin));

    const dFloat speedFactor = 2.0f;
    dFloat tireSpeed = m_veloc.dot (longitudinalPin);
    dFloat maxSpeed = dSqrt(m_veloc.dot(m_veloc)) * speedFactor + 2.0f;

    dFloat tireRps = (m_omega - chassis->m_omega).dot (lateralPin);
    if ((m_radius * dgAbsf(tireRps)) > maxSpeed) 
    {
        if ((tireRps > 0.0f) && (tireSpeed > 0.0f)) 
        {
            m_omega = chassis->m_omega + lateralPin * ((tireSpeed * speedFactor + 2.0f) / m_radius);
        } 
        else if ((tireRps < 0.0f) && (tireSpeed < 0.0f)) 
        {
            m_omega = chassis->m_omega + lateralPin * ((tireSpeed * speedFactor - 2.0f) / m_radius);
        } 
        else 
        {
            m_omega = chassis->m_omega + lateralPin * (dSign(tireRps) * dAbs (tireSpeed) * speedFactor / m_radius);
        }
    }

    tireRps = (m_omega - chassis->m_omega).dot (lateralPin);
    const dFloat maxRpsAccel = 4.0f;
    if ((tireRps - m_prevRps) > maxRpsAccel) 
    {
        tireRps = m_prevRps + maxRpsAccel;
        m_omega = chassis->m_omega + lateralPin * tireRps;
    } 
    else if ((tireRps - m_prevRps) < -maxRpsAccel) 
    {
        tireRps = m_prevRps - maxRpsAccel;
        m_omega = chassis->m_omega + lateralPin * tireRps;
    }

    m_prevRps = tireRps;
    m_averageRps = tireRps;
    const int count = sizeof (m_averageRpsToople)/sizeof (m_averageRpsToople[0]);
    for (int i = 1; i < count; i ++) 
    {
        m_averageRps += m_averageRpsToople[i];
        m_averageRpsToople[i - 1] = m_averageRpsToople[i];
    }
    m_averageRpsToople[count - 1] = tireRps;
    m_averageRps *= dFloat (1.0f / count);
*/

/*
static int xxxxxxx;
static FILE* xxx;
if (!xxx) 
{
    fopen_s (&xxx, "tire_rpm.csv", "wt");
    fprintf (xxx, "gear, rps, torque\n");
}

int gear = m_controller->GetEngine()->GetGear();
//if ((gear == 2) && (m_myIndex == 4)) {
if ((gear > 1) && (m_myIndex == 4)) {
    dFloat tireRps = (m_omega - m_controller->m_chassisState.m_omega) % m_matrix[0];
    fprintf (xxx, "%d, %f, %f,\n", gear, -tireRps, m_engineTorque);
    fflush (xxx);
    dTrace (("%d, %d, %f, %f,\n", xxxxxxx, gear, -tireRps, m_engineTorque));
    xxxxxxx ++;
}
*/
}

CustomVehicleControllerManager::CustomVehicleControllerManager(NewtonWorld* const world)
	:CustomControllerManager<CustomVehicleController> (world, VEHICLE_PLUGIN_NAME)
{
}

CustomVehicleControllerManager::~CustomVehicleControllerManager()
{
}



CustomVehicleController* CustomVehicleControllerManager::CreateVehicle (NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, const dVector& gravityVector)
{
	CustomVehicleController* const controller = CreateController();
	controller->Init(chassisShape, vehicleFrame, mass, gravityVector);
	return controller;
}

void CustomVehicleControllerManager::DestroyController (CustomVehicleController* const controller)
{
	controller->Cleanup();
	CustomControllerManager<CustomVehicleController>::DestroyController(controller);
}


void CustomVehicleController::Init (NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, const dVector& gravityVector)
{
	CustomVehicleControllerManager* const manager = (CustomVehicleControllerManager*) GetManager(); 
	NewtonWorld* const world = manager->GetWorld(); 

	// create a compound collision 	
	NewtonCollision* const vehShape = NewtonCreateCompoundCollision(world, 0);
	NewtonCompoundCollisionBeginAddRemove(vehShape);

	// if the shape is a compound collision ass all the pieces one at a time
	int shapeType = NewtonCollisionGetType (chassisShape);
	if (shapeType == SERIALIZE_ID_COMPOUND) {
		dAssert (0);
	} else {
		dAssert ((shapeType == SERIALIZE_ID_CONVEXHULL) || (shapeType == SERIALIZE_ID_BOX));
		NewtonCompoundCollisionAddSubCollision (vehShape, chassisShape);
	}
	NewtonCompoundCollisionEndAddRemove (vehShape);	

	// create the rigid body for this vehicle
	dMatrix locationMatrix (GetIdentityMatrix());
	m_body = NewtonCreateDynamicBody(world, vehShape, &locationMatrix[0][0]);

	// set vehicle mass, inertia and center of mass
	NewtonBodySetMassProperties (m_body, mass, vehShape);

	// set linear and angular drag to zero
	dVector drag(0.0f, 0.0f, 0.0f, 0.0f);
	NewtonBodySetLinearDamping(m_body, 0);
	NewtonBodySetAngularDamping(m_body, &drag[0]);

	// destroy the collision help shape
	NewtonDestroyCollision (vehShape);

	// initialize vehicle internal components
	NewtonBodyGetCentreOfMass (m_body, &m_chassisState.m_com[0]);

	m_chassisState.m_gravity = gravityVector;
    m_chassisState.Init(this, vehicleFrame);

	m_stateList.Append(&m_staticWorld);
	m_stateList.Append(&m_chassisState);

	// create the normalized size tire shape
	m_tireCastShape = NewtonCreateChamferCylinder(world, 0.5f, 1.0f, 0, NULL);

	// initialize all components to empty
	m_engine = NULL;
	m_steering = NULL;
	m_brakes = NULL;
	m_handBrakes = NULL;


	// set default tire model, Create a simplified normalized Tire Curve
	// we will use a simple piece wise curve from Pacejkas tire model, 
	// http://en.wikipedia.org/wiki/File:Magic_Formula_Curve.png
	//an application can use advance curves like platting the complete Pacejkas empirical equation
	//dFloat slips[] = {0.0f, 0.1f, 0.2f, 1.0f};
	//dFloat normalizedLongitudinalForce[] = {0.0f, 0.8f, 1.0f, 1.0f};
	//SetLongitudinalSlipRatio (sizeof (slips) / sizeof (slips[0]), slips, normalizedLongitudinalForce);
	SetLongitudinalSlipRatio (0.2f);
	SetLateralSlipAngle(3.0f);
}


void CustomVehicleController::Cleanup()
{
	SetBrakes(NULL);
	SetEngine(NULL);
	SetSteering(NULL);
	SetHandBrakes(NULL);
	NewtonDestroyCollision(m_tireCastShape);
}


void CustomVehicleController::SetLateralSlipAngle(dFloat maxLongitudinalSlipAngleIndDegrees)
{
	dClamp (maxLongitudinalSlipAngleIndDegrees, dFloat(1.0f), dFloat(30.0f));
	dFloat slips[] = {0.0f, maxLongitudinalSlipAngleIndDegrees * 3.141592f / 180.0f, 90.0f * 3.141592f / 180.0f};
	dFloat force[] = {0.0f, 1.0f, dSqrt (1.0f - VEHICLE_SIDESLEP_NORMALIZED_FRICTION_AT_MAX_SLIP_ANGLE * VEHICLE_SIDESLEP_NORMALIZED_FRICTION_AT_MAX_SLIP_ANGLE)};
	m_tireLateralSlipAngle.InitalizeCurve(sizeof (slips) / sizeof (slips[0]), slips, force);
}
                                  
void CustomVehicleController::SetLongitudinalSlipRatio(dFloat maxLongitudinalSlipRatio)
{
	dClamp(maxLongitudinalSlipRatio, dFloat(0.01f), dFloat(0.9f));

	dFloat slips[] = {0.0f, maxLongitudinalSlipRatio, 1.0f};
	dFloat force[] = {0.0f, 1.0f, dSqrt (1.0f - VEHICLE_SIDESLEP_NORMALIZED_FRICTION_AT_MAX_SLIP_ANGLE * VEHICLE_SIDESLEP_NORMALIZED_FRICTION_AT_MAX_SLIP_ANGLE)};
	m_tireLongitidialSlipRatio.InitalizeCurve(sizeof (slips) / sizeof (slips[0]), slips, force);
}


void CustomVehicleController::SetCenterOfGravity(const dVector& comRelativeToGeomtriCenter)
{
	dMatrix localFrame (m_chassisState.m_localFrame);
	m_chassisState.m_comOffset = comRelativeToGeomtriCenter;
	m_chassisState.Init(this, localFrame);
}

const CustomVehicleController::ChassisBodyState& CustomVehicleController::GetChassisState () const
{
	return m_chassisState;
}


CustomVehicleController::EngineComponent* CustomVehicleController::GetEngine() const
{
	return m_engine;
}

CustomVehicleController::SteeringComponent* CustomVehicleController::GetSteering() const
{
	return m_steering;
}

CustomVehicleController::BrakeComponent* CustomVehicleController::GetBrakes() const
{
	return m_brakes;
}

CustomVehicleController::BrakeComponent* CustomVehicleController::GetHandBrakes() const
{
	return m_handBrakes;
}


void CustomVehicleController::SetEngine(EngineComponent* const engine)
{
	if (m_engine) {
		delete m_engine;
	}
	m_engine = engine;
}

void CustomVehicleController::SetSteering(SteeringComponent* const steering)
{
	if (m_steering) {
		delete m_steering;
	}
	m_steering = steering;
}


void CustomVehicleController::SetBrakes(BrakeComponent* const brakes)
{
	if (m_brakes) {
		delete m_brakes;
	}
	m_brakes = brakes;
}

void CustomVehicleController::SetHandBrakes(BrakeComponent* const brakes)
{
	if (m_handBrakes) {
		delete m_handBrakes;
	}
	m_handBrakes = brakes;
}



CustomVehicleController::TireBodyState* CustomVehicleController::AddTire (const TireCreationInfo& tireInfo)
{
	TireList::CustomListNode* const tireNode = m_tireList.Append();
	TireBodyState& tire = tireNode->GetInfo();
	tire.Init(this, tireInfo);

	m_stateList.Append(&tire);
	return &tireNode->GetInfo();
}

CustomVehicleController::TireBodyState* CustomVehicleController::GetFirstTire () const
{
	return m_tireList.GetFirst() ? &m_tireList.GetFirst()->GetInfo() : NULL;
}

CustomVehicleController::TireBodyState* CustomVehicleController::GetNextTire (TireBodyState* const tire) const
{
	TireList::CustomListNode* const tireNode = m_tireList.GetNodeFromInfo(*tire);
	return tireNode->GetNext() ? &tireNode->GetNext()->GetInfo() : NULL;
}


void* CustomVehicleController::GetUserData (TireBodyState* const tire) const
{
	return tire->m_userData;
}

dMatrix CustomVehicleController::GetTireLocalMatrix (TireBodyState* const tire) const
{
	return tire->CalculateMatrix ();
}

dMatrix CustomVehicleController::GetTireGlobalMatrix (TireBodyState* const tire) const
{
	dMatrix matrix;
	NewtonBodyGetMatrix(GetBody(), &matrix[0][0]);
	return GetTireLocalMatrix (tire) * matrix;
}


void CustomVehicleController::UpdateTireTransforms ()
{
	for (TireList::CustomListNode* node = m_tireList.GetFirst(); node; node = node->GetNext()) {
		TireBodyState* const tire = &node->GetInfo();
		tire->UpdateTransform();
	}
}


void CustomVehicleController::PostUpdate(dFloat timestep, int threadIndex)
{
	NewtonBody* const body = GetBody();
	NewtonBodyGetMatrix(body, &m_chassisState.m_matrix[0][0]);
	for (TireList::CustomListNode* node = m_tireList.GetFirst(); node; node = node->GetNext()) {
		TireBodyState* const tire = &node->GetInfo();
		tire->UpdateTransform();
	}
}



int CustomVehicleController::GetActiveJoints(VehicleJoint** const jointArray)
{
	int jointCount = 0;

	// add the joints that connect tire to chassis
	for (TireList::CustomListNode* node = m_tireList.GetFirst(); node; node = node->GetNext()) {
		TireBodyState* const tire = &node->GetInfo();
		jointArray[jointCount] = &tire->m_chassisJoint;
		jointCount ++;
		dAssert (jointCount < VEHICLE_CONTROLLER_MAX_JOINTS);
	}

	// add all contact joints if any
	for (TireList::CustomListNode* node = m_tireList.GetFirst(); node; node = node->GetNext()) {
		TireBodyState* const tire = &node->GetInfo();
		if (tire->m_contactJoint.m_contactCount) {
			jointArray[jointCount] = &tire->m_contactJoint;
			jointCount ++;
			dAssert (jointCount < VEHICLE_CONTROLLER_MAX_JOINTS);
		}
	}

//	for (int i = 0; i < m_angularJointCount; i ++) {
//		constraintArray[jointCount] = &m_angularVelocityLinks[i];
//		jointCount ++;
//	}
//	for (int i = 0; i < m_trackSteeringCount; i ++) {
//		constraintArray[jointCount] = &m_trackSteering[i];
//		jointCount ++;
//	}

//	if (m_dryFriction.m_maxForce > 1.0f) {
//		constraintArray[jointCount] = &m_dryFriction;
//		jointCount ++;
//	}
	return jointCount;
}

int CustomVehicleController::BuildJacobianMatrix (int jointCount, VehicleJoint** const jointArray, dFloat timestep, VehicleJoint::JacobianPair* const jacobianArray, VehicleJoint::JacobianColum* const jacobianColumnArray)
{
	int rowCount = 0;

	VehicleJoint::ParamInfo constraintParams;
	constraintParams.m_timestep = timestep;
	constraintParams.m_timestepInv = 1.0f / timestep;

	// calculate Jacobian derivative for each active joint	
	for (int j = 0; j < jointCount; j ++) {
		VehicleJoint* const joint = jointArray[j];
		constraintParams.m_count = 0;
		joint->JacobianDerivative (&constraintParams); 

		int dofCount = constraintParams.m_count;
		joint->m_count = dofCount;
		joint->m_start = rowCount;

		// copy the rows and columns from the Jacobian derivative descriptor
		for (int i = 0; i < dofCount; i ++) {
			VehicleJoint::JacobianColum* const col = &jacobianColumnArray[rowCount];
			jacobianArray[rowCount] = constraintParams.m_jacobians[i]; 
			col->m_diagDamp = 1.0f;
			col->m_coordenateAccel = constraintParams.m_jointAccel[i];
			col->m_jointLowFriction = constraintParams.m_jointLowFriction[i];
			col->m_jointHighFriction = constraintParams.m_jointHighFriction[i];

			rowCount ++;
			dAssert (rowCount < VEHICLE_CONTROLLER_MAX_JACOBIANS_PAIRS);
		}


		// complete the derivative matrix for this joint
		int index = joint->m_start;
		BodyState* const state0 = joint->m_state0;
		BodyState* const state1 = joint->m_state1;

		const dMatrix& invInertia0 = state0->m_invInertia;
		const dMatrix& invInertia1 = state1->m_invInertia;

		dFloat invMass0 = state0->m_invMass;
		dFloat invMass1 = state1->m_invMass;
		dFloat weight = 0.9f;
		for (int i = 0; i < dofCount; i ++) {
			VehicleJoint::JacobianPair* const row = &jacobianArray[index];
			VehicleJoint::JacobianColum* const col = &jacobianColumnArray[index];

			dVector JMinvIM0linear (row->m_jacobian_IM0.m_linear.Scale (invMass0));
			dVector JMinvIM1linear (row->m_jacobian_IM1.m_linear.Scale (invMass1));
			dVector JMinvIM0angular = invInertia0.UnrotateVector(row->m_jacobian_IM0.m_angular);
			dVector JMinvIM1angular = invInertia1.UnrotateVector(row->m_jacobian_IM1.m_angular);

			dVector tmpDiag (JMinvIM0linear.CompProduct(row->m_jacobian_IM0.m_linear) + 
							 JMinvIM0angular.CompProduct(row->m_jacobian_IM0.m_angular) +
							 JMinvIM1linear.CompProduct(row->m_jacobian_IM1.m_linear) + 
							 JMinvIM1angular.CompProduct(row->m_jacobian_IM1.m_angular));

			dVector tmpAccel (JMinvIM0linear.CompProduct (state0->m_externalForce) + 
							  JMinvIM0angular.CompProduct(state0->m_externalTorque) + 
							  JMinvIM1linear.CompProduct (state1->m_externalForce) + 
							  JMinvIM1angular.CompProduct(state1->m_externalTorque));

			dFloat extenalAcceleration = -(tmpAccel[0] + tmpAccel[1] + tmpAccel[2]);

			col->m_deltaAccel = extenalAcceleration;
			col->m_coordenateAccel += extenalAcceleration;

			col->m_force = joint->m_jointFeebackForce[i] * weight;

			dFloat stiffness = VEHICLE_PSD_DAMP_TOL * col->m_diagDamp;
			dFloat diag = (tmpDiag[0] + tmpDiag[1] + tmpDiag[2]);
			dAssert (diag > dFloat (0.0f));
			col->m_diagDamp = diag * stiffness;

			diag *= (dFloat(1.0f) + stiffness);
			col->m_invDJMinvJt = dFloat(1.0f) / diag;
			index ++;
		}
	}

	return rowCount;
}

void CustomVehicleController::CalculateReactionsForces (int jointCount, VehicleJoint** const jointArray, dFloat timestepSrc, VehicleJoint::JacobianPair* const jacobianArray, VehicleJoint::JacobianColum* const jacobianColumnArray)
{
	VehicleJoint::Jacobian stateVeloc[VEHICLE_CONTROLLER_MAX_JOINTS / 2];
	VehicleJoint::Jacobian internalForces[VEHICLE_CONTROLLER_MAX_JOINTS / 2];

	int stateIndex = 0;
	dVector zero(dFloat (0.0f), dFloat (0.0f), dFloat (0.0f), dFloat (0.0f));
	for (CustomList<BodyState*>::CustomListNode* stateNode = m_stateList.GetFirst(); stateNode; stateNode = stateNode->GetNext()) {
		BodyState* const state = stateNode->GetInfo();
		stateVeloc[stateIndex].m_linear = state->m_veloc;
		stateVeloc[stateIndex].m_angular = state->m_omega;

		internalForces[stateIndex].m_linear = zero;
		internalForces[stateIndex].m_angular = zero;

		state->m_myIndex = stateIndex;
		stateIndex ++;
		dAssert (stateIndex < int (sizeof (stateVeloc)/sizeof (stateVeloc[0])));
	}

	for (int i = 0; i < jointCount; i ++) {
		VehicleJoint::Jacobian y0;
		VehicleJoint::Jacobian y1;
		y0.m_linear = zero;
		y0.m_angular = zero;
		y1.m_linear = zero;
		y1.m_angular = zero;
		VehicleJoint* const constraint = jointArray[i];
		int first = constraint->m_start;
		int count = constraint->m_count;
		for (int j = 0; j < count; j ++) { 
			VehicleJoint::JacobianPair* const row = &jacobianArray[j + first];
			const VehicleJoint::JacobianColum* const col = &jacobianColumnArray[j + first];
			dFloat val = col->m_force; 
			y0.m_linear += row->m_jacobian_IM0.m_linear.Scale(val);
			y0.m_angular += row->m_jacobian_IM0.m_angular.Scale(val);
			y1.m_linear += row->m_jacobian_IM1.m_linear.Scale(val);
			y1.m_angular += row->m_jacobian_IM1.m_angular.Scale(val);
		}
		int m0 = constraint->m_state0->m_myIndex;
		int m1 = constraint->m_state1->m_myIndex;
		internalForces[m0].m_linear += y0.m_linear;
		internalForces[m0].m_angular += y0.m_angular;
		internalForces[m1].m_linear += y1.m_linear;
		internalForces[m1].m_angular += y1.m_angular;
	}


	dFloat invTimestepSrc = dFloat (1.0f) / timestepSrc;
	dFloat invStep = dFloat (0.25f);
	dFloat timestep = timestepSrc * invStep;
	dFloat invTimestep = invTimestepSrc * dFloat (4.0f);

	int maxPasses = 5;
	dFloat firstPassCoef = dFloat (0.0f);
	dFloat maxAccNorm = dFloat (1.0e-2f);

	for (int step = 0; step < 4; step ++) {
		VehicleJoint::JointAccelerationDecriptor joindDesc;
		joindDesc.m_timeStep = timestep;
		joindDesc.m_invTimeStep = invTimestep;
		joindDesc.m_firstPassCoefFlag = firstPassCoef;

		for (int curJoint = 0; curJoint < jointCount; curJoint ++) {
			VehicleJoint* const constraint = jointArray[curJoint];
			joindDesc.m_rowsCount = constraint->m_count;
			joindDesc.m_rowMatrix = &jacobianArray[constraint->m_start];
			joindDesc.m_colMatrix = &jacobianColumnArray[constraint->m_start];
			constraint->JointAccelerations (&joindDesc);
		}
		firstPassCoef = dFloat (1.0f);

		dFloat accNorm = dFloat (1.0e10f);
		for (int passes = 0; (passes < maxPasses) && (accNorm > maxAccNorm); passes ++) {
			accNorm = dFloat (0.0f);
			for (int curJoint = 0; curJoint < jointCount; curJoint ++) {

				VehicleJoint* const constraint = jointArray[curJoint];
				int index = constraint->m_start;
				int rowsCount = constraint->m_count;
				int m0 = constraint->m_state0->m_myIndex;
				int m1 = constraint->m_state1->m_myIndex;

				dVector linearM0 (internalForces[m0].m_linear);
				dVector angularM0 (internalForces[m0].m_angular);
				dVector linearM1 (internalForces[m1].m_linear);
				dVector angularM1 (internalForces[m1].m_angular);

				BodyState* const state0 = constraint->m_state0;
				BodyState* const state1 = constraint->m_state1;
				const dMatrix& invInertia0 = state0->m_invInertia;
				const dMatrix& invInertia1 = state1->m_invInertia;
				dFloat invMass0 = state0->m_invMass;
				dFloat invMass1 = state1->m_invMass;

				for (int k = 0; k < rowsCount; k ++) {
					VehicleJoint::JacobianPair* const row = &jacobianArray[index];
					VehicleJoint::JacobianColum* const col = &jacobianColumnArray[index];
	
					dVector JMinvIM0linear (row->m_jacobian_IM0.m_linear.Scale (invMass0));
					dVector JMinvIM1linear (row->m_jacobian_IM1.m_linear.Scale (invMass1));
					dVector JMinvIM0angular = invInertia0.UnrotateVector(row->m_jacobian_IM0.m_angular);
					dVector JMinvIM1angular = invInertia1.UnrotateVector(row->m_jacobian_IM1.m_angular);

					dVector acc (JMinvIM0linear.CompProduct(linearM0) + 
								 JMinvIM0angular.CompProduct(angularM0) + 
							  	 JMinvIM1linear.CompProduct(linearM1) + 
						         JMinvIM1angular.CompProduct(angularM1));


					dFloat a = col->m_coordenateAccel - acc.m_x - acc.m_y - acc.m_z - col->m_force * col->m_diagDamp;
					dFloat f = col->m_force + col->m_invDJMinvJt * a;

					dFloat lowerFrictionForce = col->m_jointLowFriction;
					dFloat upperFrictionForce = col->m_jointHighFriction;

					if (f > upperFrictionForce) {
						a = dFloat (0.0f);
						f = upperFrictionForce;
					} else if (f < lowerFrictionForce) {
						a = dFloat (0.0f);
						f = lowerFrictionForce;
					}

					accNorm = dMax (accNorm, dAbs (a));
					dFloat prevValue = f - col->m_force;
					col->m_force = f;

					linearM0 += row->m_jacobian_IM0.m_linear.Scale (prevValue);
					angularM0 += row->m_jacobian_IM0.m_angular.Scale (prevValue);
					linearM1 += row->m_jacobian_IM1.m_linear.Scale (prevValue);
					angularM1 += row->m_jacobian_IM1.m_angular.Scale (prevValue);
					index ++;
				}
				internalForces[m0].m_linear = linearM0;
				internalForces[m0].m_angular = angularM0;
				internalForces[m1].m_linear = linearM1;
				internalForces[m1].m_angular = angularM1;
			}
		}

		for (CustomList<BodyState*>::CustomListNode* stateNode = m_stateList.GetFirst()->GetNext(); stateNode; stateNode = stateNode->GetNext()) {
			BodyState* const state = stateNode->GetInfo();
			int index = state->m_myIndex;
			dVector force (state->m_externalForce + internalForces[index].m_linear);
			dVector torque (state->m_externalTorque + internalForces[index].m_angular);
			state->IntegrateForce(timestep, force, torque);
		}
	}

	//dFloat maxAccNorm2 = maxAccNorm * maxAccNorm;
	for (CustomList<BodyState*>::CustomListNode* stateNode = m_stateList.GetFirst()->GetNext(); stateNode; stateNode = stateNode->GetNext()) {
		BodyState* const state = stateNode->GetInfo();
		int index = state->m_myIndex;
        state->CalculateAverageAcceleration (invTimestepSrc, stateVeloc[index].m_linear, stateVeloc[index].m_angular);
	}

	for (int i = 0; i < jointCount; i ++) {
		VehicleJoint* const constraint = jointArray[i];
		int first = constraint->m_start;
		int count = constraint->m_count;
		for (int j = 0; j < count; j ++) { 
			const VehicleJoint::JacobianColum* const col = &jacobianColumnArray[j + first];
			dFloat val = col->m_force; 
			constraint->m_jointFeebackForce[j] = val;
		}
		constraint->UpdateSolverForces (jacobianArray);
	}
}

void CustomVehicleController::PreUpdate(dFloat timestep, int threadIndex)
{
	VehicleJoint* jointArray[VEHICLE_CONTROLLER_MAX_JOINTS];
	VehicleJoint::JacobianColum jacobianColumn[VEHICLE_CONTROLLER_MAX_JACOBIANS_PAIRS];
	VehicleJoint::JacobianPair jacobianPairArray[VEHICLE_CONTROLLER_MAX_JACOBIANS_PAIRS];


//dTrace (("\n"));

	// apply all external forces and torques to chassis and all tire velocities
	dFloat timestepInv = 1.0f / timestep;
	NewtonBody* const body = GetBody();
	CustomControllerConvexCastPreFilter castFilter (body);
	m_chassisState.UpdateDynamicInputs();
	for (TireList::CustomListNode* node = m_tireList.GetFirst(); node; node = node->GetNext()) {
		TireBodyState* const tire = &node->GetInfo();
		tire->Collide(castFilter, timestepInv);
		tire->UpdateDynamicInputs(timestep);
	}


	// update all components
	if (m_engine) {
		m_engine->Update(timestep);
	}
	if (m_steering) {
		m_steering->Update(timestep);
	}

	if (m_handBrakes) {
		m_handBrakes->Update(timestep);
	}

	if (m_brakes) {
		m_brakes->Update(timestep);
	}


	// Get the number of active joints for this integration step
	int jointCount = GetActiveJoints(jointArray);
	BuildJacobianMatrix (jointCount, jointArray, timestep, jacobianPairArray, jacobianColumn);
	CalculateReactionsForces (jointCount, jointArray, timestep, jacobianPairArray, jacobianColumn);

	NewtonBodySetForce (body, &m_chassisState.m_externalForce[0]);
	NewtonBodySetTorque (body, &m_chassisState.m_externalTorque[0]);

	//dTrace (("%f %f %f\n", m_chassisState.m_externalTorque[0], m_chassisState.m_externalTorque[1], m_chassisState.m_externalTorque[2]));

	// integrate tires angular velocity
	const dVector& chassisOmega = m_chassisState.m_omega;
	for (TireList::CustomListNode* node = m_tireList.GetFirst(); node; node = node->GetNext()) {
		TireBodyState* const tire = &node->GetInfo();
		dVector relOmega (tire->m_omega - chassisOmega);
		tire->m_rotatonSpeed = relOmega % tire->m_matrix[0];
		tire->m_rotationAngle = dMod (tire->m_rotationAngle + tire->m_rotatonSpeed * timestep, 2.0f * 3.141592f);
		//tire->m_rotationAngle = 0;
	}
}




