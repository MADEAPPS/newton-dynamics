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
#include <CustomVehicleControllerComponent.h>


void CustomVehicleControllerComponent::dInterpolationCurve::InitalizeCurve (int points, const dFloat* const steps, const dFloat* const values)
{
	m_count = points;
	dAssert (points < int (sizeof(m_nodes)/sizeof (m_nodes[0])));
	memset (m_nodes, 0, sizeof (m_nodes));
	for (int i = 0; i < m_count; i ++) {
		m_nodes[i].m_param = steps[i];
		m_nodes[i].m_value = values[i];
	}
}

dFloat CustomVehicleControllerComponent::dInterpolationCurve::GetValue (dFloat param) const
{
	dAssert (m_count);
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




CustomVehicleControllerComponentSteering::CustomVehicleControllerComponentSteering (CustomVehicleController* const controller, dFloat maxAngleInRadians)
	:CustomVehicleControllerComponent (controller)
	,m_maxAngle (dAbs (maxAngleInRadians))
{
}

void CustomVehicleControllerComponentSteering::AddSteeringTire (CustomVehicleControllerBodyStateTire* const tireNode, dFloat sign)
{
	dTireSignPair& pair = m_steeringTires.Append()->GetInfo();

	pair.m_sign = (sign >= 0.0f) ? 1.0f : -1.0f;
	pair.m_tireNode = m_controller->m_tireList.GetNodeFromInfo (*tireNode);
}


void CustomVehicleControllerComponentSteering::Update (dFloat timestep)
{
	for (dList<dTireSignPair>::dListNode* node = m_steeringTires.GetFirst(); node; node = node->GetNext()) {
		dTireSignPair& pair = node->GetInfo();
		CustomVehicleControllerBodyStateTire& tire = pair.m_tireNode->GetInfo();
		tire.m_steeringAngle = m_maxAngle * m_param * pair.m_sign;
	}
}


CustomVehicleControllerComponentBrake::CustomVehicleControllerComponentBrake (CustomVehicleController* const controller, dFloat maxBrakeTorque)
	:CustomVehicleControllerComponent (controller)
	,m_maxBrakeTorque (dAbs (maxBrakeTorque))
{
}

void CustomVehicleControllerComponentBrake::AddBrakeTire (CustomVehicleControllerBodyStateTire* const tire)
{
	m_brakeTires.Append(m_controller->m_tireList.GetNodeFromInfo (*tire));
}


void CustomVehicleControllerComponentBrake::Update (dFloat timestep)
{
	for (dList<dList<CustomVehicleControllerBodyStateTire>::dListNode*>::dListNode* node = m_brakeTires.GetFirst(); node; node = node->GetNext()) {
		CustomVehicleControllerBodyStateTire& tire = node->GetInfo()->GetInfo();
		tire.m_breakTorque = dMax (tire.m_breakTorque, dAbs (m_maxBrakeTorque * m_param));
	}
}




CustomVehicleControllerComponentEngine::dGearBox::dGearBox(CustomVehicleController* const controller, dFloat reverseGearRatio, int gearCount, const dFloat* const gearBox)
	:m_gearsCount(gearCount + 2)
	,m_currentGear(NULL)
    ,m_controller(controller)
	,m_automatic(true)
{
	memset (m_gears, 0, sizeof (m_gears));

	dAssert (gearCount < (m_maxGears - 2));
	for (int i = 0; i < gearCount; i ++) {
        m_gears[i + m_firstGear] = new dGearState (gearBox[i], 0.9f, 0.3f, dGearID(i + m_firstGear)); 
	}

    for (int i = 0; i < gearCount - 1; i ++) {
        m_gears[i + m_firstGear]->m_next = m_gears[i + m_firstGear + 1]; 
    }
    

    for (int i = gearCount - 1; i; i --) {
        m_gears[i + m_firstGear]->m_prev = m_gears[i + m_firstGear - 1]; 
    }
 
    m_gears[m_reverseGear] = new dReverseGearState(reverseGearRatio);
    m_gears[m_newtralGear] = new dNeutralGearState(m_gears[m_firstGear], m_gears[m_reverseGear]);

    m_gears[m_firstGear]->m_prev = m_gears[m_firstGear];
    m_gears[m_firstGear + gearCount - 1]->m_next = m_gears[m_firstGear + gearCount - 1];

    m_currentGear = m_gears[m_newtralGear];

    for (int i = 0; i < m_gearsCount; i ++) {
        m_gears[i]->m_neutral = m_gears[m_newtralGear];
        m_gears[i]->m_reverse = m_gears[m_reverseGear];
    }
}

CustomVehicleControllerComponentEngine::dGearBox::~dGearBox ()
{
	for (int i = 0; i < m_gearsCount; i ++) {
		delete m_gears[i]; 
	}
}


void CustomVehicleControllerComponentEngine::dGearBox::SetOptimalShiftLimits (dFloat minShift, dFloat maxShift)
{
	minShift = dMax (minShift - 0.05f, 0.10f);
	maxShift = dMin (maxShift + 0.05f, 0.95f);
	for (int i = m_firstGear; i < m_gearsCount; i ++) {
		dGearState* const state = m_gears[i];
		state->m_shiftUp = maxShift;
		state->m_shiftDown = minShift;
	}
}

CustomVehicleControllerComponentEngine::dGearBox::dGearState* CustomVehicleControllerComponentEngine::dGearBox::dNeutralGearState::Update(CustomVehicleController* const vehicle)
{
    const CustomVehicleControllerComponentEngine* const engine = vehicle->GetEngine();
    dFloat param = engine->GetParam();

    if (param > dFloat (1.0e-3f)) {
        return m_next;
    } else if (param < dFloat(-1.0e-3f)) {
        return m_prev;
    }
    return this;
}

CustomVehicleControllerComponentEngine::dGearBox::dGearState* CustomVehicleControllerComponentEngine::dGearBox::dGearState::Update(CustomVehicleController* const vehicle)
{
    const CustomVehicleControllerComponentEngine* const engine = vehicle->GetEngine();

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



dFloat CustomVehicleControllerComponentEngine::dGearBox::GetGearRatio(int gear) const 
{
	return m_gears[gear]->m_ratio;
}

void CustomVehicleControllerComponentEngine::dGearBox::SetTransmissionMode (bool mode) 
{
	m_automatic = mode;
}

bool CustomVehicleControllerComponentEngine::dGearBox::GetTransmissionMode () const 
{
	return m_automatic;
}


void CustomVehicleControllerComponentEngine::dGearBox::SetGear(int gear) 
{
    for (int i = 0; i < m_gearsCount; i ++) {
        if (m_gears[i]->m_id == gear) {
            m_currentGear = m_gears[i];
            break;
        }
    }
}

int CustomVehicleControllerComponentEngine::dGearBox::GetGear() const 
{
	return m_currentGear->m_id;
}

int CustomVehicleControllerComponentEngine::dGearBox::GetGearCount() const 
{
	return m_gearsCount;
}


void CustomVehicleControllerComponentEngine::dGearBox::Update(dFloat timestep)
{
	if (m_automatic) {
        m_currentGear = m_currentGear->Update(m_controller);
	}
}


CustomVehicleControllerComponentEngine::CustomVehicleControllerComponentEngine (CustomVehicleController* const controller, dGearBox* const gearBox, CustomVehicleControllerBodyStateTire* const leftTire, CustomVehicleControllerBodyStateTire* const righTire)
	:CustomVehicleControllerComponent (controller)
	,m_gearBox(gearBox)
	,m_leftTire(controller->m_tireList.GetNodeFromInfo (*leftTire))
	,m_righTire(controller->m_tireList.GetNodeFromInfo (*righTire))
	,m_speedMPS(0.0f)
	,m_topSpeedMPS(0.0f)
	,m_crownGearRatio(1.0f)
    ,m_momentOfInertia(10.0f)
//    ,m_engineResistance(0.0f)
	,m_engineIdleFriction(0.0f)
    ,m_engineIdleResistance(0.0f)
	,m_radiansPerSecundsAtRedLine(0.0f)
	,m_radiansPerSecundsAtPeakPower(0.0f)
	,m_radiansPerSecundsAtIdleTorque(0.0f)
	,m_engineSwitch(true)
{
}

CustomVehicleControllerComponentEngine::~CustomVehicleControllerComponentEngine()
{
	if (m_gearBox) {
		delete m_gearBox;
	}
}



dFloat CustomVehicleControllerComponentEngine::GetInertia() const
{
	return m_momentOfInertia;
}

void CustomVehicleControllerComponentEngine::SetInertia(dFloat inertia)
{
	m_momentOfInertia = inertia;
	m_controller->m_engineState.UpdateInertia();
}



CustomVehicleControllerComponentEngine::dGearBox* CustomVehicleControllerComponentEngine::GetGearBox() const
{
    return m_gearBox;
}

void CustomVehicleControllerComponentEngine::SetGear (int gear)
{
	m_gearBox->SetGear(gear);
}

int CustomVehicleControllerComponentEngine::GetGear () const
{
	return m_gearBox->GetGear();
}

void CustomVehicleControllerComponentEngine::SetTransmissionMode (bool mode)
{
	m_gearBox->SetTransmissionMode(mode);
}

bool CustomVehicleControllerComponentEngine::GetTransmissionMode () const
{
	return m_gearBox->GetTransmissionMode();
}

/*
dFloat CustomVehicleControllerComponentEngine::GetRedLineResistance() const
{
    return m_engineResistance;
}

dFloat CustomVehicleControllerComponentEngine::GetIdleResistance() const
{
    return m_engineIdleResistance;
}


dFloat CustomVehicleControllerComponentEngine::GetIdleRadianPerSeconds () const
{
	return m_radiansPerSecundsAtIdleTorque;
}

dFloat CustomVehicleControllerComponentEngine::GetDifferencialGearRatio() const
{
	dAssert(0);
	return m_crownGearRatio;
}
*/

dFloat CustomVehicleControllerComponentEngine::GetTorque (dFloat radianPerSeconds) const
{
    return m_torqueCurve.GetValue(radianPerSeconds);
}

dList<CustomVehicleControllerBodyStateTire>::dListNode* CustomVehicleControllerComponentEngine::GetLeftTireNode() const
{
    return m_leftTire;
}

dList<CustomVehicleControllerBodyStateTire>::dListNode* CustomVehicleControllerComponentEngine::GetRightTireNode() const
{
    return m_righTire;
}


dFloat CustomVehicleControllerComponentEngine::GetTopSpeed () const
{
	return m_topSpeedMPS;
}

dFloat CustomVehicleControllerComponentEngine::GetSpeed () const
{
	return m_speedMPS;
}

dFloat CustomVehicleControllerComponentEngine::GetRPM () const
{
	return m_controller->m_engineState.m_radianPerSecund * m_crownGearRatio * 9.55f;
}

dFloat CustomVehicleControllerComponentEngine::GetTopRPM () const
{
    return m_radiansPerSecundsAtPeakPower * m_crownGearRatio * 9.55f;
}

dFloat CustomVehicleControllerComponentEngine::GetRedLineRPM() const 
{
	return m_radiansPerSecundsAtRedLine * m_crownGearRatio * 9.55f;
}

void CustomVehicleControllerComponentEngine::SetTopSpeed (dFloat topSpeedMPS, dFloat rpsAtPickPower)
{
	dAssert (topSpeedMPS >= 0.0f);
	dAssert (topSpeedMPS < 100.0f);

	CustomVehicleControllerBodyStateTire* const tire = &m_leftTire->GetInfo();
	dAssert (tire);
	m_topSpeedMPS = topSpeedMPS;

	// drive train geometrical relations
	// G0 = m_differentialGearRatio
	// G1 = m_transmissionGearRatio
	// s = topSpeedMPS
	// r = tireRatio
	// wt = rpsAtTire
	// we = rpsAtPickPower
	// we = G1 * G0 * wt;
	// wt = e / r
	// we = G0 * G1 * s / r
	// G0 = r * we / (G1 * s)

	// using the top gear and the optimal engine torque for the calculations
	dFloat topGearRatio = m_gearBox->GetGearRatio(m_gearBox->GetGearCount() - 1);
	m_crownGearRatio = tire->m_radio * rpsAtPickPower / (topSpeedMPS * topGearRatio);
/*
	// calculate internal tire rolling resistance, (assume no transmission power lost)
	dFloat gain = m_differentialGearRatio * topGearRatio;
	dFloat tireTopOmega = m_topSpeedMPS / tire->m_radio;

	dFloat engineTorque = m_torqueCurve.GetValue(m_radiansPerSecundsAtPeakPower);
	dFloat tireTorque = engineTorque * gain;
	dFloat tireRollingResistance = tireTorque / (tireTopOmega * tireTopOmega);

	for (dList<CustomVehicleControllerBodyStateTire>::dListNode* node = m_controller->m_tireList.GetFirst(); node; node = node->GetNext()) {
		CustomVehicleControllerBodyStateTire* const tire = &node->GetInfo();
		tire->m_idleRollingResistance = tireRollingResistance;
	}
*/
}


void CustomVehicleControllerComponentEngine::Update (dFloat timestep)
{
    m_controller->m_engineState.Update(timestep, m_controller);

/*
static FILE* xxx;
if (!xxx) 
{
	fopen_s (&xxx, "torque_rpm.csv", "wt");
	fprintf (xxx, "gear, tire_torque, tire_rps\n");
}

static int xxxxx;
dTrace (("%d %f\n", xxxxx, leftRPS));
xxxxx ++;
fprintf (xxx, "%d, %f, %f,\n", gear * 100, leftTorque, leftRPS);
fflush (xxx);
*/

	// set the vehicle speed
	const CustomVehicleControllerBodyStateChassis& chassis = m_controller->m_chassisState;
	dVector front (chassis.m_matrix.RotateVector(chassis.m_localFrame[0]));
	m_speedMPS = chassis.m_veloc % front;
}


void CustomVehicleControllerComponentEngine::ConvertToMetricSystem (
	dFloat& vehicleSpeedInKilometerPerHours, dFloat& engineMomentOfInertia,
	dFloat& idleTorqueInPoundFoot, dFloat& revolutionsPerMinutesAtIdleTorque, 
	dFloat& peakTorqueInPoundFoot, dFloat& revolutionsPerMinutesAtPeakTorque, 
	dFloat& peakHorsePower, dFloat& revolutionsPerMinutesAtPeakHorsePower, 
	dFloat& torqueAtRedLineInPoundFoot, dFloat& revolutionsPerMinutesAtRedLineTorque) const
{
	const dFloat horsePowerToWatts = 735.5f;
	const dFloat kmhToMetersPerSecunds = 0.278f;
	const dFloat rpmToRadiansPerSecunds = 0.105f;
	const dFloat poundFootToNewtonMeters = 1.356f;
		
	peakHorsePower *= horsePowerToWatts;
	idleTorqueInPoundFoot *= poundFootToNewtonMeters;
	peakTorqueInPoundFoot *= poundFootToNewtonMeters;
	torqueAtRedLineInPoundFoot *= poundFootToNewtonMeters;

	revolutionsPerMinutesAtIdleTorque *= rpmToRadiansPerSecunds;
	revolutionsPerMinutesAtPeakTorque *= rpmToRadiansPerSecunds;
	revolutionsPerMinutesAtPeakHorsePower *= rpmToRadiansPerSecunds;
	revolutionsPerMinutesAtRedLineTorque *= rpmToRadiansPerSecunds;

	vehicleSpeedInKilometerPerHours *= kmhToMetersPerSecunds;
}


void CustomVehicleControllerComponentEngine::InitEngineTorqueCurve (
	dFloat vehicleSpeed, dFloat engineMomentOfInertia,
	dFloat idleTorque, dFloat rpsAtIdleTorque, 
	dFloat peakTorque, dFloat rpsAtPeakTorque, 
	dFloat peakHorsePower, dFloat rpsAtPeakHorsePower, 
	dFloat torqueAtRedLine, dFloat rpsAtRedLineTorque)
{
	int oldGear = m_gearBox->GetGear();
	m_gearBox->SetGear(m_gearBox->GetGearCount() - 1);

	m_momentOfInertia = engineMomentOfInertia;
	m_controller->m_engineState.Init (m_controller);

	ConvertToMetricSystem (vehicleSpeed, engineMomentOfInertia, idleTorque, rpsAtIdleTorque, peakTorque, rpsAtPeakTorque, peakHorsePower, rpsAtPeakHorsePower, torqueAtRedLine, rpsAtRedLineTorque);
	SetTopSpeed (vehicleSpeed, rpsAtPeakHorsePower);				  	   

	dFloat torqueAtPeakPower = peakHorsePower / rpsAtPeakHorsePower;
	dAssert (rpsAtIdleTorque > 0.0f);
	dAssert (rpsAtIdleTorque < rpsAtPeakTorque);
	dAssert (rpsAtPeakTorque < rpsAtPeakHorsePower);
	dAssert (rpsAtPeakHorsePower < rpsAtRedLineTorque);

	dAssert (idleTorque > 0.0f);
	dAssert (idleTorque < peakTorque);
	dAssert (peakTorque > torqueAtPeakPower);
	dAssert (torqueAtPeakPower > torqueAtRedLine);
	dAssert (torqueAtRedLine > 0.0f);
	dAssert (peakTorque * rpsAtPeakTorque < peakHorsePower);

	dFloat rpsTable[5];
	dFloat torqueTable[5];

	rpsTable[0] = 0.0f;
	rpsTable[1] = rpsAtIdleTorque / m_crownGearRatio;
	rpsTable[2] = rpsAtPeakTorque / m_crownGearRatio;
	rpsTable[3] = rpsAtPeakHorsePower / m_crownGearRatio;
	rpsTable[4] = rpsAtRedLineTorque / m_crownGearRatio;

	torqueTable[0] = idleTorque * m_crownGearRatio;
	torqueTable[1] = idleTorque * m_crownGearRatio;
	torqueTable[2] = peakTorque * m_crownGearRatio;
	torqueTable[3] = torqueAtPeakPower * m_crownGearRatio;
	torqueTable[4] = rpsAtRedLineTorque * m_crownGearRatio;

	m_torqueCurve.InitalizeCurve (sizeof (rpsTable)/sizeof (rpsTable[0]), rpsTable, torqueTable);

//	m_engineResistance = redLineTorque / (rpsAtRedLineTorque * rpsAtRedLineTorque * 0.99f * 0.99f);

	m_radiansPerSecundsAtIdleTorque = rpsTable[1];
	m_radiansPerSecundsAtPeakPower = rpsTable[3];
	m_radiansPerSecundsAtRedLine = rpsTable[4];


	dFloat redLineRps = rpsTable[4];
	dFloat idleEngineRps = rpsTable[1];;
	dFloat idleEngineTorque = GetTorque (idleEngineRps);
	dFloat redLineEngineTorque = GetTorque (redLineRps);

	m_engineIdleFriction = idleEngineTorque * 0.5f;
	dAssert ((redLineEngineTorque - m_engineIdleFriction) > 0.0f);
	m_engineIdleResistance = (redLineEngineTorque - m_engineIdleFriction) / (rpsAtRedLineTorque * rpsAtRedLineTorque);


//	m_radiansPerSecundsAtIdleTorque = rpsAtIdleTorque;
//	m_radiansPerSecundsAtPeakPower = rpsAtPeakPower;
	m_radiansPerSecundsAtRedLine = rpsAtRedLineTorque;

//	m_gearBox->SetOptimalShiftLimits (rpsAtPeakTorque / rpsAtRedLineTorque, rpsAtPeakPower/ rpsAtRedLineTorque);


	m_gearBox->SetGear(oldGear);
}
