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


CustomVehicleControllerBodyState::CustomVehicleControllerBodyState()
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

void CustomVehicleControllerBodyState::Init(CustomVehicleController* const controller)
{
	m_controller = controller;
}

dFloat CustomVehicleControllerBodyState::GetMass () const
{
	return m_mass;
}

const dMatrix& CustomVehicleControllerBodyState::GetMatrix () const
{
	return m_matrix;
}

const dMatrix& CustomVehicleControllerBodyState::GetLocalMatrix () const
{
	return m_localFrame;
}

const dVector& CustomVehicleControllerBodyState::GetCenterOfMass () const
{
	return m_globalCentreOfMass;
}



void CustomVehicleControllerBodyState::UpdateInertia()
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

void CustomVehicleControllerBodyState::IntegrateForce (dFloat timestep, const dVector& force, const dVector& torque)
{
	dVector accel (force.Scale (m_invMass));
	dVector alpha (m_invInertia.RotateVector(torque));
	m_veloc += accel.Scale (timestep);
	m_omega += alpha.Scale (timestep);
}

void CustomVehicleControllerBodyState::ApplyNetForceAndTorque (dFloat invTimestep, const dVector& veloc, const dVector& omega)
{
	dVector accel = (m_veloc - veloc).Scale(invTimestep);
	dVector alpha = (m_omega - omega).Scale(invTimestep);

	m_externalForce = accel.Scale(m_mass);
	alpha = m_matrix.UnrotateVector(alpha);
	m_externalTorque = m_matrix.RotateVector(alpha.CompProduct(m_localInertia));
}



void CustomVehicleControllerBodyStateChassis::Init (CustomVehicleController* const controller, const dMatrix& localframe)
{
    CustomVehicleControllerBodyState::Init (controller);
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

dFloat CustomVehicleControllerBodyStateChassis::GetAerodynamicsDowforceCoeficient () const
{
	return m_aerodynamicsDownForceCoefficient;
}

void CustomVehicleControllerBodyStateChassis::SetAerodynamicsDownforceCoefficient (dFloat maxDownforceInGravities, dFloat topSpeed)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	NewtonBody* const body = m_controller->GetBody();
	NewtonBodyGetMassMatrix(body, &mass, &Ixx, &Iyy, &Izz);
	m_aerodynamicsDownForceCoefficient = mass * maxDownforceInGravities / (topSpeed * topSpeed);
}

void CustomVehicleControllerBodyStateChassis::SetDryRollingFrictionTorque (dFloat dryRollingFrictionTorque)
{
	m_dryRollingFrictionTorque = dAbs (dryRollingFrictionTorque);
}

dFloat CustomVehicleControllerBodyStateChassis::GetDryRollingFrictionTorque () const
{
	return m_dryRollingFrictionTorque;
}


void CustomVehicleControllerBodyStateChassis::UpdateDynamicInputs()
{
	NewtonBody* const body = m_controller->GetBody();

	NewtonBodyGetMatrix (body, &m_matrix[0][0]);
	NewtonBodyGetVelocity (body, &m_veloc[0]);
	NewtonBodyGetOmega (body, &m_omega[0]);

	NewtonBodyGetForceAcc(body, &m_externalForce[0]);
	NewtonBodyGetTorqueAcc(body, &m_externalTorque[0]);

	dFloat frontSpeed = dMin (m_veloc % m_matrix[0], 50.0f);
	m_externalForce -= m_matrix[1].Scale (m_aerodynamicsDownForceCoefficient * frontSpeed * frontSpeed);

	m_globalCentreOfMass = m_matrix.TransformVector(m_localFrame.m_posit);

	UpdateInertia();
}

void CustomVehicleControllerBodyStateChassis::ApplyNetForceAndTorque (dFloat invTimestep, const dVector& veloc, const dVector& omega)
{
	CustomVehicleControllerBodyState::ApplyNetForceAndTorque (invTimestep, veloc, omega);
	NewtonBody* const body = m_controller->GetBody();
	NewtonBodySetForce (body, &m_externalForce[0]);
	NewtonBodySetTorque (body, &m_externalTorque[0]);
}


void CustomVehicleControllerBodyStateTire::Init (CustomVehicleController* const controller, const TireCreationInfo& tireInfo)
{
	CustomVehicleControllerBodyState::Init (controller);

	NewtonBody* const body = m_controller->GetBody();

	const dMatrix& vehicleFrame = m_controller->m_chassisState.m_localFrame;

	// build a normalized size collision shape and scale to math the tire size, make it is also transparent to collision  
	NewtonCollisionSetScale (m_controller->m_tireCastShape, tireInfo.m_width, tireInfo.m_radio, tireInfo.m_radio);
	NewtonCollisionSetCollisionMode (m_controller->m_tireCastShape, 0);

	// calculate the location of the tire matrix
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
	NewtonCollisionSetCollisionMode (m_controller->m_tireCastShape, 1);
	NewtonCollisionSetMatrix (m_controller->m_tireCastShape, &identMatrix[0][0]);

	// get the collision shape
	m_shape = NewtonCompoundCollisionGetCollisionFromNode (vehShape, tireShapeNode);

	// initialize all constants
	m_userData = tireInfo.m_userData;
	m_mass = dMax (tireInfo.m_mass, m_controller->m_chassisState.m_mass / 50.0f); 
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
	m_adhesionCoefficient = 1.0f;
	m_idleRollingResistance = 0.0f;
	m_tireLoad = dVector(0.0f, 0.0f, 0.0f, 0.0f);
	m_lateralForce = dVector(0.0f, 0.0f, 0.0f, 0.0f);
	m_longitudinalForce = dVector(0.0f, 0.0f, 0.0f, 0.0f);

	m_speed = 0.0f;
	m_posit = m_suspensionlenght;
	m_matrix = CalculateSteeringMatrix ();

	UpdateInertia();

	// initialize the joints that connect tghsi tire to other vehicle componets;
	m_chassisJoint.Init(m_controller, &m_controller->m_chassisState, this);
	m_contactJoint.Init(m_controller, &m_controller->m_staticWorld, this);
}

void* CustomVehicleControllerBodyStateTire::GetUserData() const
{
	return m_userData;
}

dMatrix CustomVehicleControllerBodyStateTire::CalculateSuspensionMatrix () const
{
	dMatrix matrix (m_localFrame);
	matrix.m_posit -= m_localFrame.m_up.Scale (m_posit);
	return matrix;
}

dMatrix CustomVehicleControllerBodyStateTire::CalculateSteeringMatrix () const
{
	return dYawMatrix(m_steeringAngle) * CalculateSuspensionMatrix ();
}

dMatrix CustomVehicleControllerBodyStateTire::CalculateLocalMatrix () const
{
	return dPitchMatrix(m_rotationAngle) * CalculateSteeringMatrix ();
}

dMatrix CustomVehicleControllerBodyStateTire::CalculateGlobalMatrix () const
{
	dMatrix matrix;
	NewtonBodyGetMatrix(m_controller->GetBody(), &matrix[0][0]);
	return CalculateLocalMatrix () * matrix;
}


void CustomVehicleControllerBodyStateTire::UpdateTransform()
{
	//dMatrix localMatrix (CustomVehicleController::TireBodyState::CalculateSteeringMatrix ());
	dMatrix localMatrix (CalculateLocalMatrix ());
	NewtonCollisionSetMatrix(m_shape, &localMatrix[0][0]);	
}


void CustomVehicleControllerBodyStateTire::Collide (CustomControllerConvexCastPreFilter& filter, dFloat timestepInv)
{
	NewtonBody* const body = m_controller->GetBody();
	NewtonWorld* const world = NewtonBodyGetWorld(body);
	const dMatrix& controllerMatrix = m_controller->m_chassisState.m_matrix;

	dFloat posit0 = m_posit;

	m_posit = 0.0f;
	m_speed = 0.0f;
	dMatrix localMatrix (CustomVehicleControllerBodyStateTire::CalculateSteeringMatrix ());
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


void CustomVehicleControllerBodyStateTire::UpdateDynamicInputs(dFloat timestep)
{
	CustomVehicleControllerBodyStateChassis& chassis = m_controller->m_chassisState;

//	m_dryFrictionTorque = chassis.m_dryRollingFrictionTorque;

	m_matrix = CalculateSteeringMatrix() * chassis.m_matrix;
	m_globalCentreOfMass = m_matrix.m_posit;
	UpdateInertia();

	// get the velocity state for this tire
	dVector relPosit (m_matrix.m_posit - chassis.m_globalCentreOfMass);
	m_omega = chassis.m_omega + m_matrix[0].Scale (m_rotatonSpeed);
	m_veloc = chassis.m_veloc + chassis.m_omega * relPosit + m_matrix[1].Scale (m_speed);

	// set the initial force on this tire
	m_externalForce = chassis.m_gravity.Scale (m_mass);
	m_externalTorque = dVector (0.0f, 0.0f, 0.0f, 0.0f);

	// calculate force an torque generate by the suspension
	m_tireLoad = dVector(0.0f, 0.0f, 0.0f, 0.0f);
	m_lateralForce = dVector(0.0f, 0.0f, 0.0f, 0.0f);
	m_longitudinalForce = dVector(0.0f, 0.0f, 0.0f, 0.0f);
	if (m_contactJoint.m_contactCount) {
		dFloat distance = m_suspensionlenght - m_posit;
		dAssert (distance >= 0.0f);
		dAssert (distance <= m_suspensionlenght);
		if (distance <= dFloat(1.0e-3f)) {
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
		dVector torque (relPosit * m_tireLoad);
		chassis.m_externalForce += m_tireLoad;
		chassis.m_externalTorque += torque;

		// the spring apply the same force in the opposite direction to the tire
		m_externalForce -= m_tireLoad;
	}
}


/*
dFloat CustomVehicleControllerBodyStateTire::GetAdhesionCoefficient() const
{
	return m_adhesionCoefficient * 0.5f;
}
void CustomVehicleControllerBodyStateTire::SetAdhesionCoefficient(dFloat Coefficient)
{
	m_adhesionCoefficient = 2.0f * dClamp (Coefficient, dFloat(0.0f), dFloat(1.0f));
}

void CustomVehicleControllerBodyStateTire::IntegrateForce (dFloat timestep, const dVector& force, const dVector& torque)
{
	BodyState::IntegrateForce (timestep, force, torque);
}
*/



void CustomVehicleControllerBodyStateTire::ApplyNetForceAndTorque (dFloat invTimestep, const dVector& veloc, const dVector& omega)
{
	CustomVehicleControllerBodyState::ApplyNetForceAndTorque (invTimestep, veloc, omega);
	const CustomVehicleControllerBodyStateChassis& chassis = m_controller->m_chassisState;

#if 0
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
#endif

	// integrate tires angular velocity
	dVector relOmega (m_omega - chassis.m_omega);
	m_rotatonSpeed = relOmega % m_matrix[0];
	m_rotationAngle = dMod (m_rotationAngle + m_rotatonSpeed / invTimestep, 2.0f * 3.141592f);
}



void CustomVehicleControllerBodyStateEngine::Init (CustomVehicleController* const controller)
{
    CustomVehicleControllerBodyState::Init (controller);

//  NewtonBody* const body = m_controller->GetBody();
//  const dMatrix& vehicleFrame = m_controller->m_chassisState.m_localFrame;

	CustomVehicleControllerComponentEngine* const engine = controller->GetEngine();

    m_mass = 0.0f; 
    m_invMass = 0.0f;
    m_radianPerSecund = 0.0f;

    m_localInertia[0] = engine->GetInertia();
    m_localInertia[1] = engine->GetInertia();
    m_localInertia[2] = engine->GetInertia();
    m_localInertia[3] = 0.0f;

    m_localInvInertia[0] = 1.0f / m_localInertia[0];
    m_localInvInertia[1] = 1.0f / m_localInertia[1];
    m_localInvInertia[2] = 1.0f / m_localInertia[2];
    m_localInvInertia[3] = 0.0f;

	UpdateInertia();

	m_idleFriction.Init (m_controller, this, &controller->m_staticWorld);
	m_leftTire.Init (m_controller, this, &engine->GetLeftTireNode()->GetInfo());
    m_rightTire.Init (m_controller, this, &engine->GetRightTireNode()->GetInfo());
}

int CustomVehicleControllerBodyStateEngine::CalculateActiveJoints (CustomVehicleController* const controller, CustomVehicleControllerJoint** const jointArray)
{
	int count = 0;

    CustomVehicleControllerComponentEngine* const engine = controller->GetEngine();
	if (engine) {
		int gear = engine->GetGear();
		if (gear != CustomVehicleControllerComponentEngine::dGearBox::m_newtralGear) {
			count = 2;
			jointArray[0] = &m_leftTire;
			jointArray[1] = &m_rightTire;
		} else {
			count = 1;
			jointArray[0] = &m_idleFriction;
		}
	}

    return count;
}

void CustomVehicleControllerBodyStateEngine::ApplyNetForceAndTorque (dFloat invTimestep, const dVector& veloc, const dVector& omega)
{
//	ApplyNetForceAndTorque (invTimestep, veloc, omega)

    m_veloc = dVector (0.0f, 0.0f, 0.0f, 0.0f);
    m_externalForce = dVector (0.0f, 0.0f, 0.0f, 0.0f);
	m_externalTorque = dVector (0.0f, 0.0f, 0.0f, 0.0f);
	m_radianPerSecund = m_omega % m_matrix[0];

/*
	dFloat idleRps = m_controller->GetEngine()->GetIdleRadianPerSeconds();
	if (m_radianPerSecund < (idleRps * 0.25f)) {
		m_radianPerSecund = idleRps * 0.25f;
		m_omega = m_matrix[0].Scale (m_radianPerSecund);
	}
*/
}

void CustomVehicleControllerBodyStateEngine::Update (dFloat timestep, CustomVehicleController* const controller)
{
    CustomVehicleControllerComponentEngine* const engine = controller->GetEngine();
    CustomVehicleControllerComponentEngine::dGearBox* const gearBox = engine->GetGearBox();

	gearBox->Update (timestep);
gearBox->SetGear (CustomVehicleControllerComponentEngine::dGearBox::m_newtralGear);

	int gear = gearBox->GetGear();
    dFloat torque = 0.0f;
    dFloat param = engine->m_engineSwitch ? dMax (engine->GetParam(), 0.1f) : 0.0f;
	if (gear == CustomVehicleControllerComponentEngine::dGearBox::m_newtralGear) {

		dFloat nominalTorque = engine->GetTorque (m_radianPerSecund) * param;
		dFloat resistance = engine->m_engineIdleResistance * m_radianPerSecund * m_radianPerSecund;

		torque = nominalTorque - resistance;

//		dFloat idleRps = engine->GetIdleRadianPerSeconds();
//		dFloat idleTorque = engine->GetTorque (idleRps);
//      torque = idleTorque * param;
//      torque -= m_radianPerSecund * m_radianPerSecund * engine->GetIdleResistance();
		m_idleFriction.m_omega = engine->m_radiansPerSecundsAtIdleTorque;
		m_idleFriction.m_friction = engine->m_engineIdleFriction;

	} else {
dAssert (0);
/*
        dFloat gearGain = gearBox->GetGearRatio(gear) * engine->GetDifferencialGearRatio();
        dFloat nominalTorque = engine->GetTorque (m_radianPerSecund);

        torque = nominalTorque * param;
        torque -= m_radianPerSecund * m_radianPerSecund * engine->GetRedLineResistance();

        m_leftTire.m_powerTrainGain = gearGain;
        m_rightTire.m_powerTrainGain = gearGain;
*/
	}

    m_externalForce = dVector (0.0f, 0.0f, 0.0f, 0.0f);
    m_externalTorque = dVector (torque, 0.0f, 0.0f, 0.0f);

	CustomVehicleControllerBodyStateChassis* const chassis = &m_controller->m_chassisState;
	dFloat sign = (gear != CustomVehicleControllerComponentEngine::dGearBox::m_newtralGear) ? ((gear == CustomVehicleControllerComponentEngine::dGearBox::m_reverseGear) ? -1.0f : 1.0f) : 0.0f;
	// apply the engine torque on this tire
	dVector reactionTorque (chassis->m_matrix[2].Scale (torque * sign * 0.25f));

	// there is not direct joint between the tire and the chassis, but there tire inertia should apply some toque to the engine
	// I will use an arbitrary 25% of the torque goes to the chassis.
	chassis->m_externalTorque += reactionTorque;

}

