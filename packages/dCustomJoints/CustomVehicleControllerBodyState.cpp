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
	:dComplemtaritySolver::dBodyState()
	,m_controller(NULL)
{
}

void CustomVehicleControllerBodyState::Init(CustomVehicleController* const controller)
{
	m_controller = controller;
}

CustomVehicleController* CustomVehicleControllerBodyState::GetController() const
{
	return m_controller;
}

dFloat CustomVehicleControllerBodyState::GetMass () const
{
	return dComplemtaritySolver::dBodyState::GetMass();
}

const dMatrix& CustomVehicleControllerBodyState::GetMatrix () const
{
	return dComplemtaritySolver::dBodyState::GetMatrix();
}

const dMatrix& CustomVehicleControllerBodyState::GetLocalMatrix () const
{
	return dComplemtaritySolver::dBodyState::GetLocalMatrix();
}

const dVector& CustomVehicleControllerBodyState::GetCenterOfMass () const
{
	return dComplemtaritySolver::dBodyState::GetCenterOfMass();
}



void CustomVehicleControllerBodyState::UpdateInertia()
{
	dComplemtaritySolver::dBodyState::UpdateInertia();
}

void CustomVehicleControllerBodyState::IntegrateForce (dFloat timestep, const dVector& force, const dVector& torque)
{
	dComplemtaritySolver::dBodyState::IntegrateForce(timestep, force, torque);
}

void CustomVehicleControllerBodyState::ApplyNetForceAndTorque (dFloat invTimestep, const dVector& veloc, const dVector& omega)
{
	dComplemtaritySolver::dBodyState::ApplyNetForceAndTorque (invTimestep, veloc, omega);
}


void CustomVehicleControllerBodyStateContact::Init (CustomVehicleController* const controller, const NewtonBody* const body)
{
	CustomVehicleControllerBodyState::Init (controller);

	m_newtonBody = (NewtonBody*)body;

	m_localFrame = dGetIdentityMatrix();
	NewtonBodyGetCentreOfMass(body, &m_localFrame[3][0]);

	NewtonBodyGetMatrix(body, &m_matrix[0][0]);
	NewtonBodyGetMassMatrix(body, &m_mass, &m_localInertia[0], &m_localInertia[1], &m_localInertia[2]);

	NewtonBodyGetOmega(body, &m_omega[0]);
	NewtonBodyGetVelocity(body, &m_veloc[0]);

	m_globalCentreOfMass = m_matrix.TransformVector(m_localFrame.m_posit);

/*
	dVector force (0.0f, 0.0f, 0.0f, 0.0f);
	dVector torque (0.0f, 0.0f, 0.0f, 0.0f);
	if (m_invMass > 0.0f) {
		const NewtonBody* const otherBody = m_controller->GetBody();

		NewtonBodyGetForceAcc(m_newtonBody, &force[0]);
		NewtonBodyGetTorqueAcc (m_newtonBody, &torque[0]);
		for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint (m_newtonBody); joint; joint = NewtonBodyGetNextContactJoint (m_newtonBody, joint)) {
			if (NewtonJointIsActive(joint) && (NewtonJointGetBody0(joint) != otherBody) && (NewtonJointGetBody1(joint) != otherBody)) {
			for (void* contact = NewtonContactJointGetFirstContact (joint); contact; contact = NewtonContactJointGetNextContact (joint, contact)) {
				dVector point;
				dVector normal;	
				dVector tangentDir0;
				dVector tangentDir1;
				dVector contactForce;

				NewtonMaterial* const material = NewtonContactGetMaterial (contact);
				NewtonMaterialGetContactForce (material, m_newtonBody, &contactForce[0]);
				NewtonMaterialGetContactPositionAndNormal (material, m_newtonBody, &point[0], &normal[0]);

				force += contactForce;
				torque += (point - m_globalCentreOfMass) * contactForce;
			}
		}
	}
	}

	m_externalForce = force;
	m_externalTorque = torque;

	if (m_mass > 1.0e-3f) {
		m_invMass = 1.0f / m_mass;
		m_localInvInertia[0] = 1.0f / m_localInertia[0];
		m_localInvInertia[1] = 1.0f / m_localInertia[1];
		m_localInvInertia[2] = 1.0f / m_localInertia[2];
	} else {
		m_invMass = 0.0f;
		m_localInvInertia[0] = 0.0f;
		m_localInvInertia[1] = 0.0f;
		m_localInvInertia[2] = 0.0f;
	}
*/

	m_externalForce = dVector (0.0f, 0.0f, 0.0f, 0.0f);
	m_externalTorque = dVector (0.0f, 0.0f, 0.0f, 0.0f);

	m_maxExternalAccel2 = 50.0f * 50.0f;
	m_maxExternalAlpha2 = 100.0 * 100.0f;

	m_invMass = 0.0f;
	m_localInvInertia[0] = 0.0f;
	m_localInvInertia[1] = 0.0f;
	m_localInvInertia[2] = 0.0f;
	UpdateInertia();
}

void CustomVehicleControllerBodyStateContact::UpdateDynamicInputs()
{
	dAssert (0);
}

void CustomVehicleControllerBodyStateContact::ApplyNetForceAndTorque (dFloat invTimestep, const dVector& veloc, const dVector& omega)
{
	if (m_invMass > 0.0f) {
/*
		dVector force (m_externalForce + m_foceAcc);
		dVector torque (m_externalTorque + m_torqueAcc);
		dVector accel (force.Scale(m_invMass));
		dVector alpha (m_invInertia.RotateVector(torque));
		
		dFloat accelMag2 = accel % accel;
		dFloat alphaMag2 = alpha % alpha;
		for (int i = 0; (i < 8) && ((accelMag2 > m_maxExterenalAccel2) || (alphaMag2 > m_maxExterenalAlpha2)); i ++) {
			m_foceAcc = m_foceAcc.Scale (0.9f);
			m_torqueAcc = m_torqueAcc.Scale (0.9f);
			force = m_externalForce + m_foceAcc;
			torque = m_externalForce + m_torqueAcc;
			accel = force.Scale(m_invMass);
			alpha = m_invInertia.RotateVector(torque);
			accelMag2 = accel % accel;
			alphaMag2 = alpha % alpha;
		}
*/
		dAssert (0);
//		NewtonBodyAddForce(m_newtonBody, &m_foceAcc[0]);
//		NewtonBodyAddTorque(m_newtonBody, &m_torqueAcc[0]);
	}
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

	NewtonBodyGetForceAcc (body, &m_externalForce[0]);
	NewtonBodyGetTorqueAcc (body, &m_externalTorque[0]);
	m_externalForce.m_w = 0.0f;
	m_externalTorque.m_w = 0.0f;

/*
	dVector force;
	dVector torque;
	NewtonBodyGetForceAcc (body, &force[0]);
	NewtonBodyGetTorqueAcc (body, &torque[0]);
	const int maxContacts = int (sizeof (m_controller->m_externalContactJoints) / sizeof (m_controller->m_externalContactJoints[0]));
	for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint (body); joint && (m_controller->m_externalChassisContactCount < maxContacts); joint = NewtonBodyGetNextContactJoint (body, joint)) {
		NewtonBody* const otherBody = NewtonJointGetBody1(joint);
		dAssert (NewtonJointGetBody0(joint) == body);
		dAssert (otherBody != body);

		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;
		dFloat mass;
		NewtonBodyGetMassMatrix (otherBody, &mass, &Ixx, &Iyy, &Izz);
		if (mass > 0.0f) {
			int count = 0;
			NewtonWorldConvexCastReturnInfo contacts[8];
			void* contact = NewtonContactJointGetFirstContact (joint);
			if (contact) {
				CustomVehicleControllerBodyStateContact* const externalBody = m_controller->GetContactBody(otherBody);

				CustomVehicleControllerChassisContactJoint* const externalJoint = &m_controller->m_externalContactJoints[m_controller->m_externalChassisContactCount];
				externalJoint->Init(m_controller, externalBody, this);
				for (; contact && count < (sizeof (contacts) / sizeof (contacts[0])); contact = NewtonContactJointGetNextContact (joint, contact)) {
					dVector point;
					dVector normal;	
					dVector tangentDir0;
					dVector tangentDir1;
					dVector contactForce;

					NewtonMaterial* const material = NewtonContactGetMaterial (contact);
					NewtonMaterialGetContactForce (material, otherBody, &contactForce[0]);
					NewtonMaterialGetContactPositionAndNormal (material, otherBody, &point[0], &normal[0]);

					contacts[count].m_point[0] = point.m_x;
					contacts[count].m_point[1] = point.m_y;
					contacts[count].m_point[2] = point.m_z;
				
					contacts[count].m_normal[0] = normal.m_x;
					contacts[count].m_normal[1] = normal.m_y;
					contacts[count].m_normal[2] = normal.m_z;
					count ++;

					force += contactForce;
					torque += (point - m_globalCentreOfMass) * contactForce;

					externalBody->m_externalForce -= contactForce;
					externalBody->m_externalTorque -= (point - externalBody->m_globalCentreOfMass) * contactForce;
				}
				externalJoint->SetContacts (count, contacts);
				m_controller->m_externalChassisContactCount ++;
			}
		}
	}
*/

	dFloat frontSpeed = dMin(m_veloc % m_matrix[0], dFloat(50.0f));
	m_externalForce -= m_matrix[1].Scale (m_aerodynamicsDownForceCoefficient * frontSpeed * frontSpeed);

	m_globalCentreOfMass = m_matrix.TransformVector(m_localFrame.m_posit);
	UpdateInertia();
}

void CustomVehicleControllerBodyStateChassis::PutToSleep()
{
	NewtonBody* const body = m_controller->GetBody();

	dVector zero (0.0f, 0.0f, 0.0f, 0.0f); 
	NewtonBodySetForce (body, &zero[0]);
	NewtonBodySetTorque(body, &zero[0]);
	NewtonBodySetOmega(body, &zero[0]);
	NewtonBodySetVelocity(body, &zero[0]);
}

bool CustomVehicleControllerBodyStateChassis::IsSleeping () const
{
	const dFloat velocError = 1.0e-4f;
	dFloat mag2 = m_veloc % m_veloc;
	if (mag2 < velocError) {
		mag2 = m_omega % m_omega;
		if (mag2 < velocError) {
			const dFloat accelError = 1.0e-2f;
			NewtonBody* const body = m_controller->GetBody();
			dVector force;
			NewtonBodyGetForce (body, &force[0]);
			dVector accel (force.Scale(m_invMass));
			mag2 = accel % accel;
			if (mag2 < accelError) {
				dVector torque;
				NewtonBodyGetTorque(body, &torque[0]);
				dVector alpha (m_invInertia.RotateVector(torque));
				mag2 = alpha % alpha;
				return (mag2 < accelError) ? true : false;
			}
		}
	}
	return false;
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
	NewtonCollisionSetMode (m_controller->m_tireCastShape, 0);

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
	dMatrix identMatrix (dGetIdentityMatrix());
	NewtonCollisionSetMode (m_controller->m_tireCastShape, 1);
	NewtonCollisionSetMatrix (m_controller->m_tireCastShape, &identMatrix[0][0]);

	// get the collision shape
	m_shape = NewtonCompoundCollisionGetCollisionFromNode (vehShape, tireShapeNode);

	// update tire info
	UpdateInfo (tireInfo);

	// initialize the joints that connect this tire to other vehicle componets;
	m_chassisJoint.Init(m_controller, &m_controller->m_chassisState, this);
}

void CustomVehicleControllerBodyStateTire::UpdateInfo(const TireCreationInfo& tireInfo)
{
	const dMatrix& vehicleFrame = m_controller->m_chassisState.m_localFrame;

	// build a normalized size collision shape and scale to math the tire size, make it is also transparent to collision  
	NewtonCollisionSetScale (m_controller->m_tireCastShape, tireInfo.m_width, tireInfo.m_radio, tireInfo.m_radio);
	NewtonCollisionSetMode (m_controller->m_tireCastShape, 0);

	// calculate the location of the tire matrix
	m_localFrame = vehicleFrame * dYawMatrix(-3.141592f * 0.5f);
	m_localFrame.m_posit = tireInfo.m_location + m_localFrame.m_up.Scale (tireInfo.m_suspesionlenght);
	m_localFrame.m_posit.m_w = 1.0f;

	// restore the cast shape transform to identity
	dMatrix identMatrix (dGetIdentityMatrix());
	NewtonCollisionSetMode (m_controller->m_tireCastShape, 1);
	NewtonCollisionSetMatrix (m_controller->m_tireCastShape, &identMatrix[0][0]);

	// get the collision shape
	//m_shape = NewtonCompoundCollisionGetCollisionFromNode (vehShape, tireShapeNode);

	// initialize all constants
	m_userData = tireInfo.m_userData;
	m_mass = dMax (tireInfo.m_mass, m_controller->m_chassisState.m_mass / 50.0f); 
	m_invMass = 1.0f / m_mass;
	m_radio = tireInfo.m_radio;
	m_width = tireInfo.m_width;
	m_locationSign = dSign (m_localFrame.m_posit % m_localFrame.m_front);
	m_lateralStiffness = tireInfo.m_lateralStiffness;
	m_longitudialStiffness = tireInfo.m_longitudialStiffness;
	m_aligningMomentTrail = tireInfo.m_aligningMomentTrail;

	m_localInertia[0] = m_mass * (0.50f * m_radio * m_radio);
	m_localInertia[1] = m_mass * (0.25f * m_radio * m_radio + (1.0f / 12.0f) * m_width * m_width);
	m_localInertia[2] = m_localInertia[1];
	m_localInertia[3] = 0.0f;
	m_localInvInertia[0] = 1.0f / m_localInertia[0];
	m_localInvInertia[1] = 1.0f / m_localInertia[1];
	m_localInvInertia[2] = 1.0f / m_localInertia[2];
	m_localInvInertia[3] = 0.0f;

	m_restSprunMass = 0.0f;
	m_dampingRatio = tireInfo.m_dampingRatio;
	m_springStrength = tireInfo.m_springStrength;
	m_suspensionLenght = tireInfo.m_suspesionlenght;

	// initialize all local variables to default values
	m_brakeTorque = 0.0f;
	m_engineTorque = 0.0f;
	m_rotationalSpeed = 0.0f;
	m_rotationAngle = 0.0f;
	m_steeringAngle = 0.0f;

	m_tireLoad = dVector(0.0f, 0.0f, 0.0f, 0.0f);
	m_lateralForce = dVector(0.0f, 0.0f, 0.0f, 0.0f);
	m_longitudinalForce = dVector(0.0f, 0.0f, 0.0f, 0.0f);

	m_speed = 0.0f;
	m_posit = m_suspensionLenght;
	m_matrix = CalculateSteeringMatrix ();

	m_lateralSlip = 0.0f;
	m_longitudinalSlip = 0.0f;
	m_aligningTorque = 0.0f;
	m_contactCount = 0;

	UpdateInertia();
}

void CustomVehicleControllerBodyStateTire::GetInfo(TireCreationInfo& tireInfo) const
{
	tireInfo.m_location = m_localFrame.m_posit - m_localFrame.m_up.Scale (m_suspensionLenght);
	tireInfo.m_mass = m_mass;
	tireInfo.m_radio = m_radio;
	tireInfo.m_width = m_width;
	tireInfo.m_dampingRatio = m_dampingRatio;
	tireInfo.m_springStrength = m_springStrength;
	tireInfo.m_suspesionlenght = m_suspensionLenght;
	tireInfo.m_lateralStiffness = m_lateralStiffness;
	tireInfo.m_longitudialStiffness = m_longitudialStiffness;
	tireInfo.m_aligningMomentTrail = m_aligningMomentTrail;
	tireInfo.m_userData = m_userData;
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
	dMatrix localMatrix (CalculateLocalMatrix ());
	NewtonCollisionSetMatrix(m_shape, &localMatrix[0][0]);	
}


void CustomVehicleControllerBodyStateTire::Collide (CustomControllerConvexCastPreFilter& filter, dFloat timestepInv, int threadId)
{
	NewtonBody* const body = m_controller->GetBody();
	NewtonWorld* const world = NewtonBodyGetWorld(body);
	const dMatrix& controllerMatrix = m_controller->m_chassisState.m_matrix;

	dFloat posit0 = m_posit;

	m_posit = 0.0f;
	m_speed = 0.0f;
	dMatrix localMatrix (CustomVehicleControllerBodyStateTire::CalculateSteeringMatrix ());
	m_posit = m_suspensionLenght;

	dFloat hitParam;
	dMatrix tireMatrix (localMatrix * controllerMatrix);
	dVector rayDestination (tireMatrix.TransformVector(localMatrix.m_up.Scale(-m_suspensionLenght)));   

	m_contactCount = 0;
	NewtonWorldConvexCastReturnInfo contacts[4];

/*
NewtonCollision* xxx = NewtonCreateSphere(world, 1.0f, 0, NULL);
dMatrix xxxx0 (dGetIdentityMatrix());
dVector xxxx1 (0.0f, 0.0f, 0.0f, 0.0f);
xxxx0.m_posit.m_y = 2.0f;
//NewtonCollisionSetScale (xxx, 0.3f, 0.5, 0.3f);
int xxx3 = NewtonWorldConvexCast (world, &xxxx0[0][0], &xxxx1[0], xxx, &hitParam, &filter, CustomControllerConvexCastPreFilter::Prefilter, contacts, sizeof (contacts) / sizeof (contacts[0]), 0);
*/

	NewtonCollisionSetScale (m_controller->m_tireCastShape, m_width, m_radio, m_radio);
	int contactCount = NewtonWorldConvexCast (world, &tireMatrix[0][0], &rayDestination[0], m_controller->m_tireCastShape, &hitParam, &filter, CustomControllerConvexCastPreFilter::Prefilter, contacts, sizeof (contacts) / sizeof (contacts[0]), threadId);
	if (contactCount) {
		m_posit = hitParam * m_suspensionLenght;
		m_speed = (posit0 - m_posit) * timestepInv;
		for (int i = 1; i < contactCount; i ++) {
			int j = i;
			NewtonWorldConvexCastReturnInfo tmp (contacts[i]);
			for (; j && (contacts[j - 1].m_hitBody > tmp.m_hitBody); j --) {
				contacts[j] = contacts[j - 1];
			}
			contacts[j] = tmp;
		}

		for (int index = 0; (index < contactCount) && (m_contactCount < 2); ) {
			const NewtonBody* const body = contacts[index].m_hitBody;
			int count = 1;
			for (int j = index + 1; (j < contactCount) && (contacts[j].m_hitBody == body); j ++) {
				count ++;
			}
			CustomVehicleControllerTireContactJoint* const joint = &m_contactJoint[m_contactCount];
			CustomVehicleControllerBodyStateContact* const externalBody = m_controller->GetContactBody (body);
			joint->Init(m_controller, externalBody, this);
			joint->SetContacts (count, &contacts[index]);
			m_contactCount ++;
			index += count;
		}
	}

	// project contact to the surface of the tire shape
	for (int i = 0; i < m_contactCount; i ++) {
		CustomVehicleControllerTireContactJoint& contact = m_contactJoint[i];
		for (int j = 0; j < contact.m_contactCount; j ++) {
			dVector radius (contact.m_contactsPoint[j] - m_matrix[3]);
			radius -= m_matrix[0].Scale (m_matrix[0] % radius);
			dAssert ((radius % radius) > 0.0f);
			radius = radius.Scale (m_radio / dSqrt (radius % radius));
			contact.m_contactsPoint[j] = m_matrix[3] + radius;
		}
	}
}


void CustomVehicleControllerBodyStateTire::UpdateDynamicInputs(dFloat timestep)
{
	CustomVehicleControllerBodyStateChassis& chassis = m_controller->m_chassisState;

	m_matrix = CalculateSteeringMatrix() * chassis.m_matrix;
	m_globalCentreOfMass = m_matrix.m_posit;
	UpdateInertia();

	// get the velocity state for this tire
	dVector relPosit (m_matrix.m_posit - chassis.m_globalCentreOfMass);
	m_omega = chassis.m_omega + m_matrix[0].Scale (m_rotationalSpeed);
	m_veloc = chassis.m_veloc + chassis.m_omega * relPosit + m_matrix[1].Scale (m_speed);

	// set the initial force on this tire
	m_externalForce = chassis.m_gravity.Scale (m_mass);
	m_externalTorque = dVector (0.0f, 0.0f, 0.0f, 0.0f);

	m_tireLoad = dVector(0.0f, 0.0f, 0.0f, 0.0f);
	m_lateralForce = dVector(0.0f, 0.0f, 0.0f, 0.0f);
	m_longitudinalForce = dVector(0.0f, 0.0f, 0.0f, 0.0f);

	// calculate force an torque generate by the suspension
	if (m_contactCount) {
		dFloat distance = m_suspensionLenght - m_posit;
		dAssert (distance >= 0.0f);
		dAssert (distance <= m_suspensionLenght);
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
		dVector torque (relPosit * m_tireLoad);
		chassis.m_externalForce += m_tireLoad;
		chassis.m_externalTorque += torque;

		// the spring apply the same force in the opposite direction to the tire
		m_externalForce -= m_tireLoad;
	}
}


void CustomVehicleControllerBodyStateTire::CalculateRollingResistance (dFloat topSpeed)
{
	m_maxAngularVelocity = topSpeed / m_radio;
}


void CustomVehicleControllerBodyStateTire::ApplyNetForceAndTorque (dFloat invTimestep, const dVector& veloc, const dVector& omega)
{
	CustomVehicleControllerBodyState::ApplyNetForceAndTorque (invTimestep, veloc, omega);
	const CustomVehicleControllerBodyStateChassis& chassis = m_controller->m_chassisState;

	// integrate tires angular velocity
	dVector relOmega (m_omega - chassis.m_omega);
	m_rotationalSpeed = relOmega % m_matrix[0];
	if (dAbs (m_rotationalSpeed) > m_maxAngularVelocity) {
		m_rotationalSpeed *= 0.9f;
		m_omega = m_matrix[0].Scale (m_rotationalSpeed) + chassis.m_omega;
	}
	m_rotationAngle = dMod (m_rotationAngle + m_rotationalSpeed / invTimestep, 2.0f * 3.141592f);
}

