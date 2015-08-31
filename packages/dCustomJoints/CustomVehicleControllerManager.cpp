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

// replaced the tire model base of Pacejkas (which seems to always produce a very poor tire behavior) 
// with the the method use in this paper 
// http://www.ricblues.nl/techniek/Technisch%20Specialist%2093430/6%20Remgedrag%20ABS%20weggedrag/Carsim%20-%20remsimulatieprogramma/Handleiding%20carsim.pdf
// basically th replace the Pajecka equation with the with the two series expansions 
// f = x - |x| * x / 3 + x * x * x / 27
// T = x - |x| * x + x * x * x / 3 - |x| * x * x * x / 27 
// they also have a better tire friction model that the naive friction circle projection

// for the differential equation I am using information from here
// http://web.mit.edu/2.972/www/reports/differential/differential.html

// NewtonCustomJoint.cpp: implementation of the NewtonCustomJoint class.
//
//////////////////////////////////////////////////////////////////////
#include <CustomJointLibraryStdAfx.h>
#include <CustomJoint.h>

#include <CustomHinge.h>
#include <CustomVehicleControllerManager.h>

class CustomVehicleController::dWeightDistibutionSolver: public dSymmetricBiconjugateGradientSolve
{
	public:
	dWeightDistibutionSolver()
		:dSymmetricBiconjugateGradientSolve()
		, m_count(0)
	{
	}

	virtual void MatrixTimeVector(dFloat64* const out, const dFloat64* const v) const
	{
		dComplemtaritySolver::dJacobian invMassJacobians;
		invMassJacobians.m_linear = dVector(0.0f, 0.0f, 0.0f, 0.0f);
		invMassJacobians.m_angular = dVector(0.0f, 0.0f, 0.0f, 0.0f);
		for (int i = 0; i < m_count; i++) {
			invMassJacobians.m_linear += m_invMassJacobians[i].m_linear.Scale(dFloat(v[i]));
			invMassJacobians.m_angular += m_invMassJacobians[i].m_angular.Scale(dFloat(v[i]));
		}

		for (int i = 0; i < m_count; i++) {
			out[i] = m_diagRegularizer[i] * v[i] + invMassJacobians.m_linear % m_jacobians[i].m_linear + invMassJacobians.m_angular % m_jacobians[i].m_angular;
		}
	}

	virtual bool InversePrecoditionerTimeVector(dFloat64* const out, const dFloat64* const v) const
	{
		for (int i = 0; i < m_count; i++) {
			out[i] = v[i] * m_invDiag[i];
		}
		return true;
	}

	dComplemtaritySolver::dJacobian m_jacobians[256];
	dComplemtaritySolver::dJacobian m_invMassJacobians[256];
	dFloat m_invDiag[256];
	dFloat m_diagRegularizer[256];
	int m_count;
};


class CustomVehicleController::BodyPartTire::WheelJoint: public CustomJoint
{
	public:
	WheelJoint (const dMatrix& pinAndPivotFrame, NewtonBody* const tire, NewtonBody* const parentBody, BodyPartTire* const tireData)
		:CustomJoint (6, tire, parentBody)
		,m_data (tireData)
		,m_targetAngle(0.0f)
		,m_restSprunMass(0.0f)
	{
		CalculateLocalMatrix (pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
	}

	void ApplySuspetionForce(dFloat timestep, dFloat posit)
	{
		
	}

	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		dMatrix matrix0;
		dMatrix matrix1;

		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		CalculateGlobalMatrix(matrix0, matrix1);

		dFloat posit = (matrix0.m_posit - matrix1.m_posit) % matrix1.m_up;

		// Restrict the movement on the pivot point along all two orthonormal axis direction perpendicular to the motion
		const dVector& p0 = matrix0.m_posit;
		dVector p1(matrix1.m_posit + matrix1.m_up.Scale(posit));
		NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1.m_front[0]);
		NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1.m_right[0]);

		if (posit >= m_data->m_data.m_suspesionlenght) {
			NewtonUserJointAddLinearRow(m_joint, &p0[0], &p0[0], &matrix1.m_up[0]);
			NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
		} else if (posit <= 0.0f) {
			NewtonUserJointAddLinearRow(m_joint, &p0[0], &p0[0], &matrix1.m_up[0]);
			NewtonUserJointSetRowMinimumFriction (m_joint, 0.0f);
		}

		dMatrix matrix1_1;
		matrix1_1.m_up = matrix1.m_up;
		matrix1_1.m_right = matrix0.m_front * matrix1.m_up;
		matrix1_1.m_right = matrix1_1.m_right.Scale(1.0f / dSqrt(matrix1_1.m_right % matrix1_1.m_right));
		matrix1_1.m_front = matrix1_1.m_up * matrix1_1.m_right;

		dVector omega0(0.0f, 0.0f, 0.0f, 0.0f);
		dVector omega1(0.0f, 0.0f, 0.0f, 0.0f);
		NewtonBodyGetOmega(m_body0, &omega0[0]);
		if (m_body1) {
			NewtonBodyGetOmega(m_body1, &omega1[0]);
		}
		dVector relOmega(omega0 - omega1);

		dFloat angle = -CalculateAngle(matrix0.m_front, matrix1_1.m_front, matrix1_1.m_right);
		dFloat omega = (relOmega % matrix1_1.m_right);
		dFloat alphaError = -(angle + omega * timestep) / (timestep * timestep);
		NewtonUserJointAddAngularRow(m_joint, -angle, &matrix1_1.m_right[0]);
		NewtonUserJointSetRowAcceleration(m_joint, alphaError);
		NewtonUserJointSetRowStiffness(m_joint, 1.0f);

		dFloat steerAngle = m_targetAngle - CalculateAngle (matrix1.m_front, matrix1_1.m_front, matrix1_1.m_up);
		dFloat steerOmega = (relOmega % matrix1_1.m_up);
		dFloat alphaSteerError = (steerAngle - steerOmega * timestep) / (timestep * timestep);
		NewtonUserJointAddAngularRow (m_joint, -steerAngle, &matrix1_1.m_up[0]);
		NewtonUserJointSetRowAcceleration(m_joint, alphaSteerError);
		NewtonUserJointSetRowStiffness(m_joint, 1.0f);
		
		dVector tireVeloc;
		dVector chassisCom;
		dVector chassisVeloc;
		dMatrix chassisMatrix;
		NewtonBodyGetVelocity(GetBody0(), &tireVeloc[0]);
		NewtonBodyGetPointVelocity(GetBody1(), &p0[0], &chassisVeloc[0]);
		NewtonBodyGetCentreOfMass(GetBody1(), &chassisCom[0]);
		NewtonBodyGetMatrix(GetBody1(), &chassisMatrix[0][0]);

		posit = dClamp(posit, 0.0f, m_data->m_data.m_suspesionlenght);
		chassisCom = p0 - chassisMatrix.TransformVector(chassisCom);
		dFloat speed = (tireVeloc - chassisVeloc) % matrix1.m_up;
		dFloat load = - NewtonCalculateSpringDamperAcceleration (timestep, m_data->m_data.m_springStrength, posit, m_data->m_data.m_dampingRatio, speed);
		dVector force (matrix1.m_up.Scale (load * m_restSprunMass));
		dVector torque (chassisCom  * force);

		NewtonBodyAddForce(GetBody1(), &force[0]);
		NewtonBodyAddTorque(GetBody1(), &torque[0]);
		force = force.Scale (-1.0f);
		NewtonBodyAddForce(GetBody0(), &force[0]);
	}

	BodyPartTire* m_data;
	dFloat m_targetAngle;
	dFloat m_restSprunMass;
};


CustomVehicleController::BodyPartTire::BodyPartTire()
	:BodyPart()
{

}

CustomVehicleController::BodyPartTire::~BodyPartTire()
{
}

void CustomVehicleController::BodyPartTire::Init (BodyPart* const parentPart, const dMatrix& locationInGlobaSpase, const CreationInfo& info)
{
	m_data = info;
	m_parent = parentPart;
	m_userData = info.m_userData;
	m_controller = parentPart->m_controller;

	NewtonWorld* const world = ((CustomVehicleControllerManager*)m_controller->GetManager())->GetWorld();
	NewtonCollisionSetScale(m_controller->m_tireCastShape, m_data.m_width, m_data.m_radio, m_data.m_radio);

	// create the rigid body that will make this bone
	dFloat dir = dSign(m_data.m_aligningPinDir);
	dMatrix matrix (dYawMatrix(0.5f * 3.1415927f* dir) * locationInGlobaSpase);
	m_body = NewtonCreateDynamicBody(world, m_controller->m_tireCastShape, &matrix[0][0]);

	// save the user data with the bone body (usually the visual geometry)
	//NewtonBodySetUserData(m_body, this);

	// set the standard force and torque call back
	NewtonBodySetForceAndTorqueCallback(m_body, m_controller->m_forceAndTorque);

	// tire are highly non linear, sung spherical inertia matrix make the calculation more accurate 
	dFloat inertia = 2.0f * m_data.m_mass * m_data.m_radio * m_data.m_radio / 5.0f;
	NewtonBodySetMassMatrix (m_body, m_data.m_mass, inertia, inertia, inertia);

	//NewtonBodySetMaterialGroupID (body, m_material);
	//NewtonCollisionSetUserID(collision, definition.m_bodyPartID);
	m_joint = new WheelJoint (matrix, m_body, parentPart->m_body, this);
}


#if 0
#define VEHICLE_SLEEP_COUNTER										16
#define VEHICLE_CONTROLLER_MAX_BODIES								32
#define VEHICLE_CONTROLLER_MAX_JOINTS								64
#define VEHICLE_CONTROLLER_MAX_JACOBIANS_PAIRS						(VEHICLE_CONTROLLER_MAX_JOINTS * 4)
#define VEHICLE_SIDESLEP_NORMALIZED_FRICTION_AT_MAX_SLIP_ANGLE		dFloat(0.75f)
#define VEHICLE_SIDESLEP_NORMALIZED_FRICTION_AT_MAX_SIDESLIP_RATIO	dFloat(0.95f)


CustomVehicleControllerTireCollisionFilter::CustomVehicleControllerTireCollisionFilter (const CustomVehicleController* const controller)
	:CustomControllerConvexCastPreFilter(controller->GetBody())
	,m_controller(controller)
{

}


class CustomVehicleController::dTireForceSolverSolver: public dComplemtaritySolver
{
	public:
	dTireForceSolverSolver(CustomVehicleController* const controller, dFloat timestep, int threadId)
		:dComplemtaritySolver()
		,m_controller(controller)
	{
		m_controller->m_externalContactStatesCount = 0;
		m_controller->m_freeContactList = m_controller->m_externalContactStatesPool.GetFirst();

		// apply all external forces and torques to chassis and all tire velocities
		dFloat timestepInv = 1.0f / timestep;
		m_controller->m_chassisState.UpdateDynamicInputs();

		m_controller->m_sleepCounter --;
		bool isSleeping = m_controller->IsSleeping();
		if (isSleeping) {
			if (m_controller->m_sleepCounter > 0) {
				isSleeping = false;
			}
		} else {
			m_controller->m_sleepCounter = VEHICLE_SLEEP_COUNTER;
		}

		if (isSleeping) {
			m_controller->m_chassisState.PutToSleep();

		} else {
			dAssert (controller->m_contactFilter);
			for (dList<CustomVehicleControllerBodyStateTire>::dListNode* node = m_controller->m_tireList.GetFirst(); node; node = node->GetNext()) {
				CustomVehicleControllerBodyStateTire* const tire = &node->GetInfo();
				tire->Collide(*controller->m_contactFilter, timestepInv, threadId);
				tire->UpdateDynamicInputs(timestep);
			}

			// update all components
			if (m_controller->m_engine) {
				m_controller->m_engine->Update(timestep);
			}

			if (m_controller->m_steering) {
				m_controller->m_steering->Update(timestep);
			}

			if (m_controller->m_handBrakes) {
				m_controller->m_handBrakes->Update(timestep);
			}

			if (m_controller->m_brakes) {
				m_controller->m_brakes->Update(timestep);
			}

			// Get the number of active joints for this integration step
			int bodyCount = 0;
			for (dList<CustomVehicleControllerBodyState*>::dListNode* stateNode = m_controller->m_stateList.GetFirst(); stateNode; stateNode = stateNode->GetNext()) {
				m_bodyArray[bodyCount] = stateNode->GetInfo();
				dAssert (bodyCount < int (sizeof (m_bodyArray) / sizeof(m_bodyArray[0])));
				bodyCount ++;
			}

			for (int i = 0; i < m_controller->m_externalContactStatesCount; i ++) {
				m_bodyArray[bodyCount] = m_controller->m_externalContactStates[i];
				dAssert (bodyCount < int (sizeof (m_bodyArray) / sizeof(m_bodyArray[0])));
				bodyCount ++;
			}

			int jointCount = GetActiveJoints();
			BuildJacobianMatrix (jointCount, m_jointArray, timestep, m_jacobianPairArray, m_jacobianColumn, sizeof (m_jacobianPairArray)/ sizeof (m_jacobianPairArray[0]));
			CalculateReactionsForces (bodyCount, m_bodyArray, jointCount, m_jointArray, timestep, m_jacobianPairArray, m_jacobianColumn);
		}
	}

	~dTireForceSolverSolver()
	{
	}

	int GetActiveJoints()
	{
		int jointCount = 0;

		// add all contact joints if any
		for (dList<CustomVehicleControllerBodyStateTire>::dListNode* node = m_controller->m_tireList.GetFirst(); node; node = node->GetNext()) {
			CustomVehicleControllerBodyStateTire* const tire = &node->GetInfo();
			for (int i = 0; i < tire->m_contactCount; i ++) {
				m_jointArray[jointCount] = &tire->m_contactJoint[i];
				jointCount ++;
				dAssert (jointCount < VEHICLE_CONTROLLER_MAX_JOINTS);
			}
		}

		// add the joints that connect tire to chassis
		for (dList<CustomVehicleControllerBodyStateTire>::dListNode* node = m_controller->m_tireList.GetFirst(); node; node = node->GetNext()) {
			CustomVehicleControllerBodyStateTire* const tire = &node->GetInfo();
			m_jointArray[jointCount] = &tire->m_chassisJoint;
			jointCount ++;
			dAssert (jointCount < VEHICLE_CONTROLLER_MAX_JOINTS);
		}

		if (m_controller->m_engine) {
			jointCount += m_controller->m_engine->AddDifferentialJoints(&m_jointArray[jointCount]);
		}

		for (dList<CustomVehicleControllerEngineDifferencialJoint>::dListNode* node = m_controller->m_tankTireLinks.GetFirst(); node; node = node->GetNext()) {
			m_jointArray[jointCount] = &node->GetInfo();
			jointCount ++;
			dAssert (jointCount < VEHICLE_CONTROLLER_MAX_JOINTS);
		}
		//for (int i = 0; i < m_trackSteeringCount; i ++) {
		//	constraintArray[jointCount] = &m_trackSteering[i];
		//	jointCount ++;
		//}

		return jointCount;
	}

	dBodyState* m_bodyArray[VEHICLE_CONTROLLER_MAX_BODIES];
	dBilateralJoint* m_jointArray[VEHICLE_CONTROLLER_MAX_JOINTS];
	dJacobianColum m_jacobianColumn[VEHICLE_CONTROLLER_MAX_JACOBIANS_PAIRS];
	dJacobianPair m_jacobianPairArray[VEHICLE_CONTROLLER_MAX_JACOBIANS_PAIRS];
	CustomVehicleController* m_controller;
};





void CustomVehicleControllerManager::DrawSchematic (const CustomVehicleController* const controller, dFloat scale) const
{
	controller->DrawSchematic(scale);
}

void CustomVehicleControllerManager::DrawSchematicCallback (const CustomVehicleController* const controller, const char* const partName, dFloat value, int pointCount, const dVector* const lines) const
{

}






const CustomVehicleControllerBodyStateChassis& CustomVehicleController::GetChassisState () const
{
	return m_chassisState;
}

dFloat CustomVehicleController::GetAerodynamicsDowforceCoeficient () const
{
	return m_chassisState.GetAerodynamicsDowforceCoeficient();
}

void CustomVehicleController::SetAerodynamicsDownforceCoefficient (dFloat maxDownforceInGravity, dFloat topSpeed)
{
	m_chassisState.SetAerodynamicsDownforceCoefficient (maxDownforceInGravity, topSpeed);
}


void CustomVehicleController::SetDryRollingFrictionTorque (dFloat dryRollingFrictionTorque)
{
	m_chassisState.SetDryRollingFrictionTorque (dryRollingFrictionTorque);
}

dFloat CustomVehicleController::GetDryRollingFrictionTorque () const
{
	return m_chassisState.GetDryRollingFrictionTorque();
}


CustomVehicleControllerBodyStateTire* CustomVehicleController::GetFirstTire () const
{
	return m_tireList.GetFirst() ? &m_tireList.GetFirst()->GetInfo() : NULL;
}

CustomVehicleControllerBodyStateTire* CustomVehicleController::GetNextTire (CustomVehicleControllerBodyStateTire* const tire) const
{
	dList<CustomVehicleControllerBodyStateTire>::dListNode* const tireNode = m_tireList.GetNodeFromInfo(*tire);
	return tireNode->GetNext() ? &tireNode->GetNext()->GetInfo() : NULL;
}






CustomVehicleControllerComponentEngine* CustomVehicleController::GetEngine() const
{
	return m_engine;
}

CustomVehicleControllerComponentSteering* CustomVehicleController::GetSteering() const
{
	return m_steering;
}

CustomVehicleControllerComponentBrake* CustomVehicleController::GetBrakes() const
{
	return m_brakes;
}

CustomVehicleControllerComponentBrake* CustomVehicleController::GetHandBrakes() const
{
	return m_handBrakes;
}

void CustomVehicleController::SetEngine(CustomVehicleControllerComponentEngine* const engine)
{
	if (m_engine) {
		delete m_engine;
	}
	m_engine = engine;
}

void CustomVehicleController::SetSteering(CustomVehicleControllerComponentSteering* const steering)
{
	if (m_steering) {
		delete m_steering;
	}
	m_steering = steering;
}

void CustomVehicleController::SetContactFilter(CustomVehicleControllerTireCollisionFilter* const filter)
{
	if (m_contactFilter) {
		delete m_contactFilter;
	}
	m_contactFilter = filter;
}

void CustomVehicleController::SetBrakes(CustomVehicleControllerComponentBrake* const brakes)
{
	if (m_brakes) {
		delete m_brakes;
	}
	m_brakes = brakes;
}

void CustomVehicleController::SetHandBrakes(CustomVehicleControllerComponentBrake* const brakes)
{
	if (m_handBrakes) {
		delete m_handBrakes;
	}
	m_handBrakes = brakes;
}


void CustomVehicleController::LinksTiresKinematically (int count, CustomVehicleControllerBodyStateTire** const tires)
{
	dFloat radio0 = tires[0]->m_radio;
	for (int i = 1; i < count; i ++) {
		CustomVehicleControllerEngineDifferencialJoint* const link = &m_tankTireLinks.Append()->GetInfo();
		link->Init(this, tires[0], tires[i]);
		link->m_radio0 = radio0;
		link->m_radio1 = tires[i]->m_radio;
	}
}



CustomVehicleControllerBodyStateContact* CustomVehicleController::GetContactBody (const NewtonBody* const body)
{
	for (int i = 0; i < m_externalContactStatesCount; i ++) {
		if (m_externalContactStates[i]->m_newtonBody == body) {
			return m_externalContactStates[i];
		}
	}

	dAssert (m_externalContactStatesPool.GetCount() < 32);
	if (!m_freeContactList) {
		m_freeContactList = m_externalContactStatesPool.Append();
	}
	CustomVehicleControllerBodyStateContact* const externalBody = &m_freeContactList->GetInfo();
	m_freeContactList = m_freeContactList->GetNext(); 
	externalBody->Init (this, body);
	m_externalContactStates[m_externalContactStatesCount] = externalBody;
	m_externalContactStatesCount ++;
	dAssert (m_externalContactStatesCount < int (sizeof (m_externalContactStates) / sizeof (m_externalContactStates[0])));

	return externalBody;
}


bool CustomVehicleController::IsSleeping()
{
	bool inputChanged = (m_engine && m_engine->ParamChanged()) || (m_steering && m_steering->ParamChanged()) || (m_brakes && m_brakes->ParamChanged()) || (m_handBrakes && m_handBrakes->ParamChanged()); 
	if (inputChanged) {
		return false;
	}

	if (!m_chassisState.IsSleeping()) {
		return false;
	}

	return true;
}


void CustomVehicleController::DrawSchematic (dFloat scale) const
{
	dVector array [32];

	dMatrix projectionMatrix (dGetIdentityMatrix());
	projectionMatrix[0][0] = scale;
	projectionMatrix[1][1] = 0.0f;
	projectionMatrix[2][1] = scale;
	projectionMatrix[2][2] = 0.0f;
	CustomVehicleControllerManager* const manager = (CustomVehicleControllerManager*)GetManager();
	const dMatrix& chassisMatrix = m_chassisState.GetMatrix();
	const dMatrix& chassisFrameMatrix = m_chassisState.GetLocalMatrix();
	dMatrix worldToComMatrix ((chassisFrameMatrix * chassisMatrix).Inverse() * projectionMatrix);

	{
		// draw vehicle chassis
		dVector p0 (D_CUSTOM_LARGE_VALUE, D_CUSTOM_LARGE_VALUE, D_CUSTOM_LARGE_VALUE, 0.0f);
		dVector p1 (-D_CUSTOM_LARGE_VALUE, -D_CUSTOM_LARGE_VALUE, -D_CUSTOM_LARGE_VALUE, 0.0f);
		
		for (dList<CustomVehicleControllerBodyStateTire>::dListNode* node = m_tireList.GetFirst(); node; node = node->GetNext()) {
			CustomVehicleControllerBodyStateTire* const tire = &node->GetInfo();
			dMatrix matrix (tire->CalculateSteeringMatrix() * m_chassisState.GetMatrix());
			dVector p (worldToComMatrix.TransformVector(matrix.m_posit));
			p0 = dVector (dMin (p.m_x, p0.m_x), dMin (p.m_y, p0.m_y), dMin (p.m_z, p0.m_z), 1.0f);
			p1 = dVector (dMax (p.m_x, p1.m_x), dMax (p.m_y, p1.m_y), dMax (p.m_z, p1.m_z), 1.0f);
		}

		array[0] = dVector (p0.m_x, p0.m_y, p0.m_z, 1.0f);
		array[1] = dVector (p1.m_x, p0.m_y, p0.m_z, 1.0f);
		array[2] = dVector (p1.m_x, p1.m_y, p0.m_z, 1.0f);
		array[3] = dVector (p0.m_x, p1.m_y, p0.m_z, 1.0f);
		manager->DrawSchematicCallback(this, "chassis", 0, 4, array);
	}

	{
		// draw vehicle tires
		for (dList<CustomVehicleControllerBodyStateTire>::dListNode* node = m_tireList.GetFirst(); node; node = node->GetNext()) {

			CustomVehicleControllerBodyStateTire* const tire = &node->GetInfo();

			dFloat width = tire->m_width * 0.5f;
			dFloat radio = tire->m_radio;
			dMatrix matrix (tire->CalculateSteeringMatrix() * m_chassisState.GetMatrix());

			array[0] = worldToComMatrix.TransformVector(matrix.TransformVector(dVector ( width, 0.0f,  radio, 0.0f)));
			array[1] = worldToComMatrix.TransformVector(matrix.TransformVector(dVector ( width, 0.0f, -radio, 0.0f)));
			array[2] = worldToComMatrix.TransformVector(matrix.TransformVector(dVector (-width, 0.0f, -radio, 0.0f)));
			array[3] = worldToComMatrix.TransformVector(matrix.TransformVector(dVector (-width, 0.0f,  radio, 0.0f)));
			manager->DrawSchematicCallback(this, "tire", 0, 4, array);
		}
	}

	{
		// draw vehicle velocity
		//dVector veloc1;
		//NewtonBodyGetVelocity(GetBody(), &veloc[0]);
		dVector veloc (m_chassisState.GetVelocity());
//dVector xxx (veloc1 - veloc);
//dAssert (dAbs(xxx % xxx) < 1.0e-3f);

		dVector localVelocity (chassisFrameMatrix.UnrotateVector (chassisMatrix.UnrotateVector (veloc)));
		localVelocity.m_y = 0.0f;

		localVelocity = projectionMatrix.RotateVector(localVelocity);

		array[0] = dVector (0.0f, 0.0f, 0.0f, 0.0f);
		array[1] = localVelocity.Scale (0.25f);
		manager->DrawSchematicCallback(this, "velocity", 0, 2, array);
	}


	{

		dFloat scale (2.0f / (m_chassisState.GetMass() * m_chassisState.m_gravityMag));
		// draw vehicle forces
		for (dList<CustomVehicleControllerBodyStateTire>::dListNode* node = m_tireList.GetFirst(); node; node = node->GetNext()) {

			CustomVehicleControllerBodyStateTire* const tire = &node->GetInfo();
			//dVector p0 (tire->GetCenterOfMass());
			dMatrix matrix (tire->CalculateSteeringMatrix() * m_chassisState.GetMatrix());

//dTrace (("(%f %f %f) (%f %f %f)\n", p0.m_x, p0.m_y, p0.m_z, matrix.m_posit.m_x, matrix.m_posit.m_y, matrix.m_posit.m_z ));
			dVector origin (worldToComMatrix.TransformVector(matrix.m_posit));

			dVector lateralForce (chassisFrameMatrix.UnrotateVector(chassisMatrix.UnrotateVector(tire->GetLateralForce())));
			lateralForce = lateralForce.Scale (-scale);
			lateralForce = projectionMatrix.RotateVector (lateralForce);
//dTrace (("(%f %f %f)\n", lateralForce.m_x, lateralForce.m_y, lateralForce.m_z ));

			array[0] = origin;
			array[1] = origin + lateralForce;
			manager->DrawSchematicCallback(this, "lateralForce", 0, 2, array);


			dVector longitudinalForce (chassisFrameMatrix.UnrotateVector(chassisMatrix.UnrotateVector(tire->GetLongitudinalForce())));
			longitudinalForce = longitudinalForce.Scale (-scale);
			longitudinalForce = projectionMatrix.RotateVector (longitudinalForce);
			//dTrace (("(%f %f %f)\n", lateralForce.m_x, lateralForce.m_y, lateralForce.m_z ));

			array[0] = origin;
			array[1] = origin + longitudinalForce;
			manager->DrawSchematicCallback(this, "longitudinalForce", 0, 2, array);


//			dVector p2 (p0 - tire->GetLateralForce().Scale (scale));

/*
			// offset the origin of of tire force so that they are visible
			const dMatrix& tireMatrix = tire.GetLocalMatrix ();
			p0 += chassis.GetMatrix()[2].Scale ((tireMatrix.m_posit.m_z > 0.0f ? 1.0f : -1.0f) * 0.25f);

			// draw the tire load 
			dVector p1 (p0 + tire.GetTireLoad().Scale (scale));
			glColor3f (0.0f, 0.0f, 1.0f);
			glVertex3f (p0.m_x, p0.m_y, p0.m_z);
			glVertex3f (p1.m_x, p1.m_y, p1.m_z);

			// show tire lateral force
			dVector p2 (p0 - tire.GetLateralForce().Scale (scale));
			glColor3f(1.0f, 0.0f, 0.0f);
			glVertex3f (p0.m_x, p0.m_y, p0.m_z);
			glVertex3f (p2.m_x, p2.m_y, p2.m_z);

			// show tire longitudinal force
			dVector p3 (p0 - tire.GetLongitudinalForce().Scale (scale));
			glColor3f(0.0f, 1.0f, 0.0f);
			glVertex3f (p0.m_x, p0.m_y, p0.m_z);
			glVertex3f (p3.m_x, p3.m_y, p3.m_z);
*/
		}
	}
}

#endif


CustomVehicleControllerManager::CustomVehicleControllerManager(NewtonWorld* const world)
	:CustomControllerManager<CustomVehicleController> (world, VEHICLE_PLUGIN_NAME)
{
}

CustomVehicleControllerManager::~CustomVehicleControllerManager()
{
}



void CustomVehicleControllerManager::DestroyController(CustomVehicleController* const controller)
{
	controller->Cleanup();
	CustomControllerManager<CustomVehicleController>::DestroyController(controller);
}



void CustomVehicleController::PreUpdate(dFloat timestep, int threadIndex)
{
	if (m_finalized) {
//		dTireForceSolverSolver tireSolver(this, timestep, threadIndex);
	}
}

void CustomVehicleController::PostUpdate(dFloat timestep, int threadIndex)
{
	if (m_finalized) {
		//NewtonBody* const body = GetBody();
		//NewtonBodyGetMatrix(body, &m_chassisState.m_matrix[0][0]);
/*
		int stack = 1;
		void* stackPool[32];
		stackPool[0] = NewtonSkeletonContainerGetRoot (m_skeleton);
		while (stack) {
			stack --;
			void* const node = stackPool[stack];
			NewtonBody* const body = NewtonSkeletonContainerGetBodyFromNode (m_skeleton, node);
//			tire->UpdateTransform();
			for (void* ptr = NewtonSkeletonContainerFirstChild (m_skeleton, node); ptr; ptr = NewtonSkeletonContainerNextSibling (m_skeleton, ptr)) {
				stackPool[stack] = ptr;
				stack ++;
			}
		}
*/	
	}
}

CustomVehicleController* CustomVehicleControllerManager::CreateVehicle(NewtonBody* const body, const dMatrix& vehicleFrame, NewtonApplyForceAndTorque forceAndTorque, void* const userData)
{
	dAssert (0);
//	CustomVehicleController* const controller = CreateController();
//	controller->Init(body, vehicleFrame, gravityVector);
//	return controller;
return NULL;	
}

CustomVehicleController* CustomVehicleControllerManager::CreateVehicle(NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, NewtonApplyForceAndTorque forceAndTorque, void* const userData)
{
	CustomVehicleController* const controller = CreateController();
	controller->Init(chassisShape, vehicleFrame, mass, forceAndTorque, userData);
	return controller;
}

void CustomVehicleController::Init(NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, NewtonApplyForceAndTorque forceAndTorque, void* const userData)
{
	CustomVehicleControllerManager* const manager = (CustomVehicleControllerManager*)GetManager();
	NewtonWorld* const world = manager->GetWorld();

	// create a body and an call the low level init function
	dMatrix locationMatrix(dGetIdentityMatrix());
	NewtonBody* const body = NewtonCreateDynamicBody(world, chassisShape, &locationMatrix[0][0]);

	// set vehicle mass, inertia and center of mass
	NewtonBodySetMassProperties(body, mass, chassisShape);

	// initialize 
	Init(body, vehicleFrame, forceAndTorque, userData);
}

void CustomVehicleController::Init(NewtonBody* const body, const dMatrix& vehicleFrame, NewtonApplyForceAndTorque forceAndTorque, void* const userData)
{
	m_body = body;
	m_finalized = false;
	m_localFrame = vehicleFrame;
	m_forceAndTorque = forceAndTorque;
//	m_externalContactStatesCount = 0;
//	m_sleepCounter = VEHICLE_SLEEP_COUNTER;
//	m_freeContactList = m_externalContactStatesPool.GetFirst();
	
	
	CustomVehicleControllerManager* const manager = (CustomVehicleControllerManager*)GetManager();
	NewtonWorld* const world = manager->GetWorld();

	// set linear and angular drag to zero
	dVector drag(0.0f, 0.0f, 0.0f, 0.0f);
	NewtonBodySetLinearDamping(m_body, 0);
	NewtonBodySetAngularDamping(m_body, &drag[0]);

	// set the standard force and torque call back
	NewtonBodySetForceAndTorqueCallback(body, m_forceAndTorque);

	// create the normalized size tire shape
	m_tireCastShape = NewtonCreateChamferCylinder(world, 0.5f, 1.0f, 0, NULL);


/*
	m_chassisState.Init(this, vehicleFrame);

	m_stateList.Append(&m_chassisState);


	// initialize all components to empty
	m_engine = NULL;
	m_brakes = NULL;
	m_steering = NULL;
	m_handBrakes = NULL;
	m_contactFilter = new CustomVehicleControllerTireCollisionFilter(this);

	SetDryRollingFrictionTorque(100.0f / 4.0f);
	SetAerodynamicsDownforceCoefficient(0.5f * dSqrt(gravityVector % gravityVector), 60.0f * 0.447f);
*/

	m_collisionAggregate = NewtonCollisionAggregateCreate(world);
	NewtonCollisionAggregateSetSelfCollision (m_collisionAggregate, 0);
	NewtonCollisionAggregateAddBody (m_collisionAggregate, m_body);

	m_skeleton = NewtonSkeletonContainerCreate(world, m_body, NULL);

	m_chassis.Init(this, userData);
	m_bodyPartsList.Append(&m_chassis);
}

void CustomVehicleController::Cleanup()
{
//	SetBrakes(NULL);
//	SetEngine(NULL);
//	SetSteering(NULL);
//	SetHandBrakes(NULL);
//	SetContactFilter(NULL);
	NewtonDestroyCollision(m_tireCastShape);
}


dList<CustomVehicleController::BodyPart*>::dListNode* CustomVehicleController::GetFirstPart() const
{
	return m_bodyPartsList.GetFirst();
}

dList<CustomVehicleController::BodyPart*>::dListNode* CustomVehicleController::GetNextPart(dList<BodyPart*>::dListNode* const part) const
{
	return part->GetNext();
}

void CustomVehicleController::SetCenterOfGravity(const dVector& comRelativeToGeomtriCenter)
{
	NewtonBodySetCentreOfMass(m_body, &comRelativeToGeomtriCenter[0]);
//	dMatrix localFrame(m_chassisState.m_localFrame);
//	m_chassisState.m_comOffset = comRelativeToGeomtriCenter;
//	m_chassisState.Init(this, localFrame);
}


void CustomVehicleController::Finalize()
{
	dWeightDistibutionSolver solver;
	dFloat64 unitAccel[256];
	dFloat64 sprungMass[256];

	// make sure tire are aligned
/*
	memset (unitAccel, sizeof (unitAccel), 0);
	int index = 0;
	for (TireList::dListNode* node0 = m_tireList.GetFirst(); node0; node0 = node0->GetNext()) {
		if (unitAccel[index] == 0) {
			CustomVehicleControllerBodyStateTire* const tire0 = &node0->GetInfo();
			for (TireList::dListNode* node1 = node0->GetNext(); node1; node1 = node1->GetNext()) {
				CustomVehicleControllerBodyStateTire* const tire1 = &node->GetInfo();
			}
		}
	}
*/

	m_finalized = true;
	NewtonSkeletonContainerFinalize(m_skeleton);

	int count = 0;
//	m_chassisState.m_matrix = dGetIdentityMatrix();
//	m_chassisState.UpdateInertia();
	dVector dir (0.0f, 1.0f, 0.0f, 0.0f);
	
	dMatrix matrix;
	dVector com;
	dVector invInertia;
	dFloat invMass;
	NewtonBodyGetMatrix(m_body, &matrix[0][0]);
	NewtonBodyGetCentreOfMass(m_body, &com[0]);
	NewtonBodyGetInvMass(m_body, &invMass, &invInertia[0], &invInertia[1], &invInertia[2]);
	matrix = matrix.Inverse();

	for (dList<BodyPartTire>::dListNode* node = m_tireList.GetFirst(); node; node = node->GetNext()) {
		BodyPartTire* const tire = &node->GetInfo();
		
		dMatrix tireMatrix;
		NewtonBodyGetMatrix(tire->GetBody(), &tireMatrix[0][0]);
		tireMatrix = tireMatrix * matrix;

		dVector posit  (tireMatrix.m_posit - com);  

		dComplemtaritySolver::dJacobian &jacobian0 = solver.m_jacobians[count];
		dComplemtaritySolver::dJacobian &invMassJacobian0 = solver.m_invMassJacobians[count];
		jacobian0.m_linear = dir;
		jacobian0.m_angular = posit * dir;
		jacobian0.m_angular.m_w = 0.0f;

		invMassJacobian0.m_linear = jacobian0.m_linear.Scale(invMass);
		invMassJacobian0.m_angular = jacobian0.m_angular.CompProduct(invInertia);

		dFloat diagnal = jacobian0.m_linear % invMassJacobian0.m_linear + jacobian0.m_angular % invMassJacobian0.m_angular;
		solver.m_diagRegularizer[count] = diagnal * 0.005f;
		solver.m_invDiag[count] = 1.0f / (diagnal + solver.m_diagRegularizer[count]);

		unitAccel[count] = 1.0f;
		sprungMass[count] = 0.0f;
		count ++;
	}

	if (count) {
		solver.m_count = count;
		solver.Solve (count, 1.0e-6f, sprungMass, unitAccel);
	}

	int index = 0;
	for (dList<BodyPartTire>::dListNode* node = m_tireList.GetFirst(); node; node = node->GetNext()) {
		BodyPartTire* const tire = &node->GetInfo();
		BodyPartTire::WheelJoint* const tireJoint = (BodyPartTire::WheelJoint*) tire->GetJoint();
		tireJoint->m_restSprunMass = dFloat (5.0f * dFloor (sprungMass[index] / 5.0f + 0.5f));
//		if (m_engine) {
//			tire->CalculateRollingResistance (m_engine->GetTopSpeed());
//		}
		index ++;
	}

//	NewtonBody* const body = GetBody();
//	NewtonBodyGetMatrix(body, &m_chassisState.m_matrix[0][0]);
//	m_chassisState.UpdateInertia();
//	m_sleepCounter = VEHICLE_SLEEP_COUNTER;

	m_finalized = true;
}


CustomVehicleController::BodyPartTire* CustomVehicleController::AddTire(const BodyPartTire::CreationInfo& tireInfo)
{
	dList<BodyPartTire>::dListNode* const tireNode = m_tireList.Append();
	BodyPartTire& tire = tireNode->GetInfo();


	// calculate the tire matrix location,
	dMatrix matrix;
	NewtonBodyGetMatrix(m_body, &matrix[0][0]);
	matrix = m_localFrame * matrix;
	matrix.m_posit = matrix.TransformVector (tireInfo.m_location);
	matrix.m_posit.m_w = 1.0f;

	tire.Init(&m_chassis, matrix, tireInfo);
	m_bodyPartsList.Append(&tire);
	return &tireNode->GetInfo();
}

