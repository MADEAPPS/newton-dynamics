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
#include <CustomVehicleControllerJoint.h>
#include <CustomVehicleControllerManager.h>
#include <CustomVehicleControllerComponent.h>
#include <CustomVehicleControllerBodyState.h>

#define VEHICLE_CONTROLLER_MAX_BODIES								32
#define VEHICLE_CONTROLLER_MAX_JOINTS								32
#define VEHICLE_CONTROLLER_MAX_JACOBIANS_PAIRS						(VEHICLE_CONTROLLER_MAX_JOINTS * 4)
#define VEHICLE_SIDESLEP_NORMALIZED_FRICTION_AT_MAX_SLIP_ANGLE		dFloat(0.75f)
#define VEHICLE_SIDESLEP_NORMALIZED_FRICTION_AT_MAX_SIDESLIP_RATIO	dFloat(0.95f)


class CustomVehicleController::dTireForceSolverSolver: public dComplemtaritySolver
{
	public:
	dTireForceSolverSolver(CustomVehicleController* const controller, dFloat timestep)
		:dComplemtaritySolver()
		,m_controller(controller)
	{
		m_controller->m_externalContactStatesCount = 0;
		m_controller->m_freeContactList = m_controller->m_externalContactStatesPoll.GetFirst();

		// apply all external forces and torques to chassis and all tire velocities
		dFloat timestepInv = 1.0f / timestep;
		m_controller->m_chassisState.UpdateDynamicInputs();

		NewtonBody* const body = controller->GetBody();
		CustomControllerConvexCastPreFilter castFilter (body);
		for (dTireList::dListNode* node = m_controller->m_tireList.GetFirst(); node; node = node->GetNext()) {
			CustomVehicleControllerBodyStateTire* const tire = &node->GetInfo();
			tire->Collide(castFilter, timestepInv);
			tire->UpdateDynamicInputs(timestep);

//tire->SetVeloc(dVector (0.0f, 0.0f, 0.0f, 0.0f));
//dMatrix matrix (tire->GetMatrix());
//tire->SetOmega(matrix[0].Scale (5.0f));
//tire->SetForce(dVector (0.0f, 0.0f, 0.0f, 0.0f));
//tire->SetTorque(dVector (0.0f, 0.0f, 0.0f, 0.0f));
		}

/*
static int xxx;
xxx ++;
if (xxx == 29)
xxx *=1;
if (xxx > 500) 
{
dMatrix matrix (controller->m_chassisState.GetMatrix());
dMatrix matrix1 (controller->m_chassisState.GetMatrix());
//controller->m_chassisState.SetVeloc(matrix[0].Scale (-5.0f));
//controller->m_chassisState.SetOmega(dVector (0.0f, 0.0f, 0.0f, 0.0f));
//controller->m_chassisState.SetForce(dVector (0.0f, 0.0f, 0.0f, 0.0f));
//controller->m_chassisState.SetTorque(dVector (0.0f, 0.0f, 0.0f, 0.0f));
}
*/

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

/*
		for (dTireList::dListNode* node = controller->m_tireList.GetFirst();  node; node = node->GetNext()) {
			CustomVehicleControllerBodyStateTire& tire = node->GetInfo();
			for (int i = 0; i < tire.m_contactCount; i ++) {
				m_bodyArray[bodyCount] = &tire.m_contactBody[i];
				dAssert (bodyCount < int (sizeof (m_bodyArray) / sizeof(m_bodyArray[0])));
				bodyCount ++;
			}
		}
*/

		for (int i = 0; i < m_controller->m_externalContactStatesCount; i ++) {
			m_bodyArray[bodyCount] = m_controller->m_externalContactStates[i];
			dAssert (bodyCount < int (sizeof (m_bodyArray) / sizeof(m_bodyArray[0])));
			bodyCount ++;
		}

		int jointCount = GetActiveJoints();
		BuildJacobianMatrix (jointCount, m_jointArray, timestep, jacobianPairArray, jacobianColumn, sizeof (jacobianPairArray)/ sizeof (jacobianPairArray[0]));
		CalculateReactionsForces (bodyCount, m_bodyArray, jointCount, m_jointArray, timestep, jacobianPairArray, jacobianColumn);
	}

	~dTireForceSolverSolver()
	{
	}

	int GetActiveJoints()
	{
		int jointCount = 0;

		// add all contact joints if any
		for (dTireList::dListNode* node = m_controller->m_tireList.GetFirst(); node; node = node->GetNext()) {
			CustomVehicleControllerBodyStateTire* const tire = &node->GetInfo();
			for (int i = 0; i < tire->m_contactCount; i ++) {
				m_jointArray[jointCount] = &tire->m_contactJoint[i];
				jointCount ++;
				dAssert (jointCount < VEHICLE_CONTROLLER_MAX_JOINTS);
			}
		}

		// add the joints that connect tire to chassis
		for (dTireList::dListNode* node = m_controller->m_tireList.GetFirst(); node; node = node->GetNext()) {
			CustomVehicleControllerBodyStateTire* const tire = &node->GetInfo();
			m_jointArray[jointCount] = &tire->m_chassisJoint;
			jointCount ++;
			dAssert (jointCount < VEHICLE_CONTROLLER_MAX_JOINTS);
		}

		if (m_controller->m_engine) {
			jointCount += m_controller->m_engine->AddDifferentialJoints(&m_jointArray[jointCount]);
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

	dBodyState* m_bodyArray[VEHICLE_CONTROLLER_MAX_BODIES];
	dBilateralJoint* m_jointArray[VEHICLE_CONTROLLER_MAX_JOINTS];
	dJacobianColum jacobianColumn[VEHICLE_CONTROLLER_MAX_JACOBIANS_PAIRS];
	dJacobianPair jacobianPairArray[VEHICLE_CONTROLLER_MAX_JACOBIANS_PAIRS];

	CustomVehicleController* m_controller;
};

class CustomVehicleController::dWeightDistibutionSolver: public dSymmetricBiconjugateGradientSolve
{
	public:
	dWeightDistibutionSolver ()
		:dSymmetricBiconjugateGradientSolve()
		,m_count(0)
	{
	}

	virtual void MatrixTimeVector (dFloat64* const out, const dFloat64* const v) const
	{
		dComplemtaritySolver::dJacobian invMassJacobians [VEHICLE_CONTROLLER_MAX_JOINTS];
		for (int i = 0; i < m_count; i ++) {
			out[i] = m_diagRegularizer[i] * v[i];
			invMassJacobians[i].m_linear = m_invMassJacobians[i].m_linear.Scale (dFloat(v[i]));
			invMassJacobians[i].m_angular = m_invMassJacobians[i].m_angular.Scale (dFloat(v[i]));
		}

		for (int i = 0; i < m_count; i ++) {
			out[i] = invMassJacobians[i].m_linear % m_jacobians[i].m_linear + invMassJacobians[i].m_angular % m_jacobians[i].m_angular;
		}
	}

	virtual bool InversePrecoditionerTimeVector (dFloat64* const out, const dFloat64* const v) const
	{
		for (int i = 0; i < m_count; i ++) {
			out[i] = v[i] * m_invDiag[i];
		}
		return true;
	}

	dComplemtaritySolver::dJacobian m_jacobians [VEHICLE_CONTROLLER_MAX_JOINTS];
	dComplemtaritySolver::dJacobian m_invMassJacobians [VEHICLE_CONTROLLER_MAX_JOINTS];
	dFloat m_invDiag[VEHICLE_CONTROLLER_MAX_JOINTS];
	dFloat m_diagRegularizer[VEHICLE_CONTROLLER_MAX_JOINTS];

	int m_count;
};


CustomVehicleControllerManager::CustomVehicleControllerManager(NewtonWorld* const world)
	:CustomControllerManager<CustomVehicleController> (world, VEHICLE_PLUGIN_NAME)
{
}

CustomVehicleControllerManager::~CustomVehicleControllerManager()
{
}

void CustomVehicleControllerManager::DestroyController (CustomVehicleController* const controller)
{
	controller->Cleanup();
	CustomControllerManager<CustomVehicleController>::DestroyController(controller);
}


CustomVehicleController* CustomVehicleControllerManager::CreateVehicle (NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, const dVector& gravityVector)
{
	CustomVehicleController* const controller = CreateController();
	controller->Init(chassisShape, vehicleFrame, mass, gravityVector);
	return controller;
}


void CustomVehicleController::Init (NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, const dVector& gravityVector)
{
	m_finalized = false;
	m_externalContactStatesCount = 0;
	m_freeContactList = m_externalContactStatesPoll.GetFirst();
	CustomVehicleControllerManager* const manager = (CustomVehicleControllerManager*) GetManager(); 
	NewtonWorld* const world = manager->GetWorld(); 

	// create a compound collision 	
	NewtonCollision* const vehShape = NewtonCreateCompoundCollision(world, 0);
	NewtonCompoundCollisionBeginAddRemove(vehShape);

	// if the shape is a compound collision ass all the pieces one at a time
	int shapeType = NewtonCollisionGetType (chassisShape);
	if (shapeType == SERIALIZE_ID_COMPOUND) {
		for (void* node = NewtonCompoundCollisionGetFirstNode(chassisShape); node; node = NewtonCompoundCollisionGetNextNode (chassisShape, node)) { 
			NewtonCollision* const subCollision = NewtonCompoundCollisionGetCollisionFromNode(chassisShape, node);
			NewtonCompoundCollisionAddSubCollision (vehShape, subCollision);
		}
	} else {
		dAssert ((shapeType == SERIALIZE_ID_CONVEXHULL) || (shapeType == SERIALIZE_ID_BOX));
		NewtonCompoundCollisionAddSubCollision (vehShape, chassisShape);
	}
	NewtonCompoundCollisionEndAddRemove (vehShape);	

	// create the rigid body for this vehicle
	dMatrix locationMatrix (dGetIdentityMatrix());
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
	m_chassisState.m_comOffset = dVector (0.0f, 0.0f, 0.0f, 0.0f);

	m_chassisState.m_gravity = gravityVector;
	m_chassisState.m_gravityMag = dSqrt (gravityVector % gravityVector);
	m_chassisState.Init(this, vehicleFrame);

//	m_stateList.Append(&m_staticWorld);
	m_stateList.Append(&m_chassisState);

	// create the normalized size tire shape
	m_tireCastShape = NewtonCreateChamferCylinder(world, 0.5f, 1.0f, 0, NULL);

	// initialize all components to empty
	m_engine = NULL;
	m_brakes = NULL;
	m_steering = NULL;
	m_handBrakes = NULL;

	SetDryRollingFrictionTorque (100.0f/4.0f);
	SetAerodynamicsDownforceCoefficient (0.5f * dSqrt (gravityVector % gravityVector), 60.0f * 0.447f);
}


void CustomVehicleController::Cleanup()
{
	SetBrakes(NULL);
	SetEngine(NULL);
	SetSteering(NULL);
	SetHandBrakes(NULL);
	NewtonDestroyCollision(m_tireCastShape);
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
	dTireList::dListNode* const tireNode = m_tireList.GetNodeFromInfo(*tire);
	return tireNode->GetNext() ? &tireNode->GetNext()->GetInfo() : NULL;
}


void CustomVehicleController::SetCenterOfGravity(const dVector& comRelativeToGeomtriCenter)
{
	dMatrix localFrame (m_chassisState.m_localFrame);
	m_chassisState.m_comOffset = comRelativeToGeomtriCenter;
	m_chassisState.Init(this, localFrame);
}


CustomVehicleControllerBodyStateTire* CustomVehicleController::AddTire (const CustomVehicleControllerBodyStateTire::TireCreationInfo& tireInfo)
{
	dTireList::dListNode* const tireNode = m_tireList.Append();
	CustomVehicleControllerBodyStateTire& tire = tireNode->GetInfo();
	tire.Init(this, tireInfo);

	m_stateList.Append(&tire);
	return &tireNode->GetInfo();
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


void CustomVehicleController::Finalize()
{
	dWeightDistibutionSolver solver;
	dFloat64 unitAccel[VEHICLE_CONTROLLER_MAX_JOINTS];
	dFloat64 sprungMass[VEHICLE_CONTROLLER_MAX_JOINTS];


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
	
	int count = 0;
	m_chassisState.m_matrix = dGetIdentityMatrix();
	m_chassisState.UpdateInertia();
	dVector dir (m_chassisState.m_localFrame[1]);
	for (dTireList::dListNode* node = m_tireList.GetFirst(); node; node = node->GetNext()) {
		CustomVehicleControllerBodyStateTire* const tire = &node->GetInfo();
		dVector posit  (tire->m_localFrame.m_posit);  
		dComplemtaritySolver::dJacobian &jacobian0 = solver.m_jacobians[count];
		dComplemtaritySolver::dJacobian &invMassJacobian0 = solver.m_invMassJacobians[count];
		jacobian0.m_linear = dir;
		jacobian0.m_angular = posit * dir;
		jacobian0.m_angular.m_w = 0.0f;

		invMassJacobian0.m_linear = jacobian0.m_linear.Scale(m_chassisState.m_invMass);
		invMassJacobian0.m_angular = jacobian0.m_angular.CompProduct(m_chassisState.m_localInvInertia);

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
	for (dTireList::dListNode* node = m_tireList.GetFirst(); node; node = node->GetNext()) {
		CustomVehicleControllerBodyStateTire* const tire = &node->GetInfo();
		tire->m_restSprunMass = dFloat (5.0f * dFloor (sprungMass[index] / 5.0f + 0.5f));
		if (m_engine) {
			tire->CalculateRollingResistance (m_engine->GetTopSpeed());
		}
		index ++;
	}

	m_finalized = true;
}


CustomVehicleControllerBodyStateContact* CustomVehicleController::GetContactBody (const NewtonBody* const body)
{
	for (int i = 0; i < m_externalContactStatesCount; i ++) {
		if (m_externalContactStates[i]->m_newtonBody == body) {
			return m_externalContactStates[i];
		}
	}

	dAssert (m_externalContactStatesPoll.GetCount() < 32);
	if (!m_freeContactList) {
		m_freeContactList = m_externalContactStatesPoll.Append();
	}
	CustomVehicleControllerBodyStateContact* const externalBody = &m_freeContactList->GetInfo();
	m_freeContactList = m_freeContactList->GetNext(); 
	externalBody->Init (this, body);
	m_externalContactStates[m_externalContactStatesCount] = externalBody;
	m_externalContactStatesCount ++;
	dAssert (m_externalContactStatesCount < int (sizeof (m_externalContactStates) / sizeof (m_externalContactStates[0])));

	return externalBody;
}

void CustomVehicleController::PreUpdate (dFloat timestep, int threadIndex)
{
	if (m_finalized) {
		dTireForceSolverSolver tireSolver (this, timestep);	
	}
}

void CustomVehicleController::PostUpdate(dFloat timestep, int threadIndex)
{
	if (m_finalized) {
		NewtonBody* const body = GetBody();
		NewtonBodyGetMatrix(body, &m_chassisState.m_matrix[0][0]);
		for (dTireList::dListNode* node = m_tireList.GetFirst(); node; node = node->GetNext()) {
			CustomVehicleControllerBodyStateTire* const tire = &node->GetInfo();
			tire->UpdateTransform();
		}
	}
}
