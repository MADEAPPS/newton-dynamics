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
#include <CustomJointLibraryStdAfx.h>
#include <CustomJoint.h>
#include <CustomVehicleControllerJoint.h>
#include <CustomVehicleControllerManager.h>
#include <CustomVehicleControllerComponent.h>
#include <CustomVehicleControllerBodyState.h>

#define VEHICLE_CONTROLLER_MAX_JOINTS			32
#define VEHICLE_CONTROLLER_MAX_JACOBIANS_PAIRS	(VEHICLE_CONTROLLER_MAX_JOINTS * 4)

#define VEHICLE_PSD_DAMP_TOL                    dFloat(1.0e-4f)

#if 0
#define VEHICLE_VEL_DAMP				        dFloat(100.0f)
#define VEHICLE_POS_DAMP				        dFloat(1500.0f)
#define VEHICLE_MAX_FRICTION_BOUND	            dFloat(1.0e15f)
#define VEHICLE_MIN_FRICTION_BOUND			    -VEHICLE_MAX_FRICTION_BOUND




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


const CustomVehicleController::ChassisBodyState& CustomVehicleController::GetChassisState () const
{
	return m_chassisState;
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





#endif



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
	m_brakes = NULL;
	m_steering = NULL;
	m_handBrakes = NULL;

/*
	// set default tire model, Create a simplified normalized Tire Curve
	// we will use a simple piece wise curve from Pacejkas tire model, 
	// http://en.wikipedia.org/wiki/File:Magic_Formula_Curve.png
	//an application can use advance curves like platting the complete Pacejkas empirical equation
	//dFloat slips[] = {0.0f, 0.1f, 0.2f, 1.0f};
	//dFloat normalizedLongitudinalForce[] = {0.0f, 0.8f, 1.0f, 1.0f};
	//SetLongitudinalSlipRatio (sizeof (slips) / sizeof (slips[0]), slips, normalizedLongitudinalForce);
	SetLongitudinalSlipRatio (0.2f);
	SetLateralSlipAngle(3.0f);
*/
}


void CustomVehicleController::Cleanup()
{
	SetBrakes(NULL);
	SetEngine(NULL);
	SetSteering(NULL);
	SetHandBrakes(NULL);
	NewtonDestroyCollision(m_tireCastShape);
}



void CustomVehicleController::SetCenterOfGravity(const dVector& comRelativeToGeomtriCenter)
{
	dMatrix localFrame (m_chassisState.m_localFrame);
	m_chassisState.m_comOffset = comRelativeToGeomtriCenter;
	m_chassisState.Init(this, localFrame);
}


CustomVehicleControllerBodyStateTire* CustomVehicleController::AddTire (const CustomVehicleControllerBodyStateTire::TireCreationInfo& tireInfo)
{
	TireList::dListNode* const tireNode = m_tireList.Append();
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
		for (dList<CustomVehicleControllerBodyState*>::dListNode* node = m_stateList.GetFirst(); node; node = node->GetNext()) {
			if (node->GetInfo() == &m_engineState) {
				m_stateList.Remove(node);
				break;
			}
		}
		delete m_engine;
	}

	m_engine = engine;
	if (m_engine) {
		m_engineState.Init(this);
		m_stateList.Append(&m_engineState);
	}
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


int CustomVehicleController::GetActiveJoints(CustomVehicleControllerJoint** const jointArray)
{
	// add the engine joints
	int jointCount = m_engineState.CalculateActiveJoints(this, &jointArray[0]);

	// add all contact joints if any
	for (TireList::dListNode* node = m_tireList.GetFirst(); node; node = node->GetNext()) {
		CustomVehicleControllerBodyStateTire* const tire = &node->GetInfo();
		if (tire->m_contactJoint.m_contactCount) {
			jointArray[jointCount] = &tire->m_contactJoint;
			jointCount ++;
			dAssert (jointCount < VEHICLE_CONTROLLER_MAX_JOINTS);
		}
	}

	// add the joints that connect tire to chassis
	for (TireList::dListNode* node = m_tireList.GetFirst(); node; node = node->GetNext()) {
		CustomVehicleControllerBodyStateTire* const tire = &node->GetInfo();
		jointArray[jointCount] = &tire->m_chassisJoint;
		jointCount ++;
		dAssert (jointCount < VEHICLE_CONTROLLER_MAX_JOINTS);
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





int CustomVehicleController::BuildJacobianMatrix (int jointCount, CustomVehicleControllerJoint** const jointArray, dFloat timestep, CustomVehicleControllerJoint::JacobianPair* const jacobianArray, CustomVehicleControllerJoint::JacobianColum* const jacobianColumnArray)
{
	int rowCount = 0;

	CustomVehicleControllerJoint::ParamInfo constraintParams;
	constraintParams.m_timestep = timestep;
	constraintParams.m_timestepInv = 1.0f / timestep;

	// calculate Jacobian derivative for each active joint	
	for (int j = 0; j < jointCount; j ++) {
		CustomVehicleControllerJoint* const joint = jointArray[j];
		constraintParams.m_count = 0;
		joint->JacobianDerivative (&constraintParams); 

		int dofCount = constraintParams.m_count;
		joint->m_count = dofCount;
		joint->m_start = rowCount;

		// copy the rows and columns from the Jacobian derivative descriptor
		for (int i = 0; i < dofCount; i ++) {
			CustomVehicleControllerJoint::JacobianColum* const col = &jacobianColumnArray[rowCount];
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
		CustomVehicleControllerBodyState* const state0 = joint->m_state0;
		CustomVehicleControllerBodyState* const state1 = joint->m_state1;

		const dMatrix& invInertia0 = state0->m_invInertia;
		const dMatrix& invInertia1 = state1->m_invInertia;

		dFloat invMass0 = state0->m_invMass;
		dFloat invMass1 = state1->m_invMass;
		dFloat weight = 0.9f;
		for (int i = 0; i < dofCount; i ++) {
			CustomVehicleControllerJoint::JacobianPair* const row = &jacobianArray[index];
			CustomVehicleControllerJoint::JacobianColum* const col = &jacobianColumnArray[index];

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

void CustomVehicleController::CalculateReactionsForces (int jointCount, CustomVehicleControllerJoint** const jointArray, dFloat timestepSrc, CustomVehicleControllerJoint::JacobianPair* const jacobianArray, CustomVehicleControllerJoint::JacobianColum* const jacobianColumnArray)
{
	CustomVehicleControllerJoint::Jacobian stateVeloc[VEHICLE_CONTROLLER_MAX_JOINTS / 2];
	CustomVehicleControllerJoint::Jacobian internalForces[VEHICLE_CONTROLLER_MAX_JOINTS / 2];

	int stateIndex = 0;
	dVector zero(dFloat (0.0f), dFloat (0.0f), dFloat (0.0f), dFloat (0.0f));
	for (dList<CustomVehicleControllerBodyState*>::dListNode* stateNode = m_stateList.GetFirst(); stateNode; stateNode = stateNode->GetNext()) {
		CustomVehicleControllerBodyState* const state = stateNode->GetInfo();
		stateVeloc[stateIndex].m_linear = state->m_veloc;
		stateVeloc[stateIndex].m_angular = state->m_omega;

		internalForces[stateIndex].m_linear = zero;
		internalForces[stateIndex].m_angular = zero;

		state->m_myIndex = stateIndex;
		stateIndex ++;
		dAssert (stateIndex < int (sizeof (stateVeloc)/sizeof (stateVeloc[0])));
	}

	for (int i = 0; i < jointCount; i ++) {
		CustomVehicleControllerJoint::Jacobian y0;
		CustomVehicleControllerJoint::Jacobian y1;
		y0.m_linear = zero;
		y0.m_angular = zero;
		y1.m_linear = zero;
		y1.m_angular = zero;
		CustomVehicleControllerJoint* const constraint = jointArray[i];
		int first = constraint->m_start;
		int count = constraint->m_count;
		for (int j = 0; j < count; j ++) { 
			CustomVehicleControllerJoint::JacobianPair* const row = &jacobianArray[j + first];
			const CustomVehicleControllerJoint::JacobianColum* const col = &jacobianColumnArray[j + first];
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
		CustomVehicleControllerJoint::JointAccelerationDecriptor joindDesc;
		joindDesc.m_timeStep = timestep;
		joindDesc.m_invTimeStep = invTimestep;
		joindDesc.m_firstPassCoefFlag = firstPassCoef;

		for (int curJoint = 0; curJoint < jointCount; curJoint ++) {
			CustomVehicleControllerJoint* const constraint = jointArray[curJoint];
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

				CustomVehicleControllerJoint* const constraint = jointArray[curJoint];
				int index = constraint->m_start;
				int rowsCount = constraint->m_count;
				int m0 = constraint->m_state0->m_myIndex;
				int m1 = constraint->m_state1->m_myIndex;

				dVector linearM0 (internalForces[m0].m_linear);
				dVector angularM0 (internalForces[m0].m_angular);
				dVector linearM1 (internalForces[m1].m_linear);
				dVector angularM1 (internalForces[m1].m_angular);

				CustomVehicleControllerBodyState* const state0 = constraint->m_state0;
				CustomVehicleControllerBodyState* const state1 = constraint->m_state1;
				const dMatrix& invInertia0 = state0->m_invInertia;
				const dMatrix& invInertia1 = state1->m_invInertia;
				dFloat invMass0 = state0->m_invMass;
				dFloat invMass1 = state1->m_invMass;

				for (int k = 0; k < rowsCount; k ++) {
					CustomVehicleControllerJoint::JacobianPair* const row = &jacobianArray[index];
					CustomVehicleControllerJoint::JacobianColum* const col = &jacobianColumnArray[index];

					dVector JMinvIM0linear (row->m_jacobian_IM0.m_linear.Scale (invMass0));
					dVector JMinvIM1linear (row->m_jacobian_IM1.m_linear.Scale (invMass1));
					dVector JMinvIM0angular = invInertia0.UnrotateVector(row->m_jacobian_IM0.m_angular);
					dVector JMinvIM1angular = invInertia1.UnrotateVector(row->m_jacobian_IM1.m_angular);
					dVector acc (JMinvIM0linear.CompProduct(linearM0) + JMinvIM0angular.CompProduct(angularM0) + JMinvIM1linear.CompProduct(linearM1) + JMinvIM1angular.CompProduct(angularM1));

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

		for (dList<CustomVehicleControllerBodyState*>::dListNode* stateNode = m_stateList.GetFirst()->GetNext(); stateNode; stateNode = stateNode->GetNext()) {
			CustomVehicleControllerBodyState* const state = stateNode->GetInfo();
			int index = state->m_myIndex;
			dVector force (state->m_externalForce + internalForces[index].m_linear);
			dVector torque (state->m_externalTorque + internalForces[index].m_angular);
			state->IntegrateForce(timestep, force, torque);
		}
	}

	for (int i = 0; i < jointCount; i ++) {
		CustomVehicleControllerJoint* const constraint = jointArray[i];
		int first = constraint->m_start;
		int count = constraint->m_count;
		for (int j = 0; j < count; j ++) { 
			const CustomVehicleControllerJoint::JacobianColum* const col = &jacobianColumnArray[j + first];
			dFloat val = col->m_force; 
			constraint->m_jointFeebackForce[j] = val;
		}
	}

	for (dList<CustomVehicleControllerBodyState*>::dListNode* stateNode = m_stateList.GetFirst()->GetNext(); stateNode; stateNode = stateNode->GetNext()) {
		CustomVehicleControllerBodyState* const state = stateNode->GetInfo();
		int index = state->m_myIndex;
		state->CalculateNetForceAndTorque (invTimestepSrc, stateVeloc[index].m_linear, stateVeloc[index].m_angular);
	}

	for (int i = 0; i < jointCount; i ++) {
		CustomVehicleControllerJoint* const constraint = jointArray[i];
		constraint->UpdateSolverForces (jacobianArray);
	}
}



void CustomVehicleController::PreUpdate(dFloat timestep, int threadIndex)
{
	CustomVehicleControllerJoint* jointArray[VEHICLE_CONTROLLER_MAX_JOINTS];
	CustomVehicleControllerJoint::JacobianColum jacobianColumn[VEHICLE_CONTROLLER_MAX_JACOBIANS_PAIRS];
	CustomVehicleControllerJoint::JacobianPair jacobianPairArray[VEHICLE_CONTROLLER_MAX_JACOBIANS_PAIRS];

	// apply all external forces and torques to chassis and all tire velocities
	dFloat timestepInv = 1.0f / timestep;
	NewtonBody* const body = GetBody();
	CustomControllerConvexCastPreFilter castFilter (body);
	m_chassisState.UpdateDynamicInputs();
	for (TireList::dListNode* node = m_tireList.GetFirst(); node; node = node->GetNext()) {
		CustomVehicleControllerBodyStateTire* const tire = &node->GetInfo();
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
}


void CustomVehicleController::PostUpdate(dFloat timestep, int threadIndex)
{
//	dAssert (0);
/*
	NewtonBody* const body = GetBody();
	NewtonBodyGetMatrix(body, &m_chassisState.m_matrix[0][0]);
	for (TireList::CustomListNode* node = m_tireList.GetFirst(); node; node = node->GetNext()) {
		TireBodyState* const tire = &node->GetInfo();
		tire->UpdateTransform();
	}
*/
}

