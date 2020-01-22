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

#include "dStdafxVehicle.h"
#include "dVehicleNode.h"
#include "dVehicleTire.h"
#include "dVehicleEngine.h"
#include "dVehicleMultiBody.h"
#include "dVehicleManager.h"
#include "dVehicleDifferential.h"

//static int xxxx;

dVehicleMultiBody::dVehicleMultiBody(NewtonBody* const body, const dMatrix& localFrame, dFloat gravityMag)
	:dVehicle(body, localFrame, gravityMag)
	,dVehicleSolver()
	,m_collidingNodes(8)
	,m_collidingIndex(0)
	,m_sleepCounter(0)
{
	m_brakeControl.Init(this);
	m_engineControl.Init(this);
	m_steeringControl.Init(this);
	m_handBrakeControl.Init(this);

	AddControl(&m_brakeControl);
	AddControl(&m_engineControl);
	AddControl(&m_steeringControl);
	AddControl(&m_handBrakeControl);

	m_localFrame.m_posit = dVector(0.0f, 0.0f, 0.0f, 1.0f);
	dAssert(m_localFrame.TestOrthogonal());

	// set linear and angular drag to zero
	dVector drag(0.0f);
	NewtonBodySetLinearDamping(m_newtonBody, 0.0f);
	NewtonBodySetAngularDamping(m_newtonBody, &drag[0]);

	/*
	m_aerodynamicsDownForce0 = 0.0f;
	m_aerodynamicsDownForce1 = 0.0f;
	m_aerodynamicsDownSpeedCutOff = 0.0f;
	m_aerodynamicsDownForceCoefficient = 0.0f;
	SetAerodynamicsDownforceCoefficient(0.5f, 0.4f, 1.0f);
	*/

	// set the inertia matrix;
	dVector tmp;
	NewtonBodyGetMass(m_newtonBody, &tmp.m_w, &tmp.m_x, &tmp.m_y, &tmp.m_z);
	m_proxyBody.SetMass(tmp.m_w);
	m_proxyBody.SetInertia(tmp.m_x, tmp.m_y, tmp.m_z);

	dMatrix matrix(dGetIdentityMatrix());
	NewtonBodyGetCentreOfMass(m_newtonBody, &matrix.m_posit[0]);
	matrix.m_posit.m_w = 1.0f;
	m_proxyBody.SetLocalMatrix(matrix);
}

dVehicleMultiBody::~dVehicleMultiBody()
{
	if (m_managerNode) {
		m_manager->RemoveRoot(this);
	}
}

void dVehicleMultiBody::Finalize()
{
	dVector minP;
	dVector maxP;
	NewtonCollision* const collision = NewtonBodyGetCollision(m_newtonBody);
	CalculateAABB(collision, dGetIdentityMatrix(), minP, maxP);

	for (dVehicleNodeChildrenList::dListNode* node = m_children.GetFirst(); node; node = node->GetNext()) {
		dVehicleTire* const tire = node->GetInfo()->GetAsTire();
		if (tire) {
			dVector tireMinP;
			dVector tireMaxP;

			dMatrix tireMatrix(tire->GetHardpointMatrix(0.0f));
			tire->CalculateNodeAABB(tireMatrix, tireMinP, tireMaxP);

			minP = minP.Min(tireMinP);
			maxP = maxP.Max(tireMaxP);
		}
	}

	m_obbOrigin = (maxP + minP).Scale(0.5f);
	m_obbSize = (maxP - minP).Scale(0.5f) + dVector(0.1f, 0.1f, 0.1f, 0.0f);

	ApplyExternalForce();
	dVehicleSolver::Finalize();
}

const void dVehicleMultiBody::Debug(dCustomJoint::dDebugDisplay* const debugContext) const
{
	const dMatrix& matrix = m_proxyBody.GetMatrix();

	// draw velocity
	dVector veloc(m_proxyBody.GetVelocity());
	veloc = veloc - matrix.m_up.Scale(veloc.DotProduct3(matrix.m_up));

	dFloat mag = dSqrt(veloc.DotProduct3(veloc));
	veloc = veloc.Scale(dLog(mag) / (mag + 0.1f));

	debugContext->SetColor(dVector(1.0f, 0.65f, 0.0f, 1.0f));
	dVector p0(matrix.m_posit + matrix.m_up.Scale(1.0f));
	dVector p1(p0 + veloc);
	debugContext->DrawLine(p0, p1);

	debugContext->SetColor(dVector(0.5f, 0.5f, 0.5f, 1.0f));
	dVector p2(p0 + matrix.m_front.Scale(2.0f));
	debugContext->DrawLine(p0, p2);

	//draw vehicle weight
	if (m_gravity.DotProduct3(m_gravity) > 0.1f) {
		// for now weight is normalize to 2.0
		debugContext->SetColor(dVector(0.0f, 0.0f, 1.0f, 1.0f));
		dVector gravityDir(m_gravity.Normalize());
		dVector p3(p0 - gravityDir.Scale(2.0f));
		debugContext->DrawLine(p0, p3);
	}

	dVehicleNode::Debug(debugContext);
}

void dVehicleMultiBody::ApplyDriverInputs(const dDriverInput& driveInputs, dFloat timestep)
{
	m_brakeControl.SetParam(driveInputs.m_brakePedal);
	m_handBrakeControl.SetParam(driveInputs.m_handBrakeValue);
	m_steeringControl.SetParam(driveInputs.m_steeringValue);
	m_engineControl.SetParam(driveInputs.m_throttle);

m_engineControl.SetIgnition(driveInputs.m_ignitionKey);
#if 0
	if (m_engineControl) {
		m_engineControl->SetClutch(driveInputs.m_clutchPedal);
		m_engineControl->SetGear(dVehicleEngineInterface::m_firstGear);
	}

	if (m_engineControl) {
		m_engineControl->SetDifferentialLock(driveInputs.m_lockDifferential ? true : false);

		switch (m_engineControl->m_drivingState) {
		case dEngineController::m_engineOff:
		{
			if (driveInputs.m_ignitionKey) {
				m_engineControl->SetIgnition(true);
				m_engineControl->SetGear(m_engineControl->GetNeutralGear());
				m_engineControl->m_drivingState = dEngineController::m_engineIdle;
				if (m_handBrakesControl) {
					m_handBrakesControl->SetParam(driveInputs.m_handBrakeValue);
				}
			}
			else {
				m_engineControl->SetIgnition(false);
				m_engineControl->SetGear(m_engineControl->GetFirstGear());
				if (m_handBrakesControl) {
					m_handBrakesControl->SetParam(1.0f - driveInputs.m_handBrakeValue);
				}
			}
			break;
		}

		case dEngineController::m_engineStop:
		{
			m_engineControl->SetParam(0.0f);
			m_engineControl->SetClutchParam(1.0f);
			if (dAbs(m_engineControl->GetSpeed()) < 4.0f) {
				m_engineControl->m_stopDelay = int(2.0f / timestep);
				m_engineControl->m_drivingState = dEngineController::m_engineStopDelay;
			}
			if (m_brakesControl) {
				m_engineControl->SetGear(m_engineControl->GetNeutralGear());
				m_brakesControl->SetParam(1.0f);
				m_brakesControl->SetParam(1.0f);
			}
			break;
		}

		case dEngineController::m_engineStopDelay:
		{
			m_engineControl->m_stopDelay--;
			if ((m_engineControl->m_stopDelay < 0) || driveInputs.m_ignitionKey) {
				m_engineControl->m_drivingState = dEngineController::m_engineIdle;
			}
			m_engineControl->SetGear(m_engineControl->GetNeutralGear());
			m_brakesControl->SetParam(1.0f);
			m_brakesControl->SetParam(1.0f);
			break;
		}

		case dEngineController::m_engineIdle:
		{
			if (!driveInputs.m_ignitionKey) {
				m_engineControl->m_drivingState = dEngineController::m_engineOff;
			}
			else {
				m_engineControl->SetGear(driveInputs.m_gear);
				m_engineControl->SetParam(driveInputs.m_throttle);

				if (m_engineControl->m_automaticTransmissionMode) {
					m_engineControl->SetClutchParam(0.0f);
				}
				else {
					m_engineControl->SetClutchParam(1.0f - driveInputs.m_clutchPedal);
				}
				if (m_handBrakesControl) {
					m_handBrakesControl->SetParam(driveInputs.m_handBrakeValue);
				}

				if (m_engineControl->GetGear() == m_engineControl->GetReverseGear()) {
					m_engineControl->m_drivingState = dEngineController::m_driveReverse;
				}
				else if (m_engineControl->GetGear() != m_engineControl->GetNeutralGear()) {
					m_engineControl->m_drivingState = dEngineController::m_driveForward;
				}
			}
			break;
		}

		case dEngineController::m_driveForward:
		{
			m_engineControl->SetParam(driveInputs.m_throttle);

			if ((driveInputs.m_brakePedal > 0.1f) && (m_engineControl->GetRPM() < 1.1f * m_engineControl->GetIdleRPM())) {
				m_engineControl->SetClutchParam(0.0f);
			}
			else {
				m_engineControl->SetClutchParam(driveInputs.m_clutchPedal);
			}
			if (m_handBrakesControl) {
				m_handBrakesControl->SetParam(driveInputs.m_handBrakeValue);
			}

			if (!m_engineControl->GetTransmissionMode()) {
				dAssert(0);
				//m_engineControl->SetGear(driveInputs.m_gear);
			}
			else {
				if (m_engineControl->GetSpeed() < 5.0f) {
					if (driveInputs.m_gear == m_engineControl->GetReverseGear()) {
						m_engineControl->SetGear(driveInputs.m_gear);
						if (m_brakesControl) {
							m_brakesControl->SetParam(1.0f);
							m_brakesControl->SetParam(1.0f);
						}
						m_engineControl->m_drivingState = dEngineController::m_engineIdle;
					}
					else if (driveInputs.m_gear == m_engineControl->GetNeutralGear()) {
						m_engineControl->SetGear(driveInputs.m_gear);
						m_engineControl->m_drivingState = dEngineController::m_engineIdle;
					}
				}
			}
			if (!driveInputs.m_ignitionKey) {
				m_engineControl->m_drivingState = dEngineController::m_engineStop;
			}
			break;
		}

		case dEngineController::m_driveReverse:
		{
			m_engineControl->SetParam(driveInputs.m_throttle);

			if ((driveInputs.m_brakePedal > 0.1f) && (m_engineControl->GetRPM() < 1.1f * m_engineControl->GetIdleRPM())) {
				m_engineControl->SetClutchParam(0.0f);
			}
			else {
				m_engineControl->SetClutchParam(driveInputs.m_clutchPedal);
			}
			if (m_handBrakesControl) {
				m_handBrakesControl->SetParam(driveInputs.m_handBrakeValue);
			}

			if (!m_engineControl->GetTransmissionMode()) {
				dAssert(0);
				//m_engineControl->SetGear(driveInputs.m_gear);
			}
			else {
				if (m_engineControl->GetSpeed() < 5.0f) {
					if (driveInputs.m_gear == m_engineControl->GetNeutralGear()) {
						m_engineControl->SetGear(driveInputs.m_gear);
						m_engineControl->m_drivingState = dEngineController::m_engineIdle;
					}
					else if (driveInputs.m_gear != m_engineControl->GetReverseGear()) {
						m_engineControl->SetGear(driveInputs.m_gear);
						if (m_brakesControl) {
							m_brakesControl->SetParam(1.0f);
							m_brakesControl->SetParam(1.0f);
						}
						m_engineControl->m_drivingState = dEngineController::m_engineIdle;
					}
				}
			}
			if (!driveInputs.m_ignitionKey) {
				m_engineControl->m_drivingState = dEngineController::m_engineStop;
			}
			break;
		}

		default:
			dAssert(0);
		}

	}
	else if (m_handBrakesControl) {
		m_handBrakesControl->SetParam(driveInputs.m_handBrakeValue);
	}

#endif
}

int dVehicleMultiBody::GetKinematicLoops(dVehicleLoopJoint** const jointArray)
{
	return dVehicleNode::GetKinematicLoops(jointArray);
}

dVehicleTire* dVehicleMultiBody::AddTire(const dMatrix& locationInGlobalSpace, const dTireInfo& tireInfo)
{
	dVehicleTire* const tire = new dVehicleTire(this, locationInGlobalSpace, tireInfo);
	return tire;
}

dVehicleDifferential* dVehicleMultiBody::AddDifferential(dFloat mass, dFloat radius, dVehicleTire* const leftTire, dVehicleTire* const rightTire)
{
	dVehicleDifferential* const differential = new dVehicleDifferential(this, mass, radius, leftTire, rightTire);
	return differential;
}

dVehicleDifferential* dVehicleMultiBody::AddDifferential(dFloat mass, dFloat radius, dVehicleDifferential* const differential0, dVehicleDifferential* const differential1)
{
	dVehicleDifferential* const differential = new dVehicleDifferential(this, mass, radius, differential0, differential1, dYawMatrix (90.0f * dDegreeToRad));
	return differential;
}

dVehicleEngine* dVehicleMultiBody::AddEngine(const dEngineInfo& engineInfo, dVehicleDifferential* const differential)
{
	dVehicleEngine* const engine = new dVehicleEngine(this, engineInfo, differential);
	return engine;
}

dVehicleEngineControl* dVehicleMultiBody::GetEngineControl()
{
	return &m_engineControl;
}

dVehicleSteeringControl* dVehicleMultiBody::GetSteeringControl()
{
	return &m_steeringControl;
}

dVehicleBrakeControl* dVehicleMultiBody::GetBrakeControl()
{
	return &m_brakeControl;
}

dVehicleBrakeControl* dVehicleMultiBody::GetHandBrakeControl()
{
	return &m_handBrakeControl;
}

void dVehicleMultiBody::ApplyExternalForce()
{
	dMatrix matrix;
	dVector vector(0.0f);

	m_collidingIndex = 0;

	// get data from engine rigid body and copied to the vehicle chassis body
	NewtonBodyGetMatrix(m_newtonBody, &matrix[0][0]);
	m_proxyBody.SetMatrix(matrix);
	m_proxyBody.UpdateInertia();

	NewtonBodyGetVelocity(m_newtonBody, &vector[0]);
	m_proxyBody.SetVeloc(vector);

	NewtonBodyGetOmega(m_newtonBody, &vector[0]);
	m_proxyBody.SetOmega(vector);

	NewtonBodyGetForce(m_newtonBody, &vector[0]);
	m_proxyBody.SetForce(vector);
	m_gravity = vector.Scale(m_proxyBody.GetInvMass());

	NewtonBodyGetTorque(m_newtonBody, &vector[0]);
	m_proxyBody.SetTorque(vector);

/*
static int xxx;
dVector veloc(m_proxyBody.GetVelocity());
veloc = veloc - matrix.m_up.Scale(veloc.DotProduct3(matrix.m_up));
float xxxx = dAbs (veloc.DotProduct3(matrix.m_front)) + 0.01f;
float yyyy = veloc.DotProduct3(matrix.m_right);
dTrace(("%d chassisBeta=%f  ", xxx, dAtan2(yyyy, xxxx) * dRadToDegree));
xxx ++;
*/

	dVehicleNode::ApplyExternalForce();
}

dVehicleCollidingNode* dVehicleMultiBody::FindCollideNode(dVehicleNode* const node0, NewtonBody* const body)
{
	NewtonBody* dynamicBody = body;
	if (dynamicBody) {
		dFloat invMass;
		dFloat invIxx;
		dFloat invIyy;
		dFloat invIzz;
		NewtonBodyGetInvMass(body, &invMass, &invIxx, &invIyy, &invIzz);
		dynamicBody = invMass > 0.0f ? body : NULL;
	}

	for (int i = 0; i < m_collidingIndex; i ++) {
		dVehicleCollidingNode* const node1 = &m_collidingNodes[i];
		if (node1->m_body == dynamicBody) {
			return node1;
		}
	}

	dVehicleCollidingNode* const node1 = &m_collidingNodes[m_collidingIndex];
	node1->m_body = dynamicBody;
	node1->m_index = m_collidingIndex + dVehicleSolver::m_nodeCount;
	dVehicleSolver::m_nodesOrder[node1->m_index] = node1;

	dVector vector(0.0f);
	dMatrix matrix(dGetIdentityMatrix());
	dComplementaritySolver::dBodyState& proxyBody = node1->GetProxyBody();
	if (dynamicBody) {
		dFloat Mass;
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;

		NewtonBodyGetMass(dynamicBody, &Mass, &Ixx, &Iyy, &Izz);
		proxyBody.SetMass(Mass);
		proxyBody.SetInertia(Ixx, Iyy, Izz);

		NewtonBodyGetCentreOfMass(dynamicBody, &matrix.m_posit[0]);
		matrix.m_posit.m_w = 1.0f;
		proxyBody.SetLocalMatrix(matrix);

		NewtonBodyGetMatrix(dynamicBody, &matrix[0][0]);
		proxyBody.SetMatrix(matrix);

		NewtonBodyGetVelocity(dynamicBody, &vector[0]);
		proxyBody.SetVeloc(vector);

		NewtonBodyGetOmega(dynamicBody, &vector[0]);
		proxyBody.SetOmega(vector);

		NewtonBodyGetForce(dynamicBody, &vector[0]);
		proxyBody.SetForce(vector);

		NewtonBodyGetTorque(dynamicBody, &vector[0]);
		proxyBody.SetTorque(vector);
	} else {
		proxyBody.SetMass(0.0f);
		proxyBody.SetInertia(0.0f, 0.0f, 0.0f);
		proxyBody.SetLocalMatrix(matrix);
		proxyBody.SetMatrix(matrix);
		proxyBody.SetVeloc(vector);
		proxyBody.SetOmega(vector);
		proxyBody.SetForce(vector);
		proxyBody.SetTorque(vector);
	}
	proxyBody.UpdateInertia();

	m_collidingIndex ++;
	dAssert (m_collidingIndex <= 8);
	return node1;
}

int dVehicleMultiBody::OnAABBOverlap(const NewtonBody * const body, void* const context)
{
	dCollectCollidingBodies* const bodyList = (dCollectCollidingBodies*)context;
	if (body != bodyList->m_exclude) {
		bodyList->m_array[bodyList->m_count] = (NewtonBody*)body;
		bodyList->m_count++;
		dAssert(bodyList->m_count < sizeof(bodyList->m_array) / sizeof(bodyList->m_array[1]));
	}
	return 1;
}

void dVehicleMultiBody::CalculateTireContacts(dFloat timestep)
{
	const dMatrix& matrix = m_proxyBody.GetMatrix();
	dVector origin(matrix.TransformVector(m_obbOrigin));
	dVector size(matrix.m_front.Abs().Scale(m_obbSize.m_x) + matrix.m_up.Abs().Scale(m_obbSize.m_y) + matrix.m_right.Abs().Scale(m_obbSize.m_z));

	dVector p0 (origin - size);
	dVector p1 (origin + size);

	dCollectCollidingBodies bodyList(GetBody());
	NewtonWorld* const world = NewtonBodyGetWorld(GetBody());
	NewtonWorldForEachBodyInAABBDo(world, &p0.m_x, &p1.m_x, OnAABBOverlap, &bodyList);
	
	for (dVehicleNodeChildrenList::dListNode* node = m_children.GetFirst(); node; node = node->GetNext()) {
		dVehicleTire* const tire = node->GetInfo()->GetAsTire();
		if (tire) {
			tire->CalculateContacts(bodyList, timestep);
		}
	}
}

void dVehicleMultiBody::Integrate(dFloat timestep)
{
	m_proxyBody.IntegrateForce(timestep, m_proxyBody.GetForce(), m_proxyBody.GetTorque());
	m_proxyBody.IntegrateVelocity(timestep);
	dVehicleNode::Integrate(timestep);
}

void dVehicleMultiBody::CalculateFreeDof(dFloat timestep)
{
	dVehicleNode::CalculateFreeDof(timestep);

	dVector force(m_proxyBody.GetForce());
	dVector torque(m_proxyBody.GetTorque());
	NewtonBodySetForce(m_newtonBody, &force[0]);
	NewtonBodySetTorque(m_newtonBody, &torque[0]);
}

bool dVehicleMultiBody::CheckSleeping()
{
return false;

	for (int i = 0; i < m_controlerCount; i ++) {
		if (m_control[i]->ParamChanged()) {
			m_sleepCounter = 0;
			return false;
		}
	}

	dVector vector(0.0f);
	NewtonBodyGetVelocity(m_newtonBody, &vector[0]);
	if (vector.DotProduct3(vector) < 1.0e-2f) {
		NewtonBodyGetOmega(m_newtonBody, &vector[0]);
		if (vector.DotProduct3(vector) < 1.0e-2f) {
			NewtonBodyGetAcceleration(m_newtonBody, &vector[0]);
			if (vector.DotProduct3(vector) < 1.0e-2f) {
				NewtonBodyGetAlpha(m_newtonBody, &vector[0]);
				if (vector.DotProduct3(vector) < 1.0e-2f) {
					return true;
				}
			}
		}
	}
	
	m_sleepCounter = 0;
	return false;
}

void dVehicleMultiBody::PreUpdate(dFloat timestep)
{
	m_manager->UpdateDriverInput(this, timestep);

	for (int i = 0; i < m_controlerCount; i ++) {
		m_control[i]->Update(timestep);
	}

	if (CheckSleeping()) {
		m_sleepCounter ++;	
		if (m_sleepCounter > 60) {
			dVector vector (0.0f);
			NewtonBodySetForce(m_newtonBody, &vector[0]);
			NewtonBodySetTorque(m_newtonBody, &vector[0]);
			NewtonBodySetVelocityNoSleep(m_newtonBody, &vector[0]);
			NewtonBodySetVelocityNoSleep(m_newtonBody, &vector[0]);
			return;
		}
	}

//xxxx++;
//dTrace(("\nframe %d ", xxxx));

	ApplyExternalForce();
	CalculateTireContacts(timestep);
	dVehicleSolver::Update(timestep);
	Integrate(timestep);
	CalculateFreeDof(timestep);

#if 0
	FILE* file_xxx = fopen("xxxxx.csv", "wb");
	fprintf(file_xxx, "tireforce\n");
	m_load = 1000.0f;
	for (int i = 0; i < 100; i++) {
		float u = 0.01f * i;
		TireForces(0.0f, u, 1.0f);
		//fprintf(file_xxx, "%f,\n", m_tireModel.m_longitunalForce);
		fprintf(file_xxx, "%f,\n", m_tireModel.m_lateralForce);
	}
	fclose(file_xxx);
#endif
}


