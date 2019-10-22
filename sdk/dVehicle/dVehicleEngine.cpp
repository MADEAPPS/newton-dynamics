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
#include "dVehicleEngine.h"
#include "dVehicleChassis.h"
#include "dVehicleDifferential.h"


dVehicleEngine::dEngineMetricInfo::dEngineMetricInfo(const dEngineInfo& info)
	:dEngineInfo(info)
{
	const dFloat horsePowerToWatts = 735.5f;
	const dFloat rpmToRadiansPerSecunds = 0.105f;
	const dFloat poundFootToNewtonMeters = 1.356f;

	m_clutchTorque *= poundFootToNewtonMeters;
	m_idleTorque *= poundFootToNewtonMeters;
	m_peakTorque *= poundFootToNewtonMeters;

	m_rpmAtPeakTorque *= rpmToRadiansPerSecunds;
	m_rpmAtPeakHorsePower *= rpmToRadiansPerSecunds;
	m_rpmAtRedLine *= rpmToRadiansPerSecunds;
	m_rpmAtIdleTorque *= rpmToRadiansPerSecunds;

	m_peakHorsePower *= horsePowerToWatts;
	m_peakPowerTorque = m_peakHorsePower / m_rpmAtPeakHorsePower;

	dAssert(m_rpmAtIdleTorque > 0.0f);
	dAssert(m_rpmAtIdleTorque < m_rpmAtPeakHorsePower);
	dAssert(m_rpmAtPeakTorque < m_rpmAtPeakHorsePower);
	dAssert(m_rpmAtPeakHorsePower < m_rpmAtRedLine);

	dAssert(m_idleTorque > 0.0f);
	dAssert(m_peakTorque > m_peakPowerTorque);
	dAssert((m_peakTorque * m_rpmAtPeakTorque) < m_peakHorsePower);
}

dFloat dVehicleEngine::dEngineMetricInfo::GetTorque (dFloat rpm) const
{
	dAssert(0);
	return 0;
/*
	dAssert(rpm >= -0.1f);
	const int maxIndex = sizeof (m_torqueCurve) / sizeof (m_torqueCurve[0]);
	rpm = dClamp(rpm, dFloat(0.0f), m_torqueCurve[maxIndex - 1].m_rpm);

	for (int i = 1; i < maxIndex; i++) {
		if (rpm <= m_torqueCurve[i].m_rpm) {
			dFloat rpm1 = m_torqueCurve[i].m_rpm;
			dFloat rpm0 = m_torqueCurve[i - 1].m_rpm;

			dFloat torque1 = m_torqueCurve[i].m_torque;
			dFloat torque0 = m_torqueCurve[i - 1].m_torque;

			dFloat torque = torque0 + (rpm - rpm0) * (torque1 - torque0) / (rpm1 - rpm0);
			return torque;
		}
	}

	return m_torqueCurve[maxIndex - 1].m_torque;
*/
}


dVehicleEngine::dVehicleEngine(dVehicleChassis* const chassis, const dEngineInfo& info, dVehicleDifferential* const differential)
	:dVehicleNode(chassis)
	,dBilateralJoint()
	,m_localAxis(dYawMatrix(90.0f * dDegreeToRad))
	,m_info(info)
	,m_metricInfo(info)
	,m_differential(differential)
	//,m_blockJoint()
	//,m_crankJoint()
	//,m_gearBox()
	,m_omega(0.0f)
{
	Init(&m_proxyBody, &GetParent()->GetProxyBody());

	dFloat inertia = (2.0f / 5.0f) * m_metricInfo.m_mass * m_metricInfo.m_armatureRadius * m_metricInfo.m_armatureRadius;
	m_proxyBody.SetMass(m_metricInfo.m_mass);
	m_proxyBody.SetInertia(inertia, inertia, inertia);
	m_proxyBody.UpdateInertia();
/*
	dVehicleSingleBody* const chassis = (dVehicleSingleBody*) ((dVehicleNode*)m_parent)->GetAsVehicle();
	// set the tire joint
	m_blockJoint.Init(&m_proxyBody, chassis->GetProxyBody());

	m_gearBox.Init(&m_proxyBody, differential->GetProxyBody());
	m_gearBox.SetOwners(this, differential);

	m_crankJoint.Init(&m_proxyBody, chassis->m_groundNode.GetProxyBody());
	m_crankJoint.SetOwners(this, &chassis->m_groundNode);

	SetInfo(info);
*/

	m_gearBox.SetOwners(this, m_differential);
	m_gearBox.Init(&m_proxyBody, &m_differential->GetProxyBody());
}

dVehicleEngine::~dVehicleEngine()
{
}

void dVehicleEngine::SetInfo(const dEngineInfo& info)
{
	m_info = info;
	InitEngineTorqueCurve();
}

dFloat dVehicleEngine::GetSpeed() const
{
	dMatrix matrix;
	dVector veloc(0.0f);
	dAssert (GetParent()->GetAsVehicle());
	NewtonBody* const chassis = GetParent()->GetAsVehicle()->GetBody();

	NewtonBodyGetMatrix(chassis, &matrix[0][0]);
	NewtonBodyGetVelocity(chassis, &veloc[0]);
	return veloc.DotProduct3(matrix.m_front);
}


void dVehicleEngine::InitEngineTorqueCurve()
{
	m_metricInfo = dEngineMetricInfo(m_info);

	m_metricInfo.m_torqueCurve[0] = dEngineTorqueNode(0.0f, m_metricInfo.m_idleTorque);
	m_metricInfo.m_torqueCurve[1] = dEngineTorqueNode(m_metricInfo.m_rpmAtIdleTorque, m_metricInfo.m_idleTorque);
	m_metricInfo.m_torqueCurve[2] = dEngineTorqueNode(m_metricInfo.m_rpmAtPeakTorque, m_metricInfo.m_peakTorque);
	m_metricInfo.m_torqueCurve[3] = dEngineTorqueNode(m_metricInfo.m_rpmAtPeakHorsePower, m_metricInfo.m_peakPowerTorque);
	m_metricInfo.m_torqueCurve[4] = dEngineTorqueNode(m_metricInfo.m_rpmAtRedLine, m_metricInfo.m_idleTorque);
}

#if 0
dFloat dVehicleEngine::GetRpm() const
{
	return -m_omega * 9.549f;
}

dFloat dVehicleEngine::GetRedLineRpm() const
{
	return m_metricInfo.m_rpmAtRedLine * 9.549f;
}

void dVehicleEngine::SetThrottle (dFloat throttle)
{
	dFloat torque = m_metricInfo.GetTorque(dAbs (m_omega));
	dFloat omega = m_metricInfo.m_rpmAtRedLine * dClamp(throttle, dFloat (0.0f), dFloat (1.0f));
	m_crankJoint.SetTorqueAndRpm(torque, omega);
}

void dVehicleEngine::SetGear (int gear)
{
	gear = dClamp (gear, int (m_reverseGear), m_metricInfo.m_gearsCount);
	dFloat ratio = m_metricInfo.m_gearRatios[gear];
	m_gearBox.SetGearRatio(ratio);
}

void dVehicleEngine::SetClutch (dFloat clutch) 
{
	clutch = dClamp (clutch, dFloat (0.0f), dFloat (1.0f));
	m_gearBox.SetClutchTorque(clutch * m_metricInfo.m_clutchTorque);
}
#endif

const void dVehicleEngine::Debug(dCustomJoint::dDebugDisplay* const debugContext) const
{
//	dAssert(0);
//	dVehicleEngineInterface::Debug(debugContext);
}

int dVehicleEngine::GetKinematicLoops(dVehicleLoopJoint** const jointArray)
{
	jointArray[0] = &m_gearBox;
	return 1;
}

void dVehicleEngine::CalculateFreeDof()
{
	dVehicleChassis* const chassisNode = GetParent()->GetAsVehicle();
	dComplementaritySolver::dBodyState* const chassisBody = &chassisNode->GetProxyBody();
	const dMatrix chassisMatrix(m_localAxis * chassisBody->GetMatrix());

	dVector omega(m_proxyBody.GetOmega());
	dVector chassisOmega(chassisBody->GetOmega());
	dVector relativeOmega(omega - chassisOmega);
	m_omega = chassisMatrix.m_front.DotProduct3(relativeOmega);
}

void dVehicleEngine::Integrate(dFloat timestep)
{
	m_proxyBody.IntegrateForce(timestep, m_proxyBody.GetForce(), m_proxyBody.GetTorque());
}

void dVehicleEngine::ApplyExternalForce()
{
	dVehicleChassis* const chassisNode = GetParent()->GetAsVehicle();
	dComplementaritySolver::dBodyState* const chassisBody = &chassisNode->GetProxyBody();

	dMatrix matrix(chassisBody->GetMatrix());
	matrix.m_posit = matrix.TransformVector(chassisBody->GetCOM());
	m_proxyBody.SetMatrix(matrix);

	matrix = m_localAxis * matrix;
	m_proxyBody.SetVeloc(chassisBody->GetVelocity());
	m_proxyBody.SetOmega(chassisBody->GetOmega() + matrix.m_front.Scale(m_omega));
	m_proxyBody.SetTorque(dVector(0.0f));

dTrace (("speed:%f (kmh)\n", GetSpeed() * 3.6f));
dVector xxxx(matrix.m_front.Scale(-500.0f));
m_proxyBody.SetTorque(xxxx);

	m_proxyBody.SetForce(chassisNode->GetGravity().Scale(m_proxyBody.GetMass()));
}

void dVehicleEngine::UpdateSolverForces(const dComplementaritySolver::dJacobianPair* const jacobians) const
{
	dAssert(0);
}

void dVehicleEngine::JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams)
{
	dComplementaritySolver::dBodyState* const engine = m_state0;
	dMatrix matrix(m_localAxis * engine->GetMatrix());

	/// three rigid attachment to chassis
	AddLinearRowJacobian(constraintParams, matrix.m_posit, matrix.m_front);
	AddLinearRowJacobian(constraintParams, matrix.m_posit, matrix.m_up);
	AddLinearRowJacobian(constraintParams, matrix.m_posit, matrix.m_right);

	// angular constraints	
	AddAngularRowJacobian(constraintParams, matrix.m_up, 0.0f);
	AddAngularRowJacobian(constraintParams, matrix.m_right, 0.0f);
}

void dVehicleEngine::dGearBoxAndClutchJoint::JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams)
{
	m_dof = 0;
	m_count = 0;
	constraintParams->m_count = 0;
	if (dAbs(m_gearRatio) > 1.0e-3f) {

		dVehicleEngine* const engineNode = GetOwner0()->GetAsEngine();
		dAssert (engineNode);
		//dComplementaritySolver::dBodyState* const chassis = &engineNode->GetProxyBody();
		//const dEngineInfo& info = engineNode->GetInfo();

		const dMatrix& matrix = m_state0->GetMatrix();
		AddAngularRowJacobian(constraintParams, matrix.m_right, 0.0f);

		dComplementaritySolver::dJacobian &jacobian0 = constraintParams->m_jacobians[0].m_jacobian_J01;
		dComplementaritySolver::dJacobian &jacobian1 = constraintParams->m_jacobians[0].m_jacobian_J10;

		//dFloat gain = info.m_crownGear * m_gearRatio;
		dFloat gain = 1.0f;
		jacobian1.m_angular = jacobian1.m_angular.Scale(gain);

		const dVector& omega0 = m_state0->GetOmega();
		const dVector& omega1 = m_state1->GetOmega();

		const dVector relVeloc(omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular);
		dFloat w = relVeloc.m_x + relVeloc.m_y + relVeloc.m_z;

		constraintParams->m_jointAccel[0] = -w * constraintParams->m_timestepInv;
		constraintParams->m_jointLowFrictionCoef[0] = -m_clutchTorque;
		constraintParams->m_jointHighFrictionCoef[0] = m_clutchTorque;

		//constraintParams->m_jointLowFrictionCoef[0] = -50;
		//constraintParams->m_jointHighFrictionCoef[0] = 50;

		m_dof = 1;
		m_count = 1;
		constraintParams->m_count = 1;
	}
}