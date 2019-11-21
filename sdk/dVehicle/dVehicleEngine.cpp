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
#include "dVehicleMultiBody.h"
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
}

dVehicleEngine::dVehicleEngine(dVehicleMultiBody* const chassis, const dEngineInfo& info, dVehicleDifferential* const differential)
	:dVehicleNode(chassis)
	,dBilateralJoint()
	,m_localAxis(dYawMatrix(-90.0f * dDegreeToRad))
	,m_info(info)
	,m_metricInfo(info)
	,m_gearBox()
	,m_differential(differential)
	,m_omega(0.0f)
	,m_throttle(0.0f)
	,m_throttleSpeed(1.e1f)
	,m_differentialMode(differential->GetMode())
	,m_currentGear(dEngineInfo::m_neutralGear)
	,m_ignitionKey0 (false)
	,m_ignitionKey1(false)
{
m_currentGear = dEngineInfo::m_firstGear;

	InitEngineTorqueCurve();
	Init(&m_proxyBody, &GetParent()->GetProxyBody());

	dFloat inertia = (2.0f / 5.0f) * m_metricInfo.m_mass * m_metricInfo.m_armatureRadius * m_metricInfo.m_armatureRadius;
	m_proxyBody.SetMass(m_metricInfo.m_mass);
	m_proxyBody.SetInertia(inertia, inertia, inertia);
	m_proxyBody.UpdateInertia();

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
	dAssert (GetParent()->GetAsVehicleMultiBody());
	NewtonBody* const chassis = GetParent()->GetAsVehicleMultiBody()->GetBody();

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

dFloat dVehicleEngine::GetRpm() const
{
	return -m_omega * 9.549f;
}

dFloat dVehicleEngine::GetRedLineRpm() const
{
	return m_metricInfo.m_rpmAtRedLine * 9.549f;
}

void dVehicleEngine::SetGear(dEngineInfo::dGearRatioIndex gear)
{
//	m_gearTimer = 30;
	m_currentGear = dClamp(gear, dEngineInfo::m_reverseGear, dEngineInfo::dGearRatioIndex (m_metricInfo.m_gearsCount));
	dFloat ratio = m_metricInfo.m_gearRatios[m_currentGear];
	m_gearBox.SetGearRatio(ratio);
}

void dVehicleEngine::UpdateAutomaticGearBox(dFloat timestep)
{
m_metricInfo.m_gearsCount = 4;
//	m_gearTimer--;
//	if (m_gearTimer < 0) {

	dFloat omega = dAbs (m_omega);

//dTrace (("(gear: %d) (throttle: %f) (omega: %f %f %f) ", m_currentGear, m_throttle, omega, m_metricInfo.m_rpmAtPeakTorque, m_metricInfo.m_rpmAtPeakHorsePower));
//SetGear(dEngineInfo::dGearRatioIndex(dEngineInfo::m_firstGear + 1));
//return;
		
		switch (m_currentGear) 
		{
			case dEngineInfo::m_neutralGear:
			{
				SetGear(dEngineInfo::m_neutralGear);
				break;
			}

			case dEngineInfo::m_reverseGear:
			{
			   SetGear(dEngineInfo::m_reverseGear);
			   break;
			}

			default:
			{
				if (omega > m_metricInfo.m_rpmAtPeakHorsePower) {
					if (m_currentGear < (m_metricInfo.m_gearsCount - 1)) {
						SetGear(dEngineInfo::dGearRatioIndex (m_currentGear + 1));
					}
				} else if (omega < m_metricInfo.m_rpmAtPeakTorque) {
					if (m_currentGear > dEngineInfo::m_firstGear) {
						SetGear(dEngineInfo::dGearRatioIndex (m_currentGear - 1));
					}
				}
			}
		}
//	}
}

bool dVehicleEngine::InputChanged() const
{
	if (m_ignitionKey0 != m_ignitionKey1) {
		return true;
	}

	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	const dComplementaritySolver::dBodyState& proxy = GetProxyBody();
	proxy.GetInertia(Ixx, Iyy, Izz);
	dVector alpha(proxy.GetTorque().Scale(1.0f / Ixx));
	dFloat alphaMag2 = alpha.DotProduct3(alpha);
	return alphaMag2 > 1.0f;
}

void dVehicleEngine::SetIgnition(bool mode)
{
	m_ignitionKey1 = m_ignitionKey0;
	m_ignitionKey0 = mode;
}

void dVehicleEngine::SetThrottle (dFloat throttle, dFloat timestep)
{
	if (m_ignitionKey0) {
		throttle = dMax(throttle, m_metricInfo.m_rpmAtIdleTorque / m_metricInfo.m_rpmAtRedLine);
	} else {
		throttle = 0.0f;
	}

	m_throttle = dAbs (m_omega / m_metricInfo.m_rpmAtRedLine);
	dFloat step = throttle - m_throttle;
	dFloat maxStep = 2.0f * m_throttleSpeed * timestep;
	if (step > maxStep) {
		step = m_throttleSpeed * timestep;
	} else if (step < -maxStep) {
		step = -m_throttleSpeed * timestep;
	} else {
		step *= 0.25f;
	}
	m_throttle = dClamp(m_throttle + step, dFloat(0.0f), dFloat(1.0f));
}

void dVehicleEngine::SetDifferentialMode(int differentialMode) 
{
	if (differentialMode != m_differentialMode) {
		m_differentialMode = differentialMode;
		m_differential->SetMode(m_differentialMode);
	}
}

#if 0
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
	dVehicleMultiBody* const chassisNode = GetParent()->GetAsVehicleMultiBody();
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
	dVehicleMultiBody* const chassisNode = GetParent()->GetAsVehicleMultiBody();
	dComplementaritySolver::dBodyState* const chassisBody = &chassisNode->GetProxyBody();

	dMatrix matrix(chassisBody->GetMatrix());
	matrix.m_posit = matrix.TransformVector(chassisBody->GetCOM());
	m_proxyBody.SetMatrix(matrix);

	matrix = m_localAxis * matrix;
	m_proxyBody.SetVeloc(chassisBody->GetVelocity());
	m_proxyBody.SetOmega(chassisBody->GetOmega() + matrix.m_front.Scale(m_omega));
	m_proxyBody.SetTorque(dVector(0.0f));
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

	// try steady state calibration
	int index = constraintParams->m_count;
	dFloat rpm = m_throttle * m_metricInfo.m_rpmAtRedLine;
	dFloat torque = m_metricInfo.GetTorque(rpm);

	AddAngularRowJacobian(constraintParams, matrix.m_front, 0.0f);
	constraintParams->m_jointAccel[index] -= rpm * constraintParams->m_timestepInv;
	constraintParams->m_jointLowFrictionCoef[index] = -torque;
	constraintParams->m_jointHighFrictionCoef[index] = torque;

//dTrace (("rpm %f   torque %f\n", rpm, torque));
}

void dVehicleEngine::dGearBoxAndClutchJoint::JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams)
{
	m_dof = 0;
	m_count = 0;
	constraintParams->m_count = 0;
	if (dAbs(m_gearRatio) > 1.0e-3f) {

		dAssert(GetOwner0()->GetAsEngine());
		dVehicleEngine* const engineNode = GetOwner0()->GetAsEngine();
		dVector gearBoxPin (m_state0->GetMatrix().RotateVector(engineNode->m_localAxis.m_front));
		AddAngularRowJacobian(constraintParams, gearBoxPin, 0.0f);

		dComplementaritySolver::dJacobian &jacobian0 = constraintParams->m_jacobians[0].m_jacobian_J01;
		dComplementaritySolver::dJacobian &jacobian1 = constraintParams->m_jacobians[0].m_jacobian_J10;

		//dFloat gain = -1.0f;
		dFloat gain = m_crowndGear * m_gearRatio;
//dTrace (("(gearGain %f) ", gain));
//if (gain < 11)
//gain = 5;

		jacobian1.m_angular = jacobian1.m_angular.Scale(-gain);

		const dVector& omega0 = m_state0->GetOmega();
		const dVector& omega1 = m_state1->GetOmega();

		const dVector relVeloc(omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular);
		dFloat w = relVeloc.m_x + relVeloc.m_y + relVeloc.m_z;

		constraintParams->m_jointAccel[0] = -w * constraintParams->m_timestepInv;
		constraintParams->m_jointLowFrictionCoef[0] = -m_clutchTorque;
		constraintParams->m_jointHighFrictionCoef[0] = m_clutchTorque;

		m_dof = 1;
		m_count = 1;
		constraintParams->m_count = 1;
	}
}