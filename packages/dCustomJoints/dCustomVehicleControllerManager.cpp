/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


#include "dCustomJointLibraryStdAfx.h"
#include "dCustomJoint.h"
#include "dCustomGear.h"
#include "dCustomHinge.h"
#include "dCustomUniversal.h"
#include "dCustomVehicleControllerManager.h"

//#define D_PLOT_ENGINE_CURVE


#ifdef D_PLOT_ENGINE_CURVE 
static FILE* file_xxx;
#endif


#define D_VEHICLE_NEUTRAL_GEAR						0
#define D_VEHICLE_REVERSE_GEAR						1
#define D_VEHICLE_FIRST_GEAR						2
#define D_VEHICLE_MAX_DRIVETRAIN_DOF				32
#define D_VEHICLE_REGULARIZER						dFloat(1.0001f)


#define D_LIMITED_SLIP_DIFFERENTIAL_LOCK_RPS		dFloat(10.0f)

#define D_VEHICLE_ENGINE_IDLE_GAS_VALVE				dFloat(0.1f)
#define D_VEHICLE_ENGINE_IDLE_FRICTION_COEFFICIENT	dFloat(0.25f)

#define D_VEHICLE_MAX_SIDESLIP_ANGLE				dFloat(35.0f * 3.1416f / 180.0f)
#define D_VEHICLE_MAX_SIDESLIP_RATE					dFloat(15.0f * 3.1416f / 180.0f)

class dCustomVehicleControllerManager::dTireFilter: public dCustomControllerConvexCastPreFilter
{
	public:
	dTireFilter(const dCustomVehicleController::dBodyPartTire* const tire, const dCustomVehicleController* const controller)
		:dCustomControllerConvexCastPreFilter(tire->GetBody())
		,m_tire (tire)
		,m_controller(controller)
	{
	}

	unsigned Prefilter(const NewtonBody* const body, const NewtonCollision* const myCollision)
	{
		dAssert(body != m_me);
		for (int i = 0; i < m_tire->m_collidingCount; i ++) {
			if (m_tire->m_contactInfo[i].m_hitBody == body) {
				return 0;
			}
		}

		for (dList<dCustomVehicleController::dBodyPart*>::dListNode* node = m_controller->m_bodyPartsList.GetFirst(); node; node = node->GetNext()) {
			if (node->GetInfo()->GetBody() == body) {
				return 0;
			}
		}

		return (body != m_controller->GetBody()) ? 1 : 0;
	}

	const dCustomVehicleController* m_controller;
	const dCustomVehicleController::dBodyPartTire* m_tire;
};


class dCustomVehicleController::dEngineJoint: public dCustomHinge
{
	public:
		dEngineJoint(const dMatrix& pinAndPivotFrame, NewtonBody* const engineBody, NewtonBody* const chassisBody)
		:dCustomHinge(pinAndPivotFrame, engineBody, chassisBody)
		,m_maxrmp(50.0f)
	{
		dMatrix engineMatrix;
		dMatrix chassisMatrix;

		EnableLimits(false);
		SetDryFriction(0.0f);

		NewtonBodyGetMatrix(engineBody, &engineMatrix[0][0]);
		NewtonBodyGetMatrix(chassisBody, &chassisMatrix[0][0]);
		m_baseOffsetMatrix = engineMatrix * chassisMatrix.Inverse();
	}

	void SetRedLineRPM(dFloat redLineRmp)
	{
		m_maxrmp = dAbs(redLineRmp);
	}

	void SetDryFriction(dFloat friction)
	{
		friction = dAbs(friction);
		SetFriction(friction > 1.0f ? friction : 0.0f);
	}

	void ProjectError()
	{
		dMatrix chassisMatrix;
		dVector engineOmega(0.0f);
		dVector chassisOmega(0.0f);
		dVector engineVelocity(0.0f);

		NewtonBody* const engineBody = GetBody0();
		NewtonBody* const chassisBody = GetBody1();
		
		NewtonBodyGetMatrix(chassisBody, &chassisMatrix[0][0]);
		dMatrix engineMatrix(m_baseOffsetMatrix * chassisMatrix);
		NewtonBodySetMatrixNoSleep(engineBody, &engineMatrix[0][0]);

		NewtonBodyGetPointVelocity(chassisBody, &engineMatrix.m_posit[0], &engineVelocity[0]);
		NewtonBodySetVelocityNoSleep(engineBody, &engineVelocity[0]);

		NewtonBodyGetOmega(engineBody, &engineOmega[0]);
		NewtonBodyGetOmega(chassisBody, &chassisOmega[0]);

		chassisMatrix = GetMatrix1() * chassisMatrix;
		dVector projectOmega(chassisMatrix.m_front.Scale(engineOmega.DotProduct3(chassisMatrix.m_front)) +
							 chassisMatrix.m_up.Scale(chassisOmega.DotProduct3(chassisMatrix.m_up)) +
							 chassisMatrix.m_right.Scale(chassisOmega.DotProduct3(chassisMatrix.m_right)));
		NewtonBodySetOmegaNoSleep(engineBody, &projectOmega[0]);
	}

	void SubmitConstraintsFreeDof(dFloat timestep, const dMatrix& matrix0, const dMatrix& matrix1)
	{
		dFloat alpha = m_jointOmega / timestep;
		NewtonUserJointAddAngularRow(m_joint, 0, &matrix1.m_front[0]);
		NewtonUserJointSetRowAcceleration(m_joint, -alpha);
		if (m_jointOmega <= 0.0f) {
			dFloat idleAlpha = - m_jointOmega / timestep;
			NewtonUserJointSetRowAcceleration(m_joint, idleAlpha);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_friction);
		} else if (m_jointOmega >= m_maxrmp) {
			dFloat redLineAlpha = (m_maxrmp - m_jointOmega) / timestep;
			NewtonUserJointSetRowAcceleration(m_joint, redLineAlpha);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_friction);
		} else {
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_friction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_friction);
		}
		NewtonUserJointSetRowStiffness(m_joint, 1.0f);
	}

	dMatrix m_baseOffsetMatrix;
	dFloat m_maxrmp;
};


class dCustomVehicleController::dDifferentialJoint: public dCustomUniversal
{
	public:
	dDifferentialJoint(const dMatrix& pinAndPivotFrame, NewtonBody* const differentialBody, NewtonBody* const chassisBody)
		:dCustomUniversal(pinAndPivotFrame, differentialBody, chassisBody)
		,m_slipDifferentialSpeed(0.0f)
		,m_slipDifferentialOn(true)
	{
		dMatrix engineMatrix;
		dMatrix chassisMatrix;

		EnableLimit_0(false);
		EnableLimit_1(false);
		NewtonBodyGetMatrix(differentialBody, &engineMatrix[0][0]);
		NewtonBodyGetMatrix(chassisBody, &chassisMatrix[0][0]);
		m_baseOffsetMatrix = engineMatrix * chassisMatrix.Inverse();
	}

	void ProjectError()
	{
		dMatrix chassisMatrix;
		dVector chassisOmega(0.0f);
		dVector differentialOmega(0.0f);
		dVector differentialVelocity(0.0f);

		NewtonBody* const chassisBody = GetBody1();
		NewtonBody* const differentialBody = GetBody0();

		NewtonBodyGetMatrix(chassisBody, &chassisMatrix[0][0]);
		dMatrix differentialMatrix(m_baseOffsetMatrix * chassisMatrix);
		NewtonBodySetMatrixNoSleep(differentialBody, &differentialMatrix[0][0]);

		NewtonBodyGetPointVelocity(chassisBody, &differentialMatrix.m_posit[0], &differentialVelocity[0]);
		NewtonBodySetVelocityNoSleep(differentialBody, &differentialVelocity[0]);

		NewtonBodyGetOmega(chassisBody, &chassisOmega[0]);
		NewtonBodyGetOmega(differentialBody, &differentialOmega[0]);

		chassisMatrix = GetMatrix1() * chassisMatrix;
		dVector projectOmega(chassisMatrix.m_front.Scale(differentialOmega.DotProduct3(chassisMatrix.m_front)) +
							 chassisMatrix.m_up.Scale(differentialOmega.DotProduct3(chassisMatrix.m_up)) +
							 chassisMatrix.m_right.Scale(chassisOmega.DotProduct3(chassisMatrix.m_right)));
		NewtonBodySetOmegaNoSleep(differentialBody, &differentialOmega[0]);
	}

	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		dMatrix chassisMatrix;
		dMatrix differentialMatrix;
		dVector chassisOmega(0.0f);
		dVector differentialOmega(0.0f);

		dCustomUniversal::SubmitConstraints(timestep, threadIndex);

		// y axis controls the slip differential feature.
		NewtonBody* const chassisBody = GetBody1();
		NewtonBody* const diffentialBody = GetBody0();

		NewtonBodyGetOmega(diffentialBody, &differentialOmega[0]);
		NewtonBodyGetOmega(chassisBody, &chassisOmega[0]);

		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		CalculateGlobalMatrix(differentialMatrix, chassisMatrix);
		dVector relOmega(differentialOmega - chassisOmega);

		// apply differential
		dFloat differentailOmega = differentialMatrix.m_front.DotProduct3(relOmega);
		if (differentailOmega > D_LIMITED_SLIP_DIFFERENTIAL_LOCK_RPS) {
			dFloat wAlpha = (D_LIMITED_SLIP_DIFFERENTIAL_LOCK_RPS - differentailOmega) / timestep;
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &differentialMatrix.m_front[0]);
			NewtonUserJointSetRowAcceleration(m_joint, wAlpha);
			NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
		} else if (differentailOmega < -D_LIMITED_SLIP_DIFFERENTIAL_LOCK_RPS) {
			dFloat wAlpha = (-D_LIMITED_SLIP_DIFFERENTIAL_LOCK_RPS - differentailOmega) / timestep;
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &differentialMatrix.m_front[0]);
			NewtonUserJointSetRowAcceleration(m_joint, wAlpha);
			NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
		}
	}

	dMatrix m_baseOffsetMatrix;
	dFloat m_slipDifferentialSpeed;
	bool m_slipDifferentialOn;
};


class dCustomVehicleController::dWheelJoint: public dCustomJoint
{
	public:
	dWheelJoint(const dMatrix& pinAndPivotFrame, NewtonBody* const tireBody, NewtonBody* const chassisBody, dBodyPartTire* const tireData)
		:dCustomJoint(6, tireBody, chassisBody)
		,m_lateralDir(0.0f)
		,m_longitudinalDir(0.0f)
		,m_tire(tireData)
		,m_tireLoad(0.0f)
		,m_steerRate(0.5f * 3.1416f)
		,m_steerAngle0(0.0f)
		,m_steerAngle1(0.0f)
		,m_brakeTorque(0.0f)
	{
		CalculateLocalMatrix(pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
	}

	dFloat CalculateTireParametricPosition(const dMatrix& tireMatrix, const dMatrix& chassisMatrix) const
	{
		const dVector& chassisP0 = chassisMatrix.m_posit;
		dVector chassisP1(chassisMatrix.m_posit + chassisMatrix.m_up.Scale(m_tire->m_data.m_suspesionlenght));
		dVector p1p0(chassisP1 - chassisP0);
		dVector q1p0(tireMatrix.m_posit - chassisP0);
		dFloat num = q1p0.DotProduct3(p1p0);
		dFloat den = p1p0.DotProduct3(p1p0);
		return num / den;
	}

	void ProjectError()
	{
		dMatrix tireMatrix;
		dMatrix chassisMatrix;
		dVector tireVeloc(0.0f);
		dVector tireOmega(0.0f);
		dVector chassisVeloc(0.0f);
		dVector chassisOmega(0.0f);

		NewtonBody* const tire = m_body0;
		NewtonBody* const chassis = m_body1;
		dAssert(m_body0 == m_tire->m_joint->GetBody0());

		CalculateGlobalMatrix(tireMatrix, chassisMatrix);
		chassisMatrix = dYawMatrix(m_steerAngle0) * chassisMatrix;

		tireMatrix.m_front = chassisMatrix.m_front;
		tireMatrix.m_right = tireMatrix.m_front.CrossProduct(tireMatrix.m_up);
		tireMatrix.m_right = tireMatrix.m_right.Scale(1.0f / dSqrt(tireMatrix.m_right.DotProduct3(tireMatrix.m_right)));
		tireMatrix.m_up = tireMatrix.m_right.CrossProduct(tireMatrix.m_front);

		dVector projectPosition(chassisMatrix.m_up.Scale(tireMatrix.m_posit.DotProduct3(chassisMatrix.m_up)) +
								chassisMatrix.m_front.Scale(chassisMatrix.m_posit.DotProduct3(chassisMatrix.m_front)) +
								chassisMatrix.m_right.Scale(chassisMatrix.m_posit.DotProduct3(chassisMatrix.m_right)));
		tireMatrix.m_posit = projectPosition;

		tireMatrix = GetMatrix0().Inverse() * tireMatrix;
		NewtonBodySetMatrixNoSleep(tire, &tireMatrix[0][0]);

		NewtonBodyGetVelocity(tire, &tireVeloc[0]);
		NewtonBodyGetPointVelocity(chassis, &tireMatrix.m_posit[0], &chassisVeloc[0]);
		dVector projectVelocity(chassisMatrix.m_up.Scale(tireVeloc.DotProduct3(chassisMatrix.m_up)) +
								chassisMatrix.m_front.Scale(chassisVeloc.DotProduct3(chassisMatrix.m_front)) +
								chassisMatrix.m_right.Scale(chassisVeloc.DotProduct3(chassisMatrix.m_right)));
		NewtonBodySetVelocityNoSleep(tire, &projectVelocity[0]);

		NewtonBodyGetOmega(tire, &tireOmega[0]);
		NewtonBodyGetOmega(chassis, &chassisOmega[0]);
		dVector projectOmega(chassisMatrix.m_front.Scale(tireOmega.DotProduct3(chassisMatrix.m_front)) +
							 chassisMatrix.m_up.Scale(chassisOmega.DotProduct3(chassisMatrix.m_up)) +
							 chassisMatrix.m_right.Scale(chassisOmega.DotProduct3(chassisMatrix.m_right)));
		NewtonBodySetOmegaNoSleep(tire, &projectOmega[0]);
	}

	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		dMatrix tireMatrix;
		dMatrix chassisMatrix;

		NewtonBody* const tire = m_body0;
		NewtonBody* const chassis = m_body1;
		dAssert(m_body0 == m_tire->GetBody());
		dAssert(m_body1 == m_tire->GetParent()->GetBody());

		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		CalculateGlobalMatrix(tireMatrix, chassisMatrix);
		chassisMatrix = dYawMatrix(m_steerAngle0) * chassisMatrix;

		m_lateralDir = chassisMatrix.m_front;
		m_longitudinalDir = chassisMatrix.m_right;

		NewtonUserJointAddLinearRow(m_joint, &tireMatrix.m_posit[0], &chassisMatrix.m_posit[0], &m_lateralDir[0]);
		NewtonUserJointSetRowAcceleration(m_joint, NewtonUserCalculateRowZeroAccelaration(m_joint));

		NewtonUserJointAddLinearRow(m_joint, &tireMatrix.m_posit[0], &chassisMatrix.m_posit[0], &m_longitudinalDir[0]);
		NewtonUserJointSetRowAcceleration(m_joint, NewtonUserCalculateRowZeroAccelaration(m_joint));

		dFloat angle = -CalculateAngle(tireMatrix.m_front, chassisMatrix.m_front, chassisMatrix.m_right);
		NewtonUserJointAddAngularRow(m_joint, -angle, &chassisMatrix.m_right[0]);
		NewtonUserJointSetRowAcceleration(m_joint, NewtonUserCalculateRowZeroAccelaration(m_joint));

		angle = -CalculateAngle(tireMatrix.m_front, chassisMatrix.m_front, chassisMatrix.m_up);
		NewtonUserJointAddAngularRow(m_joint, -angle, &chassisMatrix.m_up[0]);
		NewtonUserJointSetRowAcceleration(m_joint, NewtonUserCalculateRowZeroAccelaration(m_joint));

		dFloat param = CalculateTireParametricPosition(tireMatrix, chassisMatrix);
		if (param >= 1.0f) {
			dVector posit(chassisMatrix.m_posit + m_tire->m_data.m_suspesionlenght);
			NewtonUserJointAddLinearRow(m_joint, &tireMatrix.m_posit[0], &posit[0], &chassisMatrix.m_up[0]);
			NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
		} else if (param <= 0.0f) {
			NewtonUserJointAddLinearRow(m_joint, &tireMatrix.m_posit[0], &chassisMatrix.m_posit[0], &chassisMatrix.m_up[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
		} else if (m_tire->m_data.m_suspentionType == dBodyPartTire::Info::m_roller) {
			dAssert(0);
			NewtonUserJointAddLinearRow(m_joint, &tireMatrix.m_posit[0], &chassisMatrix.m_posit[0], &chassisMatrix.m_up[0]);
		}

		if (m_brakeTorque > 1.0e-3f) {
			dVector tireOmega(0.0f);
			dVector chassisOmega(0.0f);
			NewtonBodyGetOmega(tire, &tireOmega[0]);
			NewtonBodyGetOmega(chassis, &chassisOmega[0]);
			dVector relOmega(tireOmega - chassisOmega);

			dFloat speed = relOmega.DotProduct3(m_lateralDir);
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &m_lateralDir[0]);
			NewtonUserJointSetRowAcceleration(m_joint, -speed / timestep);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_brakeTorque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_brakeTorque);
		}

		m_brakeTorque = 0.0f;
	}

	dFloat GetTireLoad() const
	{
		return NewtonUserJointGetRowForce(m_joint, 4);
	}

	dVector GetLongitudinalForce() const
	{
		return m_longitudinalDir.Scale(NewtonUserJointGetRowForce(m_joint, 1));
	}

	dVector GetLateralForce() const
	{
		return m_lateralDir.Scale(NewtonUserJointGetRowForce(m_joint, 0));
	}

	dVector m_lateralDir;
	dVector m_longitudinalDir;
	dBodyPartTire* m_tire;
	dFloat m_tireLoad;
	dFloat m_steerRate;
	dFloat m_steerAngle0;
	dFloat m_steerAngle1;
	dFloat m_brakeTorque;
};

class dCustomVehicleController::dGearBoxJoint: public dCustomGear
{
	public:
	dGearBoxJoint(const dVector& childPin, NewtonBody* const differential, NewtonBody* const engine, dFloat maxFrictionToque)
		:dCustomGear(1, differential, engine)
		,m_param (1.0f)
		,m_cluthFrictionTorque (maxFrictionToque)
	{
		dMatrix pinAndPivotFrame(dGrammSchmidt(childPin));
		CalculateLocalMatrix(pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
	}

	void SetGearRatio(dFloat gear)
	{
		m_gearRatio = -gear;
	}

	void SetFritionTorque(dFloat param)
	{
		m_param = param;
	}

	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		if (m_param > 0.1f) {
			dCustomGear::SubmitConstraints(timestep, threadIndex);
			if (m_param < 0.9f) {
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_param * m_cluthFrictionTorque);
				NewtonUserJointSetRowMaximumFriction(m_joint,  m_param * m_cluthFrictionTorque);
			}
		}
	}

	dFloat m_param;
	dFloat m_cluthFrictionTorque;
};

class dCustomVehicleController::dAxelJoint: public dCustomGear
{
	public:
	dAxelJoint(const dVector& childPin, const dVector& parentPin, const dVector& referencePin, NewtonBody* const child, NewtonBody* const parent, NewtonBody* const parentReference)
		:dCustomGear(1.0f, childPin, parentPin, child, parent)
		,m_parentReference(parentReference)
	{
		dMatrix dommyMatrix;
		// calculate the local matrix for body body0
 		dMatrix pinAndPivot0(dGrammSchmidt(childPin));

		CalculateLocalMatrix(pinAndPivot0, m_localMatrix0, dommyMatrix);
		m_localMatrix0.m_posit = dVector(0.0f, 0.0f, 0.0f, 1.0f);

		// calculate the local matrix for body body1  
		dMatrix pinAndPivot1(dGrammSchmidt(parentPin));
		CalculateLocalMatrix(pinAndPivot1, dommyMatrix, m_localMatrix1);
		m_localMatrix1.m_posit = dVector(0.0f, 0.0f, 0.0f, 1.0f);

		dMatrix referenceMatrix;
		NewtonBodyGetMatrix(m_parentReference, &referenceMatrix[0][0]);
		m_pintOnReference = referenceMatrix.UnrotateVector(referencePin);
	}

	void SetGear(dFloat gearRatio)
	{
		m_gearRatio = gearRatio;
	}

	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		dMatrix matrix0;
		dMatrix matrix1;
		dMatrix referenceMatrix;
		dVector omega0(0.0f);
		dVector omega1(0.0f);
		dFloat jacobian0[6];
		dFloat jacobian1[6];

		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		CalculateGlobalMatrix(matrix0, matrix1);
		NewtonBodyGetMatrix(m_parentReference, &referenceMatrix[0][0]);

		// calculate the angular velocity for both bodies
		dVector dir0(matrix0.m_front);
		dVector dir2(matrix1.m_front);
		dVector dir3(referenceMatrix.RotateVector(m_pintOnReference));
		dVector dir1(dir2 + dir3);

		jacobian0[0] = 0.0f;
		jacobian0[1] = 0.0f;
		jacobian0[2] = 0.0f;
		jacobian0[3] = dir0.m_x;
		jacobian0[4] = dir0.m_y;
		jacobian0[5] = dir0.m_z;

		jacobian1[0] = 0.0f;
		jacobian1[1] = 0.0f;
		jacobian1[2] = 0.0f;
		jacobian1[3] = dir1.m_x;
		jacobian1[4] = dir1.m_y;
		jacobian1[5] = dir1.m_z;

		NewtonBodyGetOmega(m_body0, &omega0[0]);
		NewtonBodyGetOmega(m_body1, &omega1[0]);

		dFloat w0 = omega0.DotProduct3(dir0);
		dFloat w1 = omega1.DotProduct3(dir1);

		dFloat relOmega = w0 + w1;
		dFloat invTimestep = (timestep > 0.0f) ? 1.0f / timestep : 1.0f;
		dFloat relAccel = -0.5f * relOmega * invTimestep;
		NewtonUserJointAddGeneralRow(m_joint, jacobian0, jacobian1);
		NewtonUserJointSetRowAcceleration(m_joint, relAccel);
	}

	dVector m_pintOnReference;
	NewtonBody* m_parentReference;
};

/*
class dCustomVehicleController::dLateralDynamicsJoint: public dCustomJoint
{
	public:
	dLateralDynamicsJoint(dCustomVehicleController* const vehicle)
		:dCustomJoint(2, vehicle->m_chassis.GetBody(), NULL)
		,m_vehicle (vehicle)
		,m_sideSlipAngle(0.0f)
		,m_maxSideSlipAngle(35.0f * 3.1416f / 180.0f)
		,m_maxSideSlipAngleRate(15.0f * 3.1416f / 180.0f)
	{
		SetSolverModel(1);
	}

	~dLateralDynamicsJoint()
	{
	}
	
	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		dMatrix matrix;
		dVector veloc;

		NewtonBody* const chassisBody = m_vehicle->m_chassis.GetBody();
		NewtonBodyGetMatrix(chassisBody, &matrix[0][0]);
		NewtonBodyGetVelocity(chassisBody, &veloc[0]);

		dFloat speed_x = veloc.DotProduct3(matrix.m_front);
		dFloat speed_z = veloc.DotProduct3(matrix.m_right);
		if (dAbs(speed_x) > 1.0f) {

			dFloat beta = dAtan2(speed_z, dAbs(speed_x));
			dFloat betaRate = (beta - m_sideSlipAngle) / timestep;
			m_sideSlipAngle = beta;
			dTrace(("b(%f) rate(%f)\n", beta * 180.0f / 3.1416f, betaRate * 180.0f / 3.1416f));

			if ((dAbs(beta) > m_maxSideSlipAngle)) {
				dVector xxx(matrix.m_up.Scale(-8000.0f * dSign(beta)));
				NewtonBodyAddTorque(chassisBody, &xxx[0]);
			} else if (dAbs(betaRate) > m_maxSideSlipAngleRate) {
				dVector xxx(matrix.m_up.Scale(-8000.0f * dSign(betaRate)));
				NewtonBodyAddTorque(chassisBody, &xxx[0]);
			}
		}
	}

	dCustomVehicleController* m_vehicle;
	
	dFloat m_sideSlipAngle;
	dFloat m_maxSideSlipAngle;
	dFloat m_maxSideSlipAngleRate;
};
*/

void dCustomVehicleController::dBodyPartChassis::ApplyDownForce ()
{
	// add aerodynamics forces
	dMatrix matrix;
	dVector veloc(0.0f);

	NewtonBody* const body = GetBody();
	NewtonBodyGetVelocity(body, &veloc[0]);
	NewtonBodyGetMatrix(body, &matrix[0][0]);

	veloc -= matrix.m_up.Scale (veloc.DotProduct3(matrix.m_up));
	dFloat downForceMag = m_aerodynamicsDownForceCoefficient * veloc.DotProduct3(veloc);
	if (downForceMag > m_aerodynamicsDownForce0) {
		dFloat speed = dSqrt (veloc.DotProduct3(veloc));
		dFloat topSpeed = GetController()->GetEngine() ? GetController()->GetEngine()->GetTopGear() : 30.0f;
		dFloat speedRatio = (speed - m_aerodynamicsDownSpeedCutOff) / (topSpeed - speed); 
		downForceMag = m_aerodynamicsDownForce0 + (m_aerodynamicsDownForce1 - m_aerodynamicsDownForce0) * speedRatio; 
	}

	dVector downforce(matrix.m_up.Scale (-downForceMag));
	NewtonBodyAddForce(body, &downforce[0]);
}


dCustomVehicleController::dBodyPartTire::dBodyPartTire()
	:dBodyPart()
	,m_lateralSlip(0.0f)
	,m_longitudinalSlip(0.0f)
	,m_aligningTorque(0.0f)
	,m_index(0)
	,m_collidingCount(0)
{
}

dCustomVehicleController::dBodyPartTire::~dBodyPartTire()
{
}

/*
// Using brush tire model explained by Giancarlo Genta in his book
void dCustomVehicleController::dBodyPartTire::dFrictionModel::CalculateTireFrictionCoeficents(const dBodyPartTire* const tire, const NewtonBody* const otherBody, const NewtonMaterial* const material, dFloat tireLoad, dFloat& longitudinalForce, dFloat& lateralForce, dFloat& aligningTorque) const
{
	const dCustomVehicleController* const controller = tire->GetController();
//	const dFloat gravityMag = controller->m_gravityMag;
	const dFloat totalMass = controller->m_totalMass;
	dFloat phy_y = tire->m_lateralSlip * tire->m_data.m_lateralStiffness * totalMass;
	dFloat phy_x = tire->m_longitudinalSlip * tire->m_data.m_longitudialStiffness * totalMass;
	dFloat gamma = dMax(dSqrt(phy_x * phy_x + phy_y * phy_y), dFloat(0.1f));

	dFloat fritionCoeficicent = dClamp(GetFrictionCoefficient(material, tire->GetBody(), otherBody), dFloat(0.0f), dFloat(1.0f));
	tireLoad *= fritionCoeficicent;
	dFloat phyMax = 3.0f * tireLoad + 1.0f;

	dFloat F = (gamma <= phyMax) ? (gamma * (1.0f - gamma / phyMax + gamma * gamma / (3.0f * phyMax * phyMax))) : tireLoad;

	dFloat fraction = F / gamma;
	lateralForce = -phy_y * fraction;
	longitudinalForce = -phy_x * fraction;

	aligningTorque = 0.0f;
}
*/

// Using brush tire model explained by Giancarlo Genta in his book, adapted to calculate friction coefficient instead tire forces
void dCustomVehicleController::dBodyPartTire::dFrictionModel::CalculateTireFrictionCoefficents(
	const dBodyPartTire* const tire, const NewtonBody* const otherBody, const NewtonMaterial* const material, 
	dFloat longitudinalSlip, dFloat lateralSlip, dFloat longitudinalStiffness, dFloat lateralStiffness,
	dFloat& longitudinalFrictionCoef, dFloat& lateralFrictionCoef, dFloat& aligningTorqueCoef) const
{
	dAssert (lateralSlip >= 0.0f);
//	dAssert (longitudinalSlip >= 0.0f);
	dAssert (lateralStiffness >= 0.0f);
	dAssert (longitudinalStiffness >= 0.0f);
	dFloat den = 1.0f / (1.0f + longitudinalSlip);

	lateralSlip *= den;
	longitudinalSlip *= den;

	dFloat phy_y = dAbs (lateralStiffness * lateralSlip * 4.0f);
	dFloat phy_x = dAbs (longitudinalStiffness * longitudinalSlip * 4.0f);

	dFloat gamma = dMax(dSqrt(phy_x * phy_x + phy_y * phy_y), dFloat(0.1f));
	dFloat fritionCoeficicent = dClamp(GetFrictionCoefficient(material, tire->GetBody(), otherBody), dFloat(0.0f), dFloat(1.0f));

	dFloat normalTireLoad = 1.0f * fritionCoeficicent;
	dFloat phyMax = 3.0f * normalTireLoad + 1.0e-3f;
	dFloat F = (gamma <= phyMax) ? (gamma * (1.0f - gamma / phyMax + gamma * gamma / (3.0f * phyMax * phyMax))) : normalTireLoad;

	dFloat fraction = F / gamma;
	dAssert (fraction > 0.0f);
	lateralFrictionCoef = phy_y * fraction;
	longitudinalFrictionCoef = phy_x * fraction;

	dAssert (lateralFrictionCoef >= 0.0f);
	dAssert (lateralFrictionCoef <= 1.1f);
	dAssert (longitudinalFrictionCoef >= 0.0f);
	dAssert (longitudinalFrictionCoef <= 1.1f);

	aligningTorqueCoef = 0.0f;
}

void dCustomVehicleController::dBodyPartTire::Init (dBodyPart* const parentPart, const dMatrix& locationInGlobalSpase, const Info& info)
{
	m_data = info;
	m_parent = parentPart;
	m_userData = info.m_userData;
	m_controller = parentPart->m_controller;

	m_collidingCount = 0;
	m_lateralSlip = 0.0f;
	m_aligningTorque = 0.0f;
	m_longitudinalSlip = 0.0f;

	dCustomVehicleControllerManager* const manager = (dCustomVehicleControllerManager*)m_controller->GetManager();

	NewtonWorld* const world = ((dCustomVehicleControllerManager*)m_controller->GetManager())->GetWorld();
	NewtonCollisionSetScale(manager->m_tireShapeTemplate, m_data.m_width, m_data.m_radio, m_data.m_radio);

	// create the rigid body that will make this bone
	dMatrix matrix (dYawMatrix(-0.5f * 3.1415927f) * locationInGlobalSpase);
	m_body = NewtonCreateDynamicBody(world, manager->m_tireShapeTemplate, &matrix[0][0]);
	NewtonCollision* const collision = NewtonBodyGetCollision(m_body);
	NewtonCollisionSetUserData1 (collision, this);
	
	NewtonBodySetMaterialGroupID(m_body, manager->GetTireMaterial());

	dVector drag(0.0f);
	NewtonBodySetLinearDamping(m_body, 0);
	NewtonBodySetAngularDamping(m_body, &drag[0]);
	NewtonBodySetMaxRotationPerStep(m_body, 3.141692f);
	
	// set the standard force and torque call back
	NewtonBodySetForceAndTorqueCallback(m_body, m_controller->m_forceAndTorqueCallback);

	// tire are highly non linear, sung spherical inertia matrix make the calculation more accurate 
	dFloat inertia = 2.0f * m_data.m_mass * m_data.m_radio * m_data.m_radio / 5.0f;
	NewtonBodySetMassMatrix (m_body, m_data.m_mass, inertia, inertia, inertia);

	matrix.m_posit += matrix.m_front.Scale(m_data.m_pivotOffset);
	m_joint = new dWheelJoint (matrix, m_body, parentPart->m_body, this);
	
	dMatrix chassisMatrix;
	NewtonBodyGetMatrix(parentPart->m_body, &chassisMatrix[0][0]);
	m_data.m_location = chassisMatrix.UntransformVector(matrix.m_posit);
}

dFloat dCustomVehicleController::dBodyPartTire::GetRPM() const
{
	dVector omega(0.0f); 
	dWheelJoint* const joint = (dWheelJoint*) m_joint;
	NewtonBodyGetOmega(m_body, &omega[0]);
	return joint->m_lateralDir.DotProduct3(omega) * 9.55f;
}


void dCustomVehicleController::dBodyPartTire::SetSteerAngle (dFloat angleParam, dFloat timestep)
{
	dWheelJoint* const tire = (dWheelJoint*)m_joint;
	tire->m_steerAngle1 = -angleParam * m_data.m_maxSteeringAngle;

	if (tire->m_steerAngle0 < tire->m_steerAngle1) {
		tire->m_steerAngle0 += tire->m_steerRate * timestep;
		if (tire->m_steerAngle0 > tire->m_steerAngle1) {
			tire->m_steerAngle0 = tire->m_steerAngle1;
		}
	} else if (tire->m_steerAngle0 > tire->m_steerAngle1) {
		tire->m_steerAngle0 -= tire->m_steerRate * timestep;
		if (tire->m_steerAngle0 < tire->m_steerAngle1) {
			tire->m_steerAngle0 = tire->m_steerAngle1;
		}
	}
}

void dCustomVehicleController::dBodyPartTire::SetBrakeTorque(dFloat torque)
{
	dWheelJoint* const tire = (dWheelJoint*)m_joint;
	tire->m_brakeTorque = dMax (torque, tire->m_brakeTorque);
}

void dCustomVehicleController::dBodyPartTire::ProjectError()
{
	dWheelJoint* const tire = (dWheelJoint*)m_joint;
	tire->ProjectError();
}

dFloat dCustomVehicleController::dBodyPartTire::GetLateralSlip () const
{
	return m_lateralSlip;
}

dFloat dCustomVehicleController::dBodyPartTire::GetLongitudinalSlip () const
{
	return m_longitudinalSlip;
}


dCustomVehicleController::dBodyPartEngine::dBodyPartEngine(dCustomVehicleController* const controller, dFloat mass, dFloat armatureRadius)
	:dBodyPart()
{
	dMatrix matrix;
	dVector origin;

	m_parent = &controller->m_chassis;
	m_controller = controller;

	NewtonBody* const chassisBody = m_controller->GetBody();
	NewtonWorld* const world = ((dCustomVehicleControllerManager*)m_controller->GetManager())->GetWorld();

	// get engine location (place at the body center of mass)
	NewtonBodyGetCentreOfMass(chassisBody, &origin[0]);
	NewtonBodyGetMatrix(chassisBody, &matrix[0][0]);

//origin.m_y += 2.0f;
	matrix.m_posit = matrix.TransformVector(origin);

	NewtonCollision* const collision = NewtonCreateSphere(world, 0.1f, 0, NULL);
//NewtonCollision* const collision = NewtonCreateCylinder(world, 0.1f, 0.1f, 0.5f, 0, NULL);
	NewtonCollisionSetMode(collision, 0);
	m_body = NewtonCreateDynamicBody(world, collision, &matrix[0][0]);
	NewtonDestroyCollision(collision);

	// make engine inertia spherical (also make scale inertia but twice the radio for more stability)
	const dFloat engineInertiaScale = 9.0f;
	const dFloat inertia = 2.0f * engineInertiaScale * mass * armatureRadius * armatureRadius / 5.0f;
	NewtonBodySetMassMatrix(m_body, mass, inertia, inertia, inertia);

	dVector drag(0.0f);
	NewtonBodySetLinearDamping(m_body, 0);
	NewtonBodySetAngularDamping(m_body, &drag[0]);
	NewtonBodySetMaxRotationPerStep(m_body, 3.141692f * 2.0f);
	NewtonBodySetForceAndTorqueCallback(m_body, m_controller->m_forceAndTorqueCallback);

	dMatrix pinMatrix (matrix);
	pinMatrix.m_front = matrix.m_right;
	pinMatrix.m_up = matrix.m_up;
	pinMatrix.m_right = pinMatrix.m_front.CrossProduct(pinMatrix.m_up);
	m_joint = new dEngineJoint(pinMatrix, m_body, chassisBody);
}

dCustomVehicleController::dBodyPartEngine::~dBodyPartEngine()
{
}

void dCustomVehicleController::dBodyPartEngine::ProjectError()
{
	dEngineJoint* const joint = (dEngineJoint*)m_joint;
	joint->ProjectError();
}

void dCustomVehicleController::dBodyPartEngine::ApplyTorque(dFloat torqueMag)
{
	dMatrix matrix;
	NewtonBody* const chassis = m_controller->GetBody();
	NewtonBody* const engine = m_controller->m_engine->GetBody();
	NewtonBodyGetMatrix(chassis, &matrix[0][0]);
	dVector torque(matrix.m_right.Scale(torqueMag));
	NewtonBodyAddTorque(engine, &torque[0]);
}


dCustomVehicleController::dBodyPartDifferential::dBodyPartDifferential()
	:dBodyPart()
{
}

void dCustomVehicleController::dBodyPartDifferential::Init(dCustomVehicleController* const controller, dFloat mass, dFloat inertia)
{
	dMatrix matrix;
	dVector origin;

	m_parent = &controller->m_chassis;
	m_controller = controller;

	NewtonBody* const chassisBody = m_controller->GetBody();
	NewtonWorld* const world = ((dCustomVehicleControllerManager*)m_controller->GetManager())->GetWorld();

	// get engine location (place at the body center of mass)
	NewtonBodyGetCentreOfMass(chassisBody, &origin[0]);
	NewtonBodyGetMatrix(chassisBody, &matrix[0][0]);
/*
static int xxx;
origin.m_y += 1.0f;
if (xxx == 0)
origin.m_x -= 1.0f;
else if (xxx == 1)
origin.m_x += 1.0f;
xxx++;
*/
	matrix.m_posit = matrix.TransformVector(origin);

	NewtonCollision* const collision = NewtonCreateSphere(world, 0.1f, 0, NULL);
//NewtonCollision* const collision = NewtonCreateCylinder(world, 0.25f, 0.25f, 0.5f, 0, NULL);
	NewtonCollisionSetMode(collision, 0);
	m_body = NewtonCreateDynamicBody(world, collision, &matrix[0][0]);
	NewtonDestroyCollision(collision);

	NewtonBodySetMassMatrix(m_body, mass, inertia, inertia, inertia);

	dVector drag(0.0f);
	NewtonBodySetLinearDamping(m_body, 0);
	NewtonBodySetAngularDamping(m_body, &drag[0]);
	NewtonBodySetMaxRotationPerStep(m_body, 3.141692f * 2.0f);
	NewtonBodySetForceAndTorqueCallback(m_body, m_controller->m_forceAndTorqueCallback);

	dMatrix pinMatrix(matrix);
	pinMatrix.m_front = matrix.m_front;
	pinMatrix.m_up = matrix.m_right;
	pinMatrix.m_right = pinMatrix.m_front.CrossProduct(pinMatrix.m_up);
	m_joint = new dDifferentialJoint(pinMatrix, m_body, chassisBody);
}

dCustomVehicleController::dBodyPartDifferential::~dBodyPartDifferential()
{
}

void dCustomVehicleController::dBodyPartDifferential::ProjectError()
{
	dDifferentialJoint* const joint = (dDifferentialJoint*)m_joint;
	joint->ProjectError();
}


void dCustomVehicleController::dEngineController::dInfo::ConvertToMetricSystem()
{
	const dFloat horsePowerToWatts = 735.5f;
	const dFloat kmhToMetersPerSecunds = 0.278f;
	const dFloat rpmToRadiansPerSecunds = 0.105f;
	const dFloat poundFootToNewtonMeters = 1.356f;

	m_idleTorque *= poundFootToNewtonMeters;
	m_peakTorque *= poundFootToNewtonMeters;

	m_rpmAtPeakTorque *= rpmToRadiansPerSecunds;
	m_rpmAtPeakHorsePower *= rpmToRadiansPerSecunds;
	m_rpmAtRedLine *= rpmToRadiansPerSecunds;
	m_rpmAtIdleTorque *= rpmToRadiansPerSecunds;

	m_peakHorsePower *= horsePowerToWatts;
	m_vehicleTopSpeed *= kmhToMetersPerSecunds;

	m_peakPowerTorque = m_peakHorsePower / m_rpmAtPeakHorsePower;

	dAssert(m_rpmAtIdleTorque > 0.0f);
	dAssert(m_rpmAtIdleTorque < m_rpmAtPeakHorsePower);
	dAssert(m_rpmAtPeakTorque < m_rpmAtPeakHorsePower);
	dAssert(m_rpmAtPeakHorsePower < m_rpmAtRedLine);

	dAssert(m_idleTorque > 0.0f);
	dAssert(m_peakTorque > m_peakPowerTorque);
	dAssert((m_peakTorque * m_rpmAtPeakTorque) < m_peakHorsePower);
}

dCustomVehicleController::dEngineController::dEngineController(dCustomVehicleController* const controller, const dInfo& info, dBodyPartDifferential* const differential, dBodyPartTire* const crownGearCalculator)
	:dController(controller)
	,m_info(info)
	,m_infoCopy(info)
	,m_controller(controller)
	,m_crownGearCalculator(crownGearCalculator)
	,m_differential(differential)
	,m_gearBoxJoint(NULL)
	,m_clutchParam(1.0f)
	,m_gearTimer(0)
	,m_currentGear(D_VEHICLE_NEUTRAL_GEAR)
	,m_ignitionKey(false)
	,m_automaticTransmissionMode(true)
{
	dMatrix chassisMatrix;
	dMatrix differentialMatrix;
	NewtonBody* const chassisBody = controller->m_chassis.GetBody();
	NewtonBodyGetMatrix (chassisBody, &chassisMatrix[0][0]);
//	chassisMatrix = controller->m_localFrame * chassisMatrix;
	
	NewtonBody* const engineBody = controller->m_engine->GetBody();
	dAssert (engineBody == controller->m_engine->GetJoint()->GetBody0());

	NewtonBody* const differentialBody = m_differential->GetBody();
	NewtonBodyGetMatrix (engineBody, &differentialMatrix[0][0]);
	m_gearBoxJoint = new dGearBoxJoint(chassisMatrix.m_right, differentialBody, engineBody, m_info.m_clutchFrictionTorque);
	SetInfo(info);
}

dCustomVehicleController::dEngineController::~dEngineController()
{
}

dCustomVehicleController::dEngineController::dInfo dCustomVehicleController::dEngineController::GetInfo() const
{
	return m_infoCopy;
}

void dCustomVehicleController::dEngineController::SetInfo(const dInfo& info)
{
	m_info = info;
	m_infoCopy = info;
	m_info.m_clutchFrictionTorque = dMax (dFloat(10.0f), dAbs (m_info.m_clutchFrictionTorque));
	m_infoCopy.m_clutchFrictionTorque = m_info.m_clutchFrictionTorque;
	InitEngineTorqueCurve();

	dAssert(info.m_gearsCount < (int(sizeof (m_info.m_gearRatios) / sizeof (m_info.m_gearRatios[0])) - D_VEHICLE_FIRST_GEAR));
	m_info.m_gearsCount = info.m_gearsCount + D_VEHICLE_FIRST_GEAR;

	m_info.m_gearRatios[D_VEHICLE_NEUTRAL_GEAR] = 0.0f;
	m_info.m_gearRatios[D_VEHICLE_REVERSE_GEAR] = -dAbs(info.m_reverseGearRatio);
	for (int i = 0; i < (m_info.m_gearsCount - D_VEHICLE_FIRST_GEAR); i++) {
		m_info.m_gearRatios[i + D_VEHICLE_FIRST_GEAR] = dAbs(info.m_gearRatios[i]);
	}

	for (dList<dBodyPartTire>::dListNode* tireNode = m_controller->m_tireList.GetFirst(); tireNode; tireNode = tireNode->GetNext()) {
		dBodyPartTire& tire = tireNode->GetInfo();
		dFloat angle = (1.0f / 30.0f) * (0.277778f) * info.m_vehicleTopSpeed / tire.m_data.m_radio;
		NewtonBodySetMaxRotationPerStep(tire.GetBody(), angle);
	}
	m_controller->SetAerodynamicsDownforceCoefficient (info.m_aerodynamicDownforceFactor, info.m_aerodynamicDownForceSurfaceCoeficident, info.m_aerodynamicDownforceFactorAtTopSpeed);
}

bool dCustomVehicleController::dEngineController::GetDifferentialLock() const
{
	return m_info.m_differentialLock ? true : false;
}

void dCustomVehicleController::dEngineController::SetDifferentialLock(bool lock)
{
	m_info.m_differentialLock = lock ? 1 : 0;
}

dFloat dCustomVehicleController::dEngineController::GetTopGear() const
{
	return m_info.m_gearRatios[m_info.m_gearsCount - 1];
}

void dCustomVehicleController::dEngineController::CalculateCrownGear()
{
	dAssert(m_info.m_vehicleTopSpeed >= 0.0f);
	dAssert(m_info.m_vehicleTopSpeed < 100.0f);

	m_info.m_crownGearRatio = 1.0f;
	dBodyPartTire* const tire = m_crownGearCalculator;
	dAssert(tire);

	// drive train geometrical relations
	// G0 = m_differentialGearRatio
	// G1 = m_transmissionGearRatio
	// s = topSpeedMPS
	// r = tireRadio
	// wt = rpsAtTire
	// we = rpsAtPickPower
	// we = G1 * G0 * wt;
	// wt = e / r
	// we = G0 * G1 * s / r
	// G0 = r * we / (G1 * s)
	// using the top gear and the optimal engine torque for the calculations
	dFloat topGearRatio = GetTopGear();
	dFloat tireRadio = tire->GetInfo().m_radio;
	m_info.m_crownGearRatio = tireRadio * m_info.m_rpmAtPeakHorsePower / (m_info.m_vehicleTopSpeed * topGearRatio);

	// bake crown gear with the engine pwer curve
	m_info.m_idleFriction *= m_info.m_crownGearRatio;
	m_info.m_idleTorque *= m_info.m_crownGearRatio;
	m_info.m_peakPowerTorque *= m_info.m_crownGearRatio;
	m_info.m_peakTorque *= m_info.m_crownGearRatio;
	m_info.m_rpmAtIdleTorque /= m_info.m_crownGearRatio;
	m_info.m_rpmAtPeakTorque /= m_info.m_crownGearRatio;
	m_info.m_rpmAtPeakHorsePower /= m_info.m_crownGearRatio;
	m_info.m_rpmAtRedLine /= m_info.m_crownGearRatio;
}


void dCustomVehicleController::dEngineController::InitEngineTorqueCurve()
{
	m_info.ConvertToMetricSystem();

	CalculateCrownGear();

	m_info.m_idleFriction = m_info.m_idleTorque * D_VEHICLE_ENGINE_IDLE_FRICTION_COEFFICIENT;

	dFloat rpmStep = m_info.m_rpmAtIdleTorque;
	dFloat torqueLose = m_info.m_idleTorque + (m_info.m_peakTorque - m_info.m_idleTorque) * rpmStep / m_info.m_rpmAtPeakTorque - m_info.m_idleFriction;
	m_info.m_viscousDrag0 = torqueLose / (rpmStep * rpmStep);

	torqueLose = m_info.m_peakTorque - m_info.m_peakPowerTorque - m_info.m_idleFriction;
	rpmStep = m_info.m_rpmAtPeakHorsePower - m_info.m_rpmAtPeakTorque;
	m_info.m_viscousDrag1 = torqueLose / (rpmStep * rpmStep);

	torqueLose = m_info.m_peakPowerTorque - m_info.m_idleFriction;
	rpmStep = m_info.m_rpmAtRedLine - m_info.m_rpmAtPeakHorsePower;
	m_info.m_viscousDrag2 = torqueLose / (rpmStep * rpmStep);
}

void dCustomVehicleController::dEngineController::PlotEngineCurve() const
{
#ifdef D_PLOT_ENGINE_CURVE 
	dFloat rpm0 = m_torqueRPMCurve.m_nodes[0].m_param;
	dFloat rpm1 = m_torqueRPMCurve.m_nodes[m_torqueRPMCurve.m_count - 1].m_param;
	int steps = 40;
	dFloat omegaStep = (rpm1 - rpm0) / steps;
	dTrace(("rpm\ttorque\tpower\n"));
	for (int i = 0; i < steps; i++) {
		dFloat r = rpm0 + omegaStep * i;
		dFloat torque = m_torqueRPMCurve.GetValue(r);
		dFloat power = r * torque;
		const dFloat horsePowerToWatts = 735.5f;
		const dFloat rpmToRadiansPerSecunds = 0.105f;
		const dFloat poundFootToNewtonMeters = 1.356f;
		dTrace(("%6.2f\t%6.2f\t%6.2f\n", r / 0.105f, torque / 1.356f, power / 735.5f));
	}
#endif
}


dFloat dCustomVehicleController::dEngineController::GetGearRatio () const
{
	return m_info.m_crownGearRatio * m_info.m_gearRatios[m_currentGear];
}


void dCustomVehicleController::dEngineController::UpdateAutomaticGearBox(dFloat timestep, dFloat omega)
{
m_info.m_gearsCount = 4;

	m_gearTimer--;
	if (m_gearTimer < 0) {
		switch (m_currentGear) 
		{
			case D_VEHICLE_NEUTRAL_GEAR:
			{
			   SetGear(D_VEHICLE_NEUTRAL_GEAR);
			   break;
			}

			case D_VEHICLE_REVERSE_GEAR:
			{
				SetGear(D_VEHICLE_REVERSE_GEAR);
				break;
			}

			default:
			{
			   if (omega > m_info.m_rpmAtPeakHorsePower) {
					if (m_currentGear < (m_info.m_gearsCount - 1)) {
						SetGear(m_currentGear + 1);
					}
				} else if (omega < m_info.m_rpmAtPeakTorque) {
					if (m_currentGear > D_VEHICLE_FIRST_GEAR) {
						SetGear(m_currentGear - 1);
					}
				}
			}
		}
	}
}

void dCustomVehicleController::dEngineController::ApplyTorque(dFloat torque)
{
	m_controller->m_engine->ApplyTorque(torque);
}

void dCustomVehicleController::dEngineController::Update(dFloat timestep)
{
	dFloat omega = GetRadiansPerSecond();
	if (m_automaticTransmissionMode) {
		UpdateAutomaticGearBox (timestep, omega);
	}

	dFloat engineTorque = 0.0f;
	if (m_ignitionKey) {
		engineTorque = m_info.m_idleTorque + (m_info.m_peakTorque - m_info.m_idleTorque) * m_param;
		dFloat rpmIdleStep = dMin(omega, m_info.m_rpmAtIdleTorque);
		engineTorque -= m_info.m_viscousDrag0 * rpmIdleStep * rpmIdleStep;
		if (omega > m_info.m_rpmAtPeakTorque) {
			dFloat rpmStep1 = dMin(omega, m_info.m_rpmAtPeakHorsePower) - m_info.m_rpmAtPeakTorque;
			engineTorque -= m_info.m_viscousDrag1 * rpmStep1 * rpmStep1;
			if (omega > m_info.m_rpmAtPeakHorsePower) {
				dFloat rpmStep2 = omega - m_info.m_rpmAtPeakHorsePower;
				engineTorque = dMax (engineTorque - m_info.m_viscousDrag2 * rpmStep2 * rpmStep2, dFloat(0.0f));
			}
		}
	}

	dEngineJoint* const engineJoint = (dEngineJoint*) m_controller->m_engine->m_joint;

	engineJoint->SetRedLineRPM(m_info.m_rpmAtRedLine);
	engineJoint->SetDryFriction(m_info.m_idleFriction);
	ApplyTorque(engineTorque);
}

bool dCustomVehicleController::dEngineController::GetTransmissionMode() const
{
	return m_automaticTransmissionMode;
}

void dCustomVehicleController::dEngineController::SetIgnition(bool key)
{
	m_ignitionKey = key;
}

bool dCustomVehicleController::dEngineController::GetIgnition() const
{
	return m_ignitionKey;
}


void dCustomVehicleController::dEngineController::SetTransmissionMode(bool mode)
{
	m_automaticTransmissionMode = mode;
}

void dCustomVehicleController::dEngineController::SetClutchParam (dFloat cluthParam)
{
	m_clutchParam = dClamp (cluthParam, dFloat(0.0f), dFloat(1.0f));
	if (m_gearBoxJoint) {
		m_gearBoxJoint->SetFritionTorque(m_clutchParam);
	}
}


int dCustomVehicleController::dEngineController::GetGear() const
{
	return m_currentGear;
}

void dCustomVehicleController::dEngineController::SetGear(int gear)
{
	m_gearTimer = 30;
	m_currentGear = dClamp(gear, 0, m_info.m_gearsCount);
	m_gearBoxJoint->SetGearRatio(m_info.m_gearRatios[m_currentGear] * m_info.m_gearRatiosSign);
}

int dCustomVehicleController::dEngineController::GetNeutralGear() const
{
	return D_VEHICLE_NEUTRAL_GEAR;
}

int dCustomVehicleController::dEngineController::GetReverseGear() const
{
	return D_VEHICLE_REVERSE_GEAR;
}

int dCustomVehicleController::dEngineController::GetFirstGear() const
{
	return D_VEHICLE_FIRST_GEAR;
}

int dCustomVehicleController::dEngineController::GetLastGear() const
{
	return m_info.m_gearsCount - 1;
}

dFloat dCustomVehicleController::dEngineController::GetRadiansPerSecond() const
{
	dMatrix matrix;
	dVector omega(0.0f);

	NewtonBody* const chassis = m_controller->GetBody();
	NewtonBody* const engine = m_controller->m_engine->GetBody();

	NewtonBodyGetMatrix(chassis, &matrix[0][0]);
	NewtonBodyGetOmega(engine, &omega[0]);
	return omega.DotProduct3(matrix.m_right);
}

dFloat dCustomVehicleController::dEngineController::GetRPM() const
{
	return m_info.m_crownGearRatio * GetRadiansPerSecond() * 9.55f;
}

dFloat dCustomVehicleController::dEngineController::GetIdleRPM() const
{
	return m_info.m_crownGearRatio * m_info.m_rpmAtIdleTorque * 9.55f;
}

dFloat dCustomVehicleController::dEngineController::GetRedLineRPM() const
{
	return m_info.m_crownGearRatio * m_info.m_rpmAtRedLine * 9.55f;
}

dFloat dCustomVehicleController::dEngineController::GetSpeed() const
{
	dMatrix matrix;
	dVector veloc(0.0f);
	NewtonBody* const chassis = m_controller->GetBody();

	NewtonBodyGetMatrix(chassis, &matrix[0][0]);
	NewtonBodyGetVelocity(chassis, &veloc[0]);

//	dVector pin (matrix.RotateVector (m_controller->m_localFrame.m_front));
	dVector pin (matrix.m_front);
	return pin.DotProduct3(veloc);
}

dFloat dCustomVehicleController::dEngineController::GetTopSpeed() const
{
	return m_info.m_vehicleTopSpeed;
}

dCustomVehicleController::dSteeringController::dSteeringController (dCustomVehicleController* const controller)
	:dController(controller)
	,m_isSleeping(false)
{
}


void dCustomVehicleController::dSteeringController::Update(dFloat timestep)
{
	m_isSleeping = true;
	for (dList<dBodyPartTire*>::dListNode* node = m_tires.GetFirst(); node; node = node->GetNext()) {
		dBodyPartTire& tire = *node->GetInfo();
		dWheelJoint* const joint = (dWheelJoint*)tire.m_joint;
		tire.SetSteerAngle(m_param, timestep);
		m_isSleeping &= dAbs (joint->m_steerAngle1 - joint->m_steerAngle0) < 1.0e-4f;
	}
}

void dCustomVehicleController::dSteeringController::AddTire (dCustomVehicleController::dBodyPartTire* const tire)
{
	m_tires.Append(tire);
}

dCustomVehicleController::dBrakeController::dBrakeController(dCustomVehicleController* const controller, dFloat maxBrakeTorque)
	:dController(controller)
	,m_maxTorque(maxBrakeTorque)
{
}

void dCustomVehicleController::dBrakeController::AddTire(dBodyPartTire* const tire)
{
	m_tires.Append(tire);
}

void dCustomVehicleController::dBrakeController::Update(dFloat timestep)
{
	dFloat torque = m_maxTorque * m_param;
   	for (dList<dBodyPartTire*>::dListNode* node = m_tires.GetFirst(); node; node = node->GetNext()) {
		dBodyPartTire& tire = *node->GetInfo();
		tire.SetBrakeTorque (torque);
	}
}



void dCustomVehicleControllerManager::DrawSchematic (const dCustomVehicleController* const controller, dFloat scale) const
{
	controller->DrawSchematic(scale);
}

void dCustomVehicleControllerManager::DrawSchematicCallback (const dCustomVehicleController* const controller, const char* const partName, dFloat value, int pointCount, const dVector* const lines) const
{
}


void dCustomVehicleController::DrawSchematic(dFloat scale) const
{
	dMatrix projectionMatrix(dGetIdentityMatrix());
	projectionMatrix[0][0] = scale;
	projectionMatrix[1][1] = 0.0f;
	projectionMatrix[2][1] = -scale;
	projectionMatrix[2][2] = 0.0f;
	dCustomVehicleControllerManager* const manager = (dCustomVehicleControllerManager*)GetManager();

	dMatrix matrix0;
	dVector com(0.0f);
	dFloat arrayPtr[32][4];
	dVector* const array = (dVector*)arrayPtr;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	NewtonBody* const chassisBody = m_chassis.GetBody();

	dFloat velocityScale = 0.125f;

	NewtonBodyGetCentreOfMass(chassisBody, &com[0]);
	NewtonBodyGetMatrix(chassisBody, &matrix0[0][0]);
	matrix0.m_posit = matrix0.TransformVector(com);

	NewtonBodyGetMass(chassisBody, &mass, &Ixx, &Iyy, &Izz);
//	dMatrix chassisMatrix(GetLocalFrame() * matrix0);
	dMatrix chassisMatrix(matrix0);
	dMatrix worldToComMatrix(chassisMatrix.Inverse() * projectionMatrix);
	{
		// draw vehicle chassis
		dVector p0(D_CUSTOM_LARGE_VALUE, D_CUSTOM_LARGE_VALUE, D_CUSTOM_LARGE_VALUE, 0.0f);
		dVector p1(-D_CUSTOM_LARGE_VALUE, -D_CUSTOM_LARGE_VALUE, -D_CUSTOM_LARGE_VALUE, 0.0f);

		for (dList<dBodyPartTire>::dListNode* node = m_tireList.GetFirst(); node; node = node->GetNext()) {
			dMatrix matrix;
			dBodyPartTire* const tire = &node->GetInfo();
			NewtonBody* const tireBody = tire->GetBody();
			
			NewtonBodyGetMatrix(tireBody, &matrix[0][0]);
			//dMatrix matrix (tire->CalculateSteeringMatrix() * m_chassisState.GetMatrix());
			dVector p(worldToComMatrix.TransformVector(matrix.m_posit));
			p0 = dVector(dMin(p.m_x, p0.m_x), dMin(p.m_y, p0.m_y), dMin(p.m_z, p0.m_z), 1.0f);
			p1 = dVector(dMax(p.m_x, p1.m_x), dMax(p.m_y, p1.m_y), dMax(p.m_z, p1.m_z), 1.0f);
		}

		array[0] = dVector(p0.m_x, p0.m_y, p0.m_z, 1.0f);
		array[1] = dVector(p1.m_x, p0.m_y, p0.m_z, 1.0f);
		array[2] = dVector(p1.m_x, p1.m_y, p0.m_z, 1.0f);
		array[3] = dVector(p0.m_x, p1.m_y, p0.m_z, 1.0f);
		manager->DrawSchematicCallback(this, "chassis", 0, 4, array);
	}

	{
		// draw vehicle tires
		for (dList<dBodyPartTire>::dListNode* node = m_tireList.GetFirst(); node; node = node->GetNext()) {
			dMatrix matrix;
			dBodyPartTire* const tire = &node->GetInfo();
			dFloat width = tire->m_data.m_width * 0.5f;
			dFloat radio = tire->m_data.m_radio;
			NewtonBody* const tireBody = tire->GetBody();
			NewtonBodyGetMatrix(tireBody, &matrix[0][0]);
			matrix.m_up = chassisMatrix.m_up;
			matrix.m_right = matrix.m_front.CrossProduct(matrix.m_up);

			array[0] = worldToComMatrix.TransformVector(matrix.TransformVector(dVector(width, 0.0f, radio, 0.0f)));
			array[1] = worldToComMatrix.TransformVector(matrix.TransformVector(dVector(width, 0.0f, -radio, 0.0f)));
			array[2] = worldToComMatrix.TransformVector(matrix.TransformVector(dVector(-width, 0.0f, -radio, 0.0f)));
			array[3] = worldToComMatrix.TransformVector(matrix.TransformVector(dVector(-width, 0.0f, radio, 0.0f)));
			manager->DrawSchematicCallback(this, "tire", 0, 4, array);
		}
	}

	{
		// draw vehicle velocity
		dVector veloc(0.0f);
		dVector omega(0.0f);

		NewtonBodyGetOmega(chassisBody, &omega[0]);
		NewtonBodyGetVelocity(chassisBody, &veloc[0]);

		dVector localVelocity(chassisMatrix.UnrotateVector(veloc));
		localVelocity.m_y = 0.0f;

		localVelocity = projectionMatrix.RotateVector(localVelocity);
		array[0] = dVector(0.0f);
		array[1] = localVelocity.Scale(velocityScale);
		manager->DrawSchematicCallback(this, "velocity", 0, 2, array);

		dVector localOmega(chassisMatrix.UnrotateVector(omega));
		array[0] = dVector(0.0f);
		array[1] = dVector(0.0f, localOmega.m_y * 10.0f, 0.0f, 0.0f);
		manager->DrawSchematicCallback(this, "omega", 0, 2, array);
	}

	{
		dFloat scale1(2.0f / (mass * 10.0f));
		// draw vehicle forces
		for (dList<dBodyPartTire>::dListNode* node = m_tireList.GetFirst(); node; node = node->GetNext()) {
			dBodyPartTire* const tire = &node->GetInfo();
			dMatrix matrix;
			NewtonBody* const tireBody = tire->GetBody();
			NewtonBodyGetMatrix(tireBody, &matrix[0][0]);
			matrix.m_up = chassisMatrix.m_up;
			matrix.m_right = matrix.m_front.CrossProduct(matrix.m_up);

			dVector origin(worldToComMatrix.TransformVector(matrix.m_posit));

			dVector lateralForce(chassisMatrix.UnrotateVector(GetTireLateralForce(tire)));
			lateralForce = lateralForce.Scale(-scale1);
			lateralForce = projectionMatrix.RotateVector(lateralForce);

			array[0] = origin;
			array[1] = origin + lateralForce;
			manager->DrawSchematicCallback(this, "lateralForce", 0, 2, array);

			dVector longitudinalForce(chassisMatrix.UnrotateVector(GetTireLongitudinalForce(tire)));
			longitudinalForce = longitudinalForce.Scale(-scale1);
			longitudinalForce = projectionMatrix.RotateVector(longitudinalForce);

			array[0] = origin;
			array[1] = origin + longitudinalForce;
			manager->DrawSchematicCallback(this, "longitudinalForce", 0, 2, array);

			dVector veloc(0.0f);
			NewtonBodyGetVelocity(tireBody, &veloc[0]);
			veloc = chassisMatrix.UnrotateVector(veloc);
			veloc.m_y = 0.0f;
			veloc = projectionMatrix.RotateVector(veloc);
			array[0] = origin;
			array[1] = origin + veloc.Scale(velocityScale);
			manager->DrawSchematicCallback(this, "tireVelocity", 0, 2, array);
		}
	}
}

dCustomVehicleControllerManager::dCustomVehicleControllerManager(NewtonWorld* const world, int materialCount, int* const materialsList)
	:dCustomControllerManager<dCustomVehicleController> (world, VEHICLE_PLUGIN_NAME)
	,m_tireMaterial(NewtonMaterialCreateGroupID(world))
{
	// create the normalized size tire shape
	m_tireShapeTemplate = NewtonCreateChamferCylinder(world, 0.5f, 1.0f, 0, NULL);
	m_tireShapeTemplateData = NewtonCollisionDataPointer(m_tireShapeTemplate);

	// create a tire material and associate with the material the vehicle new to collide 
	for (int i = 0; i < materialCount; i ++) {
		NewtonMaterialSetCallbackUserData (world, m_tireMaterial, materialsList[i], this);
		if (m_tireMaterial != materialsList[i]) {
			NewtonMaterialSetContactGenerationCallback (world, m_tireMaterial, materialsList[i], OnContactGeneration);
		}
		NewtonMaterialSetCollisionCallback(world, m_tireMaterial, materialsList[i], OnTireAABBOverlap, OnTireContactsProcess);
	}
}

dCustomVehicleControllerManager::~dCustomVehicleControllerManager()
{
	NewtonDestroyCollision(m_tireShapeTemplate);
}

void dCustomVehicleControllerManager::DestroyController(dCustomVehicleController* const controller)
{
	controller->Cleanup();
	dCustomControllerManager<dCustomVehicleController>::DestroyController(controller);
}

int dCustomVehicleControllerManager::GetTireMaterial() const
{
	return m_tireMaterial;
}

dCustomVehicleController* dCustomVehicleControllerManager::CreateVehicle(NewtonBody* const body, const dMatrix& vehicleFrame, NewtonApplyForceAndTorque forceAndTorque, void* const userData, dFloat gravityMag)
{
	dCustomVehicleController* const controller = CreateController();
	controller->Init (body, vehicleFrame, forceAndTorque, userData, gravityMag);
	return controller;
}

dCustomVehicleController* dCustomVehicleControllerManager::CreateVehicle(NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, NewtonApplyForceAndTorque forceAndTorque, void* const userData, dFloat gravityMag)
{
	dCustomVehicleController* const controller = CreateController();
	controller->Init(chassisShape, vehicleFrame, mass, forceAndTorque, userData, gravityMag);
	return controller;
}

void dCustomVehicleController::Init(NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, NewtonApplyForceAndTorque forceAndTorque, void* const userData, dFloat gravityMag)
{
	dCustomVehicleControllerManager* const manager = (dCustomVehicleControllerManager*)GetManager();
	NewtonWorld* const world = manager->GetWorld();

	// create a body and an call the low level init function
	dMatrix locationMatrix(dGetIdentityMatrix());
	NewtonBody* const body = NewtonCreateDynamicBody(world, chassisShape, &locationMatrix[0][0]);

	// set vehicle mass, inertia and center of mass
	NewtonBodySetMassProperties(body, mass, chassisShape);

	// initialize 
	Init(body, vehicleFrame, forceAndTorque, userData, gravityMag);
}

void dCustomVehicleController::Init(NewtonBody* const body, const dMatrix& vehicleFrame___, NewtonApplyForceAndTorque forceAndTorque, void* const userData, dFloat gravityMag)
{
	m_body = body;
	m_speed = 0.0f;
	m_sideSlip = 0.0f;
	m_prevSideSlip = 0.0f;
	m_finalized = false;
	m_gravityMag = gravityMag;
	m_weightDistribution = 0.5f;
//	m_lateralDynamicControl = NULL;
	m_forceAndTorqueCallback = forceAndTorque;

	dCustomVehicleControllerManager* const manager = (dCustomVehicleControllerManager*)GetManager();
	NewtonWorld* const world = manager->GetWorld();

	// set linear and angular drag to zero
	dVector drag(0.0f);
	NewtonBodySetLinearDamping(m_body, 0);
	NewtonBodySetAngularDamping(m_body, &drag[0]);

	// set the standard force and torque call back
	NewtonBodySetForceAndTorqueCallback(body, m_forceAndTorqueCallback);

	m_contactFilter = new dBodyPartTire::dFrictionModel(this);

	m_engine = NULL;
	m_brakesControl = NULL;
	m_engineControl = NULL;
	m_handBrakesControl = NULL;
	m_steeringControl = NULL;
	
	m_collisionAggregate = NewtonCollisionAggregateCreate(world);
	NewtonCollisionAggregateSetSelfCollision (m_collisionAggregate, 0);
	NewtonCollisionAggregateAddBody (m_collisionAggregate, m_body);

	m_chassis.Init(this, userData);
	m_bodyPartsList.Append(&m_chassis);

	SetAerodynamicsDownforceCoefficient(0.5f, 0.4f, 1.0f);

#ifdef D_PLOT_ENGINE_CURVE 
	file_xxx = fopen("vehiceLog.csv", "wb");
	fprintf (file_xxx, "eng_rpm, eng_torque, eng_nominalTorque,\n");
#endif
}

void dCustomVehicleController::Cleanup()
{
	if (m_engine) {
		delete m_engine;
	}

	SetBrakes(NULL);
	SetEngine(NULL);
	SetSteering(NULL);
	SetHandBrakes(NULL);
	SetContactFilter(NULL);
//	m_lateralDynamicControl = NULL;
}

const dCustomVehicleController::dBodyPart* dCustomVehicleController::GetChassis() const
{
	return &m_chassis;
}

/*
const dMatrix& dCustomVehicleController::GetLocalFrame() const
{
	dAssert (0);
//	return m_localFrame;
	return dGetIdentityMatrix();
}
*/

dMatrix dCustomVehicleController::GetTransform() const
{
	dMatrix matrix;
	NewtonBodyGetMatrix (m_chassis.GetBody(), &matrix[0][0]);
	return matrix;
}

void dCustomVehicleController::SetTransform(const dMatrix& matrix)
{
	NewtonBodySetMatrixRecursive (m_chassis.GetBody(), &matrix[0][0]);
}


dCustomVehicleController::dEngineController* dCustomVehicleController::GetEngine() const
{
	return m_engineControl;
}


dCustomVehicleController::dSteeringController* dCustomVehicleController::GetSteering() const
{
	return m_steeringControl;
}

dCustomVehicleController::dBrakeController* dCustomVehicleController::GetBrakes() const
{
	return m_brakesControl;
}

dCustomVehicleController::dBrakeController* dCustomVehicleController::GetHandBrakes() const
{
	return m_handBrakesControl;
}

void dCustomVehicleController::SetEngine(dEngineController* const engineControl)
{
	if (m_engineControl) {
		delete m_engineControl;
	}
	m_engineControl = engineControl;
}


void dCustomVehicleController::SetHandBrakes(dBrakeController* const handBrakes)
{
	if (m_handBrakesControl) {
		delete m_handBrakesControl;
	}
	m_handBrakesControl = handBrakes;
}

void dCustomVehicleController::SetBrakes(dBrakeController* const brakes)
{
	if (m_brakesControl) {
		delete m_brakesControl;
	}
	m_brakesControl = brakes;
}


void dCustomVehicleController::SetSteering(dSteeringController* const steering)
{
	if (m_steeringControl) {
		delete m_steeringControl;
	}
	m_steeringControl = steering;
}

void dCustomVehicleController::SetContactFilter(dBodyPartTire::dFrictionModel* const filter)
{
	if (m_contactFilter) {
		delete m_contactFilter;
	}
	m_contactFilter = filter;
}

dList<dCustomVehicleController::dBodyPartTire>::dListNode* dCustomVehicleController::GetFirstTire() const
{
	return m_tireList.GetFirst();
}

dList<dCustomVehicleController::dBodyPartTire>::dListNode* dCustomVehicleController::GetNextTire(dList<dCustomVehicleController::dBodyPartTire>::dListNode* const tireNode) const
{
	return tireNode->GetNext();
}

dList<dCustomVehicleController::dBodyPart*>::dListNode* dCustomVehicleController::GetFirstBodyPart() const
{
	return m_bodyPartsList.GetFirst();
}

dList<dCustomVehicleController::dBodyPart*>::dListNode* dCustomVehicleController::GetNextBodyPart(dList<dBodyPart*>::dListNode* const part) const
{
	return part->GetNext();
}

void dCustomVehicleController::SetCenterOfGravity(const dVector& comRelativeToGeomtriCenter)
{
	NewtonBodySetCentreOfMass(m_body, &comRelativeToGeomtriCenter[0]);
}

dFloat dCustomVehicleController::GetWeightDistribution() const
{
	return m_weightDistribution;
}

void dCustomVehicleController::SetWeightDistribution(dFloat weightDistribution)
{
	dFloat totalMass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;

	NewtonBodyGetMass (m_body, &totalMass, &Ixx, &Iyy, &Izz);

	m_totalMass = totalMass;
	m_weightDistribution = dClamp (weightDistribution, dFloat(0.0f), dFloat(1.0f));
	if (m_finalized) {
		dFloat factor = m_weightDistribution - 0.5f;

		dMatrix matrix;
		dMatrix tireMatrix;
		dVector origin(0.0f);
		dVector totalMassOrigin (0.0f);
		dFloat xMin = 1.0e10f;
		dFloat xMax = -1.0e10f;

		NewtonBodyGetMatrix(m_body, &matrix[0][0]);
		NewtonBodyGetCentreOfMass(m_body, &origin[0]);
		matrix.m_posit = matrix.TransformVector(origin);
		totalMassOrigin = origin.Scale (totalMass);

		matrix = matrix.Inverse();
		if (m_engine) {
			dMatrix engineMatrixMatrix;
			dFloat mass;
			NewtonBodyGetMass (m_engine->GetBody(), &mass, &Ixx, &Iyy, &Izz);
			NewtonBodyGetMatrix(m_engine->GetBody(), &engineMatrixMatrix[0][0]);
			totalMass += mass;
			engineMatrixMatrix = engineMatrixMatrix * matrix;
			totalMassOrigin += engineMatrixMatrix.m_posit.Scale (mass);
		}

		for (dList<dBodyPartTire>::dListNode* node = m_tireList.GetFirst(); node; node = node->GetNext()) {
			dFloat mass;
			dBodyPartTire* const tire = &node->GetInfo();
			NewtonBodyGetMatrix(tire->GetBody(), &tireMatrix[0][0]);
			NewtonBodyGetMass (tire->GetBody(), &mass, &Ixx, &Iyy, &Izz);

			totalMass += mass;
			tireMatrix = tireMatrix * matrix;
			totalMassOrigin += tireMatrix.m_posit.Scale (mass);
			xMin = dMin (xMin, tireMatrix.m_posit.m_x); 
			xMax = dMax (xMax, tireMatrix.m_posit.m_x); 
		}
		origin = totalMassOrigin.Scale (1.0f / totalMass);

		dVector vehCom (0.0f);
		NewtonBodyGetCentreOfMass(m_body, &vehCom[0]);
		vehCom.m_x = origin.m_x + (xMax - xMin) * factor;
		vehCom.m_z = origin.m_z;
		NewtonBodySetCentreOfMass(m_body, &vehCom[0]);
		m_totalMass = totalMass;
	}

//	if (m_lateralDynamicControl) {
//		delete m_lateralDynamicControl;
//	}
//	m_lateralDynamicControl = new dLateralDynamicsJoint(this);
}

void dCustomVehicleController::Finalize()
{
	m_finalized = true;
	SetWeightDistribution (m_weightDistribution);
}

bool dCustomVehicleController::ControlStateChanged() const
{
	bool inputChanged = (m_steeringControl && m_steeringControl->ParamChanged());
	inputChanged = inputChanged || (m_steeringControl && !m_steeringControl->m_isSleeping);
	inputChanged = inputChanged || (m_engineControl && m_engineControl ->ParamChanged());
	inputChanged = inputChanged || (m_brakesControl && m_brakesControl->ParamChanged());
	inputChanged = inputChanged || (m_handBrakesControl && m_handBrakesControl->ParamChanged());
//	inputChanged = inputChanged || m_hasNewContact;
	return inputChanged;
}

dCustomVehicleController::dBodyPartTire* dCustomVehicleController::AddTire(const dBodyPartTire::Info& tireInfo)
{
	dList<dBodyPartTire>::dListNode* const tireNode = m_tireList.Append();
	dBodyPartTire& tire = tireNode->GetInfo();

	// calculate the tire matrix location,
	dMatrix matrix;
	NewtonBodyGetMatrix(m_body, &matrix[0][0]);
//	matrix = m_localFrame * matrix;
	matrix.m_posit = matrix.TransformVector (tireInfo.m_location);
	matrix.m_posit.m_w = 1.0f;

	tire.Init(&m_chassis, matrix, tireInfo);
	tire.m_index = m_tireList.GetCount() - 1;

	m_bodyPartsList.Append(&tire);
	NewtonCollisionAggregateAddBody (m_collisionAggregate, tire.GetBody());
	return &tireNode->GetInfo();
}

dCustomVehicleController::dBodyPartDifferential* dCustomVehicleController::AddDifferential(dBodyPartTire* const leftTire, dBodyPartTire* const rightTire)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	NewtonBodyGetMass(leftTire->GetBody(), &mass, &Ixx, &Iyy, &Izz);

	dList<dBodyPartDifferential>::dListNode* const differentialNode = m_differentialList.Append();
	dBodyPartDifferential* const differential = &differentialNode->GetInfo();
	differential->Init(this, mass, Ixx);

	NewtonBody* const chassisBody = GetBody();
	NewtonBody* const differentialBody = differential->GetBody();

	m_bodyPartsList.Append(differential);
	NewtonCollisionAggregateAddBody(m_collisionAggregate, differentialBody);

	dMatrix chassisMatrix;
	dMatrix differentialMatrix;
	NewtonBodyGetMatrix(chassisBody, &chassisMatrix[0][0]);
	NewtonBodyGetMatrix(differentialBody, &differentialMatrix[0][0]);
	
	dMatrix leftTireMatrix;
	NewtonBody* const leftTireBody = leftTire->GetBody();
	dAssert(leftTireBody == leftTire->GetJoint()->GetBody0());
	NewtonBodyGetMatrix(leftTireBody, &leftTireMatrix[0][0]);
	leftTireMatrix = leftTire->GetJoint()->GetMatrix0() * leftTireMatrix;
	new dAxelJoint(leftTireMatrix[0], differentialMatrix[0].Scale(-1.0f), chassisMatrix[2], leftTireBody, differentialBody, chassisBody);

	dMatrix rightTireMatrix;
	NewtonBody* const rightTireBody = rightTire->GetBody();
	dAssert(rightTireBody == rightTire->GetJoint()->GetBody0());
	NewtonBodyGetMatrix(rightTireBody, &rightTireMatrix[0][0]);
	rightTireMatrix = rightTire->GetJoint()->GetMatrix0() * rightTireMatrix;
	new dAxelJoint(rightTireMatrix[0], differentialMatrix[0].Scale(1.0f), chassisMatrix[2], rightTireBody, differentialBody, chassisBody);

	return differential;
}


dCustomVehicleController::dBodyPartDifferential* dCustomVehicleController::AddDifferential(dBodyPartDifferential* const leftDifferential, dBodyPartDifferential* const rightDifferential)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	NewtonBodyGetMass(leftDifferential->GetBody(), &mass, &Ixx, &Iyy, &Izz);

	dList<dBodyPartDifferential>::dListNode* const differentialNode = m_differentialList.Append();
	dBodyPartDifferential* const differential = &differentialNode->GetInfo();
	differential->Init(this, mass, Ixx);

	NewtonBody* const chassisBody = GetBody();
	NewtonBody* const differentialBody = differential->GetBody();

	m_bodyPartsList.Append(differential);
	NewtonCollisionAggregateAddBody(m_collisionAggregate, differentialBody);

	dMatrix chassisMatrix;
	dMatrix differentialMatrix;
	NewtonBodyGetMatrix(chassisBody, &chassisMatrix[0][0]);
	NewtonBodyGetMatrix(differentialBody, &differentialMatrix[0][0]);

	dMatrix leftDifferentialMatrix;
	NewtonBody* const leftDifferentialBody = leftDifferential->GetBody();
	dAssert(leftDifferentialBody == leftDifferential->GetJoint()->GetBody0());
	NewtonBodyGetMatrix(leftDifferentialBody, &leftDifferentialMatrix[0][0]);
	leftDifferentialMatrix = leftDifferential->GetJoint()->GetMatrix0() * leftDifferentialMatrix;
	new dAxelJoint(leftDifferentialMatrix[1], differentialMatrix[0].Scale(-1.0f), chassisMatrix[2], leftDifferentialBody, differentialBody, chassisBody);

	dMatrix rightDifferentialMatrix;
	NewtonBody* const rightDifferentialBody = rightDifferential->GetBody();
	dAssert(rightDifferentialBody == rightDifferential->GetJoint()->GetBody0());
	NewtonBodyGetMatrix(rightDifferentialBody, &rightDifferentialMatrix[0][0]);
	rightDifferentialMatrix = rightDifferential->GetJoint()->GetMatrix0() * rightDifferentialMatrix;
	new dAxelJoint(rightDifferentialMatrix[1], differentialMatrix[0].Scale(1.0f), chassisMatrix[2], rightDifferentialBody, differentialBody, chassisBody);

	return differential;
}


void dCustomVehicleController::LinkTiresKinematically(dBodyPartTire* const tire0, dBodyPartTire* const tire1)
{
	dWheelJoint* const joint0 = (dWheelJoint*)tire0->GetJoint();
	dWheelJoint* const joint1 = (dWheelJoint*)tire1->GetJoint();

	dMatrix matrix;
	dMatrix tireMatrix0;
	dMatrix tireMatrix1;
	joint0->CalculateGlobalMatrix(tireMatrix0, matrix);
	joint1->CalculateGlobalMatrix(tireMatrix1, matrix);

	dFloat gearRatio = tire0->m_data.m_radio / tire1->m_data.m_radio;
	new dCustomGear(gearRatio, tireMatrix0.m_front, tireMatrix1.m_front.Scale (-1.0f), tire0->GetBody(), tire1->GetBody());
}

dCustomVehicleController::dBodyPartEngine* dCustomVehicleController::AddEngine (dFloat mass, dFloat armatureRadius)
{
	m_engine = new dBodyPartEngine (this, mass, armatureRadius);

	m_bodyPartsList.Append(m_engine);
	NewtonCollisionAggregateAddBody(m_collisionAggregate, m_engine->GetBody());
	return m_engine;
}

dVector dCustomVehicleController::GetTireNormalForce(const dBodyPartTire* const tire) const
{
	dWheelJoint* const joint = (dWheelJoint*) tire->GetJoint();
	dFloat force = joint->GetTireLoad();
	return dVector (0.0f, force, 0.0f, 0.0f);
}

dVector dCustomVehicleController::GetTireLateralForce(const dBodyPartTire* const tire) const
{
	dWheelJoint* const joint = (dWheelJoint*)tire->GetJoint();
	return joint->GetLateralForce();
}

dVector dCustomVehicleController::GetTireLongitudinalForce(const dBodyPartTire* const tire) const
{
	dWheelJoint* const joint = (dWheelJoint*)tire->GetJoint();
	return joint->GetLongitudinalForce();
}

dFloat dCustomVehicleController::GetAerodynamicsDowforceCoeficient() const
{
	return m_chassis.m_aerodynamicsDownForceCoefficient;
}

void dCustomVehicleController::SetAerodynamicsDownforceCoefficient(dFloat downWeightRatioAtSpeedFactor, dFloat speedFactor, dFloat maxWeightAtTopSpeed)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	dAssert (speedFactor >= 0.0f);
	dAssert (speedFactor <= 1.0f);
	dAssert (downWeightRatioAtSpeedFactor >= 0.0f);
	dAssert (downWeightRatioAtSpeedFactor < maxWeightAtTopSpeed);
	NewtonBody* const body = GetBody();
	NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);
	dFloat topSpeed = m_engineControl ? m_engineControl->GetTopSpeed() : 25.0f;
	m_chassis.m_aerodynamicsDownSpeedCutOff = topSpeed * speedFactor;
	m_chassis.m_aerodynamicsDownForce0 = mass * downWeightRatioAtSpeedFactor * dAbs (m_gravityMag);
	m_chassis.m_aerodynamicsDownForce1 = mass * maxWeightAtTopSpeed * dAbs (m_gravityMag);
	m_chassis.m_aerodynamicsDownForceCoefficient = m_chassis.m_aerodynamicsDownForce0 / (m_chassis.m_aerodynamicsDownSpeedCutOff * m_chassis.m_aerodynamicsDownSpeedCutOff);
}

int dCustomVehicleControllerManager::OnTireAABBOverlap(const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex)
{
	dCustomVehicleControllerManager* const manager = (dCustomVehicleControllerManager*)NewtonMaterialGetMaterialPairUserData(material);

	const NewtonCollision* const collision0 = NewtonBodyGetCollision(body0);
	const void* const data0 = NewtonCollisionDataPointer(collision0);
	if (data0 == manager->m_tireShapeTemplateData) {
		const NewtonBody* const otherBody = body1;
		const dCustomVehicleController::dBodyPartTire* const tire = (dCustomVehicleController::dBodyPartTire*) NewtonCollisionGetUserData1(collision0);
		dAssert(tire->GetParent()->GetBody() != otherBody);
		return manager->OnTireAABBOverlap(material, tire, otherBody);
	} 
	const NewtonCollision* const collision1 = NewtonBodyGetCollision(body1);
	dAssert (NewtonCollisionDataPointer(collision1) == manager->m_tireShapeTemplateData) ;
	const NewtonBody* const otherBody = body0;
	const dCustomVehicleController::dBodyPartTire* const tire = (dCustomVehicleController::dBodyPartTire*) NewtonCollisionGetUserData1(collision1);
	dAssert(tire->GetParent()->GetBody() != otherBody);
	return manager->OnTireAABBOverlap(material, tire, otherBody);
}

int dCustomVehicleControllerManager::OnTireAABBOverlap(const NewtonMaterial* const material, const dCustomVehicleController::dBodyPartTire* const tire, const NewtonBody* const otherBody) const
{
	for (int i = 0; i < tire->m_collidingCount; i ++) {
		if (otherBody == tire->m_contactInfo[i].m_hitBody) {
			return 1;
		}
	}
//	tire->GetController()->m_hasNewContact |= tire->m_data.m_hasFender ? false : true;
	return tire->m_data.m_hasFender ? 0 : 1;
}

int dCustomVehicleControllerManager::OnContactGeneration (const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonCollision* const collision0, const NewtonBody* const body1, const NewtonCollision* const collision1, NewtonUserContactPoint* const contactBuffer, int maxCount, int threadIndex)
{
	dCustomVehicleControllerManager* const manager = (dCustomVehicleControllerManager*) NewtonMaterialGetMaterialPairUserData(material);
	const void* const data0 = NewtonCollisionDataPointer(collision0);
//	const void* const data1 = NewtonCollisionDataPointer(collision1);
	dAssert ((data0 == manager->m_tireShapeTemplateData) || (NewtonCollisionDataPointer(collision1) == manager->m_tireShapeTemplateData));
	dAssert (!((data0 == manager->m_tireShapeTemplateData) && (NewtonCollisionDataPointer(collision1) == manager->m_tireShapeTemplateData)));

	if (data0 == manager->m_tireShapeTemplateData) {
		const NewtonBody* const otherBody = body1;
		const NewtonCollision* const tireCollision = collision0;
		const NewtonCollision* const otherCollision = collision1;
		const dCustomVehicleController::dBodyPartTire* const tire = (dCustomVehicleController::dBodyPartTire*) NewtonCollisionGetUserData1(tireCollision);
		dAssert (tire->GetBody() == body0);
		return manager->OnContactGeneration (tire, otherBody, otherCollision, contactBuffer, maxCount, threadIndex);
	} 
	dAssert (NewtonCollisionDataPointer(collision1) == manager->m_tireShapeTemplateData);
	const NewtonBody* const otherBody = body0;
	const NewtonCollision* const tireCollision = collision1;
	const NewtonCollision* const otherCollision = collision0;
	const dCustomVehicleController::dBodyPartTire* const tire = (dCustomVehicleController::dBodyPartTire*) NewtonCollisionGetUserData1(tireCollision);
	dAssert (tire->GetBody() == body1);
	int count = manager->OnContactGeneration(tire, otherBody, otherCollision, contactBuffer, maxCount, threadIndex);

	for (int i = 0; i < count; i ++) {	
		contactBuffer[i].m_normal[0] *= -1.0f;
		contactBuffer[i].m_normal[1] *= -1.0f;
		contactBuffer[i].m_normal[2] *= -1.0f;
		dSwap (contactBuffer[i].m_shapeId0, contactBuffer[i].m_shapeId1);
	}
	return count;
}


int dCustomVehicleControllerManager::Collide(dCustomVehicleController::dBodyPartTire* const tire, int threadIndex) const
{
	dMatrix tireMatrix;
	dMatrix chassisMatrix;

	const NewtonWorld* const world = GetWorld();
	const NewtonBody* const tireBody = tire->GetBody();
	const NewtonBody* const vehicleBody = tire->GetParent()->GetBody();
	dCustomVehicleController* const controller = tire->GetController();

	NewtonBodyGetMatrix(tireBody, &tireMatrix[0][0]);
	NewtonBodyGetMatrix(vehicleBody, &chassisMatrix[0][0]);

//	chassisMatrix = controller->m_localFrame * chassisMatrix;
	dVector tireSidePin (tireMatrix.RotateVector(tire->GetJoint()->GetMatrix0().m_front));
	chassisMatrix.m_posit = chassisMatrix.TransformVector(tire->m_data.m_location) - tireSidePin.Scale (tire->m_data.m_pivotOffset);
	chassisMatrix.m_posit.m_w = 1.0f;

	dVector suspensionSpan (chassisMatrix.m_up.Scale(tire->m_data.m_suspesionlenght));

	dMatrix tireSweeptMatrix;
	tireSweeptMatrix.m_up = chassisMatrix.m_up;
	tireSweeptMatrix.m_right = tireMatrix.m_front.CrossProduct(chassisMatrix.m_up);
	tireSweeptMatrix.m_right = tireSweeptMatrix.m_right.Scale(1.0f / dSqrt(tireSweeptMatrix.m_right.DotProduct3(tireSweeptMatrix.m_right)));
	tireSweeptMatrix.m_front = tireSweeptMatrix.m_up.CrossProduct(tireSweeptMatrix.m_right);
	tireSweeptMatrix.m_posit = chassisMatrix.m_posit + suspensionSpan;

	NewtonCollision* const tireCollision = NewtonBodyGetCollision(tireBody);
	dTireFilter filter(tire, controller);

	dFloat timeOfImpact;
	tire->m_collidingCount = 0;
	const int maxContactCount = 2;
	dAssert (sizeof (tire->m_contactInfo) / sizeof (tire->m_contactInfo[0]) > 2);
	int count = NewtonWorldConvexCast (world, &tireSweeptMatrix[0][0], &chassisMatrix.m_posit[0], tireCollision, &timeOfImpact, &filter, dCustomControllerConvexCastPreFilter::Prefilter, tire->m_contactInfo, maxContactCount, threadIndex);

	if (timeOfImpact < 1.0e-2f) {
		class CheckBadContact: public dTireFilter
		{
			public:
			CheckBadContact(const dCustomVehicleController::dBodyPartTire* const tire, const dCustomVehicleController* const controller, int oldCount, NewtonWorldConvexCastReturnInfo* const oldInfo)
				:dTireFilter(tire, controller)
				,m_oldCount(oldCount) 
				,m_oldInfo(oldInfo)
			{
			}

			unsigned Prefilter(const NewtonBody* const body, const NewtonCollision* const myCollision)
			{
				for (int i = 0; i < m_oldCount; i ++) {
					if (body == m_oldInfo[i].m_hitBody) {
						return 0;
					}
				}

				return dTireFilter::Prefilter(body, myCollision);
			}

			int m_oldCount;
			NewtonWorldConvexCastReturnInfo* m_oldInfo;
		};
			
		
		dFloat timeOfImpact1;
		NewtonWorldConvexCastReturnInfo contactInfo[4];
		CheckBadContact checkfilter (tire, controller, count, tire->m_contactInfo);
		int count1 = NewtonWorldConvexCast (world, &tireSweeptMatrix[0][0], &chassisMatrix.m_posit[0], tireCollision, &timeOfImpact1, &checkfilter, dCustomControllerConvexCastPreFilter::Prefilter, contactInfo, maxContactCount, threadIndex);
		if (count1) {
			count = count1;
			timeOfImpact = timeOfImpact1;
			for (int i = 0; i < count; i ++) {
				tire->m_contactInfo[i] = contactInfo[i];
			}
		}
	}


	if (count) {
		timeOfImpact = 1.0f - timeOfImpact;
		dFloat num = (tireMatrix.m_posit - chassisMatrix.m_up.Scale (0.25f * tire->m_data.m_suspesionlenght) - chassisMatrix.m_posit).DotProduct3(suspensionSpan);
		dFloat tireParam = dMax (num / (tire->m_data.m_suspesionlenght * tire->m_data.m_suspesionlenght), dFloat(0.0f));

		if (tireParam <= timeOfImpact) {
			tireSweeptMatrix.m_posit = chassisMatrix.m_posit + chassisMatrix.m_up.Scale(timeOfImpact * tire->m_data.m_suspesionlenght);
			for (int i = count - 1; i >= 0; i --) {
				dVector p (tireSweeptMatrix.UntransformVector (dVector (tire->m_contactInfo[i].m_point[0], tire->m_contactInfo[i].m_point[1], tire->m_contactInfo[i].m_point[2], 1.0f)));
				if ((p.m_y >= -(tire->m_data.m_radio * 0.5f)) || (dAbs (p.m_x / p.m_y) > 0.4f)) {
					tire->m_contactInfo[i] = tire->m_contactInfo[count - 1];
					count --;
				}
			}
			if (count) {
/*
				dFloat x1 = timeOfImpact * tire->m_data.m_suspesionlenght;
				dFloat x0 = (tireMatrix.m_posit - chassisMatrix.m_posit).DotProduct3(chassisMatrix.m_up);
				dFloat x10 = x1 - x0;
				if (x10 > (1.0f / 32.0f)) {
					dFloat param = 1.0e10f;
					x1 = x0 + (1.0f / 32.0f);
					dMatrix origin (chassisMatrix);
					origin.m_posit = chassisMatrix.m_posit + chassisMatrix.m_up.Scale(x1);
					NewtonWorldConvexCast (world, &chassisMatrix[0][0], &tireSweeptMatrix.m_posit[0], tireCollision, &param, &filter, dCustomControllerConvexCastPreFilter::Prefilter, NULL, 0, threadIndex);
					count = (param < 1.0f) ? 0 : count;
				}
				if (count) {
					tireMatrix.m_posit = chassisMatrix.m_posit + chassisMatrix.m_up.Scale(x1);
					NewtonBodySetMatrixNoSleep(tireBody, &tireMatrix[0][0]);
				}
*/
				dFloat x = timeOfImpact * tire->m_data.m_suspesionlenght;
				dFloat step = (tireSweeptMatrix.m_posit - tireMatrix.m_posit).DotProduct3(chassisMatrix.m_up);
				if (step < -1.0f / 32.0f) {
					count = 0;
				}

				if (count) {
					tireMatrix.m_posit = chassisMatrix.m_posit + chassisMatrix.m_up.Scale(x);
					NewtonBodySetMatrixNoSleep(tireBody, &tireMatrix[0][0]);
				}
			}
		} else {
			count = 0;
		}
	}

	tire->m_collidingCount = count;
	if (!tire->m_data.m_hasFender) {
		count = NewtonWorldCollide (world, &tireMatrix[0][0], tireCollision, &filter, dCustomControllerConvexCastPreFilter::Prefilter, &tire->m_contactInfo[count], maxContactCount, threadIndex);
		for (int i = 0; i < count; i++) {
			if (tire->m_contactInfo[tire->m_collidingCount + i].m_penetration == 0.0f) {
				tire->m_contactInfo[tire->m_collidingCount + i].m_penetration = 1.0e-5f;
			}
		}
		tire->m_collidingCount += count;
	}

	return tire->m_collidingCount ? 1 : 0;
}


void dCustomVehicleControllerManager::OnTireContactsProcess (const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
{
	void* const contact = NewtonContactJointGetFirstContact(contactJoint);
	dAssert (contact);
	NewtonMaterial* const material = NewtonContactGetMaterial(contact);
	dCustomVehicleControllerManager* const manager = (dCustomVehicleControllerManager*) NewtonMaterialGetMaterialPairUserData(material);

	const NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
	const NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);
	const NewtonCollision* const collision0 = NewtonBodyGetCollision(body0);
	const void* const data0 = NewtonCollisionDataPointer(collision0);
	if (data0 == manager->m_tireShapeTemplateData) {
		const NewtonBody* const otherBody = body1;
		dCustomVehicleController::dBodyPartTire* const tire = (dCustomVehicleController::dBodyPartTire*) NewtonCollisionGetUserData1(collision0);
		dAssert(tire->GetParent()->GetBody() != otherBody);
		manager->OnTireContactsProcess(contactJoint, tire, otherBody, timestep);
	} else {
		const NewtonCollision* const collision1 = NewtonBodyGetCollision(body1);
		const void* const data1 = NewtonCollisionDataPointer(collision1);
		if (data1 == manager->m_tireShapeTemplateData) {
			const NewtonCollision* const collision2 = NewtonBodyGetCollision(body1);
			dAssert(NewtonCollisionDataPointer(collision2) == manager->m_tireShapeTemplateData);
			const NewtonBody* const otherBody = body0;
			dCustomVehicleController::dBodyPartTire* const tire = (dCustomVehicleController::dBodyPartTire*) NewtonCollisionGetUserData1(collision2);
			dAssert(tire->GetParent()->GetBody() != otherBody);
			manager->OnTireContactsProcess(contactJoint, tire, otherBody, timestep);
		}
	}
}


int dCustomVehicleControllerManager::OnContactGeneration(const dCustomVehicleController::dBodyPartTire* const tire, const NewtonBody* const otherBody, const NewtonCollision* const othercollision, NewtonUserContactPoint* const contactBuffer, int maxCount, int threadIndex) const
{
	int count = 0;
	NewtonCollision* const collisionA = NewtonBodyGetCollision(tire->GetBody());
	dLong tireID = NewtonCollisionGetUserID(collisionA);
	for (int i = 0; i < tire->m_collidingCount; i++) {
		if (otherBody == tire->m_contactInfo[i].m_hitBody) {
			contactBuffer[count].m_point[0] = tire->m_contactInfo[i].m_point[0];
			contactBuffer[count].m_point[1] = tire->m_contactInfo[i].m_point[1];
			contactBuffer[count].m_point[2] = tire->m_contactInfo[i].m_point[2];
			contactBuffer[count].m_point[3] = 1.0f;
			contactBuffer[count].m_normal[0] = tire->m_contactInfo[i].m_normal[0];
			contactBuffer[count].m_normal[1] = tire->m_contactInfo[i].m_normal[1];
			contactBuffer[count].m_normal[2] = tire->m_contactInfo[i].m_normal[2];
			contactBuffer[count].m_normal[3] = 0.0f;
			contactBuffer[count].m_penetration = tire->m_contactInfo[i].m_penetration;
			contactBuffer[count].m_shapeId0 = tireID;
			contactBuffer[count].m_shapeId1 = tire->m_contactInfo[i].m_contactID;
			count++;
		}
	}
	return count;
}

void dCustomVehicleControllerManager::OnTireContactsProcess(const NewtonJoint* const contactJoint, dCustomVehicleController::dBodyPartTire* const tire, const NewtonBody* const otherBody, dFloat timestep)
{
	dMatrix tireMatrix;
	dMatrix chassisMatrix;
	dVector tireOmega(0.0f);
	dVector tireVeloc(0.0f);

	NewtonBody* const tireBody = tire->GetBody();
	dAssert((tireBody == NewtonJointGetBody0(contactJoint)) || (tireBody == NewtonJointGetBody1(contactJoint)));
	const dCustomVehicleController* const controller = tire->GetController();
	dCustomVehicleController::dWheelJoint* const tireJoint = (dCustomVehicleController::dWheelJoint*) tire->GetJoint();

	dAssert(tireJoint->GetBody0() == tireBody);
	tireJoint->CalculateGlobalMatrix(tireMatrix, chassisMatrix);

	NewtonBodyGetOmega(tireBody, &tireOmega[0]);
	NewtonBodyGetVelocity(tireBody, &tireVeloc[0]);

	dVector lateralPin (tireMatrix.m_front);
	dVector longitudinalPin (tireMatrix.m_front.CrossProduct(chassisMatrix.m_up));

	tire->m_lateralSlip = 0.0f;
	tire->m_aligningTorque = 0.0f;
	tire->m_longitudinalSlip = 0.0f;

	for (void* contact = NewtonContactJointGetFirstContact(contactJoint); contact; contact = NewtonContactJointGetNextContact(contactJoint, contact)) {
		dVector contactPoint(0.0f);
		dVector contactNormal(0.0f);

		NewtonMaterial* const material = NewtonContactGetMaterial(contact);
		NewtonMaterialContactRotateTangentDirections(material, &lateralPin[0]);
		NewtonMaterialGetContactPositionAndNormal(material, tireBody, &contactPoint[0], &contactNormal[0]);
			
		dVector tireAnglePin(contactNormal.CrossProduct(lateralPin));
		dFloat pinMag2 = tireAnglePin.DotProduct3(tireAnglePin);
		if (pinMag2 > 0.25f) {
			// brush rubber tire friction model
			// project the contact point to the surface of the collision shape
			dVector contactPatch(contactPoint - lateralPin.Scale((contactPoint - tireMatrix.m_posit).DotProduct3(lateralPin)));
			dVector dp(contactPatch - tireMatrix.m_posit);
			dVector radius(dp.Scale(tire->m_data.m_radio / dSqrt(dp.DotProduct3(dp))));

			dVector lateralContactDir(0.0f);
			dVector longitudinalContactDir(0.0f);
			NewtonMaterialGetContactTangentDirections(material, tireBody, &lateralContactDir[0], &longitudinalContactDir[0]);

			dFloat tireOriginLongitudinalSpeed = tireVeloc.DotProduct3(longitudinalContactDir);
			dFloat tireContactLongitudinalSpeed = -longitudinalContactDir.DotProduct3 (tireOmega.CrossProduct(radius));

			if ((dAbs(tireOriginLongitudinalSpeed) < (1.0f)) || (dAbs(tireContactLongitudinalSpeed) < 0.1f)) {
				// vehicle moving low speed, do normal coulomb friction
				NewtonMaterialSetContactFrictionCoef(material, 1.0f, 1.0f, 0);
				NewtonMaterialSetContactFrictionCoef(material, 1.0f, 1.0f, 1);
			} else {

				// calculating Brush tire model with longitudinal and lateral coupling 
				// for friction coupling according to Motor Vehicle dynamics by: Giancarlo Genta 
				// reduces to this, which may have a divide by zero locked, so I am clamping to some small value
				// dFloat k = (vw - vx) / vx;
				if (dAbs(tireContactLongitudinalSpeed) < 0.01f) {
					tireContactLongitudinalSpeed = 0.01f * dSign(tireContactLongitudinalSpeed);
				}
				tire->m_longitudinalSlip = (tireContactLongitudinalSpeed - tireOriginLongitudinalSpeed) / tireOriginLongitudinalSpeed;

				dFloat lateralSpeed = tireVeloc.DotProduct3(lateralPin);
				dFloat longitudinalSpeed = tireVeloc.DotProduct3(longitudinalPin);
				dAssert (dAbs (longitudinalSpeed) > 0.01f);

				tire->m_lateralSlip = dAbs(lateralSpeed / longitudinalSpeed);

dVector chassisVeloc;
dVector chassisOmega;
dMatrix chassisMatrix1;
NewtonBody* const chassisBody = controller->GetBody();
NewtonBodyGetOmega(chassisBody, &chassisOmega[0]);
NewtonBodyGetVelocity(chassisBody, &chassisVeloc[0]);
NewtonBodyGetMatrix(chassisBody, &chassisMatrix1[0][0]);
dFloat x = chassisVeloc.DotProduct3 (chassisMatrix1.m_front);
dFloat y = chassisOmega.CrossProduct(tireMatrix.m_posit - chassisMatrix1.m_posit).DotProduct3 (chassisMatrix1.m_right);
dFloat xxx = tireJoint->m_steerAngle0;
dFloat xxxx = dFloat(atan (controller->m_sideSlip + y/x)) + xxx;
dTrace (("%d %f %f\n", tire->m_index, xxxx * 180.0f/3.1416f, dAtan2 (lateralSpeed, dAbs(longitudinalSpeed)) * 180.0f/3.1416f));

	
				dFloat aligningMoment;
				dFloat lateralFrictionCoef;
				dFloat longitudinalFrictionCoef;
				dFloat lateralSlipSensitivity = 2.0f;
				controller->m_contactFilter->CalculateTireFrictionCoefficents(tire, otherBody, material, 
					tire->m_longitudinalSlip, tire->m_lateralSlip * lateralSlipSensitivity, 
					tire->m_data.m_longitudialStiffness, tire->m_data.m_lateralStiffness, 
					longitudinalFrictionCoef, lateralFrictionCoef, aligningMoment);

//dTrace (("%d %f %f\n", tire->m_index, longitudinalFrictionCoef, lateralFrictionCoef));

				NewtonMaterialSetContactFrictionCoef(material, lateralFrictionCoef, lateralFrictionCoef, 0);
				NewtonMaterialSetContactFrictionCoef(material, longitudinalFrictionCoef, longitudinalFrictionCoef, 1);
#if 0
				NewtonMaterialSetContactFrictionCoef(material, 1.0f, 1.0f, 0);
				NewtonMaterialSetContactFrictionCoef(material, 1.0f, 1.0f, 1);
#endif
			}

		} else {
			// vehicle moving low speed, do normal coulomb friction
			NewtonMaterialSetContactFrictionCoef(material, 1.0f, 1.0f, 0);
			NewtonMaterialSetContactFrictionCoef(material, 1.0f, 1.0f, 1);
		}

		NewtonMaterialSetContactElasticity(material, 0.0f);
	}
}


dVector dCustomVehicleController::GetLastLateralForce(dBodyPartTire* const tire) const
{
	return (GetTireLateralForce(tire) + GetTireLongitudinalForce(tire)).Scale (-1.0f);
}


void dCustomVehicleController::ApplySuspensionForces(dFloat timestep) const
{
	dMatrix chassisMatrix;
	dMatrix chassisInvInertia;
	dVector chassisOrigin(0.0f);
	dVector chassisForce(0.0f);
	dVector chassisTorque(0.0f);

	const int maxSize = 64;
	dComplentaritySolver::dJacobianPair m_jt[maxSize];
	dComplentaritySolver::dJacobianPair m_jInvMass[maxSize];
	dBodyPartTire* tires[maxSize];
	dFloat accel[maxSize];
	dFloat massMatrix[maxSize * maxSize];
	dFloat chassisMass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;

	NewtonBody* const chassisBody = m_chassis.GetBody();
	NewtonBodyGetCentreOfMass(chassisBody, &chassisOrigin[0]);
	NewtonBodyGetMatrix(chassisBody, &chassisMatrix[0][0]);
	NewtonBodyGetInvMass(chassisBody, &chassisMass, &Ixx, &Iyy, &Izz);
	NewtonBodyGetInvInertiaMatrix(chassisBody, &chassisInvInertia[0][0]);

	chassisOrigin = chassisMatrix.TransformVector(chassisOrigin);
	int tireCount = 0;
	for (dList<dBodyPartTire>::dListNode* tireNode = m_tireList.GetFirst(); tireNode; tireNode = tireNode->GetNext()) {
		dBodyPartTire& tire = tireNode->GetInfo();

		if (tire.m_data.m_suspentionType != dBodyPartTire::Info::m_roller) {
			tires[tireCount] = &tire;

			const dWheelJoint* const joint = (dWheelJoint*)tire.GetJoint();
			NewtonBody* const tireBody = tire.GetBody();
			dAssert(tireBody == joint->GetBody0());
			dAssert(chassisBody == joint->GetBody1());

			dMatrix tireMatrix;
			//dMatrix chassisMatrix;
			dVector tireVeloc(0.0f);
			dVector chassisPivotVeloc(0.0f);

			joint->CalculateGlobalMatrix(tireMatrix, chassisMatrix);
			NewtonBodyGetVelocity(tireBody, &tireVeloc[0]);
			NewtonBodyGetPointVelocity(chassisBody, &tireMatrix.m_posit[0], &chassisPivotVeloc[0]);

			dFloat param = joint->CalculateTireParametricPosition(tireMatrix, chassisMatrix);
			param = dClamp(param, dFloat(-0.25f), dFloat(1.0f));

			dFloat x = tire.m_data.m_suspesionlenght * param;
			dFloat v = ((tireVeloc - chassisPivotVeloc).DotProduct3(chassisMatrix.m_up));

			dFloat weight = 1.0f;
			switch (tire.m_data.m_suspentionType)
			{
				case dBodyPartTire::Info::m_offroad:
					weight = 0.9f;
					break;
				case dBodyPartTire::Info::m_confort:
					weight = 1.0f;
					break;
				case dBodyPartTire::Info::m_race:
					weight = 1.1f;
					break;
			}
			accel[tireCount] = -NewtonCalculateSpringDamperAcceleration(timestep, tire.m_data.m_springStrength * weight, x, tire.m_data.m_dampingRatio, v);

			dMatrix tireInvInertia;
			dFloat tireMass;

			NewtonBodyGetInvMass(tireBody, &tireMass, &Ixx, &Iyy, &Izz);
			NewtonBodyGetInvInertiaMatrix(tireBody, &tireInvInertia[0][0]);

			m_jt[tireCount].m_jacobian_IM0.m_linear = chassisMatrix.m_up.Scale(-1.0f);
			m_jt[tireCount].m_jacobian_IM0.m_angular = dVector(0.0f);
			m_jt[tireCount].m_jacobian_IM1.m_linear = chassisMatrix.m_up;
			m_jt[tireCount].m_jacobian_IM1.m_angular = (tireMatrix.m_posit - chassisOrigin).CrossProduct(chassisMatrix.m_up);

			m_jInvMass[tireCount].m_jacobian_IM0.m_linear = m_jt[tireCount].m_jacobian_IM0.m_linear.Scale(tireMass);
			m_jInvMass[tireCount].m_jacobian_IM0.m_angular = tireInvInertia.RotateVector(m_jt[tireCount].m_jacobian_IM0.m_angular);
			m_jInvMass[tireCount].m_jacobian_IM1.m_linear = m_jt[tireCount].m_jacobian_IM1.m_linear.Scale(chassisMass);
			m_jInvMass[tireCount].m_jacobian_IM1.m_angular = chassisInvInertia.RotateVector(m_jt[tireCount].m_jacobian_IM1.m_angular);

			tireCount++;
		}
	}

	for (int i = 0; i < tireCount; i++) {
		dFloat* const row = &massMatrix[i * tireCount];
		dFloat aii = m_jInvMass[i].m_jacobian_IM0.m_linear.DotProduct3(m_jt[i].m_jacobian_IM0.m_linear) + m_jInvMass[i].m_jacobian_IM0.m_angular.DotProduct3(m_jt[i].m_jacobian_IM0.m_angular) +
					 m_jInvMass[i].m_jacobian_IM1.m_linear.DotProduct3(m_jt[i].m_jacobian_IM1.m_linear) + m_jInvMass[i].m_jacobian_IM1.m_angular.DotProduct3(m_jt[i].m_jacobian_IM1.m_angular);

		row[i] = aii * 1.0001f;
		for (int j = i + 1; j < tireCount; j++) {
			dFloat aij = m_jInvMass[i].m_jacobian_IM1.m_linear.DotProduct3(m_jt[j].m_jacobian_IM1.m_linear) + m_jInvMass[i].m_jacobian_IM1.m_angular.DotProduct3(m_jt[j].m_jacobian_IM1.m_angular);
			row[j] = aij;
			massMatrix[j * tireCount + i] = aij;
		}
	}

	dCholeskyFactorization(tireCount, massMatrix);
	dCholeskySolve(tireCount, tireCount, massMatrix, accel);

	for (int i = 0; i < tireCount; i++) {
		dVector tireForce(m_jt[i].m_jacobian_IM0.m_linear.Scale(accel[i]));
		dVector tireTorque(m_jt[i].m_jacobian_IM0.m_angular.Scale(accel[i]));
		NewtonBodyAddForce(tires[i]->GetBody(), &tireForce[0]);
		NewtonBodyAddTorque(tires[i]->GetBody(), &tireTorque[0]);
		chassisForce += m_jt[i].m_jacobian_IM1.m_linear.Scale(accel[i]);
		chassisTorque += m_jt[i].m_jacobian_IM1.m_angular.Scale(accel[i]);
	}
	NewtonBodyAddForce(chassisBody, &chassisForce[0]);
	NewtonBodyAddTorque(chassisBody, &chassisTorque[0]);
}


void dCustomVehicleController::CalculateSideSlipDynamics(dFloat timestep)
{
	dMatrix matrix;
	dVector veloc;

	NewtonBody* const chassisBody = m_chassis.GetBody();
	NewtonBodyGetMatrix(chassisBody, &matrix[0][0]);
	NewtonBodyGetVelocity(chassisBody, &veloc[0]);

	dFloat speed_x = veloc.DotProduct3(matrix.m_front);
	dFloat speed_z = veloc.DotProduct3(matrix.m_right);
	if (dAbs(speed_x) > 1.0f) {
		m_prevSideSlip = m_sideSlip;
		m_sideSlip = speed_z / dAbs(speed_x);
	} else {
		m_sideSlip = 0.0f;
		m_prevSideSlip = 0.0f;
	}

static int xxx;
dTrace (("\n%d b(%f) rate(%f)\n", xxx, m_sideSlip * 180.0f/3.1416f, (m_sideSlip - m_prevSideSlip) * (180.0f / 3.1416f) / timestep));
xxx ++;

if ((dAbs (m_sideSlip * 180.0f/3.1416f) > 35.0f))  {
	dVector xxx1 (matrix.m_up.Scale (-8000.0f * dSign(m_sideSlip)));
	NewtonBodyAddTorque (chassisBody, &xxx1[0]);
} else {
	dFloat betaRate = (m_sideSlip - m_prevSideSlip) / timestep;
	if (dAbs (betaRate * 180.0f/3.1416f) > 15.0f) {
		dVector xxx1 (matrix.m_up.Scale (-8000.0f * dSign(betaRate)));
		NewtonBodyAddTorque (chassisBody, &xxx1[0]);
	}
}

}


void dCustomVehicleController::PostUpdate(dFloat timestep, int threadIndex)
{
	dTimeTrackerEvent(__FUNCTION__);
	if (m_finalized) {
		for (dList<dBodyPart*>::dListNode* bodyPartNode = m_bodyPartsList.GetFirst(); bodyPartNode; bodyPartNode = bodyPartNode->GetNext()) {
			dBodyPart* const bodyPart = bodyPartNode->GetInfo();
			bodyPart->ProjectError();
		}

		if (!NewtonBodyGetSleepState(m_body)) {
			dCustomVehicleControllerManager* const manager = (dCustomVehicleControllerManager*)GetManager();
			for (dList<dBodyPartTire>::dListNode* tireNode = m_tireList.GetFirst(); tireNode; tireNode = tireNode->GetNext()) {
				dBodyPartTire& tire = tireNode->GetInfo();
				manager->Collide(&tire, threadIndex);
			}
		}

#ifdef D_PLOT_ENGINE_CURVE 
		dFloat engineOmega = m_engine->GetRPM();
		dFloat tireTorque = m_engine->GetLeftSpiderGear()->m_tireTorque + m_engine->GetRightSpiderGear()->m_tireTorque;
		dFloat engineTorque = m_engine->GetLeftSpiderGear()->m_engineTorque + m_engine->GetRightSpiderGear()->m_engineTorque;
		fprintf(file_xxx, "%f, %f, %f,\n", engineOmega, engineTorque, m_engine->GetNominalTorque());
#endif
	}
}


void dCustomVehicleController::PreUpdate(dFloat timestep, int threadIndex)
{
	dTimeTrackerEvent(__FUNCTION__);
	if (m_finalized) {
		m_chassis.ApplyDownForce ();
		CalculateSideSlipDynamics(timestep);
		ApplySuspensionForces (timestep);

		if (m_brakesControl) {
			m_brakesControl->Update(timestep);
		}

		if (m_handBrakesControl) {
			m_handBrakesControl->Update(timestep);
		}

		if (m_steeringControl) {
			m_steeringControl->Update(timestep);
		}

		if (m_engineControl) {
			m_engineControl->Update(timestep);
		}

		if (ControlStateChanged()) 
		{
			NewtonBodySetSleepState(m_body, 0);
		}
	}
}
