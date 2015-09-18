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
#include "CustomJointLibraryStdAfx.h"
#include "CustomJoint.h"
#include "CustomHinge.h"
#include "CustomVehicleControllerManager.h"

//#define D_PLOT_ENGINE_CURVE

#ifdef D_PLOT_ENGINE_CURVE 
static FILE* file_xxx;
#endif

#define D_VEHICLE_NEWTRAL_GEAR		0
#define D_VEHICLE_REVERSE_GEAR		1
#define D_VEHICLE_FIRST_GEAR		2

class CustomVehicleController::dWeightDistibutionSolver: public dSymmetricBiconjugateGradientSolve
{
	public:
	dWeightDistibutionSolver()
		:dSymmetricBiconjugateGradientSolve()
		,m_count(0)
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



void CustomVehicleController::dInterpolationCurve::InitalizeCurve(int points, const dFloat* const steps, const dFloat* const values)
{
	m_count = points;
	dAssert(points < int(sizeof(m_nodes) / sizeof (m_nodes[0])));
	memset(m_nodes, 0, sizeof (m_nodes));
	for (int i = 0; i < m_count; i++) {
		m_nodes[i].m_param = steps[i];
		m_nodes[i].m_value = values[i];
	}
}

dFloat CustomVehicleController::dInterpolationCurve::GetValue(dFloat param) const
{
	dFloat interplatedValue = 0.0f;
	if (m_count) {
		param = dClamp(param, 0.0f, m_nodes[m_count - 1].m_param);
		interplatedValue = m_nodes[m_count - 1].m_value;
		for (int i = 1; i < m_count; i++) {
			if (param < m_nodes[i].m_param) {
				dFloat df = m_nodes[i].m_value - m_nodes[i - 1].m_value;
				dFloat ds = m_nodes[i].m_param - m_nodes[i - 1].m_param;
				dFloat step = param - m_nodes[i - 1].m_param;

				interplatedValue = m_nodes[i - 1].m_value + df * step / ds;
				break;
			}
		}
	}
	return interplatedValue;
}


class CustomVehicleController::WheelJoint: public CustomJoint
{
	public:
	WheelJoint (const dMatrix& pinAndPivotFrame, NewtonBody* const tire, NewtonBody* const parentBody, BodyPartTire* const tireData)
		:CustomJoint (6, tire, parentBody)
		,m_data (tireData)
		,m_tireLoad(0.0f)
		,m_restLoad(0.0f)
		,m_steerAngle(0.0f)
		,m_brakeTorque(0.0f)
		,m_gravityMag(0.0f)
		,m_restSprunMass(0.0f)
	{
		CalculateLocalMatrix (pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
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
		NewtonUserJointSetRowAcceleration (m_joint, NewtonUserCalculateRowZeroAccelaration(m_joint));
		NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1.m_right[0]);
		NewtonUserJointSetRowAcceleration (m_joint, NewtonUserCalculateRowZeroAccelaration(m_joint));

		dMatrix matrix1_1;
		matrix1_1.m_up = matrix1.m_up;
		matrix1_1.m_right = matrix0.m_front * matrix1.m_up;
		matrix1_1.m_right = matrix1_1.m_right.Scale(1.0f / dSqrt(matrix1_1.m_right % matrix1_1.m_right));
		matrix1_1.m_front = matrix1_1.m_up * matrix1_1.m_right;

		NewtonBody* const tire = m_body0;
		NewtonBody* const chassis = m_body1;

		dVector tireOmega;
		dVector chassisOmega;
		NewtonBodyGetOmega(tire, &tireOmega[0]);
		NewtonBodyGetOmega(chassis, &chassisOmega[0]);
		dVector relOmega(tireOmega - chassisOmega);

		dFloat angle = -CalculateAngle(matrix0.m_front, matrix1_1.m_front, matrix1_1.m_right);
		dFloat omega = (relOmega % matrix1_1.m_right);
		dFloat alphaError = -(angle + omega * timestep) / (timestep * timestep);
		NewtonUserJointAddAngularRow(m_joint, -angle, &matrix1_1.m_right[0]);
		NewtonUserJointSetRowAcceleration(m_joint, alphaError);
		NewtonUserJointSetRowStiffness(m_joint, 1.0f);

		dFloat steerAngle = m_steerAngle - CalculateAngle (matrix1.m_front, matrix1_1.m_front, matrix1_1.m_up);
		dFloat steerOmega = (relOmega % matrix1_1.m_up);
		dFloat alphaSteerError = (steerAngle - steerOmega * timestep) / (timestep * timestep);
		NewtonUserJointAddAngularRow (m_joint, -steerAngle, &matrix1_1.m_up[0]);
		NewtonUserJointSetRowAcceleration(m_joint, alphaSteerError);
		NewtonUserJointSetRowStiffness(m_joint, 1.0f);

		if (posit > m_data->m_data.m_suspesionlenght) {
			dFloat x = dMax(posit, m_data->m_data.m_suspesionlenght + 0.01f);
			dVector p0(matrix1.m_posit + matrix1.m_up.Scale(x));
			NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1.m_up[0]);
			NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
		} else if (posit < 0.0f) {
			dFloat x = dMax(posit, -0.01f);
			dVector p0(matrix1.m_posit + matrix1.m_up.Scale(x));
			NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1.m_up[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
		}

		if (m_brakeTorque > 1.0e-3f) {
			dFloat speed = relOmega % matrix1_1.m_front;
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1_1.m_front[0]);
			NewtonUserJointSetRowAcceleration(m_joint, -speed / timestep);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_brakeTorque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_brakeTorque);
		}
	
		dVector tireVeloc;
		dVector chassisCom;
		dVector chassisVeloc;
		dMatrix chassisMatrix;
		NewtonBodyGetVelocity(tire, &tireVeloc[0]);
		NewtonBodyGetPointVelocity(chassis, &p0[0], &chassisVeloc[0]);
		NewtonBodyGetCentreOfMass(chassis, &chassisCom[0]);
		NewtonBodyGetMatrix(chassis, &chassisMatrix[0][0]);

		posit = dClamp(posit, 0.0f, m_data->m_data.m_suspesionlenght);
		chassisCom = p0 - chassisMatrix.TransformVector(chassisCom);
		dFloat speed = dClamp ((tireVeloc - chassisVeloc) % matrix1.m_up, -20.0f, 20.0f);
		dFloat load = - NewtonCalculateSpringDamperAcceleration (timestep, m_data->m_data.m_springStrength, posit, m_data->m_data.m_dampingRatio, speed);
		if (load < 0.0f) {
			load = 0.0f;
		}

		m_tireLoad = load * m_restSprunMass;
		dVector force (matrix1.m_up.Scale (m_tireLoad));
		dVector torque (chassisCom  * force);
		NewtonBodyAddForce(chassis, &force[0]);
		NewtonBodyAddTorque(chassis, &torque[0]);

		force = force.Scale (-1.0f);
		dVector tireWeight;
		NewtonBodyGetForceAcc(tire, &tireWeight[0]);
		NewtonBodyAddForce(tire, &force[0]);

		dFloat weight = -(matrix1.m_up % tireWeight);
		m_gravityMag = weight / m_data->m_data.m_mass;
		m_restLoad = (m_data->m_data.m_mass + m_restSprunMass) * m_gravityMag;
		m_tireLoad += weight;
		dAssert (m_tireLoad > 0.0f);
		m_brakeTorque = 0.0f;
	}

	dFloat GetGravityMag() const
	{
		return m_gravityMag;
	}

	dFloat GetTireRestLoad() const
	{
		return m_restLoad;
	}

	dFloat GetTireLoad () const
	{
		return m_tireLoad;
	}

	BodyPartTire* m_data;
	dFloat m_tireLoad;
	dFloat m_restLoad;
	dFloat m_steerAngle;
	dFloat m_brakeTorque;
	dFloat m_gravityMag;
	dFloat m_restSprunMass;
};

class CustomVehicleController::EngineJoint: public CustomJoint
{
	public:
	EngineJoint(NewtonBody* const engine, NewtonBody* const chassis, dFloat dryResistance)
		:CustomJoint(6, engine, chassis)
		,m_baseMatrix(dGetIdentityMatrix())
		,m_dryResistance(dryResistance)
	{
		dMatrix engineMatrix;
		dMatrix chassisMatrix;
		NewtonBodyGetMatrix(engine, &engineMatrix[0][0]);
		NewtonBodyGetMatrix(chassis, &chassisMatrix[0][0]);
		CalculateLocalMatrix(engineMatrix, m_localMatrix0, m_localMatrix1);
		m_baseMatrix = engineMatrix * chassisMatrix.Inverse();
		ResetMatrix();

		m_dryResistance *= 2.0f;
	}

	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		dMatrix matrix0;
		dMatrix matrix1;

		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		CalculateGlobalMatrix(matrix0, matrix1);

		// Restrict the movement on the pivot point along all tree orthonormal direction
		NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_front[0]);
		NewtonUserJointSetRowAcceleration (m_joint, NewtonUserCalculateRowZeroAccelaration(m_joint));
		NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_up[0]);
		NewtonUserJointSetRowAcceleration (m_joint, NewtonUserCalculateRowZeroAccelaration(m_joint));
		NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_right[0]);
		NewtonUserJointSetRowAcceleration (m_joint, NewtonUserCalculateRowZeroAccelaration(m_joint));

		// construct an orthogonal coordinate system with these two vectors
		dMatrix matrix1_1;
		matrix1_1.m_up = matrix1.m_up;
		matrix1_1.m_right = matrix0.m_front * matrix1.m_up;
		matrix1_1.m_right = matrix1_1.m_right.Scale(1.0f / dSqrt(matrix1_1.m_right % matrix1_1.m_right));
		matrix1_1.m_front = matrix1_1.m_up * matrix1_1.m_right;

		dVector omega0;
		dVector omega1;
		NewtonBodyGetOmega(m_body0, &omega0[0]);
		NewtonBodyGetOmega(m_body1, &omega1[0]);
		dVector relOmega(omega0 - omega1);

//dFloat angle1 = -CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
//NewtonUserJointAddAngularRow(m_joint, -angle1, &matrix1.m_up[0]);

		dFloat angle = -CalculateAngle(matrix0.m_front, matrix1_1.m_front, matrix1_1.m_right);
		dFloat omega = (relOmega % matrix1_1.m_right);
		dFloat alphaError = -(angle + omega * timestep) / (timestep * timestep);

		NewtonUserJointAddAngularRow(m_joint, -angle, &matrix1_1.m_right[0]);
		NewtonUserJointSetRowAcceleration(m_joint, alphaError);
		NewtonUserJointSetRowStiffness(m_joint, 1.0f);


		dFloat longOmega = relOmega % matrix1.m_front;
		if (dAbs(longOmega) < 0.25f) {
			dVector drag(0.7f, 0.7f, 0.7f, 0.0f);
			NewtonBodySetAngularDamping(m_body0, &drag[0]);
		} else {
			dVector drag(0.0f, 0.0f, 0.0f, 0.0f);
			NewtonBodySetAngularDamping(m_body0, &drag[0]);
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_front[0]);
			NewtonUserJointSetRowAcceleration(m_joint, -longOmega / timestep);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_dryResistance);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_dryResistance);
		}

/*
dMatrix m0;
dMatrix m1;
dFloat angle0 = -CalculateAngle(matrix1_1.m_up, matrix0.m_up, matrix1_1.m_front);
dFloat angle1 = CalculateAngle(matrix1.m_front, matrix1_1.m_front, matrix1_1.m_up);
dMatrix pitch(dPitchMatrix(angle0));
dMatrix yaw(dYawMatrix(angle1));
NewtonBodyGetMatrix(m_body0, &m0[0][0]);
NewtonBodyGetMatrix(m_body1, &m1[0][0]);
m0 = pitch * matrix0;
m1 = yaw *  matrix1;
dTrace(("%f %f", angle0, angle1));
*/
	}

	void ResetMatrix()
	{
		dMatrix diffMatrix;
		dMatrix chassisMatrix;

		NewtonBody* const diff = GetBody0(); 
		NewtonBody* const chassis = GetBody1(); 

		NewtonBodyGetMatrix(diff, &diffMatrix[0][0]);
		NewtonBodyGetMatrix(chassis, &chassisMatrix[0][0]);

		dMatrix matrix (m_baseMatrix * chassisMatrix);
		dFloat angle = -CalculateAngle(diffMatrix.m_up, matrix.m_up, matrix.m_front);
		matrix = dPitchMatrix(angle) * matrix;
		NewtonBodySetMatrixNoSleep(diff, &matrix[0][0]);
	}

	dMatrix GetBaseMatrix() const
	{
		dMatrix chassisMatrix;
		NewtonBody* const chassis = GetBody1(); 
		NewtonBodyGetMatrix(chassis, &chassisMatrix[0][0]);
		return m_baseMatrix * chassisMatrix;
	}

	dMatrix m_baseMatrix; 
	dFloat m_dryResistance;
};

class CustomVehicleController::DifferentialSpiderGearJoint: public CustomJoint
{
	public:
	DifferentialSpiderGearJoint(NewtonBody* const tire, NewtonBody* const engine, EngineJoint* const engineJoint)
		:CustomJoint(2, tire, engine)
		,m_engineJoint(engineJoint)
		,m_clutchTorque(D_CUSTOM_LARGE_VALUE)		
		,m_gearGain(0.0f)
		,m_tireTorque(0.0f)
		,m_engineTorque(0.0f)
	{
		dMatrix tireMatrix;
		dMatrix engineMatrix;

		NewtonBodyGetMatrix(tire, &tireMatrix[0][0]);
		NewtonBodyGetMatrix(engine, &engineMatrix[0][0]);

		dMatrix align (dYawMatrix (0.5f * 3.1415925f));
		dMatrix tireMatrixPivot (align * engineMatrix);
		tireMatrixPivot.m_posit = tireMatrix.m_posit;
		m_localMatrix0 = tireMatrixPivot * tireMatrix.Inverse();

		tireMatrixPivot.m_posit = engineMatrix.m_posit;
		m_localMatrix1 = tireMatrixPivot * engineMatrix.Inverse();
		m_engineAnkle = dVector (dSign((tireMatrix.m_posit - engineMatrix.m_posit) % engineMatrix[0]), 1.0f, 0.0f, 1.0f);
	}

	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		NewtonBody* const tire = m_body0;
		NewtonBody* const diff = m_body1;

		// Get the global matrices of each rigid body.
		dMatrix tireMatrix;
		dMatrix engineMatrix(m_engineJoint->GetBaseMatrix());
		NewtonBodyGetMatrix(tire, &tireMatrix[0][0]);

		// calculate the pivot points
		dVector tireOmega;
		dVector engineOmega;
		dVector chassisOmega;
		dVector tirePivotVeloc;
		dVector enginePivotVeloc;
		
		NewtonBodyGetOmega(tire, &tireOmega[0]);
		NewtonBodyGetOmega(diff, &engineOmega[0]);
		NewtonBodyGetOmega(m_engineJoint->GetBody1(), &chassisOmega[0]);

		tireOmega -= chassisOmega;
		engineOmega -= chassisOmega;
		
		dVector tirePivot (tireMatrix.m_up.Scale (m_gearGain));
		dVector enginePivot (engineMatrix.RotateVector(m_engineAnkle));

		tirePivotVeloc = tireOmega * tirePivot;
		enginePivotVeloc = engineOmega * enginePivot;

		dVector tireDir (tireMatrix.RotateVector(m_localMatrix0.m_front).Scale (1.0f));
		dVector engineDir (engineMatrix.RotateVector(m_localMatrix1.m_front));

		dVector tire_cross(tirePivot * tireDir);
		dVector engine_cross(enginePivot * engineDir);

		dFloat torque = -NewtonUserJointGetRowForce(m_joint, 0);
		m_engineTorque = engine_cross.Scale (torque) % engineMatrix.m_front;
		m_tireTorque = tire_cross.Scale (torque) % tireMatrix.RotateVector(m_localMatrix0.m_right);

		dFloat jacobian0[6];
		dFloat jacobian1[6];
		jacobian0[0] = tireDir[0];
		jacobian0[1] = tireDir[1];
		jacobian0[2] = tireDir[2];
		jacobian0[3] = tire_cross[0];
		jacobian0[4] = tire_cross[1];
		jacobian0[5] = tire_cross[2];

		jacobian1[0] = engineDir[0];
		jacobian1[1] = engineDir[1];
		jacobian1[2] = engineDir[2];
		jacobian1[3] = engine_cross[0];
		jacobian1[4] = engine_cross[1];
		jacobian1[5] = engine_cross[2];

		dFloat relSpeed = tirePivotVeloc % tireDir + enginePivotVeloc % engineDir;

//dTrace(("(%f  %f %f) ", relSpeed, tirePivotVeloc % tireDir, diffPivotVeloc % diffDir));
//dTrace (("(%f %f)  ", m_diffJoint->GetJointOmega_0(), m_diffJoint->GetJointOmega_1()));

		
		NewtonUserJointAddGeneralRow(m_joint, jacobian0, jacobian1);
		NewtonUserJointSetRowAcceleration(m_joint, -relSpeed / timestep);
		if (m_clutchTorque < (D_CUSTOM_LARGE_VALUE * 0.1f)) {
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_clutchTorque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_clutchTorque);
		}
	}

	void SetGain (dFloat gain)
	{
		m_gearGain = gain;
	}

	void SetClutch(dFloat torque)
	{
		m_clutchTorque = dClamp (torque, dFloat(0.1f), D_CUSTOM_LARGE_VALUE);
	}


	EngineJoint* m_engineJoint;
	dVector m_engineAnkle; 
	dFloat m_clutchTorque;
	dFloat m_gearGain;
	dFloat m_tireTorque;
	dFloat m_engineTorque;
};

CustomVehicleController::BodyPartTire::BodyPartTire()
	:BodyPart()
{
}

CustomVehicleController::BodyPartTire::~BodyPartTire()
{
}

void CustomVehicleController::BodyPartTire::Init (BodyPart* const parentPart, const dMatrix& locationInGlobaSpase, const Info& info)
{
	m_data = info;
	m_parent = parentPart;
	m_userData = info.m_userData;
	m_controller = parentPart->m_controller;

	m_lateralSlip = 0.0f;
	m_aligningTorque = 0.0f;
	m_longitudinalSlip = 0.0f;

	CustomVehicleControllerManager* const manager = (CustomVehicleControllerManager*)m_controller->GetManager();

	NewtonWorld* const world = ((CustomVehicleControllerManager*)m_controller->GetManager())->GetWorld();
	NewtonCollisionSetScale(manager->m_tireShapeTemplate, m_data.m_width, m_data.m_radio, m_data.m_radio);

	// create the rigid body that will make this bone
	dMatrix matrix (dYawMatrix(-0.5f * 3.1415927f) * locationInGlobaSpase);
	m_body = NewtonCreateDynamicBody(world, manager->m_tireShapeTemplate, &matrix[0][0]);
	NewtonCollision* const collision = NewtonBodyGetCollision(m_body);
	NewtonCollisionSetUserData1 (collision, this);
	
	NewtonBodySetMaterialGroupID(m_body, manager->GetTireMaterial());

	dVector drag(0.0f, 0.0f, 0.0f, 0.0f);
	NewtonBodySetLinearDamping(m_body, 0);
	NewtonBodySetAngularDamping(m_body, &drag[0]);
	NewtonBodySetMaxRotationPerStep(m_body, 3.141692f * 0.5f);
	
	// set the standard force and torque call back
	NewtonBodySetForceAndTorqueCallback(m_body, m_controller->m_forceAndTorque);

	// tire are highly non linear, sung spherical inertia matrix make the calculation more accurate 
	dFloat inertia = 2.0f * m_data.m_mass * m_data.m_radio * m_data.m_radio / 5.0f;
	NewtonBodySetMassMatrix (m_body, m_data.m_mass, inertia, inertia, inertia);

	//NewtonBodySetMaterialGroupID (body, m_material);
	//NewtonCollisionSetUserID(collision, definition.m_bodyPartID);
	m_joint = new WheelJoint (matrix, m_body, parentPart->m_body, this);
}

void CustomVehicleController::BodyPartTire::SetSteerAngle (dFloat angle)
{
	WheelJoint* const tire = (WheelJoint*)m_joint;
	tire->m_steerAngle = angle;
}

void CustomVehicleController::BodyPartTire::SetBrakeTorque(dFloat torque)
{
	WheelJoint* const tire = (WheelJoint*)m_joint;
	tire->m_brakeTorque = dMax (torque, tire->m_brakeTorque);
}


CustomVehicleController::BodyPartEngine::BodyPartEngine (CustomVehicleController* const controller, const Info& info)
	:BodyPart()
	,m_data(info)
	,m_norminalTorque(0.0f)
	,m_currentGear(2)
	,m_gearTimer(0)
{
	m_parent = &controller->m_chassis;
	m_controller = controller;

	dAssert (m_data.m_leftTire);
	dAssert (m_data.m_rightTire);

	NewtonWorld* const world = ((CustomVehicleControllerManager*)m_controller->GetManager())->GetWorld();

	//NewtonCollision* const collision = NewtonCreateNull(world);
	NewtonCollision* const collision = NewtonCreateSphere(world, 0.1f, 0, NULL);
	//NewtonCollision* const collision = NewtonCreateCylinder(world, 0.2f, 0.2f, 0, NULL);

	dMatrix matrix;
	dMatrix offset (dYawMatrix (-0.5f * 3.14159213f) * m_controller->m_localFrame);
//offset.m_posit.m_y += 1.0f;
//offset.m_posit.m_x -= 2.0f;

	NewtonBodyGetMatrix(m_controller->GetBody(), &matrix[0][0]);
	matrix = offset * matrix;

	m_body = NewtonCreateDynamicBody(world, collision, &matrix[0][0]);
	NewtonDestroyCollision(collision);

	dFloat inertia = 2.0f * m_data.m_mass * m_data.m_radio * m_data.m_radio / 5.0f;
	NewtonBodySetMassMatrix(m_body, m_data.m_mass, inertia, inertia, inertia);
	NewtonBodySetForceAndTorqueCallback(m_body, m_controller->m_forceAndTorque);

	dVector drag(0.0f, 0.0f, 0.0f, 0.0f);
	NewtonBodySetLinearDamping(m_body, 0);
	NewtonBodySetAngularDamping(m_body, &drag[0]);
	NewtonBodySetMaxRotationPerStep(m_body, 3.141692f * 0.5f);

	InitEngineTorqueCurve();

	dAssert(info.m_gearsCount < (int(sizeof (m_data.m_gearRatios) / sizeof (m_data.m_gearRatios[0])) - D_VEHICLE_FIRST_GEAR));
	m_data.m_gearsCount = info.m_gearsCount + D_VEHICLE_FIRST_GEAR;
	m_data.m_gearRatios[D_VEHICLE_NEWTRAL_GEAR] = 0.0f;
	m_data.m_gearRatios[D_VEHICLE_REVERSE_GEAR] = -dAbs(info.m_reverseGearRatio);
	for (int i = 0; i < info.m_gearsCount; i++) {
		m_data.m_gearRatios[i + D_VEHICLE_FIRST_GEAR] = dAbs(info.m_gearRatios[i]);
	}

	EngineJoint* const joint = new EngineJoint(m_body, m_parent->GetBody(), m_data.m_engineIdleDryDrag);
	m_joint = joint;

	m_leftGear = new DifferentialSpiderGearJoint (m_data.m_leftTire->GetBody(), m_body, joint);
	m_rigntGear = new DifferentialSpiderGearJoint (m_data.m_rightTire->GetBody(), m_body, joint);
}

CustomVehicleController::BodyPartEngine::~BodyPartEngine()
{
}


void CustomVehicleController::BodyPartEngine::Info::ConvertToMetricSystem()
{
	const dFloat horsePowerToWatts = 735.5f;
	const dFloat kmhToMetersPerSecunds = 0.278f;
	const dFloat rpmToRadiansPerSecunds = 0.105f;
	const dFloat poundFootToNewtonMeters = 1.356f;

	m_idleTorque *= poundFootToNewtonMeters;
	m_peakTorque *= poundFootToNewtonMeters;
	m_redLineTorque *= poundFootToNewtonMeters;

	m_rpmAtPeakTorque *= rpmToRadiansPerSecunds;
	m_rpmAtPeakHorsePower *= rpmToRadiansPerSecunds;
	m_rpmAtReadLineTorque *= rpmToRadiansPerSecunds;
	m_rpmAtIdleTorque *= rpmToRadiansPerSecunds;
	
	m_peakHorsePower *= horsePowerToWatts;
	m_vehicleTopSpeed *= kmhToMetersPerSecunds;

	m_peakPowerTorque = m_peakHorsePower / m_rpmAtPeakHorsePower;

	//m_idleTorque = m_peakTorque * 0.5f;
	m_peakPowerTorque = m_peakTorque * 0.5f;
	dAssert(m_rpmAtIdleTorque > 0.0f);
	dAssert(m_rpmAtIdleTorque < m_rpmAtPeakHorsePower);
	dAssert(m_rpmAtPeakTorque < m_rpmAtPeakHorsePower);
	dAssert(m_rpmAtPeakHorsePower < m_rpmAtReadLineTorque);


	dAssert(m_idleTorque > 0.0f);
	dAssert(m_peakTorque > m_peakPowerTorque);
	dAssert(m_peakPowerTorque > m_redLineTorque);
	dAssert(m_redLineTorque > 0.0f);
	dAssert((m_peakTorque * m_rpmAtPeakTorque) < m_peakHorsePower);
}


dFloat CustomVehicleController::BodyPartEngine::GetTopGear() const
{
	return m_data.m_gearRatios[m_data.m_gearsCount - 1];
}


void CustomVehicleController::BodyPartEngine::SetTopSpeed()
{
	dAssert(m_data.m_vehicleTopSpeed >= 0.0f);
	dAssert(m_data.m_vehicleTopSpeed < 100.0f);

	const BodyPartTire* const tire  = m_data.m_leftTire;
	dFloat effectiveRadio = tire->m_data.m_radio;

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
	m_data.m_crownGearRatio = effectiveRadio * m_data.m_rpmAtPeakHorsePower / (m_data.m_vehicleTopSpeed * topGearRatio);
}


void CustomVehicleController::BodyPartEngine::InitEngineTorqueCurve()
{
	m_data.ConvertToMetricSystem();
	SetTopSpeed();

	dFloat rpsTable[5];
	dFloat torqueTable[5];

	rpsTable[0] = 0.0f;
	rpsTable[1] = m_data.m_rpmAtIdleTorque;
	rpsTable[2] = m_data.m_rpmAtPeakTorque;
	rpsTable[3] = m_data.m_rpmAtPeakHorsePower;
	rpsTable[4] = m_data.m_rpmAtReadLineTorque;

	torqueTable[0] = m_data.m_idleTorque;
	torqueTable[1] = m_data.m_idleTorque;
	torqueTable[2] = m_data.m_peakTorque;
	torqueTable[3] = m_data.m_peakPowerTorque;
	torqueTable[4] = m_data.m_redLineTorque;

	const int count = sizeof (rpsTable) / sizeof (rpsTable[0]);
	for (int i = 0; i < count; i++) {
		rpsTable[i] /= m_data.m_crownGearRatio;
		torqueTable[i] *= m_data.m_crownGearRatio;
	}

	m_torqueRPMCurve.InitalizeCurve(sizeof (rpsTable) / sizeof (rpsTable[0]), rpsTable, torqueTable);
	m_data.m_engineIdleDryDrag = dMin (torqueTable[0], torqueTable[4]) * 0.5f;

	dFloat W = rpsTable[4];
	dFloat T = m_torqueRPMCurve.GetValue(W) - m_data.m_engineIdleDryDrag;

	m_data.m_engineIdleViscousDrag = T / (W * W);
	if (m_data.m_engineIdleViscousDrag < 1.0e-4f) {
		m_data.m_engineIdleViscousDrag = 1.0e-4f;
	}
}


dFloat CustomVehicleController::BodyPartEngine::GetRedLineRPM() const
{
	return m_data.m_rpmAtReadLineTorque * 9.55f;
}

dFloat CustomVehicleController::BodyPartEngine::GetSpeed() const
{
	dMatrix matrix;
	dVector veloc;

	EngineJoint* const joint = (EngineJoint*)GetJoint();
	NewtonBody* const chassis = joint->GetBody1();

	NewtonBodyGetMatrix(chassis, &matrix[0][0]);
	NewtonBodyGetVelocity(chassis, &veloc[0]);

	matrix = joint->GetMatrix1() * matrix;
	return dAbs(matrix.m_front % veloc);
}

dFloat CustomVehicleController::BodyPartEngine::GetRPM() const
{
	dMatrix matrix;
	dVector omega0;
	dVector omega1;

	EngineJoint* const joint = (EngineJoint*) GetJoint();
	NewtonBody* const engine = joint->GetBody0();
	NewtonBody* const chassis = joint->GetBody1();

	NewtonBodyGetOmega(engine, &omega0[0]);
	NewtonBodyGetOmega(chassis, &omega1[0]);
	NewtonBodyGetMatrix(engine, &matrix[0][0]);

	matrix = joint->GetMatrix0() * matrix;
	dVector omega (omega0 - omega1);

	dFloat speed = (omega % matrix.m_front) * m_data.m_crownGearRatio * 9.55f;
	return speed;
}

void CustomVehicleController::BodyPartEngine::Update(dFloat timestep, dFloat gasVal)
{
	dVector omega;
	dMatrix matrix;

	m_leftGear->SetGain (m_data.m_gearRatios[m_currentGear]);
	m_rigntGear->SetGain (m_data.m_gearRatios[m_currentGear]);

	EngineJoint* const joint = (EngineJoint*)GetJoint();
	NewtonBody* const engine = joint->GetBody0();
	NewtonBodyGetOmega(engine, &omega[0]);
	NewtonBodyGetMatrix(engine, &matrix[0][0]);
	matrix = joint->GetMatrix0() * matrix;

	dFloat W = (omega % matrix.m_front);
	dFloat T = m_torqueRPMCurve.GetValue(W) * gasVal;
	dFloat d = m_data.m_engineIdleViscousDrag;

	m_norminalTorque = T - d * W * W;

	dVector torque (matrix.m_front.Scale (m_norminalTorque));
	NewtonBodyAddTorque(engine, &torque[0]);
}

void CustomVehicleController::BodyPartEngine::UpdateAutomaticGearBox(dFloat timestep)
{
	m_gearTimer --;
	if (m_gearTimer < 0) {
		dVector omega;
		dMatrix matrix;

		EngineJoint* const joint = (EngineJoint*)GetJoint();
		NewtonBody* const engine = joint->GetBody0();
		NewtonBodyGetOmega(engine, &omega[0]);
		NewtonBodyGetMatrix(engine, &matrix[0][0]);
		omega = matrix.UnrotateVector(omega);
		dFloat W = omega.m_x * m_data.m_crownGearRatio;
	
		switch (m_currentGear) 
		{
			case D_VEHICLE_NEWTRAL_GEAR:
			{
				dAssert (0);
				break;
			}

			case D_VEHICLE_REVERSE_GEAR:
			{
				  dAssert(0);
				  break;
			}

			default:
			{
				if (W > m_data.m_rpmAtPeakHorsePower) {
					if (m_currentGear < (m_data.m_gearsCount - 1)) {
						SetGear(m_currentGear + 1);
					}
				} else if (W < m_data.m_rpmAtPeakTorque) {
					if (m_currentGear > D_VEHICLE_FIRST_GEAR) {
						SetGear(m_currentGear - 1);
					}
				}
			}
		}
	}
}


int CustomVehicleController::BodyPartEngine::GetGear() const
{
	return m_currentGear;
}

void CustomVehicleController::BodyPartEngine::SetGear(int gear)
{
	m_gearTimer = 30;
	dFloat oldGain = m_data.m_gearRatios[m_currentGear];
	m_currentGear = dClamp(gear, 0, m_data.m_gearsCount);

	dVector omega;
	dMatrix matrix;

	EngineJoint* const joint = (EngineJoint*)GetJoint();
	NewtonBody* const engine = joint->GetBody0();
	NewtonBodyGetOmega(engine, &omega[0]);
	NewtonBodyGetMatrix(engine, &matrix[0][0]);
	omega = matrix.UnrotateVector(omega);
	omega.m_x *= m_data.m_gearRatios[m_currentGear] / oldGain;
	omega = matrix.RotateVector(omega);
	NewtonBodySetOmega(engine, &omega[0]);
}



CustomVehicleController::SteeringController::SteeringController (CustomVehicleController* const controller, dFloat maxAngle)
	:Controller(controller)
	,m_maxAngle(dAbs (maxAngle))
	,m_akermanWheelBaseWidth(0.0f)
	,m_akermanAxelSeparation(0.0f)
{
}


void CustomVehicleController::SteeringController::CalculateAkermanParameters(
	const BodyPartTire* const rearLeftTire, const BodyPartTire* const rearRightTire,
	const BodyPartTire* const frontLeftTire, const BodyPartTire* const frontRightTire)
{
/*
	const dMatrix& leftRearMatrix = rearLeftTire->GetLocalMatrix();
	const dMatrix& rightRearMatrix = rearRightTire->GetLocalMatrix();
	dVector rearDist(rightRearMatrix.m_posit - leftRearMatrix.m_posit);
	m_akermanWheelBaseWidth = (rearDist % leftRearMatrix.m_front) * 0.5f;

	const dMatrix& frontLeftTireMatrix = frontLeftTire->GetLocalMatrix();
	dVector akermanAxelSeparation(frontLeftTireMatrix.m_posit - leftRearMatrix.m_posit);
	m_akermanAxelSeparation = dAbs(akermanAxelSeparation % frontLeftTireMatrix.m_right);
*/
}

void CustomVehicleController::SteeringController::Update(dFloat timestep)
{
	dFloat angle = m_maxAngle * m_param;
	if ((m_akermanWheelBaseWidth == 0.0f) || (dAbs(angle) < (2.0f * 3.141592f / 180.0f))) {
		for (dList<BodyPartTire*>::dListNode* node = m_tires.GetFirst(); node; node = node->GetNext()) {
			BodyPartTire& tire = *node->GetInfo();
			tire.SetSteerAngle(angle);
		}
	} else {
		dAssert (0);
/*
		dAssert(dAbs(angle) >= (2.0f * 3.141592f / 180.0f));
		dFloat posit = m_akermanAxelSeparation / dTan(dAbs(angle));
		dFloat sign = dSign(angle);
		dFloat leftAngle = sign * dAtan2(m_akermanAxelSeparation, posit + m_akermanWheelBaseWidth);
		dFloat righAngle = sign * dAtan2(m_akermanAxelSeparation, posit - m_akermanWheelBaseWidth);
		for (dList<BodyPartTire*>::dListNode* node = m_steeringTires.GetFirst(); node; node = node->GetNext()) {
			BodyPartTire& tire = *node->GetInfo();
			tire.SetSteerAngle ((sign * tire.m_data.m_l > 0.0f) ? leftAngle : righAngle);
		}
*/	
	}
}


void CustomVehicleController::SteeringController::AddTire (CustomVehicleController::BodyPartTire* const tire)
{
	m_tires.Append(tire);
}


CustomVehicleController::BrakeController::BrakeController(CustomVehicleController* const controller, dFloat maxBrakeTorque)
	:Controller(controller)
	,m_maxTorque(maxBrakeTorque)
{
}

void CustomVehicleController::BrakeController::AddTire(BodyPartTire* const tire)
{
	m_tires.Append(tire);
}

void CustomVehicleController::BrakeController::Update(dFloat timestep)
{
	dFloat torque = m_maxTorque * m_param;
	for (dList<BodyPartTire*>::dListNode* node = m_tires.GetFirst(); node; node = node->GetNext()) {
		BodyPartTire& tire = *node->GetInfo();
		tire.SetBrakeTorque (torque);
	}
}


CustomVehicleController::EngineController::EngineController (CustomVehicleController* const controller, BodyPartEngine* const engine)
	:Controller(controller)
	,m_engine(engine)
	,m_automaticTransmissionMode(true)
{
}

void CustomVehicleController::EngineController::Update(dFloat timestep)
{
	if (m_automaticTransmissionMode) {
		m_engine->UpdateAutomaticGearBox (timestep);
	}
	m_engine->Update (timestep, m_param);
}

bool CustomVehicleController::EngineController::GetTransmissionMode() const
{
	return m_automaticTransmissionMode;
}

void CustomVehicleController::EngineController::SetTransmissionMode(bool mode)
{
	m_automaticTransmissionMode = mode;
}

int CustomVehicleController::EngineController::GetGear() const
{
	return m_engine->GetGear();
}

void CustomVehicleController::EngineController::SetGear(int gear)
{
	return m_engine->SetGear(gear);
}



dFloat CustomVehicleController::EngineController::GetRPM() const
{
	return m_engine->GetRPM();
}

dFloat CustomVehicleController::EngineController::GetRedLineRPM() const
{
	return m_engine->GetRedLineRPM();
}

dFloat CustomVehicleController::EngineController::GetSpeed() const
{
	return m_engine->GetSpeed();
}


CustomVehicleController::ClutchController::ClutchController(CustomVehicleController* const controller, BodyPartEngine* const engine, dFloat maxClutchTorque)
	:Controller(controller)
	,m_engine(engine)
	,m_maxTorque(maxClutchTorque)
{
}

void CustomVehicleController::ClutchController::Update(dFloat timestep)
{
	dFloat torque = m_maxTorque * m_param;
	m_engine->GetLeftGear()->SetClutch(torque);
	m_engine->GetRightGear()->SetClutch(torque);
}

#if 0

void CustomVehicleControllerManager::DrawSchematic (const CustomVehicleController* const controller, dFloat scale) const
{
	controller->DrawSchematic(scale);
}

void CustomVehicleControllerManager::DrawSchematicCallback (const CustomVehicleController* const controller, const char* const partName, dFloat value, int pointCount, const dVector* const lines) const
{
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


CustomVehicleControllerManager::CustomVehicleControllerManager(NewtonWorld* const world, int materialCount, int* const otherMaterials)
	:CustomControllerManager<CustomVehicleController> (world, VEHICLE_PLUGIN_NAME)
	,m_tireMaterial(NewtonMaterialCreateGroupID(world))
{
	// create the normalized size tire shape
	m_tireShapeTemplate = NewtonCreateChamferCylinder(world, 0.5f, 1.0f, 0, NULL);
	m_tireShapeTemplateData = NewtonCollisionDataPointer(m_tireShapeTemplate);

	// create a tire material and associate with the material the vehicle new to collide 
	for (int i = 0; i < materialCount; i ++) {
		NewtonMaterialSetCollisionCallback(world, m_tireMaterial, otherMaterials[i], this, OnTireAABBOverlap, OnTireContactsProcess);
	}
}

CustomVehicleControllerManager::~CustomVehicleControllerManager()
{
	NewtonDestroyCollision(m_tireShapeTemplate);
}

void CustomVehicleControllerManager::DestroyController(CustomVehicleController* const controller)
{
	controller->Cleanup();
	CustomControllerManager<CustomVehicleController>::DestroyController(controller);
}

int CustomVehicleControllerManager::GetTireMaterial() const
{
	return m_tireMaterial;
}



int CustomVehicleControllerManager::OnTireAABBOverlap (const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex)
{
	CustomVehicleControllerManager* const manager = (CustomVehicleControllerManager*) NewtonMaterialGetMaterialPairUserData(material);

	dAssert ((NewtonCollisionDataPointer ((NewtonBodyGetCollision(body0))) == manager->m_tireShapeTemplateData) || (NewtonCollisionDataPointer ((NewtonBodyGetCollision(body1))) == manager->m_tireShapeTemplateData)); 
	const void* const data0 = NewtonCollisionDataPointer (NewtonBodyGetCollision(body0));
	const NewtonBody* const tireBody = (data0 == manager->m_tireShapeTemplateData) ? body0 : body1;
	const NewtonBody* const otherBody = (data0 == manager->m_tireShapeTemplateData) ? body1 : body0;

	NewtonCollision* const collision = NewtonBodyGetCollision(tireBody);
	const CustomVehicleController::BodyPartTire* const tire	= (CustomVehicleController::BodyPartTire*) NewtonCollisionGetUserData1(collision);
	return manager->OnTireAABBOverlap(material, tire, otherBody);
}

void CustomVehicleControllerManager::OnTireContactsProcess (const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
{
	void* const contact = NewtonContactJointGetFirstContact(contactJoint);
	if (contact) {
		NewtonMaterial* const material = NewtonContactGetMaterial(contact);
		CustomVehicleControllerManager* const manager = (CustomVehicleControllerManager*) NewtonMaterialGetMaterialPairUserData(material);

		const NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
		const NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);
		dAssert((NewtonCollisionDataPointer((NewtonBodyGetCollision(body0))) == manager->m_tireShapeTemplateData) || (NewtonCollisionDataPointer((NewtonBodyGetCollision(body1))) == manager->m_tireShapeTemplateData));
		const void* const data0 = NewtonCollisionDataPointer(NewtonBodyGetCollision(body0));
		const NewtonBody* const tireBody = (data0 == manager->m_tireShapeTemplateData) ? body0 : body1;
		const NewtonBody* const otherBody = (data0 == manager->m_tireShapeTemplateData) ? body1 : body0;
		NewtonCollision* const collision = NewtonBodyGetCollision(tireBody);
		CustomVehicleController::BodyPartTire* const tire = (CustomVehicleController::BodyPartTire*) NewtonCollisionGetUserData1(collision);
		manager->OnTireContactsProcess(contactJoint, tire, otherBody, timestep);
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

	m_engine = NULL;
	
	CustomVehicleControllerManager* const manager = (CustomVehicleControllerManager*)GetManager();
	NewtonWorld* const world = manager->GetWorld();

	// set linear and angular drag to zero
	dVector drag(0.0f, 0.0f, 0.0f, 0.0f);
	NewtonBodySetLinearDamping(m_body, 0);
	NewtonBodySetAngularDamping(m_body, &drag[0]);

	// set the standard force and torque call back
	NewtonBodySetForceAndTorqueCallback(body, m_forceAndTorque);

	m_contactFilter = new CustomVehicleControllerTireCollisionFilter(this);

/*
	SetDryRollingFrictionTorque(100.0f / 4.0f);
	SetAerodynamicsDownforceCoefficient(0.5f * dSqrt(gravityVector % gravityVector), 60.0f * 0.447f);
*/
	m_cluthControl = NULL;
	m_brakesControl = NULL;
	m_engineControl = NULL;
	m_handBrakesControl = NULL;
	m_steeringControl = NULL;
	
	m_collisionAggregate = NewtonCollisionAggregateCreate(world);
	NewtonCollisionAggregateSetSelfCollision (m_collisionAggregate, 0);
	NewtonCollisionAggregateAddBody (m_collisionAggregate, m_body);

	m_skeleton = NewtonSkeletonContainerCreate(world, m_body, NULL);

	m_chassis.Init(this, userData);
	m_bodyPartsList.Append(&m_chassis);

#ifdef D_PLOT_ENGINE_CURVE 
	file_xxx = fopen("vehiceLog.csv", "wb");
	fprintf (file_xxx, "eng_rpm, eng_torque, eng_nominalTorque,\n");
#endif
}

void CustomVehicleController::Cleanup()
{
	SetClutch(NULL);
	SetBrakes(NULL);
	SetEngine(NULL);
	SetSteering(NULL);
	SetHandBrakes(NULL);
	SetContactFilter(NULL);

	if (m_engine) {
		delete m_engine;
	}
}


CustomVehicleController::EngineController* CustomVehicleController::GetEngine() const
{
	return m_engineControl;
}

CustomVehicleController::ClutchController* CustomVehicleController::GetClutch() const
{
	return m_cluthControl;
}

CustomVehicleController::SteeringController* CustomVehicleController::GetSteering() const
{
	return m_steeringControl;
}

CustomVehicleController::BrakeController* CustomVehicleController::GetBrakes() const
{
	return m_brakesControl;
}

CustomVehicleController::BrakeController* CustomVehicleController::GetHandBrakes() const
{
	return m_handBrakesControl;
}

void CustomVehicleController::SetEngine(EngineController* const engineControl)
{
	if (m_engineControl) {
		delete m_engineControl;
	}
	m_engineControl = engineControl;
}

void CustomVehicleController::SetClutch(ClutchController* const cluth)
{
	if (m_cluthControl) {
		delete m_cluthControl;
	}
	m_cluthControl = cluth;
}


void CustomVehicleController::SetHandBrakes(BrakeController* const handBrakes)
{
	if (m_handBrakesControl) {
		delete m_handBrakesControl;
	}
	m_handBrakesControl = handBrakes;
}

void CustomVehicleController::SetBrakes(BrakeController* const brakes)
{
	if (m_brakesControl) {
		delete m_brakesControl;
	}
	m_brakesControl = brakes;
}


void CustomVehicleController::SetSteering(SteeringController* const steering)
{
	if (m_steeringControl) {
		delete m_steeringControl;
	}
	m_steeringControl = steering;
}

void CustomVehicleController::SetContactFilter(CustomVehicleControllerTireCollisionFilter* const filter)
{
	if (m_contactFilter) {
		delete m_contactFilter;
	}
	m_contactFilter = filter;
}


dList<CustomVehicleController::BodyPart*>::dListNode* CustomVehicleController::GetFirstBodyPart() const
{
	return m_bodyPartsList.GetFirst();
}

dList<CustomVehicleController::BodyPart*>::dListNode* CustomVehicleController::GetNextBodyPart(dList<BodyPart*>::dListNode* const part) const
{
	return part->GetNext();
}

void CustomVehicleController::SetCenterOfGravity(const dVector& comRelativeToGeomtriCenter)
{
	NewtonBodySetCentreOfMass(m_body, &comRelativeToGeomtriCenter[0]);
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
		WheelJoint* const tireJoint = (WheelJoint*) tire->GetJoint();
		tireJoint->m_restSprunMass = dFloat (5.0f * dFloor (sprungMass[index] / 5.0f + 0.5f));
//		if (m_engine) {
//			tire->CalculateRollingResistance (m_engine->GetTopSpeed());
//		}
		index ++;
	}

//NewtonBodySetMassMatrix (m_body, 0.0f, 0.0f, 0.0f, 0.0f);

	m_finalized = true;
}

bool CustomVehicleController::ControlStateChanged() const
{
	bool inputChanged = (m_steeringControl && m_steeringControl->ParamChanged());
	inputChanged = inputChanged || (m_engineControl && m_engineControl ->ParamChanged());
	inputChanged = inputChanged || (m_brakesControl && m_brakesControl->ParamChanged());
	inputChanged = inputChanged || (m_handBrakesControl && m_handBrakesControl->ParamChanged());
	return inputChanged;
}

CustomVehicleController::BodyPartTire* CustomVehicleController::AddTire(const BodyPartTire::Info& tireInfo)
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
	tire.m_index = m_tireList.GetCount() - 1;

	m_bodyPartsList.Append(&tire);
	NewtonCollisionAggregateAddBody (m_collisionAggregate, tire.GetBody());
	NewtonSkeletonContainerAttachBone (m_skeleton, tire.GetBody(), m_chassis.GetBody());
	return &tireNode->GetInfo();
}


CustomVehicleController::BodyPartEngine* CustomVehicleController::AddEngine (const BodyPartEngine::Info& engineInfo)
{
	if (m_engine) {
		delete m_engine;
	}

	m_engine = new BodyPartEngine(this, engineInfo);

	NewtonCollisionAggregateAddBody(m_collisionAggregate, m_engine->GetBody());
	NewtonSkeletonContainerAttachBone(m_skeleton, m_engine->GetBody(), m_chassis.GetBody());
	return m_engine;
}


void CustomVehicleController::PostUpdate(dFloat timestep, int threadIndex)
{
	if (m_finalized) {
		if (m_engine)  {
			EngineJoint* const joint = (EngineJoint*)m_engine->GetJoint();
			joint->ResetMatrix();
		}
		
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

#ifdef D_PLOT_ENGINE_CURVE 
		dFloat engineOmega = m_engine->GetRPM();
		dFloat tireTorque = m_engine->GetLeftGear()->m_tireTorque + m_engine->GetRightGear()->m_tireTorque;
		dFloat engineTorque = m_engine->GetLeftGear()->m_engineTorque + m_engine->GetRightGear()->m_engineTorque;
		fprintf (file_xxx, "%f, %f, %f,\n", engineOmega, engineTorque, m_engine->GetNominalTorque());
#endif
	}


//static int xxx;
//dTrace (("  %d\n", xxx));
//xxx ++;

}


void CustomVehicleController::PreUpdate(dFloat timestep, int threadIndex)
{
	if (m_finalized) {
		if (m_engineControl) {
			m_engineControl->Update(timestep);
		}

		if (m_cluthControl) {
			m_cluthControl->Update(timestep);
		}

		if (m_steeringControl) {
			m_steeringControl->Update(timestep);
		}

		if (m_brakesControl) {
			m_brakesControl->Update(timestep);
		}

		if (m_handBrakesControl) {
			m_handBrakesControl->Update(timestep);
		}

		if (ControlStateChanged()) {
			NewtonBodySetSleepState(m_body, 0);
		}
	}
}

int CustomVehicleControllerManager::OnTireAABBOverlap(const NewtonMaterial* const material, const CustomVehicleController::BodyPartTire* const tire, const NewtonBody* const otherBody) const
{
	return true;
}

void CustomVehicleControllerManager::OnTireContactsProcess(const NewtonJoint* const contactJoint, CustomVehicleController::BodyPartTire* const tire, const NewtonBody* const otherBody, dFloat timestep)
{
	dAssert ((tire->GetBody() == NewtonJointGetBody0(contactJoint)) || (tire->GetBody() == NewtonJointGetBody1(contactJoint)));

	dMatrix tireMatrix;
	dVector tireOmega;
	dVector tireVeloc;
	
	NewtonBody* const tireBody = tire->GetBody();
	const CustomVehicleController* const controller = tire->GetController();
	CustomVehicleController::WheelJoint* const tireJoint = (CustomVehicleController::WheelJoint*) tire->GetJoint();

	dAssert (tireJoint->GetBody0() == tireBody);
	NewtonBodyGetMatrix(tireBody, &tireMatrix[0][0]);
	tireMatrix = tireJoint->GetMatrix0() * tireMatrix;

	NewtonBodyGetOmega(tireBody, &tireOmega[0]);
	NewtonBodyGetVelocity(tireBody, &tireVeloc[0]);

static int xxx;
xxx ++;
if (xxx > 1100)
xxx *=1;

	tire->m_lateralSlip = 0.0f;
	tire->m_aligningTorque = 0.0f;
	tire->m_longitudinalSlip = 0.0f;

	for (void* contact = NewtonContactJointGetFirstContact(contactJoint); contact; contact = NewtonContactJointGetNextContact(contactJoint, contact)) {
		dVector posit;
		dVector normal;
		NewtonMaterial* const material = NewtonContactGetMaterial(contact);
		NewtonMaterialGetContactPositionAndNormal (material, tireBody, &posit[0], &normal[0]);

		const dVector& lateralPin = tireMatrix.m_front;
		dVector tireAnglePin(normal * lateralPin);
		dFloat pinMag2 = tireAnglePin % tireAnglePin;
		if (pinMag2 > 0.25f) {
			// brush rubber tire friction model
			// project the contact point to the surface of the collision shape
			dVector pointOnPlane(posit - lateralPin.Scale((posit - tireMatrix.m_posit) % lateralPin));
			dVector dp(pointOnPlane - tireMatrix.m_posit);
//			posit = tireMatrix.m_posit + dp.Scale(tire->m_data.m_radio / dSqrt(dp % dp));
			dVector radius (dp.Scale(tire->m_data.m_radio / dSqrt(dp % dp)));

			dVector lateralContactDir;
			dVector longitudinalContactDir;
			NewtonMaterialContactRotateTangentDirections (material, &lateralPin[0]);
			NewtonMaterialGetContactTangentDirections (material, tireBody, &lateralContactDir[0], &longitudinalContactDir[0]);

//			dVector contactPoint(m_contactsPoint[i]);
//			dVector hitBodyPointVelocity;
//			NewtonBody* const hitBody = externalBody->m_newtonBody;
//			NewtonBodyGetPointVelocity(hitBody, &contactPoint[0], &hitBodyPointVelocity[0]);
//			hitBodyPointVelocity.m_w = 0.0f;
//			dVector headingVeloc(tire->m_veloc + hitBodyPointVelocity);
//			headingVeloc -= normal.Scale(headingVeloc % normal);
//			dFloat v = lateralPin % headingVeloc;
//			dFloat u = longitudinalPin % headingVeloc;

			dFloat u = tireVeloc % longitudinalContactDir;
			dFloat u_rel = NewtonMaterialGetContactTangentSpeed (material, 1);
			dFloat v_rel = NewtonMaterialGetContactTangentSpeed (material, 0);

//			dVector contactRotationalVeloc(tireOmega * radius);
//			dFloat Rw = longitudinalContactDir % contactRotationalVeloc;

//			dFloat u_relAbs = dAbs(u_rel);
			dFloat v_relAbs = dAbs(v_rel);
//			dFloat wrAbs = dAbs(Rw);
			dFloat u_abs = dAbs(u);

			// calculate lateral slip angle
			dFloat sideSlipAngle = 0.25f;
			dFloat lateralSpeed = v_rel;
			if (u_abs > (1.0f / 32.0f)) {
				sideSlipAngle = dAtan2(v_relAbs, u_abs);
				dAssert(sideSlipAngle >= 0.0f);
				dAssert(sideSlipAngle <= (3.141592f * 0.5f));

				// max sideSlip = tan(20.0f)
				if (sideSlipAngle > 0.364f) {
					lateralSpeed = v_rel - 0.364f * u_abs * dSign(v_rel);
				}
			} else {
				if (v_rel > (1.0f / 128.0f)) {
					sideSlipAngle = (3.141592f * 90.0f / 180.0f);
				}
			}
			tire->m_lateralSlip = sideSlipAngle * (180.0f / (3.141592f * 90.0f));

			// calculate longitudinal slip ratio 
			dFloat longitudinalSlipRatio = 1.0f;
			if (u_abs > (1.0f / 32.0f)) {
				longitudinalSlipRatio = dClamp(u_rel / u, -1.0f, 1.0f);
			} else {
//				if (wrAbs < 1.0e-4f) {
//					longitudinalSlipRatio = 0.0f;
//				} else {
				longitudinalSlipRatio = dSign(u_rel);
//				}
			}
			tire->m_longitudinalSlip = longitudinalSlipRatio;
//

			// get the normalize tire load
			dFloat tireLoad = tireJoint->GetTireLoad();
			dFloat restTireLoad = tireJoint->GetTireRestLoad();
			dFloat normalizedTireLoad = dClamp(tireLoad / restTireLoad, 0.0f, 4.0f);

			// calculate longitudinal and lateral forces magnitude when no friction Limit (for now ignore camber angle effects)
			dFloat camberEffect = 0.0f;
			dFloat longitudinalStiffness = restTireLoad * tire->m_data.m_longitudialStiffness;
			dFloat lateralStiffness = restTireLoad * tire->m_data.m_lateralStiffness * normalizedTireLoad;
			dFloat Teff = dTan(sideSlipAngle - camberEffect);

			dFloat Fy0 = lateralStiffness * Teff;
			dFloat Fx0 = longitudinalStiffness * longitudinalSlipRatio;

			// for now assume tire/road friction is 1.0
			//dFloat contactGroundFriction = 1.5f;
			dFloat contactGroundFriction = controller->m_contactFilter->GetTireFrictionCoefficient (material, tireBody, otherBody);

			dFloat tireLoadFriction = contactGroundFriction * tireLoad;
			dFloat K = dSqrt(Fx0 * Fx0 + Fy0 * Fy0) / tireLoadFriction;
			dAssert(K >= 0.0f);

			// now use the friction curve approximation 
			// http://www.ricblues.nl/techniek/Technisch%20Specialist%2093430/6%20Remgedrag%20ABS%20weggedrag/Carsim%20-%20remsimulatieprogramma/Handleiding%20carsim.pdf
			// basically it replaces Pajecka equation with a series expansion 
			// f = x - |x| * x / 3 + x * x * x / 27
			// m = x - |x| * x + x * x * x / 3 + x * x * x * x / 27
			dFloat tireForceCoef = dMin(K * (1.0f - K / 3.0f + K * K / 27.0f), 1.0f);


			dFloat nu = 1.0f;
			if (K < 2.0f * 3.141592f) {
				dFloat lateralToLongitudinalRatio = lateralStiffness / longitudinalStiffness;
				nu = 0.5f * (1.0f + lateralToLongitudinalRatio - (1.0f - lateralToLongitudinalRatio) * dCos(0.5f * K));
			}

			dFloat f0 = tireLoadFriction / dSqrt(longitudinalSlipRatio * longitudinalSlipRatio + (nu * Teff) * (nu * Teff));
			dFloat lateralForce = dAbs(nu * Teff * tireForceCoef * f0);
			dFloat longitudinalForce = dAbs(longitudinalSlipRatio * tireForceCoef * f0);

			// ignore the tire alignment torque for now
			dFloat k1 = dMin(K, 3.0f);
			dFloat tireMomentCoef = k1 * (1.0f - k1 + k1 * k1 / 3.0f - k1 * k1 * k1 / 27.0f);
			tire->m_aligningTorque = nu * tire->m_data.m_aligningMomentTrail * Teff * tireMomentCoef * f0;

			NewtonMaterialSetContactElasticity (material, 0.1f);
//			NewtonMaterialSetContactTangentFriction (material, lateralForce, 0);
//			NewtonMaterialSetContactTangentFriction (material, longitudinalForce, 1);

			NewtonMaterialSetContactFrictionCoef(material, 1.0f, 1.0f, 0);
			NewtonMaterialSetContactFrictionCoef(material, 1.0f, 1.0f, 1);

if (tire->m_index == 2){
dTrace (("%f %f %f\n", K, lateralForce, longitudinalForce));
}

		} else {
			NewtonMaterialSetContactFrictionCoef(material, 1.0f, 1.0f, 0);
			NewtonMaterialSetContactFrictionCoef(material, 1.0f, 1.0f, 1);
		}
	}
}
