/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndSkyBox.h"
#include "ndDemoMesh.h"
#include "ndVehicleUI.h"
#include "ndMeshLoader.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndVehicleCommon.h"
#include "ndMakeStaticMap.h"
#include "ndCompoundScene.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"
#include "ndBasicPlayerCapsule.h"

class ndVehicleDectriptorLav25: public ndVehicleDectriptor
{
	public:
	ndVehicleDectriptorLav25()
		:ndVehicleDectriptor("lav_25.fbx")
	{
		m_chassisMass = 2000.0f;
		m_chassisAngularDrag = 0.25f;
		m_comDisplacement = ndVector(0.0f, -0.55f, 0.0f, 0.0f);

		ndFloat32 idleTorquePoundFoot = 250.0f;
		ndFloat32 idleRmp = 600.0f;
		ndFloat32 horsePower = 500.0f;
		ndFloat32 rpm0 = 3000.0f;
		ndFloat32 rpm1 = 4000.0f;
		ndFloat32 horsePowerAtRedLine = 150.0f;
		ndFloat32 redLineRpm = 5000.0f;
		m_engine.Init(idleTorquePoundFoot, idleRmp, 
					  horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);

		m_transmission.m_torqueConverter = 10000.0f;

		m_frontTire.m_mass = 100.0f;
		m_frontTire.m_steeringAngle = 25.0f * ndDegreeToRad;
		m_frontTire.m_springK = 500.0f;
		m_frontTire.m_damperC = 50.0f;
		m_frontTire.m_regularizer = 0.2f;
		m_frontTire.m_lowerStop = -0.05f;
		m_frontTire.m_upperStop = 0.4f;
		m_frontTire.m_verticalOffset = -0.1f;
		m_frontTire.m_brakeTorque = 1000.0f;

		m_rearTire.m_mass = 100.0f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 500.0f;
		m_rearTire.m_damperC = 50.0f;
		m_rearTire.m_regularizer = 0.2f;
		m_rearTire.m_lowerStop = -0.05f;
		m_rearTire.m_upperStop = 0.4f;
		m_rearTire.m_verticalOffset = -0.1f;
		m_rearTire.m_brakeTorque = 2500.0f;

		m_transmission.m_crownGearRatio = 20.0f;
		m_torsionBarType = m_fourWheelAxle;
		m_differentialType = m_eightWheeldrive;
		
		// Get a stock pacejka curve set and modified a litle for dramatic driving
		ndTireFrictionModel::ndPacejkaTireModel lateral;
		ndTireFrictionModel::ndPacejkaTireModel longitudinal;
		m_frontTire.GetPacejkaCurves(ndTireFrictionModel::m_pacejkaTruck, longitudinal, lateral);
		lateral.m_d = 0.3f;

		// override the tire cuves.
		m_rearTire.SetPacejkaCurves(longitudinal, lateral);
		m_frontTire.SetPacejkaCurves(longitudinal, lateral);

		// plot the curve to check it is a value form
		m_frontTire.PlotPacejkaCurves("truckTireModel");
	}
};

class ndVehicleDectriptorTractor : public ndVehicleDectriptor
{
	public:
	ndVehicleDectriptorTractor()
		:ndVehicleDectriptor("tractor.fbx")
	{
		m_comDisplacement = ndVector(0.0f, -0.55f, 0.0f, 0.0f);
		m_chassisMass = 2000.0f;
		m_chassisAngularDrag = 0.25f;

		ndFloat32 idleTorquePoundFoot = 250.0f;
		ndFloat32 idleRmp = 600.0f;
		ndFloat32 horsePower = 500.0f;
		ndFloat32 rpm0 = 3000.0f;
		ndFloat32 rpm1 = 4000.0f;
		ndFloat32 horsePowerAtRedLine = 150.0f;
		ndFloat32 redLineRpm = 5000.0f;
		m_engine.Init(idleTorquePoundFoot, idleRmp,
					  horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);

		m_transmission.m_gearsCount = 2;
		m_transmission.m_crownGearRatio = 20.0f;
		m_transmission.m_reverseRatio = -3.0f;
		m_transmission.m_forwardRatios[0] = 4.0f;
		m_transmission.m_forwardRatios[1] = 3.0f;

		m_frontTire.m_mass = 100.0f;
		m_frontTire.m_steeringAngle = 25.0f * ndDegreeToRad;
		m_frontTire.m_springK = 1000.0f;
		m_frontTire.m_damperC = 25.0f;
		m_frontTire.m_regularizer = 0.01f;
		m_frontTire.m_lowerStop = -0.05f;
		m_frontTire.m_upperStop = 0.4f;
		m_frontTire.m_verticalOffset = -0.1f;
		m_frontTire.m_brakeTorque = 1000.0f;

		m_rearTire.m_mass = 200.0f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 1000.0f;
		m_rearTire.m_damperC = 25.0f;
		m_rearTire.m_regularizer = 0.01f;
		m_rearTire.m_lowerStop = -0.05f;
		m_rearTire.m_upperStop = 0.4f;
		m_rearTire.m_verticalOffset = -0.1f;
		m_rearTire.m_brakeTorque = 2500.0f;

		m_transmission.m_crownGearRatio = 20.0f;
		m_torsionBarType = m_fourWheelAxle;
		m_differentialType = m_fourWheeldrive;

		// Get a stock pacejka curve set and modified a litle for dramatic driving
		ndTireFrictionModel::ndPacejkaTireModel lateral;
		ndTireFrictionModel::ndPacejkaTireModel longitudinal;
		m_frontTire.GetPacejkaCurves(ndTireFrictionModel::m_pacejkaTruck, longitudinal, lateral);
		lateral.m_d = 0.3f;

		// override the tire cuves.
		m_rearTire.SetPacejkaCurves(longitudinal, lateral);
		m_frontTire.SetPacejkaCurves(longitudinal, lateral);

		// plot the curve to check it is a value form
		m_frontTire.PlotPacejkaCurves("truckTireModel");
	}
};

class ndVehicleDectriptorBigRig : public ndVehicleDectriptor
{
	public:
	ndVehicleDectriptorBigRig()
		:ndVehicleDectriptor("truck.fbx")
	{
		m_chassisMass = 2000.0f;
		m_chassisAngularDrag = 0.25f;
		m_comDisplacement = ndVector(0.0f, -0.55f, 0.0f, 0.0f);

		ndFloat32 idleTorquePoundFoot = 250.0f;
		ndFloat32 idleRmp = 600.0f;
		ndFloat32 horsePower = 500.0f;
		ndFloat32 rpm0 = 3000.0f;
		ndFloat32 rpm1 = 4000.0f;
		ndFloat32 horsePowerAtRedLine = 150.0f;
		ndFloat32 redLineRpm = 5000.0f;
		m_engine.Init(idleTorquePoundFoot, idleRmp,
			horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);

		m_transmission.m_torqueConverter = 10000.0f;

		m_frontTire.m_mass = 100.0f;
		m_frontTire.m_verticalOffset = -0.3f;
		m_frontTire.m_steeringAngle = 25.0f * ndDegreeToRad;
		m_frontTire.m_springK = 500.0f;
		m_frontTire.m_damperC = 50.0f;
		m_frontTire.m_regularizer = 0.2f;
		m_frontTire.m_lowerStop = -0.05f;
		m_frontTire.m_upperStop = 0.4f;
		m_frontTire.m_brakeTorque = 1000.0f;

		m_rearTire.m_mass = 100.0f;
		m_rearTire.m_verticalOffset = -0.3f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 500.0f;
		m_rearTire.m_damperC = 50.0f;
		m_rearTire.m_regularizer = 0.2f;
		m_rearTire.m_lowerStop = -0.05f;
		m_rearTire.m_upperStop = 0.4f;
		m_rearTire.m_brakeTorque = 2500.0f;

		m_transmission.m_crownGearRatio = 20.0f;
		m_torsionBarType = m_fourWheelAxle;
		m_differentialType = m_eightWheeldrive;

		// Get a stock pacejka curve set and modified a litle for dramatic driving
		ndTireFrictionModel::ndPacejkaTireModel lateral;
		ndTireFrictionModel::ndPacejkaTireModel longitudinal;
		m_frontTire.GetPacejkaCurves(ndTireFrictionModel::m_pacejkaTruck, longitudinal, lateral);
		lateral.m_d = 0.3f;

		// override the tire cuves.
		m_rearTire.SetPacejkaCurves(longitudinal, lateral);
		m_frontTire.SetPacejkaCurves(longitudinal, lateral);

		// plot the curve to check it is a value form
		m_frontTire.PlotPacejkaCurves("truckTireModel");
	}
};

static ndVehicleDectriptorLav25 lav25Desc;
static ndVehicleDectriptorBigRig bigRigDesc;
static ndVehicleDectriptorTractor tractorDesc;

static ndDemoEntity* LoadVehicleMeshModel(ndDemoEntityManager* const scene, const char* const filename)
{
	ndMeshLoader loader;
	ndDemoEntity* const vehicleEntity = loader.LoadEntity(filename, scene);
	return vehicleEntity;
}

static ndBodyKinematic* MakeChildPart(ndDemoEntityManager* const scene, ndBodyKinematic* const parentBody, const char* const partName, ndFloat32 mass)
{
	//ndDemoEntity* const parentEntity = (ndDemoEntity*)parentBody->GetNotifyCallback()->GetUserData();
	ndVehicleEntityNotify* const notify = (ndVehicleEntityNotify*)parentBody->GetNotifyCallback();
	ndSharedPtr<ndDemoEntity> parentEntity (notify->m_entity);

	ndSharedPtr<ndDemoEntity> vehPart (parentEntity->Find(parentEntity, partName));
	ndShapeInstance* const vehCollision = vehPart->CreateCollisionFromChildren();
	ndSharedPtr<ndShapeInstance> vehCollisionPtr(vehCollision);

	ndBodyKinematic* const vehBody = new ndBodyDynamic();
	const ndMatrix matrix(vehPart->CalculateGlobalMatrix(nullptr));
	vehBody->SetNotifyCallback(new ndDemoEntityNotify(scene, vehPart, parentBody));
	vehBody->SetMatrix(matrix);
	vehBody->SetCollisionShape(*vehCollision);
	vehBody->SetMassMatrix(mass, *vehCollision);
	return vehBody;
}

static ndMultiBodyVehicle* CreateFlatBedTruck(ndDemoEntityManager* const scene, const ndVehicleDectriptor& desc, const ndMatrix& matrix, ndVehicleUI* const vehicleUI)
{
	ndMultiBodyVehicle* const vehicle = new ndMultiBodyVehicle;

	vehicle->SetNotifyCallback(ndSharedPtr<ndModelNotify>(new ndVehicleCommonNotify(desc, vehicle, vehicleUI)));
	ndVehicleCommonNotify* const notifyCallback = (ndVehicleCommonNotify*)*vehicle->GetNotifyCallback();

	ndSharedPtr<ndDemoEntity> rootEntity (LoadVehicleMeshModel(scene, desc.m_name));
	scene->AddEntity(rootEntity);

	ndSharedPtr<ndDemoEntity> chassisEntity(rootEntity->GetChildren().GetFirst()->GetInfo());
	chassisEntity->ResetMatrix(chassisEntity->CalculateGlobalMatrix() * matrix);

	// 1- add chassis to the vehicle model 
	// create the vehicle chassis as a normal rigid body
	const ndVehicleDectriptor& configuration = notifyCallback->m_desc;

	ndSharedPtr<ndBody> chassisBody(notifyCallback->CreateChassis(scene, chassisEntity, configuration.m_chassisMass));
	vehicle->AddChassis(chassisBody);

	ndBodyDynamic* const chassis = vehicle->GetChassis();
	chassis->SetAngularDamping(ndVector(configuration.m_chassisAngularDrag));

	// lower vehicle com;
	ndVector com(chassis->GetCentreOfMass());
	const ndMatrix localFrame(vehicle->GetLocalFrame());
	com += localFrame.m_up.Scale(configuration.m_comDisplacement.m_y);
	com += localFrame.m_front.Scale(configuration.m_comDisplacement.m_x);
	com += localFrame.m_right.Scale(configuration.m_comDisplacement.m_z);
	chassis->SetCentreOfMass(com);

	//2- add all tires
	ndVehicleDectriptor::ndTireDefinition f0_tireConfiguration(configuration.m_frontTire);
	ndVehicleDectriptor::ndTireDefinition f1_tireConfiguration(configuration.m_frontTire);
	ndSharedPtr<ndBody> fl_tire_body(notifyCallback->CreateTireBody(scene, chassis, f0_tireConfiguration, "fl_tire"));
	ndSharedPtr<ndBody> fr_tire_body(notifyCallback->CreateTireBody(scene, chassis, f1_tireConfiguration, "fr_tire"));
	vehicle->AddTire(f0_tireConfiguration, fr_tire_body);
	vehicle->AddTire(f0_tireConfiguration, fl_tire_body);

	ndVehicleDectriptor::ndTireDefinition r0_tireConfiguration(configuration.m_rearTire);
	ndVehicleDectriptor::ndTireDefinition r1_tireConfiguration(configuration.m_rearTire);
	ndVehicleDectriptor::ndTireDefinition r2_tireConfiguration(configuration.m_rearTire);
	ndVehicleDectriptor::ndTireDefinition r3_tireConfiguration(configuration.m_rearTire);
	ndSharedPtr<ndBody> rl_tire0_body(notifyCallback->CreateTireBody(scene, chassis, r0_tireConfiguration, "rl_midle_tire"));
	ndSharedPtr<ndBody> rr_tire0_body(notifyCallback->CreateTireBody(scene, chassis, r1_tireConfiguration, "rr_midle_tire"));
	ndSharedPtr<ndBody> rl_tire1_body(notifyCallback->CreateTireBody(scene, chassis, r2_tireConfiguration, "rl_tire"));
	ndSharedPtr<ndBody> rr_tire1_body(notifyCallback->CreateTireBody(scene, chassis, r3_tireConfiguration, "rr_tire"));

	ndMultiBodyVehicleTireJoint* const rr_tire0 = vehicle->AddTire(r0_tireConfiguration, rr_tire0_body);
	ndMultiBodyVehicleTireJoint* const rl_tire0 = vehicle->AddTire(r1_tireConfiguration, rl_tire0_body);
	ndMultiBodyVehicleTireJoint* const rr_tire1 = vehicle->AddTire(r2_tireConfiguration, rr_tire1_body);
	ndMultiBodyVehicleTireJoint* const rl_tire1 = vehicle->AddTire(r3_tireConfiguration, rl_tire1_body);

	// 3- add the slip differential, this vehicle is rear wheel drive but has four wheels instead of two.
	// we link each wheel pair with one slip differential, 
	// the each differential is linked to one master slip differential
	ndMultiBodyVehicleDifferential* const rearDifferential0 = vehicle->AddDifferential(configuration.m_differentialMass, configuration.m_differentialRadius, rl_tire0, rr_tire0, configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
	ndMultiBodyVehicleDifferential* const rearDifferential1 = vehicle->AddDifferential(configuration.m_differentialMass, configuration.m_differentialRadius, rl_tire1, rr_tire1, configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
	ndMultiBodyVehicleDifferential* const differential = vehicle->AddDifferential(configuration.m_differentialMass, configuration.m_differentialRadius, rearDifferential0, rearDifferential1, configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);

	// 4 - add a motor
	ndMultiBodyVehicleMotor* const motor = vehicle->AddMotor(configuration.m_motorMass, configuration.m_motorRadius);
	motor->SetMaxRpm(configuration.m_engine.GetRedLineRadPerSec() * dRadPerSecToRpm);

	// 5- add the gear box
	ndMultiBodyVehicleGearBox* const gearBox = vehicle->AddGearBox(differential);
	gearBox->SetIdleOmega(configuration.m_engine.GetIdleRadPerSec() * dRadPerSecToRpm);

	notifyCallback->m_currentGear = sizeof(configuration.m_transmission.m_forwardRatios) / sizeof(configuration.m_transmission.m_forwardRatios[0]) + 1;
	return vehicle;
}

static ndMultiBodyVehicle* CreateLav25Vehicle(ndDemoEntityManager* const scene, const ndVehicleDectriptor& desc, const ndMatrix& matrix, ndVehicleUI* const vehicleUI)
{
	class TranspoterController : public ndVehicleCommonNotify
	{
		public:
		TranspoterController(const ndVehicleDectriptor& desc, ndMultiBodyVehicle* const vehicle, ndVehicleUI* const ui)
			:ndVehicleCommonNotify(desc, vehicle, ui)
			,m_turretEffector(nullptr)
		{
			m_cannonHigh = 0.0f;
			m_turretAngle = 0.0f;
		}

		virtual void ApplyInputs(ndFloat32 timestep) override
		{
			ndVehicleCommonNotify::ApplyInputs(timestep);
			if (m_turretEffector && m_isPlayer)
			{
				ndPhysicsWorld* const world = (ndPhysicsWorld*)GetModel()->GetWorld();
				ndDemoEntityManager* const scene = world->GetManager();
				ndFixSizeArray<char, 32> buttons;
				scene->GetJoystickButtons(buttons);

				bool wakeUpVehicle = false;
				if (buttons[2])
				{
					wakeUpVehicle = true;
					m_turretAngle += 5.0e-3f;
				}
				else if (buttons[1])
				{
					wakeUpVehicle = true;
					m_turretAngle -= 5.0e-3f;
				}

				if (buttons[0])
				{
					wakeUpVehicle = true;
					m_cannonHigh -= 2.0e-3f;
				}
				else if (buttons[3])
				{
					wakeUpVehicle = true;
					m_cannonHigh += 2.0e-3f;
				}

				m_cannonHigh = ndClamp(m_cannonHigh, -ndFloat32(0.1f), ndFloat32(0.5f));
				m_turretAngle = ndClamp(m_turretAngle, -ndFloat32(2.0f) * ndPi, ndFloat32(2.0f) * ndPi);

				if (wakeUpVehicle)
				{
					ndMatrix effectorMatrix(ndPitchMatrix(m_turretAngle));
					effectorMatrix.m_posit.m_x = m_cannonHigh;
					m_turretEffector->SetOffsetMatrix(effectorMatrix);
					ndMultiBodyVehicle* const vehicle = (ndMultiBodyVehicle*)GetModel();
					vehicle->GetChassis()->SetSleepState(false);
				}
			}
		}

		ndIk6DofEffector* m_turretEffector;
		ndFloat32 m_cannonHigh;
		ndFloat32 m_turretAngle;
	};

	ndMultiBodyVehicle* const vehicle = new ndMultiBodyVehicle;

	vehicle->SetNotifyCallback(ndSharedPtr<ndModelNotify>(new TranspoterController(desc, vehicle, vehicleUI)));
	TranspoterController* const notifyCallback = (TranspoterController*)*vehicle->GetNotifyCallback();

	ndSharedPtr<ndDemoEntity> rootEntity(LoadVehicleMeshModel(scene, desc.m_name));
	scene->AddEntity(rootEntity);

	ndSharedPtr<ndDemoEntity> chassisEntity(rootEntity->GetChildren().GetFirst()->GetInfo());
	chassisEntity->ResetMatrix(chassisEntity->CalculateGlobalMatrix() * matrix);

	// 1- add chassis to the vehicle model 
	// create the vehicle chassis as a normal rigid body
	const ndVehicleDectriptor& configuration = notifyCallback->m_desc;

	ndSharedPtr<ndBody> chassisBody(notifyCallback->CreateChassis(scene, chassisEntity, configuration.m_chassisMass));
	vehicle->AddChassis(chassisBody);

	ndBodyDynamic* const chassis = vehicle->GetChassis();
	chassis->SetAngularDamping(ndVector(configuration.m_chassisAngularDrag));

	// lower vehicle com;
	ndVector com(chassis->GetCentreOfMass());
	const ndMatrix localFrame(vehicle->GetLocalFrame());
	com += localFrame.m_up.Scale(configuration.m_comDisplacement.m_y);
	com += localFrame.m_front.Scale(configuration.m_comDisplacement.m_x);
	com += localFrame.m_right.Scale(configuration.m_comDisplacement.m_z);
	chassis->SetCentreOfMass(com);

	//2- add all tires
	ndVehicleDectriptor::ndTireDefinition r0_tireConfiguration(configuration.m_rearTire);
	ndVehicleDectriptor::ndTireDefinition r1_tireConfiguration(configuration.m_rearTire);
	ndVehicleDectriptor::ndTireDefinition r2_tireConfiguration(configuration.m_rearTire);
	ndVehicleDectriptor::ndTireDefinition r3_tireConfiguration(configuration.m_rearTire);
	ndSharedPtr<ndBody> rr_tire0_body = notifyCallback->CreateTireBody(scene, chassis, r0_tireConfiguration, "rtire_3");
	ndSharedPtr<ndBody> rl_tire0_body = notifyCallback->CreateTireBody(scene, chassis, r1_tireConfiguration, "ltire_3");
	ndSharedPtr<ndBody> rr_tire1_body = notifyCallback->CreateTireBody(scene, chassis, r2_tireConfiguration, "rtire_2");
	ndSharedPtr<ndBody> rl_tire1_body = notifyCallback->CreateTireBody(scene, chassis, r3_tireConfiguration, "ltire_2");
	ndMultiBodyVehicleTireJoint* const rr_tire0 = vehicle->AddTire(r0_tireConfiguration, rr_tire0_body);
	ndMultiBodyVehicleTireJoint* const rl_tire0 = vehicle->AddTire(r1_tireConfiguration, rl_tire0_body);
	ndMultiBodyVehicleTireJoint* const rr_tire1 = vehicle->AddTire(r2_tireConfiguration, rr_tire1_body);
	ndMultiBodyVehicleTireJoint* const rl_tire1 = vehicle->AddTire(r3_tireConfiguration, rl_tire1_body);

	ndVehicleDectriptor::ndTireDefinition f0_tireConfiguration(configuration.m_frontTire);
	ndVehicleDectriptor::ndTireDefinition f1_tireConfiguration(configuration.m_frontTire);
	ndVehicleDectriptor::ndTireDefinition f2_tireConfiguration(configuration.m_frontTire);
	ndVehicleDectriptor::ndTireDefinition f3_tireConfiguration(configuration.m_frontTire);
	ndSharedPtr<ndBody> fr_tire0_body = notifyCallback->CreateTireBody(scene, chassis, f0_tireConfiguration, "rtire_0");
	ndSharedPtr<ndBody> fl_tire0_body = notifyCallback->CreateTireBody(scene, chassis, f1_tireConfiguration, "ltire_0");
	ndSharedPtr<ndBody> fr_tire1_body = notifyCallback->CreateTireBody(scene, chassis, f2_tireConfiguration, "rtire_1");
	ndSharedPtr<ndBody> fl_tire1_body = notifyCallback->CreateTireBody(scene, chassis, f3_tireConfiguration, "ltire_1");
	ndMultiBodyVehicleTireJoint* const fr_tire0 = vehicle->AddTire(f0_tireConfiguration, fr_tire0_body);
	ndMultiBodyVehicleTireJoint* const fl_tire0 = vehicle->AddTire(f1_tireConfiguration, fl_tire0_body);
	ndMultiBodyVehicleTireJoint* const fr_tire1 = vehicle->AddTire(f2_tireConfiguration, fr_tire1_body);
	ndMultiBodyVehicleTireJoint* const fl_tire1 = vehicle->AddTire(f3_tireConfiguration, fl_tire1_body);

	// 3- add the slip differential, this vehicle is all wheel drive but has eight wheels instead of two.
	// we link each wheel pair with one slip differential. 
	// them each differential is link to one master slip differential, until there is onel one master
	ndMultiBodyVehicleDifferential* const rearDifferential0 = vehicle->AddDifferential(configuration.m_differentialMass, configuration.m_differentialRadius, rl_tire0, rr_tire0, configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
	ndMultiBodyVehicleDifferential* const rearDifferential1 = vehicle->AddDifferential(configuration.m_differentialMass, configuration.m_differentialRadius, rl_tire1, rr_tire1, configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);

	ndMultiBodyVehicleDifferential* const frontDifferential0 = vehicle->AddDifferential(configuration.m_differentialMass, configuration.m_differentialRadius, fl_tire0, fr_tire0, configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
	ndMultiBodyVehicleDifferential* const frontDifferential1 = vehicle->AddDifferential(configuration.m_differentialMass, configuration.m_differentialRadius, fl_tire1, fr_tire1, configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);

	ndMultiBodyVehicleDifferential* const rearDifferential = vehicle->AddDifferential(configuration.m_differentialMass, configuration.m_differentialRadius, rearDifferential0, rearDifferential1, configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
	ndMultiBodyVehicleDifferential* const frontDifferential = vehicle->AddDifferential(configuration.m_differentialMass, configuration.m_differentialRadius, frontDifferential0, frontDifferential1, configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);

	ndMultiBodyVehicleDifferential* const differential = vehicle->AddDifferential(configuration.m_differentialMass, configuration.m_differentialRadius, rearDifferential, frontDifferential, configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);

	// 4 - add a motor
	ndMultiBodyVehicleMotor* const motor = vehicle->AddMotor(configuration.m_motorMass, configuration.m_motorRadius);
	motor->SetMaxRpm(configuration.m_engine.GetRedLineRadPerSec() * dRadPerSecToRpm);
	
	// 5- add the gear box
	ndMultiBodyVehicleGearBox* const gearBox = vehicle->AddGearBox(differential);
	gearBox->SetIdleOmega(configuration.m_engine.GetIdleRadPerSec() * dRadPerSecToRpm);

	// 6 add any extra functionality
	auto BuildTurret = [vehicle, notifyCallback, scene]()
	{
		const ndMatrix localFrame(vehicle->GetLocalFrame());
		ndBodyDynamic* const chassis = vehicle->GetChassis();
		const ndVehicleDectriptor& configuration = notifyCallback->m_desc;

		// turret body
		ndSharedPtr<ndBody>turretBody(MakeChildPart(scene, chassis, "turret", configuration.m_chassisMass * 0.05f));
		const ndMatrix turretMatrix(localFrame * turretBody->GetMatrix());
		ndSharedPtr<ndJointBilateralConstraint> turretHinge(new ndJointHinge(turretMatrix, turretBody->GetAsBodyKinematic(), chassis));
		ndModelArticulation::ndNode* const turretNode = vehicle->AddLimb(vehicle->GetRoot(), turretBody, turretHinge);
		
		// cannon body
		ndSharedPtr<ndBody>canonBody(MakeChildPart(scene, turretBody->GetAsBodyKinematic(), "canon", configuration.m_chassisMass * 0.025f));
		ndMatrix cannonMatrix(localFrame * canonBody->GetMatrix());
		ndSharedPtr<ndJointBilateralConstraint> cannonHinge(new ndJointHinge(cannonMatrix, canonBody->GetAsBodyKinematic(), turretBody->GetAsBodyKinematic()));
		vehicle->AddLimb(turretNode, canonBody, cannonHinge);
		
		// link the effector for controlling the turret
		ndVehicleEntityNotify* const notify = (ndVehicleEntityNotify*)turretBody->GetNotifyCallback();
		ndSharedPtr<ndDemoEntity> turretEntity(notify->m_entity);
		ndSharedPtr<ndDemoEntity> effectorEntity = turretEntity->Find(turretEntity, "effector");
		ndMatrix effectorMatrix(localFrame * effectorEntity->CalculateGlobalMatrix(nullptr));
		effectorMatrix.m_posit = turretBody->GetMatrix().m_posit;
		
		// We need to remember the object in the notify, but for now just added to the model no controls.
		ndIk6DofEffector* const effector = new ndIk6DofEffector(effectorMatrix, effectorMatrix, canonBody->GetAsBodyKinematic(), chassis);
		effector->EnableAxisX(true);
		effector->EnableAxisY(false);
		effector->EnableAxisZ(false);
		effector->EnableRotationAxis(ndIk6DofEffector::m_fixAxis);
		ndSharedPtr<ndJointBilateralConstraint> effectorPtr(effector);
		vehicle->AddCloseLoop(effectorPtr);

		notifyCallback->m_turretEffector = effector;
	};
	BuildTurret();
	
	notifyCallback->m_currentGear = sizeof(configuration.m_transmission.m_forwardRatios) / sizeof(configuration.m_transmission.m_forwardRatios[0]) + 1;
	return vehicle;
}

static ndMultiBodyVehicle* CreateTractor(ndDemoEntityManager* const scene, const ndVehicleDectriptor& desc, const ndMatrix& matrix, ndVehicleUI* const vehicleUI)
{
	class TractorController : public ndVehicleCommonNotify
	{
		public:
		TractorController(const ndVehicleDectriptor& desc, ndMultiBodyVehicle* const vehicle, ndVehicleUI* const ui)
			:ndVehicleCommonNotify(desc, vehicle, ui)
			,m_armHinge(nullptr)
			,m_bucketHinge(nullptr)
		{
			m_armAngle = 0.0f;
			m_bucketAngle = 0.0f;
		}

		virtual void ApplyInputs(ndFloat32 timestep) override
		{
			ndVehicleCommonNotify::ApplyInputs(timestep);

			if (m_isPlayer)
			{
				ndPhysicsWorld* const world = (ndPhysicsWorld*)GetModel()->GetWorld();
				ndDemoEntityManager* const scene = ((ndPhysicsWorld*)world)->GetManager();
				ndFixSizeArray<char, 32> buttons;

				bool wakeUpVehicle = false;
				scene->GetJoystickButtons(buttons);
				if (buttons[0])
				{
					wakeUpVehicle = true;
					m_armAngle = m_armAngle + timestep * 0.25f;
				}
				else if (buttons[3])
				{
					wakeUpVehicle = true;
					m_armAngle = m_armAngle - timestep * 0.25f;
				}

				if (buttons[1])
				{
					wakeUpVehicle = true;
					m_bucketAngle = m_bucketAngle + timestep * 0.5f;
				}
				else if (buttons[2])
				{
					wakeUpVehicle = true;
					m_bucketAngle = m_bucketAngle - timestep * 0.5f;
				}

				m_armAngle = ndClamp(m_armAngle, ndFloat32(-10.0f) * ndDegreeToRad, ndFloat32(45.0f) * ndDegreeToRad);
				m_bucketAngle = ndClamp(m_bucketAngle, ndFloat32(-75.0f) * ndDegreeToRad, ndFloat32(80.0f) * ndDegreeToRad);

				if (wakeUpVehicle)
				{
					m_armHinge->SetTargetAngle(m_armAngle);
					m_bucketHinge->SetTargetAngle(m_bucketAngle);
					ndMultiBodyVehicle* const vehicle = (ndMultiBodyVehicle*)GetModel();
					vehicle->GetChassis()->SetSleepState(false);
				}
			}
		}

		ndJointHinge* m_armHinge;
		ndJointHinge* m_bucketHinge;
		ndFloat32 m_armAngle;
		ndFloat32 m_bucketAngle;
	};

	ndMultiBodyVehicle* const vehicle = new ndMultiBodyVehicle;

	vehicle->SetNotifyCallback(ndSharedPtr<ndModelNotify>(new TractorController(desc, vehicle, vehicleUI)));
	TractorController* const notifyCallback = (TractorController*)*vehicle->GetNotifyCallback();

	ndSharedPtr<ndDemoEntity> rootEntity(LoadVehicleMeshModel(scene, desc.m_name));
	scene->AddEntity(rootEntity);

	ndSharedPtr<ndDemoEntity> chassisEntity(rootEntity->GetChildren().GetFirst()->GetInfo());
	chassisEntity->ResetMatrix(chassisEntity->CalculateGlobalMatrix()* matrix);

	// 1- add chassis to the vehicle model 
	// create the vehicle chassis as a normal rigid body
	const ndVehicleDectriptor& configuration = notifyCallback->m_desc;

	ndSharedPtr<ndBody> chassisBody(notifyCallback->CreateChassis(scene, chassisEntity, configuration.m_chassisMass));
	vehicle->AddChassis(chassisBody);

	ndBodyDynamic* const chassis = vehicle->GetChassis();
	chassis->SetAngularDamping(ndVector(configuration.m_chassisAngularDrag));

	// lower vehicle com;
	ndVector com(chassis->GetCentreOfMass());
	const ndMatrix localFrame(vehicle->GetLocalFrame());
	com += localFrame.m_up.Scale(configuration.m_comDisplacement.m_y);
	com += localFrame.m_front.Scale(configuration.m_comDisplacement.m_x);
	com += localFrame.m_right.Scale(configuration.m_comDisplacement.m_z);
	chassis->SetCentreOfMass(com);

	//2- add all tires
	ndVehicleDectriptor::ndTireDefinition r0_tireConfiguration(configuration.m_rearTire);
	ndVehicleDectriptor::ndTireDefinition r1_tireConfiguration(configuration.m_rearTire);
	ndSharedPtr<ndBody> rr_tire0_body(notifyCallback->CreateTireBody(scene, chassis, r0_tireConfiguration, "rr_tire"));
	ndSharedPtr<ndBody> rl_tire0_body(notifyCallback->CreateTireBody(scene, chassis, r1_tireConfiguration, "rl_tire"));
	ndMultiBodyVehicleTireJoint* const rr_tire0 = vehicle->AddTire(r0_tireConfiguration, rr_tire0_body);
	ndMultiBodyVehicleTireJoint* const rl_tire0 = vehicle->AddTire(r1_tireConfiguration, rl_tire0_body);

	auto MakeFronAxel = [vehicle, chassis, notifyCallback, scene]()
	{
		const ndVehicleDectriptor& configuration = notifyCallback->m_desc;
		ndSharedPtr<ndBody> axleBody(MakeChildPart(scene, chassis, "front_axel", configuration.m_chassisMass * 0.2f));

		// connect the part to the main body with a hinge
		const ndMatrix localFrame(vehicle->GetLocalFrame());
		const ndMatrix hingeFrame(localFrame * axleBody->GetMatrix());
		ndJointHinge* const hinge = new ndJointHinge(hingeFrame, axleBody->GetAsBodyKinematic(), chassis);
		hinge->SetLimitState(true);
		hinge->SetLimits(-15.0f * ndDegreeToRad, 15.0f * ndDegreeToRad);
		ndSharedPtr<ndJointBilateralConstraint> hingePtr(hinge);
		vehicle->AddLimb(vehicle->GetRoot(), axleBody, hingePtr);
		return axleBody;
	};

	ndVehicleDectriptor::ndTireDefinition f0_tireConfiguration(configuration.m_frontTire);
	ndVehicleDectriptor::ndTireDefinition f1_tireConfiguration(configuration.m_frontTire);
	ndSharedPtr<ndBody> frontAxel_body(MakeFronAxel());
	ndSharedPtr<ndBody> fr_tire0_body(notifyCallback->CreateTireBody(scene, frontAxel_body->GetAsBodyDynamic(), f0_tireConfiguration, "fr_tire"));
	ndSharedPtr<ndBody> fl_tire0_body(notifyCallback->CreateTireBody(scene, frontAxel_body->GetAsBodyDynamic(), f1_tireConfiguration, "fl_tire"));
	ndMultiBodyVehicleTireJoint* const fr_tire0 = vehicle->AddAxleTire(f0_tireConfiguration, fr_tire0_body, frontAxel_body);
	ndMultiBodyVehicleTireJoint* const fl_tire0 = vehicle->AddAxleTire(f1_tireConfiguration, fl_tire0_body, frontAxel_body);

	// 3- add 4 x 4 the slip differential
	// this vehicle is rear wheel drive but has four wheels instead of two.
	// we link each wheel pair with one slip differential, 
	// the each differential is linked to one master slip differential
	ndMultiBodyVehicleDifferential* const rearDifferential = vehicle->AddDifferential(configuration.m_differentialMass, configuration.m_differentialRadius, rl_tire0, rr_tire0, configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
	ndMultiBodyVehicleDifferential* const frontDifferential = vehicle->AddDifferential(configuration.m_differentialMass, configuration.m_differentialRadius, fl_tire0, fr_tire0, configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
	ndMultiBodyVehicleDifferential* const differential = vehicle->AddDifferential(configuration.m_differentialMass, configuration.m_differentialRadius, rearDifferential, frontDifferential, configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
	differential->SetSlipOmega(2.0f * differential->GetSlipOmega());

	// 4- add a motor
	ndMultiBodyVehicleMotor* const motor = vehicle->AddMotor(configuration.m_motorMass, configuration.m_motorRadius);
	motor->SetMaxRpm(configuration.m_engine.GetRedLineRadPerSec()* dRadPerSecToRpm);

	// 5- add the gear box
	ndMultiBodyVehicleGearBox* const gearBox = vehicle->AddGearBox(differential);
	gearBox->SetIdleOmega(configuration.m_engine.GetIdleRadPerSec()* dRadPerSecToRpm);

	// add the extra acticulations. bucke and hydraulics.
	auto BuildBucket = [vehicle, notifyCallback, scene]()
	{
		const ndMatrix localFrame(vehicle->GetLocalFrame());
		ndBodyDynamic* const chassis = vehicle->GetChassis();
		const ndVehicleDectriptor& configuration = notifyCallback->m_desc;

		// add main arm
		ndSharedPtr<ndBody>armBody(MakeChildPart(scene, chassis, "arms", configuration.m_chassisMass * 0.05f));
		ndMatrix armMatrix(localFrame* armBody->GetMatrix());
		ndSharedPtr<ndJointBilateralConstraint> armHinge(new ndJointHinge(armMatrix, armBody->GetAsBodyKinematic(), chassis));
		notifyCallback->m_armHinge = (ndJointHinge*)*armHinge;
		notifyCallback->m_armHinge->SetAsSpringDamper(0.01f, 1500.0f, 20.0f);
		ndModelArticulation::ndNode* const armNode = vehicle->AddLimb(vehicle->GetRoot(), armBody, armHinge);

		// add main bucket
		ndSharedPtr<ndBody>frontBucketBody(MakeChildPart(scene, armBody->GetAsBodyKinematic(), "frontBucket", configuration.m_chassisMass * 0.025f));
		ndMatrix frontBucketMatrix(localFrame* frontBucketBody->GetMatrix());
		ndSharedPtr<ndJointBilateralConstraint> bucketHinge(new ndJointHinge(frontBucketMatrix, frontBucketBody->GetAsBodyKinematic(), armBody->GetAsBodyKinematic()));
		notifyCallback->m_bucketHinge = (ndJointHinge*)*bucketHinge;
		notifyCallback->m_bucketHinge->SetAsSpringDamper(0.01f, 1500.0f, 20.0f);
		vehicle->AddLimb(armNode, frontBucketBody, bucketHinge);

		//auto AddHydraulic = [vehicle, notifyCallback, scene](ndBodyDynamic* const parentBody, const char* const name0, const char* const name1, ndBodyKinematic* const attachmentBody, const char* const attachement)
		auto AddHydraulic = [vehicle, notifyCallback, scene](ndBodyDynamic* const parentBody, const char* const name0, const char* const name1, ndBodyKinematic* const, const char* const)
		{
			const ndMatrix localFrame(vehicle->GetLocalFrame());
			const ndVehicleDectriptor& configuration = notifyCallback->m_desc;
			ndSharedPtr<ndBody>baseBody(MakeChildPart(scene, parentBody, name0, configuration.m_chassisMass * 0.01f));
			ndMatrix matrix0(localFrame * baseBody->GetMatrix());
			ndSharedPtr<ndJointBilateralConstraint>hingeJoint(new ndJointHinge(matrix0, baseBody->GetAsBodyKinematic(), parentBody));
			ndModelArticulation::ndNode* const hingeNode = vehicle->AddLimb(vehicle->FindByBody(parentBody), baseBody, hingeJoint);

			ndSharedPtr<ndBody>boomBody(MakeChildPart(scene, baseBody->GetAsBodyKinematic(), name1, configuration.m_chassisMass * 0.01f));
			ndMatrix matrix1(localFrame * boomBody->GetMatrix());
			ndSharedPtr<ndJointBilateralConstraint>boomJoint(new ndJointSlider(matrix1, boomBody->GetAsBodyKinematic(), baseBody->GetAsBodyKinematic()));
			vehicle->AddLimb(hingeNode, boomBody, boomJoint);
			
			ndAssert(0);
			//ndBodyDynamic* const chassis = vehicle->GetChassis();
			//ndDemoEntity* const parentEntity = (ndDemoEntity*)chassis->GetNotifyCallback()->GetUserData();
			//ndDemoEntity* const attachmentNode = parentEntity->Find(attachement);
			//matrix0.m_posit = attachmentNode->CalculateGlobalMatrix().m_posit;
			//
			//ndIk6DofEffector* const attachementJoint = new ndIk6DofEffector(matrix0, matrix0, boomBody->GetAsBodyKinematic(), attachmentBody);
			//attachementJoint->EnableAxisX(false);
			//attachementJoint->EnableAxisY(true);
			//attachementJoint->EnableAxisZ(true);
			//attachementJoint->EnableRotationAxis(ndIk6DofEffector::m_disabled);
			//ndSharedPtr<ndJointBilateralConstraint> attachementJointPtr(attachementJoint);
			//vehicle->AddCloseLoop(attachementJointPtr);
		};

		AddHydraulic(chassis, "armHydraulicPiston_left", "armHydraulic_left", armBody->GetAsBodyKinematic(), "attach0_left");
		AddHydraulic(chassis, "armHydraulicPiston_right", "armHydraulic_right", armBody->GetAsBodyKinematic(), "attach0_right");
		AddHydraulic(armBody->GetAsBodyDynamic(), "frontBucketHydraulic001", "frontBucketHydraulicPiston001", frontBucketBody->GetAsBodyKinematic(), "attachment_frontBucket001");
		AddHydraulic(armBody->GetAsBodyDynamic(), "frontBucketHydraulic002", "frontBucketHydraulicPiston002", frontBucketBody->GetAsBodyKinematic(), "attachment_frontBucket002");
	};
	BuildBucket();

	notifyCallback->m_currentGear = sizeof(configuration.m_transmission.m_forwardRatios) / sizeof(configuration.m_transmission.m_forwardRatios[0]) + 1;
	return vehicle;
}

void ndHeavyVehicle (ndDemoEntityManager* const scene)
{
	ndMatrix sceneLocation(ndGetIdentityMatrix());
	//BuildFloorBox(scene, sceneLocation);
	//BuildFlatPlane(scene, true);
	//BuildGridPlane(scene, 120, 4.0f, 0.0f);
	//BuildStaticMesh(scene, "track.fbx", true);
	BuildCompoundScene(scene, sceneLocation);
	//BuildStaticMesh(scene, "playerarena.fbx", true);
	//BuildSplineTrack(scene, "playerarena.fbx", true);
	//sceneLocation.m_posit.m_x = -200.0f;
	//sceneLocation.m_posit.m_z = -200.0f;
	//BuildHeightFieldTerrain(scene, sceneLocation);
	
	ndVehicleMaterial material;
	material.m_restitution = 0.1f;
	material.m_staticFriction0 = 0.8f;
	material.m_staticFriction1 = 0.8f;
	material.m_dynamicFriction0 = 0.8f;
	material.m_dynamicFriction1 = 0.8f;

	ndContactCallback* const callback = (ndContactCallback*)scene->GetWorld()->GetContactNotify();
	callback->RegisterMaterial(material, ndDemoContactCallback::m_modelPart, ndDemoContactCallback::m_default);
	callback->RegisterMaterial(material, ndDemoContactCallback::m_modelPart, ndDemoContactCallback::m_modelPart);
	callback->RegisterMaterial(material, ndDemoContactCallback::m_vehicleTirePart, ndDemoContactCallback::m_default);
	callback->RegisterMaterial(material, ndDemoContactCallback::m_vehicleTirePart, ndDemoContactCallback::m_modelPart);
	callback->RegisterMaterial(material, ndDemoContactCallback::m_vehicleTirePart, ndDemoContactCallback::m_vehicleTirePart);

	ndPhysicsWorld* const world = scene->GetWorld();
	ndVector location(0.0f, 2.0f, 0.0f, 1.0f);
	
	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit = location;

	// add a model for general controls
	ndSharedPtr<ndModel> controls(new ndVehicleSelector());
	world->AddModel(controls);

	ndSharedPtr<ndUIEntity> vehicleUI(new ndVehicleUI(scene));
	scene->Set2DDisplayRenderFunction(vehicleUI);
	
	ndSharedPtr<ndModel> vehicle0(CreateFlatBedTruck(scene, bigRigDesc, matrix, (ndVehicleUI*)*vehicleUI));
	
	matrix.m_posit.m_x += 6.0f;
	matrix.m_posit.m_z += 6.0f;
	ndSharedPtr<ndModel> vehicle1(CreateLav25Vehicle(scene, lav25Desc, matrix, (ndVehicleUI*)*vehicleUI));

	matrix.m_posit.m_z -= 12.0f;
	//ndSharedPtr<ndModel> vehicle2(CreateTractor(scene, tractorDesc, matrix, (ndVehicleUI*)*vehicleUI));

	world->AddModel(vehicle0);
	vehicle0->AddBodiesAndJointsToWorld();

	world->AddModel(vehicle1);
	vehicle1->AddBodiesAndJointsToWorld();

	//world->AddModel(vehicle2);
	//vehicle2->AddBodiesAndJointsToWorld();

	ndVehicleCommonNotify* const notifyCallback = (ndVehicleCommonNotify*)*vehicle0->GetNotifyCallback();
	notifyCallback->SetAsPlayer(scene);
	
	matrix.m_posit.m_x += 25.0f;
	matrix.m_posit.m_z += 6.0f;
	AddPlanks(scene, matrix, 1000.0f, 5);
	
	ndQuaternion rot;
	ndVector origin(-10.0f, 2.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}
