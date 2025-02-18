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
#include "ndSoundManager.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndCompoundScene.h"
#include "ndVehicleCommon.h"
#include "ndMakeStaticMap.h"
#include "ndContactCallback.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"
#include "ndBasicPlayerCapsule.h"
#include "ndHeightFieldPrimitive.h"

class ndVehicleDectriptorViper : public ndVehicleDectriptor
{
	public:
	ndVehicleDectriptorViper()
		:ndVehicleDectriptor("viper.fbx")
	{
		m_useHardSolverMode = true;
		//m_useHardSolverMode = false;
		m_comDisplacement = ndVector(0.25f, -0.35f, 0.0f, 0.0f);

		ndFloat32 idleTorquePoundFoot = 300.0f;
		ndFloat32 idleRmp = 700.0f;
		ndFloat32 horsePower = 400.0f;
		ndFloat32 rpm0 = 5000.0f;
		ndFloat32 rpm1 = 6200.0f;
		ndFloat32 horsePowerAtRedLine = 100.0f;
		ndFloat32 redLineRpm = 8000.0f;
		m_engine.Init(idleTorquePoundFoot, idleRmp, 
					  horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);

		m_frontTire.m_mass = 25.0f;
		m_frontTire.m_handBrakeTorque = 0.0f;
		m_frontTire.m_frictionModel = ndTireFrictionModel::m_brushModel;
		m_frontTire.m_longitudinalStiffness = 10.0f * DEMO_GRAVITY;
		m_frontTire.m_laterialStiffness = 2.0f * m_frontTire.m_longitudinalStiffness;

		m_rearTire.m_mass = 25.0f;
		m_rearTire.m_handBrakeTorque = 100000.0f;
		m_rearTire.m_frictionModel = ndTireFrictionModel::m_brushModel;
		m_rearTire.m_longitudinalStiffness = 10.0f * DEMO_GRAVITY;
		m_rearTire.m_laterialStiffness = 2.0f * m_rearTire.m_longitudinalStiffness;
	}
};

class ndVehicleDectriptorJeep : public ndVehicleDectriptor
{
	public:
	ndVehicleDectriptorJeep()
		:ndVehicleDectriptor("jeep.fbx")
	{
		//m_useHardSolverMode = true;
		m_useHardSolverMode = false;
		m_comDisplacement = ndVector(0.0f, -0.50f, 0.0f, 0.0f);

		ndFloat32 idleTorquePoundFoot = 200.0f;
		ndFloat32 idleRmp = 800.0f;
		ndFloat32 horsePower = 400.0f;
		ndFloat32 rpm0 = 5000.0f;
		ndFloat32 rpm1 = 6200.0f;
		ndFloat32 horsePowerAtRedLine = 400.0f;
		ndFloat32 redLineRpm = 8000.0f;
		m_engine.Init(idleTorquePoundFoot, idleRmp, 
					  horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);

		m_frontTire.m_mass = 25.0f;
		m_frontTire.m_verticalOffset = -0.15f;
		m_frontTire.m_steeringAngle = 35.0f * ndDegreeToRad;
		m_frontTire.m_springK = 800.0f;
		m_frontTire.m_damperC = 50.0f;
		m_frontTire.m_regularizer = 0.1f;
		m_frontTire.m_lowerStop = -0.05f;
		m_frontTire.m_upperStop = 0.4f;
		m_frontTire.m_brakeTorque = 1500.0f;
		m_frontTire.m_handBrakeTorque = 0.0f;

		m_rearTire.m_mass = 25.0f;
		m_rearTire.m_verticalOffset = -0.15f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 800.0f;
		m_rearTire.m_damperC = 50.0f;
		m_rearTire.m_regularizer = 0.1f;
		m_rearTire.m_lowerStop = -0.05f;
		m_rearTire.m_upperStop = 0.4f;
		m_rearTire.m_brakeTorque = 3000.0f;
		m_rearTire.m_handBrakeTorque = 100000.0f;
		m_torsionBarType = m_fourWheelAxle;
		m_differentialType = m_fourWheeldrive;

		m_rearTire.m_frictionModel = ndTireFrictionModel::m_brushModel;
		m_frontTire.m_frictionModel = ndTireFrictionModel::m_brushModel;
		m_rearTire.m_longitudinalStiffness = 10.0f * DEMO_GRAVITY;
		m_frontTire.m_longitudinalStiffness = 10.0f * DEMO_GRAVITY;
		m_rearTire.m_laterialStiffness = 2.0f * m_rearTire.m_longitudinalStiffness;
		m_frontTire.m_laterialStiffness = 2.0f * m_frontTire.m_longitudinalStiffness;
	}
};

class ndVehicleDectriptorMonsterTruck0: public ndVehicleDectriptor
{
	public:
	ndVehicleDectriptorMonsterTruck0()
		:ndVehicleDectriptor("monsterTruck0.fbx")
	{
		m_comDisplacement = ndVector(0.0f, -0.7f, 0.0f, 0.0f);

		ndFloat32 idleTorquePoundFoot = 250.0f;
		ndFloat32 idleRmp = 800.0f;
		ndFloat32 horsePower = 400.0f;
		ndFloat32 rpm0 = 5000.0f;
		ndFloat32 rpm1 = 6200.0f;
		ndFloat32 horsePowerAtRedLine = 150.0f;
		ndFloat32 redLineRpm = 8000.0f;
		m_engine.Init(idleTorquePoundFoot, idleRmp, 
					  horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);

		m_frontTire.m_mass = 100.0f;
		m_frontTire.m_verticalOffset = 0.0f;
		m_frontTire.m_steeringAngle = 35.0f * ndDegreeToRad;
		m_frontTire.m_springK = 500.0f;
		m_frontTire.m_damperC = 50.0f;
		m_frontTire.m_regularizer = 0.2f;
		m_frontTire.m_lowerStop = -0.05f;
		m_frontTire.m_upperStop = 0.4f;
		m_frontTire.m_brakeTorque = 1000.0f;
		m_frontTire.m_handBrakeTorque = 0.0f;

		m_rearTire.m_mass = 100.0f;
		m_rearTire.m_verticalOffset = 0.0f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 500.0f;
		m_rearTire.m_damperC = 50.0f;
		m_rearTire.m_regularizer = 0.2f;
		m_rearTire.m_lowerStop = -0.05f;
		m_rearTire.m_upperStop = 0.4f;
		m_rearTire.m_brakeTorque = 5000.0f;
		m_rearTire.m_handBrakeTorque = 1000000.0f;
		m_torsionBarType = m_fourWheelAxle;
		m_differentialType = m_fourWheeldrive;

		m_rearTire.m_frictionModel = ndTireFrictionModel::m_brushModel;
		m_frontTire.m_frictionModel = ndTireFrictionModel::m_brushModel;
		m_rearTire.m_longitudinalStiffness = 10.0f * DEMO_GRAVITY;
		m_frontTire.m_longitudinalStiffness = 10.0f * DEMO_GRAVITY;
		m_rearTire.m_laterialStiffness = 2.0f * m_rearTire.m_longitudinalStiffness;
		m_frontTire.m_laterialStiffness = 2.0f * m_frontTire.m_longitudinalStiffness;
	}
};

class ndVehicleDectriptorMonsterTruck1 : public ndVehicleDectriptor
{
	public:
	ndVehicleDectriptorMonsterTruck1()
		:ndVehicleDectriptor("monsterTruck1.fbx")
	{
		m_comDisplacement = ndVector(0.0f, -0.7f, 0.0f, 0.0f);

		// reset gear box ratios
		m_transmission.m_gearsCount = 4;
		m_transmission.m_forwardRatios[0] = 2.5f;
		m_transmission.m_forwardRatios[1] = 1.5f;
		m_transmission.m_forwardRatios[2] = 1.1f;
		m_transmission.m_forwardRatios[3] = 0.8f;
		m_transmission.m_crownGearRatio = 20.0f;

		ndFloat32 idleTorquePoundFoot = 300.0f;
		ndFloat32 idleRmp = 800.0f;
		ndFloat32 horsePower = 600.0f;
		ndFloat32 rpm0 = 5000.0f;
		ndFloat32 rpm1 = 6200.0f;
		ndFloat32 horsePowerAtRedLine = 150.0f;
		ndFloat32 redLineRpm = 8000.0f;
		m_engine.Init(idleTorquePoundFoot, idleRmp,
			horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);

		m_frontTire.m_mass = 100.0f;
		m_frontTire.m_verticalOffset = 0.0f;
		m_frontTire.m_steeringAngle = 35.0f * ndDegreeToRad;
		m_frontTire.m_springK = 500.0f;
		m_frontTire.m_damperC = 50.0f;
		m_frontTire.m_regularizer = 0.2f;
		m_frontTire.m_lowerStop = -0.05f;
		m_frontTire.m_upperStop = 0.4f;
		m_frontTire.m_brakeTorque = 10000.0f;
		m_frontTire.m_handBrakeTorque = 0.0f;

		m_rearTire.m_mass = 100.0f;
		m_rearTire.m_verticalOffset = 0.0f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 500.0f;
		m_rearTire.m_damperC = 50.0f;
		m_rearTire.m_regularizer = 0.2f;
		m_rearTire.m_lowerStop = -0.05f;
		m_rearTire.m_upperStop = 0.4f;
		m_rearTire.m_brakeTorque = 10000.0f;
		m_rearTire.m_handBrakeTorque = 50000.0f;
		m_torsionBarType = m_fourWheelAxle;
		m_differentialType = m_fourWheeldrive;

		m_rearTire.m_frictionModel = ndTireFrictionModel::m_brushModel;
		m_frontTire.m_frictionModel = ndTireFrictionModel::m_brushModel;
		m_rearTire.m_longitudinalStiffness = 10.0f * DEMO_GRAVITY;
		m_frontTire.m_longitudinalStiffness = 10.0f * DEMO_GRAVITY;
		m_rearTire.m_laterialStiffness = 2.0f * m_rearTire.m_longitudinalStiffness;
		m_frontTire.m_laterialStiffness = 2.0f * m_frontTire.m_longitudinalStiffness;
	}
};

static ndVehicleDectriptorJeep jeepDesc;
static ndVehicleDectriptorViper viperDesc;
static ndVehicleDectriptorMonsterTruck0 monterTruckDesc0;
static ndVehicleDectriptorMonsterTruck1 monterTruckDesc1;

static const char* engineSounds[] =
{
	"engine_start.wav",
	"engine_rpm.wav",
	"tire_skid.wav",
};

static ndDemoEntity* LoadVehicleMeshModel(ndDemoEntityManager* const scene, const char* const filename)
{
	ndMeshLoader loader;
	ndDemoEntity* const vehicleEntity = loader.LoadEntity(filename, scene);
	scene->AddEntity(vehicleEntity);
	return vehicleEntity;
}

static ndMultiBodyVehicle* CreateBasicVehicle(ndDemoEntityManager* const scene, const ndVehicleDectriptor& desc, const ndMatrix& matrix, ndVehicleUI* const vehicleUI)
{
	ndMultiBodyVehicle* const vehicle = new ndMultiBodyVehicle;

	vehicle->SetNotifyCallback(ndSharedPtr<ndModelNotify>(new ndVehicleCommonNotify(desc, vehicle, vehicleUI)));
	ndVehicleCommonNotify* const notifyCallback = (ndVehicleCommonNotify*)*vehicle->GetNotifyCallback();

	ndDemoEntity* const vehicleEntityDummyRoot = LoadVehicleMeshModel(scene, desc.m_name);

	ndDemoEntity* const vehicleEntity = vehicleEntityDummyRoot->GetFirstChild();
	vehicleEntity->ResetMatrix(vehicleEntity->CalculateGlobalMatrix() * matrix);

	// 1- add chassis to the vehicle model
	// create the vehicle chassis as a normal rigid body
	const ndVehicleDectriptor& configuration = notifyCallback->m_desc;
	vehicle->AddChassis(ndSharedPtr<ndBody>(notifyCallback->CreateChassis(scene, vehicleEntity, configuration.m_chassisMass)));

	ndBodyDynamic* const chassis = vehicle->GetChassis();
	chassis->SetAngularDamping(ndVector(configuration.m_chassisAngularDrag));

	// lower vehicle com;
	ndVector com(chassis->GetCentreOfMass());
	const ndMatrix localFrame(vehicle->GetLocalFrame());
	com += localFrame.m_up.Scale(configuration.m_comDisplacement.m_y);
	com += localFrame.m_front.Scale(configuration.m_comDisplacement.m_x);
	com += localFrame.m_right.Scale(configuration.m_comDisplacement.m_z);
	chassis->SetCentreOfMass(com);

	// 2- each tire
	// create the tire as a normal rigid body
	// and attach them to the chassis with a tire joints
	ndVehicleDectriptor::ndTireDefinition rr_tireConfiguration(configuration.m_rearTire);
	ndVehicleDectriptor::ndTireDefinition rl_tireConfiguration(configuration.m_rearTire);
	ndSharedPtr<ndBody> rr_tire_body(notifyCallback->CreateTireBody(scene, chassis, rr_tireConfiguration, "rr_tire"));
	ndSharedPtr<ndBody> rl_tire_body(notifyCallback->CreateTireBody(scene, chassis, rl_tireConfiguration, "rl_tire"));
	ndMultiBodyVehicleTireJoint* const rr_tire = vehicle->AddTire(rr_tireConfiguration, rr_tire_body);
	ndMultiBodyVehicleTireJoint* const rl_tire = vehicle->AddTire(rl_tireConfiguration, rl_tire_body);

	ndVehicleDectriptor::ndTireDefinition fr_tireConfiguration(configuration.m_frontTire);
	ndVehicleDectriptor::ndTireDefinition fl_tireConfiguration(configuration.m_frontTire);
	ndSharedPtr<ndBody> fr_tire_body(notifyCallback->CreateTireBody(scene, chassis, fr_tireConfiguration, "fr_tire"));
	ndSharedPtr<ndBody> fl_tire_body(notifyCallback->CreateTireBody(scene, chassis, fl_tireConfiguration, "fl_tire"));
	ndMultiBodyVehicleTireJoint* const fr_tire = vehicle->AddTire(fr_tireConfiguration, fr_tire_body);
	ndMultiBodyVehicleTireJoint* const fl_tire = vehicle->AddTire(fl_tireConfiguration, fl_tire_body);

	notifyCallback->m_currentGear = sizeof(configuration.m_transmission.m_forwardRatios) / sizeof(configuration.m_transmission.m_forwardRatios[0]) + 1;

	// 3- add differential
	// add the slip differential
	ndMultiBodyVehicleDifferential* differential = nullptr;
	switch (configuration.m_differentialType)
	{
		case ndVehicleDectriptor::m_rearWheelDrive:
		{
			differential = vehicle->AddDifferential(configuration.m_differentialMass, configuration.m_differentialRadius, rl_tire, rr_tire, configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
			break;
		}

		case ndVehicleDectriptor::m_frontWheelDrive:
		{
			differential = vehicle->AddDifferential(configuration.m_differentialMass, configuration.m_differentialRadius, fl_tire, fr_tire, configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
			break;
		}

		case ndVehicleDectriptor::m_fourWheeldrive:
		{
			ndMultiBodyVehicleDifferential* const rearDifferential = vehicle->AddDifferential(configuration.m_differentialMass, configuration.m_differentialRadius, rl_tire, rr_tire, configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
			ndMultiBodyVehicleDifferential* const frontDifferential = vehicle->AddDifferential(configuration.m_differentialMass, configuration.m_differentialRadius, fl_tire, fr_tire, configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
			differential = vehicle->AddDifferential(configuration.m_differentialMass, configuration.m_differentialRadius, rearDifferential, frontDifferential, configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
			break;
		}

		case ndVehicleDectriptor::m_eightWheeldrive:
		{
			ndAssert(0);
			break;
		}
	}

	// 4- add a motor
	ndMultiBodyVehicleMotor* const motor = vehicle->AddMotor(configuration.m_motorMass, configuration.m_motorRadius);
	motor->SetMaxRpm(configuration.m_engine.GetRedLineRadPerSec() * dRadPerSecToRpm);
	motor->SetFrictionLoss(configuration.m_engine.GetTorque(0.0f) * 0.5f);

	// 5- add the gear box
	ndMultiBodyVehicleGearBox* const gearBox = vehicle->AddGearBox(differential);
	gearBox->SetIdleOmega(configuration.m_engine.GetIdleRadPerSec() * dRadPerSecToRpm);

	//switch (configuration.m_torsionBarType)
	//{
	//	case ndVehicleDectriptor::m_noWheelAxle:
	//	{
	//		// no torsion bar
	//		break;
	//	}
	//
	//	case ndVehicleDectriptor::m_rearWheelAxle:
	//	{
	//		ndMultiBodyVehicleTorsionBar* const torsionBar = AddTorsionBar(world->GetSentinelBody());
	//		torsionBar->AddAxel(rl_tire->GetBody0(), rr_tire->GetBody0());
	//		torsionBar->SetTorsionTorque(configuration.m_torsionBarSpringK, configuration.m_torsionBarDamperC, configuration.m_torsionBarRegularizer);
	//		break;
	//	}
	//
	//	case ndVehicleDectriptor::m_frontWheelAxle:
	//	{
	//		ndMultiBodyVehicleTorsionBar* const torsionBar = AddTorsionBar(world->GetSentinelBody());
	//		torsionBar->AddAxel(fl_tire->GetBody0(), fr_tire->GetBody0());
	//		torsionBar->SetTorsionTorque(configuration.m_torsionBarSpringK, configuration.m_torsionBarDamperC, configuration.m_torsionBarRegularizer);
	//		break;
	//	}
	//
	//	case ndVehicleDectriptor::m_fourWheelAxle:
	//	{
	//		ndMultiBodyVehicleTorsionBar* const torsionBar = AddTorsionBar(world->GetSentinelBody());
	//		torsionBar->AddAxel(rl_tire->GetBody0(), rr_tire->GetBody0());
	//		torsionBar->AddAxel(fl_tire->GetBody0(), fr_tire->GetBody0());
	//		torsionBar->SetTorsionTorque(configuration.m_torsionBarSpringK, configuration.m_torsionBarDamperC, configuration.m_torsionBarRegularizer);
	//		break;
	//	}
	//}

	// set a soft or hard mode
	vehicle->SetVehicleSolverModel(configuration.m_useHardSolverMode ? true : false);
	return vehicle;
}

//static void TestPlayerCapsuleInteraction(ndDemoEntityManager* const scene, const ndMatrix& location)
static void TestPlayerCapsuleInteraction(ndDemoEntityManager* const, const ndMatrix&)
{
	//ndMatrix localAxis(ndGetIdentityMatrix());
	//localAxis[0] = ndVector(0.0, 1.0f, 0.0f, 0.0f);
	//localAxis[1] = ndVector(1.0, 0.0f, 0.0f, 0.0f);
	//localAxis[2] = localAxis[0].CrossProduct(localAxis[1]);
	//
	//ndFloat32 height = 1.9f;
	//ndFloat32 radio = 0.5f;
	//ndFloat32 mass = 100.0f;
	//ndDemoEntity* const entity = ndDemoEntity::LoadFbx("walker.fbx", scene);
	//new ndBasicPlayerCapsule(scene, entity, localAxis, location, mass, radio, height, height / 4.0f);
	//delete entity;
}

class ndPlacementMatrix : public ndMatrix
{
	public:
	ndPlacementMatrix(const ndMatrix base, const ndVector& offset)
		:ndMatrix (base)
	{
		m_posit += offset;
	}
};

void ndBasicVehicle (ndDemoEntityManager* const scene)
{
	ndMatrix sceneLocation(ndGetIdentityMatrix());
	sceneLocation.m_posit.m_x = -200.0f;
	sceneLocation.m_posit.m_z = -200.0f;

	//BuildFloorBox(scene, sceneLocation);
	BuildFlatPlane(scene, true);
	//BuildGridPlane(scene, 120, 4.0f, 0.0f);
	//BuildStaticMesh(scene, "track.fbx", true);
	//BuildCompoundScene(scene, ndGetIdentityMatrix());
	//BuildStaticMesh(scene, "playerarena.fbx", true);
	//BuildSplineTrack(scene, "playerarena.fbx", true);
	//BuildHeightFieldTerrain(scene, sceneLocation);

	ndPhysicsWorld* const world = scene->GetWorld();
	ndVector location(0.0f, 2.0f, 0.0f, 1.0f);
	
	ndMatrix matrix(ndGetIdentityMatrix());
	ndVector floor(FindFloor(*world, location + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit = floor;
	matrix.m_posit.m_y += 0.5f;

	ndSoundManager* const soundManager = world->GetSoundManager();
	for (ndInt32 i = 0; i < ndInt32 (sizeof(engineSounds) / sizeof(engineSounds[0])); ++i)
	{
		soundManager->CreateSoundAsset(engineSounds[i]);
	}

	ndVehicleMaterial material;
	material.m_restitution = 0.1f;
	material.m_staticFriction0 = 0.8f;
	material.m_staticFriction1 = 0.8f;
	material.m_dynamicFriction0 = 0.8f;
	material.m_dynamicFriction1 = 0.8f;

	ndContactCallback* const callback = (ndContactCallback*)scene->GetWorld()->GetContactNotify();
	callback->RegisterMaterial(material, ndDemoContactCallback::m_modelPart, ndDemoContactCallback::m_default);
	callback->RegisterMaterial(material, ndDemoContactCallback::m_modelPart, ndDemoContactCallback::m_modelPart);
	callback->RegisterMaterial(material, ndDemoContactCallback::m_vehicleTirePart, ndDemoContactCallback::m_modelPart);
	callback->RegisterMaterial(material, ndDemoContactCallback::m_vehicleTirePart, ndDemoContactCallback::m_vehicleTirePart);

	material.m_flags = material.m_flags | m_useBrushTireModel;
	callback->RegisterMaterial(material, ndDemoContactCallback::m_vehicleTirePart, ndDemoContactCallback::m_default);

	// add a model for general controls
	ndSharedPtr<ndModel> controls(new ndVehicleSelector());
	world->AddModel(controls);

	ndVehicleUI* const vehicleUI = new ndVehicleUI(scene);
	ndSharedPtr<ndUIEntity> vehicleUIPtr(vehicleUI);
	scene->Set2DDisplayRenderFunction(vehicleUIPtr);
	
	ndSharedPtr<ndModel> vehicle0 (CreateBasicVehicle(scene, jeepDesc, ndPlacementMatrix(matrix, ndVector(0.0f, 0.0f, -12.0f, 0.0f)), vehicleUI));
	//ndSharedPtr<ndModel> vehicle1 (CreateBasicVehicle(scene, viperDesc, ndPlacementMatrix(matrix, ndVector(0.0f, 0.0f, -6.0f, 0.0f)), vehicleUI));
	//ndSharedPtr<ndModel> vehicle2 (CreateBasicVehicle(scene, monterTruckDesc0, ndPlacementMatrix(matrix, ndVector(0.0f, 0.0f, 6.0f, 0.0f)), vehicleUI));
	//ndSharedPtr<ndModel> vehicle3 (CreateBasicVehicle(scene, monterTruckDesc1, ndPlacementMatrix (matrix, ndVector(0.0f, 0.0f, 0.0f, 0.0f)), vehicleUI));

	world->AddModel(vehicle0);
	//world->AddModel(vehicle1);
	//world->AddModel(vehicle2);
	//world->AddModel(vehicle3);

//ndMatrix matrix1(vehicle0->GetAsModelArticulation()->GetRoot()->m_body->GetMatrix());
//matrix1.m_posit = vehicle0->GetAsModelArticulation()->GetRoot()->m_body->GetGlobalGetCentreOfMass();
//ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(matrix1, vehicle0->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic(), world->GetSentinelBody()));
//world->AddJoint(fixJoint);

	//ndBasicMultiBodyVehicle* const vehicle = (ndBasicMultiBodyVehicle*)*vehicle3;
	//ndBasicMultiBodyVehicle* const vehicle = (ndBasicMultiBodyVehicle*)*vehicle0;
	ndVehicleCommonNotify* const notifyCallback = (ndVehicleCommonNotify*)*vehicle0->GetNotifyCallback();
	notifyCallback->SetAsPlayer(scene);
	matrix.m_posit.m_x += 5.0f;
	TestPlayerCapsuleInteraction(scene, matrix);
	
	matrix.m_posit.m_x += 20.0f;
	matrix.m_posit.m_z += 5.0f;
	AddPlanks(scene, matrix, 60.0f, 5);

	ndQuaternion rot;
	ndVector origin(-10.0f, 2.0f, -10.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}
