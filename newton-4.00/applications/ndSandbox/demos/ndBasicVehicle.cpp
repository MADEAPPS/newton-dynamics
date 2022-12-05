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
#include "ndDemoCamera.h"
#include "ndLoadFbxMesh.h"
#include "ndSoundManager.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndCompoundScene.h"
#include "ndVehicleCommon.h"
#include "ndMakeStaticMap.h"
#include "ndTargaToOpenGl.h"
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

		m_frontTire.m_handBrakeTorque = 0.0f;
		m_rearTire.m_handBrakeTorque = 100000.0f;

		m_rearTire.m_frictionModel = ndTireFrictionModel::m_brushModel;
		m_frontTire.m_frictionModel = ndTireFrictionModel::m_brushModel;
		m_rearTire.m_longitudinalStiffness = 10.0f * DEMO_GRAVITY;
		m_frontTire.m_longitudinalStiffness = 10.0f * DEMO_GRAVITY;
		m_rearTire.m_laterialStiffness = 2.0f * m_rearTire.m_longitudinalStiffness;
		m_frontTire.m_laterialStiffness = 2.0f * m_frontTire.m_longitudinalStiffness;
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

		m_frontTire.m_mass = 100.0f;
		m_frontTire.m_verticalOffset = -0.15f;
		m_frontTire.m_steeringAngle = 35.0f * ndDegreeToRad;
		m_frontTire.m_springK = 800.0f;
		m_frontTire.m_damperC = 50.0f;
		m_frontTire.m_regularizer = 0.3f;
		m_frontTire.m_upperStop = -0.05f;
		m_frontTire.m_lowerStop = 0.4f;
		m_frontTire.m_brakeTorque = 1500.0f;
		m_frontTire.m_handBrakeTorque = 0.0f;

		m_rearTire.m_mass = 100.0f;
		m_rearTire.m_verticalOffset = -0.15f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 800.0f;
		m_rearTire.m_damperC = 50.0f;
		m_rearTire.m_regularizer = 0.3f;
		m_rearTire.m_upperStop = -0.05f;
		m_rearTire.m_lowerStop = 0.4f;
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
		m_frontTire.m_upperStop = -0.05f;
		m_frontTire.m_lowerStop = 0.4f;
		m_frontTire.m_brakeTorque = 1000.0f;
		m_frontTire.m_handBrakeTorque = 0.0f;

		m_rearTire.m_mass = 100.0f;
		m_rearTire.m_verticalOffset = 0.0f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 500.0f;
		m_rearTire.m_damperC = 50.0f;
		m_rearTire.m_regularizer = 0.2f;
		m_rearTire.m_upperStop = -0.05f;
		m_rearTire.m_lowerStop = 0.4f;
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
		m_frontTire.m_upperStop = -0.05f;
		m_frontTire.m_lowerStop = 0.4f;
		m_frontTire.m_brakeTorque = 10000.0f;
		m_frontTire.m_handBrakeTorque = 0.0f;

		m_rearTire.m_mass = 100.0f;
		m_rearTire.m_verticalOffset = 0.0f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 500.0f;
		m_rearTire.m_damperC = 50.0f;
		m_rearTire.m_regularizer = 0.2f;
		m_rearTire.m_upperStop = -0.05f;
		m_rearTire.m_lowerStop = 0.4f;
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

class ndBasicMultiBodyVehicle : public ndBasicVehicle
{
	public:
	ndBasicMultiBodyVehicle(ndDemoEntityManager* const scene, const ndVehicleDectriptor& desc, const ndMatrix& matrix)
		:ndBasicVehicle(desc)
		,m_skipMarks(nullptr)
		,m_startSound(nullptr)
		,m_engineRpmSound(nullptr)
		,m_vehicleUI(nullptr)
		,m_rearAxlePivot(nullptr)
		,m_frontAxlePivot(nullptr)
		,m_rr_tire(nullptr)
		,m_rl_tire(nullptr)
		,m_fr_tire(nullptr)
		,m_fl_tire(nullptr)
	{
		m_vehicleUI = new ndVehicleUI();
		m_vehicleUI->CreateBufferUI();

		ndDemoEntity* const vehicleEntity = LoadMeshModel(scene, desc.m_name);
		vehicleEntity->ResetMatrix(vehicleEntity->CalculateGlobalMatrix() * matrix);

		ndPhysicsWorld* const world = scene->GetWorld();

		// create the vehicle chassis as a normal rigid body
		ndBodyDynamic* const chassis = CreateChassis(scene, vehicleEntity, m_configuration.m_chassisMass);
		chassis->SetAngularDamping(ndVector(m_configuration.m_chassisAngularDrag));

		// lower vehicle com;
		ndVector com(chassis->GetCentreOfMass());
		com += m_localFrame.m_up.Scale(m_configuration.m_comDisplacement.m_y);
		com += m_localFrame.m_front.Scale(m_configuration.m_comDisplacement.m_x);
		com += m_localFrame.m_right.Scale(m_configuration.m_comDisplacement.m_z);
		chassis->SetCentreOfMass(com);

		// 1- add chassis to the vehicle mode 
		//AddChassis(chassis);
		SetChassis(chassis);

if (strcmp (m_configuration.m_name, "viper.fbx") == 0)
{
	scene->GetWorld()->AddModel(this);
	return;
}

		// 2- each tire to the model, 
		// create the tire as a normal rigid body
		// and attach them to the chassis with a tire joints
		ndVehicleDectriptor::ndTireDefinition rr_tireConfiguration(m_configuration.m_rearTire);
		ndVehicleDectriptor::ndTireDefinition rl_tireConfiguration(m_configuration.m_rearTire);
		ndBodyDynamic* const rr_tire_body = CreateTireBody(scene, chassis, rr_tireConfiguration, "rr_tire");
		ndBodyDynamic* const rl_tire_body = CreateTireBody(scene, chassis, rl_tireConfiguration, "rl_tire");
		ndMultiBodyVehicleTireJoint* const rr_tire = AddTire(rr_tireConfiguration, rr_tire_body);
		ndMultiBodyVehicleTireJoint* const rl_tire = AddTire(rl_tireConfiguration, rl_tire_body);

		ndVehicleDectriptor::ndTireDefinition fr_tireConfiguration(m_configuration.m_frontTire);
		ndVehicleDectriptor::ndTireDefinition fl_tireConfiguration(m_configuration.m_frontTire);
		ndBodyDynamic* const fr_tire_body = CreateTireBody(scene, chassis, fr_tireConfiguration, "fr_tire");
		ndBodyDynamic* const fl_tire_body = CreateTireBody(scene, chassis, fl_tireConfiguration, "fl_tire");
		ndMultiBodyVehicleTireJoint* const fr_tire = AddTire(fr_tireConfiguration, fr_tire_body);
		ndMultiBodyVehicleTireJoint* const fl_tire = AddTire(fl_tireConfiguration, fl_tire_body);

		m_gearMap[sizeof(m_configuration.m_transmission.m_forwardRatios) / sizeof(m_configuration.m_transmission.m_forwardRatios[0]) + 0] = 1;
		m_gearMap[sizeof(m_configuration.m_transmission.m_forwardRatios) / sizeof(m_configuration.m_transmission.m_forwardRatios[0]) + 1] = 0;
		for (ndInt32 i = 0; i < m_configuration.m_transmission.m_gearsCount; ++i)
		{
			m_gearMap[i] = i + 2;
		}
		m_currentGear = sizeof(m_configuration.m_transmission.m_forwardRatios) / sizeof(m_configuration.m_transmission.m_forwardRatios[0]) + 1;

		// add the slip differential
		ndMultiBodyVehicleDifferential* differential = nullptr;
		switch (m_configuration.m_differentialType)
		{
			case ndVehicleDectriptor::m_rearWheelDrive:
			{
				differential = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire, rr_tire, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
				break;
			}
		
			case ndVehicleDectriptor::m_frontWheelDrive:
			{
				differential = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, fl_tire, fr_tire, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
				break;
			}
		
			case ndVehicleDectriptor::m_fourWheeldrive:
			{
				ndMultiBodyVehicleDifferential* const rearDifferential = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire, rr_tire, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
				ndMultiBodyVehicleDifferential* const frontDifferential = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, fl_tire, fr_tire, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
				differential = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rearDifferential, frontDifferential, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
				break;
			}
		
			case ndVehicleDectriptor::m_eightWheeldrive:
			{
				ndAssert(0);
				break;
			}
		}

		// add a motor
		ndMultiBodyVehicleMotor* const motor = AddMotor(m_configuration.m_motorMass, m_configuration.m_motorRadius);
		motor->SetMaxRpm(m_configuration.m_engine.GetRedLineRadPerSec() * dRadPerSecToRpm);
		motor->SetFrictionLoss(m_configuration.m_engine.GetTorque(0.0f) * 0.5f);

		// add the gear box
		ndMultiBodyVehicleGearBox* const gearBox = AddGearBox(m_motor, differential);
		gearBox->SetIdleOmega(m_configuration.m_engine.GetIdleRadPerSec() * dRadPerSecToRpm);

		switch (m_configuration.m_torsionBarType)
		{
			case ndVehicleDectriptor::m_noWheelAxle:
			{
				// no torsion bar
				break;
			}

			case ndVehicleDectriptor::m_rearWheelAxle:
			{
				ndMultiBodyVehicleTorsionBar* const torsionBar = AddTorsionBar(world->GetSentinelBody());
				torsionBar->AddAxel(rl_tire->GetBody0(), rr_tire->GetBody0());
				torsionBar->SetTorsionTorque(m_configuration.m_torsionBarSpringK, m_configuration.m_torsionBarDamperC, m_configuration.m_torsionBarRegularizer);
				break;
			}

			case ndVehicleDectriptor::m_frontWheelAxle:
			{
				ndMultiBodyVehicleTorsionBar* const torsionBar = AddTorsionBar(world->GetSentinelBody());
				torsionBar->AddAxel(fl_tire->GetBody0(), fr_tire->GetBody0());
				torsionBar->SetTorsionTorque(m_configuration.m_torsionBarSpringK, m_configuration.m_torsionBarDamperC, m_configuration.m_torsionBarRegularizer);
				break;
			}

			case ndVehicleDectriptor::m_fourWheelAxle:
			{
				ndMultiBodyVehicleTorsionBar* const torsionBar = AddTorsionBar(world->GetSentinelBody());
				torsionBar->AddAxel(rl_tire->GetBody0(), rr_tire->GetBody0());
				torsionBar->AddAxel(fl_tire->GetBody0(), fr_tire->GetBody0());
				torsionBar->SetTorsionTorque(m_configuration.m_torsionBarSpringK, m_configuration.m_torsionBarDamperC, m_configuration.m_torsionBarRegularizer);
				break;
			}
		}

		// set a soft or hard mode
		SetVehicleSolverModel(m_configuration.m_useHardSolverMode ? true : false);

		// prepare data for animating suspension
		m_rearAxlePivot = vehicleEntity->Find("rearAxlePivot");
		m_frontAxlePivot = vehicleEntity->Find("frontAxlePivot");
		m_rr_tire = rr_tire;
		m_rl_tire = rl_tire;
		m_fr_tire = fr_tire;
		m_fl_tire = fl_tire;

		// load all engine sound channels
		ndSoundManager* const soundManager = world->GetSoundManager();

		m_startEngine = false;
		m_startEngineMemory = false;
		m_startSound = soundManager->CreateSoundChannel(engineSounds[0]);
		m_engineRpmSound = soundManager->CreateSoundChannel(engineSounds[1]);
		m_skipMarks = soundManager->CreateSoundChannel(engineSounds[2]);

		m_startSound->SetAttenuationRefDistance(20.0f, 40.0f, 50.0f);
		m_engineRpmSound->SetAttenuationRefDistance(20.0f, 40.0f, 50.0f);

		m_engineRpmSound->SetLoop(true);
		m_skipMarks->SetLoop(true);

		scene->GetWorld()->AddModel(this);
	}

	~ndBasicMultiBodyVehicle()
	{
		ReleaseTexture(m_gears);
		ReleaseTexture(m_odometer);
		ReleaseTexture(m_redNeedle);
		ReleaseTexture(m_tachometer);
		ReleaseTexture(m_greenNeedle);

		delete m_skipMarks;
		delete m_startSound;
		delete m_engineRpmSound;

		if (m_vehicleUI)
		{
			delete m_vehicleUI;
		}
	}

	ndDemoEntity* LoadMeshModel(ndDemoEntityManager* const scene, const char* const filename)
	{
		ndDemoEntity* const vehicleEntity = ndDemoEntity::LoadFbx(filename, scene);
		scene->AddEntity(vehicleEntity);

		// load 2d display assets
		m_gears = LoadTexture("gears_font.tga");
		m_odometer = LoadTexture("kmh_dial.tga");
		m_tachometer = LoadTexture("rpm_dial.tga");
		m_redNeedle = LoadTexture("needle_red.tga");
		m_greenNeedle = LoadTexture("needle_green.tga");
		return vehicleEntity;
	}

	void SetAsPlayer(ndDemoEntityManager* const scene, bool mode = true)
	{
		ndBasicVehicle::SetAsPlayer(scene, mode);

		scene->SetSelectedModel(this);
		scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
		scene->Set2DDisplayRenderFunction(RenderHelp, RenderUI, this);
	}

	static void RenderUI(ndDemoEntityManager* const scene, void* const context)
	{
		ndBasicMultiBodyVehicle* const me = (ndBasicMultiBodyVehicle*)context;
		me->RenderUI(scene);
	}

	static void RenderHelp(ndDemoEntityManager* const scene, void* const context)
	{
		ndBasicMultiBodyVehicle* const me = (ndBasicMultiBodyVehicle*)context;
		me->RenderHelp(scene);
	}

	private:
	ndBodyDynamic* CreateChassis(ndDemoEntityManager* const scene, ndDemoEntity* const chassisEntity, ndFloat32 mass)
	{
		ndMatrix matrix(chassisEntity->CalculateGlobalMatrix(nullptr));
		ndShapeInstance* const chassisCollision = chassisEntity->CreateCollisionFromChildren();

		ndBodyDynamic* const body = new ndBodyDynamic();
		body->SetNotifyCallback(new ndDemoEntityNotify(scene, chassisEntity));
		body->SetMatrix(matrix);
		body->SetCollisionShape(*chassisCollision);
		body->SetMassMatrix(mass, *chassisCollision);

		delete chassisCollision;
		return body;
	}

	static void UpdateCameraCallback(ndDemoEntityManager* const manager, void* const context, ndFloat32 timestep)
	{
		ndBasicMultiBodyVehicle* const me = (ndBasicMultiBodyVehicle*)context;
		me->SetCamera(manager, timestep);
	}

	void SetCamera(ndDemoEntityManager* const manager, ndFloat32)
	{
		ndDemoCamera* const camera = manager->GetCamera();
		ndDemoEntity* const chassisEntity = (ndDemoEntity*)m_chassis->GetNotifyCallback()->GetUserData();
		ndMatrix camMatrix(camera->GetNextMatrix());
		ndMatrix playerMatrix(chassisEntity->GetNextMatrix());

		ndVector frontDir(camMatrix[0]);
		ndVector camOrigin(0.0f);
		camOrigin = playerMatrix.m_posit + ndVector(0.0f, 1.0f, 0.0f, 0.0f);
		camOrigin -= frontDir.Scale(10.0f);

		camera->SetNextMatrix(camMatrix, camOrigin);
	}

	void RenderHelp(ndDemoEntityManager* const scene)
	{
		ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print(color, "Vehicle driving keyboard control");
		scene->Print(color, "change vehicle     : 'c'");
		scene->Print(color, "accelerator        : 'w'");
		scene->Print(color, "brakes             : 's'");
		scene->Print(color, "turn left          : 'a'");
		scene->Print(color, "turn right         : 'd'");
		scene->Print(color, "hand brakes        : 'space'");

		ImGui::Separator();
		scene->Print(color, "gear box");
		scene->Print(color, "ignition            : 'i'");
		scene->Print(color, "manual transmission : '?'");
		scene->Print(color, "neutral gear	    : 'n'");
		scene->Print(color, "forward gear up     : '>'");
		scene->Print(color, "forward gear down   : '<'");
		scene->Print(color, "reverse gear	    : 'r'");
		scene->Print(color, "parking gear	    : 'p'");
	}

	void RenderUI(ndDemoEntityManager* const scene)
	{
		ndMultiBodyVehicleMotor* const motor = m_motor;
		if (motor)
		{
			ndAssert(motor);

			glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
			ndFloat32 gageSize = 200.0f;
			ndFloat32 y = (ndFloat32)scene->GetHeight() - (gageSize / 2.0f + 20.0f);

			// draw the tachometer
			ndFloat32 x = gageSize / 2 + 20.0f;
			ndFloat32 maxRpm = m_configuration.m_engine.GetRedLineRadPerSec() * dRadPerSecToRpm;
			maxRpm += 500.0f;
			//printf("%.3f \n", m_configuration.m_transmission.m_forwardRatios[m_currentGear]);
			//ndFloat32 rpm = (motor->GetRpm() / maxRpm) * m_configuration.m_transmission.m_forwardRatios[m_currentGear]; 
			ndFloat32 rpm = (motor->GetRpm() / maxRpm) * 2.85f;
			
			if (m_vehicleUI) 
			{
				if (m_vehicleUI->m_shaderHandle) 
				{
					glUseProgram(m_vehicleUI->m_shaderHandle);
					//
					glActiveTexture(GL_TEXTURE0);
					//
					m_vehicleUI->RenderGageUI(scene, m_tachometer, -x, -y, gageSize * 0.5f, 0.0f, -180.0f, 90.0f);

					ndFloat32 s = gageSize * 0.7f;
					m_vehicleUI->RenderGageUI(scene, m_redNeedle, -x, -y, s * 0.5f, rpm, -0.0f, 90.0f);
					
					x += gageSize;
					m_vehicleUI->RenderGageUI(scene, m_odometer, -x, -y, gageSize * 0.5f, 0.0f, -180.0f, 90.0f);

					ndFloat32 speed = (GetSpeed() / 100.0f) * 2.85f;
					m_vehicleUI->RenderGageUI(scene, m_greenNeedle, -x, -y, s * 0.5f, ndAbs(speed), -0.0f, 90.0f);

					// draw the current gear
					m_vehicleUI->RenderGearUI(scene, m_gearMap[m_currentGear], m_gears, -x, -y, gageSize);

					glUseProgram(0);
				}
			}
		}
	}

	void ApplyInputs(ndWorld* const world, ndFloat32 timestep)
	{
		ndBasicVehicle::ApplyInputs(world, timestep);

		if (m_motor)
		{
			if (m_startEngineMemory ^ m_startEngine)
			{
				m_startEngineMemory = m_startEngine;
				if (m_startEngine)
				{
					m_startSound->Play();
					m_engineRpmSound->Play();
					m_engineRpmSound->SetPitch(0.5f);
					m_engineRpmSound->SetVolume(0.25f);
				}
				else
				{
					if (m_startSound->IsPlaying())
					{
						m_startSound->Stop();
					}
				}
			}

			ndFloat32 rpm = m_motor->GetRpm();
			if (rpm > 1.0f)
			{
				if (!m_engineRpmSound->IsPlaying())
				{
					m_engineRpmSound->Play();
				}
				// up to two decibels of volume
				ndFloat32 maxRpm = 9000.0f;
				rpm = rpm / maxRpm;
				ndFloat32 pitchFactor = 0.5f + 0.7f * rpm;
				ndFloat32 volumeFactor = 0.25f + 0.75f * rpm;

				m_engineRpmSound->SetPitch(pitchFactor);
				m_engineRpmSound->SetVolume(volumeFactor);

				// apply positional sound
				const ndMatrix& location = m_chassis->GetMatrix();
				m_engineRpmSound->SetPosition(location.m_posit);
				m_engineRpmSound->SetVelocity(m_chassis->GetVelocity());
			}
			else
			{
				if (m_engineRpmSound->IsPlaying())
				{
					m_engineRpmSound->Stop();
				}
			}
		}

		// test convex cast for now
		if (0)
		{
			// test convex cast
			ndMultiBodyVehicleTireJoint* const tire = m_tireList.GetCount() ? m_tireList.GetFirst()->GetInfo() : nullptr;
			if (tire)
			{
				const ndWheelDescriptor& info = tire->GetInfo();
				const ndMatrix tireUpperBumperMatrix(tire->CalculateUpperBumperMatrix());
				const ndVector dest(tireUpperBumperMatrix.m_posit - tireUpperBumperMatrix.m_up.Scale(info.m_lowerStop - info.m_upperStop));
				class ndConvexCastNotifyTest : public ndConvexCastNotify
				{
					public:
					ndConvexCastNotifyTest()
						:ndConvexCastNotify()
					{
					}

					virtual ndUnsigned32 OnRayPrecastAction(const ndBody* const body, const ndShapeInstance* const)
					{
						return ndUnsigned32(!((body == m_chassis) || (body == m_tire)));
					}

					ndBodyKinematic* m_tire;
					ndBodyKinematic* m_chassis;
				};

				ndConvexCastNotifyTest convexCast;
				convexCast.m_tire = tire->GetBody0();
				convexCast.m_chassis = m_chassis;

				//convexCast.CastShape(tire->GetBody0()->GetCollisionShape(), tireUpperBumperMatrix, dest, otherBody->GetCollisionShape(), otherBody->GetMatrix());
				m_chassis->GetScene()->ConvexCast(convexCast, tire->GetBody0()->GetCollisionShape(), tireUpperBumperMatrix, dest);
				convexCast.m_param = 0.0f;
			}
		}
	}

	void PostTransformUpdate(ndWorld* const, ndFloat32)
	{
		// play body part animations
		ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)m_chassis->GetNotifyCallback();
		if (m_rearAxlePivot && m_frontAxlePivot)
		{
			const ndMatrix rearPivotMatrix((m_rearAxlePivot->CalculateGlobalMatrix(notify->m_entity) * m_chassis->GetMatrix()).Inverse());
			const ndMatrix rearLeftTireMatrix(m_rl_tire->GetBody0()->GetMatrix() * rearPivotMatrix);
			const ndMatrix rearRightTireMatrix(m_rr_tire->GetBody0()->GetMatrix() * rearPivotMatrix);
			const ndVector rearOrigin(ndVector::m_half * (rearRightTireMatrix.m_posit + rearLeftTireMatrix.m_posit));
			const ndVector rearStep(rearRightTireMatrix.m_posit - rearLeftTireMatrix.m_posit);

			const ndFloat32 rearAxleAngle = ndAtan2(-rearStep.m_y, rearStep.m_z);
			const ndQuaternion rearAxelRotation(ndVector(ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f)), rearAxleAngle);
			m_rearAxlePivot->GetFirstChild()->SetNextMatrix(rearAxelRotation, rearOrigin);

			const ndMatrix frontPivotMatrix((m_frontAxlePivot->CalculateGlobalMatrix(notify->m_entity) * m_chassis->GetMatrix()).Inverse());
			const ndMatrix frontLeftTireMatrix(m_fl_tire->GetBody0()->GetMatrix() * frontPivotMatrix);
			const ndMatrix frontRightTireMatrix(m_fr_tire->GetBody0()->GetMatrix() * frontPivotMatrix);
			const ndVector frontOrigin(ndVector::m_half * (frontRightTireMatrix.m_posit + frontLeftTireMatrix.m_posit));
			const ndVector frontStep(frontRightTireMatrix.m_posit - frontLeftTireMatrix.m_posit);

			const ndFloat32 frontAxleAngle = ndAtan2(-frontStep.m_y, frontStep.m_z);
			const ndQuaternion frontAxelRotation(ndVector(ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f)), frontAxleAngle);
			m_frontAxlePivot->GetFirstChild()->SetNextMatrix(frontAxelRotation, frontOrigin);
		}
	}

	GLuint m_gears;
	GLuint m_odometer;
	GLuint m_redNeedle;
	GLuint m_tachometer;
	GLuint m_greenNeedle;
	ndInt32 m_gearMap[8];

	ndSoundChannel* m_skipMarks;
	ndSoundChannel* m_startSound;
	ndSoundChannel* m_engineRpmSound;
	ndVehicleUI* m_vehicleUI;

	ndDemoEntity* m_rearAxlePivot;
	ndDemoEntity* m_frontAxlePivot;
	ndMultiBodyVehicleTireJoint* m_rr_tire;
	ndMultiBodyVehicleTireJoint* m_rl_tire;

	ndMultiBodyVehicleTireJoint* m_fr_tire;
	ndMultiBodyVehicleTireJoint* m_fl_tire;

	bool m_startEngineMemory;
};

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
	callback->RegisterMaterial(material, ndApplicationMaterial::m_modelPart, ndApplicationMaterial::m_default);
	callback->RegisterMaterial(material, ndApplicationMaterial::m_modelPart, ndApplicationMaterial::m_modelPart);
	callback->RegisterMaterial(material, ndApplicationMaterial::m_modelPart, ndApplicationMaterial::m_vehicleTirePart);
	callback->RegisterMaterial(material, ndApplicationMaterial::m_vehicleTirePart, ndApplicationMaterial::m_default);
	callback->RegisterMaterial(material, ndApplicationMaterial::m_vehicleTirePart, ndApplicationMaterial::m_vehicleTirePart);

	// add a model for general controls
	ndVehicleSelector* const controls = new ndVehicleSelector();
	scene->GetWorld()->AddModel(controls);
	
	//ndBasicMultiBodyVehicle* const vehicle0 = new ndBasicMultiBodyVehicle(scene, jeepDesc, ndPlacementMatrix(matrix, ndVector(0.0f, 0.0f, -12.0f, 0.0f)));
	ndBasicMultiBodyVehicle* const vehicle1 = new ndBasicMultiBodyVehicle(scene, viperDesc, ndPlacementMatrix(matrix, ndVector(0.0f, 0.0f, -6.0f, 0.0f)));
	//ndBasicMultiBodyVehicle* const vehicle2 = new ndBasicMultiBodyVehicle(scene, monterTruckDesc0, ndPlacementMatrix(matrix, ndVector(0.0f, 0.0f, 6.0f, 0.0f)));
	ndBasicMultiBodyVehicle* const vehicle3 = new ndBasicMultiBodyVehicle(scene, monterTruckDesc1, ndPlacementMatrix (matrix, ndVector(0.0f, 0.0f, 0.0f, 0.0f)));

	//ndBasicMultiBodyVehicle* const vehicle = vehicle0;
	ndBasicMultiBodyVehicle* const vehicle = vehicle3;

	vehicle->SetAsPlayer(scene);
	scene->Set2DDisplayRenderFunction(ndBasicMultiBodyVehicle::RenderHelp, ndBasicMultiBodyVehicle::RenderUI, vehicle);

	matrix.m_posit.m_x += 5.0f;
	TestPlayerCapsuleInteraction(scene, matrix);
	
	matrix.m_posit.m_x += 20.0f;
	//AddPlanks(scene, matrix, 60.0f, 5);

	ndQuaternion rot;
	ndVector origin(-10.0f, 2.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);

	//ndLoadSave loadScene;
	//loadScene.SaveModel("xxxxxx", vehicle);
}
