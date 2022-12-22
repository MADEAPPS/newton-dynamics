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
#include "ndVehicleCommon.h"
#include "ndMakeStaticMap.h"
#include "ndTargaToOpenGl.h"
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

		m_useHardSolverMode = false;

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
		m_frontTire.m_upperStop = -0.05f;
		m_frontTire.m_lowerStop = 0.4f;
		m_frontTire.m_verticalOffset = -0.1f;
		m_frontTire.m_brakeTorque = 1000.0f;
		m_frontTire.m_laterialStiffness = 20.0f * DEMO_GRAVITY;
		m_frontTire.m_longitudinalStiffness = 10.0f * DEMO_GRAVITY;

		m_rearTire.m_mass = 100.0f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 500.0f;
		m_rearTire.m_damperC = 50.0f;
		m_rearTire.m_regularizer = 0.2f;
		m_rearTire.m_upperStop = -0.05f;
		m_rearTire.m_lowerStop = 0.4f;
		m_rearTire.m_verticalOffset = -0.1f;
		m_rearTire.m_brakeTorque = 2500.0f;
		m_rearTire.m_laterialStiffness = 20.0f * DEMO_GRAVITY;
		m_rearTire.m_longitudinalStiffness = 10.0f * DEMO_GRAVITY;

		m_rearTire.m_frictionModel = ndTireFrictionModel::m_brushModel;
		m_frontTire.m_frictionModel = ndTireFrictionModel::m_brushModel;

		m_transmission.m_crownGearRatio = 20.0f;
		m_torsionBarType = m_fourWheelAxle;
		m_differentialType = m_eightWheeldrive;
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

		m_useHardSolverMode = false;

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
		m_frontTire.m_upperStop = -0.05f;
		m_frontTire.m_lowerStop = 0.4f;
		m_frontTire.m_verticalOffset = -0.1f;
		m_frontTire.m_brakeTorque = 1000.0f;
		m_frontTire.m_laterialStiffness = 20.0f * DEMO_GRAVITY;
		m_frontTire.m_longitudinalStiffness = 10.0f * DEMO_GRAVITY;

		m_rearTire.m_mass = 200.0f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 1000.0f;
		m_rearTire.m_damperC = 25.0f;
		m_rearTire.m_regularizer = 0.01f;
		m_rearTire.m_upperStop = -0.05f;
		m_rearTire.m_lowerStop = 0.4f;
		m_rearTire.m_verticalOffset = -0.1f;
		m_rearTire.m_brakeTorque = 2500.0f;
		m_rearTire.m_laterialStiffness = 20.0f * DEMO_GRAVITY;
		m_rearTire.m_longitudinalStiffness = 10.0f * DEMO_GRAVITY;

		m_transmission.m_crownGearRatio = 20.0f;
		m_torsionBarType = m_fourWheelAxle;
		m_differentialType = m_fourWheeldrive;

		m_rearTire.m_frictionModel = ndTireFrictionModel::m_brushModel;
		m_frontTire.m_frictionModel = ndTireFrictionModel::m_brushModel;
	}
};

class ndVehicleDectriptorBigRig : public ndVehicleDectriptor
{
	public:
	ndVehicleDectriptorBigRig()
		//:ndVehicleDectriptor("bigRigCabin.fbx")
		:ndVehicleDectriptor("truck.fbx")
	{
		m_chassisMass = 2000.0f;
		m_chassisAngularDrag = 0.25f;
		m_comDisplacement = ndVector(0.0f, -0.55f, 0.0f, 0.0f);

		m_useHardSolverMode = false;

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
		m_frontTire.m_upperStop = -0.05f;
		m_frontTire.m_lowerStop = 0.4f;
		m_frontTire.m_brakeTorque = 1000.0f;
		m_frontTire.m_laterialStiffness = 20.0f * DEMO_GRAVITY;
		m_frontTire.m_longitudinalStiffness = 10.0f * DEMO_GRAVITY;

		m_rearTire.m_mass = 100.0f;
		m_rearTire.m_verticalOffset = -0.3f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 500.0f;
		m_rearTire.m_damperC = 50.0f;
		m_rearTire.m_regularizer = 0.2f;
		m_rearTire.m_upperStop = -0.05f;
		m_rearTire.m_lowerStop = 0.4f;
		m_rearTire.m_brakeTorque = 2500.0f;
		m_rearTire.m_laterialStiffness = 20.0f * DEMO_GRAVITY;
		m_rearTire.m_longitudinalStiffness = 10.0f * DEMO_GRAVITY;

		m_transmission.m_crownGearRatio = 20.0f;
		m_torsionBarType = m_fourWheelAxle;
		m_differentialType = m_eightWheeldrive;

		m_rearTire.m_frictionModel = ndTireFrictionModel::m_brushModel;
		m_frontTire.m_frictionModel = ndTireFrictionModel::m_brushModel;
	}
};

static ndVehicleDectriptorLav25 lav25Desc;
static ndVehicleDectriptorBigRig bigRigDesc;
static ndVehicleDectriptorTractor tractorDesc;

class ndHeavyMultiBodyVehicle : public ndVehicleCommon
{
	public:
	ndHeavyMultiBodyVehicle(ndDemoEntityManager* const scene, const ndVehicleDectriptor& desc, const ndMatrix& matrix, ndVehicleUI* const vehicleUI)
		:ndVehicleCommon(desc)
		,m_vehicleUI(vehicleUI)
	{
		ndDemoEntity* const vehicleEntity = LoadMeshModel(scene, desc.m_name);
		
		vehicleEntity->ResetMatrix(vehicleEntity->CalculateGlobalMatrix() * matrix);
		
		// create the vehicle chassis as a normal rigid body
		ndSharedPtr<ndBodyKinematic> chassis (CreateChassis(scene, vehicleEntity, m_configuration.m_chassisMass));
		chassis->SetAngularDamping(ndVector(m_configuration.m_chassisAngularDrag));
		
		// lower vehicle com;
		ndVector com(chassis->GetCentreOfMass());
		com += m_localFrame.m_up.Scale(m_configuration.m_comDisplacement.m_y);
		com += m_localFrame.m_front.Scale(m_configuration.m_comDisplacement.m_x);
		com += m_localFrame.m_right.Scale(m_configuration.m_comDisplacement.m_z);
		chassis->SetCentreOfMass(com);
		
		// 1- add chassis to the vehicle mode 
		ndAssert(0);
		//AddChassis(chassis);
	}

	~ndHeavyMultiBodyVehicle()
	{
		if (m_vehicleUI)
		{
			delete m_vehicleUI;
		}

		ReleaseTexture(m_gears);
		ReleaseTexture(m_odometer);
		ReleaseTexture(m_redNeedle);
		ReleaseTexture(m_tachometer);
		ReleaseTexture(m_greenNeedle);
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
		ndVehicleCommon::SetAsPlayer(scene, mode);

		scene->SetSelectedModel(this);
		scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
		ndAssert(0);
		//scene->Set2DDisplayRenderFunction(RenderHelp, RenderUI, this);
	}

	static void RenderUI(ndDemoEntityManager* const scene, void* const context)
	{
		ndHeavyMultiBodyVehicle* const me = (ndHeavyMultiBodyVehicle*)context;
		me->RenderUI(scene);
	}

	static void RenderHelp(ndDemoEntityManager* const scene, void* const context)
	{
		ndHeavyMultiBodyVehicle* const me = (ndHeavyMultiBodyVehicle*)context;
		me->RenderHelp(scene);
	}

	protected:
	ndBodyKinematic* MakeChildPart(ndDemoEntityManager* const scene, ndBodyKinematic* const parentBody, const char* const partName, ndFloat32 mass) const
	{
		ndDemoEntity* const parentEntity = (ndDemoEntity*)parentBody->GetNotifyCallback()->GetUserData();

		ndDemoEntity* const vehPart = parentEntity->Find(partName);
		ndShapeInstance* const vehCollision = vehPart->CreateCollisionFromChildren();
		ndSharedPtr<ndShapeInstance> vehCollisionPtr (vehCollision);

		ndBodyKinematic* const vehBody = new ndBodyDynamic();
		const ndMatrix matrix(vehPart->CalculateGlobalMatrix(nullptr));
		vehBody->SetNotifyCallback(new ndDemoEntityNotify(scene, vehPart, parentBody));
		vehBody->SetMatrix(matrix);
		vehBody->SetCollisionShape(*vehCollision);
		vehBody->SetMassMatrix(mass, *vehCollision);
		return vehBody;
	}

	private:
	virtual void VehicleAssembly(ndDemoEntityManager* const scene) = 0;

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
		ndHeavyMultiBodyVehicle* const me = (ndHeavyMultiBodyVehicle*)context;
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
		ndAssert(0);
		ndMultiBodyVehicleMotor* const motor = *m_motor;
		if (motor)
		{
			ndAssert(motor);
			if (m_vehicleUI)
			{
				ndAssert(0);
				//m_vehicleUI->RenderDash(this);
			}
		}
	}

	protected:
	GLuint m_gears;
	GLuint m_odometer;
	GLuint m_redNeedle;
	GLuint m_tachometer;
	GLuint m_greenNeedle;
	ndInt32 m_gearMap[8];
	ndVehicleUI* m_vehicleUI;
};

class ndLav25Vehicle : public ndHeavyMultiBodyVehicle
{
	public:
	ndLav25Vehicle(ndDemoEntityManager* const scene, const ndVehicleDectriptor& desc, const ndMatrix& matrix, ndVehicleUI* const vehicleUI)
		:ndHeavyMultiBodyVehicle(scene, desc, matrix, vehicleUI)
		,m_effector(nullptr)
		,m_cannonHigh(0.0f)
		,m_turretAngle(0.0f)
	{
		VehicleAssembly(scene);
	}

	void VehicleAssembly(ndDemoEntityManager* const scene)
	{
		// 2- each tire to the model, 
		// this function will create the tire as a normal rigid body
		// and attach them to the chassis with the tire joints

		ndAssert(0);
		//ndWorld* const world = scene->GetWorld();
		//ndBodyKinematic* const chassis = m_chassis;
		//
		//ndVehicleDectriptor::ndTireDefinition r0_tireConfiguration(m_configuration.m_rearTire);
		//ndVehicleDectriptor::ndTireDefinition r1_tireConfiguration(m_configuration.m_rearTire);
		//ndVehicleDectriptor::ndTireDefinition r2_tireConfiguration(m_configuration.m_rearTire);
		//ndVehicleDectriptor::ndTireDefinition r3_tireConfiguration(m_configuration.m_rearTire);
		//ndSharedPtr<ndBodyKinematic> rr_tire0_body (CreateTireBody(scene, chassis, r0_tireConfiguration, "rtire_3"));
		//ndSharedPtr<ndBodyKinematic> rl_tire0_body (CreateTireBody(scene, chassis, r1_tireConfiguration, "ltire_3"));
		//ndSharedPtr<ndBodyKinematic> rr_tire1_body (CreateTireBody(scene, chassis, r2_tireConfiguration, "rtire_2"));
		//ndSharedPtr<ndBodyKinematic> rl_tire1_body (CreateTireBody(scene, chassis, r3_tireConfiguration, "ltire_2"));
		//ndMultiBodyVehicleTireJoint* const rr_tire0 = AddTire(r0_tireConfiguration, rr_tire0_body);
		//ndMultiBodyVehicleTireJoint* const rl_tire0 = AddTire(r1_tireConfiguration, rl_tire0_body);
		//ndMultiBodyVehicleTireJoint* const rr_tire1 = AddTire(r2_tireConfiguration, rr_tire1_body);
		//ndMultiBodyVehicleTireJoint* const rl_tire1 = AddTire(r3_tireConfiguration, rl_tire1_body);
		//
		//ndVehicleDectriptor::ndTireDefinition f0_tireConfiguration(m_configuration.m_frontTire);
		//ndVehicleDectriptor::ndTireDefinition f1_tireConfiguration(m_configuration.m_frontTire);
		//ndVehicleDectriptor::ndTireDefinition f2_tireConfiguration(m_configuration.m_frontTire);
		//ndVehicleDectriptor::ndTireDefinition f3_tireConfiguration(m_configuration.m_frontTire);
		//ndSharedPtr<ndBodyKinematic> fr_tire0_body (CreateTireBody(scene, chassis, f0_tireConfiguration, "rtire_0"));
		//ndSharedPtr<ndBodyKinematic> fl_tire0_body (CreateTireBody(scene, chassis, f1_tireConfiguration, "ltire_0"));
		//ndSharedPtr<ndBodyKinematic> fr_tire1_body (CreateTireBody(scene, chassis, f2_tireConfiguration, "rtire_1"));
		//ndSharedPtr<ndBodyKinematic> fl_tire1_body (CreateTireBody(scene, chassis, f3_tireConfiguration, "ltire_1"));
		//ndMultiBodyVehicleTireJoint* const fr_tire0 = AddTire(f0_tireConfiguration, fr_tire0_body);
		//ndMultiBodyVehicleTireJoint* const fl_tire0 = AddTire(f1_tireConfiguration, fl_tire0_body);
		//ndMultiBodyVehicleTireJoint* const fr_tire1 = AddTire(f2_tireConfiguration, fr_tire1_body);
		//ndMultiBodyVehicleTireJoint* const fl_tire1 = AddTire(f3_tireConfiguration, fl_tire1_body);
		//
		//m_gearMap[sizeof(m_configuration.m_transmission.m_forwardRatios) / sizeof(m_configuration.m_transmission.m_forwardRatios[0]) + 0] = 1;
		//m_gearMap[sizeof(m_configuration.m_transmission.m_forwardRatios) / sizeof(m_configuration.m_transmission.m_forwardRatios[0]) + 1] = 0;
		//for (ndInt32 i = 0; i < m_configuration.m_transmission.m_gearsCount; ++i)
		//{
		//	m_gearMap[i] = i + 2;
		//}
		//m_currentGear = sizeof(m_configuration.m_transmission.m_forwardRatios) / sizeof(m_configuration.m_transmission.m_forwardRatios[0]) + 1;
		//
		//// add the slip differential
		//#if 1
		//ndMultiBodyVehicleDifferential* const rearDifferential0 = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire0, rr_tire0, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		//ndMultiBodyVehicleDifferential* const rearDifferential1 = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire1, rr_tire1, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		//
		//ndMultiBodyVehicleDifferential* const frontDifferential0 = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, fl_tire0, fr_tire0, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		//ndMultiBodyVehicleDifferential* const frontDifferential1 = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, fl_tire1, fr_tire1, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		//
		//ndMultiBodyVehicleDifferential* const rearDifferential = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rearDifferential0, rearDifferential1, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		//ndMultiBodyVehicleDifferential* const frontDifferential = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, frontDifferential0, frontDifferential1, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		//
		//ndMultiBodyVehicleDifferential* const differential = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rearDifferential, frontDifferential, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		//
		//#else
		//ndMultiBodyVehicleDifferential* const frontDifferential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, fl_tire0, fr_tire0, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		//ndMultiBodyVehicleDifferential* const rearDifferential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire0, rr_tire0, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		//ndMultiBodyVehicleDifferential* const differential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rearDifferential, frontDifferential, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		//
		//LinkTires(world, fl_tire0, fl_tire1);
		//LinkTires(world, rl_tire0, rl_tire1);
		//
		//LinkTires(world, fr_tire0, fr_tire1);
		//LinkTires(world, rr_tire0, rr_tire1);
		//#endif
		//
		//// add a motor
		//ndMultiBodyVehicleMotor* const motor = AddMotor(m_configuration.m_motorMass, m_configuration.m_motorRadius);
		//motor->SetMaxRpm(m_configuration.m_engine.GetRedLineRadPerSec() * dRadPerSecToRpm);
		//
		//// add the gear box
		//ndMultiBodyVehicleGearBox* const gearBox = AddGearBox(differential);
		//gearBox->SetIdleOmega(m_configuration.m_engine.GetIdleRadPerSec() * dRadPerSecToRpm);
		//
		//// add torsion bar
		//ndMultiBodyVehicleTorsionBar* const torsionBar = AddTorsionBar(world->GetSentinelBody());
		//torsionBar->AddAxel(rl_tire0->GetBody0(), rr_tire0->GetBody0());
		//torsionBar->AddAxel(fl_tire0->GetBody0(), fr_tire0->GetBody0());
		//torsionBar->SetTorsionTorque(m_configuration.m_torsionBarSpringK, m_configuration.m_torsionBarDamperC, m_configuration.m_torsionBarRegularizer);
		//
		//// add vehicle turret
		//CreateEightWheelTurret(scene);
		//
		//// set a soft or hard mode
		//SetVehicleSolverModel(m_configuration.m_useHardSolverMode ? true : false);
	}

	void CreateEightWheelTurret(ndDemoEntityManager* const scene)
	{
		ndAssert(0);
		////turret body
		//ndSharedPtr<ndBodyKinematic>turretBody (MakeChildPart(scene, m_chassis, "turret", m_configuration.m_chassisMass * 0.05f));
		//const ndMatrix turretMatrix(m_localFrame * turretBody->GetMatrix());
		////ndJointHinge* const turretHinge = new ndJointHinge(turretMatrix, *turretBody, m_chassis);
		//ndSharedPtr<ndJointBilateralConstraint> turretHinge (new ndJointHinge(turretMatrix, *turretBody, m_chassis));
		//AddExtraBody(turretBody);
		//AddExtraJoint(turretHinge);
		//
		////cannon body
		//ndSharedPtr<ndBodyKinematic>canonBody (MakeChildPart(scene, *turretBody, "canon", m_configuration.m_chassisMass * 0.025f));
		//ndMatrix cannonMatrix(m_localFrame * canonBody->GetMatrix());
		//ndSharedPtr<ndJointBilateralConstraint> cannonHinge (new ndJointHinge(cannonMatrix, *canonBody, *turretBody));
		//AddExtraBody(canonBody);
		//AddExtraJoint(cannonHinge);
		//
		//// link the effector for controlling the turret
		//ndDemoEntity* const turretEntity = (ndDemoEntity*)turretBody->GetNotifyCallback()->GetUserData();
		//ndDemoEntity* const effectorEntity = turretEntity->Find("effector");
		//ndMatrix effectorMatrix(m_localFrame * effectorEntity->CalculateGlobalMatrix(nullptr));
		//effectorMatrix.m_posit = turretBody->GetMatrix().m_posit;
		//
		//m_effector = new ndIk6DofEffector(effectorMatrix, effectorMatrix, *canonBody, m_chassis);
		//m_effector->EnableAxisX(true);
		//m_effector->EnableAxisY(false);
		//m_effector->EnableAxisZ(false);
		//m_effector->EnableRotationAxis(ndIk6DofEffector::m_fixAxis);
		//ndSharedPtr<ndJointBilateralConstraint> effectorPtr(m_effector);
		//AddExtraJoint(effectorPtr);
	}

	void LinkTires(const ndMultiBodyVehicleTireJoint* const tire0, const ndMultiBodyVehicleTireJoint* const tire1)
	{
		ndAssert(0);
		//ndBodyKinematic* const body0 = tire0->GetBody0();
		//ndBodyKinematic* const body1 = tire1->GetBody0();
		//
		//ndShapeInfo rearInfo(body0->GetCollisionShape().GetShapeInfo());
		//ndShapeInfo frontInfo(body1->GetCollisionShape().GetShapeInfo());
		//ndFloat32 tireRatio = rearInfo.m_scale.m_y / frontInfo.m_scale.m_y;
		//
		//ndMatrix pin0(tire0->GetLocalMatrix0() * body0->GetMatrix());
		//ndMatrix pin1(tire1->GetLocalMatrix0() * body1->GetMatrix());
		//
		//ndSharedPtr<ndJointBilateralConstraint> link(new ndJointGear(tireRatio, pin0.m_front.Scale(-1.0f), body0, pin1.m_front, body1));
		////world->AddJoint(link);
		//AddExtraJoint(link);
	}

	void ApplyInputs(ndWorld* const world, ndFloat32 timestep)
	{
		ndVehicleCommon::ApplyInputs(world, timestep);

		if (m_isPlayer)
		{
			ndDemoEntityManager* const scene = ((ndPhysicsWorld*)world)->GetManager();
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
				ndMatrix effectorMatrix (ndPitchMatrix(m_turretAngle));
				effectorMatrix.m_posit.m_x = m_cannonHigh;
				m_effector->SetOffsetMatrix(effectorMatrix);

				m_chassis->SetSleepState(false);
			}
		}
	}

	ndIk6DofEffector* m_effector;
	ndFloat32 m_cannonHigh;
	ndFloat32 m_turretAngle;
};

class ndTractorVehicle : public ndHeavyMultiBodyVehicle
{
	public:
	ndTractorVehicle(ndDemoEntityManager* const scene, const ndVehicleDectriptor& desc, const ndMatrix& matrix, ndVehicleUI* const vehicleUI)
		:ndHeavyMultiBodyVehicle(scene, desc, matrix, vehicleUI)
		,m_armHinge(nullptr)
		,m_bucketHinge(nullptr)
		,m_armAngle(0.0f)
		,m_bucketAngle(0.0f)
	{
		VehicleAssembly(scene);
	}

	void VehicleAssembly(ndDemoEntityManager* const scene)
	{
		// 2- each tire to the model, 
		// this function will create the tire as a normal rigid body
		// and attach them to the chassis with the tire joints

		ndAssert(0);
		//ndWorld* const world = scene->GetWorld();
		//ndBodyKinematic* const chassis = m_chassis;
		//
		//ndVehicleDectriptor::ndTireDefinition r0_tireConfiguration(m_configuration.m_rearTire);
		//ndVehicleDectriptor::ndTireDefinition r1_tireConfiguration(m_configuration.m_rearTire);
		//ndSharedPtr<ndBodyKinematic> rr_tire0_body (CreateTireBody(scene, chassis, r0_tireConfiguration, "rr_tire"));
		//ndSharedPtr<ndBodyKinematic> rl_tire0_body (CreateTireBody(scene, chassis, r1_tireConfiguration, "rl_tire"));
		//ndMultiBodyVehicleTireJoint* const rr_tire0 = AddTire(r0_tireConfiguration, rr_tire0_body);
		//ndMultiBodyVehicleTireJoint* const rl_tire0 = AddTire(r1_tireConfiguration, rl_tire0_body);
		//
		//ndVehicleDectriptor::ndTireDefinition f0_tireConfiguration(m_configuration.m_frontTire);
		//ndVehicleDectriptor::ndTireDefinition f1_tireConfiguration(m_configuration.m_frontTire);
		//ndSharedPtr<ndBodyKinematic> frontAxel_body (MakeFronAxel(scene, chassis));
		//ndSharedPtr<ndBodyKinematic> fr_tire0_body (CreateTireBody(scene, *frontAxel_body, f0_tireConfiguration, "fr_tire"));
		//ndSharedPtr<ndBodyKinematic> fl_tire0_body (CreateTireBody(scene, *frontAxel_body, f1_tireConfiguration, "fl_tire"));
		//ndMultiBodyVehicleTireJoint* const fr_tire0 = AddAxleTire(f0_tireConfiguration, fr_tire0_body, frontAxel_body);
		//ndMultiBodyVehicleTireJoint* const fl_tire0 = AddAxleTire(f1_tireConfiguration, fl_tire0_body, frontAxel_body);
		//
		//m_gearMap[sizeof(m_configuration.m_transmission.m_forwardRatios) / sizeof(m_configuration.m_transmission.m_forwardRatios[0]) + 0] = 1;
		//m_gearMap[sizeof(m_configuration.m_transmission.m_forwardRatios) / sizeof(m_configuration.m_transmission.m_forwardRatios[0]) + 1] = 0;
		//for (ndInt32 i = 0; i < m_configuration.m_transmission.m_gearsCount; ++i)
		//{
		//	m_gearMap[i] = i + 2;
		//}
		//m_currentGear = sizeof(m_configuration.m_transmission.m_forwardRatios) / sizeof(m_configuration.m_transmission.m_forwardRatios[0]) + 1;
		//
		//// add a motor
		//ndMultiBodyVehicleMotor* const motor = AddMotor(m_configuration.m_motorMass, m_configuration.m_motorRadius);
		//motor->SetMaxRpm(m_configuration.m_engine.GetRedLineRadPerSec() * dRadPerSecToRpm);
		//
		//// add 4 x 4 the slip differential
		//ndMultiBodyVehicleDifferential* const rearDifferential = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire0, rr_tire0, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		//ndMultiBodyVehicleDifferential* const frontDifferential = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, fl_tire0, fr_tire0, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		//
		//// slip differential with high slip ratio 
		//ndMultiBodyVehicleDifferential* const differential = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rearDifferential, frontDifferential, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		//differential->SetSlipOmega(2.0f * differential->GetSlipOmega());
		//
		//// add the gear box
		//ndMultiBodyVehicleGearBox* const gearBox = AddGearBox(differential);
		//gearBox->SetIdleOmega(m_configuration.m_engine.GetIdleRadPerSec() * dRadPerSecToRpm);
		//
		//// add the bucket joints
		//CreateTractorBucket(scene);
		//
		//// set a soft of hard mode
		//SetVehicleSolverModel(m_configuration.m_useHardSolverMode ? true : false);
	}

	ndSharedPtr<ndBodyKinematic> MakeFronAxel(ndDemoEntityManager* const scene, ndBodyKinematic* const chassis)
	{
		ndAssert(0);
		return ndSharedPtr<ndBodyKinematic>(nullptr);
		//ndSharedPtr<ndBodyKinematic> axleBody(MakeChildPart(scene, m_chassis, "front_axel", m_configuration.m_chassisMass * 0.2f));
		//
		//// connect the part to the main body with a hinge
		//ndMatrix hingeFrame(m_localFrame * axleBody->GetMatrix());
		//ndJointHinge* const hinge = new ndJointHinge(hingeFrame, *axleBody, chassis);
		//hinge->SetLimits(-15.0f * ndDegreeToRad, 15.0f * ndDegreeToRad);
		//ndSharedPtr<ndJointBilateralConstraint> hingePtr(hinge);
		//AddExtraBody(axleBody);
		//AddExtraJoint(hingePtr);
		//return axleBody;
	}

	void AddHydraulic(ndDemoEntityManager* const scene, ndBodyKinematic* const parentBody, const char* const name0, const char* const name1, ndBodyKinematic* const attachmentBody, const char* const attachement)
	{
		ndAssert(0);
		//ndSharedPtr<ndBodyKinematic>body0 (MakeChildPart(scene, parentBody, name0, m_configuration.m_chassisMass * 0.01f));
		//ndMatrix matrix0(m_localFrame * body0->GetMatrix());
		////ndJointBilateralConstraint* const joint0 = new ndJointHinge(matrix0, body0, parentBody);
		//ndSharedPtr<ndJointBilateralConstraint>joint0 (new ndJointHinge(matrix0, *body0, parentBody));
		//AddExtraBody(body0);
		//AddExtraJoint(joint0);
		//
		//ndSharedPtr<ndBodyKinematic>body1 (MakeChildPart(scene, *body0, name1, m_configuration.m_chassisMass * 0.01f));
		//ndMatrix matrix1(m_localFrame * body1->GetMatrix());
		//ndSharedPtr<ndJointBilateralConstraint>joint1 (new ndJointSlider(matrix1, *body1, *body0));
		//AddExtraBody(body1);
		//AddExtraJoint(joint1);
		//
		//ndDemoEntity* const parentEntity = (ndDemoEntity*)m_chassis->GetNotifyCallback()->GetUserData();
		//ndDemoEntity* const attachmentNode = parentEntity->Find(attachement);
		//matrix0.m_posit = attachmentNode->CalculateGlobalMatrix(nullptr).m_posit;
		//
		//ndIk6DofEffector* const attachementJoint = new ndIk6DofEffector(matrix0, matrix0, *body1, attachmentBody);
		//attachementJoint->EnableAxisX(false);
		//attachementJoint->EnableAxisY(true);
		//attachementJoint->EnableAxisZ(true);
		//attachementJoint->EnableRotationAxis(ndIk6DofEffector::m_disabled);
		//
		//ndSharedPtr<ndJointBilateralConstraint> attachementJointPtr(attachementJoint);
		//AddExtraJoint(attachementJointPtr);
	}

	void CreateTractorBucket(ndDemoEntityManager* const scene)
	{
		ndAssert(0);
		//ndSharedPtr<ndBodyKinematic>armBody (MakeChildPart(scene, m_chassis, "arms", m_configuration.m_chassisMass * 0.05f));
		//ndMatrix armMatrix(m_localFrame * armBody->GetMatrix());
		//m_armHinge = new ndJointHinge(armMatrix, *armBody, m_chassis);
		//m_armHinge->SetAsSpringDamper(0.01f, 1500.0f, 20.0f);
		//ndSharedPtr<ndJointBilateralConstraint> armHingePtr(m_armHinge);
		//
		//AddExtraBody(armBody);
		//AddExtraJoint(armHingePtr);
		//AddHydraulic(scene, m_chassis, "armHydraulicPiston_left", "armHydraulic_left", *armBody, "attach0_left");
		//AddHydraulic(scene, m_chassis, "armHydraulicPiston_right", "armHydraulic_right", *armBody, "attach0_right");
		//		
		////cannon servo controller actuator
		////ndBodyKinematic* const frontBucketBody = MakeChildPart(scene, armBody, "frontBucket", m_configuration.m_chassisMass * 0.025f);
		//ndSharedPtr<ndBodyKinematic>frontBucketBody (MakeChildPart(scene, *armBody, "frontBucket", m_configuration.m_chassisMass * 0.025f));
		//ndMatrix frontBucketMatrix(m_localFrame * frontBucketBody->GetMatrix());
		//m_bucketHinge = new ndJointHinge(frontBucketMatrix, *frontBucketBody, *armBody);
		//m_bucketHinge->SetAsSpringDamper(0.01f, 1500.0f, 20.0f);
		//ndSharedPtr<ndJointBilateralConstraint> bucketHingePtr(m_bucketHinge);
		//
		//AddExtraBody(frontBucketBody);
		//AddExtraJoint(bucketHingePtr);
		//AddHydraulic(scene, *armBody, "frontBucketHydraulic001", "frontBucketHydraulicPiston001", *frontBucketBody, "attachment_frontBucket001");
		//AddHydraulic(scene, *armBody, "frontBucketHydraulic002", "frontBucketHydraulicPiston002", *frontBucketBody, "attachment_frontBucket002");
	}

	void ApplyInputs(ndWorld* const world, ndFloat32 timestep)
	{
		ndVehicleCommon::ApplyInputs(world, timestep);
		if (m_isPlayer)
		{
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

			m_armAngle = ndClamp(m_armAngle, ndFloat32 (-10.0f) * ndDegreeToRad, ndFloat32 (45.0f) * ndDegreeToRad);
			m_bucketAngle = ndClamp(m_bucketAngle, ndFloat32(-75.0f) * ndDegreeToRad, ndFloat32(80.0f) * ndDegreeToRad);
	
			if (wakeUpVehicle)
			{
				m_chassis->SetSleepState(false);
				m_armHinge->SetOffsetAngle(m_armAngle);
				m_bucketHinge->SetOffsetAngle(m_bucketAngle);
			}
		}
	}

	ndJointHinge* m_armHinge;
	ndJointHinge* m_bucketHinge;
	ndFloat32 m_armAngle;
	ndFloat32 m_bucketAngle;
};

class ndBigRigVehicle : public ndHeavyMultiBodyVehicle
{
	public:
	ndBigRigVehicle(ndDemoEntityManager* const scene, const ndVehicleDectriptor& desc, const ndMatrix& matrix, ndVehicleUI* const vehicleUI)
		:ndHeavyMultiBodyVehicle(scene, desc, matrix, vehicleUI)
	{
		VehicleAssembly(scene);
	}

	void VehicleAssembly(ndDemoEntityManager* const scene)
	{
		// 2- each tire to the model, 
		// this function will create the tire as a normal rigid body
		// and attach them to the chassis with the tire joints
	
		ndAssert(0);
		//ndWorld* const world = scene->GetWorld();
		//ndBodyKinematic* const chassis = m_chassis;
		//
		//ndVehicleDectriptor::ndTireDefinition f0_tireConfiguration(m_configuration.m_frontTire);
		//ndVehicleDectriptor::ndTireDefinition f1_tireConfiguration(m_configuration.m_frontTire);
		//ndSharedPtr<ndBodyKinematic> fl_tire_body (CreateTireBody(scene, chassis, f0_tireConfiguration, "fl_tire"));
		//ndSharedPtr<ndBodyKinematic> fr_tire_body (CreateTireBody(scene, chassis, f1_tireConfiguration, "fr_tire"));
		//AddTire(f0_tireConfiguration, fr_tire_body);
		//AddTire(f0_tireConfiguration, fl_tire_body);
		//
		//ndVehicleDectriptor::ndTireDefinition r0_tireConfiguration(m_configuration.m_rearTire);
		//ndVehicleDectriptor::ndTireDefinition r1_tireConfiguration(m_configuration.m_rearTire);
		//ndVehicleDectriptor::ndTireDefinition r2_tireConfiguration(m_configuration.m_rearTire);
		//ndVehicleDectriptor::ndTireDefinition r3_tireConfiguration(m_configuration.m_rearTire);
		//ndSharedPtr<ndBodyKinematic> rl_tire0_body (CreateTireBody(scene, chassis, r0_tireConfiguration, "rl_midle_tire"));
		//ndSharedPtr<ndBodyKinematic> rr_tire0_body (CreateTireBody(scene, chassis, r1_tireConfiguration, "rr_midle_tire"));
		//ndSharedPtr<ndBodyKinematic> rl_tire1_body (CreateTireBody(scene, chassis, r2_tireConfiguration, "rl_tire"));
		//ndSharedPtr<ndBodyKinematic> rr_tire1_body (CreateTireBody(scene, chassis, r3_tireConfiguration, "rr_tire"));
		//
		//ndMultiBodyVehicleTireJoint* const rr_tire0 = AddTire(r0_tireConfiguration, rr_tire0_body);
		//ndMultiBodyVehicleTireJoint* const rl_tire0 = AddTire(r1_tireConfiguration, rl_tire0_body);
		//ndMultiBodyVehicleTireJoint* const rr_tire1 = AddTire(r2_tireConfiguration, rr_tire1_body);
		//ndMultiBodyVehicleTireJoint* const rl_tire1 = AddTire(r3_tireConfiguration, rl_tire1_body);
		//
		//m_gearMap[sizeof(m_configuration.m_transmission.m_forwardRatios) / sizeof(m_configuration.m_transmission.m_forwardRatios[0]) + 0] = 1;
		//m_gearMap[sizeof(m_configuration.m_transmission.m_forwardRatios) / sizeof(m_configuration.m_transmission.m_forwardRatios[0]) + 1] = 0;
		//for (ndInt32 i = 0; i < m_configuration.m_transmission.m_gearsCount; ++i)
		//{
		//	m_gearMap[i] = i + 2;
		//}
		//m_currentGear = sizeof(m_configuration.m_transmission.m_forwardRatios) / sizeof(m_configuration.m_transmission.m_forwardRatios[0]) + 1;
		//
		//// add the slip differential
		//ndMultiBodyVehicleDifferential* const rearDifferential0 = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire0, rr_tire0, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		//ndMultiBodyVehicleDifferential* const rearDifferential1 = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire1, rr_tire1, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		//ndMultiBodyVehicleDifferential* const differential = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rearDifferential0, rearDifferential1, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		//
		//// add a motor
		//ndMultiBodyVehicleMotor* const motor = AddMotor(m_configuration.m_motorMass, m_configuration.m_motorRadius);
		//motor->SetMaxRpm(m_configuration.m_engine.GetRedLineRadPerSec() * dRadPerSecToRpm);
		//
		//// add the gear box
		//ndMultiBodyVehicleGearBox* const gearBox = AddGearBox(differential);
		//gearBox->SetIdleOmega(m_configuration.m_engine.GetIdleRadPerSec() * dRadPerSecToRpm);
		//
		//// set a soft or hard mode
		//SetVehicleSolverModel(m_configuration.m_useHardSolverMode ? true : false);
	}

	void ApplyInputs(ndWorld* const world, ndFloat32 timestep)
	{
		ndVehicleCommon::ApplyInputs(world, timestep);
	}
};

void ndHeavyVehicle (ndDemoEntityManager* const scene)
{
	ndVehicleMaterial material;
	material.m_restitution = 0.1f;
	material.m_staticFriction0 = 0.8f;
	material.m_staticFriction1 = 0.8f;
	material.m_dynamicFriction0 = 0.8f;
	material.m_dynamicFriction1 = 0.8f;

	ndContactCallback* const callback = (ndContactCallback*)scene->GetWorld()->GetContactNotify();
	callback->RegisterMaterial(material, ndApplicationMaterial::m_modelPart, ndApplicationMaterial::m_default);
	callback->RegisterMaterial(material, ndApplicationMaterial::m_modelPart, ndApplicationMaterial::m_modelPart);
	callback->RegisterMaterial(material, ndApplicationMaterial::m_vehicleTirePart, ndApplicationMaterial::m_default);
	callback->RegisterMaterial(material, ndApplicationMaterial::m_vehicleTirePart, ndApplicationMaterial::m_modelPart);
	callback->RegisterMaterial(material, ndApplicationMaterial::m_vehicleTirePart, ndApplicationMaterial::m_vehicleTirePart);

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
	
	ndVector location(0.0f, 2.0f, 0.0f, 1.0f);
	
	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit = location;

	ndAssert(0);
	//ndVehicleUI* const vehicleUI = new ndVehicleUI;
	
	// add a model for general controls
	ndVehicleSelector* const controls = new ndVehicleSelector();
	ndSharedPtr<ndModel> controls_ptr(controls);
	scene->GetWorld()->AddModel(controls_ptr);
	
	//ndHeavyMultiBodyVehicle* const vehicle0 = new ndBigRigVehicle(scene, bigRigDesc, matrix);
	//ndSharedPtr<ndModel> vehicle0_ptr(vehicle0);
	//scene->GetWorld()->AddModel(vehicle0_ptr);
	//
	//matrix.m_posit.m_x += 6.0f;
	//matrix.m_posit.m_z += 6.0f;
	//ndHeavyMultiBodyVehicle* const vehicle1 = new ndLav25Vehicle(scene, lav25Desc, matrix);
	//ndSharedPtr<ndModel> vehicle1_ptr(vehicle1);
	//scene->GetWorld()->AddModel(vehicle1_ptr);
	//
	//matrix.m_posit.m_z -= 12.0f;
	//ndHeavyMultiBodyVehicle* const vehicle2 = new ndTractorVehicle(scene, tractorDesc, matrix);
	//ndSharedPtr<ndModel> vehicle2_ptr(vehicle2);
	//scene->GetWorld()->AddModel(vehicle2_ptr);
	//
	//vehicle1->SetAsPlayer(scene);
	//scene->Set2DDisplayRenderFunction(ndHeavyMultiBodyVehicle::RenderHelp, ndHeavyMultiBodyVehicle::RenderUI, vehicle1);
	//
	//matrix.m_posit.m_x += 25.0f;
	//matrix.m_posit.m_z += 6.0f;
	//AddPlanks(scene, matrix, 300.0f, 5);
	//
	//ndQuaternion rot;
	//ndVector origin(-10.0f, 2.0f, 0.0f, 1.0f);
	//scene->SetCameraMatrix(rot, origin);
}
