/* Copyright (c) <2003-2021> <Newton Game Dynamics>
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
#include "ndDemoCamera.h"
#include "ndLoadFbxMesh.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndVehicleCommon.h"
#include "ndMakeStaticMap.h"
#include "ndTargaToOpenGl.h"
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
		m_comDisplacement = dVector(0.0f, -0.55f, 0.0f, 0.0f);

		dFloat32 fuelInjectionRate = 10.0f;
		dFloat32 idleTorquePoundFoot = 250.0f;
		dFloat32 idleRmp = 600.0f;
		dFloat32 horsePower = 500.0f;
		dFloat32 rpm0 = 3000.0f;
		dFloat32 rpm1 = 4000.0f;
		dFloat32 horsePowerAtRedLine = 150.0f;
		dFloat32 redLineRpm = 5000.0f;
		m_engine.Init(fuelInjectionRate, idleTorquePoundFoot, idleRmp, 
					  horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);

		m_frontTire.m_mass = 100.0f;
		m_frontTire.m_steeringAngle = 25.0f;
		m_frontTire.m_springK = 500.0f;
		m_frontTire.m_damperC = 50.0f;
		m_frontTire.m_regularizer = 0.2f;
		m_frontTire.m_upperStop = -0.05f;
		m_frontTire.m_lowerStop = 0.4f;
		m_frontTire.m_verticalOffset = -0.1f;
		m_frontTire.m_laterialStiffeness = 1.0f / 1000.0f;
		m_frontTire.m_longitudinalStiffeness = 50.0f / 1000.0f;

		m_rearTire.m_mass = 100.0f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 500.0f;
		m_rearTire.m_damperC = 50.0f;
		m_rearTire.m_regularizer = 0.2f;
		m_rearTire.m_upperStop = -0.05f;
		m_rearTire.m_lowerStop = 0.4f;
		m_rearTire.m_verticalOffset = -0.1f;
		m_rearTire.m_laterialStiffeness = 10.0f / 1000.0f;
		m_rearTire.m_longitudinalStiffeness = 50.0f / 1000.0f;

		m_transmission.m_crownGearRatio = 20.0f;
		
		m_brakeTorque = 2500.0f;
		m_handBrakeTorque = 1000.0f;
		m_frictionCoefficientScale = 1.3f;
		
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
		m_comDisplacement = dVector(0.0f, -0.55f, 0.0f, 0.0f);

		dFloat32 fuelInjectionRate = 10.0f;
		dFloat32 idleTorquePoundFoot = 250.0f;
		dFloat32 idleRmp = 600.0f;
		dFloat32 horsePower = 500.0f;
		dFloat32 rpm0 = 3000.0f;
		dFloat32 rpm1 = 4000.0f;
		dFloat32 horsePowerAtRedLine = 150.0f;
		dFloat32 redLineRpm = 5000.0f;
		m_engine.Init(fuelInjectionRate, idleTorquePoundFoot, idleRmp,
					  horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);

		m_transmission.m_gearsCount = 2;
		m_transmission.m_crownGearRatio = 20.0f;
		m_transmission.m_reverseRatio = -3.0f;
		m_transmission.m_fowardRatios[0] = 4.0f;
		m_transmission.m_fowardRatios[1] = 3.0f;

		m_frontTire.m_mass = 100.0f;
		m_frontTire.m_steeringAngle = 25.0f;
		m_frontTire.m_springK = 1000.0f;
		m_frontTire.m_damperC = 25.0f;
		m_frontTire.m_regularizer = 0.01f;
		m_frontTire.m_upperStop = -0.05f;
		m_frontTire.m_lowerStop = 0.4f;
		m_frontTire.m_verticalOffset = -0.1f;
		m_frontTire.m_laterialStiffeness = 500.0f / 1000.0f;
		m_frontTire.m_longitudinalStiffeness = 500.0f / 1000.0f;

		m_rearTire.m_mass = 100.0f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 1000.0f;
		m_rearTire.m_damperC = 25.0f;
		m_rearTire.m_regularizer = 0.01f;
		m_rearTire.m_upperStop = -0.05f;
		m_rearTire.m_lowerStop = 0.4f;
		m_rearTire.m_verticalOffset = -0.1f;
		m_rearTire.m_laterialStiffeness = 500.0f / 1000.0f;
		m_rearTire.m_longitudinalStiffeness = 500.0f / 1000.0f;

		m_transmission.m_crownGearRatio = 20.0f;

		m_brakeTorque = 2500.0f;
		m_handBrakeTorque = 1000.0f;
		m_frictionCoefficientScale = 1.3f;

		m_torsionBarType = m_fourWheelAxle;
		m_differentialType = m_fourWheeldrive;
	}
};

static ndVehicleDectriptorLav25 lav25Desc;
static ndVehicleDectriptorTractor tractorDesc;

class ndHeavyMultiBodyVehicle : public ndBasicVehicle
{
	public:
	ndHeavyMultiBodyVehicle(ndDemoEntityManager* const scene, const ndVehicleDectriptor& desc, const dMatrix& matrix)
		:ndBasicVehicle(desc)
		,m_turretHinge(nullptr)
		,m_cannonHinge(nullptr)
		,m_turretAngle(0.0f)
		,m_turretAngle0(0.0f)
		,m_cannonAngle(0.0f)
		,m_cannonAngle0(0.0f)
	{
		ndDemoEntity* const vehicleEntity = LoadMeshModel(scene, desc.m_name);
		vehicleEntity->ResetMatrix(vehicleEntity->CalculateGlobalMatrix() * matrix);

		// create the vehicle chassis as a normal rigid body
		ndBodyDynamic* const chassis = CreateChassis(scene, vehicleEntity, m_configuration.m_chassisMass);

		// lower vehicle com;
		dVector com(chassis->GetCentreOfMass());
		com += m_localFrame.m_up.Scale(m_configuration.m_comDisplacement.m_y);
		com += m_localFrame.m_front.Scale(m_configuration.m_comDisplacement.m_x);
		com += m_localFrame.m_right.Scale(m_configuration.m_comDisplacement.m_z);
		chassis->SetCentreOfMass(com);

		// 1- add chassis to the vehicle mode 
		AddChassis(chassis, DEMO_GRAVITY);

		if (!strcmp (m_configuration.m_name, "lav_25.fbx"))
		{
			CreateEightWheelTruck(scene);
		} 
		else if (!strcmp(m_configuration.m_name, "tractor.fbx"))
		{
			CreateTractor(scene);
		}
		else
		{
			dAssert(0);
		}
	}

	~ndHeavyMultiBodyVehicle()
	{
		ReleaseTexture(m_gears);
		ReleaseTexture(m_odometer);
		ReleaseTexture(m_redNeedle);
		ReleaseTexture(m_tachometer);
		ReleaseTexture(m_greenNeedle);
	}

	ndDemoEntity* LoadMeshModel(ndDemoEntityManager* const scene, const char* const filename)
	{
		fbxDemoEntity* const vehicleEntity = LoadFbxMesh(filename);
		vehicleEntity->BuildRenderMeshes(scene);
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
		scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
		scene->Set2DDisplayRenderFunction(RenderHelp, RenderUI, this);
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

	private:
	void CreateEightWheelTurret(ndDemoEntityManager* const scene)
	{
		ndWorld* const world = scene->GetWorld();

		//turret servo controller actuator
		ndBodyDynamic* const turretBody = MakeChildPart(scene, m_chassis, "turret", m_configuration.m_chassisMass * 0.05f);
		dMatrix turretMatrix(m_localFrame * turretBody->GetMatrix());
		m_turretHinge = new ndJointHingeActuator(turretMatrix, 1.5f, -5000.0f * dDegreeToRad, 5000.0f * dDegreeToRad, turretBody, m_chassis);
		world->AddJoint(m_turretHinge);
		m_turretAngle0 = -dAtan2(turretMatrix[1][2], turretMatrix[1][0]);

		//cannon servo controller actuator
		ndBodyDynamic* const canonBody = MakeChildPart(scene, turretBody, "canon", m_configuration.m_chassisMass * 0.025f);
		dMatrix cannonMatrix(m_localFrame * canonBody->GetMatrix());
		m_cannonHinge = new ndJointHingeActuator(cannonMatrix, 1.5f, -45.0f * dDegreeToRad, 5.0f * dDegreeToRad, canonBody, turretBody);
		world->AddJoint(m_cannonHinge);
		dFloat32 y = cannonMatrix[1][1];
		dFloat32 x = dSqrt (cannonMatrix[1][0] * cannonMatrix[1][0] + cannonMatrix[1][2] * cannonMatrix[1][2] + 1.0e-6f);
		m_cannonAngle0 = -dAtan2(y, x);
	}

	void CreateEightWheelTruck (ndDemoEntityManager* const scene)
	{
		// 2- each tire to the model, 
		// this function will create the tire as a normal rigid body
		// and attach them to the chassis with the tire joints

		ndWorld* const world = scene->GetWorld();
		ndBodyDynamic* const chassis = m_chassis;

		ndBodyDynamic* const rr_tire0_body = CreateTireBody(scene, chassis, m_configuration.m_rearTire, "rtire_3");
		ndBodyDynamic* const rl_tire0_body = CreateTireBody(scene, chassis, m_configuration.m_rearTire, "ltire_3");
		ndBodyDynamic* const rr_tire1_body = CreateTireBody(scene, chassis, m_configuration.m_rearTire, "rtire_2");
		ndBodyDynamic* const rl_tire1_body = CreateTireBody(scene, chassis, m_configuration.m_rearTire, "ltire_2");

		ndBodyDynamic* const fr_tire0_body = CreateTireBody(scene, chassis, m_configuration.m_frontTire, "rtire_0");
		ndBodyDynamic* const fl_tire0_body = CreateTireBody(scene, chassis, m_configuration.m_frontTire, "ltire_0");
		ndBodyDynamic* const fr_tire1_body = CreateTireBody(scene, chassis, m_configuration.m_frontTire, "rtire_1");
		ndBodyDynamic* const fl_tire1_body = CreateTireBody(scene, chassis, m_configuration.m_frontTire, "ltire_1");

		ndWheelDescriptor tireInfo;
		tireInfo.m_springK = m_configuration.m_rearTire.m_springK;
		tireInfo.m_damperC = m_configuration.m_rearTire.m_damperC;
		tireInfo.m_regularizer = m_configuration.m_rearTire.m_regularizer;
		tireInfo.m_minLimit = m_configuration.m_rearTire.m_upperStop;
		tireInfo.m_maxLimit = m_configuration.m_rearTire.m_lowerStop;
		tireInfo.m_laterialStiffeness = m_configuration.m_rearTire.m_laterialStiffeness;
		tireInfo.m_longitudinalStiffeness = m_configuration.m_rearTire.m_longitudinalStiffeness;
		ndJointWheel* const rr_tire0 = AddTire(world, tireInfo, rr_tire0_body);
		ndJointWheel* const rl_tire0 = AddTire(world, tireInfo, rl_tire0_body);
		ndJointWheel* const rr_tire1 = AddTire(world, tireInfo, rr_tire1_body);
		ndJointWheel* const rl_tire1 = AddTire(world, tireInfo, rl_tire1_body);

		tireInfo.m_springK = m_configuration.m_frontTire.m_springK;
		tireInfo.m_damperC = m_configuration.m_frontTire.m_damperC;
		tireInfo.m_regularizer = m_configuration.m_frontTire.m_regularizer;
		tireInfo.m_minLimit = m_configuration.m_frontTire.m_upperStop;
		tireInfo.m_maxLimit = m_configuration.m_frontTire.m_lowerStop;
		tireInfo.m_laterialStiffeness = m_configuration.m_frontTire.m_laterialStiffeness;
		tireInfo.m_longitudinalStiffeness = m_configuration.m_frontTire.m_longitudinalStiffeness;
		ndJointWheel* const fr_tire0 = AddTire(world, tireInfo, fr_tire0_body);
		ndJointWheel* const fl_tire0 = AddTire(world, tireInfo, fl_tire0_body);
		ndJointWheel* const fr_tire1 = AddTire(world, tireInfo, fr_tire1_body);
		ndJointWheel* const fl_tire1 = AddTire(world, tireInfo, fl_tire1_body);

		m_gearMap[sizeof(m_configuration.m_transmission.m_fowardRatios) / sizeof(m_configuration.m_transmission.m_fowardRatios[0]) + 0] = 1;
		m_gearMap[sizeof(m_configuration.m_transmission.m_fowardRatios) / sizeof(m_configuration.m_transmission.m_fowardRatios[0]) + 1] = 0;
		for (int i = 0; i < m_configuration.m_transmission.m_gearsCount; i++)
		{
			m_gearMap[i] = i + 2;
		}
		m_currentGear = sizeof(m_configuration.m_transmission.m_fowardRatios) / sizeof(m_configuration.m_transmission.m_fowardRatios[0]) + 1;

		// configure vehicle steering
		SetAsSteering(fr_tire0);
		SetAsSteering(fl_tire0);
		SetAsSteering(fr_tire1);
		SetAsSteering(fl_tire1);

		// configure the tires brake
		SetAsBrake(rr_tire0);
		SetAsBrake(rl_tire0);
		SetAsBrake(fr_tire0);
		SetAsBrake(fl_tire0);
		SetAsBrake(rr_tire1);
		SetAsBrake(rl_tire1);
		SetAsBrake(fr_tire1);
		SetAsBrake(fl_tire1);

		// configure the tires hand brake
		SetAsHandBrake(rr_tire0);
		SetAsHandBrake(rl_tire0);
		SetAsHandBrake(rr_tire1);
		SetAsHandBrake(rl_tire1);

		// add the slip differential
#if 0
		ndMultiBodyVehicleDifferential* const rearDifferential0 = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire0, rr_tire0);
		ndMultiBodyVehicleDifferential* const rearDifferential1 = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire1, rr_tire1);

		ndMultiBodyVehicleDifferential* const frontDifferential0 = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, fl_tire0, fr_tire0);
		ndMultiBodyVehicleDifferential* const frontDifferential1 = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, fl_tire1, fr_tire1);

		ndMultiBodyVehicleDifferential* const rearDifferential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rearDifferential0, rearDifferential1);
		ndMultiBodyVehicleDifferential* const frontDifferential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, frontDifferential0, frontDifferential1);

		ndMultiBodyVehicleDifferential* const differential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rearDifferential, frontDifferential);
#else
		ndMultiBodyVehicleDifferential* const frontDifferential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, fl_tire0, fr_tire0);
		ndMultiBodyVehicleDifferential* const rearDifferential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire0, rr_tire0);
		ndMultiBodyVehicleDifferential* const differential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rearDifferential, frontDifferential);

		LinkTires(world, fl_tire0, fl_tire1);
		LinkTires(world, rl_tire0, rl_tire1);

		LinkTires(world, fr_tire0, fr_tire1);
		LinkTires(world, rr_tire0, rr_tire1);
#endif

		// add a motor
		ndMultiBodyVehicleMotor* const motor = AddMotor(world, m_configuration.m_motorMass, m_configuration.m_motorRadius);
		motor->SetRpmLimits(m_configuration.m_engine.GetIdleRadPerSec() * 9.55f, m_configuration.m_engine.GetRedLineRadPerSec() * 9.55f);

		// add the gear box
		AddGearBox(world, m_motor, differential);

		// add torsion bar
		ndMultiBodyVehicleTorsionBar* const torsionBar = AddTorsionBar(world);
		torsionBar->AddAxel(rl_tire0->GetBody0(), rr_tire0->GetBody0());
		torsionBar->AddAxel(fl_tire0->GetBody0(), fr_tire0->GetBody0());
		torsionBar->SetTorsionTorque(m_configuration.m_torsionBarSpringK, m_configuration.m_torsionBarDamperC, m_configuration.m_torsionBarRegularizer);

		// add vehicle turret
		CreateEightWheelTurret(scene);
	}

	void LinkTires(ndWorld* const world, const ndJointWheel* const tire0, const ndJointWheel* const tire1) const
	{
		ndBodyKinematic* const body0 = tire0->GetBody0();
		ndBodyKinematic* const body1 = tire1->GetBody0();
		
		ndShapeInfo rearInfo(body0->GetCollisionShape().GetShapeInfo());
		ndShapeInfo frontInfo(body1->GetCollisionShape().GetShapeInfo());
		dFloat32 tireRatio = frontInfo.m_scale.m_y / rearInfo.m_scale.m_y;

		dMatrix pin0(tire0->GetLocalMatrix0() * body0->GetMatrix());
		dMatrix pin1(tire1->GetLocalMatrix0() * body1->GetMatrix());
		world->AddJoint(new ndJointGear(tireRatio, pin0.m_front.Scale(-1.0f), body0, pin1.m_front, body1));
	}

	void CreateTractorBucket(ndDemoEntityManager* const scene)
	{
		ndWorld* const world = scene->GetWorld();

		//turret servo controller actuator
		ndBodyDynamic* const frontBucketArmBody = MakeChildPart(scene, m_chassis, "arms", m_configuration.m_chassisMass * 0.05f);
		dMatrix turretMatrix(m_localFrame * frontBucketArmBody->GetMatrix());
		m_turretHinge = new ndJointHingeActuator(turretMatrix, 1.5f, -10.0f * dDegreeToRad, 55.0f * dDegreeToRad, frontBucketArmBody, m_chassis);
		world->AddJoint(m_turretHinge);
		m_turretAngle0 = -dAtan2(turretMatrix[1][2], turretMatrix[1][0]);

		//cannon servo controller actuator
		//ndBodyDynamic* const canonBody = MakeChildPart(scene, turretBody, "canon", m_configuration.m_chassisMass * 0.025f);
		//dMatrix cannonMatrix(m_localFrame * canonBody->GetMatrix());
		//m_cannonHinge = new ndJointHingeActuator(cannonMatrix, 1.5f, -45.0f * dDegreeToRad, 5.0f * dDegreeToRad, canonBody, turretBody);
		//world->AddJoint(m_cannonHinge);
		//dFloat32 y = cannonMatrix[1][1];
		//dFloat32 x = dSqrt(cannonMatrix[1][0] * cannonMatrix[1][0] + cannonMatrix[1][2] * cannonMatrix[1][2] + 1.0e-6f);
		//m_cannonAngle0 = -dAtan2(y, x);
	}

	void CreateTractor(ndDemoEntityManager* const scene)
	{
		// 2- each tire to the model, 
		// this function will create the tire as a normal rigid body
		// and attach them to the chassis with the tire joints

		ndWorld* const world = scene->GetWorld();
		ndBodyDynamic* const chassis = m_chassis;

		ndBodyDynamic* const rr_tire0_body = CreateTireBody(scene, chassis, m_configuration.m_rearTire, "rr_tire");
		ndBodyDynamic* const rl_tire0_body = CreateTireBody(scene, chassis, m_configuration.m_rearTire, "rl_tire");

		ndBodyDynamic* const frontAxel_body = MakeFronAxel(scene, chassis);
		ndBodyDynamic* const fr_tire0_body = CreateTireBody(scene, frontAxel_body, m_configuration.m_frontTire, "fr_tire");
		ndBodyDynamic* const fl_tire0_body = CreateTireBody(scene, frontAxel_body, m_configuration.m_frontTire, "fl_tire");

		ndWheelDescriptor tireInfo;
		tireInfo.m_springK = m_configuration.m_rearTire.m_springK;
		tireInfo.m_damperC = m_configuration.m_rearTire.m_damperC;
		tireInfo.m_regularizer = m_configuration.m_rearTire.m_regularizer;
		tireInfo.m_minLimit = m_configuration.m_rearTire.m_upperStop;
		tireInfo.m_maxLimit = m_configuration.m_rearTire.m_lowerStop;
		tireInfo.m_laterialStiffeness = m_configuration.m_rearTire.m_laterialStiffeness;
		tireInfo.m_longitudinalStiffeness = m_configuration.m_rearTire.m_longitudinalStiffeness;
		ndJointWheel* const rr_tire0 = AddTire(world, tireInfo, rr_tire0_body);
		ndJointWheel* const rl_tire0 = AddTire(world, tireInfo, rl_tire0_body);

		tireInfo.m_springK = m_configuration.m_frontTire.m_springK;
		tireInfo.m_damperC = m_configuration.m_frontTire.m_damperC;
		tireInfo.m_regularizer = m_configuration.m_frontTire.m_regularizer;
		tireInfo.m_minLimit = m_configuration.m_frontTire.m_upperStop;
		tireInfo.m_maxLimit = m_configuration.m_frontTire.m_lowerStop;
		tireInfo.m_laterialStiffeness = m_configuration.m_frontTire.m_laterialStiffeness;
		tireInfo.m_longitudinalStiffeness = m_configuration.m_frontTire.m_longitudinalStiffeness;
		ndJointWheel* const fr_tire0 = AddAxleTire(world, tireInfo, fr_tire0_body, frontAxel_body);
		ndJointWheel* const fl_tire0 = AddAxleTire(world, tireInfo, fl_tire0_body, frontAxel_body);

		m_gearMap[sizeof(m_configuration.m_transmission.m_fowardRatios) / sizeof(m_configuration.m_transmission.m_fowardRatios[0]) + 0] = 1;
		m_gearMap[sizeof(m_configuration.m_transmission.m_fowardRatios) / sizeof(m_configuration.m_transmission.m_fowardRatios[0]) + 1] = 0;
		for (int i = 0; i < m_configuration.m_transmission.m_gearsCount; i++)
		{
			m_gearMap[i] = i + 2;
		}
		m_currentGear = sizeof(m_configuration.m_transmission.m_fowardRatios) / sizeof(m_configuration.m_transmission.m_fowardRatios[0]) + 1;

		// configure vehicle steering
		SetAsSteering(fr_tire0);
		SetAsSteering(fl_tire0);

		// configure the tires brake
		SetAsBrake(rr_tire0);
		SetAsBrake(rl_tire0);
		SetAsBrake(fr_tire0);
		SetAsBrake(fl_tire0);

#if 0
		// add the slip differential
		ndMultiBodyVehicleDifferential* const rearDifferential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire0, rr_tire0);
		ndMultiBodyVehicleDifferential* const frontDifferential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, fl_tire0, fr_tire0);
		ndMultiBodyVehicleDifferential* const differential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rearDifferential, frontDifferential);
		differential->SetSlipOmega(100.0f);
#else
		ndMultiBodyVehicleDifferential* const differential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire0, rr_tire0);
		LinkTires(world, rl_tire0, fl_tire0);
		LinkTires(world, rr_tire0, fr_tire0);
#endif
		
		// add a motor
		ndMultiBodyVehicleMotor* const motor = AddMotor(world, m_configuration.m_motorMass, m_configuration.m_motorRadius);
		motor->SetRpmLimits(m_configuration.m_engine.GetIdleRadPerSec() * 9.55f, m_configuration.m_engine.GetRedLineRadPerSec() * 9.55f);
		
		// add the gear box
		AddGearBox(world, m_motor, differential);
		
		// add torsion bar
		//ndMultiBodyVehicleTorsionBar* const torsionBar = AddTorsionBar(world);
		//torsionBar->AddAxel(rl_tire0->GetBody0(), rr_tire0->GetBody0());
		//torsionBar->AddAxel(fl_tire0->GetBody0(), fr_tire0->GetBody0());
		//torsionBar->SetTorsionTorque(m_configuration.m_torsionBarSpringK, m_configuration.m_torsionBarDamperC, m_configuration.m_torsionBarRegularizer);

		CreateTractorBucket(scene);
	}

	ndBodyDynamic* CreateChassis(ndDemoEntityManager* const scene, ndDemoEntity* const chassisEntity, dFloat32 mass)
	{
		dMatrix matrix(chassisEntity->CalculateGlobalMatrix(nullptr));

		ndWorld* const world = scene->GetWorld();
		ndShapeInstance* const chassisCollision = chassisEntity->CreateCollisionFromchildren(scene->GetWorld());

		ndBodyDynamic* const body = new ndBodyDynamic();
		body->SetNotifyCallback(new ndDemoEntityNotify(scene, chassisEntity));
		body->SetMatrix(matrix);
		body->SetCollisionShape(*chassisCollision);
		body->SetMassMatrix(mass, *chassisCollision);

		world->AddBody(body);
		delete chassisCollision;
		return body;
	}

	void CalculateTireDimensions(const char* const tireName, dFloat32& width, dFloat32& radius, ndDemoEntity* const vehEntity)
	{
		// find the the tire visual mesh 
		ndDemoEntity* const tirePart = vehEntity->Find(tireName);
		dAssert(tirePart);

		// make a convex hull collision shape to assist in calculation of the tire shape size
		ndDemoMesh* const tireMesh = (ndDemoMesh*)tirePart->GetMesh();

		const dMatrix matrix(tirePart->GetMeshMatrix());

		dArray<dVector> temp;
		tireMesh->GetVertexArray(temp);

		dVector minVal(1.0e10f);
		dVector maxVal(-1.0e10f);
		for (dInt32 i = 0; i < temp.GetCount(); i++)
		{
			dVector p(matrix.TransformVector(temp[i]));
			minVal = minVal.GetMin(p);
			maxVal = maxVal.GetMax(p);
		}

		dVector size(maxVal - minVal);
		width = size.m_x;
		radius = size.m_y * 0.5f;
	}

	ndBodyDynamic* CreateTireBody(ndDemoEntityManager* const scene, ndBodyDynamic* const parentBody, const ndVehicleDectriptor::ndTireDefinition& definition, const char* const tireName)
	{
		dFloat32 width;
		dFloat32 radius;
		ndWorld* const world = scene->GetWorld();
		ndDemoEntity* const parentEntity = (ndDemoEntity*)parentBody->GetNotifyCallback()->GetUserData();
		CalculateTireDimensions(tireName, width, radius, parentEntity);

		ndShapeInstance tireCollision(CreateTireShape(radius, width));

		ndDemoEntity* const tireEntity = parentEntity->Find(tireName);
		dMatrix matrix(tireEntity->CalculateGlobalMatrix(nullptr));

		const dMatrix chassisMatrix(m_localFrame * m_chassis->GetMatrix());
		matrix.m_posit += chassisMatrix.m_up.Scale (definition.m_verticalOffset);

		ndBodyDynamic* const tireBody = new ndBodyDynamic();
		tireBody->SetNotifyCallback(new ndTireNotifyNotify(scene, tireEntity, parentBody));
		tireBody->SetMatrix(matrix);
		tireBody->SetCollisionShape(tireCollision);
		tireBody->SetMassMatrix(definition.m_mass, tireCollision);

		world->AddBody(tireBody);
		return tireBody;
	}

	ndBodyDynamic* MakeChildPart(ndDemoEntityManager* const scene, ndBodyDynamic* const parentBody, const char* const partName, dFloat32 mass) const
	{
		ndWorld* const world = scene->GetWorld();
		ndDemoEntity* const parentEntity = (ndDemoEntity*)parentBody->GetNotifyCallback()->GetUserData();

		ndDemoEntity* const vehPart = parentEntity->Find(partName);
		ndShapeInstance* const vehCollision = vehPart->CreateCollisionFromchildren(scene->GetWorld());

		ndBodyDynamic* const vehBody = new ndBodyDynamic();
		const dMatrix matrix(vehPart->CalculateGlobalMatrix(nullptr));
		vehBody->SetNotifyCallback(new ndTireNotifyNotify(scene, vehPart, parentBody));
		vehBody->SetMatrix(matrix);
		vehBody->SetCollisionShape(*vehCollision);
		vehBody->SetMassMatrix(mass, *vehCollision);
		world->AddBody(vehBody);

		delete vehCollision;
		return vehBody;
	}

	ndBodyDynamic* MakeFronAxel(ndDemoEntityManager* const scene, ndBodyDynamic* const chassis)
	{
		ndBodyDynamic* const axleBody = MakeChildPart(scene, m_chassis, "front_axel", m_configuration.m_chassisMass * 0.2f);

		// connect the part to the main body with a hinge
		ndWorld* const world = scene->GetWorld();
		dMatrix hingeFrame(m_localFrame * axleBody->GetMatrix());
		ndJointHinge* const hinge = new ndJointHinge(hingeFrame, axleBody, chassis);
		world->AddJoint(hinge);
		hinge->EnableLimits(true, -15.0f * dDegreeToRad, 15.0f * dDegreeToRad);
		return axleBody;
	}

	static void UpdateCameraCallback(ndDemoEntityManager* const manager, void* const context, dFloat32 timestep)
	{
		ndHeavyMultiBodyVehicle* const me = (ndHeavyMultiBodyVehicle*)context;
		me->SetCamera(manager, timestep);
	}

	void SetCamera(ndDemoEntityManager* const manager, dFloat32)
	{
		ndDemoCamera* const camera = manager->GetCamera();
		ndDemoEntity* const chassisEntity = (ndDemoEntity*)m_chassis->GetNotifyCallback()->GetUserData();
		dMatrix camMatrix(camera->GetNextMatrix());
		dMatrix playerMatrix(chassisEntity->GetNextMatrix());

		dVector frontDir(camMatrix[0]);
		dVector camOrigin(0.0f);
		camOrigin = playerMatrix.m_posit + dVector(0.0f, 1.0f, 0.0f, 0.0f);
		camOrigin -= frontDir.Scale(10.0f);

		camera->SetNextMatrix(camMatrix, camOrigin);
	}

	void DrawGage(GLuint gage, GLuint needle, dFloat32 param, dFloat32 origin_x, dFloat32 origin_y, dFloat32 size, dFloat32 minAngle, dFloat32 maxAngle) const
	{
		dMatrix origin(dGetIdentityMatrix());
		origin[1][1] = -1.0f;
		origin.m_posit = dVector(origin_x, origin_y, 0.0f, 1.0f);

		size *= 0.5f;

		// render dial
		glPushMatrix();
		glMultMatrix(&origin[0][0]);
		glBindTexture(GL_TEXTURE_2D, gage);
		glBegin(GL_QUADS);
		glTexCoord2f(0.0f, 1.0f); glVertex3f(GLfloat(-size), GLfloat(size), 0.0f);
		glTexCoord2f(0.0f, 0.0f); glVertex3f(GLfloat(-size), GLfloat(-size), 0.0f);
		glTexCoord2f(1.0f, 0.0f); glVertex3f(GLfloat(size), GLfloat(-size), 0.0f);
		glTexCoord2f(1.0f, 1.0f); glVertex3f(GLfloat(size), GLfloat(size), 0.0f);
		glEnd();

		// render needle
		minAngle *= -dDegreeToRad;
		maxAngle *= -dDegreeToRad;
		//param = 1.0f;
		dFloat32 angle = minAngle + (maxAngle - minAngle) * param;
		dMatrix needleMatrix(dRollMatrix(angle));

		dFloat32 x = size * 0.7f;
		dFloat32 y = size * 0.7f;

		glPushMatrix();
		glMultMatrix(&needleMatrix[0][0]);
		glBindTexture(GL_TEXTURE_2D, needle);
		glBegin(GL_QUADS);
		glTexCoord2f(0.0f, 1.0f); glVertex3f(GLfloat(-x), GLfloat(y), 0.0f);
		glTexCoord2f(0.0f, 0.0f); glVertex3f(GLfloat(-x), GLfloat(-y), 0.0f);
		glTexCoord2f(1.0f, 0.0f); glVertex3f(GLfloat(x), GLfloat(-y), 0.0f);
		glTexCoord2f(1.0f, 1.0f); glVertex3f(GLfloat(x), GLfloat(y), 0.0f);
		glEnd();

		glPopMatrix();
		glPopMatrix();
	}

	void DrawGear(dInt32 gear, dFloat32 origin_x, dFloat32 origin_y, dFloat32 size) const
	{
		dMatrix origin(dGetIdentityMatrix());
		origin[1][1] = -1.0f;
		origin.m_posit = dVector(origin_x + size * 0.3f, origin_y - size * 0.25f, 0.0f, 1.0f);

		glPushMatrix();
		glMultMatrix(&origin[0][0]);

		dFloat32 uwith = 0.1f;
		dFloat32 u0 = uwith * gear;
		dFloat32 u1 = u0 + uwith;

		dFloat32 x1 = 10.0f;
		dFloat32 y1 = 10.0f;
		glColor4f(1, 1, 0, 1);
		glBindTexture(GL_TEXTURE_2D, m_gears);
		glBegin(GL_QUADS);
		glTexCoord2f(GLfloat(u0), 1.0f); glVertex3f(GLfloat(-x1), GLfloat(y1), 0.0f);
		glTexCoord2f(GLfloat(u0), 0.0f); glVertex3f(GLfloat(-x1), GLfloat(-y1), 0.0f);
		glTexCoord2f(GLfloat(u1), 0.0f); glVertex3f(GLfloat(x1), GLfloat(-y1), 0.0f);
		glTexCoord2f(GLfloat(u1), 1.0f); glVertex3f(GLfloat(x1), GLfloat(y1), 0.0f);
		glEnd();

		glPopMatrix();
	}

	void RenderHelp(ndDemoEntityManager* const scene)
	{
		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
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
			dAssert(motor);

			glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
			dFloat32 gageSize = 200.0f;
			dFloat32 y = scene->GetHeight() - (gageSize / 2.0f + 20.0f);

			// draw the tachometer
			dFloat32 x = gageSize / 2 + 20.0f;
			dFloat32 maxRpm = 9000.0f;
			dFloat32 rpm = motor->GetRpm() / maxRpm;

			DrawGage(m_tachometer, m_redNeedle, rpm, x, y, gageSize, -180.0f, 90.0f);

			// draw the odometer
			x += gageSize;
			dFloat32 speed = GetSpeed() / 100.0f;
			DrawGage(m_odometer, m_greenNeedle, dAbs(speed), x, y, gageSize, -180.0f, 90.0f);

			// draw the current gear
			DrawGear(m_gearMap[m_currentGear], x, y + 98, gageSize);
		}
	}

	void MillitaryControl(ndWorld* const world)
	{
		// get the inputs to traget angles
		ndDemoEntityManager* const scene = ((ndPhysicsWorld*)world)->GetManager();
		dFixSizeBuffer<char, 32> buttons;
		scene->GetJoystickButtons(buttons);

		if (buttons[2])
		{
			m_turretAngle += 5.0e-3f;
		}
		else if (buttons[1])
		{
			m_turretAngle -= 5.0e-3f;
		}

		if (buttons[0])
		{
			m_cannonAngle += 1.0e-3f;
			if (m_cannonAngle > m_cannonHinge->GetMaxAngularLimit())
			{
				m_cannonAngle = m_cannonHinge->GetMaxAngularLimit();
			}
		}
		else if (buttons[3])
		{
			m_cannonAngle -= 1.0e-3f;
			if (m_cannonAngle < m_cannonHinge->GetMinAngularLimit())
			{
				m_cannonAngle = m_cannonHinge->GetMinAngularLimit();
			}
		}

		// apply inputs to actuators joint
		const dMatrix turretMatrix(m_turretHinge->GetLocalMatrix0() * m_turretHinge->GetBody0()->GetMatrix());
		dFloat32 turretAngle = -dAtan2(turretMatrix[1][2], turretMatrix[1][0]);
		dFloat32 turretErrorAngle = AnglesAdd(AnglesAdd (m_turretAngle, m_turretAngle0), - turretAngle);
		dFloat32 turretTargetAngle = m_turretHinge->GetAngle();
		if (dAbs(turretErrorAngle) > (0.25f * dDegreeToRad))
		{
			turretTargetAngle += turretErrorAngle;
		}
		m_turretHinge->SetTargetAngle(turretTargetAngle);
		//dTrace(("errorAngle:%f  turretAngle:%f\n", turretErrorAngle * dRadToDegree, turretAngle * dRadToDegree));

		const dMatrix cannonMatrix(m_cannonHinge->GetLocalMatrix0() * m_cannonHinge->GetBody0()->GetMatrix());
		dFloat32 y = cannonMatrix[1][1];
		dFloat32 x = dSqrt(cannonMatrix[1][0] * cannonMatrix[1][0] + cannonMatrix[1][2] * cannonMatrix[1][2] + 1.0e-6f);
		dFloat32 cannonAngle = -dAtan2(y, x);
		dFloat32 cannonErrorAngle = AnglesAdd(AnglesAdd(m_cannonAngle, m_cannonAngle0), -cannonAngle);
		
		dFloat32 cannonTargetAngle = m_cannonHinge->GetAngle();
		const dFloat32 error = 0.125f * dDegreeToRad;
		if (dAbs(cannonErrorAngle) > error)
		{
			cannonTargetAngle += cannonErrorAngle;
		}
		m_cannonHinge->SetTargetAngle(cannonTargetAngle);
		//dTrace(("errorAngle:%f  cannonAngle:%f\n", cannonErrorAngle * dRadToDegree, cannonAngle * dRadToDegree));
	}

	void TractorControl(ndWorld* const world)
	{
		ndDemoEntityManager* const scene = ((ndPhysicsWorld*)world)->GetManager();
		dFixSizeBuffer<char, 32> buttons;
		scene->GetJoystickButtons(buttons);

		if (buttons[2])
		{
			m_turretAngle += 5.0e-3f;
			if (m_turretAngle > m_turretHinge->GetMaxAngularLimit())
			{
				m_turretAngle = m_turretHinge->GetMaxAngularLimit();
			}
		}
		else if (buttons[1])
		{
			m_turretAngle -= 5.0e-3f;
			if (m_turretAngle < m_turretHinge->GetMinAngularLimit())
			{
				m_turretAngle = m_turretHinge->GetMinAngularLimit();
			}
		}

		//const dMatrix turretMatrix(m_turretHinge->GetLocalMatrix0() * m_turretHinge->GetBody0()->GetMatrix());
		//dFloat32 turretAngle = -dAtan2(turretMatrix[1][2], turretMatrix[1][0]);
		//dFloat32 turretErrorAngle = AnglesAdd(AnglesAdd(m_turretAngle, m_turretAngle0), -turretAngle);
		//dFloat32 turretTargetAngle = m_turretHinge->GetAngle();
		//if (dAbs(turretErrorAngle) > (0.25f * dDegreeToRad))
		//{
		//	turretTargetAngle += turretErrorAngle;
		//}
		m_turretHinge->SetTargetAngle(m_turretAngle + m_turretAngle0);
		//dTrace(("errorAngle:%f  turretAngle:%f\n", turretErrorAngle * dRadToDegree, turretAngle * dRadToDegree));

	}

	void Update(ndWorld* const world, dFloat32 timestep)
	{
		ndBasicVehicle::Update(world, timestep);
		if (!strcmp(m_configuration.m_name, "lav_25.fbx"))
		{
			MillitaryControl(world);
		}
		else if (!strcmp(m_configuration.m_name, "tractor.fbx"))
		{
			TractorControl(world);
		}
	}
	
	GLuint m_gears;
	GLuint m_odometer;
	GLuint m_redNeedle;
	GLuint m_tachometer;
	GLuint m_greenNeedle;
	dInt32 m_gearMap[8];

	ndJointHingeActuator* m_turretHinge;
	ndJointHingeActuator* m_cannonHinge;
	dFloat32 m_turretAngle;
	dFloat32 m_turretAngle0;
	dFloat32 m_cannonAngle;
	dFloat32 m_cannonAngle0;
};

void ndHeavyVehicle (ndDemoEntityManager* const scene)
{
	// build a floor
	//BuildFloorBox(scene);
	//BuildFlatPlane(scene, true);
	//BuildStaticMesh(scene, "track.fbx", true);
	BuildStaticMesh(scene, "playerarena.fbx", true);

	dVector location(0.0f, 2.0f, 0.0f, 1.0f);

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = location;

	// add a model for general controls
	ndVehicleSelector* const controls = new ndVehicleSelector();
	scene->GetWorld()->AddModel(controls);

	ndHeavyMultiBodyVehicle* const vehicle = new ndHeavyMultiBodyVehicle(scene, tractorDesc, matrix);
	//ndHeavyMultiBodyVehicle* const vehicle = new ndHeavyMultiBodyVehicle(scene, lav25Desc, matrix);
	scene->GetWorld()->AddModel(vehicle);
	vehicle->SetAsPlayer(scene);

	matrix.m_posit.m_x += 8.0f;
	matrix.m_posit.m_z += 2.0f;
	//scene->GetWorld()->AddModel(new ndHeavyMultiBodyVehicle(scene, lav25Desc, matrix));
	//for (dInt32 i = 0; i < 10; i++)
	//{
	//	matrix.m_posit.m_y += 4.0f;
	//	scene->GetWorld()->AddModel(new ndHeavyMultiBodyVehicle(scene, lav25Desc, matrix));
	//}
	
	matrix.m_posit.m_x += 15.0f;
	//AddPlanks(scene, matrix.m_posit, 300.0f);

	scene->Set2DDisplayRenderFunction(ndHeavyMultiBodyVehicle::RenderHelp, ndHeavyMultiBodyVehicle::RenderUI, vehicle);

	dQuaternion rot;
	dVector origin(-10.0f, 2.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
