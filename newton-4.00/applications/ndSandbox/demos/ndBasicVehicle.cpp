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
		m_comDisplacement = dVector(0.25f, -0.35f, 0.0f, 0.0f);

		dFloat32 fuelInjectionRate = 10.0f;
		dFloat32 idleTorquePoundFoot = 100.0f;
		dFloat32 idleRmp = 700.0f;
		dFloat32 horsePower = 400.0f;
		dFloat32 rpm0 = 5000.0f;
		dFloat32 rpm1 = 6200.0f;
		dFloat32 horsePowerAtRedLine = 100.0f;
		dFloat32 redLineRpm = 8000.0f;
		m_engine.Init(fuelInjectionRate, idleTorquePoundFoot, idleRmp, 
					  horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);
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
		//m_comDisplacement = dVector(0.0f, -0.55f, 0.0f, 0.0f);
		m_comDisplacement = dVector(0.0f, -0.35f, 0.0f, 0.0f);

		dFloat32 fuelInjectionRate = 10.0f;
		dFloat32 idleTorquePoundFoot = 200.0f;
		dFloat32 idleRmp = 800.0f;
		dFloat32 horsePower = 400.0f;
		dFloat32 rpm0 = 5000.0f;
		dFloat32 rpm1 = 6200.0f;
		dFloat32 horsePowerAtRedLine = 400.0f;
		dFloat32 redLineRpm = 8000.0f;
		m_engine.Init(fuelInjectionRate, idleTorquePoundFoot, idleRmp, 
					  horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);

		m_frontTire.m_mass = 100.0f;
		m_frontTire.m_verticalOffset = -0.15f;
		m_frontTire.m_steeringAngle = 35.0f * dDegreeToRad;
		m_frontTire.m_springK = 800.0f;
		m_frontTire.m_damperC = 50.0f;
		m_frontTire.m_regularizer = 0.3f;
		m_frontTire.m_upperStop = -0.05f;
		m_frontTire.m_lowerStop = 0.4f;
		m_frontTire.m_brakeTorque = 1500.0f;
		m_rearTire.m_handBrakeTorque = 0.0f;
		m_frontTire.m_laterialStiffness  = 1.0f / 1000.0f;
		m_frontTire.m_longitudinalStiffness  = 50.0f / 1000.0f;

		m_rearTire.m_mass = 100.0f;
		m_rearTire.m_verticalOffset = -0.15f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 800.0f;
		m_rearTire.m_damperC = 50.0f;
		m_rearTire.m_regularizer = 0.3f;
		m_rearTire.m_upperStop = -0.05f;
		m_rearTire.m_lowerStop = 0.4f;
		m_rearTire.m_brakeTorque = 3000.0f;
		m_rearTire.m_handBrakeTorque = 4000.0f;
		m_rearTire.m_laterialStiffness  = 0.3f / 1000.0f;
		m_rearTire.m_longitudinalStiffness  = 50.0f / 1000.0f;
		
		m_frictionCoefficientScale = 1.2f;
		m_torsionBarType = m_fourWheelAxle;
		m_differentialType = m_fourWheeldrive;
	}
};

class ndVehicleDectriptorMonsterTruck: public ndVehicleDectriptor
{
	public:
	ndVehicleDectriptorMonsterTruck()
		:ndVehicleDectriptor("monsterTruck.fbx")
	{
		m_comDisplacement = dVector(0.0f, -0.55f, 0.0f, 0.0f);

		dFloat32 fuelInjectionRate = 10.0f;
		dFloat32 idleTorquePoundFoot = 250.0f;
		dFloat32 idleRmp = 800.0f;
		dFloat32 horsePower = 400.0f;
		dFloat32 rpm0 = 5000.0f;
		dFloat32 rpm1 = 6200.0f;
		dFloat32 horsePowerAtRedLine = 150.0f;
		dFloat32 redLineRpm = 8000.0f;
		m_engine.Init(fuelInjectionRate, idleTorquePoundFoot, idleRmp, 
					  horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);

		m_frontTire.m_mass = 100.0f;
		m_frontTire.m_verticalOffset = 0.0f;
		m_frontTire.m_steeringAngle = 35.0f * dDegreeToRad;
		m_frontTire.m_springK = 500.0f;
		m_frontTire.m_damperC = 50.0f;
		m_frontTire.m_regularizer = 0.2f;
		m_frontTire.m_upperStop = -0.05f;
		m_frontTire.m_lowerStop = 0.4f;
		m_frontTire.m_brakeTorque = 1000.0f;
		m_frontTire.m_handBrakeTorque = 0.0f;
		m_frontTire.m_laterialStiffness  = 1.0f / 1000.0f;
		m_frontTire.m_longitudinalStiffness  = 50.0f / 1000.0f;

		m_rearTire.m_mass = 100.0f;
		m_rearTire.m_verticalOffset = 0.0f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 500.0f;
		m_rearTire.m_damperC = 50.0f;
		m_rearTire.m_regularizer = 0.2f;
		m_rearTire.m_upperStop = -0.05f;
		m_rearTire.m_lowerStop = 0.4f;
		m_rearTire.m_brakeTorque = 2000.0f;
		m_rearTire.m_handBrakeTorque = 3000.0f;
		m_rearTire.m_laterialStiffness  = 1.0f / 1000.0f;
		m_rearTire.m_longitudinalStiffness  = 50.0f / 1000.0f;
		
		m_frictionCoefficientScale = 1.3f;
	
		m_torsionBarType = m_fourWheelAxle;
		m_differentialType = m_fourWheeldrive;
	}
};

static ndVehicleDectriptorJeep jeepDesc;
static ndVehicleDectriptorViper viperDesc;
static ndVehicleDectriptorMonsterTruck monterTruckDesc;

static const char* engineSounds[] =
{
	"engine_start.wav",
	"engine_rpm.wav",
	"tire_skid.wav",
};

class ndBasicMultiBodyVehicle : public ndBasicVehicle
{
	public:
	ndBasicMultiBodyVehicle(ndDemoEntityManager* const scene, const ndVehicleDectriptor& desc, const dMatrix& matrix)
		:ndBasicVehicle(desc)
		,m_vehicleUI(nullptr)
	{
		m_vehicleUI = new ndVehicleUI();
		m_vehicleUI->CreateBufferUI();

		ndDemoEntity* const vehicleEntity = LoadMeshModel(scene, desc.m_name);
		vehicleEntity->ResetMatrix(vehicleEntity->CalculateGlobalMatrix() * matrix);

		ndPhysicsWorld* const world = scene->GetWorld();

		// create the vehicle chassis as a normal rigid body
		ndBodyDynamic* const chassis = CreateChassis(scene, vehicleEntity, m_configuration.m_chassisMass);
		chassis->SetAngularDamping(dVector(m_configuration.m_chassisAngularDrag));

		// lower vehicle com;
		dVector com(chassis->GetCentreOfMass());
		com += m_localFrame.m_up.Scale(m_configuration.m_comDisplacement.m_y);
		com += m_localFrame.m_front.Scale(m_configuration.m_comDisplacement.m_x);
		com += m_localFrame.m_right.Scale(m_configuration.m_comDisplacement.m_z);
		chassis->SetCentreOfMass(com);

		// 1- add chassis to the vehicle mode 
		AddChassis(chassis);

		// 2- each tire to the model, 
		// create the tire as a normal rigid body
		// and attach them to the chassis with a tire joints
		ndBodyDynamic* const rr_tire_body = CreateTireBody(scene, chassis, m_configuration.m_rearTire, "rr_tire");
		ndBodyDynamic* const rl_tire_body = CreateTireBody(scene, chassis, m_configuration.m_rearTire, "rl_tire");
		ndMultiBodyVehicleTireJoint* const rr_tire = AddTire(world, m_configuration.m_rearTire, rr_tire_body);
		ndMultiBodyVehicleTireJoint* const rl_tire = AddTire(world, m_configuration.m_rearTire, rl_tire_body);

		ndBodyDynamic* const fr_tire_body = CreateTireBody(scene, chassis, m_configuration.m_frontTire, "fr_tire");
		ndBodyDynamic* const fl_tire_body = CreateTireBody(scene, chassis, m_configuration.m_frontTire, "fl_tire");
		ndMultiBodyVehicleTireJoint* const fr_tire = AddTire(world, m_configuration.m_frontTire, fr_tire_body);
		ndMultiBodyVehicleTireJoint* const fl_tire = AddTire(world, m_configuration.m_frontTire, fl_tire_body);

		m_gearMap[sizeof(m_configuration.m_transmission.m_fowardRatios) / sizeof(m_configuration.m_transmission.m_fowardRatios[0]) + 0] = 1;
		m_gearMap[sizeof(m_configuration.m_transmission.m_fowardRatios) / sizeof(m_configuration.m_transmission.m_fowardRatios[0]) + 1] = 0;
		for (int i = 0; i < m_configuration.m_transmission.m_gearsCount; i++)
		{
			m_gearMap[i] = i + 2;
		}
		m_currentGear = sizeof(m_configuration.m_transmission.m_fowardRatios) / sizeof(m_configuration.m_transmission.m_fowardRatios[0]) + 1;

		// add the slip differential
		ndMultiBodyVehicleDifferential* differential = nullptr;
		switch (m_configuration.m_differentialType)
		{
			case ndVehicleDectriptor::m_rearWheelDrive:
			{
				differential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire, rr_tire, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
				break;
			}

			case ndVehicleDectriptor::m_frontWheelDrive:
			{
				differential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, fl_tire, fr_tire, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
				break;
			}

			case ndVehicleDectriptor::m_fourWheeldrive:
			{
				ndMultiBodyVehicleDifferential* const rearDifferential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire, rr_tire, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
				ndMultiBodyVehicleDifferential* const frontDifferential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, fl_tire, fr_tire, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
				differential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rearDifferential, frontDifferential, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
				break;
			}

			case ndVehicleDectriptor::m_eightWheeldrive:
			{
				dAssert(0);
				break;
			}
		}

		// add a motor
		ndMultiBodyVehicleMotor* const motor = AddMotor(world, m_configuration.m_motorMass, m_configuration.m_motorRadius);
		motor->SetRpmLimits(m_configuration.m_engine.GetIdleRadPerSec() * 9.55f, m_configuration.m_engine.GetRedLineRadPerSec() * dRadPerSecToRpm);

		// add the gear box
		AddGearBox(world, m_motor, differential);

		switch (m_configuration.m_torsionBarType)
		{
			case ndVehicleDectriptor::m_noWheelAxle:
			{
				// no torsion bar
				break;
			}

			case ndVehicleDectriptor::m_rearWheelAxle:
			{
				ndMultiBodyVehicleTorsionBar* const torsionBar = AddTorsionBar(world);
				torsionBar->AddAxel(rl_tire->GetBody0(), rr_tire->GetBody0());
				torsionBar->SetTorsionTorque(m_configuration.m_torsionBarSpringK, m_configuration.m_torsionBarDamperC, m_configuration.m_torsionBarRegularizer);
				break;
			}

			case ndVehicleDectriptor::m_frontWheelAxle:
			{
				ndMultiBodyVehicleTorsionBar* const torsionBar = AddTorsionBar(world);
				torsionBar->AddAxel(fl_tire->GetBody0(), fr_tire->GetBody0());
				torsionBar->SetTorsionTorque(m_configuration.m_torsionBarSpringK, m_configuration.m_torsionBarDamperC, m_configuration.m_torsionBarRegularizer);
				break;
			}

			case ndVehicleDectriptor::m_fourWheelAxle:
			{
				ndMultiBodyVehicleTorsionBar* const torsionBar = AddTorsionBar(world);
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
		m_startSound = soundManager->CreateSoundChannel(engineSounds[0]);
		m_engineRpmSound = soundManager->CreateSoundChannel(engineSounds[1]);
		m_skipMarks = soundManager->CreateSoundChannel(engineSounds[2]);

		m_startSound->SetAttenuationRefDistance(20.0f, 40.0f, 50.0f);
		m_engineRpmSound->SetAttenuationRefDistance(20.0f, 40.0f, 50.0f);

		m_engineRpmSound->SetLoop(true);
		m_skipMarks->SetLoop(true);
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
		ndBasicMultiBodyVehicle* const me = (ndBasicMultiBodyVehicle*)context;
		me->RenderUI(scene);
	}

	static void RenderHelp(ndDemoEntityManager* const scene, void* const context)
	{
		ndBasicMultiBodyVehicle* const me = (ndBasicMultiBodyVehicle*)context;
		me->RenderHelp(scene);
	}

	private:
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

	static void UpdateCameraCallback(ndDemoEntityManager* const manager, void* const context, dFloat32 timestep)
	{
		ndBasicMultiBodyVehicle* const me = (ndBasicMultiBodyVehicle*)context;
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
			dFloat32 maxRpm = m_configuration.m_engine.GetRedLineRadPerSec() * dRadPerSecToRpm;
			maxRpm += 500.0f;
			//printf("%.3f \n", m_configuration.m_transmission.m_fowardRatios[m_currentGear]);
			//dFloat32 rpm = (motor->GetRpm() / maxRpm) * m_configuration.m_transmission.m_fowardRatios[m_currentGear]; 
			dFloat32 rpm = (motor->GetRpm() / maxRpm) * 2.85f;
			
			if (m_vehicleUI) 
			{
				if (m_vehicleUI->m_shaderHandle) 
				{
					glUseProgram(m_vehicleUI->m_shaderHandle);
					//
					glActiveTexture(GL_TEXTURE0);
					//
					m_vehicleUI->RenderGageUI(scene, m_tachometer, -x, -y, gageSize * 0.5f, 0.0f, -180.0f, 90.0f);

					dFloat32 s = gageSize * 0.7f;
					m_vehicleUI->RenderGageUI(scene, m_redNeedle, -x, -y, s * 0.5f, rpm, -0.0f, 90.0f);
					
					x += gageSize;
					m_vehicleUI->RenderGageUI(scene, m_odometer, -x, -y, gageSize * 0.5f, 0.0f, -180.0f, 90.0f);

					dFloat32 speed = (GetSpeed() / 100.0f) * 2.85f;
					m_vehicleUI->RenderGageUI(scene, m_greenNeedle, -x, -y, s * 0.5f, dAbs(speed), -0.0f, 90.0f);

					// draw the current gear
					m_vehicleUI->RenderGearUI(scene, m_gearMap[m_currentGear], m_gears, -x, -y, gageSize);

					glUseProgram(0);
				}
			}
		}
	}

	void ApplyInputs(ndWorld* const world, dFloat32 timestep)
	{
		ndBasicVehicle::ApplyInputs(world, timestep);

		if (m_motor)
		{
			bool startEngine = m_motor->GetStart();
			if (m_startEngine ^ startEngine)
			{
				m_startEngine = startEngine;
				if (startEngine)
				{
					m_startSound->Play();
					m_engineRpmSound->Play();
					m_engineRpmSound->SetPitch(0.5f);
					m_engineRpmSound->SetVolume(0.25f);
				}
				else
				{
					m_startSound->Stop();
					m_engineRpmSound->Stop();
				}
			}

			dFloat32 maxRpm = 9000.0f;
			dFloat32 rpm = m_motor->GetRpm() / maxRpm;
			dFloat32 pitchFactor = 0.5f + 0.7f * rpm;
			m_engineRpmSound->SetPitch(pitchFactor);

			// up to two decibels of volume
			dFloat32 volumeFactor = 0.25f + 0.75f * rpm;
			m_engineRpmSound->SetVolume(volumeFactor);
			//dTrace(("%f\n", volumeFactor));

			// apply positional sound
			const dMatrix& location = m_chassis->GetMatrix();
			m_engineRpmSound->SetPosition(location.m_posit);
			m_engineRpmSound->SetVelocity(m_chassis->GetVelocity());
		}

		// test convex cast for now
		if (1)
		{
			// test convex cast
			ndMultiBodyVehicleTireJoint* const tire = m_tireList.GetCount() ? m_tireList.GetFirst()->GetInfo() : nullptr;
			if (tire)
			{
				const ndWheelDescriptor& info = tire->GetInfo();
				const dMatrix tireUpperBumperMatrix(tire->CalculateUpperBumperMatrix());
				const dVector dest(tireUpperBumperMatrix.m_posit - tireUpperBumperMatrix.m_up.Scale(info.m_lowerStop - info.m_upperStop));
				class ndConvexCastNotifyTest : public ndConvexCastNotify
				{
					public:
					ndConvexCastNotifyTest()
						:ndConvexCastNotify()
					{
					}

					virtual dUnsigned32 OnRayPrecastAction(const ndBody* const body, const ndShapeInstance* const)
					{
						return !((body == m_chassis) || (body == m_tire));
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

	void PostTransformUpdate(ndWorld* const, dFloat32)
	{
		if (m_rearAxlePivot && m_frontAxlePivot)
		{
			// play moving part animations, ex suspension nodes.
			ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)m_chassis->GetNotifyCallback();

			const dMatrix rearPivotMatrix((m_rearAxlePivot->CalculateGlobalMatrix(notify->m_entity) * m_chassis->GetMatrix()).Inverse());
			const dMatrix rearLeftTireMatrix(m_rl_tire->GetBody0()->GetMatrix() * rearPivotMatrix);
			const dMatrix rearRightTireMatrix(m_rr_tire->GetBody0()->GetMatrix() * rearPivotMatrix);
			const dVector rearOrigin(dVector::m_half * (rearRightTireMatrix.m_posit + rearLeftTireMatrix.m_posit));
			const dVector rearStep(rearRightTireMatrix.m_posit - rearLeftTireMatrix.m_posit);

			dFloat32 rearAxleAngle = dAtan2(-rearStep.m_y, rearStep.m_z);
			dQuaternion rearAxelRotation(dVector(dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f)), rearAxleAngle);
			m_rearAxlePivot->GetChild()->SetNextMatrix(rearAxelRotation, rearOrigin);

			const dMatrix frontPivotMatrix((m_frontAxlePivot->CalculateGlobalMatrix(notify->m_entity) * m_chassis->GetMatrix()).Inverse());
			const dMatrix frontLeftTireMatrix(m_fl_tire->GetBody0()->GetMatrix() * frontPivotMatrix);
			const dMatrix frontRightTireMatrix(m_fr_tire->GetBody0()->GetMatrix() * frontPivotMatrix);
			const dVector frontOrigin(dVector::m_half * (frontRightTireMatrix.m_posit + frontLeftTireMatrix.m_posit));
			const dVector frontStep(frontRightTireMatrix.m_posit - frontLeftTireMatrix.m_posit);

			dFloat32 frontAxleAngle = dAtan2(-frontStep.m_y, frontStep.m_z);
			dQuaternion frontAxelRotation(dVector(dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f)), frontAxleAngle);
			m_frontAxlePivot->GetChild()->SetNextMatrix(frontAxelRotation, frontOrigin);
		}
	}

	GLuint m_gears;
	GLuint m_odometer;
	GLuint m_redNeedle;
	GLuint m_tachometer;
	GLuint m_greenNeedle;
	dInt32 m_gearMap[8];

	ndSoundChannel* m_skipMarks;
	ndSoundChannel* m_startSound;
	ndSoundChannel* m_engineRpmSound;
	bool m_startEngine;
	ndVehicleUI* m_vehicleUI;

	ndDemoEntity* m_rearAxlePivot;
	ndDemoEntity* m_frontAxlePivot;
	ndMultiBodyVehicleTireJoint* m_rr_tire;
	ndMultiBodyVehicleTireJoint* m_rl_tire;

	ndMultiBodyVehicleTireJoint* m_fr_tire;
	ndMultiBodyVehicleTireJoint* m_fl_tire;

};

void ndBasicVehicle (ndDemoEntityManager* const scene)
{
	dMatrix sceneLocation(dGetIdentityMatrix());

	//BuildFloorBox(scene, sceneLocation);
	BuildFlatPlane(scene, true);
	//BuildGridPlane(scene, 120, 4.0f, 0.0f);
	//BuildStaticMesh(scene, "track.fbx", true);
	//BuildCompoundScene(scene, sceneLocation);
	//BuildStaticMesh(scene, "playerarena.fbx", true);
	//BuildSplineTrack(scene, "playerarena.fbx", true);
	sceneLocation.m_posit.m_x = -200.0f;
	sceneLocation.m_posit.m_z = -200.0f;
	//BuildHeightFieldTerrain(scene, sceneLocation);

	ndPhysicsWorld* const world = scene->GetWorld();
	dVector location(0.0f, 2.0f, 0.0f, 1.0f);
	
	dMatrix matrix(dGetIdentityMatrix());
	dVector floor(FindFloor(*world, location + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit = floor;
	matrix.m_posit.m_y += 0.5f;

	//dVector location(0.0f, 1.75f, 0.0f, 1.0f);
	//dMatrix matrix(dGetIdentityMatrix());
	//matrix = dYawMatrix(180.0f * dDegreeToRad);
	//matrix = matrix * dRollMatrix(-70.0f * dDegreeToRad);
	//matrix.m_posit = location;

	ndSoundManager* const soundManager = world->GetSoundManager();
	for (int i = 0; i < dInt32 (sizeof(engineSounds) / sizeof(engineSounds[0])); i++)
	{
		soundManager->CreateSoundAsset(engineSounds[i]);
	}

#if 0
	// use a test tone
	dVector testPosit(20.0f, 0.25f, 0.0f, 1.0f);
	soundManager->CreateSoundAsset("ctone.wav");
	ndSoundChannel* xxxx = soundManager->CreateSoundChannel("ctone.wav");
	xxxx->SetPosition(testPosit);
	xxxx->SetVolume(1.0f);
	xxxx->SetLoop(true);
	xxxx->SetAttenuationRefDistance(10.0f, 40.0f, 60.0f);
	xxxx->Play();
	ndBodyKinematic* xxxx1 = AddBox(scene, testPosit, 0.0f, 4.0f, 0.5f, 5.0f);
	//xxxx1->GetCollisionShape().SetCollisionMode(false);
#endif


	// add a model for general controls
	ndVehicleSelector* const controls = new ndVehicleSelector();
	scene->GetWorld()->AddModel(controls);
	
	//ndBasicMultiBodyVehicle* const vehicle = new ndBasicMultiBodyVehicle(scene, viperDesc, matrix);
	//ndBasicMultiBodyVehicle* const vehicle = new ndBasicMultiBodyVehicle(scene, jeepDesc, matrix);
	ndBasicMultiBodyVehicle* const vehicle = new ndBasicMultiBodyVehicle(scene, monterTruckDesc, matrix);
	scene->GetWorld()->AddModel(vehicle);
	vehicle->SetAsPlayer(scene);
	scene->Set2DDisplayRenderFunction(ndBasicMultiBodyVehicle::RenderHelp, ndBasicMultiBodyVehicle::RenderUI, vehicle);

	//matrix.m_posit.m_x += 8.0f;
	//matrix.m_posit.m_z += 6.0f;
	//scene->GetWorld()->AddModel(new ndBasicMultiBodyVehicle(scene, jeepDesc, matrix));
	//
	//matrix.m_posit.m_z -= 14.0f;
	//scene->GetWorld()->AddModel(new ndBasicMultiBodyVehicle(scene, monterTruckDesc, matrix));
	//
	//matrix.m_posit.m_x += 15.0f;
	//AddPlanks(scene, matrix.m_posit, 60.0f);

	dQuaternion rot;
	dVector origin(-10.0f, 2.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
