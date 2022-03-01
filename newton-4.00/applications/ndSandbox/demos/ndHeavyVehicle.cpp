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
#include "ndVehicleCommon.h"
#include "ndMakeStaticMap.h"
#include "ndTargaToOpenGl.h"
#include "ndCompoundScene.h"
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
		m_frontTire.m_laterialStiffness  = 1.0f / 1000.0f;
		m_frontTire.m_longitudinalStiffness  = 50.0f / 1000.0f;

		m_rearTire.m_mass = 100.0f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 500.0f;
		m_rearTire.m_damperC = 50.0f;
		m_rearTire.m_regularizer = 0.2f;
		m_rearTire.m_upperStop = -0.05f;
		m_rearTire.m_lowerStop = 0.4f;
		m_rearTire.m_verticalOffset = -0.1f;
		m_rearTire.m_brakeTorque = 2500.0f;
		m_rearTire.m_laterialStiffness  = 10.0f / 1000.0f;
		m_rearTire.m_longitudinalStiffness  = 50.0f / 1000.0f;

		m_transmission.m_crownGearRatio = 20.0f;
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
		m_frontTire.m_laterialStiffness  = 500.0f / 1000.0f;
		m_frontTire.m_longitudinalStiffness  = 500.0f / 1000.0f;

		m_rearTire.m_mass = 200.0f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 1000.0f;
		m_rearTire.m_damperC = 25.0f;
		m_rearTire.m_regularizer = 0.01f;
		m_rearTire.m_upperStop = -0.05f;
		m_rearTire.m_lowerStop = 0.4f;
		m_rearTire.m_verticalOffset = -0.1f;
		m_rearTire.m_brakeTorque = 2500.0f;
		m_rearTire.m_laterialStiffness  = 500.0f / 1000.0f;
		m_rearTire.m_longitudinalStiffness  = 500.0f / 1000.0f;

		m_transmission.m_crownGearRatio = 20.0f;

		m_frictionCoefficientScale = 1.3f;

		m_torsionBarType = m_fourWheelAxle;
		m_differentialType = m_fourWheeldrive;
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
		m_frontTire.m_laterialStiffness = 1.0f / 1000.0f;
		m_frontTire.m_longitudinalStiffness = 50.0f / 1000.0f;

		m_rearTire.m_mass = 100.0f;
		m_rearTire.m_verticalOffset = -0.3f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 500.0f;
		m_rearTire.m_damperC = 50.0f;
		m_rearTire.m_regularizer = 0.2f;
		m_rearTire.m_upperStop = -0.05f;
		m_rearTire.m_lowerStop = 0.4f;
		m_rearTire.m_brakeTorque = 2500.0f;
		m_rearTire.m_laterialStiffness = 10.0f / 1000.0f;
		m_rearTire.m_longitudinalStiffness = 50.0f / 1000.0f;

		m_transmission.m_crownGearRatio = 20.0f;
		m_frictionCoefficientScale = 1.3f;

		m_torsionBarType = m_fourWheelAxle;
		m_differentialType = m_eightWheeldrive;
	}
};

static ndVehicleDectriptorLav25 lav25Desc;
static ndVehicleDectriptorBigRig bigRigDesc;
static ndVehicleDectriptorTractor tractorDesc;

class ndHeavyMultiBodyVehicle : public ndBasicVehicle
{
	public:
	ndHeavyMultiBodyVehicle(ndDemoEntityManager* const scene, const ndVehicleDectriptor& desc, const ndMatrix& matrix)
		:ndBasicVehicle(desc)
		,m_vehicleUI(nullptr)
	{
		m_vehicleUI = new ndVehicleUI();
		m_vehicleUI->CreateBufferUI();

		ndDemoEntity* const vehicleEntity = LoadMeshModel(scene, desc.m_name);

		vehicleEntity->ResetMatrix(vehicleEntity->CalculateGlobalMatrix() * matrix);

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
		AddChassis(chassis);
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
		fbxDemoEntity* const vehicleEntity = scene->LoadFbxMesh(filename);
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
		ndHeavyMultiBodyVehicle* const me = (ndHeavyMultiBodyVehicle*)context;
		me->RenderUI(scene);
	}

	static void RenderHelp(ndDemoEntityManager* const scene, void* const context)
	{
		ndHeavyMultiBodyVehicle* const me = (ndHeavyMultiBodyVehicle*)context;
		me->RenderHelp(scene);
	}

	protected:
	ndBodyDynamic* MakeChildPart(ndDemoEntityManager* const scene, ndBodyDynamic* const parentBody, const char* const partName, ndFloat32 mass) const
	{
		ndDemoEntity* const parentEntity = (ndDemoEntity*)parentBody->GetNotifyCallback()->GetUserData();

		ndDemoEntity* const vehPart = parentEntity->Find(partName);
		ndShapeInstance* const vehCollision = vehPart->CreateCollisionFromChildren();

		ndBodyDynamic* const vehBody = new ndBodyDynamic();
		const ndMatrix matrix(vehPart->CalculateGlobalMatrix(nullptr));
		vehBody->SetNotifyCallback(new ndDemoEntityNotify(scene, vehPart, parentBody));
		vehBody->SetMatrix(matrix);
		vehBody->SetCollisionShape(*vehCollision);
		vehBody->SetMassMatrix(mass, *vehCollision);

		delete vehCollision;
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
		ndMultiBodyVehicleMotor* const motor = m_motor;
		if (motor)
		{
			dAssert(motor);

			glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
			ndFloat32 gageSize = 200.0f;
			ndFloat32 y = scene->GetHeight() - (gageSize / 2.0f + 20.0f);

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
					m_vehicleUI->RenderGageUI(scene, m_greenNeedle, -x, -y, s * 0.5f, dAbs(speed), -0.0f, 90.0f);

					// draw the current gear
					m_vehicleUI->RenderGearUI(scene, m_gearMap[m_currentGear], m_gears, -x, -y, gageSize);

					glUseProgram(0);
				}
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
	ndLav25Vehicle(ndDemoEntityManager* const scene, const ndVehicleDectriptor& desc, const ndMatrix& matrix)
		:ndHeavyMultiBodyVehicle(scene, desc, matrix)
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

		ndWorld* const world = scene->GetWorld();
		ndBodyDynamic* const chassis = m_chassis;

		ndBodyDynamic* const rr_tire0_body = CreateTireBody(scene, chassis, m_configuration.m_rearTire, "rtire_3");
		ndBodyDynamic* const rl_tire0_body = CreateTireBody(scene, chassis, m_configuration.m_rearTire, "ltire_3");
		ndBodyDynamic* const rr_tire1_body = CreateTireBody(scene, chassis, m_configuration.m_rearTire, "rtire_2");
		ndBodyDynamic* const rl_tire1_body = CreateTireBody(scene, chassis, m_configuration.m_rearTire, "ltire_2");
		ndMultiBodyVehicleTireJoint* const rr_tire0 = AddTire(m_configuration.m_rearTire, rr_tire0_body);
		ndMultiBodyVehicleTireJoint* const rl_tire0 = AddTire(m_configuration.m_rearTire, rl_tire0_body);
		ndMultiBodyVehicleTireJoint* const rr_tire1 = AddTire(m_configuration.m_rearTire, rr_tire1_body);
		ndMultiBodyVehicleTireJoint* const rl_tire1 = AddTire(m_configuration.m_rearTire, rl_tire1_body);

		ndBodyDynamic* const fr_tire0_body = CreateTireBody(scene, chassis, m_configuration.m_frontTire, "rtire_0");
		ndBodyDynamic* const fl_tire0_body = CreateTireBody(scene, chassis, m_configuration.m_frontTire, "ltire_0");
		ndBodyDynamic* const fr_tire1_body = CreateTireBody(scene, chassis, m_configuration.m_frontTire, "rtire_1");
		ndBodyDynamic* const fl_tire1_body = CreateTireBody(scene, chassis, m_configuration.m_frontTire, "ltire_1");
		ndMultiBodyVehicleTireJoint* const fr_tire0 = AddTire(m_configuration.m_frontTire, fr_tire0_body);
		ndMultiBodyVehicleTireJoint* const fl_tire0 = AddTire(m_configuration.m_frontTire, fl_tire0_body);
		ndMultiBodyVehicleTireJoint* const fr_tire1 = AddTire(m_configuration.m_frontTire, fr_tire1_body);
		ndMultiBodyVehicleTireJoint* const fl_tire1 = AddTire(m_configuration.m_frontTire, fl_tire1_body);

		m_gearMap[sizeof(m_configuration.m_transmission.m_forwardRatios) / sizeof(m_configuration.m_transmission.m_forwardRatios[0]) + 0] = 1;
		m_gearMap[sizeof(m_configuration.m_transmission.m_forwardRatios) / sizeof(m_configuration.m_transmission.m_forwardRatios[0]) + 1] = 0;
		for (int i = 0; i < m_configuration.m_transmission.m_gearsCount; i++)
		{
			m_gearMap[i] = i + 2;
		}
		m_currentGear = sizeof(m_configuration.m_transmission.m_forwardRatios) / sizeof(m_configuration.m_transmission.m_forwardRatios[0]) + 1;

		// add the slip differential
		#if 1
		ndMultiBodyVehicleDifferential* const rearDifferential0 = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire0, rr_tire0, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		ndMultiBodyVehicleDifferential* const rearDifferential1 = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire1, rr_tire1, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);

		ndMultiBodyVehicleDifferential* const frontDifferential0 = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, fl_tire0, fr_tire0, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		ndMultiBodyVehicleDifferential* const frontDifferential1 = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, fl_tire1, fr_tire1, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);

		ndMultiBodyVehicleDifferential* const rearDifferential = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rearDifferential0, rearDifferential1, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		ndMultiBodyVehicleDifferential* const frontDifferential = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, frontDifferential0, frontDifferential1, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);

		ndMultiBodyVehicleDifferential* const differential = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rearDifferential, frontDifferential, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);

		#else
		ndMultiBodyVehicleDifferential* const frontDifferential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, fl_tire0, fr_tire0, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		ndMultiBodyVehicleDifferential* const rearDifferential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire0, rr_tire0, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		ndMultiBodyVehicleDifferential* const differential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rearDifferential, frontDifferential, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);

		LinkTires(world, fl_tire0, fl_tire1);
		LinkTires(world, rl_tire0, rl_tire1);

		LinkTires(world, fr_tire0, fr_tire1);
		LinkTires(world, rr_tire0, rr_tire1);
		#endif

		// add a motor
		ndMultiBodyVehicleMotor* const motor = AddMotor(m_configuration.m_motorMass, m_configuration.m_motorRadius);
		motor->SetMaxRpm(m_configuration.m_engine.GetRedLineRadPerSec() * dRadPerSecToRpm);

		// add the gear box
		ndMultiBodyVehicleGearBox* const gearBox = AddGearBox(m_motor, differential);
		gearBox->SetIdleOmega(m_configuration.m_engine.GetIdleRadPerSec() * dRadPerSecToRpm);

		// add torsion bar
		ndMultiBodyVehicleTorsionBar* const torsionBar = AddTorsionBar(world->GetSentinelBody());
		torsionBar->AddAxel(rl_tire0->GetBody0(), rr_tire0->GetBody0());
		torsionBar->AddAxel(fl_tire0->GetBody0(), fr_tire0->GetBody0());
		torsionBar->SetTorsionTorque(m_configuration.m_torsionBarSpringK, m_configuration.m_torsionBarDamperC, m_configuration.m_torsionBarRegularizer);

		// add vehicle turret
		CreateEightWheelTurret(scene);

		// set a soft or hard mode
		SetVehicleSolverModel(m_configuration.m_useHardSolverMode ? true : false);
	}

	void CreateEightWheelTurret(ndDemoEntityManager* const scene)
	{
		//turret body
		ndBodyDynamic* const turretBody = MakeChildPart(scene, m_chassis, "turret", m_configuration.m_chassisMass * 0.05f);
		const ndMatrix turretMatrix(m_localFrame * turretBody->GetMatrix());
		ndJointHinge* const turretHinge = new ndJointHinge(turretMatrix, turretBody, m_chassis);
		AddExtraBody(turretBody);
		AddExtraJoint(turretHinge);
		
		//cannon body
		ndBodyDynamic* const canonBody = MakeChildPart(scene, turretBody, "canon", m_configuration.m_chassisMass * 0.025f);
		ndMatrix cannonMatrix(m_localFrame * canonBody->GetMatrix());
		ndJointHinge* const cannonHinge = new ndJointHinge(cannonMatrix, canonBody, turretBody);
		AddExtraBody(canonBody);
		AddExtraJoint(cannonHinge);

		// link the effector for controlling the turret
		ndDemoEntity* const turretEntity = (ndDemoEntity*)turretBody->GetNotifyCallback()->GetUserData();
		ndDemoEntity* const effectorEntity = turretEntity->Find("effector");
		ndMatrix effectorMatrix(m_localFrame * effectorEntity->CalculateGlobalMatrix(nullptr));
		effectorMatrix.m_posit = turretBody->GetMatrix().m_posit;
		
		m_effector = new ndIk6DofEffector(effectorMatrix, canonBody, m_chassis);
		m_effector->EnableAxisX(true);
		m_effector->EnableAxisY(false);
		m_effector->EnableAxisZ(false);
		m_effector->EnableFixAxisRotation(true);
		m_effector->EnableShortPathRotation(false);
		AddExtraJoint(m_effector);
	}

	void LinkTires(ndWorld* const world, const ndMultiBodyVehicleTireJoint* const tire0, const ndMultiBodyVehicleTireJoint* const tire1) const
	{
		ndBodyKinematic* const body0 = tire0->GetBody0();
		ndBodyKinematic* const body1 = tire1->GetBody0();

		ndShapeInfo rearInfo(body0->GetCollisionShape().GetShapeInfo());
		ndShapeInfo frontInfo(body1->GetCollisionShape().GetShapeInfo());
		ndFloat32 tireRatio = rearInfo.m_scale.m_y / frontInfo.m_scale.m_y;

		ndMatrix pin0(tire0->GetLocalMatrix0() * body0->GetMatrix());
		ndMatrix pin1(tire1->GetLocalMatrix0() * body1->GetMatrix());
		world->AddJoint(new ndJointGear(tireRatio, pin0.m_front.Scale(-1.0f), body0, pin1.m_front, body1));
	}

	void ApplyInputs(ndWorld* const world, ndFloat32 timestep)
	{
		ndBasicVehicle::ApplyInputs(world, timestep);

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
			
			m_cannonHigh = dClamp(m_cannonHigh, -ndFloat32(0.1f), ndFloat32(0.3f));
			m_turretAngle = dClamp(m_turretAngle, -ndFloat32(2.0f) * ndPi, ndFloat32(2.0f) * ndPi);
			
			if (wakeUpVehicle)
			{
				ndMatrix effectorMatrix (dPitchMatrix(m_turretAngle));
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
	ndTractorVehicle(ndDemoEntityManager* const scene, const ndVehicleDectriptor& desc, const ndMatrix& matrix)
		:ndHeavyMultiBodyVehicle(scene, desc, matrix)
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

		ndBodyDynamic* const chassis = m_chassis;

		ndBodyDynamic* const rr_tire0_body = CreateTireBody(scene, chassis, m_configuration.m_rearTire, "rr_tire");
		ndBodyDynamic* const rl_tire0_body = CreateTireBody(scene, chassis, m_configuration.m_rearTire, "rl_tire");
		ndMultiBodyVehicleTireJoint* const rr_tire0 = AddTire(m_configuration.m_rearTire, rr_tire0_body);
		ndMultiBodyVehicleTireJoint* const rl_tire0 = AddTire(m_configuration.m_rearTire, rl_tire0_body);

		ndBodyDynamic* const frontAxel_body = MakeFronAxel(scene, chassis);
		ndBodyDynamic* const fr_tire0_body = CreateTireBody(scene, frontAxel_body, m_configuration.m_frontTire, "fr_tire");
		ndBodyDynamic* const fl_tire0_body = CreateTireBody(scene, frontAxel_body, m_configuration.m_frontTire, "fl_tire");
		ndMultiBodyVehicleTireJoint* const fr_tire0 = AddAxleTire(m_configuration.m_frontTire, fr_tire0_body, frontAxel_body);
		ndMultiBodyVehicleTireJoint* const fl_tire0 = AddAxleTire(m_configuration.m_frontTire, fl_tire0_body, frontAxel_body);

		m_gearMap[sizeof(m_configuration.m_transmission.m_forwardRatios) / sizeof(m_configuration.m_transmission.m_forwardRatios[0]) + 0] = 1;
		m_gearMap[sizeof(m_configuration.m_transmission.m_forwardRatios) / sizeof(m_configuration.m_transmission.m_forwardRatios[0]) + 1] = 0;
		for (int i = 0; i < m_configuration.m_transmission.m_gearsCount; i++)
		{
			m_gearMap[i] = i + 2;
		}
		m_currentGear = sizeof(m_configuration.m_transmission.m_forwardRatios) / sizeof(m_configuration.m_transmission.m_forwardRatios[0]) + 1;

		// add a motor
		ndMultiBodyVehicleMotor* const motor = AddMotor(m_configuration.m_motorMass, m_configuration.m_motorRadius);
		motor->SetMaxRpm(m_configuration.m_engine.GetRedLineRadPerSec() * dRadPerSecToRpm);

		// add 4 x 4 the slip differential
		ndMultiBodyVehicleDifferential* const rearDifferential = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire0, rr_tire0, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		ndMultiBodyVehicleDifferential* const frontDifferential = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, fl_tire0, fr_tire0, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);

		// slip differential with high slip ratio 
		ndMultiBodyVehicleDifferential* const differential = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rearDifferential, frontDifferential, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		differential->SetSlipOmega(2.0f * differential->GetSlipOmega());

		// add the gear box
		ndMultiBodyVehicleGearBox* const gearBox = AddGearBox(m_motor, differential);
		gearBox->SetIdleOmega(m_configuration.m_engine.GetIdleRadPerSec() * dRadPerSecToRpm);

		// add the bucket joints
		CreateTractorBucket(scene);

		// set a soft of hard mode
		SetVehicleSolverModel(m_configuration.m_useHardSolverMode ? true : false);
	}

	ndBodyDynamic* MakeFronAxel(ndDemoEntityManager* const scene, ndBodyDynamic* const chassis)
	{
		ndBodyDynamic* const axleBody = MakeChildPart(scene, m_chassis, "front_axel", m_configuration.m_chassisMass * 0.2f);

		// connect the part to the main body with a hinge
		ndMatrix hingeFrame(m_localFrame * axleBody->GetMatrix());
		ndJointHinge* const hinge = new ndJointHinge(hingeFrame, axleBody, chassis);
		hinge->SetLimits(-15.0f * ndDegreeToRad, 15.0f * ndDegreeToRad);
		AddExtraBody(axleBody);
		AddExtraJoint(hinge);
		return axleBody;
	}

	void AddHydraulic(ndDemoEntityManager* const scene, ndBodyDynamic* const parentBody, const char* const name0, const char* const name1, ndBodyDynamic* const attachmentBody, const char* const attachement)
	{
		ndBodyDynamic* const body0 = MakeChildPart(scene, parentBody, name0, m_configuration.m_chassisMass * 0.01f);
		ndMatrix matrix0(m_localFrame * body0->GetMatrix());
		ndJointBilateralConstraint* const joint0 = new ndJointHinge(matrix0, body0, parentBody);
		AddExtraBody(body0);
		AddExtraJoint(joint0);

		ndBodyDynamic* const body1 = MakeChildPart(scene, body0, name1, m_configuration.m_chassisMass * 0.01f);
		ndMatrix matrix1(m_localFrame * body1->GetMatrix());
		ndJointBilateralConstraint* const joint1 = new ndJointSlider(matrix1, body1, body0);
		AddExtraBody(body1);
		AddExtraJoint(joint1);
		
		ndDemoEntity* const parentEntity = (ndDemoEntity*)m_chassis->GetNotifyCallback()->GetUserData();
		ndDemoEntity* const attachmentNode = parentEntity->Find(attachement);
		matrix0.m_posit = attachmentNode->CalculateGlobalMatrix(nullptr).m_posit;

		ndIk6DofEffector* const attachementJoint = new ndIk6DofEffector(matrix0, body1, attachmentBody);
		attachementJoint->EnableAxisX(false);
		attachementJoint->EnableAxisY(true);
		attachementJoint->EnableAxisZ(true);
		attachementJoint->EnableFixAxisRotation(false);
		attachementJoint->EnableShortPathRotation(false);
		AddExtraJoint(attachementJoint);
	}

	void CreateTractorBucket(ndDemoEntityManager* const scene)
	{
		ndBodyDynamic* const frontBucketArmBody = MakeChildPart(scene, m_chassis, "arms", m_configuration.m_chassisMass * 0.05f);
		ndMatrix turretMatrix(m_localFrame * frontBucketArmBody->GetMatrix());
		m_armHinge = new ndJointHinge(turretMatrix, frontBucketArmBody, m_chassis);
		m_armHinge->SetLimits(-10.0f * ndDegreeToRad, 55.0f * ndDegreeToRad);
		AddExtraBody(frontBucketArmBody);
		AddExtraJoint(m_armHinge);
		AddHydraulic(scene, m_chassis, "armHydraulicPiston_left", "armHydraulic_left", frontBucketArmBody, "attach0_left");
		AddHydraulic(scene, m_chassis, "armHydraulicPiston_right", "armHydraulic_right", frontBucketArmBody, "attach0_right");
		m_armAngle = -ndAtan2(turretMatrix[1][2], turretMatrix[1][0]);
		
		//cannon servo controller actuator
		ndBodyDynamic* const frontBucketBody = MakeChildPart(scene, frontBucketArmBody, "frontBucket", m_configuration.m_chassisMass * 0.025f);
		ndMatrix frontBucketMatrix(m_localFrame * frontBucketBody->GetMatrix());
		m_bucketHinge = new ndJointHinge(frontBucketMatrix, frontBucketBody, frontBucketArmBody);
		m_bucketHinge->SetLimits(-75.0f * ndDegreeToRad, 80.0f * ndDegreeToRad);
		AddExtraBody(frontBucketBody);
		AddExtraJoint(m_bucketHinge);
		AddHydraulic(scene, frontBucketArmBody, "frontBucketHydraulic001", "frontBucketHydraulicPiston001", frontBucketBody, "attachment_frontBucket001");
		AddHydraulic(scene, frontBucketArmBody, "frontBucketHydraulic002", "frontBucketHydraulicPiston002", frontBucketBody, "attachment_frontBucket002");

		ndFloat32 y = frontBucketMatrix[1][1];
		ndFloat32 x = ndSqrt(frontBucketMatrix[1][0] * frontBucketMatrix[1][0] + frontBucketMatrix[1][2] * frontBucketMatrix[1][2] + 1.0e-6f);
		m_bucketAngle = -ndAtan2(y, x);
	}

	void ApplyInputs(ndWorld* const world, ndFloat32 timestep)
	{
		ndBasicVehicle::ApplyInputs(world, timestep);
		if (m_isPlayer)
		{
			ndDemoEntityManager* const scene = ((ndPhysicsWorld*)world)->GetManager();
			ndFixSizeArray<char, 32> buttons;
		
			bool wakeUpVehicle = false;
			scene->GetJoystickButtons(buttons);
			if (buttons[0])
			{
				wakeUpVehicle = true;
				//m_armAngle = dMin(m_armAngle + 5.0e-3f, m_armHinge->GetMaxAngularLimit());
				//m_armHinge->SetTargetAngle(m_armAngle);
			}
			else if (buttons[3])
			{
				wakeUpVehicle = true;
				//m_armAngle = dMax(m_armAngle - 5.0e-3f, m_armHinge->GetMinAngularLimit());
				//m_armHinge->SetTargetAngle(m_armAngle);
			}
		
			if (buttons[1])
			{
				wakeUpVehicle = true;
				//m_bucketAngle = dMin(m_bucketAngle + 5.0e-3f, m_bucketHinge->GetMaxAngularLimit());
				//m_bucketHinge->SetTargetAngle(m_bucketAngle);
			}
			else if (buttons[2])
			{
				wakeUpVehicle = true;
				//m_bucketAngle = dMax(m_bucketAngle - 5.0e-3f, m_bucketHinge->GetMinAngularLimit());
				//m_bucketHinge->SetTargetAngle(m_bucketAngle);
			}
			//const ndMatrix bucketMatrix(m_bucketHinge->GetLocalMatrix0() * m_bucketHinge->GetBody0()->GetMatrix());
			//ndFloat32 y = bucketMatrix[1][1];
			//ndFloat32 x = ndSqrt(bucketMatrix[1][0] * bucketMatrix[1][0] + bucketMatrix[1][2] * bucketMatrix[1][2] + 1.0e-6f);
			//ndFloat32 bucketAngle = -ndAtan2(y, x);
			//ndFloat32 bucketErrorAngle = AnglesAdd(AnglesAdd(m_bucketAngle, m_bucketAngle), -bucketAngle);
			//ndFloat32 bucketTargetAngle = m_bucketHinge->GetAngle();
			//const ndFloat32 error = 0.125f * ndDegreeToRad;
			//if (dAbs(bucketErrorAngle) > error)
			//{
			//	bucketTargetAngle += bucketErrorAngle;
			//}
			//m_bucketHinge->SetTargetAngle(bucketTargetAngle);
		
			if (wakeUpVehicle)
			{
				m_chassis->SetSleepState(false);
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
	ndBigRigVehicle(ndDemoEntityManager* const scene, const ndVehicleDectriptor& desc, const ndMatrix& matrix)
		:ndHeavyMultiBodyVehicle(scene, desc, matrix)
	{
		VehicleAssembly(scene);
	}

	void VehicleAssembly(ndDemoEntityManager* const scene)
	{
		// 2- each tire to the model, 
		// this function will create the tire as a normal rigid body
		// and attach them to the chassis with the tire joints
	
		//ndWorld* const world = scene->GetWorld();
		ndBodyDynamic* const chassis = m_chassis;

		ndVehicleDectriptor::ndTireDefinition frontTireInfo(m_configuration.m_frontTire);
		ndBodyDynamic* const fl_tire_body = CreateTireBody(scene, chassis, frontTireInfo, "fl_tire");
		ndBodyDynamic* const fr_tire_body = CreateTireBody(scene, chassis, frontTireInfo, "fr_tire");
		AddTire(frontTireInfo, fr_tire_body);
		AddTire(frontTireInfo, fl_tire_body);

		ndVehicleDectriptor::ndTireDefinition rearTireInfo(m_configuration.m_rearTire);
		ndBodyDynamic* const rl_tire0_body = CreateTireBody(scene, chassis, rearTireInfo, "rl_midle_tire");
		ndBodyDynamic* const rr_tire0_body = CreateTireBody(scene, chassis, rearTireInfo, "rr_midle_tire");
		ndBodyDynamic* const rl_tire1_body = CreateTireBody(scene, chassis, rearTireInfo, "rl_tire");
		ndBodyDynamic* const rr_tire1_body = CreateTireBody(scene, chassis, rearTireInfo, "rr_tire");
		
		ndMultiBodyVehicleTireJoint* const rr_tire0 = AddTire(rearTireInfo, rr_tire0_body);
		ndMultiBodyVehicleTireJoint* const rl_tire0 = AddTire(rearTireInfo, rl_tire0_body);
		ndMultiBodyVehicleTireJoint* const rr_tire1 = AddTire(rearTireInfo, rr_tire1_body);
		ndMultiBodyVehicleTireJoint* const rl_tire1 = AddTire(rearTireInfo, rl_tire1_body);

		m_gearMap[sizeof(m_configuration.m_transmission.m_forwardRatios) / sizeof(m_configuration.m_transmission.m_forwardRatios[0]) + 0] = 1;
		m_gearMap[sizeof(m_configuration.m_transmission.m_forwardRatios) / sizeof(m_configuration.m_transmission.m_forwardRatios[0]) + 1] = 0;
		for (int i = 0; i < m_configuration.m_transmission.m_gearsCount; i++)
		{
			m_gearMap[i] = i + 2;
		}
		m_currentGear = sizeof(m_configuration.m_transmission.m_forwardRatios) / sizeof(m_configuration.m_transmission.m_forwardRatios[0]) + 1;

		// add the slip differential
		ndMultiBodyVehicleDifferential* const rearDifferential0 = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire0, rr_tire0, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		ndMultiBodyVehicleDifferential* const rearDifferential1 = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire1, rr_tire1, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		ndMultiBodyVehicleDifferential* const differential = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rearDifferential0, rearDifferential1, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);

		// add a motor
		ndMultiBodyVehicleMotor* const motor = AddMotor(m_configuration.m_motorMass, m_configuration.m_motorRadius);
		motor->SetMaxRpm(m_configuration.m_engine.GetRedLineRadPerSec() * dRadPerSecToRpm);

		// add the gear box
		ndMultiBodyVehicleGearBox* const gearBox = AddGearBox(m_motor, differential);
		gearBox->SetIdleOmega(m_configuration.m_engine.GetIdleRadPerSec() * dRadPerSecToRpm);

		// set a soft or hard mode
		SetVehicleSolverModel(m_configuration.m_useHardSolverMode ? true : false);
	}

	void ApplyInputs(ndWorld* const world, ndFloat32 timestep)
	{
		ndBasicVehicle::ApplyInputs(world, timestep);
	}
};

void ndHeavyVehicle (ndDemoEntityManager* const scene)
{
	ndMatrix sceneLocation(dGetIdentityMatrix());
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

	ndMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = location;

	// add a model for general controls
	ndVehicleSelector* const controls = new ndVehicleSelector();
	scene->GetWorld()->AddModel(controls);

	//ndHeavyMultiBodyVehicle* const vehicle0 = new ndBigRigVehicle(scene, bigRigDesc, matrix);
	//scene->GetWorld()->AddModel(vehicle0);
	
	matrix.m_posit.m_x += 6.0f;
	matrix.m_posit.m_z += 6.0f;
	//ndHeavyMultiBodyVehicle* const vehicle1 = new ndLav25Vehicle(scene, lav25Desc, matrix);
	//scene->GetWorld()->AddModel(vehicle1);
	
	matrix.m_posit.m_z -= 12.0f;
	ndHeavyMultiBodyVehicle* const vehicle2 = new ndTractorVehicle(scene, tractorDesc, matrix);
	scene->GetWorld()->AddModel(vehicle2);

	vehicle2->SetAsPlayer(scene);
	scene->Set2DDisplayRenderFunction(ndHeavyMultiBodyVehicle::RenderHelp, ndHeavyMultiBodyVehicle::RenderUI, vehicle2);
	
	matrix.m_posit.m_x += 25.0f;
	matrix.m_posit.m_z += 6.0f;
	AddPlanks(scene, matrix.m_posit, 300.0f, 5);

	ndQuaternion rot;
	ndVector origin(-10.0f, 2.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);

	//ndLoadSave loadScene;
	//loadScene.SaveModel("xxxxxx", vehicle);
}
