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
		SetNotifyCallback(new ndVehicleCommonNotify(this));
		ndDemoEntity* const vehicleEntityRoot = LoadMeshModel(scene, desc.m_name);
		ndDemoEntity* const vehicleEntity = vehicleEntityRoot->GetFirstChild();
		
		vehicleEntity->ResetMatrix(vehicleEntity->CalculateGlobalMatrix() * matrix);
		
		// create the vehicle chassis as a normal rigid body
		//ndBodyKinematic* const chassis = CreateChassis(scene, vehicleEntity, m_configuration.m_chassisMass);
		ndSharedPtr<ndBody> chassisBody (CreateChassis(scene, vehicleEntity, m_configuration.m_chassisMass));
		AddChassis(chassisBody);
		m_chassis->SetAngularDamping(ndVector(m_configuration.m_chassisAngularDrag));
		
		// lower vehicle com;
		ndVector com(m_chassis->GetCentreOfMass());
		com += m_localFrame.m_up.Scale(m_configuration.m_comDisplacement.m_y);
		com += m_localFrame.m_front.Scale(m_configuration.m_comDisplacement.m_x);
		com += m_localFrame.m_right.Scale(m_configuration.m_comDisplacement.m_z);
		m_chassis->SetCentreOfMass(com);
	}

	~ndHeavyMultiBodyVehicle()
	{
	}

	ndDemoEntity* LoadMeshModel(ndDemoEntityManager* const scene, const char* const filename)
	{
		ndMeshLoader loader;
		ndDemoEntity* const vehicleEntity = loader.LoadEntity(filename, scene);
		scene->AddEntity(vehicleEntity);
		return vehicleEntity;
	}

	void SetAsPlayer(ndDemoEntityManager* const scene, bool mode = true)
	{
		ndVehicleCommon::SetAsPlayer(scene, mode);

		m_vehicleUI->SetVehicle(this);
		scene->SetSelectedModel(this);
		scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
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

	protected:
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
		ndAssert(0);
		//VehicleAssembly(scene);
	}
#if 0
	void VehicleAssembly(ndDemoEntityManager* const scene)
	{
		// 2- each tire to the model, 
		// this function will create the tire as a normal rigid body
		// and attach them to the chassis with the tire joints
		ndWorld* const world = scene->GetWorld();
		ndBodyKinematic* const chassis = m_chassis;
		
		ndVehicleDectriptor::ndTireDefinition r0_tireConfiguration(m_configuration.m_rearTire);
		ndVehicleDectriptor::ndTireDefinition r1_tireConfiguration(m_configuration.m_rearTire);
		ndVehicleDectriptor::ndTireDefinition r2_tireConfiguration(m_configuration.m_rearTire);
		ndVehicleDectriptor::ndTireDefinition r3_tireConfiguration(m_configuration.m_rearTire);
		ndBodyKinematic* const rr_tire0_body = CreateTireBody(scene, chassis, r0_tireConfiguration, "rtire_3");
		ndBodyKinematic* const rl_tire0_body = CreateTireBody(scene, chassis, r1_tireConfiguration, "ltire_3");
		ndBodyKinematic* const rr_tire1_body = CreateTireBody(scene, chassis, r2_tireConfiguration, "rtire_2");
		ndBodyKinematic* const rl_tire1_body = CreateTireBody(scene, chassis, r3_tireConfiguration, "ltire_2");
		ndMultiBodyVehicleTireJoint* const rr_tire0 = AddTire(r0_tireConfiguration, rr_tire0_body);
		ndMultiBodyVehicleTireJoint* const rl_tire0 = AddTire(r1_tireConfiguration, rl_tire0_body);
		ndMultiBodyVehicleTireJoint* const rr_tire1 = AddTire(r2_tireConfiguration, rr_tire1_body);
		ndMultiBodyVehicleTireJoint* const rl_tire1 = AddTire(r3_tireConfiguration, rl_tire1_body);
		
		ndVehicleDectriptor::ndTireDefinition f0_tireConfiguration(m_configuration.m_frontTire);
		ndVehicleDectriptor::ndTireDefinition f1_tireConfiguration(m_configuration.m_frontTire);
		ndVehicleDectriptor::ndTireDefinition f2_tireConfiguration(m_configuration.m_frontTire);
		ndVehicleDectriptor::ndTireDefinition f3_tireConfiguration(m_configuration.m_frontTire);
		ndBodyKinematic* const fr_tire0_body = CreateTireBody(scene, chassis, f0_tireConfiguration, "rtire_0");
		ndBodyKinematic* const fl_tire0_body = CreateTireBody(scene, chassis, f1_tireConfiguration, "ltire_0");
		ndBodyKinematic* const fr_tire1_body = CreateTireBody(scene, chassis, f2_tireConfiguration, "rtire_1");
		ndBodyKinematic* const fl_tire1_body = CreateTireBody(scene, chassis, f3_tireConfiguration, "ltire_1");
		ndMultiBodyVehicleTireJoint* const fr_tire0 = AddTire(f0_tireConfiguration, fr_tire0_body);
		ndMultiBodyVehicleTireJoint* const fl_tire0 = AddTire(f1_tireConfiguration, fl_tire0_body);
		ndMultiBodyVehicleTireJoint* const fr_tire1 = AddTire(f2_tireConfiguration, fr_tire1_body);
		ndMultiBodyVehicleTireJoint* const fl_tire1 = AddTire(f3_tireConfiguration, fl_tire1_body);

		ndSharedPtr<ndBody> rr_tire0_body_Ptr(rr_tire0_body);
		ndSharedPtr<ndBody> rl_tire0_body_Ptr(rl_tire0_body);
		ndSharedPtr<ndBody> rr_tire1_body_Ptr(rr_tire1_body);
		ndSharedPtr<ndBody> rl_tire1_body_Ptr(rl_tire1_body);
		ndSharedPtr<ndBody> fr_tire0_body_Ptr(fr_tire0_body);
		ndSharedPtr<ndBody> fl_tire0_body_Ptr(fl_tire0_body);
		ndSharedPtr<ndBody> fr_tire1_body_Ptr(fr_tire1_body);
		ndSharedPtr<ndBody> fl_tire1_body_Ptr(fl_tire1_body);

		world->AddBody(rr_tire0_body_Ptr);
		world->AddBody(rl_tire0_body_Ptr);
		world->AddBody(rr_tire1_body_Ptr);
		world->AddBody(rl_tire1_body_Ptr);
		world->AddBody(fr_tire0_body_Ptr);
		world->AddBody(fl_tire0_body_Ptr);
		world->AddBody(fr_tire1_body_Ptr);
		world->AddBody(fl_tire1_body_Ptr);
		
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
		ndMultiBodyVehicleGearBox* const gearBox = AddGearBox(differential);
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
		ndWorld* const world = scene->GetWorld();
		ndSharedPtr<ndBody>turretBody (MakeChildPart(scene, m_chassis, "turret", m_configuration.m_chassisMass * 0.05f));
		const ndMatrix turretMatrix(m_localFrame * turretBody->GetMatrix());
		ndSharedPtr<ndJointBilateralConstraint> turretHinge (new ndJointHinge(turretMatrix, turretBody->GetAsBodyKinematic(), m_chassis));
		world->AddBody(turretBody);
		world->AddJoint(turretHinge);
		
		//cannon body
		ndSharedPtr<ndBody>canonBody (MakeChildPart(scene, turretBody->GetAsBodyKinematic(), "canon", m_configuration.m_chassisMass * 0.025f));
		ndMatrix cannonMatrix(m_localFrame * canonBody->GetMatrix());
		ndSharedPtr<ndJointBilateralConstraint> cannonHinge (new ndJointHinge(cannonMatrix, canonBody->GetAsBodyKinematic(), turretBody->GetAsBodyKinematic()));
		world->AddBody(canonBody);
		world->AddJoint(cannonHinge);
		
		// link the effector for controlling the turret
		ndDemoEntity* const turretEntity = (ndDemoEntity*)turretBody->GetNotifyCallback()->GetUserData();
		ndDemoEntity* const effectorEntity = turretEntity->Find("effector");
		ndMatrix effectorMatrix(m_localFrame * effectorEntity->CalculateGlobalMatrix(nullptr));
		effectorMatrix.m_posit = turretBody->GetMatrix().m_posit;
		
		m_effector = new ndIk6DofEffector(effectorMatrix, effectorMatrix, canonBody->GetAsBodyKinematic(), m_chassis);
		m_effector->EnableAxisX(true);
		m_effector->EnableAxisY(false);
		m_effector->EnableAxisZ(false);
		m_effector->EnableRotationAxis(ndIk6DofEffector::m_fixAxis);
		ndSharedPtr<ndJointBilateralConstraint> effectorPtr(m_effector);
		world->AddJoint(effectorPtr);
	}

	void LinkTires(ndDemoEntityManager* const scene, const ndMultiBodyVehicleTireJoint* const tire0, const ndMultiBodyVehicleTireJoint* const tire1)
	{
		ndBodyKinematic* const body0 = tire0->GetBody0();
		ndBodyKinematic* const body1 = tire1->GetBody0();
		
		ndShapeInfo rearInfo(body0->GetCollisionShape().GetShapeInfo());
		ndShapeInfo frontInfo(body1->GetCollisionShape().GetShapeInfo());
		ndFloat32 tireRatio = rearInfo.m_scale.m_y / frontInfo.m_scale.m_y;
		
		ndMatrix pin0(tire0->GetLocalMatrix0() * body0->GetMatrix());
		ndMatrix pin1(tire1->GetLocalMatrix0() * body1->GetMatrix());
		
		ndWorld* const world = scene->GetWorld();
		ndSharedPtr<ndJointBilateralConstraint> link(new ndJointGear(tireRatio, pin0.m_front.Scale(-1.0f), body0, pin1.m_front, body1));
		world->AddJoint(link);
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
#endif
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
		ndAssert(0);
		//VehicleAssembly(scene);
	}

#if 0
	void VehicleAssembly(ndDemoEntityManager* const scene)
	{
		// 2- each tire to the model, 
		// this function will create the tire as a normal rigid body
		// and attach them to the chassis with the tire joints

		ndWorld* const world = scene->GetWorld();
		ndBodyKinematic* const chassis = m_chassis;
		
		ndVehicleDectriptor::ndTireDefinition r0_tireConfiguration(m_configuration.m_rearTire);
		ndVehicleDectriptor::ndTireDefinition r1_tireConfiguration(m_configuration.m_rearTire);
		ndBodyKinematic* const rr_tire0_body = CreateTireBody(scene, chassis, r0_tireConfiguration, "rr_tire");
		ndBodyKinematic* const rl_tire0_body = CreateTireBody(scene, chassis, r1_tireConfiguration, "rl_tire");
		ndMultiBodyVehicleTireJoint* const rr_tire0 = AddTire(r0_tireConfiguration, rr_tire0_body);
		ndMultiBodyVehicleTireJoint* const rl_tire0 = AddTire(r1_tireConfiguration, rl_tire0_body);
		
		ndVehicleDectriptor::ndTireDefinition f0_tireConfiguration(m_configuration.m_frontTire);
		ndVehicleDectriptor::ndTireDefinition f1_tireConfiguration(m_configuration.m_frontTire);
		ndBodyKinematic* const frontAxel_body = MakeFronAxel(scene, chassis);
		ndBodyKinematic* const fr_tire0_body = CreateTireBody(scene, frontAxel_body, f0_tireConfiguration, "fr_tire");
		ndBodyKinematic* const fl_tire0_body = CreateTireBody(scene, frontAxel_body, f1_tireConfiguration, "fl_tire");
		ndMultiBodyVehicleTireJoint* const fr_tire0 = AddAxleTire(f0_tireConfiguration, fr_tire0_body, frontAxel_body);
		ndMultiBodyVehicleTireJoint* const fl_tire0 = AddAxleTire(f1_tireConfiguration, fl_tire0_body, frontAxel_body);

		ndSharedPtr<ndBody> rr_tire0_body_Ptr(rr_tire0_body);
		ndSharedPtr<ndBody> rl_tire0_body_Ptr(rl_tire0_body);
		ndSharedPtr<ndBody> fr_tire0_body_Ptr(fr_tire0_body);
		ndSharedPtr<ndBody> fl_tire0_body_Ptr(fl_tire0_body);
		
		world->AddBody(rr_tire0_body_Ptr);
		world->AddBody(rl_tire0_body_Ptr);
		world->AddBody(fr_tire0_body_Ptr);
		world->AddBody(fl_tire0_body_Ptr);
		
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
		ndMultiBodyVehicleGearBox* const gearBox = AddGearBox(differential);
		gearBox->SetIdleOmega(m_configuration.m_engine.GetIdleRadPerSec() * dRadPerSecToRpm);
		
		// add the bucket joints
		CreateTractorBucket(scene);
		
		// set a soft of hard mode
		SetVehicleSolverModel(m_configuration.m_useHardSolverMode ? true : false);
	}

	ndBodyKinematic* MakeFronAxel(ndDemoEntityManager* const scene, ndBodyKinematic* const chassis)
	{
		ndSharedPtr<ndBody> axleBody(MakeChildPart(scene, m_chassis, "front_axel", m_configuration.m_chassisMass * 0.2f));
		
		// connect the part to the main body with a hinge
		ndWorld* const world = scene->GetWorld();
		ndMatrix hingeFrame(m_localFrame * axleBody->GetMatrix());
		ndJointHinge* const hinge = new ndJointHinge(hingeFrame, axleBody->GetAsBodyKinematic(), chassis);
		hinge->SetLimitState(true);
		hinge->SetLimits(-15.0f * ndDegreeToRad, 15.0f * ndDegreeToRad);
		ndSharedPtr<ndJointBilateralConstraint> hingePtr(hinge);

		world->AddBody(axleBody);
		world->AddJoint(hingePtr);
		return axleBody->GetAsBodyKinematic();
	}

	void AddHydraulic(ndDemoEntityManager* const scene, ndBodyKinematic* const parentBody, const char* const name0, const char* const name1, ndBodyKinematic* const attachmentBody, const char* const attachement)
	{
		ndWorld* const world = scene->GetWorld();
		ndSharedPtr<ndBody>body0 (MakeChildPart(scene, parentBody, name0, m_configuration.m_chassisMass * 0.01f));
		ndMatrix matrix0(m_localFrame * body0->GetMatrix());
		ndSharedPtr<ndJointBilateralConstraint>joint0 (new ndJointHinge(matrix0, body0->GetAsBodyKinematic(), parentBody));
		world->AddBody(body0);
		world->AddJoint(joint0);
		
		ndSharedPtr<ndBody>body1 (MakeChildPart(scene, body0->GetAsBodyKinematic(), name1, m_configuration.m_chassisMass * 0.01f));
		ndMatrix matrix1(m_localFrame * body1->GetMatrix());
		ndSharedPtr<ndJointBilateralConstraint>joint1 (new ndJointSlider(matrix1, body1->GetAsBodyKinematic(), body0->GetAsBodyKinematic()));
		world->AddBody(body1);
		world->AddJoint(joint1);
		
		ndDemoEntity* const parentEntity = (ndDemoEntity*)m_chassis->GetNotifyCallback()->GetUserData();
		ndDemoEntity* const attachmentNode = parentEntity->Find(attachement);
		matrix0.m_posit = attachmentNode->CalculateGlobalMatrix(nullptr).m_posit;
		
		ndIk6DofEffector* const attachementJoint = new ndIk6DofEffector(matrix0, matrix0, body1->GetAsBodyKinematic(), attachmentBody);
		attachementJoint->EnableAxisX(false);
		attachementJoint->EnableAxisY(true);
		attachementJoint->EnableAxisZ(true);
		attachementJoint->EnableRotationAxis(ndIk6DofEffector::m_disabled);
		
		ndSharedPtr<ndJointBilateralConstraint> attachementJointPtr(attachementJoint);
		world->AddJoint(attachementJointPtr);
	}

	void CreateTractorBucket(ndDemoEntityManager* const scene)
	{
		ndWorld* const world = scene->GetWorld();
		ndSharedPtr<ndBody>armBody (MakeChildPart(scene, m_chassis, "arms", m_configuration.m_chassisMass * 0.05f));
		ndMatrix armMatrix(m_localFrame * armBody->GetMatrix());
		m_armHinge = new ndJointHinge(armMatrix, armBody->GetAsBodyKinematic(), m_chassis);
		m_armHinge->SetAsSpringDamper(0.01f, 1500.0f, 20.0f);
		ndSharedPtr<ndJointBilateralConstraint> armHingePtr(m_armHinge);
		
		world->AddBody(armBody);
		world->AddJoint(armHingePtr);
		AddHydraulic(scene, m_chassis, "armHydraulicPiston_left", "armHydraulic_left", armBody->GetAsBodyKinematic(), "attach0_left");
		AddHydraulic(scene, m_chassis, "armHydraulicPiston_right", "armHydraulic_right", armBody->GetAsBodyKinematic(), "attach0_right");
				
		//cannon servo controller actuator
		ndSharedPtr<ndBody>frontBucketBody (MakeChildPart(scene, armBody->GetAsBodyKinematic(), "frontBucket", m_configuration.m_chassisMass * 0.025f));
		ndMatrix frontBucketMatrix(m_localFrame * frontBucketBody->GetMatrix());
		m_bucketHinge = new ndJointHinge(frontBucketMatrix, frontBucketBody->GetAsBodyKinematic(), armBody->GetAsBodyKinematic());
		m_bucketHinge->SetAsSpringDamper(0.01f, 1500.0f, 20.0f);
		ndSharedPtr<ndJointBilateralConstraint> bucketHingePtr(m_bucketHinge);
		
		world->AddBody(frontBucketBody);
		world->AddJoint(bucketHingePtr);
		AddHydraulic(scene, armBody->GetAsBodyKinematic(), "frontBucketHydraulic001", "frontBucketHydraulicPiston001", frontBucketBody->GetAsBodyKinematic(), "attachment_frontBucket001");
		AddHydraulic(scene, armBody->GetAsBodyKinematic(), "frontBucketHydraulic002", "frontBucketHydraulicPiston002", frontBucketBody->GetAsBodyKinematic(), "attachment_frontBucket002");
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
				m_armHinge->SetTargetAngle(m_armAngle);
				m_bucketHinge->SetTargetAngle(m_bucketAngle);
			}
		}
	}
#endif

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
	
		//ndWorld* const world = scene->GetWorld();
		ndBodyKinematic* const chassis = m_chassis;
		
		ndVehicleDectriptor::ndTireDefinition f0_tireConfiguration(m_configuration.m_frontTire);
		ndVehicleDectriptor::ndTireDefinition f1_tireConfiguration(m_configuration.m_frontTire);
		ndSharedPtr<ndBody> fl_tire_body (CreateTireBody(scene, chassis, f0_tireConfiguration, "fl_tire"));
		ndSharedPtr<ndBody> fr_tire_body (CreateTireBody(scene, chassis, f1_tireConfiguration, "fr_tire"));
		AddTire(f0_tireConfiguration, fr_tire_body);
		AddTire(f0_tireConfiguration, fl_tire_body);
		
		ndVehicleDectriptor::ndTireDefinition r0_tireConfiguration(m_configuration.m_rearTire);
		ndVehicleDectriptor::ndTireDefinition r1_tireConfiguration(m_configuration.m_rearTire);
		ndVehicleDectriptor::ndTireDefinition r2_tireConfiguration(m_configuration.m_rearTire);
		ndVehicleDectriptor::ndTireDefinition r3_tireConfiguration(m_configuration.m_rearTire);
		ndSharedPtr<ndBody> rl_tire0_body (CreateTireBody(scene, chassis, r0_tireConfiguration, "rl_midle_tire"));
		ndSharedPtr<ndBody> rr_tire0_body (CreateTireBody(scene, chassis, r1_tireConfiguration, "rr_midle_tire"));
		ndSharedPtr<ndBody> rl_tire1_body (CreateTireBody(scene, chassis, r2_tireConfiguration, "rl_tire"));
		ndSharedPtr<ndBody> rr_tire1_body (CreateTireBody(scene, chassis, r3_tireConfiguration, "rr_tire"));
		
		ndMultiBodyVehicleTireJoint* const rr_tire0 = AddTire(r0_tireConfiguration, rr_tire0_body);
		ndMultiBodyVehicleTireJoint* const rl_tire0 = AddTire(r1_tireConfiguration, rl_tire0_body);
		ndMultiBodyVehicleTireJoint* const rr_tire1 = AddTire(r2_tireConfiguration, rr_tire1_body);
		ndMultiBodyVehicleTireJoint* const rl_tire1 = AddTire(r3_tireConfiguration, rl_tire1_body);

		//ndSharedPtr<ndBody> fl_tire_body_Ptr(fl_tire_body);
		//ndSharedPtr<ndBody> fr_tire_body_Ptr(fr_tire_body);
		//ndSharedPtr<ndBody> rl_tire0_body_Ptr(rl_tire0_body);
		//ndSharedPtr<ndBody> rr_tire0_body_Ptr(rr_tire0_body);
		//ndSharedPtr<ndBody> rl_tire1_body_Ptr(rl_tire1_body);
		//ndSharedPtr<ndBody> rr_tire1_body_Ptr(rr_tire1_body);
		//world->AddBody(fl_tire_body_Ptr);
		//world->AddBody(fr_tire_body_Ptr);
		//world->AddBody(rl_tire0_body_Ptr);
		//world->AddBody(rr_tire0_body_Ptr);
		//world->AddBody(rl_tire1_body_Ptr);
		//world->AddBody(rr_tire1_body_Ptr);

		m_currentGear = sizeof(m_configuration.m_transmission.m_forwardRatios) / sizeof(m_configuration.m_transmission.m_forwardRatios[0]) + 1;
		
		// add the slip differential
		ndMultiBodyVehicleDifferential* const rearDifferential0 = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire0, rr_tire0, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		ndMultiBodyVehicleDifferential* const rearDifferential1 = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire1, rr_tire1, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		ndMultiBodyVehicleDifferential* const differential = AddDifferential(m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rearDifferential0, rearDifferential1, m_configuration.m_slipDifferentialRmpLock / dRadPerSecToRpm);
		
		// add a motor
		ndMultiBodyVehicleMotor* const motor = AddMotor(m_configuration.m_motorMass, m_configuration.m_motorRadius);
		motor->SetMaxRpm(m_configuration.m_engine.GetRedLineRadPerSec() * dRadPerSecToRpm);
		
		// add the gear box
		ndMultiBodyVehicleGearBox* const gearBox = AddGearBox(differential);
		gearBox->SetIdleOmega(m_configuration.m_engine.GetIdleRadPerSec() * dRadPerSecToRpm);
		
		// set a soft or hard mode
		SetVehicleSolverModel(m_configuration.m_useHardSolverMode ? true : false);
	}

	void ApplyInputs(ndWorld* const world, ndFloat32 timestep)
	{
		ndVehicleCommon::ApplyInputs(world, timestep);
	}
};

void ndHeavyVehicle (ndDemoEntityManager* const scene)
{
	ndMatrix sceneLocation(ndGetIdentityMatrix());
	//BuildFloorBox(scene, sceneLocation);
	BuildFlatPlane(scene, true);
	//BuildGridPlane(scene, 120, 4.0f, 0.0f);
	//BuildStaticMesh(scene, "track.fbx", true);
	//BuildCompoundScene(scene, sceneLocation);
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

	ndVehicleUI* const vehicleUI = new ndVehicleUI(scene);
	ndSharedPtr<ndUIEntity> vehicleUIPtr(vehicleUI);
	scene->Set2DDisplayRenderFunction(vehicleUIPtr);
	
	ndSharedPtr<ndModel> vehicle0(new ndBigRigVehicle(scene, bigRigDesc, matrix, vehicleUI));
	
	matrix.m_posit.m_x += 6.0f;
	matrix.m_posit.m_z += 6.0f;
	//ndSharedPtr<ndModel> vehicle1(new ndLav25Vehicle(scene, lav25Desc, matrix, vehicleUI));

	matrix.m_posit.m_z -= 12.0f;
	//ndSharedPtr<ndModel> vehicle2(new ndTractorVehicle(scene, tractorDesc, matrix, vehicleUI));

	world->AddModel(vehicle0);
	//world->AddModel(vehicle1);
	//world->AddModel(vehicle2);

	ndHeavyMultiBodyVehicle* const vehicle = (ndHeavyMultiBodyVehicle*)*vehicle0;
	vehicle->SetAsPlayer(scene);
	
	matrix.m_posit.m_x += 25.0f;
	matrix.m_posit.m_z += 6.0f;
	AddPlanks(scene, matrix, 300.0f, 5);
	
	ndQuaternion rot;
	ndVector origin(-10.0f, 2.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}
