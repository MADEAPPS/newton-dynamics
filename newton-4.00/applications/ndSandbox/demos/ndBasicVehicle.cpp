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
#include "ndSoundManager.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndVehicleCommon.h"
#include "ndMakeStaticMap.h"
#include "ndTargaToOpenGl.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"
#include "ndBasicPlayerCapsule.h"

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
		m_comDisplacement = dVector(0.0f, -0.55f, 0.0f, 0.0f);

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
		m_frontTire.m_steeringAngle = 35.0f * dDegreeToRad;
		m_frontTire.m_springK = 500.0f;
		m_frontTire.m_damperC = 50.0f;
		m_frontTire.m_regularizer = 0.3f;
		m_frontTire.m_upperStop = -0.05f;
		m_frontTire.m_lowerStop = 0.4f;
		m_frontTire.m_brakeTorque = 1500.0f;
		m_rearTire.m_handBrakeTorque = 0.0f;
		m_frontTire.m_verticalOffset = -0.15f;
		m_frontTire.m_laterialStiffness  = 1.0f / 1000.0f;
		m_frontTire.m_longitudinalStiffness  = 50.0f / 1000.0f;

		m_rearTire.m_mass = 100.0f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 500.0f;
		m_rearTire.m_damperC = 50.0f;
		m_rearTire.m_regularizer = 0.3f;
		m_rearTire.m_upperStop = -0.05f;
		m_rearTire.m_lowerStop = 0.4f;
		m_rearTire.m_brakeTorque = 3000.0f;
		m_rearTire.m_handBrakeTorque = 4000.0f;
		m_rearTire.m_verticalOffset = -0.15f;
		m_rearTire.m_laterialStiffness  = 0.3f / 1000.0f;
		m_rearTire.m_longitudinalStiffness  = 50.0f / 1000.0f;

#if 0
// Dave check here
m_comDisplacement.m_y = -1.0f;
m_frontTire.m_verticalOffset = -1.0f;
m_rearTire.m_verticalOffset = -1.0f;
#endif
		
		m_frictionCoefficientScale = 1.3f;
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
		m_frontTire.m_steeringAngle = 35.0f * dDegreeToRad;
		m_frontTire.m_springK = 500.0f;
		m_frontTire.m_damperC = 50.0f;
		m_frontTire.m_regularizer = 0.2f;
		m_frontTire.m_upperStop = -0.05f;
		m_frontTire.m_lowerStop = 0.4f;
		m_frontTire.m_verticalOffset = 0.0f;
		m_frontTire.m_brakeTorque = 1000.0f;
		m_frontTire.m_handBrakeTorque = 0.0f;
		m_frontTire.m_laterialStiffness  = 1.0f / 1000.0f;
		m_frontTire.m_longitudinalStiffness  = 50.0f / 1000.0f;

		m_rearTire.m_mass = 100.0f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 500.0f;
		m_rearTire.m_damperC = 50.0f;
		m_rearTire.m_regularizer = 0.2f;
		m_rearTire.m_upperStop = -0.05f;
		m_rearTire.m_lowerStop = 0.4f;
		m_rearTire.m_verticalOffset = 0.0f;
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

static char* engineSounds[] =
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
	{
		ndDemoEntity* const vehicleEntity = LoadMeshModel(scene, desc.m_name);
		vehicleEntity->ResetMatrix(vehicleEntity->CalculateGlobalMatrix() * matrix);

		ndPhysicsWorld* const world = scene->GetWorld();

		// create the vehicle chassis as a normal rigid body
		ndBodyDynamic* const chassis = CreateChassis(scene, vehicleEntity, m_configuration.m_chassisMass);

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
		ndWheelDescriptor tireInfo;
		tireInfo.m_springK = m_configuration.m_rearTire.m_springK;
		tireInfo.m_damperC = m_configuration.m_rearTire.m_damperC;
		tireInfo.m_minLimit = m_configuration.m_rearTire.m_upperStop;
		tireInfo.m_maxLimit = m_configuration.m_rearTire.m_lowerStop;
		tireInfo.m_brakeTorque = m_configuration.m_rearTire.m_brakeTorque;
		tireInfo.m_regularizer = m_configuration.m_rearTire.m_regularizer;
		tireInfo.m_steeringAngle = m_configuration.m_rearTire.m_steeringAngle;
		tireInfo.m_handBrakeTorque = m_configuration.m_rearTire.m_handBrakeTorque;
		tireInfo.m_laterialStiffness  = m_configuration.m_rearTire.m_laterialStiffness;
		tireInfo.m_longitudinalStiffness  = m_configuration.m_rearTire.m_longitudinalStiffness;
		ndBodyDynamic* const rr_tire_body = CreateTireBody(scene, chassis, m_configuration.m_rearTire, "rr_tire");
		ndMultiBodyVehicleTireJoint* const rr_tire = AddTire(world, tireInfo, rr_tire_body);

		ndBodyDynamic* const rl_tire_body = CreateTireBody(scene, chassis, m_configuration.m_rearTire, "rl_tire");
		ndMultiBodyVehicleTireJoint* const rl_tire = AddTire(world, tireInfo, rl_tire_body);

		tireInfo.m_springK = m_configuration.m_frontTire.m_springK;
		tireInfo.m_damperC = m_configuration.m_frontTire.m_damperC;
		tireInfo.m_minLimit = m_configuration.m_frontTire.m_upperStop;
		tireInfo.m_maxLimit = m_configuration.m_frontTire.m_lowerStop;
		tireInfo.m_brakeTorque = m_configuration.m_frontTire.m_brakeTorque;
		tireInfo.m_regularizer = m_configuration.m_frontTire.m_regularizer;
		tireInfo.m_steeringAngle = m_configuration.m_frontTire.m_steeringAngle;
		tireInfo.m_handBrakeTorque = m_configuration.m_frontTire.m_handBrakeTorque;
		tireInfo.m_laterialStiffness  = m_configuration.m_frontTire.m_laterialStiffness;
		tireInfo.m_longitudinalStiffness  = m_configuration.m_frontTire.m_longitudinalStiffness;
		ndBodyDynamic* const fr_tire_body = CreateTireBody(scene, chassis, m_configuration.m_frontTire, "fr_tire");
		ndMultiBodyVehicleTireJoint* const fr_tire = AddTire(world, tireInfo, fr_tire_body);

		ndBodyDynamic* const fl_tire_body = CreateTireBody(scene, chassis, m_configuration.m_frontTire, "fl_tire");
		ndMultiBodyVehicleTireJoint* const fl_tire = AddTire(world, tireInfo, fl_tire_body);

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

		// load all engine sound channels
		ndSoundManager* const soundManager = world->GetSoundManager();

		m_startEngine = false;
		m_startSound = soundManager->CreateSoundChannel(engineSounds[0]);
		m_engineRpmSound = soundManager->CreateSoundChannel(engineSounds[1]);
		m_skipMarks = soundManager->CreateSoundChannel(engineSounds[2]);
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

	ndBodyDynamic* CreateTireBody(ndDemoEntityManager* const scene, ndBodyDynamic* const chassis, const ndVehicleDectriptor::ndTireDefinition& definition, const char* const tireName)
	{
		dFloat32 width;
		dFloat32 radius;
		ndWorld* const world = scene->GetWorld();
		ndDemoEntity* const chassisEntity = (ndDemoEntity*)chassis->GetNotifyCallback()->GetUserData();
		CalculateTireDimensions(tireName, width, radius, chassisEntity);

		ndShapeInstance tireCollision(CreateTireShape(radius, width));

		ndDemoEntity* const tireEntity = chassisEntity->Find(tireName);
		dMatrix matrix(tireEntity->CalculateGlobalMatrix(nullptr));

		const dMatrix chassisMatrix(m_localFrame * m_chassis->GetMatrix());
		matrix.m_posit += chassisMatrix.m_up.Scale (definition.m_verticalOffset);

		ndBodyDynamic* const tireBody = new ndBodyDynamic();
		tireBody->SetNotifyCallback(new ndDemoEntityNotify(scene, tireEntity, chassis));
		tireBody->SetMatrix(matrix);
		tireBody->SetCollisionShape(tireCollision);
		tireBody->SetMassMatrix(definition.m_mass, tireCollision);

		world->AddBody(tireBody);
		return tireBody;
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
			//dFloat32 maxRpm = m_configuration.m_engine.GetRedLineRadPerSec() * dRadPerSecToRpm;
			dFloat32 maxRpm = 9000.0f;
			dFloat32 rpm = motor->GetRpm() / maxRpm;

			DrawGage(m_tachometer, m_redNeedle, rpm, x, y, gageSize, -180.0f, 90.0f);

			// draw the odometer
			x += gageSize;
			//dFloat32 speed = engine ? dAbs(engine->GetSpeed()) / engine->GetTopSpeed() : 0.0f;
			dFloat32 speed = GetSpeed() / 100.0f;
			DrawGage(m_odometer, m_greenNeedle, dAbs(speed), x, y, gageSize, -180.0f, 90.0f);

			// draw the current gear
			DrawGear(m_gearMap[m_currentGear], x, y + 98, gageSize);
		}
	}

	void ApplyInputs(ndWorld* const world, dFloat32 timestep)
	{
		ndBasicVehicle::ApplyInputs(world, timestep);

		//if (m_motor)
		if (0)
		{
			bool startEngine = m_motor->GetStart();
			if (m_startEngine ^ startEngine)
			{
				m_startEngine = startEngine;
				if (startEngine)
				{
					m_startSound->Play();
					m_engineRpmSound->Play();
					m_engineRpmSound->SetVolume(1.0f);
					m_engineRpmSound->SetPitch(0.5f);
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
			dFloat32 volumeFactor = 1.0f + 1.0f * rpm;
			m_engineRpmSound->SetVolume(volumeFactor);
			//dTrace(("%f\n", volumeFactor));

			// apply positional sound
			const dMatrix& location = m_chassis->GetMatrix();
			m_engineRpmSound->SetPosition(location.m_posit);
			m_engineRpmSound->SetVelocity(m_chassis->GetVelocity());
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
};

void ndBasicVehicle (ndDemoEntityManager* const scene)
{
	// build a floor
	//BuildFloorBox(scene);
	BuildFlatPlane(scene, true);
	//BuildGridPlane(scene, 120, 4.0f, 0.0f);
	//BuildStaticMesh(scene, "track.fbx", true);
	//BuildStaticMesh(scene, "playerarena.fbx", true);
	//BuildSplineTrack(scene, "playerarena.fbx", true);

	dVector location(0.0f, 2.0f, 0.0f, 1.0f);
	
	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = location;

	//dVector location(0.0f, 1.75f, 0.0f, 1.0f);
	//dMatrix matrix(dGetIdentityMatrix());
	//matrix = dYawMatrix(180.0f * dDegreeToRad);
	//matrix = matrix * dRollMatrix(-70.0f * dDegreeToRad);
	//matrix.m_posit = location;

	ndPhysicsWorld* const world = scene->GetWorld();
	ndSoundManager* const soundManager = world->GetSoundManager();
	for (int i = 0; i < sizeof(engineSounds) / sizeof(engineSounds[0]); i++)
	{
		soundManager->CreateSoundAsset(engineSounds[i]);
	}


	soundManager->CreateSoundAsset("ctone.wav");
	ndSoundChannel* xxxx = soundManager->CreateSoundChannel("ctone.wav");
	dVector testPosit(20.0f, 0.25f, 0.0f, 1.0f);
	xxxx->SetPosition(testPosit);
	xxxx->SetVolume(1.0f);
	xxxx->SetLoop(true);
	xxxx->Play();
	AddBox(scene, testPosit, 0.0f, 4.0f, 0.5f, 5.0f);


	// add a model for general controls
	ndVehicleSelector* const controls = new ndVehicleSelector();
	scene->GetWorld()->AddModel(controls);
	
	//ndBasicMultiBodyVehicle* const vehicle = new ndBasicMultiBodyVehicle(scene, viperDesc, matrix);
	ndBasicMultiBodyVehicle* const vehicle = new ndBasicMultiBodyVehicle(scene, jeepDesc, matrix);
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
