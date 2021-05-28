/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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
#include "ndMakeStaticMap.h"
#include "ndTargaToOpenGl.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"
#include "ndBasicPlayerCapsule.h"

class nvVehicleDectriptor
{
	public:
	class ndGearBox
	{
		public:
		dInt32 m_gearsCount;
		union
		{
			struct
			{
				dFloat32 m_fowardRatios[5];
				dFloat32 m_reverseRatio;
				dFloat32 m_neutral;
			};
			dFloat32 m_ratios[8];
		};
		dFloat32 m_crownGearRatio;
		dFloat32 m_torqueConverter;
		
	};

	enum ndDifferentialType
	{
		m_rearWheelDrive,
		m_frontWheelDrive,
		m_fourWheeldrive,
	};

	enum ndTorsionBarType
	{
		m_noWheelAxle,
		m_rearWheelAxle,
		m_frontWheelAxle,
		m_fourWheelAxle,
	};

	nvVehicleDectriptor()
	{
		m_name[0] = 0;
		SetDefualt();
	}

	nvVehicleDectriptor(const char* const name)
	{
		strcpy(m_name, name);
		SetDefualt();
	}

	void SetDefualt()
	{
		m_chassisMass = 1000.0f;
		m_rearTireMass = 20.0f;
		m_frontTireMass = 20.0f;
		m_suspensionRegularizer = 0.1f;
		m_comDisplacement = dVector::m_zero;

		m_transmission.m_gearsCount = 4;
		m_transmission.m_neutral = 0.0f;
		m_transmission.m_crownGearRatio = 10.0f;
		m_transmission.m_reverseRatio = -3.0f;
		m_transmission.m_fowardRatios[0] = 2.25f;
		m_transmission.m_fowardRatios[1] = 1.60f;
		m_transmission.m_fowardRatios[2] = 1.10f;
		m_transmission.m_fowardRatios[3] = 0.80f;
		m_transmission.m_torqueConverter = 1000.0f;

		m_steeringAngle = 35.0f;
		m_brakeTorque = 1500.0f;
		m_handBrakeTorque = 1500.0f;

		m_motorMass = 20.0f;
		m_motorRadius = 0.25f;

		m_differentialMass = 20.0f;
		m_differentialRadius = 0.25f;

		m_springK = 1000.0f;
		m_damperC = 20.0f;
		m_regularizer = 0.1f;
		m_minLimit = -0.05f;
		m_maxLimit = 0.2f;
		m_laterialStiffeness = 0.5f;
		m_longitudinalStiffeness = 0.5f;

		m_frictionCoefficientScale = 1.5f;

		m_torsionBarTorque = 0.0f;

		m_torsionBarType = m_noWheelAxle;
		m_differentialType = m_rearWheelDrive;
	}

	char m_name[32];

	dVector m_comDisplacement;
	dFloat32 m_chassisMass;
	dFloat32 m_rearTireMass;
	dFloat32 m_frontTireMass;
	dFloat32 m_suspensionRegularizer;

	ndGearBox m_transmission;

	dFloat32 m_steeringAngle;
	dFloat32 m_springK;
	dFloat32 m_damperC;
	dFloat32 m_regularizer;
	dFloat32 m_minLimit;
	dFloat32 m_maxLimit;
	dFloat32 m_laterialStiffeness;
	dFloat32 m_longitudinalStiffeness;
	dFloat32 m_frictionCoefficientScale;

	dFloat32 m_brakeTorque;
	dFloat32 m_handBrakeTorque;

	dFloat32 m_motorMass;
	dFloat32 m_motorRadius;

	dFloat32 m_differentialMass;
	dFloat32 m_differentialRadius;

	dFloat32 m_torsionBarTorque;
	
	ndTorsionBarType m_torsionBarType;
	ndDifferentialType m_differentialType;
};

class nvVehicleDectriptorViper : public nvVehicleDectriptor
{
	public:
	nvVehicleDectriptorViper()
		:nvVehicleDectriptor("viper1.fbx")
	{
		//static nvVehicleDectriptor sportViper("viper1.fbx", 20.0f, 20.0f, 0.025f, dVector(0.25f, -0.35f, 0.0f, 0.0f));
		m_comDisplacement = dVector(0.25f, -0.35f, 0.0f, 0.0f);
	}
};

class nvVehicleDectriptorMonsterTruck: public nvVehicleDectriptor
{
	public:
	nvVehicleDectriptorMonsterTruck()
		:nvVehicleDectriptor("monsterTruck.fbx")
	{
		//"monsterTruck.fbx", 100.0f, 100.0f, 0.1f, dVector(0.0f, -0.55f, 0.0f, 0.0f));
		m_comDisplacement = dVector(0.0f, -0.55f, 0.0f, 0.0f);
		m_rearTireMass = 100.0f;
		m_frontTireMass = 100.0f;

		m_brakeTorque = 5000.0f;
		m_handBrakeTorque = 5000.0f;

		m_springK = 500.0f;
		m_damperC = 50.0f;
		m_suspensionRegularizer = 0.2f;
		m_maxLimit = 0.4f;

		m_laterialStiffeness = 1.0f/1000.0f;
		m_longitudinalStiffeness = 50.0f/1000.0f;
		m_frictionCoefficientScale = 1.3f;

		m_differentialType = m_fourWheeldrive;

		m_torsionBarType = m_fourWheelAxle;
		m_torsionBarTorque = 1000.0f;
	}
};

static nvVehicleDectriptorViper viperDesc;
static nvVehicleDectriptorMonsterTruck monterTruckDesc;

static void AddShape(ndDemoEntityManager* const scene,
	ndDemoInstanceEntity* const rootEntity, const ndShapeInstance& sphereShape,
	dFloat32 mass, const dVector& origin, const dFloat32 diameter, dInt32 count)
{
	dMatrix matrix(dRollMatrix(90.0f * dDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();

	dVector floor(FindFloor(*world, matrix.m_posit + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y = floor.m_y + diameter * 0.5f + 7.0f;

	for (dInt32 i = 0; i < count; i++)
	{
		ndBodyDynamic* const body = new ndBodyDynamic();
		ndDemoEntity* const entity = new ndDemoEntity(matrix, rootEntity);

		body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
		body->SetMatrix(matrix);
		body->SetCollisionShape(sphereShape);
		body->SetMassMatrix(mass, sphereShape);
		body->SetAngularDamping(dVector(dFloat32(0.5f)));

		world->AddBody(body);
		matrix.m_posit.m_y += diameter * 2.5f;
	}
}

static void AddSomeObstacles(ndDemoEntityManager* const scene, const dVector& origin)
//static void AddSomeObstacles(ndDemoEntityManager* const, const dVector&)
{
	dFloat32 diameter = 0.5f;
	ndShapeInstance shape(new ndShapeCapsule(diameter * 0.5f, diameter * 0.5f, diameter * 1.0f));
	ndDemoMeshIntance* const instanceMesh = new ndDemoMeshIntance("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga");

	ndDemoInstanceEntity* const rootEntity = new ndDemoInstanceEntity(instanceMesh);
	scene->AddEntity(rootEntity);

	const dInt32 n = 4;
	const dInt32 stackHigh = 2;
	for (dInt32 i = 0; i < n; i++)
	{
		for (dInt32 j = 0; j < n; j++)
		{
			dVector location((j - n / 2) * 4.0f, 0.0f, (i - n / 2) * 4.0f, 0.0f);
			AddShape(scene, rootEntity, shape, 10.0f, location + origin, diameter, stackHigh);
		}
	}

	instanceMesh->Release();
}

static void PlaceRampRamp(ndDemoEntityManager* const scene, 
	const dMatrix& location, ndDemoMesh* const geometry, ndShapeInstance& ramp)
{
	ndDemoEntity* const entity = new ndDemoEntity(location, nullptr);
	scene->AddEntity(entity);
	entity->SetMesh(geometry, dGetIdentityMatrix());

	ndBodyDynamic* const body = new ndBodyDynamic();
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(location);
	body->SetCollisionShape(ramp);
	scene->GetWorld()->AddBody(body);
}

static void AddRamps(ndDemoEntityManager* const scene, const dVector& origin)
{
	ndShapeInstance box(new ndShapeBox(5.0f, 0.125f, 6.f));
	dMatrix uvMatrix(dGetIdentityMatrix());
	uvMatrix[0][0] *= 0.25f;
	uvMatrix[1][1] *= 0.25f;
	uvMatrix[2][2] *= 0.25f;
	uvMatrix.m_posit = dVector(-0.5f, -0.5f, 0.0f, 1.0f);
	const char* const textureName = "wood_3.tga";
	ndDemoMesh* const geometry = new ndDemoMesh("box", scene->GetShaderCache(), &box, textureName, textureName, textureName, 1.0f, uvMatrix);

	dMatrix matrix(dRollMatrix(20.0f * dDegreeToRad));
	matrix.m_posit = origin;
	PlaceRampRamp(scene, matrix, geometry, box);

	matrix = dGetIdentityMatrix();
	matrix.m_posit = origin;
	matrix.m_posit.m_x += 4.8f;
	matrix.m_posit.m_y += 0.85f;
	PlaceRampRamp(scene, matrix, geometry, box);

	matrix.m_posit.m_x += 5.0f;
	PlaceRampRamp(scene, matrix, geometry, box);

	matrix = matrix * dRollMatrix(-20.0f * dDegreeToRad);
	matrix.m_posit.m_x += 5.0f;
	matrix.m_posit.m_y = origin.m_y;
	PlaceRampRamp(scene, matrix, geometry, box);

	geometry->Release();
}

class ndTireNotifyNotify : public ndDemoEntityNotify
{
	public:
	ndTireNotifyNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity, ndBodyDynamic* const chassis)
		:ndDemoEntityNotify(manager, entity)
		,m_chassis(chassis)
	{
	}

	void OnTranform(dInt32, const dMatrix& matrix)
	{
		dMatrix parentMatrix(m_chassis->GetMatrix());
		dMatrix localMatrix(matrix * parentMatrix.Inverse());

		dQuaternion rot(localMatrix);
		m_entity->SetMatrix(rot, localMatrix.m_posit);
	}

	ndBodyDynamic* m_chassis;
};

class ndBasicMultiBodyVehicle : public ndMultiBodyVehicle
{
	public:
	ndBasicMultiBodyVehicle(ndDemoEntityManager* const scene, const nvVehicleDectriptor& desc, const dMatrix& matrix)
		:ndMultiBodyVehicle(dVector(1.0f, 0.0f, 0.0f, 0.0f), dVector(0.0f, 1.0f, 0.0f, 0.0f))
		,m_configuration(desc)
		,m_steerAngle(0.0f)
		,m_ignition()
		,m_neutralGear()
		,m_reverseGear()
		,m_automaticGearUp()
		,m_automaticGearDown()
		,m_currentGear(0)
		,m_isPlayer(false)
	{
		ndDemoEntity* const vehicleEntity = LoadMeshModel(scene, desc.m_name);
		vehicleEntity->ResetMatrix(vehicleEntity->CalculateGlobalMatrix() * matrix);

		ndWorld* const world = scene->GetWorld();

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
#if 1 
		// 2- each tire to the model, 
		// this function will create the tire as a normal rigid body
		// and attach them to the chassis with the tire joints
		ndBodyDynamic* const rr_tire_body = CreateTireBody(scene, chassis, m_configuration.m_rearTireMass, "rr_tire");
		ndBodyDynamic* const rl_tire_body = CreateTireBody(scene, chassis, m_configuration.m_rearTireMass, "rl_tire");
		ndBodyDynamic* const fr_tire_body = CreateTireBody(scene, chassis, m_configuration.m_frontTireMass, "fr_tire");
		ndBodyDynamic* const fl_tire_body = CreateTireBody(scene, chassis, m_configuration.m_frontTireMass, "fl_tire");

		ndWheelDescriptor tireInfo;
		tireInfo.m_springK = m_configuration.m_springK;
		tireInfo.m_damperC = m_configuration.m_damperC;
		tireInfo.m_regularizer = m_configuration.m_suspensionRegularizer;
		tireInfo.m_minLimit = m_configuration.m_minLimit;
		tireInfo.m_maxLimit = m_configuration.m_maxLimit;
		tireInfo.m_laterialStiffeness = m_configuration.m_laterialStiffeness;
		tireInfo.m_longitudinalStiffeness = m_configuration.m_longitudinalStiffeness;

		ndJointWheel* const rr_tire = AddTire(world, tireInfo, rr_tire_body);
		ndJointWheel* const rl_tire = AddTire(world, tireInfo, rl_tire_body);
		ndJointWheel* const fr_tire = AddTire(world, tireInfo, fr_tire_body);
		ndJointWheel* const fl_tire = AddTire(world, tireInfo, fl_tire_body);


		m_gearMap[sizeof(m_configuration.m_transmission.m_fowardRatios) / sizeof(m_configuration.m_transmission.m_fowardRatios[0]) + 0] = 1;
		m_gearMap[sizeof(m_configuration.m_transmission.m_fowardRatios) / sizeof(m_configuration.m_transmission.m_fowardRatios[0]) + 1] = 0;
		for (int i = 0; i < m_configuration.m_transmission.m_gearsCount; i++)
		{
			m_gearMap[i] = i + 2;
		}
		m_currentGear = sizeof(m_configuration.m_transmission.m_fowardRatios) / sizeof(m_configuration.m_transmission.m_fowardRatios[0]) + 1;

		// configure vehicle steering
		SetAsSteering(fr_tire);
		SetAsSteering(fl_tire);

		// configure the tires brake
		SetAsBrake(rr_tire);
		SetAsBrake(rl_tire);
		SetAsBrake(fr_tire);
		SetAsBrake(fl_tire);

		// configure the tires hand brake
		SetAsHandBrake(rr_tire);
		SetAsHandBrake(rl_tire);

		// add the slip differential
		ndMultiBodyVehicleDifferential* differential = nullptr;
		switch (m_configuration.m_differentialType)
		{
			case nvVehicleDectriptor::m_rearWheelDrive:
			{
				differential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire, rr_tire);
				break;
			}

			case nvVehicleDectriptor::m_frontWheelDrive:
			{
				differential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, fl_tire, fr_tire);				break;
			}

			case nvVehicleDectriptor::m_fourWheeldrive:
			{
				ndMultiBodyVehicleDifferential* const rearDifferential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rl_tire, rr_tire);
				ndMultiBodyVehicleDifferential* const frontDifferential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, fl_tire, fr_tire);
				differential = AddDifferential(world, m_configuration.m_differentialMass, m_configuration.m_differentialRadius, rearDifferential, frontDifferential);
				break;
			}
		}

		// add a motor
		AddMotor(world, m_configuration.m_motorMass, m_configuration.m_motorRadius);

		// add the gear box
		AddGearBox(world, m_motor, differential);

		switch (m_configuration.m_torsionBarType)
		{
			case nvVehicleDectriptor::m_noWheelAxle:
			{
				// no torsion bar
				break;
			}

			case nvVehicleDectriptor::m_fourWheelAxle:
			{
				AddTorsionBar(world);
				break;
			}
		}
#endif
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

	void SetAsPlayer(ndDemoEntityManager* const scene)
	{
		m_isPlayer = true;
		scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
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

	ndBodyDynamic* CreateTireBody(ndDemoEntityManager* const scene, ndBodyDynamic* const chassis, dFloat32 mass, const char* const tireName)
	{
		//rr_tire
		dFloat32 width;
		dFloat32 radius;
		ndWorld* const world = scene->GetWorld();
		ndDemoEntity* const chassisEntity = (ndDemoEntity*)chassis->GetNotifyCallback()->GetUserData();
		CalculateTireDimensions(tireName, width, radius, chassisEntity);

		ndShapeInstance tireCollision(CreateTireShape(radius, width));

		ndDemoEntity* const tireEntity = chassisEntity->Find(tireName);
		dMatrix matrix(tireEntity->CalculateGlobalMatrix(nullptr));

		ndBodyDynamic* const tireBody = new ndBodyDynamic();
		tireBody->SetNotifyCallback(new ndTireNotifyNotify(scene, tireEntity, chassis));
		tireBody->SetMatrix(matrix);
		tireBody->SetCollisionShape(tireCollision);
		tireBody->SetMassMatrix(mass, tireCollision);

		world->AddBody(tireBody);
		return tireBody;
	}

	void Update(ndWorld* const world, dFloat32 timestep)
	{
		if (m_isPlayer && m_motor)
		{
			ndDemoEntityManager* const scene = ((ndPhysicsWorld*)world)->GetManager();
			dFloat32 axis[32];
			char buttons[32];
			scene->GetJoystickAxis(axis);
			scene->GetJoystickButtons(buttons);

			dFloat32 brake = m_configuration.m_brakeTorque * dFloat32(scene->GetKeyState('S'));
			if (brake == 0.0f)
			{
				dFloat32 val = (axis[4] + 1.0f) * 0.5f;
				brake = m_configuration.m_brakeTorque * val * val * val;
				//dTrace(("brake %f\n", brake));
			}

			dFloat32 throttle = dFloat32(scene->GetKeyState('W')) ? 1.0f : 0.0f;
			if (throttle == 0.0f)
			{
				throttle = (axis[5] + 1.0f) * 0.5f;
				throttle = throttle * throttle * throttle;
				//dTrace(("throttle %f\n", throttle));
			}

			dFloat32 steerAngle = m_configuration.m_steeringAngle * (dFloat32(scene->GetKeyState('A')) - dFloat32(scene->GetKeyState('D')));
			if (dAbs(steerAngle) == 0.0f)
			{
				steerAngle = -m_configuration.m_steeringAngle * (axis[0] * axis[0] * axis[0]);
			}
			m_steerAngle = m_steerAngle + (steerAngle - m_steerAngle) * 0.15f;


			static dInt32 xxxx;
			xxxx++;
			throttle = dClamp(throttle, 0.0f, 0.7f);

			dFloat32 handBrake = m_configuration.m_handBrakeTorque * dFloat32(scene->GetKeyState(' ') || buttons[4]);
			//dTrace(("handBrake %f\n", handBrake));

			if (m_ignition.Update(scene->GetKeyState('I') || buttons[7]))
			{
				m_motor->SetStart(!m_motor->GetStart());
			}

			// transmission front gear up
			if (m_automaticGearUp.Update(scene->GetKeyState('T') || buttons[11]))
			{
				if (m_currentGear > m_configuration.m_transmission.m_gearsCount)
				{
					m_currentGear = 0;
				}
				else
				{
					m_currentGear++;
					if (m_currentGear >= m_configuration.m_transmission.m_gearsCount)
					{
						m_currentGear = m_configuration.m_transmission.m_gearsCount - 1;
					}
				}
				//m_gearBox->SetRatio(4.0f);
				dFloat32 gearGain = m_configuration.m_transmission.m_crownGearRatio * m_configuration.m_transmission.m_fowardRatios[m_currentGear];
				m_gearBox->SetRatio(gearGain);
			}

			// transmission front gear down
			if (m_automaticGearDown.Update(scene->GetKeyState('T') || buttons[13]))
			{
				//m_currentGear = 2;
				//m_gearBox->SetRatio(4.0f);
				if (m_currentGear > m_configuration.m_transmission.m_gearsCount)
				{
					m_currentGear = 0;
				}
				else
				{
					m_currentGear--;
					if (m_currentGear <= 0)
					{
						m_currentGear = 0;
					}
				}
				//m_gearBox->SetRatio(4.0f);
				dFloat32 gearGain = m_configuration.m_transmission.m_crownGearRatio * m_configuration.m_transmission.m_fowardRatios[m_currentGear];
				m_gearBox->SetRatio(gearGain);
			}

			// neural gear
			if (m_neutralGear.Update(scene->GetKeyState('N') || buttons[10]))
			{
				m_currentGear = sizeof (m_configuration.m_transmission.m_fowardRatios) / sizeof (m_configuration.m_transmission.m_fowardRatios[0]) + 1;
				m_gearBox->SetRatio(0.0f);
			}

			// reverse gear
			if (m_reverseGear.Update(scene->GetKeyState('R') || buttons[12]))
			{
				m_currentGear = sizeof(m_configuration.m_transmission.m_fowardRatios) / sizeof(m_configuration.m_transmission.m_fowardRatios[0]);

				//m_gearBox->SetRatio(-40.0f);
				dFloat32 gearGain = m_configuration.m_transmission.m_crownGearRatio * m_configuration.m_transmission.m_fowardRatios[m_currentGear];
				m_gearBox->SetRatio(gearGain);
			}

			SetBrakeTorque(brake);
			SetHandBrakeTorque(handBrake);
			SetSteeringAngle(m_steerAngle * dDegreeToRad);

			m_motor->SetThrottle(throttle);
		}

		ndMultiBodyVehicle::Update(world, timestep);
	}

	virtual dFloat32 GetFrictionCoeficient(const ndJointWheel* const, const ndContactMaterial&) const
	{
		//return dFloat32(1.5f);
		return m_configuration.m_frictionCoefficientScale;
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
		scene->Print(color, "accelerator        : 'w'");
		scene->Print(color, "brakes             : 's'");
		scene->Print(color, "turn left          : 'a'");
		scene->Print(color, "turn right         : 'd'");
		scene->Print(color, "hand brakes        : 'space'");

		ImGui::Separator();
		scene->Print(color, "gear box");
		scene->Print(color, "ignition           : 'i'");
		scene->Print(color, "neutral gear	   : 'n'");
		scene->Print(color, "forward gear	   : 't'");
		scene->Print(color, "reverse gear	   : 'r'");
		//ImGui::RadioButton("free", &m_differentialMode, 1);
		//ImGui::RadioButton("rwd", &m_differentialMode, 2);
		//ImGui::RadioButton("fwd", &m_differentialMode, 3);
		
		ImGui::Separator();
		scene->Print(color, "hide help          : 'r'");
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
			//dFloat32 rpm = engine ? engine->GetRpm() / engine->GetRedLineRpm() : 0.0f;
			//dFloat32 rpm = motor->GetSpeed() * 9.55f / 6000.0f;
			dFloat32 rpm = motor->GetRpm() / motor->GetMaxRpm();
			//dTrace(("%f %f\n", motor->GetSpeed(), rpm));

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

	nvVehicleDectriptor m_configuration;
	dFloat32 m_steerAngle;

	GLuint m_gears;
	GLuint m_odometer;
	GLuint m_redNeedle;
	GLuint m_tachometer;
	GLuint m_greenNeedle;
	dInt32 m_gearMap[8];
	
	ndDemoEntityManager::ndKeyTrigger m_ignition;
	ndDemoEntityManager::ndKeyTrigger m_neutralGear;
	ndDemoEntityManager::ndKeyTrigger m_reverseGear;
	ndDemoEntityManager::ndKeyTrigger m_automaticGearUp;
	ndDemoEntityManager::ndKeyTrigger m_automaticGearDown;
	dInt32 m_currentGear;
	bool m_isPlayer;
};

void ndBasicVehicle (ndDemoEntityManager* const scene)
{
	// build a floor
	//BuildFloorBox(scene);
	//BuildFlatPlane(scene, true);
	BuildStaticMesh(scene, "track.fbx", true);
	//BuildStaticMesh(scene, "playerarena.fbx", true);

	//dMatrix location0(dGetIdentityMatrix());
	//location0.m_posit.m_y += 2.0f;
	//location0.m_posit.m_z += 2.0f;
	//dMatrix localAxis(dGetIdentityMatrix());
	//localAxis[0] = dVector(0.0f, 1.0f, 0.0f, 0.0f);
	//localAxis[1] = dVector(1.0f, 0.0f, 0.0f, 0.0f);
	//localAxis[2] = localAxis[0].CrossProduct(localAxis[1]);
	//dFloat32 height = 1.9f;
	//dFloat32 radio = 0.5f;
	//dFloat32 mass = 100.0f;
	//new ndBasicPlayerCapsule(scene, localAxis, location0, mass, radio, height, height / 4.0f, true);

	//dVector location(0.0f, 0.5f, 0.0f, 1.0f);
	dVector location(0.0f, 2.0f, 0.0f, 1.0f);

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = location;

	//ndBasicMultiBodyVehicle* const vehicle = new ndBasicMultiBodyVehicle(scene, viperDesc, matrix);
	ndBasicMultiBodyVehicle* const vehicle = new ndBasicMultiBodyVehicle(scene, monterTruckDesc, matrix);
	scene->GetWorld()->AddModel(vehicle);
	vehicle->SetAsPlayer(scene);

	for (int i = 0; i < 0; i++) 
	{
		matrix.m_posit.m_y += 5.0f;
		scene->GetWorld()->AddModel(new ndBasicMultiBodyVehicle(scene, viperDesc, matrix));
	}

	scene->Set2DDisplayRenderFunction(ndBasicMultiBodyVehicle::RenderHelp, ndBasicMultiBodyVehicle::RenderUI, vehicle);

	//location.m_z = 4.0f;
	//location.m_y = 0.5f; 
	//AddRamps(scene, location);
	//AddSomeObstacles(scene, location);

	dQuaternion rot;
	dVector origin(-10.0f, 2.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
