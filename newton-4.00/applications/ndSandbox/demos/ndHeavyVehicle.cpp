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
#include "ndVehicleCommon.h"
#include "ndMakeStaticMap.h"
#include "ndTargaToOpenGl.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"
#include "ndBasicPlayerCapsule.h"

class nvVehicleDectriptorLav25: public nvVehicleDectriptor
{
	public:
	nvVehicleDectriptorLav25()
		:nvVehicleDectriptor("lav_25.fbx")
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
		m_frontTire.m_steeringAngle = 35.0f;
		m_frontTire.m_springK = 500.0f;
		m_frontTire.m_damperC = 50.0f;
		m_frontTire.m_regularizer = 0.2f;
		m_frontTire.m_upperStop = -0.05f;
		m_frontTire.m_lowerStop = 0.4f;
		m_rearTire.m_verticalOffset = 0.0f;
		m_frontTire.m_laterialStiffeness = 1.0f / 1000.0f;
		m_frontTire.m_longitudinalStiffeness = 50.0f / 1000.0f;

		m_rearTire.m_mass = 100.0f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 500.0f;
		m_rearTire.m_damperC = 50.0f;
		m_rearTire.m_regularizer = 0.2f;
		m_rearTire.m_upperStop = -0.05f;
		m_rearTire.m_lowerStop = 0.4f;
		m_rearTire.m_laterialStiffeness = 1.0f / 1000.0f;
		m_rearTire.m_longitudinalStiffeness = 50.0f / 1000.0f;
		
		m_brakeTorque = 5000.0f;
		m_handBrakeTorque = 5000.0f;
		m_frictionCoefficientScale = 1.3f;
		
		m_torsionBarType = m_fourWheelAxle;
		m_differentialType = m_fourWheeldrive;
	}
};

static nvVehicleDectriptorLav25 lav25Desc;



class ndHeavyMultiBodyVehicle : public ndMultiBodyVehicle
{
	public:
	ndHeavyMultiBodyVehicle(ndDemoEntityManager* const scene, const nvVehicleDectriptor& desc, const dMatrix& matrix)
		:ndMultiBodyVehicle(dVector(1.0f, 0.0f, 0.0f, 0.0f), dVector(0.0f, 1.0f, 0.0f, 0.0f))
		,m_configuration(desc)
		,m_steerAngle(0.0f)
		,m_ignition()
		,m_neutralGear()
		,m_reverseGear()
		,m_forwardGearUp()
		,m_forwardGearDown()
		,m_manualTransmission()
		,m_currentGear(0)
		,m_autoGearShiftTimer(0)
		,m_isPlayer(false)
		,m_isManualTransmission(false)
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

		// 2- each tire to the model, 
		// this function will create the tire as a normal rigid body
		// and attach them to the chassis with the tire joints
		ndBodyDynamic* const rr_tire_body = CreateTireBody(scene, chassis, m_configuration.m_rearTire, "rr_tire");
		ndBodyDynamic* const rl_tire_body = CreateTireBody(scene, chassis, m_configuration.m_rearTire, "rl_tire");
		ndBodyDynamic* const fr_tire_body = CreateTireBody(scene, chassis, m_configuration.m_frontTire, "fr_tire");
		ndBodyDynamic* const fl_tire_body = CreateTireBody(scene, chassis, m_configuration.m_frontTire, "fl_tire");

		ndWheelDescriptor tireInfo;
		tireInfo.m_springK = m_configuration.m_rearTire.m_springK;
		tireInfo.m_damperC = m_configuration.m_rearTire.m_damperC;
		tireInfo.m_regularizer = m_configuration.m_rearTire.m_regularizer;
		tireInfo.m_minLimit = m_configuration.m_rearTire.m_upperStop;
		tireInfo.m_maxLimit = m_configuration.m_rearTire.m_lowerStop;
		tireInfo.m_laterialStiffeness = m_configuration.m_rearTire.m_laterialStiffeness;
		tireInfo.m_longitudinalStiffeness = m_configuration.m_rearTire.m_longitudinalStiffeness;
		ndJointWheel* const rr_tire = AddTire(world, tireInfo, rr_tire_body);
		ndJointWheel* const rl_tire = AddTire(world, tireInfo, rl_tire_body);

		tireInfo.m_springK = m_configuration.m_frontTire.m_springK;
		tireInfo.m_damperC = m_configuration.m_frontTire.m_damperC;
		tireInfo.m_regularizer = m_configuration.m_frontTire.m_regularizer;
		tireInfo.m_minLimit = m_configuration.m_frontTire.m_upperStop;
		tireInfo.m_maxLimit = m_configuration.m_frontTire.m_lowerStop;
		tireInfo.m_laterialStiffeness = m_configuration.m_frontTire.m_laterialStiffeness;
		tireInfo.m_longitudinalStiffeness = m_configuration.m_frontTire.m_longitudinalStiffeness;
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
		ndMultiBodyVehicleMotor* const motor = AddMotor(world, m_configuration.m_motorMass, m_configuration.m_motorRadius);
		motor->SetRpmLimits(m_configuration.m_engine.GetIdleRadPerSec() * 9.55f, m_configuration.m_engine.GetRedLineRadPerSec() * 9.55f);

		// add the gear box
		AddGearBox(world, m_motor, differential);

		switch (m_configuration.m_torsionBarType)
		{
			case nvVehicleDectriptor::m_noWheelAxle:
			{
				// no torsion bar
				break;
			}

			case nvVehicleDectriptor::m_rearWheelAxle:
			{
				ndMultiBodyVehicleTorsionBar* const torsionBar = AddTorsionBar(world);
				torsionBar->AddAxel(rl_tire->GetBody0(), rr_tire->GetBody0());
				torsionBar->SetTorsionTorque(m_configuration.m_torsionBarSpringK, m_configuration.m_torsionBarDamperC, m_configuration.m_torsionBarRegularizer);
				break;
			}

			case nvVehicleDectriptor::m_frontWheelAxle:
			{
				ndMultiBodyVehicleTorsionBar* const torsionBar = AddTorsionBar(world);
				torsionBar->AddAxel(fl_tire->GetBody0(), fr_tire->GetBody0());
				torsionBar->SetTorsionTorque(m_configuration.m_torsionBarSpringK, m_configuration.m_torsionBarDamperC, m_configuration.m_torsionBarRegularizer);
				break;
			}

			case nvVehicleDectriptor::m_fourWheelAxle:
			{
				ndMultiBodyVehicleTorsionBar* const torsionBar = AddTorsionBar(world);
				torsionBar->AddAxel(rl_tire->GetBody0(), rr_tire->GetBody0());
				torsionBar->AddAxel(fl_tire->GetBody0(), fr_tire->GetBody0());
				torsionBar->SetTorsionTorque(m_configuration.m_torsionBarSpringK, m_configuration.m_torsionBarDamperC, m_configuration.m_torsionBarRegularizer);
				break;
			}
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

	ND_CLASS_RELECTION(ndHeavyMultiBodyVehicle);

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
		m_isPlayer = mode;
		scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
		scene->Set2DDisplayRenderFunction(RenderHelp, RenderUI, this);
	}

	bool IsPlayer() const
	{
		return m_isPlayer;
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

	ndBodyDynamic* CreateTireBody(ndDemoEntityManager* const scene, ndBodyDynamic* const chassis, const nvVehicleDectriptor::ndTireDefinition& definition, const char* const tireName)
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
		tireBody->SetNotifyCallback(new ndTireNotifyNotify(scene, tireEntity, chassis));
		tireBody->SetMatrix(matrix);
		tireBody->SetCollisionShape(tireCollision);
		tireBody->SetMassMatrix(definition.m_mass, tireCollision);

		world->AddBody(tireBody);
		return tireBody;
	}

	void Update(ndWorld* const world, dFloat32 timestep)
	{
		if (m_isPlayer && m_motor)
		{
			dFloat32 axis[32];
			char buttons[32];
			ndDemoEntityManager* const scene = ((ndPhysicsWorld*)world)->GetManager();

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
			}

			dFloat32 steerAngle = m_configuration.m_frontTire.m_steeringAngle * (dFloat32(scene->GetKeyState('A')) - dFloat32(scene->GetKeyState('D')));
			if (dAbs(steerAngle) == 0.0f)
			{
				steerAngle = -m_configuration.m_frontTire.m_steeringAngle * (axis[0] * axis[0] * axis[0]);
			}
			m_steerAngle = m_steerAngle + (steerAngle - m_steerAngle) * 0.15f;

			dFloat32 handBrake = m_configuration.m_handBrakeTorque * dFloat32(scene->GetKeyState(' ') || buttons[4]);

			if (m_ignition.Update(scene->GetKeyState('I') || buttons[7]))
			{
				m_motor->SetStart(!m_motor->GetStart());
			}

			if (m_manualTransmission.Update(scene->GetKeyState('?') || scene->GetKeyState('/') || buttons[6]))
			{
				m_isManualTransmission = !m_isManualTransmission;
			}

			// transmission front gear up
			if (m_forwardGearUp.Update(scene->GetKeyState('>') || scene->GetKeyState('.') || buttons[11]))
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
				dFloat32 gearGain = m_configuration.m_transmission.m_crownGearRatio * m_configuration.m_transmission.m_fowardRatios[m_currentGear];
				m_gearBox->SetRatio(gearGain);
				m_autoGearShiftTimer = AUTOMATION_TRANSMISSION_FRAME_DELAY;
			}

			// transmission front gear down
			if (m_forwardGearDown.Update(scene->GetKeyState('<') || scene->GetKeyState(',') || buttons[13]))
			{
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
				dFloat32 gearGain = m_configuration.m_transmission.m_crownGearRatio * m_configuration.m_transmission.m_fowardRatios[m_currentGear];
				m_gearBox->SetRatio(gearGain);
				m_autoGearShiftTimer = AUTOMATION_TRANSMISSION_FRAME_DELAY;
			}

			const dFloat32 omega = m_motor->GetRpm() / 9.55f;
			if (!m_isManualTransmission && (m_autoGearShiftTimer < 0))
			{
				if (m_currentGear < m_configuration.m_transmission.m_gearsCount)
				{
					if (omega < m_configuration.m_engine.GetLowGearShiftRadPerSec())
					{
						if (m_currentGear > 0)
						{
							m_currentGear--;
							dFloat32 gearGain = m_configuration.m_transmission.m_crownGearRatio * m_configuration.m_transmission.m_fowardRatios[m_currentGear];
							m_gearBox->SetRatio(gearGain);
							m_autoGearShiftTimer = AUTOMATION_TRANSMISSION_FRAME_DELAY;
						}
					}
					else if (omega > m_configuration.m_engine.GetHighGearShiftRadPerSec())
					{
						if (m_currentGear < (m_configuration.m_transmission.m_gearsCount - 1))
						{
							m_currentGear++;
							dFloat32 gearGain = m_configuration.m_transmission.m_crownGearRatio * m_configuration.m_transmission.m_fowardRatios[m_currentGear];
							m_gearBox->SetRatio(gearGain);
							m_autoGearShiftTimer = AUTOMATION_TRANSMISSION_FRAME_DELAY;
						}
					}
				}
			}
			m_autoGearShiftTimer--;

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

			if (omega <= (m_configuration.m_engine.GetIdleRadPerSec() * 1.01f))
			{
				m_gearBox->SetClutchTorque(m_configuration.m_transmission.m_idleClutchTorque);
			}
			else
			{
				m_gearBox->SetClutchTorque(m_configuration.m_transmission.m_lockedClutchTorque);
			}

			m_motor->SetThrottle(throttle);
			m_motor->SetTorque(m_configuration.m_engine.GetTorque(m_motor->GetRpm() / 9.55f));
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
			//dFloat32 maxRpm = m_configuration.m_engine.GetRedLineRadPerSec() * 9.55f;
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
	ndDemoEntityManager::ndKeyTrigger m_forwardGearUp;
	ndDemoEntityManager::ndKeyTrigger m_forwardGearDown;
	ndDemoEntityManager::ndKeyTrigger m_manualTransmission;
	
	dInt32 m_currentGear;
	dInt32 m_autoGearShiftTimer;
	bool m_isPlayer;
	bool m_isManualTransmission;
};

class ndGlobalControl : public ndModel
{
	public:
	ndGlobalControl()
		:ndModel()
		,m_changeVehicle()
	{
	}

	void Update(ndWorld* const, dFloat32)
	{
	}

	void PostUpdate(ndWorld* const world, dFloat32)
	{
		char buttons[32];
		ndDemoEntityManager* const scene = ((ndPhysicsWorld*)world)->GetManager();
		scene->GetJoystickButtons(buttons);
		if (m_changeVehicle.Update(scene->GetKeyState('C') || buttons[5]))
		{
			const ndModelList& modelList = world->GetModelList();

			dInt32 vehiclesCount = 0;
			ndHeavyMultiBodyVehicle* vehicleArray[1024];
			for (ndModelList::dListNode* node = modelList.GetFirst(); node; node = node->GetNext())
			{
				ndModel* const model = node->GetInfo();
				if (!strcmp(model->GetClassName(), "ndHeavyMultiBodyVehicle"))
				{
					vehicleArray[vehiclesCount] = (ndHeavyMultiBodyVehicle*)model->GetAsMultiBodyVehicle();
					vehiclesCount++;
				}
			}

			if (vehiclesCount > 1)
			{
				for (dInt32 i = 0; i < vehiclesCount; i++)
				{
					if (vehicleArray[i]->IsPlayer())
					{
						ndHeavyMultiBodyVehicle* const nexVehicle = vehicleArray[(i + 1) % vehiclesCount];
						vehicleArray[i]->SetAsPlayer(scene, false);
						nexVehicle->SetAsPlayer(scene, true);
						break;
					}
				}
			}
		}
	}

	ndDemoEntityManager::ndKeyTrigger m_changeVehicle;
};

void ndHeavyVehicle (ndDemoEntityManager* const scene)
{
	// build a floor
	//BuildFloorBox(scene);
	BuildFlatPlane(scene, true);
	//BuildStaticMesh(scene, "track.fbx", true);
	//BuildStaticMesh(scene, "playerarena.fbx", true);

	dVector location(0.0f, 2.0f, 0.0f, 1.0f);

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = location;

	// add a model for general controls
	ndGlobalControl* const controls = new ndGlobalControl();
	scene->GetWorld()->AddModel(controls);

	//ndHeavyMultiBodyVehicle* const vehicle = new ndHeavyMultiBodyVehicle(scene, viperDesc, matrix);
	ndHeavyMultiBodyVehicle* const vehicle = new ndHeavyMultiBodyVehicle(scene, lav25Desc, matrix);
	scene->GetWorld()->AddModel(vehicle);
	vehicle->SetAsPlayer(scene);

	//matrix.m_posit.m_x += 8.0f;
	//matrix.m_posit.m_z += 2.0f;
	//scene->GetWorld()->AddModel(new ndHeavyMultiBodyVehicle(scene, monterTruckDesc, matrix));
	//
	//matrix.m_posit.m_x += 15.0f;
	//AddPlanks(scene, matrix.m_posit);

	scene->Set2DDisplayRenderFunction(ndHeavyMultiBodyVehicle::RenderHelp, ndHeavyMultiBodyVehicle::RenderUI, vehicle);

	dQuaternion rot;
	dVector origin(-10.0f, 2.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
