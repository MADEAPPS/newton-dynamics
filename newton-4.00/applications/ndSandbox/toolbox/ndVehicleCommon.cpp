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
#include "ndDemoMesh.h"
#include "ndVehicleUI.h"
#include "ndDemoCamera.h"
#include "ndPhysicsWorld.h"
#include "ndVehicleCommon.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndGameControlerInputs.h"

ndVehicleEntityNotify::ndVehicleEntityNotify(ndMultiBodyVehicle* const me, ndDemoEntityManager* const manager, ndDemoEntity* const entity, ndBodyKinematic* const parentBody)
	:ndDemoEntityNotify(manager, entity, parentBody)
	,m_vehicle(me)
{
}

void ndVehicleEntityNotify::OnTransform(ndInt32 thread, const ndMatrix& matrix)
{
	bool test = CheckInWorld(matrix);
	if (test)
	{
		ndDemoEntityNotify::OnTransform(thread, matrix);
	}
	else
	{
		ndPhysicsWorld* const world = m_manager->GetWorld();
		world->RemoveModel(m_vehicle);
	}
}

ndVehicleEntityNotify::~ndVehicleEntityNotify()
{
}

//***************************************************************************************************
// 
// 
//***************************************************************************************************
ndVehicleDectriptor::ndEngineTorqueCurve::ndEngineTorqueCurve()
{
	// take from the data sheet of a 2005 dodge viper, 
	// some values are missing so I have to improvise them
	ndFloat32 idleTorquePoundFoot = 100.0f;
	ndFloat32 idleRmp = 800.0f;
	ndFloat32 horsePower = 400.0f;
	ndFloat32 rpm0 = 5000.0f;
	ndFloat32 rpm1 = 6200.0f;
	ndFloat32 horsePowerAtRedLine = 100.0f;
	ndFloat32 redLineRpm = 8000.0f;
	Init(idleTorquePoundFoot, idleRmp,
		horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);
}

void ndVehicleDectriptor::ndEngineTorqueCurve::Init(
	ndFloat32 idleTorquePoundFoot, ndFloat32 idleRmp,
	ndFloat32 horsePower, ndFloat32 rpm0, ndFloat32 rpm1,
	ndFloat32 horsePowerAtRedLine, ndFloat32 redLineRpm)
{
	m_torqueCurve[0] = ndTorqueTap(0.0f, idleTorquePoundFoot);
	m_torqueCurve[1] = ndTorqueTap(idleRmp, idleTorquePoundFoot);
	
	ndFloat32 power = horsePower * 746.0f;
	ndFloat32 omegaInRadPerSec = rpm0 * 0.105f;
	ndFloat32 torqueInPoundFood = (power / omegaInRadPerSec) / 1.36f;
	m_torqueCurve[2] = ndTorqueTap(rpm0, torqueInPoundFood);
	
	power = horsePower * 746.0f;
	omegaInRadPerSec = rpm1 * 0.105f;
	torqueInPoundFood = (power / omegaInRadPerSec) / 1.36f;
	m_torqueCurve[3] = ndTorqueTap(rpm1, torqueInPoundFood);
	
	power = horsePowerAtRedLine * 746.0f;
	omegaInRadPerSec = redLineRpm * 0.105f;
	torqueInPoundFood = (power / omegaInRadPerSec) / 1.36f;
	m_torqueCurve[4] = ndTorqueTap(redLineRpm, torqueInPoundFood);
}

ndFloat32 ndVehicleDectriptor::ndEngineTorqueCurve::GetIdleRadPerSec() const
{
	return m_torqueCurve[1].m_radPerSeconds;
}

ndFloat32 ndVehicleDectriptor::ndEngineTorqueCurve::GetLowGearShiftRadPerSec() const
{
	return m_torqueCurve[2].m_radPerSeconds;
}

ndFloat32 ndVehicleDectriptor::ndEngineTorqueCurve::GetHighGearShiftRadPerSec() const
{
	return m_torqueCurve[3].m_radPerSeconds;
}

ndFloat32 ndVehicleDectriptor::ndEngineTorqueCurve::GetRedLineRadPerSec() const
{
	const int maxIndex = sizeof(m_torqueCurve) / sizeof(m_torqueCurve[0]);
	return m_torqueCurve[maxIndex - 1].m_radPerSeconds;
}

ndFloat32 ndVehicleDectriptor::ndEngineTorqueCurve::GetTorque(ndFloat32 omegaInRadPerSeconds) const
{
	const int maxIndex = sizeof(m_torqueCurve) / sizeof(m_torqueCurve[0]);
	omegaInRadPerSeconds = ndClamp(omegaInRadPerSeconds, ndFloat32(0.0f), m_torqueCurve[maxIndex - 1].m_radPerSeconds);

	for (ndInt32 i = 1; i < maxIndex; ++i)
	{
		if (omegaInRadPerSeconds <= m_torqueCurve[i].m_radPerSeconds)
		{
			ndFloat32 omega0 = m_torqueCurve[i - 0].m_radPerSeconds;
			ndFloat32 omega1 = m_torqueCurve[i - 1].m_radPerSeconds;

			ndFloat32 torque0 = m_torqueCurve[i - 0].m_torqueInNewtonMeters;
			ndFloat32 torque1 = m_torqueCurve[i - 1].m_torqueInNewtonMeters;

			ndFloat32 torque = torque0 + (omegaInRadPerSeconds - omega0) * (torque1 - torque0) / (omega1 - omega0);
			return torque;
		}
	}

	return m_torqueCurve[maxIndex - 1].m_torqueInNewtonMeters;
}

ndVehicleDectriptor::ndVehicleDectriptor(const char* const fileName)
	:m_comDisplacement(ndVector::m_zero)
{
	strncpy(m_name, fileName, sizeof(m_name));

	ndFloat32 idleTorquePoundFoot = 100.0f;
	ndFloat32 idleRmp = 900.0f;
	ndFloat32 horsePower = 400.0f;
	ndFloat32 rpm0 = 5000.0f;
	ndFloat32 rpm1 = 6200.0f;
	ndFloat32 horsePowerAtRedLine = 100.0f;
	ndFloat32 redLineRpm = 8000.0f;
	m_engine.Init(idleTorquePoundFoot, idleRmp, horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);

	m_chassisMass = 1000.0f;
	m_chassisAngularDrag = 0.25f;
	m_transmission.m_gearsCount = 4;
	m_transmission.m_neutral = 0.0f;
	m_transmission.m_reverseRatio = -3.0f;
	m_transmission.m_crownGearRatio = 10.0f;

	m_transmission.m_forwardRatios[0] = 3.0f;
	m_transmission.m_forwardRatios[1] = 1.5f;
	m_transmission.m_forwardRatios[2] = 1.1f;
	m_transmission.m_forwardRatios[3] = 0.8f;

	m_transmission.m_torqueConverter = 2000.0f;
	m_transmission.m_idleClutchTorque = 200.0f;
	m_transmission.m_lockedClutchTorque = 1.0e6f;
	m_transmission.m_gearShiftDelayTicks = 300;
	m_transmission.m_manual = false;

	m_frontTire.m_mass = 20.0f;
	m_frontTire.m_springK = 1000.0f;
	m_frontTire.m_damperC = 20.0f;
	m_frontTire.m_regularizer = 0.1f;
	m_frontTire.m_upperStop = -0.05f;
	m_frontTire.m_lowerStop = 0.2f;
	m_frontTire.m_verticalOffset = 0.0f;
	m_frontTire.m_brakeTorque = 1500.0f;
	m_frontTire.m_handBrakeTorque = 1500.0f;
	m_frontTire.m_steeringAngle = 35.0f * ndDegreeToRad;

	m_rearTire.m_mass = 20.0f;
	m_rearTire.m_springK = 1000.0f;
	m_rearTire.m_damperC = 20.0f;
	m_rearTire.m_regularizer = 0.1f;
	m_rearTire.m_upperStop = -0.05f;
	m_rearTire.m_lowerStop = 0.2f;
	m_rearTire.m_steeringAngle = 0.0f;
	m_rearTire.m_verticalOffset = 0.0f;
	m_rearTire.m_brakeTorque = 1500.0f;
	m_rearTire.m_handBrakeTorque = 1000.0f;
	
	//ndFloat32 longStiffness = 10.0f * DEMO_GRAVITY * m_chassisMass;
	//ndFloat32 lateralStiffness = 2.0f * longStiffness;

	ndFloat32 longStiffness = 10.0f * DEMO_GRAVITY;
	ndFloat32 lateralStiffness = 2.0f * longStiffness;

	m_rearTire.m_longitudinalStiffness = longStiffness;
	m_frontTire.m_longitudinalStiffness = longStiffness;
	
	m_rearTire.m_laterialStiffness = lateralStiffness;
	m_frontTire.m_laterialStiffness = lateralStiffness;

	m_motorMass = 20.0f;
	m_motorRadius = 0.25f;

	m_differentialMass = 20.0f;
	m_differentialRadius = 0.25f;
	m_slipDifferentialRmpLock = 30.0f;

	m_torsionBarSpringK = 100.0f;
	m_torsionBarDamperC = 10.0f;
	m_torsionBarRegularizer = 0.15f;
	m_torsionBarType = m_noWheelAxle;

	m_differentialType = m_rearWheelDrive;

	m_useHardSolverMode = true;
}

//***************************************************************************************************
// 
// 
//***************************************************************************************************
ndVehicleSelector::ndVehicleSelector()
	:ndModel()
{
	class ndVehicleSelectorNotify : public ndModelNotify
	{
		public:
		ndVehicleSelectorNotify(ndVehicleSelector* const model)
			:ndModelNotify()
			,m_changeVehicle()
		{
			SetModel(model);
		}

		void SelectNext(ndWorld* const world)
		{
			ndDemoEntityManager* const scene = ((ndPhysicsWorld*)world)->GetManager();
			const ndModelList& modelList = world->GetModelList();

			ndFixSizeArray<ndMultiBodyVehicle*, 1024> vehicleArray;
			for (ndModelList::ndNode* node = modelList.GetFirst(); node; node = node->GetNext())
			{
				ndModel* const model = *node->GetInfo();
				if (model->GetAsMultiBodyVehicle())
				{
					vehicleArray.PushBack((ndMultiBodyVehicle*)model->GetAsMultiBodyVehicle());
				}
			}

			for (ndInt32 i = 0; i < vehicleArray.GetCount(); ++i)
			{
				ndVehicleCommonNotify* const notify = (ndVehicleCommonNotify*)*vehicleArray[i]->GetNotifyCallback();
				if (notify->m_isPlayer)
				{
					ndVehicleCommonNotify* const nextNotify = (ndVehicleCommonNotify*)*vehicleArray[(i + 1) % vehicleArray.GetCount()]->GetNotifyCallback();
					notify->SetAsPlayer(scene, false);
					nextNotify->SetAsPlayer(scene, true);
					break;
				}
			}
		}

		void PostUpdate(ndWorld* const world, ndFloat32) override
		{
			ndDemoEntityManager* const scene = ((ndPhysicsWorld*)world)->GetManager();

			ndGameControllerInputs inputs;
			inputs.Update(scene);
			if (m_changeVehicle.Update(inputs.m_buttons[ndVehicleCommonNotify::m_playerButton] ? true : false))
			{
				SelectNext(world);
			}
		}

		ndDemoEntityManager::ndKeyTrigger m_changeVehicle;
	};

	SetNotifyCallback(ndSharedPtr<ndModelNotify>(new ndVehicleSelectorNotify(this)));
}

//***************************************************************************************************
// 
// 
//***************************************************************************************************
ndVehicleMaterial::ndVehicleMaterial()
	:ndApplicationMaterial()
{
}

ndVehicleMaterial::ndVehicleMaterial(const ndVehicleMaterial& src)
	:ndApplicationMaterial(src)
{
}

ndVehicleMaterial* ndVehicleMaterial::Clone() const
{
	return new ndVehicleMaterial(*this);
}

bool ndVehicleMaterial::OnAabbOverlap(const ndContact* const joint, ndFloat32 timestep, const ndShapeInstance& instanceShape0, const ndShapeInstance& instanceShape1) const
{
	// the collision may be a sub part, get the material of the root shape. 
	const ndShapeMaterial& material0 = joint->GetBody0()->GetCollisionShape().GetMaterial();
	const ndShapeMaterial& material1 = joint->GetBody1()->GetCollisionShape().GetMaterial();

	ndUnsigned64 pointer0 = material0.m_userParam[ndDemoContactCallback::m_modelPointer].m_intData;
	ndUnsigned64 pointer1 = material1.m_userParam[ndDemoContactCallback::m_modelPointer].m_intData;
	if (pointer0 == pointer1)
	{
		// vehicle do not self collide
		return false;
	}
	return ndApplicationMaterial::OnAabbOverlap(joint, timestep, instanceShape0, instanceShape1);
}

void ndVehicleMaterial::OnContactCallback(const ndContact* const joint, ndFloat32) const
{
	// here we override contact friction if needed
	const ndMaterial* const matetial = ((ndContact*)joint)->GetMaterial();
	ndContactPointList& contactPoints = ((ndContact*)joint)->GetContactPoints();
	for (ndContactPointList::ndNode* contactNode = contactPoints.GetFirst(); contactNode; contactNode = contactNode->GetNext())
	{
		ndContactMaterial& contactPoint = contactNode->GetInfo();
		ndMaterial& material = contactPoint.m_material;
		material.m_staticFriction0 = matetial->m_staticFriction0;
		material.m_dynamicFriction0 = matetial->m_dynamicFriction0;
		material.m_staticFriction1 = matetial->m_staticFriction1;
		material.m_dynamicFriction1 = matetial->m_dynamicFriction1;
	}
}

//***************************************************************************************************
// 
// 
//***************************************************************************************************
ndVehicleCommonNotify::ndVehicleCommonNotify(const ndVehicleDectriptor& desc, ndMultiBodyVehicle* const vehicle, ndVehicleUI* const ui)
	:ndModelNotify()
	,m_inputs()
	,m_desc(desc)
{
	SetModel(vehicle);
	m_ui = ui;
	m_currentGear = 0;
	m_autoGearShiftTimer = 0;

	m_isParked = true;
	m_isPlayer = false;
	m_startEngine = false;
	m_sleepingState = false;
	m_isManualTransmission = m_desc.m_transmission.m_manual;
}

void ndVehicleCommonNotify::SetAsPlayer(ndDemoEntityManager* const scene, bool mode)
{
	ndMultiBodyVehicle* const vehicle = (ndMultiBodyVehicle*)GetModel();

	m_isPlayer = mode;
	m_ui->SetVehicle(vehicle);
	scene->SetSelectedModel(vehicle);
	scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
}

void ndVehicleCommonNotify::UpdateCameraCallback(ndDemoEntityManager* const manager, void* const context, ndFloat32 timestep)
{
	ndVehicleCommonNotify* const me = (ndVehicleCommonNotify*)context;
	me->SetCamera(manager, timestep);
}

void ndVehicleCommonNotify::CalculateTireDimensions(const char* const tireName, ndFloat32& width, ndFloat32& radius, ndDemoEntity* const vehEntity) const
{
	// find the the tire visual mesh 
	ndDemoEntity* const tirePart = vehEntity->Find(tireName);
	ndAssert(tirePart);

	// make a convex hull collision shape to assist in calculation of the tire shape size
	ndDemoMesh* const tireMesh = (ndDemoMesh*)*tirePart->GetMesh();
	ndAssert(tireMesh);

	const ndMatrix matrix(tirePart->GetMeshMatrix());

	ndArray<ndVector> temp;
	tireMesh->GetVertexArray(temp);

	ndVector minVal(1.0e10f);
	ndVector maxVal(-1.0e10f);
	for (ndInt32 i = 0; i < temp.GetCount(); ++i)
	{
		ndVector p(matrix.TransformVector(temp[i]));
		minVal = minVal.GetMin(p);
		maxVal = maxVal.GetMax(p);
	}

	ndVector size(maxVal - minVal);
	width = size.m_x;
	radius = size.m_y * 0.5f;
}

ndBodyKinematic* ndVehicleCommonNotify::CreateTireBody(ndDemoEntityManager* const scene, ndBodyKinematic* const parentBody, ndVehicleDectriptor::ndTireDefinition& definition, const char* const tireName) const
{
	ndFloat32 width;
	ndFloat32 radius;
	ndDemoEntity* const parentEntity = (ndDemoEntity*)parentBody->GetNotifyCallback()->GetUserData();
	CalculateTireDimensions(tireName, width, radius, parentEntity);
	
	ndMultiBodyVehicle* const vehicle = (ndMultiBodyVehicle*)GetModel();

	definition.m_radios = radius;
	ndShapeInstance tireCollision(vehicle->CreateTireShape(radius, width));
	
	ndDemoEntity* const tireEntity = parentEntity->Find(tireName);
	ndMatrix matrix(tireEntity->CalculateGlobalMatrix(nullptr));
	
	const ndBodyKinematic* const chassis = vehicle->GetChassis();
	ndAssert(chassis);
	
	const ndMatrix chassisMatrix(vehicle->GetLocalFrame() * chassis->GetMatrix());
	matrix.m_posit += chassisMatrix.m_up.Scale(definition.m_verticalOffset);
	
	ndBodyKinematic* const tireBody = new ndBodyDynamic();
	tireBody->SetNotifyCallback(new ndVehicleEntityNotify(vehicle, scene, tireEntity, parentBody));
	tireBody->SetMatrix(matrix);
	tireBody->SetCollisionShape(tireCollision);
	tireBody->SetMassMatrix(definition.m_mass, tireCollision);
	
	ndShapeInstance& instanceShape = tireBody->GetCollisionShape();
	instanceShape.m_shapeMaterial.m_userId = ndDemoContactCallback::m_vehicleTirePart;
	instanceShape.m_shapeMaterial.m_userParam[ndDemoContactCallback::m_modelPointer].m_ptrData = vehicle;
	return tireBody;
}

ndBodyDynamic* ndVehicleCommonNotify::CreateChassis(ndDemoEntityManager* const scene, ndDemoEntity* const chassisEntity, ndFloat32 mass)
{
	ndMatrix matrix(chassisEntity->CalculateGlobalMatrix(nullptr));
	ndSharedPtr<ndShapeInstance> chassisCollision(chassisEntity->CreateCollisionFromChildren());

	ndMultiBodyVehicle* const vehicle = (ndMultiBodyVehicle*)GetModel();
	ndBodyDynamic* const body = new ndBodyDynamic();
	body->SetNotifyCallback(new ndVehicleEntityNotify(vehicle, scene, chassisEntity, nullptr));
	body->SetMatrix(matrix);
	body->SetCollisionShape(**chassisCollision);
	body->SetMassMatrix(mass, **chassisCollision);

	return body;
}

void ndVehicleCommonNotify::ApplyInputs(ndWorld* const world, ndFloat32)
{
	ndMultiBodyVehicle* const vehicle = (ndMultiBodyVehicle*)GetModel();
	ndMultiBodyVehicleMotor* const motor = vehicle->GetMotor();

	if (m_isPlayer && motor)
	{
		ndDemoEntityManager* const scene = ((ndPhysicsWorld*)world)->GetManager();

		m_inputs.Update(scene);
		const ndFixSizeArray<ndFloat32, 8>& axis = m_inputs.m_axis;
		const ndFixSizeArray<char, 32>& buttons = m_inputs.m_buttons;
		
		//ndFloat32 brake = axis[m_brakePedal];
		//for (int i = 0; i < buttons.GetCount(); ++i)
		//{
		//	if (buttons[i])
		//	{
		//		ndTrace(("button %d\n", i));
		//	}
		//}
		
		if (m_parking.Update(buttons[m_parkingButton] ? true : false))
		{
			m_isParked = !m_isParked;
		}
		
		if (m_ignition.Update(buttons[m_ignitionButton] ? true : false))
		{
			m_startEngine = !m_startEngine;
		}
		
		if (m_manualTransmission.Update(buttons[m_automaticGearBoxButton] ? true : false))
		{
			m_isManualTransmission = !m_isManualTransmission;
		}

		ndMultiBodyVehicleGearBox* const gearBox = vehicle->GetGearBox();
		ndAssert(gearBox);
		
		// transmission front gear up
		if (m_forwardGearUp.Update(buttons[m_upGearButton] ? true : false))
		{
			m_isParked = false;
			if (m_currentGear > m_desc.m_transmission.m_gearsCount)
			{
				m_currentGear = 0;
			}
			else
			{
				m_currentGear++;
				if (m_currentGear >= m_desc.m_transmission.m_gearsCount)
				{
					m_currentGear = m_desc.m_transmission.m_gearsCount - 1;
				}
			}
			ndFloat32 gearGain = m_desc.m_transmission.m_crownGearRatio * m_desc.m_transmission.m_forwardRatios[m_currentGear];
			gearBox->SetRatio(gearGain);
			m_autoGearShiftTimer = m_desc.m_transmission.m_gearShiftDelayTicks;
		}
		
		// transmission front gear down
		if (m_forwardGearDown.Update(buttons[m_downGearButton] ? true : false))
		{
			m_isParked = false;
			if (m_currentGear > m_desc.m_transmission.m_gearsCount)
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
			ndFloat32 gearGain = m_desc.m_transmission.m_crownGearRatio * m_desc.m_transmission.m_forwardRatios[m_currentGear];
			gearBox->SetRatio(gearGain);
			m_autoGearShiftTimer = m_desc.m_transmission.m_gearShiftDelayTicks;
		}
		
		const ndFloat32 omega = motor->GetRpm() / dRadPerSecToRpm;
		if (!m_isManualTransmission && (m_autoGearShiftTimer < 0))
		{
			if (m_currentGear < m_desc.m_transmission.m_gearsCount)
			{
				if (omega < m_desc.m_engine.GetLowGearShiftRadPerSec())
				{
					if (m_currentGear > 0)
					{
						m_currentGear--;
						ndFloat32 gearGain = m_desc.m_transmission.m_crownGearRatio * m_desc.m_transmission.m_forwardRatios[m_currentGear];
						gearBox->SetRatio(gearGain);
						m_autoGearShiftTimer = m_desc.m_transmission.m_gearShiftDelayTicks;
					}
				}
				else if (omega > m_desc.m_engine.GetHighGearShiftRadPerSec())
				{
					if (m_currentGear < (m_desc.m_transmission.m_gearsCount - 1))
					{
						m_currentGear++;
						ndFloat32 gearGain = m_desc.m_transmission.m_crownGearRatio * m_desc.m_transmission.m_forwardRatios[m_currentGear];
						gearBox->SetRatio(gearGain);
						m_autoGearShiftTimer = m_desc.m_transmission.m_gearShiftDelayTicks;
					}
				}
			}
		}
		m_autoGearShiftTimer--;

		//ndTrace(("gear:%d gearGain:%f\n", m_currentGear, m_configuration.m_transmission.m_forwardRatios[m_currentGear]));
		
		// neural gear
		if (m_neutralGear.Update(buttons[m_neutralGearButton] ? true : false))
		{
			m_currentGear = sizeof(m_desc.m_transmission.m_forwardRatios) / sizeof(m_desc.m_transmission.m_forwardRatios[0]) + 1;
			gearBox->SetRatio(0.0f);
		}
		
		// reverse gear
		ndFloat32 brake = axis[m_brakePedal];
		if (m_reverseGear.Update(buttons[m_reverseGearButton] ? true : false))
		{
			m_isParked = false;
			m_currentGear = sizeof(m_desc.m_transmission.m_forwardRatios) / sizeof(m_desc.m_transmission.m_forwardRatios[0]);
			ndFloat32 reverseGearRatio = m_desc.m_transmission.m_ratios[m_currentGear];
			ndFloat32 gearGain = m_desc.m_transmission.m_crownGearRatio * reverseGearRatio;
			gearBox->SetRatio(gearGain);
		}
		
		if (m_isParked)
		{
			brake = 1.0f;
		}

		ndFloat32 steerAngle = axis[m_steeringWheel];
		ndFloat32 handBrake = buttons[m_handBreakButton] ? 1.0f : 0.0f;
		//const ndList<ndMultiBodyVehicleTireJoint*>& tireList = vehicle->GetTireList();
		for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = vehicle->GetTireList().GetFirst(); node; node = node->GetNext())
		{
			ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
			tire->SetBrake(brake);
			tire->SetSteering(steerAngle);
			tire->SetHandBrake(handBrake);
		}
		
		// set the transmission Torque converter when the power reverses.
		gearBox->SetInternalTorqueLoss(m_desc.m_transmission.m_torqueConverter);
		if (omega <= (m_desc.m_engine.GetIdleRadPerSec() * 1.01f))
		{
			gearBox->SetClutchTorque(m_desc.m_transmission.m_idleClutchTorque);
		}
		else
		{
			gearBox->SetClutchTorque(m_desc.m_transmission.m_lockedClutchTorque);
		}
		
		if (m_startEngine)
		{
			ndFloat32 throttle = axis[m_gasPedal];
			ndFloat32 currentOmega = motor->GetRpm() / dRadPerSecToRpm;
			ndFloat32 desiredOmega = ndMax(m_desc.m_engine.GetIdleRadPerSec(), throttle * m_desc.m_engine.GetRedLineRadPerSec());
			ndFloat32 torqueFromCurve = m_desc.m_engine.GetTorque(currentOmega);
			motor->SetTorqueAndRpm(torqueFromCurve, desiredOmega * dRadPerSecToRpm);
			vehicle->GetChassis()->SetSleepState(false);
		}
		else
		{
			motor->SetTorqueAndRpm(0.0f, 0.0f);
		}
	}
}

void ndVehicleCommonNotify::Update(ndWorld* const world, ndFloat32 timestep)
{
	ndModelNotify::Update(world, timestep);
	ndMultiBodyVehicle* const vehicle = (ndMultiBodyVehicle*)GetModel();
	m_sleepingState = true;
	if (vehicle && !vehicle->IsSleeping())
	{
		m_sleepingState = false;
		ApplyInputs(world, timestep);
		vehicle->Update(world, timestep);
	}
}

void ndVehicleCommonNotify::PostUpdate(ndWorld* const world, ndFloat32 timestep)
{
	ndModelNotify::PostUpdate(world, timestep);
	ndMultiBodyVehicle* const vehicle = (ndMultiBodyVehicle*)GetModel();
	if (vehicle && !m_sleepingState)
	{
		vehicle->PostUpdate(world, timestep);
	}
}

void ndVehicleCommonNotify::PostTransformUpdate(ndWorld* const, ndFloat32)
{
	// play body part animations here.
}

void ndVehicleCommonNotify::Debug(ndConstraintDebugCallback& context) const
{
	ndModelNotify::Debug(context);
	ndMultiBodyVehicle* const vehicle = (ndMultiBodyVehicle*)GetModel();
	if (vehicle)
	{
		vehicle->Debug(context);
	}
}

void ndVehicleCommonNotify::SetCamera(ndDemoEntityManager* const manager, ndFloat32)
{
	ndMultiBodyVehicle* const vehicle = (ndMultiBodyVehicle*)GetModel();

	if (vehicle->GetRoot())
	{
		ndDemoCamera* const camera = manager->GetCamera();
		ndDemoEntity* const chassisEntity = (ndDemoEntity*)vehicle->GetChassis()->GetNotifyCallback()->GetUserData();
		ndMatrix camMatrix(camera->GetNextMatrix());
		ndMatrix playerMatrix(chassisEntity->GetNextMatrix());

		ndVector frontDir(camMatrix[0]);
		ndVector camOrigin(0.0f);
		camOrigin = playerMatrix.m_posit + ndVector(0.0f, 1.0f, 0.0f, 0.0f);
		camOrigin -= frontDir.Scale(10.0f);

		camera->SetNextMatrix(camMatrix, camOrigin);
	}
}
