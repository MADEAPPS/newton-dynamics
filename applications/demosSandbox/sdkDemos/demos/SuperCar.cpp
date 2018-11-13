/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "toolbox_stdafx.h"
#include "SkyBox.h"
#include "PhysicsUtils.h"
#include "TargaToOpenGl.h"
#include "HeightFieldPrimitive.h"
#include "DemoMesh.h"
#include "DemoCamera.h"
#include "DemoEntityManager.h"
#include "DebugDisplay.h"
#include "UserPlaneCollision.h"

// some figures form the 2000 SRT Viper data sheet: http://www.vipercentral.com/specifications/
// the 2000 Vipers’ 8.4-liter engine generates
// max speed: 164 miles per hours					= 73.0f meter per seconds		
// horse power: 450 hp @ 5,200 rpm                    
// peak torque: 490 lb-ft @ 3700 rpm				
// read line:   6000 rpm : 6200 rpm fuel cut-off
// Curb weight: 3460 lb								= 1560 kilogram

// gear Box:
// 1st Gear 2.66:1 
// 2nd Gear 1.78:1 
// 3rd Gear 1.30:1 
// 4th Gear 1.00:1 
// 5th Gear 0.74:1 
// 6th Gear 0.50:1 
// Reverse 2.90:1 

#if 0

#define VEHICLE_THIRD_PERSON_VIEW_HIGHT		2.0f
#define VEHICLE_THIRD_PERSON_VIEW_DIST		7.0f
#define VEHICLE_THIRD_PERSON_VIEW_FILTER	0.125f

struct CarDefinition 
{
	dFloat m_vehicleMass;
	dFloat m_engineMass;
	dFloat m_tireMass;
	dFloat m_engineRotorRadio;
	dFloat m_frontSteeringAngle;
	dFloat m_rearSteeringAngle;
	dFloat m_vehicleWeightDistribution;
	dFloat m_cluthFrictionTorque;
	dFloat m_engineIdleTorque;
	dFloat m_engineRPMAtIdleTorque;
	dFloat m_enginePeakTorque;
	dFloat m_engineRPMAtPeakTorque;
	dFloat m_enginePeakHorsePower;
	dFloat m_egineRPMAtPeakHorsePower;
	dFloat m_engineRPMAtRedLine;
	dFloat m_vehicleTopSpeed;
	dFloat m_corneringStiffness;
	dFloat m_tireAligningMomemtTrail;
	dFloat m_tireSuspensionSpringConstant;
	dFloat m_tireSuspensionDamperConstant;
	dFloat m_tireSuspensionLength;
	dFloat m_tireBrakesTorque;
	dFloat m_tirePivotOffset;
	dFloat m_tireVerticalOffsetTweak;
	dFloat m_transmissionGearRatio0;
	dFloat m_transmissionGearRatio1;
	dFloat m_transmissionGearRatio2;
	dFloat m_transmissionGearRatio3;
	dFloat m_transmissionGearRatio4;
	dFloat m_transmissionGearRatio6;
	dFloat m_transmissionRevereGearRatio;
	dFloat m_chassisYaxisComBias;
	dFloat m_aerodynamicsDownForceWeightCoeffecient0;
	dFloat m_aerodynamicsDownForceWeightCoeffecient1;
	dFloat m_aerodynamicsDownForceSpeedFactor;
	int m_wheelHasCollisionFenders;
	int m_differentialType; // 0 rear wheel drive, 1 front wheel drive, 3 four wheel drive
	dSuspensionType m_tireSuspensionType;
};

static CarDefinition monsterTruck = 
{
	(3380.0f * 0.454f),							// MASS
	100.0f,										// ENGINE_MASS
	40.0f,										// TIRE_MASS	
	0.125f,										// ENGINE_RADIO
	20.0f,										// FRONT_AXEL_TIRE_STEER_ANGLE
	0.0f,										// REAR_AXEL_TIRE_STEER_ANGLE
	0.55f,										// VEHICLE_WEIGHT_DISTRIBUTION
	2000.0f,									// CLUTCH_FRICTION_TORQUE
	200.0f,										// IDLE_TORQUE
	800.0f,										// IDLE_TORQUE_RPM
	500.0f,										// PEAK_TORQUE
	3000.0f,									// PEAK_TORQUE_RPM
	400.0f,										// PEAK_HP
	5200.0f,									// PEAK_HP_RPM
	6000.0f,									// REDLINE_TORQUE_RPM
	264.0f,										// VEHICLE_TOP_SPEED_KMH
	4.0f,										// TIRE_CORNERING_STIFFNESS
	0.5f,										// TIRE_ALIGNING_MOMENT_TRAIL
	4000.0f,									// TIRE_SUSPENSION_SPRING
	200.0f,										// TIRE_SUSPENSION_DAMPER
	0.25f,										// TIRE_SUSPENSION_LENGTH
	2000.0f,									// TIRE_BRAKE_TORQUE
	0.0f,										// TIRE_PIVOT_OFFSET_Z
	0.0f,										// TIRE_PIVOT_OFFSET_Y
	2.66f,										// TIRE_GEAR_1
	1.78f,										// TIRE_GEAR_2
	1.30f,										// TIRE_GEAR_3
	1.00f,										// TIRE_GEAR_4
	0.74f,										// TIRE_GEAR_5
	0.50f,										// TIRE_GEAR_6
	2.90f,										// TIRE_REVERSE_GEAR
	-0.35f,										// COM_Y_OFFSET
	1.0f,										// DOWNFORCE_WEIGHT_FACTOR_0
	2.0f,										// DOWNFORCE_WEIGHT_FACTOR_1
	80.0f, 										// DOWNFORCE_WEIGHT_FACTOR_SPEED
	0,											// WHEEL_HAS_FENDER
	2,											// DIFFERENTIAL_TYPE
	m_confort,									// TIRE_SUSPENSION_TYPE
};

static CarDefinition viper = 
{
	(3380.0f * 0.454f),							// MASS
	100.0f,										// ENGINE_MASS
	40.0f,										// TIRE_MASS	
	0.125f,										// ENGINE_RADIO
	25.0f,										// FRONT_AXEL_TIRE_STEER_ANGLE
	 0.0f,										// REAR_AXEL_TIRE_STEER_ANGLE
	0.6f,										// VEHICLE_WEIGHT_DISTRIBUTION
	2000.0f,									// CLUTCH_FRICTION_TORQUE
	100.0f,										// IDLE_TORQUE
	450.0f,										// IDLE_TORQUE_RPM
	500.0f,										// PEAK_TORQUE
	3000.0f,									// PEAK_TORQUE_RPM
	400.0f,										// PEAK_HP
	5200.0f,									// PEAK_HP_RPM
	6000.0f,									// REDLINE_TORQUE_RPM
	264.0f,										// VEHICLE_TOP_SPEED_KMH
	4.0f,										// TIRE_CORNERING_STIFFNESS
	0.5f,										// TIRE_ALIGNING_MOMENT_TRAIL
	30000.0f,									// TIRE_SUSPENSION_SPRING
	700.0f,										// TIRE_SUSPENSION_DAMPER
	0.25f,										// TIRE_SUSPENSION_LENGTH
	2000.0f,									// TIRE_BRAKE_TORQUE
	0.0f,										// TIRE_PIVOT_OFFSET_Z
	0.0f,										// TIRE_PIVOT_OFFSET_Y
	2.66f,										// TIRE_GEAR_1
	1.78f,										// TIRE_GEAR_2
	1.30f,										// TIRE_GEAR_3
	1.00f,										// TIRE_GEAR_4
	0.74f,										// TIRE_GEAR_5
	0.50f,										// TIRE_GEAR_6
	2.90f,										// TIRE_REVERSE_GEAR
	0.0f,										// COM_Y_OFFSET
	1.0f,										// DOWNFORCE_WEIGHT_FACTOR_0
	2.0f,										// DOWNFORCE_WEIGHT_FACTOR_1
	80.0f, 										// DOWNFORCE_WEIGHT_FACTOR_SPEED
	1,											// WHEEL_HAS_FENDER
	0,											// DIFFERENTIAL_TYPE
	m_confort,									// TIRE_SUSPENSION_TYPE
};


class SuperCarEntity: public DemoEntity
{
	public: 
	enum DrivingState
	{
		m_engineOff,
		m_engineIdle,
		m_engineStop,
		m_driveForward,
		m_preDriveForward,
		m_driveReverse,
		m_preDriveReverse,
	};

	class TireVehControllerSaved: public UserData
	{
		public: 
		TireVehControllerSaved (dCustomVehicleController* const controller)
			:UserData()
			,m_controller(controller)
		{
		}

		virtual void OnRender (dFloat timestep) const{};
		virtual void OnInterpolateMatrix (DemoEntityManager& world, dFloat param) const{};

		dCustomVehicleController* m_controller;
	};


	SuperCarEntity (DemoEntityManager* const scene, dCustomVehicleControllerManager* const manager, const dMatrix& location, const char* const filename, dFloat distanceToPath, const CarDefinition& definition)
		:DemoEntity (dGetIdentityMatrix(), NULL)
		,m_controller(NULL)
		,m_engineDifferentialLock(false)
		,m_automaticTransmission(true)
		,m_engineRPMOn(false)
		,m_distanceToPath(distanceToPath)
		,m_drivingState(m_engineOff)
	{
		// add this entity to the scene for rendering
		scene->Append(this);

		// load the visual mesh 
		LoadNGD_mesh (filename, scene->GetNewton());

		// place entity in the world
		ResetMatrix (*scene, location);
		NewtonWorld* const world = scene->GetNewton();

		// create the car rigid body
		//NewtonBody* const body = CreateChassisBody (carEntity, world, materialID, params);
		NewtonCollision* const chassisCollision = CreateChassisCollision (world);

		// create the vehicle controller
		dMatrix chassisMatrix;
#if 1
		chassisMatrix.m_front = dVector (1.0f, 0.0f, 0.0f, 0.0f);			// this is the vehicle direction of travel
#else
		chassisMatrix.m_front = dVector (0.0f, 0.0f, 1.0f, 0.0f);			// this is the vehicle direction of travel
#endif
		chassisMatrix.m_up = dVector(0.0f, 1.0f, 0.0f, 0.0f);			// this is the downward vehicle direction
		chassisMatrix.m_right = chassisMatrix.m_front.CrossProduct(chassisMatrix.m_up);	// this is in the side vehicle direction (the plane of the wheels)
		chassisMatrix.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);

		// create a default vehicle 
		m_controller = manager->CreateVehicle (chassisCollision, chassisMatrix, definition.m_vehicleMass, PhysicsApplyGravityForce, DEMO_GRAVITY);

		// get body from player
		NewtonBody* const body = m_controller->GetBody();

		// set the user data
		NewtonBodySetUserData(body, this);

		// set the transform callback
		NewtonBodySetTransformCallback (body, DemoEntity::TransformCallback);

		// set the player matrix 
		NewtonBodySetMatrix(body, &location[0][0]);

		// destroy the collision helper shape 
		NewtonDestroyCollision(chassisCollision);

		for (int i = 0; i < int ((sizeof (m_gearMap) / sizeof (m_gearMap[0]))); i ++) {
			m_gearMap[i] = i;
		}
	}

	~SuperCarEntity ()
	{
		((dCustomVehicleControllerManager*)m_controller->GetManager())->DestroyController(m_controller);
	}

	void SetGearMap(dEngineController* const engine)
	{
		int start = engine->GetFirstGear();
		int count = engine->GetLastGear() - start;
		for (int i = 0; i < count; i++) {
			m_gearMap[i + 2] = start + i;
		}
		m_gearMap[0] = engine->GetNeutralGear();
		m_gearMap[1] = engine->GetReverseGear();
	}

	NewtonCollision* CreateChassisCollision (NewtonWorld* const world) const
	{
		DemoEntity* const chassis = dHierarchy<DemoEntity>::Find("car_body");
		dAssert (chassis);

		DemoMesh* const mesh = (DemoMesh*)chassis->GetMesh();
		dAssert (mesh->IsType(DemoMesh::GetRttiType()));
		//dAssert (chassis->GetMeshMatrix().TestIdentity());
		const dMatrix& meshMatrix = chassis->GetMeshMatrix();

		dFloat* const temp = new dFloat[mesh->m_vertexCount * 3];
		meshMatrix.TransformTriplex (&temp[0], 3 * sizeof (dFloat), mesh->m_vertex, 3 * sizeof (dFloat), mesh->m_vertexCount);
		NewtonCollision* const shape = NewtonCreateConvexHull (world, mesh->m_vertexCount, &temp[0], 3 * sizeof (dFloat), 0.001f, 0, NULL);
		delete[] temp;
		return shape;
	}

	NewtonBody* CreateChassisBody (NewtonWorld* const world) const
	{
		dAssert (0);
		return NULL;
	}

	// interpolate all skeleton transform 
	virtual void InterpolateMatrix (DemoEntityManager& world, dFloat param)
	{
		DemoEntity::InterpolateMatrix (world, param);
		if (m_controller){
			for (dList<NewtonBody*>::dListNode* node = m_controller->GetFirstBodyPart()->GetNext(); node; node = m_controller->GetNextBodyPart(node)) {
				NewtonBody* const body = node->GetInfo();
				DemoEntity* const entPart = (DemoEntity*)NewtonBodyGetUserData(body);
				if (entPart) {
					entPart->InterpolateMatrix(world, param);
				}
			}
		}
	}

	void CalculateTireDimensions (const char* const tireName, dFloat& width, dFloat& radius) const
	{
		NewtonBody* const body = m_controller->GetBody();
		NewtonWorld* const world = NewtonBodyGetWorld(body);

		// find the the tire visual mesh 
		DemoEntity* const entity = (DemoEntity*) NewtonBodyGetUserData(body);
		DemoEntity* const tirePart = entity->Find (tireName);
		dAssert (tirePart);

		// make a convex hull collision shape to assist in calculation of the tire shape size
		DemoMesh* const tireMesh = (DemoMesh*)tirePart->GetMesh();
		dAssert (tireMesh->IsType(DemoMesh::GetRttiType()));
		//dAssert (tirePart->GetMeshMatrix().TestIdentity());
		const dMatrix& meshMatrix = tirePart->GetMeshMatrix();
		dVector* const temp = new dVector [tireMesh->m_vertexCount];
		meshMatrix.TransformTriplex (&temp[0].m_x, sizeof (dVector), tireMesh->m_vertex, 3 * sizeof (dFloat), tireMesh->m_vertexCount);
		NewtonCollision* const collision = NewtonCreateConvexHull(world, tireMesh->m_vertexCount, &temp[0].m_x, sizeof (dVector), 0, 0, NULL);
		delete[] temp;

		// get the location of this tire relative to the car chassis
		dMatrix tireMatrix (tirePart->CalculateGlobalMatrix(entity));

		// find the support points that can be used to define the with and high of the tire collision mesh
		dVector extremePoint(0.0f);
		dVector upDir (tireMatrix.UnrotateVector(dVector (0.0f, 1.0f, 0.0f, 0.0f)));
		NewtonCollisionSupportVertex (collision, &upDir[0], &extremePoint[0]);
		radius = dAbs (upDir.DotProduct3(extremePoint));

		dVector widthDir (tireMatrix.UnrotateVector(dVector (0.0f, 0.0f, 1.0f, 0.0f)));
		NewtonCollisionSupportVertex (collision, &widthDir[0], &extremePoint[0]);
		width = widthDir.DotProduct3(extremePoint);

		widthDir = widthDir.Scale (-1.0f);
		NewtonCollisionSupportVertex (collision, &widthDir[0], &extremePoint[0]);
		width += widthDir.DotProduct3(extremePoint);

		// destroy the auxiliary collision
		NewtonDestroyCollision (collision);	
	}

	dWheelJoint* AddTire (const char* const tireName, dFloat width, dFloat radius, dFloat pivotOffset, dFloat maxSteerAngle, const CarDefinition& definition)
	{
		NewtonBody* const chassisBody = m_controller->GetBody();
		DemoEntity* const chassisEntity = (DemoEntity*) NewtonBodyGetUserData(chassisBody);
		DemoEntity* const tirePart = chassisEntity->Find (tireName);

		// save the controller with the tire so that we can use it a callback 
		TireVehControllerSaved* const savedVehController = new TireVehControllerSaved(m_controller);
		tirePart->SetUserData(savedVehController);
		
		// for simplicity, tires are position in global space
		dMatrix tireMatrix(tirePart->CalculateGlobalMatrix());

		// add the offset to the tire position to account for suspension span
		//tireMatrix.m_posit += m_controller->GetUpAxis().Scale (definition.m_tirePivotOffset);
		tireMatrix.m_posit -= m_controller->GetUpAxis().Scale (definition.m_tireVerticalOffsetTweak);

		// add the tire to the vehicle
		dTireInfo tireInfo;

		tireInfo.m_mass = definition.m_tireMass;
		tireInfo.m_radio = radius;
		tireInfo.m_width = width;
		tireInfo.m_pivotOffset = pivotOffset;
		tireInfo.m_maxSteeringAngle = maxSteerAngle * dDegreeToRad; 
		tireInfo.m_dampingRatio = definition.m_tireSuspensionDamperConstant;
		tireInfo.m_springStrength = definition.m_tireSuspensionSpringConstant;
		tireInfo.m_suspensionLength = definition.m_tireSuspensionLength;
		tireInfo.m_corneringStiffness = definition.m_corneringStiffness;
		tireInfo.m_aligningMomentTrail = definition.m_tireAligningMomemtTrail;
		tireInfo.m_hasFender = definition.m_wheelHasCollisionFenders;
		tireInfo.m_suspentionType = definition.m_tireSuspensionType;

		dWheelJoint* const tireJoint = m_controller->AddTire (tireMatrix, tireInfo);
		NewtonBody* const tireBody = tireJoint->GetTireBody (); 

		// add the user data and a tire transform callback that calculate the tire local space matrix
		NewtonBodySetUserData (tireBody, tirePart);
		NewtonBodySetTransformCallback (tireBody, TireTransformCallback);
		return tireJoint;
	}

	// this transform make sure the tire matrix is relative to the chassis 
	// Note: this transform us only use because the tire in the model are children of the chassis
	static void TireTransformCallback(const NewtonBody* const tireBody, const dFloat* const tireMatrix, int threadIndex)
	{
		DemoEntity* const tirePart = (DemoEntity*)NewtonBodyGetUserData(tireBody);
		TireVehControllerSaved* const tireUserData = (TireVehControllerSaved*)tirePart->GetUserData();
		dCustomVehicleController* const controller = tireUserData->m_controller;
		NewtonBody* const chassisBody = controller->GetBody();
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(NewtonBodyGetWorld(tireBody));
		dAssert (scene);

		// calculate the local space matrix,
		dMatrix parentMatrix;
		dMatrix matrix (tireMatrix);
		NewtonBodyGetMatrix(chassisBody, &parentMatrix[0][0]);
		matrix = matrix * parentMatrix.Inverse();
		dQuaternion rot (matrix);
		tirePart->SetMatrix(*scene, rot, matrix.m_posit);
	}


	// this function is an example of how to make a high performance super car
	void BuildWheelCar (const CarDefinition& definition)
	{

		// step one: find the location of each tire, in the visual mesh and add them one by one to the vehicle controller 
		dFloat width;
		dFloat radius;

		// Muscle cars have the front engine, we need to shift the center of mass to the front to represent that
		m_controller->SetCenterOfGravity (dVector (0.0f, definition.m_chassisYaxisComBias, 0.0f, 0.0f)); 

		// add front axle
		// a car may have different size front an rear tire, therefore we do this separate for front and rear tires
		CalculateTireDimensions ("fl_tire", width, radius);
//		dVector offset (0.0f, 0.0f, 0.0f, 0.0f);
		dWheelJoint* const leftFrontTire = AddTire ("fl_tire", width, radius, 0.25f, definition.m_frontSteeringAngle, definition);
		dWheelJoint* const rightFrontTire = AddTire ("fr_tire", width, radius, -0.25f, definition.m_frontSteeringAngle, definition);

		// add rear axle
		// a car may have different size front an rear tire, therefore we do this separate for front and rear tires
		CalculateTireDimensions ("rl_tire", width, radius);
		dWheelJoint* const leftRearTire = AddTire ("rl_tire", width, radius, 0.0f, definition.m_rearSteeringAngle, definition);
		dWheelJoint* const rightRearTire = AddTire ("rr_tire", width, radius, 0.0f, definition.m_rearSteeringAngle, definition);
		
		// add a steering Wheel component
		dSteeringController* const steering = new dSteeringController(m_controller);
		steering->AddTire(leftFrontTire);
		steering->AddTire(rightFrontTire);
		//steering->AddTire(leftRearTire);
		//steering->AddTire(rightRearTire);
		m_controller->SetSteering(steering);

		// add vehicle brakes
		dBrakeController* const brakes = new dBrakeController (m_controller, definition.m_tireBrakesTorque);
		brakes->AddTire (leftFrontTire);
		brakes->AddTire (rightFrontTire);
		brakes->AddTire (leftRearTire);
		brakes->AddTire (rightRearTire);
		m_controller->SetBrakes(brakes);

		// add vehicle hand brakes
		dBrakeController* const handBrakes = new dBrakeController (m_controller, definition.m_tireBrakesTorque * 2.0f);
		handBrakes->AddTire (leftRearTire);
		handBrakes->AddTire (rightRearTire);
		m_controller->SetHandBrakes(handBrakes);

		// add the engine, differential and transmission 
		dEngineInfo engineInfo;
		engineInfo.m_mass = definition.m_engineMass; 
		engineInfo.m_radio = definition.m_engineRotorRadio; 
		engineInfo.m_vehicleTopSpeed = definition.m_vehicleTopSpeed;
		engineInfo.m_clutchFrictionTorque = definition.m_cluthFrictionTorque;

		engineInfo.m_idleTorque = definition.m_engineIdleTorque;
		engineInfo.m_rpmAtIdleTorque = definition.m_engineRPMAtIdleTorque;
		engineInfo.m_peakTorque = definition.m_enginePeakTorque;
		engineInfo.m_rpmAtPeakTorque = definition.m_engineRPMAtPeakTorque;
		engineInfo.m_peakHorsePower = definition.m_enginePeakHorsePower;
		engineInfo.m_rpmAtPeakHorsePower = definition.m_egineRPMAtPeakHorsePower;
		engineInfo.m_rpmAtRedLine = definition.m_engineRPMAtRedLine;

		engineInfo.m_gearsCount = 6;
		engineInfo.m_gearRatios[0] = definition.m_transmissionGearRatio0;
		engineInfo.m_gearRatios[1] = definition.m_transmissionGearRatio1;
		engineInfo.m_gearRatios[2] = definition.m_transmissionGearRatio2;
		engineInfo.m_gearRatios[3] = definition.m_transmissionGearRatio3;
		engineInfo.m_gearRatios[4] = definition.m_transmissionGearRatio4;
		engineInfo.m_gearRatios[5] = definition.m_transmissionGearRatio6;
		engineInfo.m_reverseGearRatio = definition.m_transmissionRevereGearRatio;
		engineInfo.m_gearRatiosSign = 1.0f;

		dDifferentialJoint* differential = NULL;
		switch (definition.m_differentialType) 
		{
			case 0:
			{
				// rear wheel drive differential vehicle
				engineInfo.m_gearRatiosSign = 1.0f;
				differential = m_controller->AddDifferential(leftRearTire, rightRearTire);
				break;
			}

			case 1:
			{
				// front wheel drive vehicle with differential
				engineInfo.m_gearRatiosSign = 1.0f;
				differential = m_controller->AddDifferential(leftFrontTire, rightFrontTire);
				break;
			}

			default:
			{
				dDifferentialJoint* const rearDifferential = m_controller->AddDifferential(leftRearTire, rightRearTire);
				dDifferentialJoint* const frontDifferential = m_controller->AddDifferential(leftFrontTire, rightFrontTire);
				differential = m_controller->AddDifferential(rearDifferential, frontDifferential);

				engineInfo.m_gearRatiosSign = -1.0f;
			}
		}
		dAssert(differential);

		engineInfo.m_differentialLock = 0;
		engineInfo.m_aerodynamicDownforceFactor = definition.m_aerodynamicsDownForceWeightCoeffecient0;
		engineInfo.m_aerodynamicDownforceFactorAtTopSpeed = definition.m_aerodynamicsDownForceWeightCoeffecient1;
		engineInfo.m_aerodynamicDownForceSurfaceCoeficident = definition.m_aerodynamicsDownForceSpeedFactor / definition.m_vehicleTopSpeed;

		m_controller->AddEngineJoint (engineInfo.m_mass, engineInfo.m_radio);
		dEngineController* const engineControl = new dEngineController (m_controller, engineInfo, differential, rightRearTire);
		m_controller->SetEngine(engineControl);

		// the the default transmission type
		engineControl->SetTransmissionMode(m_automaticTransmission.GetPushButtonState() ? true : false);
		engineControl->SetIgnition(true);

		// set the gear look up table
		SetGearMap(engineControl);

		// set the vehicle weigh distribution 
		m_controller->SetWeightDistribution (definition.m_vehicleWeightDistribution);

		// set the vehicle aerodynamics
		dFloat weightRatio0 = definition.m_aerodynamicsDownForceWeightCoeffecient0;
		dFloat weightRatio1 = definition.m_aerodynamicsDownForceWeightCoeffecient1;
		dFloat speedFactor = definition.m_aerodynamicsDownForceSpeedFactor / definition.m_vehicleTopSpeed;
		m_controller->SetAerodynamicsDownforceCoefficient(weightRatio0, speedFactor, weightRatio1);

		// do not forget to call finalize after all components are added or after any change is made to the vehicle
		m_controller->Finalize();
	}

	dFloat CalculateNPCControlSteerinValue (dFloat distanceAhead, dFloat pathWidth, DemoEntity* const pathEntity)
	{
		dAssert (0);
		return 0;
	}

	void ApplyNPCControl (dFloat timestep, DemoEntity* const pathEntity)
	{
	}

	dCustomVehicleController* m_controller;
	DemoEntityManager::ButtonKey m_engineDifferentialLock;
	DemoEntityManager::ButtonKey m_automaticTransmission;
	int m_gearMap[10];
	bool m_engineRPMOn;
	dFloat m_distanceToPath;
	dVector m_debugTargetHeading; 
	DrivingState m_drivingState;
};


class SuperCarVehicleControllerManager: public dCustomVehicleControllerManager
{
	public:
	SuperCarVehicleControllerManager (NewtonWorld* const world, int materialsCount, int* const materialList)
		:dCustomVehicleControllerManager (world, materialsCount, materialList)
		,m_externalView(true)
		,m_player (NULL) 
		,m_steeringAxis(0)
		,m_throttleAxis(1)
		,m_clutchAxis(2)
		,m_ignitionButton(0)
		,m_handBrakeButton(1)
		,m_gearUpButton(2)
		,m_gearDonwButton(3)
		,m_nexVehicle (true)
		,m_gearUpKey(false)
		,m_gearDownKey(false)
		,m_engineKeySwitch(false)
	{
		// hook a callback for 2d help display
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);

		scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
		scene->Set2DDisplayRenderFunction (RenderHelpMenu, RenderUI, this);

		// load 2d display assets
		m_gears = LoadTexture ("gears_font.tga");
		m_tachometer = LoadTexture ("rpm_dial.tga");
		m_odometer = LoadTexture ("kmh_dial.tga");
		m_redNeedle = LoadTexture ("needle_red.tga");
		m_greenNeedle = LoadTexture ("needle_green.tga");

		char joystickMappingName[2048];
		dGetWorkingFileName ("vehicleJoystick.txt", joystickMappingName);
		FILE* const joysticMapping = fopen (joystickMappingName, "rt");
		dAssert (joysticMapping);

		fscanf (joysticMapping, "steeringAxis:%d\n", &m_steeringAxis);
		fscanf (joysticMapping, "throttleAxis:%d\n", &m_throttleAxis);
		fscanf (joysticMapping, "clutchAxis:%d\n", &m_clutchAxis);
		fscanf (joysticMapping, "ignitionButton:%d\n", &m_ignitionButton);
		fscanf (joysticMapping, "handBrakeButton:%d\n", &m_handBrakeButton);
		fscanf (joysticMapping, "gearUpButton:%d\n", &m_gearUpButton);
		fscanf (joysticMapping, "gearDonwButton:%d\n", &m_gearDonwButton);

		fclose (joysticMapping);
	}

	~SuperCarVehicleControllerManager ()
	{
		ReleaseTexture (m_gears);
		ReleaseTexture (m_tachometer);
		ReleaseTexture (m_odometer);
		ReleaseTexture (m_redNeedle);
		ReleaseTexture (m_greenNeedle);
	}
	

	static void RenderHelpMenu (DemoEntityManager* const scene, void* const context)
	{
		SuperCarVehicleControllerManager* const me = (SuperCarVehicleControllerManager*) context;
		//me->RenderVehicleHud (scene);
		me->DrawHelp(scene);
	}

	static void RenderUI (DemoEntityManager* const scene, void* const context)
	{
		SuperCarVehicleControllerManager* const me = (SuperCarVehicleControllerManager*)context;
		me->RenderUI (scene);
	}

	static void UpdateCameraCallback(DemoEntityManager* const manager, void* const context, dFloat timestep)
	{
		SuperCarVehicleControllerManager* const me = (SuperCarVehicleControllerManager*)context;
		me->UpdateCamera(timestep);
	}

	void UpdateCamera(dFloat timestep)
	{
		SuperCarEntity* player = m_player;
		if (!player) {
			dCustomVehicleController* const controller = &GetLast()->GetInfo();
			player = (SuperCarEntity*)NewtonBodyGetUserData(controller->GetBody());
		}

		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(GetWorld());
		DemoCamera* const camera = scene->GetCamera();
		dMatrix camMatrix(camera->GetNextMatrix());
		dMatrix playerMatrix(player->GetNextMatrix());

		dVector frontDir(camMatrix[0]);
		dVector camOrigin(0.0f);
		if (m_externalView) {
			camOrigin = playerMatrix.m_posit + dVector(0.0f, VEHICLE_THIRD_PERSON_VIEW_HIGHT, 0.0f, 0.0f);
			camOrigin -= frontDir.Scale(VEHICLE_THIRD_PERSON_VIEW_DIST);
			//camOrigin = dVector (-7.0f, 3.0f, 0.0f, 0.0f);
		} else {
			dAssert(0);
			//           camMatrix = camMatrix * playerMatrix;
			//           camOrigin = playerMatrix.TransformVector(dVector(-0.8f, ARTICULATED_VEHICLE_CAMERA_EYEPOINT, 0.0f, 0.0f));
		}
		camera->SetNextMatrix(*scene, camMatrix, camOrigin);
	}

	void DrawHelp(DemoEntityManager* const scene)
	{
		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print(color, "Vehicle driving           keyboard control:   Joystick control");
		scene->Print(color, "engine switch             : 'I'               start engine");
		scene->Print(color, "accelerator               : 'W'               stick forward");
		scene->Print(color, "brakes                    : 'S'               stick back");
		scene->Print(color, "turn left                 : 'A'               stick left");
		scene->Print(color, "turn right                : 'D'               stick right");
		scene->Print(color, "engage clutch             : 'K'               button 5");
		scene->Print(color, "engage differential lock  : 'L'               button 5");
		scene->Print(color, "gear up                   : 'M'               button 2");
		scene->Print(color, "gear down                 : 'N'               button 3");
		scene->Print(color, "manual transmission       : enter             button 4");
		scene->Print(color, "hand brakes               : space             button 1");
		scene->Print(color, "next vehicle              : 'V'");
	}

	void DrawGage(GLuint gage, GLuint needle, dFloat param, dFloat origin_x, dFloat origin_y, dFloat size) const
	{
		size *= 0.5f;
		dMatrix origin (dGetIdentityMatrix());
		origin[1][1] = -1.0f;
		origin.m_posit = dVector(origin_x, origin_y, 0.0f, 1.0f);

		// render dial
		glPushMatrix();
		glMultMatrix (&origin[0][0]);
		glBindTexture(GL_TEXTURE_2D, gage);
		glBegin(GL_QUADS);
		glTexCoord2f(0.0f, 1.0f); glVertex3f(GLfloat(-size), GLfloat( size), 0.0f);
		glTexCoord2f(0.0f, 0.0f); glVertex3f(GLfloat(-size), GLfloat(-size), 0.0f);
		glTexCoord2f(1.0f, 0.0f); glVertex3f(GLfloat( size), GLfloat(-size), 0.0f);
		glTexCoord2f(1.0f, 1.0f); glVertex3f(GLfloat( size), GLfloat( size), 0.0f);
		glEnd();

		// render needle
		const dFloat minAngle = 180.0f * dDegreeToRad;
		const dFloat maxAngle = -90.0f * dDegreeToRad;
		dFloat angle = minAngle + (maxAngle - minAngle) * param;
		dMatrix needleMatrix (dRollMatrix (angle));

		dFloat x = size * 0.7f;
		dFloat y = size * 0.7f;

		glPushMatrix();
		glMultMatrix (&needleMatrix[0][0]);
		glBindTexture(GL_TEXTURE_2D, needle);
		glBegin(GL_QUADS);
		glTexCoord2f(0.0f, 1.0f); glVertex3f(GLfloat(-x), GLfloat( y), 0.0f);
		glTexCoord2f(0.0f, 0.0f); glVertex3f(GLfloat(-x), GLfloat(-y), 0.0f);
		glTexCoord2f(1.0f, 0.0f); glVertex3f(GLfloat( x), GLfloat(-y), 0.0f);
		glTexCoord2f(1.0f, 1.0f); glVertex3f(GLfloat( x), GLfloat( y), 0.0f);
		glEnd();

		glPopMatrix();
		glPopMatrix();
	}

	void DrawGear(dFloat param, dFloat origin_x, dFloat origin_y, int gear, dFloat size) const
	{
		dMatrix origin (dGetIdentityMatrix());
		origin[1][1] = -1.0f;
		origin.m_posit = dVector(origin_x + size * 0.3f, origin_y - size * 0.25f, 0.0f, 1.0f);

		glPushMatrix();
		glMultMatrix (&origin[0][0]);

		dFloat uwith = 0.1f;
		dFloat u0 = uwith * gear;
		dFloat u1 = u0 + uwith;

		dFloat x1 = 10.0f;
		dFloat y1 = 10.0f;
		glColor4f(1, 1, 0, 1);
		glBindTexture(GL_TEXTURE_2D, m_gears);
		glBegin(GL_QUADS);
		glTexCoord2f(GLfloat(u0), 1.0f); glVertex3f(GLfloat(-x1), GLfloat( y1), 0.0f);
		glTexCoord2f(GLfloat(u0), 0.0f); glVertex3f(GLfloat(-x1), GLfloat(-y1), 0.0f);
		glTexCoord2f(GLfloat(u1), 0.0f); glVertex3f(GLfloat( x1), GLfloat(-y1), 0.0f);
		glTexCoord2f(GLfloat(u1), 1.0f); glVertex3f(GLfloat( x1), GLfloat( y1), 0.0f);
		glEnd();

		glPopMatrix();
	}

	void RenderUI (DemoEntityManager* const scene)
	{
		// set to transparent color
		if (m_player) {
			dEngineController* const engine = m_player->m_controller->GetEngine();
			if (engine) {
				glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
				dFloat gageSize = 200.0f;
				dFloat y = scene->GetHeight() - (gageSize / 2.0f + 20.0f);

				// draw the tachometer
				dFloat x = gageSize / 2 + 20.0f;
				dFloat rpm = engine->GetRPM () / engine->GetRedLineRPM();
				DrawGage(m_tachometer, m_redNeedle, rpm, x, y, gageSize);

				// draw the odometer
				x += gageSize;
				//dFloat speed = dAbs(engine->GetSpeed()) * 3.6f / 340.0f;
				dFloat speed = dAbs(engine->GetSpeed())  / engine->GetTopSpeed();
				DrawGage(m_odometer, m_greenNeedle, speed, x, y, gageSize);

				// draw the current gear
				int gear = engine->GetGear();
				DrawGear(speed, x, y + 98, m_player->m_gearMap[gear], gageSize);
			}
		}
	}

	void SetAsPlayer (SuperCarEntity* const player)
	{
		dEngineController* const engine = player->m_controller->GetEngine();
		if (engine) {
			engine->SetIgnition(false);
		}
		m_player = player;
	}

	void SetNextPlayer() 
	{
		dListNode* const playerNode = GetNodeFromInfo (*m_player->m_controller);
		dListNode* const nextPlayerNode = playerNode->GetNext() ? playerNode->GetNext() : GetFirst();

		NewtonBody* const vehicleBody = nextPlayerNode->GetInfo().GetBody();
		SuperCarEntity* const player = (SuperCarEntity*) NewtonBodyGetUserData(vehicleBody);
		SetAsPlayer (player);
	}

	void ApplyPlayerControl (dCustomVehicleController* const vehicle, dFloat timestep)
	{
		NewtonBody* const body = vehicle->GetBody();
		NewtonWorld* const world = NewtonBodyGetWorld(body);
		//SuperCarEntity* const vehicleEntity = (SuperCarEntity*)NewtonBodyGetUserData(body);
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);
		dEngineController* const engine = vehicle->GetEngine();

		int gear = engine ? engine->GetGear() : 0;

		dVehicleDriverInput driverInput;
		dFloat axis[32];
		int axisCount = scene->GetJoystickAxis (axis);
		if (axisCount) {
			dFloat joyPosX;
			dFloat joyPosY;
			dFloat joyPosZ;
			
			char buttons[32];
			scene->GetJoystickButtons (buttons);

			joyPosX = axis[m_steeringAxis];
			joyPosY = -axis[m_throttleAxis];
			joyPosZ = dMax (axis[m_clutchAxis], dFloat (0.0f));
			bool ignitionButton = buttons[m_ignitionButton] ? true : false;
			bool handBreakButton = buttons[m_handBrakeButton] ? true : false;
			bool gearUpButton = buttons[m_gearUpButton] ? true : false;
			bool gearDownButton = buttons[m_gearDonwButton] ? true : false;

			gear += (int (m_gearUpKey.UpdateTrigger(gearUpButton)) - int (m_gearDownKey.UpdateTrigger(gearDownButton))); 
			
			driverInput.m_clutchPedal = joyPosZ * joyPosZ * joyPosZ;
			driverInput.m_steeringValue = joyPosX * joyPosX * joyPosX;
			driverInput.m_throttle = joyPosY > 0.0f ? dAbs(joyPosY * joyPosY * joyPosY) : 0.0f;
			driverInput.m_brakePedal = joyPosY < 0.0f ? dAbs (joyPosY * joyPosY * joyPosY) : 0.0f;

			driverInput.m_ignitionKey = m_engineKeySwitch.UpdatePushButton(ignitionButton);
			driverInput.m_handBrakeValue = handBreakButton ? 1.0f : 0.0f;
			driverInput.m_gear = gear;

			//for (int i = 0; i < joyButtons; i ++) {
			//	dTrace(("%d ", buttons[i]));
			//}
			//dTrace(("\n"));
			//dTrace (("%f %f %f %f\n", driverInput.m_steeringValue, driverInput.m_throttle, driverInput.m_brakePedal, driverInput.m_clutchPedal));
			//dTrace (("%d %d %d\n", gear, ignitionButton, m_engineKeySwitch.GetPushButtonState()));

		} else {
			driverInput.m_throttle = scene->GetKeyState('W') ? 1.0f : 0.0f;
			driverInput.m_clutchPedal = 1.0f - scene->GetKeyState('K') ? 1.0f : 0.0f;
			driverInput.m_steeringValue = (dFloat(scene->GetKeyState('D')) - dFloat(scene->GetKeyState('A')));
			driverInput.m_brakePedal = scene->GetKeyState('S') ? 1.0f : 0.0f;
			driverInput.m_ignitionKey = m_engineKeySwitch.UpdatePushButton(scene->GetKeyState ('I'));
			driverInput.m_handBrakeValue = scene->GetKeyState(' ') ? 1.0f : 0.0f;
			//driverInput.m_manualTransmission = !m_automaticTransmission.UpdatePushButton (scene, 0x0d);
			gear += m_gearUpKey.UpdateTrigger(scene->GetKeyState('M')) - m_gearUpKey.UpdateTrigger(scene->GetKeyState('N')); 
			driverInput.m_gear = gear;
			//driverInput.m_lockDifferential = m_engineDifferentialLock.UpdatePushButton(scene, 'L');
		}

//xxxxxx
#if 0
	#if 0
		static FILE* file = fopen ("log.bin", "wb");                                         
		if (file) {
			fwrite(&driverInput, sizeof(dVehicleDriverInput), 1, file);
			fflush(file);
		}
	#else 
		static FILE* file = fopen ("log.bin", "rb");
		if (file) {		
			fread(&driverInput, sizeof(dVehicleDriverInput), 1, file);
		}
	#endif
#endif

		vehicle->ApplyDefualtDriver(driverInput, timestep);
	}


	void UpdateDriverInput(dCustomVehicleController* const vehicle, dFloat timestep)
	{
		NewtonBody* const body = vehicle->GetBody();
		SuperCarEntity* const vehicleEntity = (SuperCarEntity*)NewtonBodyGetUserData(body);

		if (vehicleEntity == m_player) {
			// do player control
			ApplyPlayerControl(vehicle, timestep);
		} else {
			dAssert (0);
			// do no player control
			//vehicleEntity->ApplyNPCControl (timestep, m_raceTrackPath);
		}
	}

	virtual void PreUpdate (dFloat timestep)
	{
		// apply the vehicle controls, and all simulation time effect
		//NewtonWorld* const world = GetWorld(); 
		//DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);

		// do the base class post update
		dCustomVehicleControllerManager::PreUpdate(timestep);
	}

	virtual void PostUpdate (dFloat timestep)
	{
		// do the base class post update
		dCustomVehicleControllerManager::PostUpdate(timestep);
	}


	dMatrix CalculateSplineMatrix (dFloat64 param) const
	{
		DemoBezierCurve* const path = (DemoBezierCurve*) m_raceTrackPath->GetMesh();

		dBigVector origin (path->m_curve.CurvePoint(param)); 
		dBigVector tangent (path->m_curve.CurveDerivative(param, 1)); 
		tangent = tangent.Scale (1.0 / sqrt (tangent.DotProduct3(tangent)));
		dBigVector p1 (origin + tangent);

		dBigVector dir; 
		path->m_curve.FindClosestKnot (dir, p1, 4);

		dir -= origin;
		dir = dir.Scale (1.0 / sqrt (dir.DotProduct3(dir)));
		dMatrix matrix (dGetIdentityMatrix());

		dMatrix pathMatrix (m_raceTrackPath->GetMeshMatrix() * m_raceTrackPath->GetCurrentMatrix());
		matrix.m_front = pathMatrix.RotateVector (dir);
		matrix.m_right = matrix.m_front.CrossProduct(matrix.m_up);
		matrix.m_right.m_w = 0.0f;
		matrix.m_posit = pathMatrix.TransformVector(origin);
		return matrix;
	}


	void AddCones (DemoEntityManager* const scene)
	{
		DemoBezierCurve* const path = (DemoBezierCurve*) m_raceTrackPath->GetMesh();

		// load a street cone model and place at equal distance along the path
		DemoEntity* const coneEnt = new DemoEntity(dGetIdentityMatrix(), NULL);
		coneEnt->LoadNGD_mesh ("streetCone.ngd", scene->GetNewton());
		DemoMesh* const mesh = (DemoMesh*) coneEnt->GetMesh();
		dAssert (mesh);

		NewtonWorld* const world = scene->GetNewton();
		dMatrix offsetMatrix (coneEnt->GetMeshMatrix());
		NewtonCollision* const collision = NewtonCreateConvexHull(world, mesh->m_vertexCount, mesh->m_vertex, 3 * sizeof (dFloat), 1.0e-3f, 0, &offsetMatrix[0][0]);

		dFloat64 slineLength = path->m_curve.CalculateLength(0.01f);
		int segmentCount = int (slineLength / 5.0f);

		dFloat64 step = slineLength / segmentCount;

		dFloat streetWidth = 8.0f;

		dFloat64 acc = 0.0f;
		dFloat64 dist = 0.0;
		dBigVector p0 (path->m_curve.CurvePoint(0.0f));
		for (int i = 1; i < 1000; i ++) {
			dFloat64 t = dFloat64 (i) / 1000.0f;
			dBigVector p1 (path->m_curve.CurvePoint(t));
			dBigVector dp (p1 - p0);
			dist += sqrt (dp.DotProduct3(dp));
			if (dist > acc) {
				acc += step;
				dMatrix matrix (CalculateSplineMatrix  (dFloat64 (i-1) / 1000.0f));
				matrix.m_posit += matrix.m_right.Scale (streetWidth);
				CreateSimpleSolid (scene, mesh, 20.0f, matrix, collision, 0);
				matrix.m_posit -= matrix.m_right.Scale (streetWidth * 2.0f);
				CreateSimpleSolid (scene, mesh, 20.0f, matrix, collision, 0);
			}
			p0 = p1;
		}

		NewtonDestroyCollision(collision);
		coneEnt->Release();
	}

	void CreatedrivingTestCourt(DemoEntityManager* const scene)
	{
		// create a Bezier Spline path for AI car to drive
		m_raceTrackPath = new DemoEntity (dGetIdentityMatrix(), NULL);
		scene->Append(m_raceTrackPath);
		m_raceTrackPath->LoadNGD_mesh ("carPath.ngd", scene->GetNewton());
		DemoBezierCurve* const path = (DemoBezierCurve*) m_raceTrackPath->GetMesh();
		dAssert (path->IsType(DemoBezierCurve::GetRttiType()));

		path->SetVisible(true);
		path->SetRenderResolution (500);
	}

	void OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
	{
		dCustomVehicleControllerManager::OnDebug(debugContext);
		//draw the schematic (laterals forces diagram for the player vehicle) 
		//m_player->m_controller->DrawSchematic(debugContext, 150.0f, 120.0f, 60.0f);
		m_player->m_controller->DrawSchematic(debugContext, 350.0f, 220.0f, 100.0f);
	}

	bool m_externalView;
	SuperCarEntity* m_player;
	DemoEntity* m_raceTrackPath;

	GLuint m_gears;
	GLuint m_redNeedle;
	GLuint m_greenNeedle;
	GLuint m_odometer;
	GLuint m_tachometer;
	GLuint m_needle;
	int m_soundsCount;
	int m_steeringAxis;
	int m_throttleAxis;
	int m_clutchAxis;
	int m_ignitionButton;
	int m_handBrakeButton;
	int m_gearUpButton;
	int m_gearDonwButton;
	
	DemoEntityManager::ButtonKey m_nexVehicle;
	DemoEntityManager::ButtonKey m_gearUpKey;
	DemoEntityManager::ButtonKey m_gearDownKey;
	DemoEntityManager::ButtonKey m_engineKeySwitch;
	void* m_engineSounds[16];
};

#endif

// *************************************************************************************************
// 
// create a simple racing game with a simple controlled by a Newton AI engine and newton physics
//
// *************************************************************************************************
void SuperCar (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();
#if 0
	CreateLevelMesh (scene, "flatPlane.ngd", 1);
	//CreateLevelMesh (scene, "raceTrack2.ngd", 0);
	//CreateLevelMesh (scene, "raceTrack2.ngd", 1);
	//CreateLevelMesh (scene, "raceTrack1.ngd", 0);
	//CreateLevelMesh (scene, "raceTrack1.ngd", 1);
//	CreateHeightFieldTerrain (scene, 10, 8.0f, 1.5f, 0.2f, 200.0f, -50.0f);
	//CreateHeightFieldTerrain (scene, 10, 8.0f, 0.0f, 0.0f, 200.0f, -50.0f);
	//CreatePlaneCollision (scene, dVector (0.0f, 1.0f, 0.0f, 0.0f));

	NewtonWorld* const world = scene->GetNewton();

	int defaultMaterial = NewtonMaterialGetDefaultGroupID(scene->GetNewton());
	NewtonMaterialSetDefaultFriction(world, defaultMaterial, defaultMaterial, 0.6f, 0.5f);

	int materialList[] = {defaultMaterial };

	// create a vehicle controller manager
	SuperCarVehicleControllerManager* const manager = new SuperCarVehicleControllerManager (world, 1, materialList);

//	int defaultMaterial = NewtonMaterialGetDefaultGroupID(scene->GetNewton());
//	NewtonMaterialSetDefaultFriction(scene->GetNewton(), defaultMaterial, defaultMaterial, 0.9f, 0.9f);

	// create a Bezier Spline path for AI car to drive 
	manager->CreatedrivingTestCourt (scene);
//	manager->AddCones (scene);

	dFloat u = 1.0f;
	dVector offset (0.0f, 100.0f, 0.0f, 0.0f);
	for (int i = 0; i < 1; i ++) {
/*
		dMatrix location1 (manager->CalculateSplineMatrix (u));
		location1.m_posit += location1.m_right.Scale ( 3.0f);
		location1.m_posit = FindFloor (scene->GetNewton(), location1.m_posit + offset, 200.0f);
		location1.m_posit.m_y += 1.0f;
		SuperCarEntity* const vehicle1 = new SuperCarEntity (scene, manager, location1, "f1.ngd", 0.0f, viper);
		vehicle1->BuildWheelCar(viper);
		u -= 0.005f;

		dMatrix location2(manager->CalculateSplineMatrix(u));
		location2.m_posit = FindFloor(scene->GetNewton(), location2.m_posit + offset, 200.0f);
		location2.m_posit.m_y += 1.0f;
		SuperCarEntity* const vehicle2 = new SuperCarEntity(scene, manager, location2, "lambDiablo.ngd", 3.0f, viper);
		vehicle2->BuildWheelCar(viper);
		u -= 0.005f;

		dMatrix location0(manager->CalculateSplineMatrix(u));
		location0.m_posit += location0.m_right.Scale(3.0f);
		location0.m_posit = FindFloor(scene->GetNewton(), location0.m_posit + offset, 200.0f);
		location0.m_posit.m_y += 2.0f;
		SuperCarEntity* const vehicle0 = new SuperCarEntity(scene, manager, location0, "monsterTruck.ngd", 3.0f, monsterTruck);
		vehicle0->BuildWheelCar(monsterTruck);
		u -= 0.005f;
*/

		dMatrix location3(manager->CalculateSplineMatrix(u));
		location3.m_posit = FindFloor(scene->GetNewton(), location3.m_posit + offset, 200.0f);
		location3.m_posit.m_y += 1.0f;
		SuperCarEntity* const vehicle3 = new SuperCarEntity(scene, manager, location3, "viper.ngd", -3.0f, viper);
		vehicle3->BuildWheelCar(viper);
		u -= 0.005f;
	}

	dCustomVehicleController* const controller = &manager->GetLast()->GetInfo();
	SuperCarEntity* const vehicleEntity = (SuperCarEntity*)NewtonBodyGetUserData (controller->GetBody());

	// set this vehicle as the player
	manager->SetAsPlayer(vehicleEntity);
	//manager->SetDebugVehicle(vehicleEntity);

	// set the camera matrix, we only care the initial direction since it will be following the player vehicle
	dMatrix camMatrix (vehicleEntity->GetNextMatrix());
//	scene->SetCameraMouseLock (true);

	camMatrix.m_posit.m_x -= 5.0f;
//camMatrix = dYawMatrix (-0.5f * dPi) * camMatrix;
	scene->SetCameraMatrix(camMatrix, camMatrix.m_posit);

/*
	dMatrix location (camMatrix);
	location.m_posit.m_z += 4.0f;
	location.m_posit.m_x += 44.0f;

	int count = 5;
	dMatrix shapeOffsetMatrix (dGetIdentityMatrix());
	dVector size (3.0f, 0.125f, 3.0f, 0.0f);
	AddPrimitiveArray(scene, 50.0f, location.m_posit, size, count, count, 6.0f, _BOX_PRIMITIVE, defaultMaterial, shapeOffsetMatrix);

	size = dVector(0.75f, 0.35f, 0.75f, 0.0f);
	AddPrimitiveArray(scene, 50.0f, location.m_posit, size, count, count, 6.0f, _SPHERE_PRIMITIVE, defaultMaterial, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 50.0f, location.m_posit, size, count, count, 6.0f, _BOX_PRIMITIVE, defaultMaterial, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 50.0f, location.m_posit, size, count, count, 6.0f, _CAPSULE_PRIMITIVE, defaultMaterial, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 50.0f, location.m_posit, size, count, count, 6.0f, _CYLINDER_PRIMITIVE, defaultMaterial, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 50.0f, location.m_posit, size, count, count, 6.0f, _CHAMFER_CYLINDER_PRIMITIVE, defaultMaterial, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 50.0f, location.m_posit, size, count, count, 6.0f, _CONE_PRIMITIVE, defaultMaterial, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 50.0f, location.m_posit, size, count, count, 6.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterial, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 50.0f, location.m_posit, size, count, count, 6.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterial, shapeOffsetMatrix);

//	NewtonSerializeToFile (scene->GetNewton(), "C:/Users/Julio/Desktop/newton-dynamics/applications/media/xxxxx.bin");
*/
#endif
}

