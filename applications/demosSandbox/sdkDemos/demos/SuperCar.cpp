/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include <toolbox_stdafx.h>
#include "SkyBox.h"
#include "NewtonDemos.h"
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

// vehicle definition for a Muscle car
#define VIPER_TIRE_MASS						40.0f  
#define VIPER_ENGINE_MASS					100.0f
#define VIPER_MASS							(3380.0f * 0.454f)
#define VIPER_ENGINE_RADIO					0.125f
#define VIPER_TIRE_STEER_ANGLE				15.0f
#define VEHICLE_WEIGHT_DISTRIBUTION			0.55f

#define VIPER_CLUTCH_FRICTION_TORQUE		2000.0f
#define VIPER_IDLE_TORQUE					350.0f
#define VIPER_IDLE_TORQUE_RPM				800.0f

// torque in pound per foot
#define VIPER_PEAK_TORQUE					500.0f
#define VIPER_PEAK_TORQUE_RPM				3000.0f

#define VIPER_PEAK_HP						400.0f
#define VIPER_PEAK_HP_RPM					5200.0f

#define VIPER_REDLINE_TORQUE				100.0f
#define VIPER_REDLINE_TORQUE_RPM			6000.0f


//#define VIPER_TIRE_TOP_SPEED				164 mile / hours
#define VIPER_TIRE_TOP_SPEED_KMH			264.0f

#define VIPER_TIRE_LATERAL_STIFFNESS		(VIPER_MASS * DEMO_GRAVITY * 10.0f)
#define VIPER_TIRE_LONGITUDINAL_STIFFNESS	(VIPER_MASS * DEMO_GRAVITY *  2.0f)

#define VIPER_TIRE_ALIGNING_MOMENT_TRAIL	0.5f
#define VIPER_TIRE_SUSPENSION_SPRING		150.0f
#define VIPER_TIRE_SUSPENSION_DAMPER		10.0f
#define VIPER_TIRE_SUSPENSION_LENGTH		0.25f
#define VIPER_TIRE_BRAKE_TORQUE				3000.0f
#define VIPER_TIRE_PIVOT_OFSSET_TORQUE		-0.05f

#define VIPER_TIRE_GEAR_1					2.66f
#define VIPER_TIRE_GEAR_2					1.78f
#define VIPER_TIRE_GEAR_3 					1.30f
#define VIPER_TIRE_GEAR_4 					1.00f
#define VIPER_TIRE_GEAR_5 					0.74f
#define VIPER_TIRE_GEAR_6 					0.50f
#define VIPER_TIRE_REVERSE_GEAR				2.90f
#define VIPER_COM_Y_OFFSET					0.0f

// down force at 80 kmh
#define VIPER_DOWNFORCE_WEIGHT_FACTOR_0		1.0f
// down force at top Speed
#define VIPER_DOWNFORCE_WEIGHT_FACTOR_1		2.0f
#define VIPER_DOWNFORCE_WEIGHT_FACTOR_SPEED	80.0f 

#define VEHICLE_THIRD_PERSON_VIEW_HIGHT		2.0f
#define VEHICLE_THIRD_PERSON_VIEW_DIST		7.0f
//#define VEHICLE_THIRD_PERSON_VIEW_HIGHT		10.0f
//#define VEHICLE_THIRD_PERSON_VIEW_DIST		35.0f
#define VEHICLE_THIRD_PERSON_VIEW_FILTER	0.125f



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

	class TireAligmentTransform: public UserData
	{
		public: 
		TireAligmentTransform (const dMatrix& matrix)
			:UserData()
			,m_matrix( matrix)
		{
		}

		virtual void OnRender (dFloat timestep) const{};
		virtual void OnInterpolateMatrix (DemoEntityManager& world, dFloat param) const{};

		dMatrix m_matrix;
	};


	SuperCarEntity (DemoEntityManager* const scene, CustomVehicleControllerManager* const manager, const dMatrix& location, const char* const filename, dFloat distanceToPath)
		:DemoEntity (dGetIdentityMatrix(), NULL)
		,m_controller(NULL)
		,m_gearUpKey (false)
		,m_gearDownKey (false)
		,m_engineKeySwitch(false)
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
		chassisMatrix.m_front = dVector (1.0f, 0.0f, 0.0f, 0.0f);			// this is the vehicle direction of travel
		chassisMatrix.m_up	  = dVector (0.0f, 1.0f, 0.0f, 0.0f);			// this is the downward vehicle direction
		chassisMatrix.m_right = chassisMatrix.m_front * chassisMatrix.m_up;	// this is in the side vehicle direction (the plane of the wheels)
		chassisMatrix.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);

		// create a default vehicle 
		m_controller = manager->CreateVehicle (chassisCollision, chassisMatrix, VIPER_MASS, PhysicsApplyGravityForce, this);

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
		((CustomVehicleControllerManager*)m_controller->GetManager())->DestroyController(m_controller);
	}

	void SetGearMap(CustomVehicleController::EngineController* const engine)
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
			for (dList<CustomVehicleController::BodyPart*>::dListNode* node = m_controller->GetFirstBodyPart()->GetNext(); node; node = m_controller->GetNextBodyPart(node)) {
				CustomVehicleController::BodyPart* const part = node->GetInfo();
				DemoEntity* const entPart = (DemoEntity*) part->GetUserData();
				entPart->InterpolateMatrix (world, param);
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
		radius = dAbs (upDir % extremePoint);

		dVector widthDir (tireMatrix.UnrotateVector(dVector (0.0f, 0.0f, 1.0f, 0.0f)));
		NewtonCollisionSupportVertex (collision, &widthDir[0], &extremePoint[0]);
		width = widthDir % extremePoint;

		widthDir = widthDir.Scale (-1.0f);
		NewtonCollisionSupportVertex (collision, &widthDir[0], &extremePoint[0]);
		width += widthDir % extremePoint;

		// destroy the auxiliary collision
		NewtonDestroyCollision (collision);	
	}


	CustomVehicleController::BodyPartTire* AddTire (const char* const tireName, dFloat offset, dFloat width, dFloat radius, dFloat mass, dFloat suspensionLength, dFloat suspensionSpring, dFloat suspensionDamper, dFloat lateralStiffness, dFloat longitudinalStiffness, dFloat aligningMOmentTrail, dFloat pinDir) 
	{
		NewtonBody* const body = m_controller->GetBody();
		DemoEntity* const entity = (DemoEntity*) NewtonBodyGetUserData(body);
		DemoEntity* const tirePart = entity->Find (tireName);

		// the tire is located at position of the tire mesh relative to the chassis mesh
		dMatrix tireMatrix (tirePart->CalculateGlobalMatrix(entity));

		// add the offset location
		tireMatrix.m_posit.m_y += offset;

		// add and alignment matrix,to match visual mesh to physics collision shape
		dMatrix aligmentMatrix ((tireMatrix[0][2] > 0.0f) ? dGetIdentityMatrix() : dYawMatrix(3.141592f));

		TireAligmentTransform* const m_ligmentMatrix = new TireAligmentTransform(aligmentMatrix);
		tirePart->SetUserData(m_ligmentMatrix);

		// add the tire to the vehicle
		CustomVehicleController::BodyPartTire::Info tireInfo;
		tireInfo.m_location = tireMatrix.m_posit;
		tireInfo.m_mass = mass;
		tireInfo.m_radio = radius;
		tireInfo.m_width = width;
		tireInfo.m_dampingRatio = suspensionDamper;
		tireInfo.m_springStrength = suspensionSpring;
		tireInfo.m_suspesionlenght = suspensionLength;
		tireInfo.m_lateralStiffness = dAbs (lateralStiffness);
		tireInfo.m_longitudialStiffness = dAbs (longitudinalStiffness);
		tireInfo.m_aligningMomentTrail =  aligningMOmentTrail;
		tireInfo.m_userData = tirePart;

		return m_controller->AddTire (tireInfo);
	}


	void UpdateTireTransforms()
	{
		NewtonBody* const body = m_controller->GetBody();
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(NewtonBodyGetWorld(body));

		for (dList<CustomVehicleController::BodyPartTire>::dListNode* node = m_controller->GetFirstTire(); node; node = m_controller->GetNextTire(node)) {
			const CustomVehicleController::BodyPartTire* const part = &node->GetInfo();
			CustomVehicleController::BodyPart* const parent = part->GetParent();

			NewtonBody* const body = part->GetBody();
			NewtonBody* const parentBody = parent->GetBody();

			dMatrix matrix;
			dMatrix parentMatrix;
			NewtonBodyGetMatrix(body, &matrix[0][0]);
			NewtonBodyGetMatrix(parentBody, &parentMatrix[0][0]);

			DemoEntity* const tirePart = (DemoEntity*) part->GetUserData();
			TireAligmentTransform* const aligmentMatrix = (TireAligmentTransform*)tirePart->GetUserData();
			matrix = aligmentMatrix->m_matrix * matrix * parentMatrix.Inverse();

			dQuaternion rot (matrix);
			tirePart->SetMatrix(*scene, rot, matrix.m_posit);
		}
	}

	// this function is an example of how to make a high performance super car
	void BuildWheelCar (int differentialType, dFloat comHigh)
	{
		// step one: find the location of each tire, in the visual mesh and add them one by one to the vehicle controller 
		dFloat width;
		dFloat radius;

		// Muscle cars have the front engine, we need to shift the center of mass to the front to represent that
		m_controller->SetCenterOfGravity (dVector (0.0f, comHigh, 0.0f, 0.0f)); 

		// add front axle
		// a car may have different size front an rear tire, therefore we do this separate for front and rear tires
		CalculateTireDimensions ("fl_tire", width, radius);
		dVector offset (0.0f, 0.0f, 0.0f, 0.0f);
		CustomVehicleController::BodyPartTire* const leftFrontTire = AddTire ("fl_tire", VIPER_TIRE_PIVOT_OFSSET_TORQUE, width, radius, VIPER_TIRE_MASS, VIPER_TIRE_SUSPENSION_LENGTH, VIPER_TIRE_SUSPENSION_SPRING, VIPER_TIRE_SUSPENSION_DAMPER, VIPER_TIRE_LATERAL_STIFFNESS, VIPER_TIRE_LONGITUDINAL_STIFFNESS, VIPER_TIRE_ALIGNING_MOMENT_TRAIL, 1.0f);
		CustomVehicleController::BodyPartTire* const rightFrontTire = AddTire ("fr_tire", VIPER_TIRE_PIVOT_OFSSET_TORQUE, width, radius, VIPER_TIRE_MASS, VIPER_TIRE_SUSPENSION_LENGTH, VIPER_TIRE_SUSPENSION_SPRING, VIPER_TIRE_SUSPENSION_DAMPER, VIPER_TIRE_LATERAL_STIFFNESS, VIPER_TIRE_LONGITUDINAL_STIFFNESS, VIPER_TIRE_ALIGNING_MOMENT_TRAIL, -1.0f);

		// add rear axle
		// a car may have different size front an rear tire, therefore we do this separate for front and rear tires
		CalculateTireDimensions ("rl_tire", width, radius);
		CustomVehicleController::BodyPartTire* const leftRearTire = AddTire ("rl_tire", VIPER_TIRE_PIVOT_OFSSET_TORQUE, width, radius, VIPER_TIRE_MASS, VIPER_TIRE_SUSPENSION_LENGTH, VIPER_TIRE_SUSPENSION_SPRING, VIPER_TIRE_SUSPENSION_DAMPER, VIPER_TIRE_LATERAL_STIFFNESS, VIPER_TIRE_LONGITUDINAL_STIFFNESS, VIPER_TIRE_ALIGNING_MOMENT_TRAIL, 1.0f);
		CustomVehicleController::BodyPartTire* const rightRearTire = AddTire ("rr_tire", VIPER_TIRE_PIVOT_OFSSET_TORQUE, width, radius, VIPER_TIRE_MASS, VIPER_TIRE_SUSPENSION_LENGTH, VIPER_TIRE_SUSPENSION_SPRING, VIPER_TIRE_SUSPENSION_DAMPER, VIPER_TIRE_LATERAL_STIFFNESS, VIPER_TIRE_LONGITUDINAL_STIFFNESS, VIPER_TIRE_ALIGNING_MOMENT_TRAIL, -1.0f);

		//calculate the Ackerman parameters
		// add a steering Wheel component
		CustomVehicleController::SteeringController* const steering = new CustomVehicleController::SteeringController (m_controller, VIPER_TIRE_STEER_ANGLE * 3.141592f / 180.0f);
		steering->AddTire(leftFrontTire);
		steering->AddTire(rightFrontTire);
		steering->CalculateAkermanParameters (leftRearTire, rightRearTire, leftFrontTire, rightFrontTire);
		m_controller->SetSteering(steering);
		
		// add vehicle brakes
		CustomVehicleController::BrakeController* const brakes = new CustomVehicleController::BrakeController (m_controller, VIPER_TIRE_BRAKE_TORQUE);
		brakes->AddTire (leftFrontTire);
		brakes->AddTire (rightFrontTire);
		brakes->AddTire (leftRearTire);
		brakes->AddTire (rightRearTire);
		m_controller->SetBrakes(brakes);

		// add vehicle hand brakes
		CustomVehicleController::BrakeController* const handBrakes = new CustomVehicleController::BrakeController (m_controller, VIPER_TIRE_BRAKE_TORQUE);
		handBrakes->AddTire (leftRearTire);
		handBrakes->AddTire (rightRearTire);
		m_controller->SetHandBrakes(handBrakes);

		// add the engine, differential and transmission 
		CustomVehicleController::EngineController::Info engineInfo;
		engineInfo.m_mass = VIPER_ENGINE_MASS; 
		engineInfo.m_radio = VIPER_ENGINE_RADIO; 
		engineInfo.m_vehicleTopSpeed = VIPER_TIRE_TOP_SPEED_KMH;
		engineInfo.m_clutchFrictionTorque = VIPER_CLUTCH_FRICTION_TORQUE;
	
		engineInfo.m_peakTorque = VIPER_PEAK_TORQUE;
		engineInfo.m_rpmAtPeakTorque = VIPER_PEAK_TORQUE_RPM;
		engineInfo.m_peakHorsePower = VIPER_PEAK_HP;
		engineInfo.m_rpmAtPeakHorsePower = VIPER_PEAK_HP_RPM;
		engineInfo.m_redLineTorque = VIPER_REDLINE_TORQUE;
		engineInfo.m_rpmAtReadLineTorque = VIPER_REDLINE_TORQUE_RPM;
		engineInfo.m_idleTorque = VIPER_IDLE_TORQUE;
		engineInfo.m_rpmAtIdleTorque = VIPER_IDLE_TORQUE_RPM;

		engineInfo.m_gearsCount = 6;
		engineInfo.m_gearRatios[0] = VIPER_TIRE_GEAR_1;
		engineInfo.m_gearRatios[1] = VIPER_TIRE_GEAR_2;
		engineInfo.m_gearRatios[2] = VIPER_TIRE_GEAR_3;
		engineInfo.m_gearRatios[3] = VIPER_TIRE_GEAR_4;
		engineInfo.m_gearRatios[4] = VIPER_TIRE_GEAR_5;
		engineInfo.m_gearRatios[5] = VIPER_TIRE_GEAR_6;
		engineInfo.m_reverseGearRatio = VIPER_TIRE_REVERSE_GEAR;

		CustomVehicleController::EngineController::DifferentialAxel axel0; 
		CustomVehicleController::EngineController::DifferentialAxel axel1; 
		switch (differentialType)
		{
			case 0:
				axel0.m_leftTire = leftRearTire;
				axel0.m_rightTire = rightRearTire;
				break;
			case 1:
				axel0.m_leftTire = leftFrontTire;
				axel0.m_rightTire = rightFrontTire;
				break;

			default:
				axel0.m_leftTire = leftRearTire;
				axel0.m_rightTire = rightRearTire;
				axel1.m_leftTire = leftFrontTire;
				axel1.m_rightTire = rightFrontTire;
		}

		engineInfo.m_slipDifferentialOn = 1;
		engineInfo.m_userData = this;

		//CustomVehicleController::BodyPartEngine* const engine = new CustomVehicleController::BodyPartEngine(m_controller, engineInfo, axel0, axel1);
		//m_controller->AddEngineBodyPart (engine);
		CustomVehicleController::EngineController* const engineControl = new CustomVehicleController::EngineController (m_controller, engineInfo, axel0, axel1);

		// the the default transmission type
		engineControl->SetTransmissionMode(m_automaticTransmission.GetPushButtonState());
		engineControl->SetIgnition(true);

		m_controller->SetEngine(engineControl);

		// trace the engine curve
		//engineControl->PlotEngineCurve ();

		// set the gear look up table
		SetGearMap(engineControl);

		// set the vehicle weigh doistibution 
		m_controller->SetWeightDistribution (VEHICLE_WEIGHT_DISTRIBUTION);

		// set teh vehicle aerodynamics
		dFloat weightRatio0 = VIPER_DOWNFORCE_WEIGHT_FACTOR_0;
		dFloat weightRatio1 = VIPER_DOWNFORCE_WEIGHT_FACTOR_1;
		dFloat speedFactor = VIPER_DOWNFORCE_WEIGHT_FACTOR_SPEED / VIPER_TIRE_TOP_SPEED_KMH;
		m_controller->SetAerodynamicsDownforceCoefficient(DEMO_GRAVITY, weightRatio0, speedFactor, weightRatio1);

		// do not forget to call finalize after all components are added or after any change is made to the vehicle
		m_controller->Finalize();
	}

	void ApplyPlayerControl ()
	{
		NewtonBody* const body = m_controller->GetBody();
		NewtonWorld* const world = NewtonBodyGetWorld(body);
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);
		NewtonDemos* const mainWindow = scene->GetRootWindow();

		CustomVehicleController::EngineController* const engine = m_controller->GetEngine();
		CustomVehicleController::BrakeController* const brakes = m_controller->GetBrakes();
		CustomVehicleController::BrakeController* const handBrakes = m_controller->GetHandBrakes();
		CustomVehicleController::SteeringController* const steering = m_controller->GetSteering();

		// get the throttler input
		dFloat joyPosX;
		dFloat joyPosY;
		int joyButtons;

		int gear = engine->GetGear();
		int engineIgnitionKey = 0;
		int automaticTransmission = engine->GetTransmissionMode();
		dFloat cluthPedal = 1.0f;
		dFloat steeringVal = 0.0f;
		dFloat reverseGasPedal = 0.0f;
		dFloat forwardGasPedal = 0.0f;
		dFloat handBrakePedal = 0.0f;
		
		bool hasJopytick = mainWindow->GetJoytickPosition (joyPosX, joyPosY, joyButtons);
		if (hasJopytick) {
/*
			// apply a cubic attenuation to the joystick inputs
			joyPosX = joyPosX * joyPosX * joyPosX;
			joyPosY = joyPosY * joyPosY * joyPosY;

			steeringVal = joyPosX;
			brakePedal = (joyPosY < 0.0f) ? -joyPosY: 0.0f;
			engineGasPedal = (joyPosY >= 0.0f) ? joyPosY: 0.0f;

			gear += int (m_gearUpKey.UpdateTriggerJoystick(mainWindow, joyButtons & 2)) - int (m_gearDownKey.UpdateTriggerJoystick(mainWindow, joyButtons & 4));
			handBrakePedal = (joyButtons & 1) ? 1.0f : 0.0f;
*/
		} else {

			engineIgnitionKey = m_engineKeySwitch.UpdatePushButton(mainWindow, 'I');
			automaticTransmission = m_automaticTransmission.UpdatePushButton (mainWindow, 0x0d);
			steeringVal = (dFloat(mainWindow->GetKeyState('A')) - dFloat(mainWindow->GetKeyState('D')));
			gear += int(m_gearUpKey.UpdateTriggerButton(mainWindow, '.')) - int(m_gearDownKey.UpdateTriggerButton(mainWindow, ','));

			if (mainWindow->GetKeyState ('W')) {
				forwardGasPedal = 1.0f;
			}

			if (mainWindow->GetKeyState('S')) {
				reverseGasPedal = 1.0f;
			}

			if (mainWindow->GetKeyState(' ')) {
				handBrakePedal = 1.0f;
			}


			if (mainWindow->GetKeyState ('K')) {
 				cluthPedal = 0.0f;
			}
		}

#if 0
	#if 1
		static FILE* file = fopen ("log.bin", "wb");                                         
		if (file) {
			fwrite (&engineIgnitionKey, sizeof (int), 1, file);
			fwrite (&automaticTransmission, sizeof (int), 1, file);
			fwrite (&gear, sizeof (int), 1, file);
			fwrite (&steeringVal, sizeof (dFloat), 1, file);
			fwrite (&cluthPedal, sizeof (dFloat), 1, file);
			fwrite (&forwardGasPedal, sizeof (dFloat), 1, file);
			fwrite (&reverseGasPedal, sizeof (dFloat), 1, file);
			fwrite (&handBrakePedal, sizeof (dFloat), 1, file);
			fflush(file);
		}
	#else 
		static FILE* file = fopen ("log.bin", "rb");
		if (file) {		
			fread (&engineIgnitionKey, sizeof (int), 1, file);
		 	fread (&automaticTransmission, sizeof (int), 1, file);
			fread (&gear, sizeof (int), 1, file);
			fread (&steeringVal, sizeof (dFloat), 1, file);
			fread (&cluthPedal, sizeof (dFloat), 1, file);
			fread (&forwardGasPedal, sizeof (dFloat), 1, file);
			fread (&reverseGasPedal, sizeof (dFloat), 1, file);
			fread (&handBrakePedal, sizeof (dFloat), 1, file);
		}
	#endif
#endif


//forwardGasPedal *= 0.75f;	
		steering->SetParam(steeringVal);
		switch (m_drivingState)
		{
			case m_engineOff:
			{
				if (engineIgnitionKey) {
					m_drivingState = m_engineIdle;
					engine->SetIgnition (true);
					handBrakes->SetParam(0.0f);
					engine->SetGear (engine->GetNeutralGear());
				} else {
					engine->SetIgnition (false);
					engine->SetGear(engine->GetFirstGear());
					handBrakes->SetParam(1.0f);
				}
				break;
			}

			case m_engineIdle:
			{
				brakes->SetParam(0.0f);
				handBrakes->SetParam(handBrakePedal);
				if (!engineIgnitionKey) {
					m_drivingState = m_engineOff;
				} else {
					if (forwardGasPedal) {
						m_drivingState = m_preDriveForward;
					} else if (reverseGasPedal) {
						m_drivingState = m_preDriveReverse;
					}
				}
				break;
			}

			case m_engineStop:
			{
				if (forwardGasPedal || reverseGasPedal) {
					brakes->SetParam(1.0f);
				} else {
					m_drivingState = m_engineIdle;
				}
				break;
			}

			case m_preDriveForward:
			{
				if (engine->GetSpeed() < -5.0f) {
					brakes->SetParam(0.5f);
					engine->SetClutchParam(0.0f);
					engine->SetGear(engine->GetNeutralGear());
				} else {
					m_drivingState = m_driveForward;
					engine->SetGear(engine->GetFirstGear());
				}
				break;
			}	

			case m_driveForward:
			{
				engine->SetParam(forwardGasPedal);
				engine->SetClutchParam(cluthPedal);
				handBrakes->SetParam(handBrakePedal);
				if (reverseGasPedal) {
					brakes->SetParam(reverseGasPedal * 0.25f);
					if (engine->GetSpeed() < 5.0f) {
						engine->SetGear(engine->GetNeutralGear());
						m_drivingState = m_engineStop;
					}
				} else {
					brakes->SetParam(0.0f);
				}

				if (!engineIgnitionKey) {
					m_drivingState = m_engineStop;
				}

				break;
			}

			case m_preDriveReverse:
			{
				if (engine->GetSpeed() > 5.0f) {
					brakes->SetParam(0.5f);
					engine->SetClutchParam(0.0f);
					engine->SetGear(engine->GetNeutralGear());
				} else {
					m_drivingState = m_driveReverse;
					engine->SetGear(engine->GetReverseGear());
				}
				break;
			}

			case m_driveReverse:
			{
				engine->SetParam(reverseGasPedal);
				engine->SetClutchParam(cluthPedal);
				handBrakes->SetParam(handBrakePedal);
				if (forwardGasPedal) {
					brakes->SetParam(forwardGasPedal * 0.25f);
					if (engine->GetSpeed() > -5.0f) {
						engine->SetGear(engine->GetNeutralGear());
						m_drivingState = m_engineStop;
					}
				} else {
					brakes->SetParam(0.0f);
				}

				if (!engineIgnitionKey) {
					m_drivingState = m_engineStop;
				}
				break;
			}
		}
	}

	// based on the work of Craig Reynolds http://www.red3d.com/cwr/steer/
	dFloat CalculateNPCControlSteerinValue (dFloat distanceAhead, dFloat pathWidth, DemoEntity* const pathEntity)
	{
		const CustomVehicleController::BodyPart& chassis = *m_controller->GetChassis();
		//CustomVehicleControllerComponentSteering* const steering = m_controller->GetSteering();

		CustomVehicleController::SteeringController* const steering = m_controller->GetSteering();

		dMatrix matrix;
		dVector veloc(0.0f);
		NewtonBody* const body = chassis.GetBody();
		NewtonBodyGetVelocity(body, &veloc[0]);
		NewtonBodyGetMatrix(body, &matrix[0][0]);
		dVector lookAheadPoint (veloc.Scale (distanceAhead / dSqrt (veloc % veloc)));

		// find the closet point to the past on the spline
		dMatrix vehicleMatrix (m_controller->GetLocalFrame() * matrix);
		dMatrix pathMatrix (pathEntity->GetMeshMatrix() * pathEntity->GetCurrentMatrix());

		dVector p0 (vehicleMatrix.m_posit + lookAheadPoint);
		p0.m_y = 0.0f;
		
		dBigVector q; 
		DemoBezierCurve* const path = (DemoBezierCurve*) pathEntity->GetMesh();
		dFloat64 u = path->m_curve.FindClosestKnot (q, pathMatrix.UntransformVector(p0), 4);
		dVector p1 (pathMatrix.TransformVector(dVector (q.m_x, q.m_y, q.m_z, q.m_w)));
		p1.m_y = 0.0f;
		dVector dist (p1 - p0);
		dFloat angle = 0.0f;
//		dFloat maxAngle = steering->GetMaxSteeringAngle ();
		dFloat maxAngle = 20.0f * 3.1314f / 180.0f;
		if ((dist % dist) < (pathWidth * pathWidth)) {
			dBigVector averageTangent (0.0f, 0.0f, 0.0f, 0.0f);
			for(int i = 0; i < 4; i ++) {
				dBigVector tangent (path->m_curve.CurveDerivative (u));
				tangent = tangent.Scale (1.0f / dSqrt (tangent % tangent));
				averageTangent += tangent;
				q += tangent.Scale (5.0f);
				u = path->m_curve.FindClosestKnot (q, q, 4);
			}
			averageTangent = averageTangent.Scale (1.0f / dSqrt (averageTangent % averageTangent));
			dVector heading (pathMatrix.RotateVector(dVector (averageTangent.m_x, averageTangent.m_y, averageTangent.m_z, averageTangent.m_w)));
			heading.m_y = 0.0;
			heading = vehicleMatrix.UnrotateVector(heading);
			angle = dClamp (dAtan2 (heading.m_z, heading.m_x), -maxAngle, maxAngle);
		} else {

			// find a point in the past at some distance ahead
			for(int i = 0; i < 5; i ++) {
				dBigVector tangent (path->m_curve.CurveDerivative (u));
				q += tangent.Scale (5.0f / dSqrt (tangent % tangent));
				path->m_curve.FindClosestKnot (q, q, 4);
			}

			m_debugTargetHeading = pathMatrix.TransformVector(dVector (q.m_x, q.m_y, q.m_z, q.m_w));
			dVector localDir (vehicleMatrix.UntransformVector(m_debugTargetHeading));
			angle = dClamp (dAtan2 (localDir.m_z, localDir.m_x), -maxAngle, maxAngle);
		}
		dFloat param = steering->GetParam();
		return param + (-angle / maxAngle - param) * 0.25f;
	}

	void ApplyNPCControl (dFloat timestep, DemoEntity* const pathEntity)
	{
		//drive the vehicle by trying to follow the spline path as close as possible 
		//NewtonBody* const body = m_controller->GetBody();
		//NewtonWorld* const world = NewtonBodyGetWorld(body);
		//DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);
		//NewtonDemos* const mainWindow = scene->GetRootWindow();
		 
		CustomVehicleController::EngineController* const engine = m_controller->GetEngine();
		CustomVehicleController::SteeringController* const steering = m_controller->GetSteering();
		//CustomVehicleController::ClutchController* const clutch = m_controller->GetClutch();
		//CustomVehicleController::BrakeController* const brakes = m_controller->GetBrakes();
		//CustomVehicleController::BrakeController* const handBrakes = m_controller->GetHandBrakes();
		
		
//		if (!engine->GetKey()) 
		{
			// start engine
//			m_engineOldKeyState = engine->GetKey();
//			engine->SetKey (true);

			// AI car are manual gears
			engine->SetTransmissionMode (1);
			//clutch->SetParam(1.0f);

			// engage first Gear 
//			engine->SetGear (CustomVehicleControllerComponentEngine::dGearBox::m_firstGear + 3);
		}
		
		
		//const CustomVehicleControllerBodyStateChassis& chassis = m_controller->GetChassisState ();
		//dMatrix matrix (chassis.GetLocalMatrix() * chassis.GetMatrix());
		//dVector veloc (chassis.GetVelocity());
		//veloc.m_y = 0.0f;

		dFloat speed = dAbs(engine->GetSpeed());

		if (speed < 0.1f) {
			// if the vehicle is no moving start the motion
			engine->SetParam (0.75f);
			//steering->SetParam (-0.5f);
			return;
		}

		dFloat steeringParam = CalculateNPCControlSteerinValue (2.0f, 2.0f, pathEntity);
		steering->SetParam (steeringParam);
	}


	void Debug (DemoEntity* const m_aiPath) const 
	{
		dMatrix matrix;
		dVector com(0.0f);
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;
		dFloat mass;

		const CustomVehicleController::BodyPart* const chassis = m_controller->GetChassis ();
		NewtonBody* const chassisBody = chassis->GetBody();
		
		NewtonBodyGetCentreOfMass(chassisBody, &com[0]);
		NewtonBodyGetMassMatrix(chassisBody, &mass, &Ixx, &Iyy, &Izz);
		NewtonBodyGetMatrix(chassisBody, &matrix[0][0]);
		matrix.m_posit = matrix.TransformVector(com);
		matrix = m_controller->GetLocalFrame() * matrix;

		dFloat scale = -4.0f / (mass * DEMO_GRAVITY);
		dVector p0 (matrix.m_posit);

		glDisable (GL_LIGHTING);
		glDisable(GL_TEXTURE_2D);

		glLineWidth(3.0f);
		glBegin(GL_LINES);

		// draw vehicle weight at the center of mass
		dFloat lenght = scale * mass * DEMO_GRAVITY;
		glColor3f(0.0f, 0.0f, 1.0f);
		glVertex3f (p0.m_x, p0.m_y, p0.m_z);
		glVertex3f (p0.m_x, p0.m_y - lenght, p0.m_z);

		// draw vehicle front dir
		glColor3f(1.0f, 1.0f, 1.0f);
		dVector r0 (p0 + matrix[1].Scale (1.0f));
		dVector r1 (r0 + matrix[0].Scale (2.0f));
		glVertex3f (r0.m_x, r0.m_y, r0.m_z);
		glVertex3f (r1.m_x, r1.m_y, r1.m_z);

		// draw the velocity vector, a little higher so that is not hidden by the vehicle mesh 
		dVector veloc(0.0f);
		NewtonBodyGetVelocity(chassisBody, &veloc[0]);
		dVector q0 (p0 + matrix[1].Scale (1.0f));
		dVector q1 (q0 + veloc.Scale (0.25f));
		glColor3f(1.0f, 1.0f, 0.0f);
		glVertex3f (q0.m_x, q0.m_y, q0.m_z);
		glVertex3f (q1.m_x, q1.m_y, q1.m_z);

//int xxx = 0;
		for (dList<CustomVehicleController::BodyPartTire>::dListNode* node = m_controller->GetFirstTire(); node; node = m_controller->GetNextTire(node)) {
			const CustomVehicleController::BodyPartTire* const tire = &node->GetInfo();
			NewtonBody* const tireBody = tire->GetBody();

			dMatrix tireMatrix;
			NewtonBodyGetMatrix(tireBody, &tireMatrix[0][0]);

			dFloat sign = dSign ((tireMatrix.m_posit - matrix.m_posit) % matrix.m_right);
			tireMatrix.m_posit += matrix.m_right.Scale (sign * 0.25f);

			// draw the tire load 
			dVector normalLoad (m_controller->GetTireNormalForce(tire));
			dVector p0 (tireMatrix.m_posit);
			dVector p1 (p0 + normalLoad.Scale (scale));

			glColor3f (0.0f, 0.0f, 1.0f);
			glVertex3f (p0.m_x, p0.m_y, p0.m_z);
			glVertex3f (p1.m_x, p1.m_y, p1.m_z);

			// show tire lateral force
			dVector lateralForce (m_controller->GetTireLateralForce(tire));
			dVector p2 (p0 - lateralForce.Scale (scale));
			glColor3f(1.0f, 0.0f, 0.0f);
			glVertex3f (p0.m_x, p0.m_y, p0.m_z);
			glVertex3f (p2.m_x, p2.m_y, p2.m_z);

			// show tire longitudinal force
			dVector longitudinalForce (m_controller->GetTireLongitudinalForce(tire));
			dVector p3 (p0 - longitudinalForce.Scale (scale));
			glColor3f(0.0f, 1.0f, 0.0f);
			glVertex3f (p0.m_x, p0.m_y, p0.m_z);
			glVertex3f (p3.m_x, p3.m_y, p3.m_z);
//if (!xxx)
//dTrace(("%f ", tire.GetAligningTorque()));
//xxx ++;
		}
		
		glEnd();
		glLineWidth(1.0f);

/*
		// render AI information
		dMatrix pathMatrix (m_aiPath->GetMeshMatrix() * m_aiPath->GetCurrentMatrix());
		dBigVector q; 
		DemoBezierCurve* const path = (DemoBezierCurve*) m_aiPath->GetMesh();
		path->m_curve.FindClosestKnot (q, pathMatrix.UntransformVector(p0), 4);
		dVector p1 (pathMatrix.TransformVector(dVector (q.m_x, q.m_y, q.m_z, q.m_w)));

		glBegin(GL_LINES);
		glColor3f (1.0f, 0.0f, 1.0f);
		glVertex3f (p0.m_x, p0.m_y, p0.m_z);
		glVertex3f (p1.m_x, p1.m_y, p1.m_z);
		glEnd();

		glBegin(GL_LINES);
		glColor3f (1.0f, 0.0f, 1.0f);
		glVertex3f (p0.m_x, p0.m_y + 1.0f, p0.m_z);
		glVertex3f (m_debugTargetHeading.m_x, m_debugTargetHeading.m_y + 1.0f, m_debugTargetHeading.m_z);
		glEnd();
*/
	}


	CustomVehicleController* m_controller;
	DemoEntityManager::ButtonKey m_gearUpKey;
	DemoEntityManager::ButtonKey m_gearDownKey;
	DemoEntityManager::ButtonKey m_engineKeySwitch;
	DemoEntityManager::ButtonKey m_automaticTransmission;
	int m_gearMap[10];
	bool m_engineRPMOn;
	dFloat m_distanceToPath;
	dVector m_debugTargetHeading; 
	DrivingState m_drivingState;
};


class SuperCarVehicleControllerManager: public CustomVehicleControllerManager
{
	public:

	SuperCarVehicleControllerManager (NewtonWorld* const world, int materialsCount, int* const materialList)
		:CustomVehicleControllerManager (world, materialsCount, materialList)
		,m_externalView(true)
		,m_player (NULL) 
		,m_debugVehicle (NULL) 
		,m_drawShematic(false)
		,m_helpKey (true)
	{
		// hook a callback for 2d help display
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);
		scene->Set2DDisplayRenderFunction (RenderVehicleHud, this);

		// load 2d display assets
		m_gears = LoadTexture ("gears_font.tga");
		m_tachometer = LoadTexture ("rpm_dial.tga");
		m_odometer = LoadTexture ("kmh_dial.tga");
		m_redNeedle = LoadTexture ("needle_red.tga");
		m_greenNeedle = LoadTexture ("needle_green.tga");
	}

	~SuperCarVehicleControllerManager ()
	{
		ReleaseTexture (m_gears);
		ReleaseTexture (m_tachometer);
		ReleaseTexture (m_odometer);
		ReleaseTexture (m_redNeedle);
		ReleaseTexture (m_greenNeedle);
	}
	

	static void RenderVehicleHud (DemoEntityManager* const scene, void* const context, int lineNumber)
	{
		SuperCarVehicleControllerManager* const me = (SuperCarVehicleControllerManager*) context;
		me->RenderVehicleHud (scene, lineNumber);
	}

	void DrawGage(GLuint gage, GLuint needle, dFloat param, dFloat origin_x, dFloat origin_y, dFloat size) const
	{
		size *= 0.5f;
		dMatrix origin (dGetIdentityMatrix());
		origin.m_posit = dVector(origin_x, origin_y, 0.0f, 1.0f);

		// render dial
		glPushMatrix();
		glMultMatrix (&origin[0][0]);
		glBindTexture(GL_TEXTURE_2D, gage);
		glBegin(GL_QUADS);
		glTexCoord2f(0.0f, 1.0f); glVertex3f(-size,  size, 0.0f);
		glTexCoord2f(0.0f, 0.0f); glVertex3f(-size, -size, 0.0f);
		glTexCoord2f(1.0f, 0.0f); glVertex3f( size, -size, 0.0f);
		glTexCoord2f(1.0f, 1.0f); glVertex3f( size,  size, 0.0f);
		glEnd();

		// render needle
		const dFloat minAngle = 180.0f * 3.141592f / 180.0f;
		const dFloat maxAngle = -90.0f * 3.141592f / 180.0f;
		dFloat angle = minAngle + (maxAngle - minAngle) * param;
		dMatrix needleMatrix (dRollMatrix (angle));

		dFloat x = size * 0.7f;
		dFloat y = size * 0.7f;

		glPushMatrix();
		glMultMatrix (&needleMatrix[0][0]);
		glBindTexture(GL_TEXTURE_2D, needle);
		glBegin(GL_QUADS);
		glTexCoord2f(0.0f, 1.0f); glVertex3f(-x,  y, 0.0f);
		glTexCoord2f(0.0f, 0.0f); glVertex3f(-x, -y, 0.0f);
		glTexCoord2f(1.0f, 0.0f); glVertex3f( x, -y, 0.0f);
		glTexCoord2f(1.0f, 1.0f); glVertex3f( x,  y, 0.0f);
		glEnd();

		glPopMatrix();
		glPopMatrix();
	}

	void DrawGear(dFloat param, dFloat origin_x, dFloat origin_y, int gear, dFloat size) const
	{
		dMatrix origin (dGetIdentityMatrix());
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
		glTexCoord2f(u0, 1.0f); glVertex3f(-x1,  y1, 0.0f);
		glTexCoord2f(u0, 0.0f); glVertex3f(-x1, -y1, 0.0f);
		glTexCoord2f(u1, 0.0f); glVertex3f( x1, -y1, 0.0f);
		glTexCoord2f(u1, 1.0f); glVertex3f( x1,  y1, 0.0f);
		glEnd();

		glPopMatrix();
	}

	void DrawHelp(DemoEntityManager* const scene, int lineNumber) const
	{
		if (m_helpKey.GetPushButtonState()) {
			dVector color(1.0f, 1.0f, 0.0f, 0.0f);
			lineNumber = scene->Print (color, 10, lineNumber + 20, "Vehicle driving keyboard control:   Joystick control");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "engine switch       : 'I'           start engine");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "accelerator         : 'W'           stick forward");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "brakes              : 'S'           stick back");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "turn left           : 'A'           stick left");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "turn right          : 'D'           stick right");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "engage clutch       : 'K'           button 5");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "gear up             : '>'           button 2");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "gear down           : '<'           button 3");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "manual transmission : enter         button 4");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "hand brakes         : space         button 1");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "hide help           : 'H'");
		}
	}


	void RenderVehicleSchematic (DemoEntityManager* const scene) const
	{
		if (m_debugVehicle) {
			glDisable(GL_LIGHTING);
			glDisable(GL_TEXTURE_2D);
			glDisable(GL_DEPTH_TEST);
			glDisable (GL_BLEND);

			dFloat scale = 100.0f;
			dFloat width = scene->GetWidth();
			dFloat height = scene->GetHeight();

			dMatrix origin (dGetIdentityMatrix());
			origin.m_posit = dVector(width - 300, height - 200, 0.0f, 1.0f);

			glPushMatrix();
			glMultMatrix (&origin[0][0]);
			DrawSchematic (m_debugVehicle->m_controller, scale);
			glPopMatrix();

			glLineWidth(1.0f);
			glEnable(GL_TEXTURE_2D);
		}
	}


	void DrawSchematicCallback (const CustomVehicleController* const controller, const char* const partName, dFloat value, int pointCount, const dVector* const lines) const
	{
		if (!strcmp (partName, "chassis")) {
			glLineWidth(3.0f);
			glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
			glBegin(GL_LINES);
			dVector p0 (lines[pointCount - 1]);
			for (int i = 0; i < pointCount; i ++) {
				dVector p1 (lines[i]);
				glVertex3f(p0.m_x, p0.m_y, p0.m_z);
				glVertex3f(p1.m_x, p1.m_y, p1.m_z);
				p0 = p1;
			}
			glEnd();
		}

		if (!strcmp (partName, "tire")) {
			glLineWidth(2.0f);
			glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
			glBegin(GL_LINES);
			dVector p0 (lines[pointCount - 1]);
			for (int i = 0; i < pointCount; i ++) {
				dVector p1 (lines[i]);
				glVertex3f(p0.m_x, p0.m_y, p0.m_z);
				glVertex3f(p1.m_x, p1.m_y, p1.m_z);
				p0 = p1;
			}
			glEnd();
		}

		if (!strcmp (partName, "velocity")) {
			glLineWidth(2.0f);
			glColor4f(1.0f, 1.0f, 0.0f, 1.0f);
			glBegin(GL_LINES);
			dVector p0 (lines[0]);
			dVector p1 (lines[1]);
			glVertex3f(p0.m_x, p0.m_y, p0.m_z);
			glVertex3f(p1.m_x, p1.m_y, p1.m_z);
			glEnd();
		}

		if (!strcmp(partName, "tireVelocity")) {
			glLineWidth(2.0f);
			glColor4f(1.0f, 1.0f, 0.0f, 1.0f);
			glBegin(GL_LINES);
			dVector p0(lines[0]);
			dVector p1(lines[1]);
			glVertex3f(p0.m_x, p0.m_y, p0.m_z);
			glVertex3f(p1.m_x, p1.m_y, p1.m_z);
			glEnd();
		}

		if (!strcmp(partName, "omega")) {
			glLineWidth(2.0f);
			glColor4f(0.0f, 1.0f, 1.0f, 1.0f);
			glBegin(GL_LINES);
			dVector p0(lines[0]);
			dVector p1(lines[1]);
			glVertex3f(p0.m_x, p0.m_y, p0.m_z);
			glVertex3f(p1.m_x, p1.m_y, p1.m_z);
			glEnd();
		}

		if (!strcmp (partName, "lateralForce")) {
			glLineWidth(2.0f);
			glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
			glBegin(GL_LINES);
			dVector p0 (lines[0]);
			dVector p1 (lines[1]);
			glVertex3f(p0.m_x, p0.m_y, p0.m_z);
			glVertex3f(p1.m_x, p1.m_y, p1.m_z);
			glEnd();
		}

		if (!strcmp (partName, "longitudinalForce")) {
			glLineWidth(2.0f);
			glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
			glBegin(GL_LINES);
			dVector p0 (lines[0]);
			dVector p1 (lines[1]);
			glVertex3f(p0.m_x, p0.m_y, p0.m_z);
			glVertex3f(p1.m_x, p1.m_y, p1.m_z);
			glEnd();
		}
	}

	void RenderVehicleHud (DemoEntityManager* const scene, int lineNumber) const
	{
		// draw the player physics model
		if (m_drawShematic) {
			RenderVehicleSchematic (scene);
		}
		m_drawShematic = false;

		// set to transparent color
		glEnable (GL_BLEND);
		glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		if (m_debugVehicle) {
			CustomVehicleController::EngineController* const engine = m_debugVehicle->m_controller->GetEngine();
			if (engine) {
				glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
				dFloat gageSize = 200.0f;

				dFloat y = gageSize / 2.0f + 60.0f;

				// draw the tachometer
				dFloat x = gageSize / 2 + 40.0f;
				dFloat rpm = engine->GetRPM () / engine->GetRedLineRPM();
				DrawGage(m_tachometer, m_redNeedle, rpm, x, y, gageSize);

				// draw the odometer
				x += gageSize;
				//dFloat speed = dAbs(engine->GetSpeed()) * 3.6f / 340.0f;
				dFloat speed = dAbs(engine->GetSpeed())  / engine->GetTopSpeed();
				int gear = engine->GetGear();
				DrawGage(m_odometer, m_greenNeedle, speed, x, y, gageSize);
				DrawGear(speed, x, y, m_debugVehicle->m_gearMap[gear], gageSize);
			}
		}

		//print controllers help 
		DrawHelp(scene, lineNumber);

		// restore color and blend mode
		glDisable (GL_BLEND);
	}

	void SetAsPlayer (SuperCarEntity* const player)
	{
		CustomVehicleController::EngineController* const engine = player->m_controller->GetEngine();
		engine->SetIgnition(false);
		m_player = player;
	}

	void SetDebugVehicle (SuperCarEntity* const player)
	{
		m_debugVehicle = player;
	}

	virtual void PreUpdate (dFloat timestep)
	{
		// apply the vehicle controls, and all simulation time effect
		NewtonWorld* const world = GetWorld(); 
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);

		// set the help key
		m_helpKey.UpdatePushButton (scene->GetRootWindow(), 'H');
		
		for (dListNode* ptr = GetFirst(); ptr; ptr = ptr->GetNext()) {
			CustomVehicleController* const controller = &ptr->GetInfo();
		
			NewtonBody* const body = controller->GetBody();
			SuperCarEntity* const vehicleEntity = (SuperCarEntity*) NewtonBodyGetUserData(body);

			if (vehicleEntity == m_player) {
				// do player control
				vehicleEntity->ApplyPlayerControl ();
			} else {
				// do no player control
				vehicleEntity->ApplyNPCControl (timestep, m_raceTrackPath);
			}
		}

		// do the base class post update
		CustomVehicleControllerManager::PreUpdate(timestep);

	}

	virtual void PostUpdate (dFloat timestep)
	{
		// do the base class post update
		CustomVehicleControllerManager::PostUpdate(timestep);
		
		// update the visual transformation matrices for all vehicle tires
		for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
			CustomVehicleController* const controller = &node->GetInfo();
			SuperCarEntity* const vehicleEntity = (SuperCarEntity*)NewtonBodyGetUserData (controller->GetBody());
			vehicleEntity->UpdateTireTransforms();
		}

		if (m_player) {
			UpdateCamera (m_player, timestep);
		} else {
			CustomVehicleController* const controller = &GetLast()->GetInfo();
			SuperCarEntity* const vehicleEntity = (SuperCarEntity*)NewtonBodyGetUserData (controller->GetBody());
			UpdateCamera (vehicleEntity, timestep);
		}
	}


	void UpdateCamera (SuperCarEntity* const player, dFloat timestep)
	{
//return;		
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(GetWorld());
		DemoCamera* const camera = scene->GetCamera();
		dMatrix camMatrix (camera->GetNextMatrix ());
		dMatrix playerMatrix (player->GetNextMatrix());

		dVector frontDir (camMatrix[0]);
		dVector camOrigin(0.0f); 
		if (m_externalView) {
			camOrigin = playerMatrix.m_posit + dVector(0.0f, VEHICLE_THIRD_PERSON_VIEW_HIGHT, 0.0f, 0.0f);
			camOrigin -= frontDir.Scale(VEHICLE_THIRD_PERSON_VIEW_DIST);
		} else {
			dAssert (0);
			//            camMatrix = camMatrix * playerMatrix;
			//            camOrigin = playerMatrix.TransformVector(dVector(-0.8f, ARTICULATED_VEHICLE_CAMERA_EYEPOINT, 0.0f, 0.0f));
		}

		camera->SetNextMatrix (*scene, camMatrix, camOrigin);
	}


	dMatrix CalculateSplineMatrix (dFloat64 param) const
	{
		DemoBezierCurve* const path = (DemoBezierCurve*) m_raceTrackPath->GetMesh();

		dBigVector origin (path->m_curve.CurvePoint(param)); 
		dBigVector tangent (path->m_curve.CurveDerivative(param, 1)); 
		tangent = tangent.Scale (1.0 / sqrt (tangent % tangent));
		dBigVector p1 (origin + tangent);

		dBigVector dir; 
		path->m_curve.FindClosestKnot (dir, p1, 4);

		dir -= origin;
		dir = dir.Scale (1.0 / sqrt (dir % dir));
		dMatrix matrix (dGetIdentityMatrix());

		dMatrix pathMatrix (m_raceTrackPath->GetMeshMatrix() * m_raceTrackPath->GetCurrentMatrix());
		matrix.m_front = pathMatrix.RotateVector (dVector (dir.m_x, dir.m_y, dir.m_z, 0.0f));
		matrix.m_right = matrix.m_front * matrix.m_up;
		matrix.m_right.m_w = 0.0f;
		matrix.m_posit = pathMatrix.TransformVector(dVector (origin.m_x, origin.m_y, origin.m_z, 1.0f));
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
		int segmentCount = slineLength / 5.0f;

		dFloat64 step = slineLength / segmentCount;

		dFloat streetWidth = 8.0f;

		dFloat64 acc = 0.0f;
		dFloat64 dist = 0.0;
		dBigVector p0 (path->m_curve.CurvePoint(0.0f));
		for (int i = 1; i < 1000; i ++) {
			dFloat64 t = dFloat64 (i) / 1000.0f;
			dBigVector p1 (path->m_curve.CurvePoint(t));
			dBigVector dp (p1 - p0);
			dist += sqrt (dp % dp);
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

	// use this to display debug information about vehicle 
	void Debug () const
	{
		m_drawShematic = true;
		for (dListNode* ptr = GetFirst(); ptr; ptr = ptr->GetNext()) {
			CustomVehicleController* const controller = &ptr->GetInfo();
			SuperCarEntity* const vehicleEntity = (SuperCarEntity*)NewtonBodyGetUserData (controller->GetBody());
			vehicleEntity->Debug(m_raceTrackPath);
		}
	}


	bool m_externalView;
	SuperCarEntity* m_player;
	SuperCarEntity* m_debugVehicle;
	DemoEntity* m_raceTrackPath;

	GLuint m_gears;
	GLuint m_redNeedle;
	GLuint m_greenNeedle;
	GLuint m_odometer;
	GLuint m_tachometer;
	GLuint m_needle;
	int m_soundsCount;
	mutable bool m_drawShematic;
	DemoEntityManager::ButtonKey m_helpKey;
	void* m_engineSounds[16];
};



// *************************************************************************************************
// 
//  create a simple racing game with a simple controlled by a Newton AI engine and newton physics
//
// *************************************************************************************************
void SuperCar (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	//CreateLevelMesh (scene, "flatPlane1.ngd", 0);
	//CreateLevelMesh (scene, "flatPlane.ngd", 1);
	//CreateLevelMesh (scene, "raceTrack2.ngd", 0);
	//CreateLevelMesh (scene, "raceTrack2.ngd", 1);
	CreateLevelMesh (scene, "raceTrack1.ngd", 0);
	//CreateLevelMesh (scene, "raceTrack1.ngd", 1);
	//CreateHeightFieldTerrain (scene, 10, 8.0f, 1.5f, 0.2f, 200.0f, -50.0f);
	//CreatePlaneCollision (scene, dVector (0.0f, 1.0f, 0.0f, 0.0f));

	NewtonWorld* const world = scene->GetNewton();

	int defaulMaterial = NewtonMaterialGetDefaultGroupID(scene->GetNewton());
	NewtonMaterialSetDefaultFriction(world, defaulMaterial, defaulMaterial, 0.6f, 0.5f);

	int materialList[] = {defaulMaterial };

	// create a vehicle controller manager
	SuperCarVehicleControllerManager* const manager = new SuperCarVehicleControllerManager (world, 1, materialList);

//	int defaulMaterial = NewtonMaterialGetDefaultGroupID(scene->GetNewton());
//	NewtonMaterialSetDefaultFriction(scene->GetNewton(), defaulMaterial, defaulMaterial, 0.9f, 0.9f);

	// create a Bezier Spline path for AI car to drive                     e
	manager->CreatedrivingTestCourt (scene);
	//manager->AddCones (scene);

	dFloat u = 1.0f;
	for (int i = 0; i < 1; i ++) {
		dVector offset (0.0f, 50.0f, 0.0f, 0.0f);
/*
		dMatrix location0 (manager->CalculateSplineMatrix (u));
		location0.m_posit += location0.m_right.Scale (3.0f);
		location0.m_posit.m_y += 1.0f;
		SuperCarEntity* const vehicle0 = new SuperCarEntity (scene, manager, location0 + offset, "lambDiablo.ngd", 3.0f);
		vehicle0->BuildWheelCar(1, -0.35f);
		u -= 0.005f;
*/
		dMatrix location1 (manager->CalculateSplineMatrix (u));
		location1.m_posit = FindFloor (scene->GetNewton(), location1.m_posit + offset, 100.0f);
		location1.m_posit.m_y += 1.0f;
		SuperCarEntity* const vehicle1 = new SuperCarEntity (scene, manager, location1, "viper.ngd", -3.0f);
		vehicle1->BuildWheelCar(0, VIPER_COM_Y_OFFSET);
		u -= 0.005f;
/*
		dMatrix location2 (manager->CalculateSplineMatrix (u));
		location2.m_posit += location2.m_right.Scale ( 3.0f);
		location2.m_posit = FindFloor (scene->GetNewton(), location2.m_posit + offset, 100.0f);
		location2.m_posit.m_y += 1.0f;
		SuperCarEntity* const vehicle2 = new SuperCarEntity (scene, manager, location2, "f1.ngd", 0.0f);
		vehicle2->BuildWheelCar(2, VIPER_COM_Y_OFFSET);
		u -= 0.01f;
*/
	}

	CustomVehicleController* const controller = &manager->GetLast()->GetInfo();
	SuperCarEntity* const vehicleEntity = (SuperCarEntity*)NewtonBodyGetUserData (controller->GetBody());

	// set this vehicle as the player
	manager->SetAsPlayer(vehicleEntity);
	manager->SetDebugVehicle(vehicleEntity);

	// set the camera matrix, we only care the initial direction since it will be following the player vehicle
	dMatrix camMatrix (vehicleEntity->GetNextMatrix());
//	scene->SetCameraMouseLock (true);

	camMatrix.m_posit.m_x -= 5.0f;
//camMatrix = dYawMatrix (0.5f * 3.1416f) * camMatrix;
	scene->SetCameraMatrix(camMatrix, camMatrix.m_posit);


	dMatrix location (camMatrix);
	location.m_posit.m_z += 4.0f;
	location.m_posit.m_x += 44.0f;

	int count = 5;
	dMatrix shapeOffsetMatrix (dGetIdentityMatrix());
	dVector size (3.0f, 0.125f, 3.0f, 0.0f);
//	AddPrimitiveArray(scene, 50.0f, location.m_posit, size, count, count, 6.0f, _BOX_PRIMITIVE, defaulMaterial, shapeOffsetMatrix);

	size = dVector(1.0f, 0.5f, 1.0f, 0.0f);
//	AddPrimitiveArray(scene, 50.0f, location.m_posit, size, count, count, 6.0f, _SPHERE_PRIMITIVE, defaulMaterial, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 50.0f, location.m_posit, size, count, count, 6.0f, _BOX_PRIMITIVE, defaulMaterial, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 50.0f, location.m_posit, size, count, count, 6.0f, _CAPSULE_PRIMITIVE, defaulMaterial, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 50.0f, location.m_posit, size, count, count, 6.0f, _CYLINDER_PRIMITIVE, defaulMaterial, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 50.0f, location.m_posit, size, count, count, 6.0f, _TAPERED_CAPSULE_PRIMITIVE, defaulMaterial, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 50.0f, location.m_posit, size, count, count, 6.0f, _TAPERED_CYLINDER_PRIMITIVE, defaulMaterial, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 50.0f, location.m_posit, size, count, count, 6.0f, _CHAMFER_CYLINDER_PRIMITIVE, defaulMaterial, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 50.0f, location.m_posit, size, count, count, 6.0f, _CONE_PRIMITIVE, defaulMaterial, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 50.0f, location.m_posit, size, count, count, 6.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaulMaterial, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 50.0f, location.m_posit, size, count, count, 6.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaulMaterial, shapeOffsetMatrix);

//	NewtonSerializeToFile (scene->GetNewton(), "C:/Users/Julio/Desktop/newton-dynamics/applications/media/xxxxx.bin");

}

