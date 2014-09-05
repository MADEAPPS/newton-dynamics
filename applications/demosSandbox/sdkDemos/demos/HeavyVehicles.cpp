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
#include "TargaToOpenGl.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "HeightFieldPrimitive.h"
#include "../toolBox/DebugDisplay.h"

#define HEAVY_TRANSPORT_MASS						5000.0f
#define HEAVY_TRANSPORT_COM_Y_OFFSET				-0.2f

#define HEAVY_TRANSPORT_TIRE_MASS					150.0f
#define HEAVY_TRANSPORT_SUSPENSION_LENGTH			  1.2f	
#define HEAVY_TRANSPORT_SUSPENSION_SPRING			 30.0f
#define HEAVY_TRANSPORT_SUSPENSION_DAMPER			  1.0f	
#define HEAVY_TRANSPORT_LATERAL_STIFFNESS			  40.0f	
#define HEAVY_TRANSPORT_LONGITUDINAL_STIFFNESS		2000.0f
#define HEAVY_TRANSPORT_ALIGNING_MOMENT_TRAIL		   1.5f

#define HEAVY_VEHICLE_THIRD_PERSON_VIEW_HIGHT		2.0f
#define HEAVY_VEHICLE_THIRD_PERSON_VIEW_DIST		10.0f
#define HEAVY_VEHICLE_THIRD_PERSON_VIEW_FILTER		0.125f


class HeavyVehicleEntity: public DemoEntity
{
	public: 
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


	HeavyVehicleEntity (DemoEntityManager* const scene, CustomVehicleControllerManager* const manager, const dMatrix& location, const char* const filename)
		:DemoEntity (dGetIdentityMatrix(), NULL)
		,m_controller(NULL)
		,m_helpKey (true)
		,m_gearUpKey (false)
		,m_gearDownKey (false)
		,m_reverseGear (false)
		,m_engineKeySwitch(false)
		,m_automaticTransmission(true)
		,m_engineKeySwitchCounter(0)
		,m_engineOldKeyState(false)
		,m_engineRPMOn(false)
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
		m_controller = manager->CreateVehicle (chassisCollision, chassisMatrix, HEAVY_TRANSPORT_MASS, dVector (0.0f, DEMO_GRAVITY, 0.0f, 0.0f));

		// get body from player
		NewtonBody* const body = m_controller->GetBody();

		// set the user data
		NewtonBodySetUserData(body, this);

		// set the transform callback
		NewtonBodySetTransformCallback (body, DemoEntity::TransformCallback);

		// set the standard force and torque call back
		NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);

		// set the player matrix 
		NewtonBodySetMatrix(body, &location[0][0]);

		// destroy the collision helper shape 
		NewtonDestroyCollision(chassisCollision);

		for (int i = 0; i < int ((sizeof (m_gearMap) / sizeof (m_gearMap[0]))); i ++) {
			m_gearMap[i] = i;
		}
		m_gearMap[0] = 1;
		m_gearMap[1] = 0;
	}

	~HeavyVehicleEntity ()
	{
		((CustomVehicleControllerManager*)m_controller->GetManager())->DestroyController(m_controller);
	}

	NewtonCollision* CreateChassisCollision (NewtonWorld* const world) const
	{
		DemoEntity* const chassis = dHierarchy<DemoEntity>::Find("chassis");
		dAssert (chassis);

		DemoMesh* const mesh = chassis->GetMesh();
		//dAssert (chassis->GetMeshMatrix().TestIdentity());
		const dMatrix& meshMatrix = chassis->GetMeshMatrix();

		dVector* const temp = new dVector[mesh->m_vertexCount];
		meshMatrix.TransformTriplex (&temp[0].m_x, sizeof (dVector), mesh->m_vertex, 3 * sizeof (dFloat), mesh->m_vertexCount);
		NewtonCollision* const shape = NewtonCreateConvexHull (world, mesh->m_vertexCount, &temp[0].m_x, sizeof (dVector), 0.001f, 0, NULL);
		delete[] temp;
		return shape;
	}

	// interpolate all skeleton transform 
	virtual void InterpolateMatrix (DemoEntityManager& world, dFloat param)
	{
		DemoEntity::InterpolateMatrix (world, param);
		if (m_controller){
			for (CustomVehicleControllerBodyStateTire* tire = m_controller->GetFirstTire(); tire; tire = m_controller->GetNextTire(tire)) {
				DemoEntity* const tirePart = (DemoEntity*) tire->GetUserData();
				tirePart->InterpolateMatrix (world, param);
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
		DemoMesh* const tireMesh = tirePart->GetMesh();
		//dAssert (tirePart->GetMeshMatrix().TestIdentity());
		const dMatrix& meshMatrix = tirePart->GetMeshMatrix();
		dVector* const temp = new dVector [tireMesh->m_vertexCount];
		meshMatrix.TransformTriplex (&temp[0].m_x, sizeof (dVector), tireMesh->m_vertex, 3 * sizeof (dFloat), tireMesh->m_vertexCount);
		NewtonCollision* const collision = NewtonCreateConvexHull(world, tireMesh->m_vertexCount, &temp[0].m_x, sizeof (dVector), 0, 0, NULL);
		delete[] temp;

		// get the location of this tire relative to the car chassis
		dMatrix tireMatrix (tirePart->CalculateGlobalMatrix(entity));

		// find the support points that can be used to define the with and high of the tire collision mesh
		dVector extremePoint;
		dVector upDir (0.0f, 1.0f, 0.0f, 0.0f);
		NewtonCollisionSupportVertex (collision, &upDir[0], &extremePoint[0]);
		radius = dAbs (upDir % extremePoint);

		dVector widthDir (1.0f, 0.0f, 0.0f, 0.0f);
		NewtonCollisionSupportVertex (collision, &widthDir[0], &extremePoint[0]);
		width = widthDir % extremePoint;

		widthDir = widthDir.Scale (-1.0f);
		NewtonCollisionSupportVertex (collision, &widthDir[0], &extremePoint[0]);
		width += widthDir % extremePoint;

		// destroy the auxiliary collision
		NewtonDestroyCollision (collision);	
	}


	CustomVehicleControllerBodyStateTire* AddTire (const char* const tireName, const dVector& offset, dFloat width, dFloat radius, dFloat mass, dFloat suspensionLength, dFloat suspensionSpring, dFloat suspensionDamper, dFloat lateralStiffness, dFloat longitudinalStiffness, dFloat aligningMOmentTrail) 
	{
		NewtonBody* const body = m_controller->GetBody();
		DemoEntity* const entity = (DemoEntity*) NewtonBodyGetUserData(body);
		DemoEntity* const tirePart = entity->Find (tireName);

		// add this tire, get local position and rise it by the suspension length 
		dMatrix tireMatrix (tirePart->CalculateGlobalMatrix(entity));

		// add the offset location
		tireMatrix.m_posit += offset;

		//lower the tire base position by some distance
		tireMatrix.m_posit.m_y -= suspensionLength * 0.25f;

		// add and alignment matrix,to match visual mesh to physics collision shape
		dMatrix aligmentMatrix (dGetIdentityMatrix());
		if (tireMatrix[0][2] < 0.0f) {
			aligmentMatrix = dYawMatrix(3.141592f);
		}
		TireAligmentTransform* const m_ligmentMatrix = new TireAligmentTransform(aligmentMatrix);
		tirePart->SetUserData(m_ligmentMatrix);

		// add the tire to the vehicle
		CustomVehicleControllerBodyStateTire::TireCreationInfo tireInfo;
		tireInfo.m_location = tireMatrix.m_posit;
		tireInfo.m_mass = mass;
		tireInfo.m_radio = radius;
		tireInfo.m_width = width;
		tireInfo.m_dampingRatio = suspensionDamper;
		tireInfo.m_springStrength = suspensionSpring;
		tireInfo.m_suspesionlenght = suspensionLength;
		tireInfo.m_lateralStiffness = lateralStiffness;
		tireInfo.m_longitudialStiffness = longitudinalStiffness;
		tireInfo.m_aligningMomentTrail =  aligningMOmentTrail;
		tireInfo.m_userData = tirePart;

		return m_controller->AddTire (tireInfo);
	}


	void UpdateTireTransforms()
	{
		NewtonBody* const body = m_controller->GetBody();
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(NewtonBodyGetWorld(body));
		
#if 0
		// this is the general way for getting the tire matrices
		dMatrix rootMatrixInv (GetNextMatrix().Inverse());
		for (CustomVehicleControllerBodyStateTire* tire = m_controller->GetFirstTire(); tire; tire = m_controller->GetNextTire(tire)) {
			DemoEntity* const tirePart = (DemoEntity*) tire->GetUserData();
			TireAligmentTransform* const aligmentMatrix = (TireAligmentTransform*)tirePart->GetUserData();
			dMatrix matrix (aligmentMatrix->m_matrix * tire->CalculateGlobalMatrix() * rootMatrixInv);
			dQuaternion rot (matrix);
			tirePart->SetMatrix(*scene, rot, matrix.m_posit);
		}
#else
		// this saves some calculation since it get the tire local to the chassis
		for (CustomVehicleControllerBodyStateTire* tire = m_controller->GetFirstTire(); tire; tire = m_controller->GetNextTire(tire)) {
			DemoEntity* const tirePart = (DemoEntity*) tire->GetUserData();
			TireAligmentTransform* const aligmentMatrix = (TireAligmentTransform*)tirePart->GetUserData();
			dMatrix matrix (aligmentMatrix->m_matrix * tire->CalculateLocalMatrix());
			dQuaternion rot (matrix);
			tirePart->SetMatrix(*scene, rot, matrix.m_posit);
		}
#endif
	}

	// this function is an example of how to make a high performance super car
	void BuildAllWheelDriveVehicle ()
	{
		// step one: find the location of each tire, in the visual mesh and add them one by one to the vehicle controller 
		dFloat width;
		dFloat radius;

		// Muscle cars have the front engine, we need to shift the center of mass to the front to represent that
		m_controller->SetCenterOfGravity (dVector (0.0f, HEAVY_TRANSPORT_COM_Y_OFFSET, 0.0f, 0.0f)); 


		// a car may have different size front an rear tire, therefore we do this separate for front and rear tires
		CalculateTireDimensions ("ltire_0", width, radius);

		dVector offset (0.0f, 0.30f, 0.0f, 0.0f);
		CustomVehicleControllerBodyStateTire* const leftTire0 = AddTire ("ltire_0", offset, width, radius, HEAVY_TRANSPORT_MASS, HEAVY_TRANSPORT_SUSPENSION_LENGTH, HEAVY_TRANSPORT_SUSPENSION_SPRING, HEAVY_TRANSPORT_SUSPENSION_DAMPER, HEAVY_TRANSPORT_LATERAL_STIFFNESS, HEAVY_TRANSPORT_LONGITUDINAL_STIFFNESS, HEAVY_TRANSPORT_ALIGNING_MOMENT_TRAIL);
//		CustomVehicleControllerBodyStateTire* const leftTire1 = AddTire ("ltire_1", offset, width, radius, HEAVY_TRANSPORT_MASS, HEAVY_TRANSPORT_SUSPENSION_LENGTH, HEAVY_TRANSPORT_SUSPENSION_SPRING, HEAVY_TRANSPORT_SUSPENSION_DAMPER, HEAVY_TRANSPORT_LATERAL_STIFFNESS, HEAVY_TRANSPORT_LONGITUDINAL_STIFFNESS, HEAVY_TRANSPORT_ALIGNING_MOMENT_TRAIL);
//		CustomVehicleControllerBodyStateTire* const leftTire2 = AddTire ("ltire_2", offset, width, radius, HEAVY_TRANSPORT_MASS, HEAVY_TRANSPORT_SUSPENSION_LENGTH, HEAVY_TRANSPORT_SUSPENSION_SPRING, HEAVY_TRANSPORT_SUSPENSION_DAMPER, HEAVY_TRANSPORT_LATERAL_STIFFNESS, HEAVY_TRANSPORT_LONGITUDINAL_STIFFNESS, HEAVY_TRANSPORT_ALIGNING_MOMENT_TRAIL);
		CustomVehicleControllerBodyStateTire* const leftTire3 = AddTire ("ltire_3", offset, width, radius, HEAVY_TRANSPORT_MASS, HEAVY_TRANSPORT_SUSPENSION_LENGTH, HEAVY_TRANSPORT_SUSPENSION_SPRING, HEAVY_TRANSPORT_SUSPENSION_DAMPER, HEAVY_TRANSPORT_LATERAL_STIFFNESS, HEAVY_TRANSPORT_LONGITUDINAL_STIFFNESS, HEAVY_TRANSPORT_ALIGNING_MOMENT_TRAIL);

		CustomVehicleControllerBodyStateTire* const rightTire0 = AddTire ("rtire_0", offset, width, radius, HEAVY_TRANSPORT_MASS, HEAVY_TRANSPORT_SUSPENSION_LENGTH, HEAVY_TRANSPORT_SUSPENSION_SPRING, HEAVY_TRANSPORT_SUSPENSION_DAMPER, HEAVY_TRANSPORT_LATERAL_STIFFNESS, HEAVY_TRANSPORT_LONGITUDINAL_STIFFNESS, HEAVY_TRANSPORT_ALIGNING_MOMENT_TRAIL);
//		CustomVehicleControllerBodyStateTire* const rightTire1 = AddTire ("rtire_1", offset, width, radius, HEAVY_TRANSPORT_MASS, HEAVY_TRANSPORT_SUSPENSION_LENGTH, HEAVY_TRANSPORT_SUSPENSION_SPRING, HEAVY_TRANSPORT_SUSPENSION_DAMPER, HEAVY_TRANSPORT_LATERAL_STIFFNESS, HEAVY_TRANSPORT_LONGITUDINAL_STIFFNESS, HEAVY_TRANSPORT_ALIGNING_MOMENT_TRAIL);
//		CustomVehicleControllerBodyStateTire* const rightTire2 = AddTire ("rtire_2", offset, width, radius, HEAVY_TRANSPORT_MASS, HEAVY_TRANSPORT_SUSPENSION_LENGTH, HEAVY_TRANSPORT_SUSPENSION_SPRING, HEAVY_TRANSPORT_SUSPENSION_DAMPER, HEAVY_TRANSPORT_LATERAL_STIFFNESS, HEAVY_TRANSPORT_LONGITUDINAL_STIFFNESS, HEAVY_TRANSPORT_ALIGNING_MOMENT_TRAIL);
		CustomVehicleControllerBodyStateTire* const rightTire3 = AddTire ("rtire_3", offset, width, radius, HEAVY_TRANSPORT_MASS, HEAVY_TRANSPORT_SUSPENSION_LENGTH, HEAVY_TRANSPORT_SUSPENSION_SPRING, HEAVY_TRANSPORT_SUSPENSION_DAMPER, HEAVY_TRANSPORT_LATERAL_STIFFNESS, HEAVY_TRANSPORT_LONGITUDINAL_STIFFNESS, HEAVY_TRANSPORT_ALIGNING_MOMENT_TRAIL);

/*
		// add real tires
		CalculateTireDimensions ("rl_tire", width, radius);
		dVector offset1 (0.0f, 0.05f, 0.0f, 0.0f);
		CustomVehicleControllerBodyStateTire* const leftRearTire = AddTire ("rl_tire", offset1, width, radius, VIPER_TIRE_MASS, VIPER_TIRE_SUSPENSION_LENGTH, VIPER_TIRE_SUSPENSION_SPRING, VIPER_TIRE_SUSPENSION_DAMPER, VIPER_TIRE_LATERAL_STIFFNESS, VIPER_TIRE_LONGITUDINAL_STIFFNESS, VIPER_TIRE_ALIGNING_MOMENT_TRAIL);
		CustomVehicleControllerBodyStateTire* const rightRearTire = AddTire ("rr_tire", offset1, width, radius, VIPER_TIRE_MASS, VIPER_TIRE_SUSPENSION_LENGTH, VIPER_TIRE_SUSPENSION_SPRING, VIPER_TIRE_SUSPENSION_DAMPER, VIPER_TIRE_LATERAL_STIFFNESS, VIPER_TIRE_LONGITUDINAL_STIFFNESS, VIPER_TIRE_ALIGNING_MOMENT_TRAIL);

		// add an steering Wheel
		CustomVehicleControllerComponentSteering* const steering = new CustomVehicleControllerComponentSteering (m_controller, VIPER_TIRE_STEER_ANGLE * 3.141592f / 180.0f);
		steering->AddSteeringTire(leftFrontTire, -1.0f);
		steering->AddSteeringTire(rightFrontTire, -1.0f);
		m_controller->SetSteering(steering);

		// add vehicle brakes
		CustomVehicleControllerComponentBrake* const brakes = new CustomVehicleControllerComponentBrake (m_controller, VIPER_TIRE_BRAKE_TORQUE);
		brakes->AddBrakeTire (leftFrontTire);
		brakes->AddBrakeTire (rightFrontTire);
		brakes->AddBrakeTire (leftRearTire);
		brakes->AddBrakeTire (rightRearTire);
		m_controller->SetBrakes(brakes);

		// add vehicle hand brakes
		CustomVehicleControllerComponentBrake* const handBrakes = new CustomVehicleControllerComponentBrake (m_controller, VIPER_TIRE_BRAKE_TORQUE);
		handBrakes->AddBrakeTire (leftRearTire);
		handBrakes->AddBrakeTire (rightRearTire);
		m_controller->SetHandBrakes(handBrakes);

		// add an engine
		// first make the gear Box
		dFloat fowardSpeedGearsBoxRatios[] = {VIPER_TIRE_GEAR_1, VIPER_TIRE_GEAR_2, VIPER_TIRE_GEAR_3, VIPER_TIRE_GEAR_4, VIPER_TIRE_GEAR_5, VIPER_TIRE_GEAR_6};

		CustomVehicleControllerComponentEngine::dSingleAxelDifferencial* const frontDifferencial = new CustomVehicleControllerComponentEngine::dSingleAxelDifferencial (m_controller, leftFrontTire, rightFrontTire);
		CustomVehicleControllerComponentEngine::dSingleAxelDifferencial* const rearDifferencial = new CustomVehicleControllerComponentEngine::dSingleAxelDifferencial (m_controller, leftRearTire, rightRearTire);

		CustomVehicleControllerComponentEngine::dSingleAxelDifferencial* array[2] = {frontDifferencial, rearDifferencial};
		CustomVehicleControllerComponentEngine::dMultiAxelDifferencial* const differencial = new CustomVehicleControllerComponentEngine::dMultiAxelDifferencial (m_controller, 2, array);

		CustomVehicleControllerComponentEngine::dGearBox* const gearBox = new CustomVehicleControllerComponentEngine::dGearBox(m_controller, VIPER_TIRE_GEAR_REVERSE, sizeof (fowardSpeedGearsBoxRatios) / sizeof (fowardSpeedGearsBoxRatios[0]), fowardSpeedGearsBoxRatios); 
		CustomVehicleControllerComponentEngine* const engine = new CustomVehicleControllerComponentEngine (m_controller, gearBox, differencial);

		// the the default transmission type
		engine->SetTransmissionMode(m_automaticTransmission.GetPushButtonState());

		m_controller->SetEngine(engine);


		dFloat viperIdleRPM = VIPER_IDLE_TORQUE_RPM;
		dFloat viperIdleTorquePoundPerFoot = VIPER_IDLE_TORQUE;

		dFloat viperPeakTorqueRPM = VIPER_PEAK_TORQUE_RPM;
		dFloat viperPeakTorquePoundPerFoot = VIPER_PEAK_TORQUE;

		dFloat viperPeakHorsePowerRPM = VIPER_PEAK_HP_RPM;
		dFloat viperPeakHorsePower = VIPER_PEAK_HP;

		dFloat viperRedLineRPM = VIPER_REDLINE_TORQUE_RPM;
		dFloat viperRedLineTorquePoundPerFoot = VIPER_REDLINE_TORQUE;

		dFloat vehicleTopSpeedKPH = VIPER_TIRE_TOP_SPEED_KMH;
		engine->InitEngineTorqueCurve (vehicleTopSpeedKPH, viperIdleTorquePoundPerFoot, viperIdleRPM, viperPeakTorquePoundPerFoot, viperPeakTorqueRPM, viperPeakHorsePower, viperPeakHorsePowerRPM, viperRedLineTorquePoundPerFoot, viperRedLineRPM);
*/
		// do not forget to call finalize after all components are added or after any change is made to the vehicle
		m_controller->Finalize();
	}

	void ApplyPlayerControl ()
	{
		NewtonBody* const body = m_controller->GetBody();
		NewtonWorld* const world = NewtonBodyGetWorld(body);
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);
		NewtonDemos* const mainWindow = scene->GetRootWindow();

		CustomVehicleControllerComponentEngine* const engine = m_controller->GetEngine();
		CustomVehicleControllerComponentSteering* const steering = m_controller->GetSteering();
		CustomVehicleControllerComponentBrake* const brakes = m_controller->GetBrakes();
		CustomVehicleControllerComponentBrake* const handBrakes = m_controller->GetHandBrakes();

		// get the throttler input
		dFloat joyPosX;
		dFloat joyPosY;
		int joyButtons;

		int gear = engine ? engine->GetGear() : CustomVehicleControllerComponentEngine::dGearBox::m_newtralGear;
		dFloat steeringVal = 0.0f;
		dFloat engineGasPedal = 0.0f;
		dFloat brakePedal = 0.0f;
		dFloat handBrakePedal = 0.0f;
	
		bool hasJopytick = mainWindow->GetJoytickPosition (joyPosX, joyPosY, joyButtons);
		if (hasJopytick) {
			dAssert (0);
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
	
			// get keyboard controls
			if (mainWindow->GetKeyState ('W')) {
				engineGasPedal = 1.0f;
			}

			// see if HandBreale is on
			if (mainWindow->GetKeyState ('S')) {
				brakePedal = 1.0f;
			}

			// see if HandBreale is on
			if (mainWindow->GetKeyState (' ')) {
				handBrakePedal = 1.0f;
			}


			// get the steering input
			steeringVal = (dFloat (mainWindow->GetKeyState ('D')) - dFloat (mainWindow->GetKeyState ('A')));

/*
			// check for gear change (note key code for '>' = '.' and key code for '<' == ',')
			gear += int (m_gearUpKey.UpdateTriggerButton(mainWindow, '.')) - int (m_gearDownKey.UpdateTriggerButton(mainWindow, ','));
			// do driving heuristic for automatic transmission
			if (engine->GetTransmissionMode()) {
				dFloat speed = engine->GetSpeed();
				// check if vehicle is parked
				if ((dAbs (speed) < 1.0f) && !engineGasPedal && !brakePedal && !handBrakePedal) {
					handBrakePedal = 0.5f;
				}
			}
*/
		}

			
		// set the help key
		m_helpKey.UpdatePushButton (mainWindow, 'H');

		int reverseGear = m_reverseGear.UpdateTriggerButton (mainWindow, 'R') ? 1 : 0;

		// count the engine key switch
		m_engineKeySwitchCounter += m_engineKeySwitch.UpdateTriggerButton(mainWindow, 'Y');

		// check transmission type
		int toggleTransmission = m_automaticTransmission.UpdateTriggerButton (mainWindow, 0x0d) ? 1 : 0;

#if 0
	#if 0
		static FILE* file = fopen ("log.bin", "wb");                                         
		if (file) {
			fwrite (&m_engineKeySwitchCounter, sizeof (int), 1, file);
			fwrite (&toggleTransmission, sizeof (int), 1, file);
			fwrite (&reverseGear, sizeof (int), 1, file);
			fwrite (&gear, sizeof (int), 1, file);
			fwrite (&steeringVal, sizeof (dFloat), 1, file);
			fwrite (&engineGasPedal, sizeof (dFloat), 1, file);
			fwrite (&handBrakePedal, sizeof (dFloat), 1, file);
			fwrite (&brakePedal, sizeof (dFloat), 1, file);
			fflush(file);
		}
	#else 
		static FILE* file = fopen ("log.bin", "rb");
		if (file) {		
			fread (&m_engineKeySwitchCounter, sizeof (int), 1, file);
			fread (&toggleTransmission, sizeof (int), 1, file);
			fread (&reverseGear, sizeof (int), 1, file);
			fread (&gear, sizeof (int), 1, file);
			fread (&steeringVal, sizeof (dFloat), 1, file);
			fread (&engineGasPedal, sizeof (dFloat), 1, file);
			fread (&handBrakePedal, sizeof (dFloat), 1, file);
			fread (&brakePedal, sizeof (dFloat), 1, file);

		}
	#endif
#endif

		if (engine) {
			m_engineOldKeyState = engine->GetKey();
			engine->SetKey ((m_engineKeySwitchCounter & 1) ? true : false);

			if (toggleTransmission) {
				engine->SetTransmissionMode (!engine->GetTransmissionMode());
			}
			if (!engine->GetTransmissionMode()) {
				engine->SetGear(gear);
			}
			if (reverseGear) {
				if (engine->GetGear() == CustomVehicleControllerComponentEngine::dGearBox::m_reverseGear) {
					engine->SetGear(CustomVehicleControllerComponentEngine::dGearBox::m_newtralGear);
				} else {
					engine->SetGear(CustomVehicleControllerComponentEngine::dGearBox::m_reverseGear);
				}
			}

			engine->SetParam(engineGasPedal);
		}
		if (steering) {
			steering->SetParam(steeringVal);
		}
		if (brakes) {
			brakes->SetParam(brakePedal);
		}
		if (handBrakes) {
			handBrakes->SetParam(handBrakePedal);
		}
		
	}

	void ApplyNPCControl ()
	{
//		dAssert (0);
	}
	
	void Debug () const 
	{
		NewtonBody* const body = m_controller->GetBody();
		const CustomVehicleControllerBodyStateChassis& chassis = m_controller->GetChassisState ();

		dFloat scale = -4.0f / (chassis.GetMass() * DEMO_GRAVITY);
		dVector p0 (chassis.GetCenterOfMass());

		glDisable (GL_LIGHTING);
		glDisable(GL_TEXTURE_2D);

		glLineWidth(3.0f);
		glBegin(GL_LINES);


		// draw vehicle weight at the center of mass
		dFloat lenght = scale * chassis.GetMass() * DEMO_GRAVITY;
		glColor3f(0.0f, 0.0f, 1.0f);
		glVertex3f (p0.m_x, p0.m_y, p0.m_z);
		glVertex3f (p0.m_x, p0.m_y - lenght, p0.m_z);

		// draw vehicle front dir
		glColor3f(1.0f, 1.0f, 1.0f);
		dVector r0 (p0 + chassis.GetMatrix()[1].Scale (0.5f));
		dVector r1 (r0 + chassis.GetMatrix()[0].Scale (1.0f));
		glVertex3f (r0.m_x, r0.m_y, r0.m_z);
		glVertex3f (r1.m_x, r1.m_y, r1.m_z);


		// draw the velocity vector, a little higher so that is not hidden by the vehicle mesh 
		dVector veloc;
		NewtonBodyGetVelocity(body, &veloc[0]);
		dVector q0 (p0 + chassis.GetMatrix()[1].Scale (1.0f));
		dVector q1 (q0 + veloc.Scale (0.25f));
		glColor3f(1.0f, 1.0f, 0.0f);
		glVertex3f (q0.m_x, q0.m_y, q0.m_z);
		glVertex3f (q1.m_x, q1.m_y, q1.m_z);
		
		for (CustomVehicleControllerBodyStateTire* node = m_controller->GetFirstTire(); node; node = m_controller->GetNextTire(node)) {
			const CustomVehicleControllerBodyStateTire& tire = *node;
			dVector p0 (tire.GetCenterOfMass());

const dMatrix& tireMatrix = tire.GetLocalMatrix ();
p0 += chassis.GetMatrix()[2].Scale ((tireMatrix.m_posit.m_z > 0.0f ? 1.0f : -1.0f) * 0.25f);

			// draw the tire load 
			dVector p1 (p0 + tire.GetTireLoad().Scale (scale));
			glColor3f (0.0f, 0.0f, 1.0f);
			glVertex3f (p0.m_x, p0.m_y, p0.m_z);
			glVertex3f (p1.m_x, p1.m_y, p1.m_z);

			// show tire lateral force
			dVector p2 (p0 - tire.GetLateralForce().Scale (scale));
			glColor3f(1.0f, 0.0f, 0.0f);
			glVertex3f (p0.m_x, p0.m_y, p0.m_z);
			glVertex3f (p2.m_x, p2.m_y, p2.m_z);

			// show tire longitudinal force
			dVector p3 (p0 - tire.GetLongitudinalForce().Scale (scale));
			glColor3f (0.0f, 1.0f, 0.0f);
			glVertex3f (p0.m_x, p0.m_y, p0.m_z);
			glVertex3f (p3.m_x, p3.m_y, p3.m_z);
		}

		glEnd();

		glLineWidth(1.0f);

	}


	CustomVehicleController* m_controller;
	DemoEntityManager::ButtonKey m_helpKey;
	DemoEntityManager::ButtonKey m_gearUpKey;
	DemoEntityManager::ButtonKey m_gearDownKey;
	DemoEntityManager::ButtonKey m_reverseGear;
	DemoEntityManager::ButtonKey m_engineKeySwitch;
	DemoEntityManager::ButtonKey m_automaticTransmission;
	int m_gearMap[10];
	int m_engineKeySwitchCounter;
	bool m_engineOldKeyState;
	bool m_engineRPMOn;
};

class HeavyVehicleControllerManager: public CustomVehicleControllerManager
{
	public:
	HeavyVehicleControllerManager (NewtonWorld* const world)
		:CustomVehicleControllerManager (world)
		,m_externalView(true)
		,m_player (NULL) 
	{
		// hook a callback for 2d help display
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);
		scene->Set2DDisplayRenderFunction (RenderVehicleHud, this);
	}

	~HeavyVehicleControllerManager ()
	{
	}

	static void RenderVehicleHud (DemoEntityManager* const scene, void* const context, int lineNumber)
	{
		HeavyVehicleControllerManager* const me = (HeavyVehicleControllerManager*) context;
		me->RenderVehicleHud (scene, lineNumber);
	}

	void DrawHelp(DemoEntityManager* const scene, int lineNumber) const
	{
		if (m_player->m_helpKey.GetPushButtonState()) {
			dVector color(1.0f, 1.0f, 0.0f, 0.0f);
			lineNumber = scene->Print (color, 10, lineNumber + 20, "Vehicle driving keyboard control:   Joystick control");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "key switch			: 'Y'           start engine");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "accelerator         : 'W'           stick forward");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "brakes              : 'S'           stick back");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "turn right          : 'D'           stick right");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "turn right          : 'S'           stick left");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "toggle Reverse Gear	: 'R'           toggle reverse Gear");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "gear up             : '>'           button 2");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "gear down           : '<'           button 4");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "manual transmission : enter         button 4");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "hand brakes         : space         button 1");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "hide help           : H");
		}
	}


	void RenderVehicleHud (DemoEntityManager* const scene, int lineNumber) const
	{
		if (m_player) {
			// set to transparent color
			glEnable (GL_BLEND);
			glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

			//print controllers help 
			DrawHelp(scene, lineNumber);

			// restore color and blend mode
			glDisable (GL_BLEND);
		}
	}


	void SetAsPlayer (HeavyVehicleEntity* const player)
	{
		m_player = player;
	}


	virtual void PreUpdate (dFloat timestep)
	{
		// apply the vehicle controls, and all simulation time effect
		//NewtonWorld* const world = GetWorld(); 
		//DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);
		for (dListNode* ptr = GetFirst(); ptr; ptr = ptr->GetNext()) {
			CustomVehicleController* const controller = &ptr->GetInfo();

			NewtonBody* const body = controller->GetBody();
			HeavyVehicleEntity* const vehicleEntity = (HeavyVehicleEntity*) NewtonBodyGetUserData(body);

			if (vehicleEntity == m_player) {
				// do player control
				vehicleEntity->ApplyPlayerControl ();
			} else {
				// do no player control
				vehicleEntity->ApplyNPCControl ();
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
			HeavyVehicleEntity* const vehicleEntity = (HeavyVehicleEntity*)NewtonBodyGetUserData (controller->GetBody());
			vehicleEntity->UpdateTireTransforms();
		}

		UpdateCamera (timestep);
	}


	void UpdateCamera (dFloat timestep)
	{
		if (m_player) {
			DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(GetWorld());
			DemoCamera* const camera = scene->GetCamera();
			dMatrix camMatrix (camera->GetNextMatrix ());
			dMatrix playerMatrix (m_player->GetNextMatrix());

			dVector frontDir (camMatrix[0]);
			dVector camOrigin; 
			if (m_externalView) {
				camOrigin = playerMatrix.m_posit + dVector(0.0f, HEAVY_VEHICLE_THIRD_PERSON_VIEW_HIGHT, 0.0f, 0.0f);
				camOrigin -= frontDir.Scale(HEAVY_VEHICLE_THIRD_PERSON_VIEW_DIST);
			} else {
				dAssert (0);
				//            camMatrix = camMatrix * playerMatrix;
				//            camOrigin = playerMatrix.TransformVector(dVector(-0.8f, ARTICULATED_VEHICLE_CAMERA_EYEPOINT, 0.0f, 0.0f));
			}

			camera->SetNextMatrix (*scene, camMatrix, camOrigin);
		}
	}

	// use this to display debug information about vehicle 
	void Debug () const
	{
		for (dListNode* ptr = GetFirst(); ptr; ptr = ptr->GetNext()) {
			CustomVehicleController* const controller = &ptr->GetInfo();
			HeavyVehicleEntity* const vehicleEntity = (HeavyVehicleEntity*)NewtonBodyGetUserData (controller->GetBody());
			vehicleEntity->Debug();
		}
	}

	bool m_externalView;
	HeavyVehicleEntity* m_player;
};


void MilitaryTransport (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	CreateLevelMesh (scene, "flatPlane.ngd", 1);
//	CreateHeightFieldTerrain (scene, 10, 8.0f, 1.5f, 0.2f, 200.0f, -50.0f);

//	dMatrix camMatrix (dRollMatrix(-20.0f * 3.1416f /180.0f) * dYawMatrix(-45.0f * 3.1416f /180.0f));
	dMatrix location (dGetIdentityMatrix());
	location.m_posit = dVector (1000.0f, 100.0f, 1000.0f, 1.0f);
location.m_posit.m_x = 126.0f;
location.m_posit.m_y = 50.0f;
location.m_posit.m_z = 50.0f;

	location.m_posit = FindFloor (scene->GetNewton(), location.m_posit, 100.0f);
	location.m_posit.m_y += 2.0f;

	NewtonWorld* const world = scene->GetNewton();

	// create a vehicle controller manager
	HeavyVehicleControllerManager* const manager = new HeavyVehicleControllerManager (world);
	
	HeavyVehicleEntity* const vehicle = new HeavyVehicleEntity (scene, manager, location, "lav-25.ngd");

	vehicle->BuildAllWheelDriveVehicle();

	// set this vehicle as the player
	manager->SetAsPlayer(vehicle);

	dMatrix camMatrix (manager->m_player->GetNextMatrix());
	scene->SetCameraMouseLock (true);
	scene->SetCameraMatrix(camMatrix, camMatrix.m_posit);

/*
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (scene->GetNewton());
	dVector location (origin);
	location.m_x += 20.0f;
	location.m_z += 20.0f;
	dVector size (0.5f, 0.5f, 0.75f, 0.0f);

	int count = 5;
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _TAPERED_CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _TAPERED_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CHAMFER_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
*/
}

