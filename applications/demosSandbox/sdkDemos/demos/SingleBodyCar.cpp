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
#include "TargaToOpenGl.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "HeightFieldPrimitive.h"
#include "DebugDisplay.h"

#define VEHICLE_THIRD_PERSON_VIEW_HIGHT		2.0f
#define VEHICLE_THIRD_PERSON_VIEW_DIST		7.0f
#define VEHICLE_THIRD_PERSON_VIEW_FILTER	0.125f


class SingleBodyVehicleManager: public dVehicleManager
{
	public:
	class VehicleUserData: public DemoEntity::UserData
	{
		public:
		VehicleUserData(dVehicleChassis* const vehicle)
			:DemoEntity::UserData()
			,m_vehicleChassis(vehicle)
		{
			memset(m_gearMap, 0, sizeof (m_gearMap));
		}

		void OnRender(dFloat timestep) const
		{
		}

/*
		void OnTransformCallback(DemoEntityManager& world) const
		{
			// calculate tire Matrices
			dVehicleInterface* const vehicle = m_vehicleChassis->GetVehicle();
			dMatrix chassisMatrixInv(vehicle->GetMatrix().Inverse());

			const dList<dAnimAcyclicJoint*>& children = vehicle->GetChildren();
			for (dList<dAnimAcyclicJoint*>::dListNode* node = children.GetFirst(); node; node = node->GetNext()) {
				dVehicleTireInterface* const tire = ((dVehicleNode*)node->GetInfo())->GetAsTire();
				if (tire) {
					DemoEntity* const tireMesh = (DemoEntity*)tire->GetUserData();
					dMatrix tireMatrix(tire->GetGlobalMatrix() * chassisMatrixInv);
					dQuaternion rotation(tireMatrix);
					tireMesh->SetMatrixUsafe(rotation, tireMatrix.m_posit);
				}
			}
		}
*/

		dVehicleChassis* m_vehicleChassis;
		int m_gearMap[10];
	};

	SingleBodyVehicleManager(NewtonWorld* const world)
		:dVehicleManager(world)
		,m_player(NULL)
		,m_externalView(true)
	{
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);
		scene->SetUpdateCameraFunction(UpdateCameraCallback, this);

		scene->Set2DDisplayRenderFunction(RenderHelpMenu, RenderUI, this);

		// load 2d display assets
		m_gears = LoadTexture("gears_font.tga");
		m_odometer = LoadTexture("kmh_dial.tga");
		m_tachometer = LoadTexture("rpm_dial.tga");
		m_redNeedle = LoadTexture("needle_red.tga");
		m_greenNeedle = LoadTexture("needle_green.tga");
	}

	~SingleBodyVehicleManager()
	{
		ReleaseTexture (m_gears);
		ReleaseTexture (m_odometer);
		ReleaseTexture (m_redNeedle);
		ReleaseTexture (m_tachometer);
		ReleaseTexture (m_greenNeedle);
	}

	static void UpdateCameraCallback(DemoEntityManager* const manager, void* const context, dFloat timestep)
	{
		SingleBodyVehicleManager* const me = (SingleBodyVehicleManager*)context;
		me->UpdateCamera(timestep);
	}

	static void RenderHelpMenu(DemoEntityManager* const scene, void* const context)
	{
		//SingleBodyVehicleManager* const me = (SingleBodyVehicleManager*)context;
		//me->DrawHelp(scene);
	}

	static void RenderUI(DemoEntityManager* const scene, void* const context)
	{
		SingleBodyVehicleManager* const me = (SingleBodyVehicleManager*)context;
		me->RenderUI(scene);
	}

	void DrawGage(GLuint gage, GLuint needle, dFloat param, dFloat origin_x, dFloat origin_y, dFloat size, dFloat minAngle, dFloat maxAngle) const
	{
		size *= 0.5f;
		dMatrix origin(dGetIdentityMatrix());
		origin[1][1] = -1.0f;
		origin.m_posit = dVector(origin_x, origin_y, 0.0f, 1.0f);

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
		dFloat angle = minAngle + (maxAngle - minAngle) * param;
		dMatrix needleMatrix(dRollMatrix(angle));

		dFloat x = size * 0.7f;
		dFloat y = size * 0.7f;

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

	void DrawGear(dFloat param, dFloat origin_x, dFloat origin_y, int gear, dFloat size) const
	{
		dMatrix origin(dGetIdentityMatrix());
		origin[1][1] = -1.0f;
		origin.m_posit = dVector(origin_x + size * 0.3f, origin_y - size * 0.25f, 0.0f, 1.0f);

		glPushMatrix();
		glMultMatrix(&origin[0][0]);

		dFloat uwith = 0.1f;
		dFloat u0 = uwith * gear;
		dFloat u1 = u0 + uwith;

		dFloat x1 = 10.0f;
		dFloat y1 = 10.0f;
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

	void RenderUI(DemoEntityManager* const scene)
	{
		// set to transparent color
		if (m_player) {

			DemoEntity* const playerEnt = (DemoEntity*)NewtonBodyGetUserData(m_player->GetBody());
			
			dVehicleEngineInterface* const engine = m_player->GetEngineControl()->GetEngine();
			if (engine) {
				glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
				dFloat gageSize = 200.0f;
				dFloat y = scene->GetHeight() - (gageSize / 2.0f + 20.0f);

				// draw the tachometer
				dFloat x = gageSize / 2 + 20.0f;
				dFloat rpm = engine->GetRpm() / engine->GetRedLineRpm();
				DrawGage(m_tachometer, m_redNeedle, rpm, x, y, gageSize, -180.0f, 0.0f);

				// draw the odometer
				x += gageSize;
				//dFloat speed = dAbs(engine->GetSpeed()) * 3.6f / 340.0f;
				//dFloat speed = dAbs(engine->GetSpeed()) / engine->GetTopSpeed();
				dFloat speed = 0.0f;
				DrawGage(m_odometer, m_greenNeedle, speed, x, y, gageSize, -180.0f, 90.0f);

				// draw the current gear
				//int gear = engine->GetGear();
				int gear = 1;
				VehicleUserData* const playerData = (VehicleUserData*)playerEnt->GetUserData();
				DrawGear(speed, x, y + 98, playerData->m_gearMap[gear], gageSize);
			}
		}
	}

	void UpdateCamera(dFloat timestep)
	{
//		SuperCarEntity* player = m_player;
//		if (!player) {
//			dCustomVehicleController* const controller = &GetLast()->GetInfo();
//			player = (SuperCarEntity*)NewtonBodyGetUserData(controller->GetBody());
//		}

return;
		DemoEntity* const player = (DemoEntity*)NewtonBodyGetUserData(m_player->GetBody());
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

	void SetAsPlayer(dVehicleChassis* const player)
	{
		//dEngineController* const engine = player->m_controller->GetEngine();
		//if (engine) {
		//	engine->SetIgnition(false);
		//}
		m_player = player;
	}

	void CalculateTireDimensions(const char* const tireName, dFloat& width, dFloat& radius, NewtonWorld* const world, DemoEntity* const vehEntity) const
	{
		// find the the tire visual mesh 
		DemoEntity* const tirePart = vehEntity->Find(tireName);
		dAssert(tirePart);

		// make a convex hull collision shape to assist in calculation of the tire shape size
		DemoMesh* const tireMesh = (DemoMesh*)tirePart->GetMesh();
		dAssert(tireMesh->IsType(DemoMesh::GetRttiType()));
		//dAssert (tirePart->GetMeshMatrix().TestIdentity());
		const dMatrix& meshMatrix = tirePart->GetMeshMatrix();
		dArray<dVector> temp (tireMesh->m_vertexCount);
		meshMatrix.TransformTriplex(&temp[0].m_x, sizeof (dVector), tireMesh->m_vertex, 3 * sizeof (dFloat), tireMesh->m_vertexCount);
		NewtonCollision* const collision = NewtonCreateConvexHull(world, tireMesh->m_vertexCount, &temp[0].m_x, sizeof (dVector), 0, 0, NULL);

		// get the location of this tire relative to the car chassis
		dMatrix tireMatrix(tirePart->CalculateGlobalMatrix(vehEntity));

		// find the support points that can be used to define the with and high of the tire collision mesh
		dVector extremePoint(0.0f);
		dVector upDir(tireMatrix.UnrotateVector(dVector(0.0f, 1.0f, 0.0f, 0.0f)));
		NewtonCollisionSupportVertex(collision, &upDir[0], &extremePoint[0]);
		radius = dAbs(upDir.DotProduct3(extremePoint));

		dVector widthDir(tireMatrix.UnrotateVector(dVector(0.0f, 0.0f, 1.0f, 0.0f)));
		NewtonCollisionSupportVertex(collision, &widthDir[0], &extremePoint[0]);
		width = widthDir.DotProduct3(extremePoint);

		widthDir = widthDir.Scale(-1.0f);
		NewtonCollisionSupportVertex(collision, &widthDir[0], &extremePoint[0]);
		width += widthDir.DotProduct3(extremePoint);

		// destroy the auxiliary collision
		NewtonDestroyCollision(collision);
	}

	NewtonCollision* CreateChassisCollision(const DemoEntity* const carEntity, NewtonWorld* const world) const
	{
		DemoEntity* const chassis = carEntity->Find("car_body");
		dAssert(chassis);

		DemoMesh* const mesh = (DemoMesh*)chassis->GetMesh();
		dAssert(mesh->IsType(DemoMesh::GetRttiType()));
		const dMatrix& meshMatrix = chassis->GetMeshMatrix();

		dArray<dFloat> temp(mesh->m_vertexCount * 3);
		meshMatrix.TransformTriplex(&temp[0], 3 * sizeof (dFloat), mesh->m_vertex, 3 * sizeof (dFloat), mesh->m_vertexCount);
		NewtonCollision* const shape = NewtonCreateConvexHull(world, mesh->m_vertexCount, &temp[0], 3 * sizeof (dFloat), 0.001f, 0, NULL);
		return shape;
	}

	DemoEntity* const LoadModel_ndg(const char* const modelName)
	{
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(GetWorld());
		DemoEntity* const entity = DemoEntity::LoadNGD_mesh(modelName, GetWorld(), scene->GetShaderCache());
		return entity;
	}

	DemoEntity* const LoadVisualModel (const char* const modelName)
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);
		DemoEntity* const entity = LoadModel_ndg(modelName);
		dAssert (entity);
		scene->Append(entity);
		return entity;
	}

	dVehicleTireInterface* AddTire(dVehicleChassis* const vehicle, const char* const tireName, dFloat width, dFloat radius, dFloat vehicleMass)
	{
		DemoEntity* const entity = (DemoEntity*)vehicle->GetVehicle()->GetUserData();
		DemoEntity* const tirePart = entity->Find(tireName);

		// for simplicity, tires are position in global space
		dMatrix tireMatrix(tirePart->CalculateGlobalMatrix());

		// add the offset to the tire position to account for suspension span
		//tireMatrix.m_posit += m_controller->GetUpAxis().Scale (definition.m_tirePivotOffset);
		//tireMatrix.m_posit -= vehicle->GetUpAxis().Scale(0.0f);

		// add the tire to the vehicle
		dVehicleTireInterface::dTireInfo tireInfo;
		tireInfo.m_mass = 40.0f;
		tireInfo.m_radio = radius;
		tireInfo.m_width = width;
		tireInfo.m_pivotOffset = 0.01f;
		tireInfo.m_steerRate = 0.5f * dPi;
		tireInfo.m_frictionCoefficient = 0.8f;
		tireInfo.m_maxSteeringAngle = 40.0f * dDegreeToRad;
		
		tireInfo.m_suspensionLength = 0.22f;
		tireInfo.m_dampingRatio = 15.0f * vehicleMass;
		tireInfo.m_springStiffness = dAbs(vehicleMass * DEMO_GRAVITY * 8.0f / tireInfo.m_suspensionLength);

		tireInfo.m_corneringStiffness = dAbs(vehicleMass * DEMO_GRAVITY * 0.5f);
		tireInfo.m_longitudinalStiffness = dAbs(vehicleMass * DEMO_GRAVITY * 0.25f);

		//tireInfo.m_aligningMomentTrail = definition.m_tireAligningMomemtTrail;
		//tireInfo.m_hasFender = definition.m_wheelHasCollisionFenders;
		//tireInfo.m_suspentionType = definition.m_tireSuspensionType;

		dVehicleTireInterface* const tire = vehicle->AddTire(tireMatrix, tireInfo);
		tire->SetUserData(tirePart);
		return tire;
	}
	
	dVehicleChassis* CreateVehicle(const char* const carModelName, const dMatrix& location)
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		// load vehicle visual mesh
		DemoEntity* const vehicleEntity = LoadVisualModel (carModelName);

		// set entity world location
		vehicleEntity->ResetMatrix(*scene, location);

		// make chassis collision shape;
		NewtonCollision* const chassisCollision = CreateChassisCollision(vehicleEntity, world);

		// create the vehicle controller
		dMatrix chassisMatrix;
#if 1
		chassisMatrix.m_front = dVector(1.0f, 0.0f, 0.0f, 0.0f);			// this is the vehicle direction of travel
#else
		chassisMatrix.m_front = dVector(0.0f, 0.0f, 1.0f, 0.0f);			// this is the vehicle direction of travel
#endif
		chassisMatrix.m_up = dVector(0.0f, 1.0f, 0.0f, 0.0f);			// this is the downward vehicle direction
		chassisMatrix.m_right = chassisMatrix.m_front.CrossProduct(chassisMatrix.m_up);	// this is in the side vehicle direction (the plane of the wheels)
		chassisMatrix.m_posit = dVector(0.0f, 0.0f, 0.0f, 1.0f);

		// create a single body vehicle 
		dFloat chassisMass = 1000.0f;
		dVehicleChassis* const vehicle = CreateSingleBodyVehicle(chassisCollision, chassisMatrix, chassisMass, PhysicsApplyGravityForce, DEMO_GRAVITY);

		// save the vehicle chassis with the vehicle visual for update children matrices 
		VehicleUserData* const renderCallback = new VehicleUserData(vehicle);
		vehicleEntity->SetUserData(renderCallback);

		// get the inteface and assig a user data;
		dVehicleInterface* const vehicleRoot = vehicle->GetVehicle();
		vehicleRoot->SetUserData(vehicleEntity);

		// get body from player
		NewtonBody* const chassisBody = vehicle->GetBody();

		// set the player matrix 
		NewtonBodySetMatrix(chassisBody, &location[0][0]);

		// set the transform callback
		NewtonBodySetUserData(chassisBody, vehicleEntity);
		NewtonBodySetTransformCallback(chassisBody, DemoEntity::TransformCallback);

//		for (int i = 0; i < int((sizeof(m_gearMap) / sizeof(m_gearMap[0]))); i++) {
//			m_gearMap[i] = i;
//		}

		// destroy chassis collision shape 
		NewtonDestroyCollision(chassisCollision);


		// add Tires
		dFloat width;
		dFloat radio;
		CalculateTireDimensions ("fl_tire", width, radio, world, vehicleEntity);
		dVehicleTireInterface* const frontLeft = AddTire(vehicle, "fl_tire", width, radio, chassisMass);
		dVehicleTireInterface* const frontRight = AddTire(vehicle, "fr_tire", width, radio, chassisMass);

		CalculateTireDimensions ("rl_tire", width, radio, world, vehicleEntity);
		dVehicleTireInterface* const rearLeft = AddTire(vehicle, "rl_tire", width, radio, chassisMass);
		dVehicleTireInterface* const rearRight = AddTire(vehicle, "rr_tire", width, radio, chassisMass);
return NULL;

		// add vehicle steering control 
		dVehicleSteeringControl* const steeringControl = vehicle->GetSteeringControl();
		steeringControl->AddTire(frontLeft);
		steeringControl->AddTire(frontRight);

		// add vehicle hand brake control 
		dVehicleBrakeControl* const handBrakeControl = vehicle->GetHandBrakeControl();
		handBrakeControl->SetBrakeTorque(1000.0f);
		handBrakeControl->AddTire(rearLeft);
		handBrakeControl->AddTire(rearRight);

		// add vehicle brake control 
		dVehicleBrakeControl* const brakeControl = vehicle->GetBrakeControl();
		brakeControl->SetBrakeTorque(1000.0f);
		brakeControl->AddTire(frontLeft);
		brakeControl->AddTire(frontRight);
		brakeControl->AddTire(rearLeft);
		brakeControl->AddTire(rearRight);

		// add a differential 
		dVehicleDifferentialInterface* const differential = vehicle->AddDifferential(rearLeft, rearRight);

		// add and internal combustion engine
		dVehicleEngineInterface::dEngineInfo engineInfo;
		engineInfo.m_mass = 50.0f;
		engineInfo.m_armatureRadius = 0.125f;
		engineInfo.m_idleTorque = 200.0f;			// IDLE_TORQUE
		engineInfo.m_rpmAtIdleTorque = 450.0f;		// IDLE_TORQUE_RPM
		engineInfo.m_peakTorque = 500.0f;			// PEAK_TORQUE
		engineInfo.m_rpmAtPeakTorque = 3000.0f;		// PEAK_TORQUE_RPM
		engineInfo.m_peakHorsePower = 400.0f;		// PEAK_HP
		engineInfo.m_rpmAtPeakHorsePower = 5200.0f;	// PEAK_HP_RPM
		engineInfo.m_rpmAtRedLine = 6000.0f;		// REDLINE_TORQUE_RPM

		engineInfo.m_crownGear = 4.0f;
		engineInfo.m_clutchTorque = 600.0f;

		engineInfo.m_gearRatios[dVehicleEngineInterface::m_reverseGear] = -2.90f;	// reverse
		engineInfo.m_gearRatios[dVehicleEngineInterface::m_neutralGear] = 0.0f;     // neutral
		engineInfo.m_gearRatios[dVehicleEngineInterface::m_firstGear + 0] = 2.66f;  // GEAR_1
		engineInfo.m_gearRatios[dVehicleEngineInterface::m_firstGear + 1] = 1.78f;	// GEAR_2
		engineInfo.m_gearRatios[dVehicleEngineInterface::m_firstGear + 2] = 1.30f;	// GEAR_3
		engineInfo.m_gearRatios[dVehicleEngineInterface::m_firstGear + 3] = 1.00f;	// GEAR_4
		engineInfo.m_gearRatios[dVehicleEngineInterface::m_firstGear + 4] = 0.74f;	// GEAR_5
		engineInfo.m_gearRatios[dVehicleEngineInterface::m_firstGear + 5] = 0.50f;	// GEAR_6
		engineInfo.m_gearsCount = 8;

		dVehicleEngineInterface* const engine = vehicle->AddEngine(engineInfo, differential);
		// Set Engine Control
		dVehicleEngineControl* const engineControl = vehicle->GetEngineControl();
		engineControl->SetEngine(engine);

		// do not forget to call finalize after all components are added or after any change is made to the vehicle
		vehicle->Finalize();
		
		return vehicle;
	}

	void UpdateDriverInput(dVehicleChassis* const vehicle, dFloat timestep) 
	{
//		dVehicleSteeringControl* const steeringControl = vehicle->GetSteeringControl();

		NewtonBody* const body = vehicle->GetBody();
		NewtonWorld* const world = NewtonBodyGetWorld(body);
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		//dEngineController* const engine = vehicle->GetEngine();
		//int gear = engine ? engine->GetGear() : 0;

		dVehicleChassis::dDriverInput driverInput;

		dFloat axis[32];
		int axisCount = scene->GetJoystickAxis(axis);
axisCount = 0;
		if (axisCount) {
			//dAssert (0);
/*
			dFloat joyPosX;
			dFloat joyPosY;
			dFloat joyPosZ;

			char buttons[32];
			scene->GetJoystickButtons(buttons);

			joyPosX = axis[m_steeringAxis];
			joyPosY = -axis[m_throttleAxis];
			joyPosZ = dMax(axis[m_clutchAxis], dFloat(0.0f));
			bool ignitionButton = buttons[m_ignitionButton] ? true : false;
			bool handBreakButton = buttons[m_handBrakeButton] ? true : false;
			bool gearUpButton = buttons[m_gearUpButton] ? true : false;
			bool gearDownButton = buttons[m_gearDonwButton] ? true : false;

			gear += (int(m_gearUpKey.UpdateTrigger(gearUpButton)) - int(m_gearDownKey.UpdateTrigger(gearDownButton)));

			driverInput.m_clutchPedal = joyPosZ * joyPosZ * joyPosZ;
			driverInput.m_steeringValue = joyPosX * joyPosX * joyPosX;
			driverInput.m_throttle = joyPosY > 0.0f ? dAbs(joyPosY * joyPosY * joyPosY) : 0.0f;
			driverInput.m_brakePedal = joyPosY < 0.0f ? dAbs(joyPosY * joyPosY * joyPosY) : 0.0f;

			driverInput.m_ignitionKey = m_engineKeySwitch.UpdatePushButton(ignitionButton);
			driverInput.m_handBrakeValue = handBreakButton ? 1.0f : 0.0f;
			driverInput.m_gear = gear;

			//for (int i = 0; i < joyButtons; i ++) {
			//	dTrace(("%d ", buttons[i]));
			//}
			//dTrace(("\n"));
			//dTrace (("%f %f %f %f\n", driverInput.m_steeringValue, driverInput.m_throttle, driverInput.m_brakePedal, driverInput.m_clutchPedal));
			//dTrace (("%d %d %d\n", gear, ignitionButton, m_engineKeySwitch.GetPushButtonState()));
*/
		} else {
			driverInput.m_throttle = scene->GetKeyState('W') ? 1.0f : 0.0f;
			//driverInput.m_throttle = scene->GetKeyState('W') ? 0.5f : 0.0f;
			driverInput.m_clutchPedal = scene->GetKeyState('K') ? 0.0f : 1.0f;
			driverInput.m_steeringValue = (dFloat(scene->GetKeyState('A')) - dFloat(scene->GetKeyState('D')));
			driverInput.m_brakePedal = scene->GetKeyState('S') ? 1.0f : 0.0f;
			driverInput.m_handBrakeValue = scene->GetKeyState(' ') ? 1.0f : 0.0f;

			//driverInput.m_ignitionKey = m_engineKeySwitch.UpdatePushButton(scene->GetKeyState('I'));
			////driverInput.m_manualTransmission = !m_automaticTransmission.UpdatePushButton (scene, 0x0d);
			//gear += m_gearUpKey.UpdateTrigger(scene->GetKeyState('M')) - m_gearUpKey.UpdateTrigger(scene->GetKeyState('N'));
			//driverInput.m_gear = gear;
			//driverInput.m_lockDifferential = m_engineDifferentialLock.UpdatePushButton(scene, 'L');
		}

		driverInput.m_clutchPedal = 0.0f;
		//xxxxxx
#if 0
	#if 0
		static FILE* file = fopen("log.bin", "wb");
		if (file) {
			fwrite(&driverInput, sizeof(dVehicleDriverInput), 1, file);
			fflush(file);
		}
	#else 
		static FILE* file = fopen("log.bin", "rb");
		if (file) {
			fread(&driverInput, sizeof(dVehicleDriverInput), 1, file);
		}
	#endif
#endif

		vehicle->ApplyDriverInputs(driverInput, timestep);
	}

	dVehicleChassis* m_player;
	GLuint m_gears;
	GLuint m_odometer;
	GLuint m_redNeedle;
	GLuint m_tachometer;
	GLuint m_greenNeedle;

	bool m_externalView;
};


void SingleBodyCar(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	CreateLevelMesh (scene, "flatPlane.ngd", 1);
//	CreateHeightFieldTerrain (scene, 10, 8.0f, 5.0f, 0.2f, 200.0f, -50.0f);
//	AddPrimitiveArray (scene, 0.0f, dVector (0.0f, 0.0f, 0.0f, 0.0f), dVector (100.0f, 1.0f, 100.0f, 0.0f), 1, 1, 0, _BOX_PRIMITIVE, 0, dGetIdentityMatrix());


	dMatrix location (dGetIdentityMatrix());
	location.m_posit = dVector (0.0f, 10.0f, 0.0f, 1.0f);

	location.m_posit = FindFloor (scene->GetNewton(), location.m_posit, 100.0f);
	location.m_posit.m_y += 0.5f;

	NewtonWorld* const world = scene->GetNewton();

	// create a vehicle controller manager
	SingleBodyVehicleManager* const manager = new SingleBodyVehicleManager(world);
	
	// load 
	dVehicleChassis* const player = manager->CreateVehicle("viper.ngd", location);

	// set this vehicle as the player
//	manager->SetAsPlayer(player);

	int count = 5;
	count = 0;
	for (int i = 0; i < count; i++) {
		for (int j = 0; j < count; j++) {
			//dVector offset(j * 4.0f, 0.0f, i * 4.0f + 4.0f, 0.0f);
			dMatrix offset(location);
			offset.m_posit += dVector (j * 4.0f + 4.0f, 0.0f, i * 4.0f, 0.0f);
			manager->CreateVehicle("viper.ngd", offset);
		}
	}

/*
	DemoEntity* const vehicleEntity = (DemoEntity*)NewtonBodyGetUserData(player->GetBody());
	dMatrix camMatrix (vehicleEntity->GetNextMatrix());
	//	scene->SetCameraMouseLock (true);


//
	//	dVector location (origin);
	//	location.m_x += 20.0f;
	//	location.m_z += 20.0f;
//	location.m_posit.m_z += 4.0f;

//	int count = 1;
	dMatrix shapeOffsetMatrix (dGetIdentityMatrix());
//	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (scene->GetNewton());

	dVector size (3.0f, 0.125f, 3.0f, 0.0f);
	//AddPrimitiveArray(scene, 100.0f, location.m_posit, size, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);

	size = dVector(1.0f, 0.5f, 1.0f, 0.0f);
	//	AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 5.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	//	AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	//	AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 5.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	//	AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 5.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	//	AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 5.0f, _CHAMFER_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	//	AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 5.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	//	AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 5.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	//	AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 5.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);

	//	NewtonSerializeToFile (scene->GetNewton(), "C:/Users/Julio/Desktop/newton-dynamics/applications/media/xxxxx.bin");
*/		

//	camMatrix.m_posit.m_x -= 5.0f;
//	scene->SetCameraMatrix(camMatrix, camMatrix.m_posit);

	dQuaternion rot;
	dVector origin(-10.0f, 2.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}

