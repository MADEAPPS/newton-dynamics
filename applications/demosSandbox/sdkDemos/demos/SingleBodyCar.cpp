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

struct TIRE_DATA
{
	dFloat m_mass;
	dFloat m_pivotOffset;
	dFloat m_steerRate;
	dFloat m_frictionCoefficient;
	dFloat m_suspensionLength;
	dFloat m_dampingRatio;
	dFloat m_springStiffness;
	dFloat m_corneringStiffness;
	dFloat m_longitudinalStiffness;
};


#define VIPER_CHASSIS_MASS	1200.0f
#define VIPER_TIRE_MASS		30.0f
#define VIPER_DIFF_MASS		30.0f
#define VIPER_ENGINE_MASS	40.0f
#define VIPER_DIFF_RADIUS	0.25f
#define VIPER_ENGINE_RADIUS 0.25f

static TIRE_DATA viperTire 
{
	VIPER_TIRE_MASS,	//m_mass;
	0.01f,				//m_pivotOffset;
	0.5f,				//m_steerRate;
	1.0f,				//m_frictionCoefficient
	0.22f,				//m_suspensionLength
	15.0f,				//m_dampingRatio
	8.0f,				//m_springStiffness
	20.0f,				//m_corneringStiffness
	2.0f,				//m_longitudinalStiffness
};


#define MONSTER_TRUCK_CHASSIS_MASS	1200.0f
#define MONSTER_TRUCK_TIRE_MASS		100.0f
#define MONSTER_TRUCK_DIFF_MASS		30.0f
#define MONSTER_TRUCK_ENGINE_MASS	40.0f
#define MONSTER_TRUCK_DIFF_RADIUS	0.25f
#define MONSTER_TRUCK_ENGINE_RADIUS 0.25f

static TIRE_DATA monsterTruckTire
{
	MONSTER_TRUCK_TIRE_MASS,	//m_mass;
	0.01f,						//m_pivotOffset;
	0.5f,						//m_steerRate;
	1.35f,						//m_frictionCoefficient
	0.5f,						//m_suspensionLength
	10.0f,						//m_dampingRatio
	6.0f,						//m_springStiffness
	20.0f,						//m_corneringStiffness
	2.0f,						//m_longitudinalStiffness
};


class SingleBodyVehicleManager: public dVehicleManager
{
	public:
	SingleBodyVehicleManager(NewtonWorld* const world)
		:dVehicleManager(world)
		,m_player(NULL)
		,m_externalView(true)
		,m_differentialMode(0)
	{
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);
		scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
		scene->Set2DDisplayRenderFunction(RenderHelpMenu, RenderUI, this);
		
		//// load 2d display assets
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
		SingleBodyVehicleManager* const me = (SingleBodyVehicleManager*)context;
		me->DrawHelp(scene);
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

	void DrawHelp(DemoEntityManager* const scene) const
	{
		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print(color, "Vehicle driving keyboard control");
		//scene->Print(color, "key switch          : 'i'");
		scene->Print(color, "accelerator         : 'w'");
		scene->Print(color, "reverse             : 's'");
		scene->Print(color, "turn left           : 'a'");
		scene->Print(color, "turn right          : 'd'");
		//scene->Print(color, "engage clutch       : 'k'");
		scene->Print(color, "hand brakes         : 'space'");

		ImGui::Separator();
		scene->Print(color, "differential type");
		ImGui::RadioButton("awd",	&m_differentialMode, 0);
		ImGui::RadioButton("free",	&m_differentialMode, 1);
		ImGui::RadioButton("rwd",	&m_differentialMode, 2); 
		ImGui::RadioButton("fwd",	&m_differentialMode, 3); 

		ImGui::Separator();
		scene->Print(color, "hide help           : 'h'");
	}

	void RenderUI(DemoEntityManager* const scene)
	{
		// set to transparent color
		if (m_player) {
			dVehicleEngine* const engine = m_player->GetEngineControl() ? m_player->GetEngineControl()->GetEngine() : NULL;
			
			glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
			dFloat gageSize = 200.0f;
			dFloat y = scene->GetHeight() - (gageSize / 2.0f + 20.0f);

			// draw the tachometer
			dFloat x = gageSize / 2 + 20.0f;
			dFloat rpm = engine ? engine->GetRpm() / engine->GetRedLineRpm() : 0.0f;
			DrawGage(m_tachometer, m_redNeedle, rpm, x, y, gageSize, -180.0f, 90.0f);

			// draw the odometer
			x += gageSize;
			dFloat speed = engine ? dAbs(engine->GetSpeed()) / engine->GetTopSpeed() : 0.0f;
			DrawGage(m_odometer, m_greenNeedle, speed, x, y, gageSize, -180.0f, 90.0f);

			// draw the current gear
			//int gear = engine->GetGear();
			//int gear = 1;
			//VehicleUserData* const playerData = (VehicleUserData*)playerEnt->GetUserData();
			//DrawGear(speed, x, y + 98, playerData->m_gearMap[gear], gageSize);
		}
	}

	void SetAsPlayer(dVehicle* const player)
	{
		//dEngineController* const engine = player->m_controller->GetEngine();
		//if (engine) {
		//	engine->SetIgnition(false);
		//}
		m_player = player->GetAsVehicleMultiBody();
	}

	void UpdateCamera(dFloat timestep)
	{
return;
		if (!m_player) {
			return;
		}

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

	dVehicleTire* AddTire(dVehicleMultiBody* const vehicle, const char* const tireName, dFloat steeringAngle, dFloat width, dFloat radius, dFloat vehicleMass, const TIRE_DATA& data)
	{
		DemoEntity* const entity = (DemoEntity*)vehicle->GetUserData();
		DemoEntity* const tirePart = entity->Find(tireName);

		// for simplicity, tires are position in global space
		dMatrix tireMatrix(tirePart->CalculateGlobalMatrix());

		// add the offset to the tire position to account for suspension span
		//tireMatrix.m_posit += m_controller->GetUpAxis().Scale (definition.m_tirePivotOffset);
		//tireMatrix.m_posit -= vehicle->GetUpAxis().Scale(0.0f);

		// add the tire to the vehicle
		dTireInfo tireInfo;
		tireInfo.m_mass = data.m_mass;
		tireInfo.m_radio = radius;
		// for debugging
		//tireInfo.m_radio *= 1.25f; 
		tireInfo.m_width = width;
		tireInfo.m_pivotOffset = data.m_pivotOffset;
		tireInfo.m_steerRate = data.m_steerRate * dPi;
		tireInfo.m_frictionCoefficient = data.m_frictionCoefficient;
		tireInfo.m_maxSteeringAngle = steeringAngle * dDegreeToRad;
		tireInfo.m_suspensionLength = data.m_suspensionLength;
		tireInfo.m_dampingRatio = data.m_dampingRatio * vehicleMass;
		tireInfo.m_springStiffness = data.m_springStiffness * dAbs(vehicleMass * DEMO_GRAVITY / tireInfo.m_suspensionLength);
		tireInfo.m_corneringStiffness = data.m_corneringStiffness * dAbs(vehicleMass * DEMO_GRAVITY);
		tireInfo.m_longitudinalStiffness = data.m_longitudinalStiffness * dAbs(vehicleMass * DEMO_GRAVITY);

		//tireInfo.m_aligningMomentTrail = definition.m_tireAligningMomemtTrail;
		//tireInfo.m_suspentionType = definition.m_tireSuspensionType;

		dVehicleTire* const tire = vehicle->AddTire(tireMatrix, tireInfo);
		tire->SetUserData(tirePart);
		return tire;
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
		dArray<dVector> temp(tireMesh->m_vertexCount);
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

	NewtonBody* CreateChassisBody(dFloat mass, DemoEntity* const carEntity) const
	{
		dAssert(carEntity->Find("car_body"));
		dAssert(carEntity->Find("car_body") == carEntity);

		NewtonCollision* const shape = carEntity->CreateCollisionFromchildren(GetWorld());

		// create a body and call the low level init function
		dMatrix locationMatrix(dGetIdentityMatrix());
		NewtonBody* const body = NewtonCreateDynamicBody(GetWorld(), shape, &locationMatrix[0][0]);

		NewtonBodySetUserData(body, carEntity);
		//NewtonBodySetTransformCallback(body, DemoEntity::TransformCallback);
		NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);

		// set vehicle mass, inertia and center of mass
		NewtonBodySetMassProperties(body, mass, shape);

		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;
		NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);
		Ixx *= 1.5f;
		Iyy *= 1.5f; 
		Izz *= 1.5f;
		NewtonBodySetMassMatrix(body, mass, Ixx, Iyy, Izz);
		NewtonDestroyCollision(shape);
		return body;
	}

	dVehicle* CreateSportCar(const dMatrix& location, const DemoEntity* const entity)
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		DemoEntity* const vehicleEntity = (DemoEntity*)entity->CreateClone();
		scene->Append(vehicleEntity);

		// set entity world location
		vehicleEntity->ResetMatrix(*scene, location);

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
		NewtonBody* const chassisBody = CreateChassisBody(VIPER_CHASSIS_MASS, vehicleEntity);

		// make the vehicle a little over steering by shitting the com to the front
		dVector com(0.0f);
		NewtonBodyGetCentreOfMass(chassisBody, &com[0]);
		com += chassisMatrix.m_front.Scale (0.3f);
		com += chassisMatrix.m_up.Scale(-0.2f);
		NewtonBodySetCentreOfMass(chassisBody, &com[0]);

		// set the player matrix 
		NewtonBodySetMatrix(chassisBody, &location[0][0]);

		dVehicleMultiBody* const vehicle = new dVehicleMultiBody (chassisBody, chassisMatrix, DEMO_GRAVITY);
		AddRoot(vehicle);

		// save entity as use data
		vehicle->SetUserData(vehicleEntity);

		// save the vehicle chassis with the vehicle visual for update children matrices 
		//VehicleUserData* const renderCallback = new VehicleUserData(vehicle);
		//vehicleEntity->SetUserData(renderCallback);

		//for (int i = 0; i < int((sizeof(m_gearMap) / sizeof(m_gearMap[0]))); i++) {
		//	m_gearMap[i] = i;
		//}

		// add Tires
		dFloat width;
		dFloat radio;
		CalculateTireDimensions ("fl_tire", width, radio, world, vehicleEntity);
		dVehicleTire* const frontLeft = AddTire(vehicle, "fl_tire", 35.0f, width, radio, VIPER_CHASSIS_MASS, viperTire);
		dVehicleTire* const frontRight = AddTire(vehicle, "fr_tire", 35.0f, width, radio, VIPER_CHASSIS_MASS, viperTire);

		CalculateTireDimensions ("rl_tire", width, radio, world, vehicleEntity);
		dVehicleTire* const rearLeft = AddTire(vehicle, "rl_tire", 0.0f, width, radio, VIPER_CHASSIS_MASS, viperTire);
		dVehicleTire* const rearRight = AddTire(vehicle, "rr_tire", 0.0f, width, radio, VIPER_CHASSIS_MASS, viperTire);

		// add vehicle steering control 
		dVehicleSteeringControl* const steeringControl = vehicle->GetSteeringControl();
		steeringControl->AddTire(frontLeft);
		steeringControl->AddTire(frontRight);
		steeringControl->SetParam(1.0f);

		// add vehicle hand brake control 
		dVehicleBrakeControl* const handBrakeControl = vehicle->GetHandBrakeControl();
		handBrakeControl->SetBrakeTorque(2000.0f);
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
		dVehicleDifferential* const differential = vehicle->AddDifferential(VIPER_DIFF_MASS, VIPER_DIFF_RADIUS, rearLeft, rearRight);

		// add and internal combustion engine
		dEngineInfo engineInfo;
		engineInfo.m_mass = VIPER_ENGINE_MASS;
		engineInfo.m_armatureRadius = VIPER_ENGINE_RADIUS;
		engineInfo.m_idleTorque = 200.0f;			// IDLE_TORQUE
		engineInfo.m_rpmAtIdleTorque = 450.0f;		// IDLE_TORQUE_RPM
		engineInfo.m_peakTorque = 500.0f;			// PEAK_TORQUE
		engineInfo.m_rpmAtPeakTorque = 3000.0f;		// PEAK_TORQUE_RPM
		engineInfo.m_peakHorsePower = 400.0f;		// PEAK_HP
		engineInfo.m_rpmAtPeakHorsePower = 5200.0f;	// PEAK_HP_RPM
		engineInfo.m_rpmAtRedLine = 6000.0f;		// REDLINE_TORQUE_RPM

		engineInfo.m_crownGear = 4.0f;
		engineInfo.m_clutchTorque = 600.0f;

		engineInfo.m_topSpeedInMetersPerSeconds = 100.0f;

		engineInfo.m_gearRatios[dEngineInfo::m_reverseGear] = -2.90f;	// reverse
		engineInfo.m_gearRatios[dEngineInfo::m_neutralGear] = 0.0f;     // neutral
		engineInfo.m_gearRatios[dEngineInfo::m_firstGear + 0] = 2.66f;  // GEAR_1
		engineInfo.m_gearRatios[dEngineInfo::m_firstGear + 1] = 1.78f;	// GEAR_2
		engineInfo.m_gearRatios[dEngineInfo::m_firstGear + 2] = 1.30f;	// GEAR_3
		engineInfo.m_gearRatios[dEngineInfo::m_firstGear + 3] = 1.00f;	// GEAR_4
		engineInfo.m_gearRatios[dEngineInfo::m_firstGear + 4] = 0.74f;	// GEAR_5
		engineInfo.m_gearRatios[dEngineInfo::m_firstGear + 5] = 0.50f;	// GEAR_6
		engineInfo.m_gearsCount = 8;


		// Set Engine and engine control
		dVehicleEngine* const engine = vehicle->AddEngine(engineInfo, differential);
		dVehicleEngineControl* const engineControl = vehicle->GetEngineControl();
		engineControl->SetEngine(engine);

		// do not forget to call finalize after all components are added or after any change is made to the vehicle
		vehicle->Finalize();

		return vehicle;
	}

	dVehicle* CreateOffRoadCar(const dMatrix& location, const DemoEntity* const entity)
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		DemoEntity* const vehicleEntity = (DemoEntity*)entity->CreateClone();
		scene->Append(vehicleEntity);

		// set entity world location
		vehicleEntity->ResetMatrix(*scene, location);

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
		NewtonBody* const chassisBody = CreateChassisBody(MONSTER_TRUCK_CHASSIS_MASS, vehicleEntity);

		// make the vehicle a little over steering by shitting the com to the front
		dVector com(0.0f);
		NewtonBodyGetCentreOfMass(chassisBody, &com[0]);
		com += chassisMatrix.m_front.Scale(-0.0f);
		com += chassisMatrix.m_up.Scale(-0.5f);
		NewtonBodySetCentreOfMass(chassisBody, &com[0]);

		// set the player matrix 
		NewtonBodySetMatrix(chassisBody, &location[0][0]);

		dVehicleMultiBody* const vehicle = new dVehicleMultiBody(chassisBody, chassisMatrix, DEMO_GRAVITY);
		AddRoot(vehicle);

		// save entity as use data
		vehicle->SetUserData(vehicleEntity);

		// save the vehicle chassis with the vehicle visual for update children matrices 
		//VehicleUserData* const renderCallback = new VehicleUserData(vehicle);
		//vehicleEntity->SetUserData(renderCallback);

		//for (int i = 0; i < int((sizeof(m_gearMap) / sizeof(m_gearMap[0]))); i++) {
		//	m_gearMap[i] = i;
		//}

		// add Tires
		dFloat width;
		dFloat radio;
		CalculateTireDimensions("fl_tire", width, radio, world, vehicleEntity);
		dVehicleTire* const frontLeft = AddTire(vehicle, "fl_tire", 30.0f, width, radio, MONSTER_TRUCK_CHASSIS_MASS, monsterTruckTire);
		dVehicleTire* const frontRight = AddTire(vehicle, "fr_tire", 30.0f, width, radio, MONSTER_TRUCK_CHASSIS_MASS, monsterTruckTire);

		CalculateTireDimensions("rl_tire", width, radio, world, vehicleEntity);
		dVehicleTire* const rearLeft = AddTire(vehicle, "rl_tire", -10.0f, width, radio, MONSTER_TRUCK_CHASSIS_MASS, monsterTruckTire);
		dVehicleTire* const rearRight = AddTire(vehicle, "rr_tire", -10.0f, width, radio, MONSTER_TRUCK_CHASSIS_MASS, monsterTruckTire);

		// add vehicle steering control 
		dVehicleSteeringControl* const steeringControl = vehicle->GetSteeringControl();
		steeringControl->AddTire(frontLeft);
		steeringControl->AddTire(frontRight);
		//steeringControl->AddTire(rearLeft);
		//steeringControl->AddTire(rearRight);
		steeringControl->SetParam(1.0f);

		// add vehicle hand brake control 
		dVehicleBrakeControl* const handBrakeControl = vehicle->GetHandBrakeControl();
		handBrakeControl->SetBrakeTorque(2000.0f);
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
		dVehicleDifferential* const frontDifferential = vehicle->AddDifferential(MONSTER_TRUCK_DIFF_MASS, MONSTER_TRUCK_DIFF_RADIUS, frontLeft, frontRight);
		dVehicleDifferential* const rearDifferential = vehicle->AddDifferential(MONSTER_TRUCK_DIFF_MASS, MONSTER_TRUCK_DIFF_RADIUS, rearLeft, rearRight);
		dVehicleDifferential* const differential = vehicle->AddDifferential(MONSTER_TRUCK_DIFF_MASS, MONSTER_TRUCK_DIFF_RADIUS, frontDifferential, rearDifferential);
		//dVehicleDifferential* const differential = vehicle->AddDifferential(MONSTER_TRUCK_DIFF_MASS, MONSTER_TRUCK_DIFF_RADIUS, frontLeft, frontRight);

		// add and internal combustion engine
		dEngineInfo engineInfo;
		engineInfo.m_mass = MONSTER_TRUCK_ENGINE_MASS;
		engineInfo.m_armatureRadius = MONSTER_TRUCK_ENGINE_RADIUS;
		engineInfo.m_idleTorque = 300.0f * 1.5f;			// IDLE_TORQUE
		engineInfo.m_peakTorque = 500.0f * 1.5f;			// PEAK_TORQUE
		engineInfo.m_peakHorsePower = 400.0f * 1.5f;		// PEAK_HP

		engineInfo.m_rpmAtIdleTorque = 450.0f;		// IDLE_TORQUE_RPM
		engineInfo.m_rpmAtPeakTorque = 3000.0f;		// PEAK_TORQUE_RPM
		engineInfo.m_rpmAtPeakHorsePower = 5200.0f;	// PEAK_HP_RPM
		engineInfo.m_rpmAtRedLine = 6000.0f;		// REDLINE_TORQUE_RPM

		engineInfo.m_crownGear = 4.0f;
		engineInfo.m_clutchTorque = 600.0f;

		engineInfo.m_topSpeedInMetersPerSeconds = 100.0f;

		engineInfo.m_gearRatios[dEngineInfo::m_reverseGear] = -2.90f;	// reverse
		engineInfo.m_gearRatios[dEngineInfo::m_neutralGear] = 0.0f;     // neutral
		engineInfo.m_gearRatios[dEngineInfo::m_firstGear + 0] = 2.66f;  // GEAR_1
		engineInfo.m_gearRatios[dEngineInfo::m_firstGear + 1] = 1.78f;	// GEAR_2
		engineInfo.m_gearRatios[dEngineInfo::m_firstGear + 2] = 1.30f;	// GEAR_3
		engineInfo.m_gearRatios[dEngineInfo::m_firstGear + 3] = 1.00f;	// GEAR_4
		engineInfo.m_gearRatios[dEngineInfo::m_firstGear + 4] = 0.74f;	// GEAR_5
		engineInfo.m_gearRatios[dEngineInfo::m_firstGear + 5] = 0.50f;	// GEAR_6
		engineInfo.m_gearsCount = 8;

		// Set Engine and engine control
		dVehicleEngine* const engine = vehicle->AddEngine(engineInfo, differential);
		dVehicleEngineControl* const engineControl = vehicle->GetEngineControl();
		engineControl->SetEngine(engine);

		// do not forget to call finalize after all components are added or after any change is made to the vehicle
		vehicle->Finalize();

		return vehicle;
	}

	void UpdateDriverInput(dVehicle* const vehicle, dFloat timestep) 
	{
		if (vehicle != m_player) {
			return;
		}

		NewtonBody* const body = vehicle->GetBody();
		NewtonWorld* const world = NewtonBodyGetWorld(body);
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		dVehicleMultiBody::dDriverInput driverInput;

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
			driverInput.m_throttle = scene->GetKeyState('W') ? 0.85f : 0.0f;
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

		driverInput.m_differentialMode = m_differentialMode;

		driverInput.m_clutchPedal = 0.0f;

#if 0
	#if 0
		static FILE* file = fopen("log.bin", "wb");
		if (file) {
			fwrite(&driverInput, sizeof(dVehicleMultiBody::dDriverInput), 1, file);
			fflush(file);
		}
	#else 
		static FILE* file = fopen("log.bin", "rb");
		if (file) {
			fread(&driverInput, sizeof(dVehicleMultiBody::dDriverInput), 1, file);
		}
	#endif
#endif

		vehicle->ApplyDriverInputs(driverInput, timestep);
	}


	void OnUpdateTransform(const dVehicle* const vehicle) const 
	{
		dMatrix matrix;
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(GetWorld());
		DemoEntity* const model = (DemoEntity*) NewtonBodyGetUserData(vehicle->GetBody());
		
		NewtonBodyGetMatrix(vehicle->GetBody(), &matrix[0][0]);
		model->SetMatrix(*scene, dQuaternion (matrix), matrix.m_posit);
//dTrace(("%f %f %f\n", matrix.m_posit.m_x, matrix.m_posit.m_y, matrix.m_posit.m_z));

		dMatrix chassisMatrixInv(matrix.Inverse());
		const dList<dVehicleNode*>& children = vehicle->GetChildrenList();
		for (dList<dVehicleNode*>::dListNode* node = children.GetFirst(); node; node = node->GetNext()) {
			dVehicleTire* const tire = node->GetInfo()->GetAsTire();
			if (tire) {
				DemoEntity* const tireMesh = (DemoEntity*)tire->GetUserData();
				//dMatrix tireMatrix(tire->GetGlobalMatrix() * chassisMatrixInv);
				dMatrix tireMatrix(tire->GetLocalMatrix());
				dQuaternion rotation(tireMatrix);
				tireMesh->SetMatrixUsafe(rotation, tireMatrix.m_posit);
			}
		}
	}

	dVehicleMultiBody* m_player;
	GLuint m_gears;
	GLuint m_odometer;
	GLuint m_redNeedle;
	GLuint m_tachometer;
	GLuint m_greenNeedle;
	mutable int m_differentialMode;
	bool m_externalView;
};


static void CreateBridge(DemoEntityManager* const scene, NewtonBody* const playgroundBody)
{
	dVector p0(1.35f, 8.35f, -28.1f, 0.0f);
	dVector p1(1.35f, 8.40f, 28.9f, 0.0f);
	dVector p2(1.35f, 6.0f, 0.0, 0.0f);

	dFloat y[3];
	dFloat splineMatrix[3][3];

	y[0] = p0.m_y;
	splineMatrix[0][0] = p0.m_z * p0.m_z;
	splineMatrix[0][1] = p0.m_z;
	splineMatrix[0][2] = 1.0f;

	y[1] = p1.m_y;
	splineMatrix[1][0] = p1.m_z * p1.m_z;
	splineMatrix[1][1] = p1.m_z;
	splineMatrix[1][2] = 1.0f;

	y[2] = p2.m_y;
	splineMatrix[2][0] = p2.m_z * p2.m_z;
	splineMatrix[2][1] = p2.m_z;
	splineMatrix[2][2] = 1.0f;

	dSolveGaussian(3, &splineMatrix[0][0], y);

	dFloat plankLentgh = 3.0f;
	NewtonWorld* const world = scene->GetNewton();
	dVector size(8.0f, 0.5f, plankLentgh, 0.0f);
	NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, _BOX_PRIMITIVE, 0);
	DemoMesh* const geometry = new DemoMesh("primitive", scene->GetShaderCache(), collision, "wood_0.tga", "wood_0.tga", "wood_0.tga");

	int count = 0;
	dFloat mass = 50.0f;
	dFloat lenght = 0.0f;
	dFloat step = 1.0e-3f;
	dFloat y0 = y[0] * p0.m_z * p0.m_z + y[1] * p0.m_z + y[2];

	dVector q0(p0);
	NewtonBody* array[256];
	for (dFloat z = p0.m_z + step; z < p1.m_z; z += step) {
		dFloat y1 = y[0] * z * z + y[1] * z + y[2];
		dFloat y10 = y1 - y0;
		lenght += dSqrt(step * step + y10 * y10);
		if (lenght >= plankLentgh) {
			dVector q1(p0.m_x, y1, z, 0.0f);

			dMatrix matrix(dGetIdentityMatrix());
			matrix.m_posit = (q1 + q0).Scale(0.5f);
			matrix.m_posit.m_w = 1.0f;

			dVector right(q1 - q0);
			matrix.m_right = right.Normalize();
			matrix.m_up = matrix.m_right.CrossProduct(matrix.m_front);

			array[count] = CreateSimpleSolid(scene, geometry, mass, matrix, collision, 0);

			q0 = q1;
			lenght = 0.0f;
			count++;
		}

		y0 = y1;
	}

	dMatrix matrix;
	NewtonBodyGetMatrix(array[0], &matrix[0][0]);
	matrix.m_posit = matrix.m_posit + matrix.m_up.Scale(size.m_y * 0.5f) - matrix.m_right.Scale(size.m_z * 0.5f);
	dCustomHinge* hinge = new dCustomHinge(matrix, array[0], playgroundBody);
	hinge->SetAsSpringDamper(true, 0.9f, 0.0f, 20.0f);

	for (int i = 1; i < count; i++) {
		dMatrix matrix;
		NewtonBodyGetMatrix(array[i], &matrix[0][0]);
		matrix.m_posit = matrix.m_posit + matrix.m_up.Scale(size.m_y * 0.5f) - matrix.m_right.Scale(size.m_z * 0.5f);
		dCustomHinge* const hinge = new dCustomHinge(matrix, array[i - 1], array[i]);
		hinge->SetAsSpringDamper(true, 0.9f, 0.0f, 20.0f);
	}

	NewtonBodyGetMatrix(array[count - 1], &matrix[0][0]);
	matrix.m_posit = matrix.m_posit + matrix.m_up.Scale(size.m_y * 0.5f) + matrix.m_right.Scale(size.m_z * 0.5f);
	hinge = new dCustomHinge(matrix, array[count - 1], playgroundBody);
	hinge->SetAsSpringDamper(true, 0.9f, 0.0f, 20.0f);


	geometry->Release();
	NewtonDestroyCollision(collision);
}

static void AddBackground(DemoEntityManager* const scene)
{
//	NewtonBody* const playgroundBody = CreateLevelMesh (scene, "track.ngd", false);
	NewtonBody* const playgroundBody = CreateLevelMesh (scene, "track.ngd", true);
//	NewtonBody* const playgroundBody = CreateLevelMesh (scene, "playerarena.ngd", true);
//	CreateHeightFieldTerrain (scene, 10, 4.0f, 1.0f, 0.25f, -10.0f, 15.0f);
//	CreateHeightFieldTerrain(scene, 7, 8.0f, 5.0f, 0.2f, 200.0f, -50.0f);

	dMatrix location(dGetIdentityMatrix());
#if 0
	CreateBridge(scene, playgroundBody);

	NewtonBody* const terrain = CreateHeightFieldTerrain(scene, 7, 1.0f, 1.0f, 0.1f, 15.0f, -10.0f);
	DemoEntity* const terrainEntity = (DemoEntity*)NewtonBodyGetUserData(terrain);
	
	location.m_posit.m_x = 0.0f;
	location.m_posit.m_y -= 0.2f;
	location.m_posit.m_z = -50.0f;
	NewtonBodySetMatrix(terrain, &location[0][0]);
	terrainEntity->SetMatrixUsafe(dQuaternion(location), location.m_posit);
	terrainEntity->SetMatrixUsafe(dQuaternion(location), location.m_posit);
#endif

#if 0
	dMatrix shapeOffsetMatrix (dGetIdentityMatrix());

	location.m_posit.m_x = 2.5f;
	location.m_posit.m_y = 1.5f;
	location.m_posit.m_z = 1.6f;

	int count = 4;

	//AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 5.0f, _SPHERE_PRIMITIVE, 0, shapeOffsetMatrix);
	dVector size = dVector (4.0f, 0.25f, 1.0f, 0.0f);
	AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 6.0f, _BOX_PRIMITIVE, 0, shapeOffsetMatrix);

	size = dVector (0.75f, 1.0f, 0.75f, 0.0f);
	AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 6.0f, _CAPSULE_PRIMITIVE, 0, shapeOffsetMatrix);

	size = dVector(4.0f, 0.2f, 3.0f, 0.0f);
	AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 6.0f, _BOX_PRIMITIVE, 0, shapeOffsetMatrix);
	//AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 5.0f, _CYLINDER_PRIMITIVE, 0, shapeOffsetMatrix);
	//AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 5.0f, _CHAMFER_CYLINDER_PRIMITIVE, 0, shapeOffsetMatrix);
	//AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 5.0f, _CONE_PRIMITIVE, 0, shapeOffsetMatrix);
	//AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 5.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, 0, shapeOffsetMatrix);
	//AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 5.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, 0, shapeOffsetMatrix);
#endif

}

void SingleBodyCar(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();
	AddBackground(scene);

	dMatrix location (dGetIdentityMatrix());
	location.m_posit = dVector (0.0f, 10.0f, 0.0f, 1.0f);
	location.m_posit.m_y += 10.0f;
	location.m_posit = FindFloor (scene->GetNewton(), location.m_posit, 100.0f);
	location.m_posit.m_y += 0.5f;

	// create a vehicle manager
	NewtonWorld* const world = scene->GetNewton();
	SingleBodyVehicleManager* const manager = new SingleBodyVehicleManager(world);

	// create a sport car
	dPointer<DemoEntity> viperModel (DemoEntity::LoadNGD_mesh("viper.ngd", scene->GetNewton(), scene->GetShaderCache()));
//	dVehicle* const player0 = manager->CreateSportCar(location, viperModel.GetData());
//	manager->SetAsPlayer(player0);

	// create an monster Truck
	location.m_posit.m_x = 0.0f;
	location.m_posit.m_y = 10.0f;
	location.m_posit.m_z = 0.0f;

	location.m_posit.m_x = 140.0f;
	location.m_posit.m_z = -47.0f;

	location.m_posit = FindFloor(scene->GetNewton(), location.m_posit, 100.0f);
	location.m_posit.m_y += 1.5f;

	dPointer<DemoEntity> monsterTruck (DemoEntity::LoadNGD_mesh("monsterTruck.ngd", scene->GetNewton(), scene->GetShaderCache()));
	dVehicle* const player1 = manager->CreateOffRoadCar(location, monsterTruck.GetData());
	manager->SetAsPlayer(player1);

	int count = 10;
	count = 0;
	for (int i = 0; i < count; i++) {
		for (int j = 0; j < count; j++) {
			dMatrix offset(location);
			offset.m_posit += dVector (j * 5.0f + 4.0f, 0.0f, i * 5.0f, 0.0f);
			//manager->CreateSportCar(offset, viperModel.GetData());
			manager->CreateOffRoadCar(offset, monsterTruck.GetData());
		}
	}

	dQuaternion rot(dYawMatrix (90.0f * dDegreeToRad));
	dVector origin(-10.0f, 2.0f, 0.0f, 0.0f);
//	dVector origin(location.m_posit);
	scene->SetCameraMatrix(rot, origin);
}

