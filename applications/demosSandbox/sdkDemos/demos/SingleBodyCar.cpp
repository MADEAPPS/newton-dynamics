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


static dFloat chassisShape[] = 
{
	-2.086454f, 0.152566f, -0.709441f,
	-1.997459f, 0.059762f, -0.739932f,
	-2.012946f, 0.136518f, -0.784495f,
	-2.061989f, 0.078819f, -0.674259f,
	-1.869019f, -0.019734f, -0.746968f,
	-1.897917f, 0.113487f, -0.822021f,
	-1.885685f, 0.026161f, -0.793876f,
	-1.972096f, 0.009775f, -0.693023f,
	-1.590980f, 0.158113f, -0.861893f,
	-1.602427f, 0.048263f, -0.852512f,
	-1.609834f, -0.073154f, -0.782149f,
	-1.609834f, -0.036052f, -0.829058f,
	-2.014405f, 0.210028f, -0.796222f,
	-2.086342f, 0.213884f, -0.716477f,
	-1.899488f, 0.199730f, -0.831403f,
	-1.955945f, 0.523606f, -0.796221f,
	-2.001843f, 0.549986f, -0.763385f,
	-2.051377f, 0.575255f, -0.711786f,
	-1.883000f, 0.455491f, -0.831403f,
	-1.679823f, 0.538214f, -0.819676f,
	-2.173587f, 0.155545f, -0.542915f,
	-2.115996f, 0.048229f, -0.542915f,
	-2.096815f, 0.031283f, -0.542915f,
	-2.173587f, 0.213564f, -0.542915f,
	-2.170762f, 0.032452f, -0.289608f,
	-2.222964f, 0.158014f, -0.289608f,
	-2.222964f, 0.213564f, -0.289608f,
	-2.230410f, 0.157978f, -0.009146f,
	-2.079632f, -0.037389f, -0.069481f,
	-2.160463f, 0.632145f, -0.423298f,
	-2.177409f, 0.641785f, -0.289608f,
	-2.121689f, 0.611667f, -0.575751f,
	-2.180047f, 0.647568f, -0.046670f,
	-2.230410f, 0.213528f, 0.038257f,
	-2.178321f, 0.032530f, 0.028381f,
	-2.079632f, -0.037522f, 0.064146f,
	-2.223003f, 0.213528f, 0.281195f,
	-2.223003f, 0.157978f, 0.281195f,
	-2.170801f, 0.032416f, 0.281194f,
	-2.116035f, 0.048193f, 0.534502f,
	-2.173625f, 0.213528f, 0.534502f,
	-2.096854f, 0.031247f, 0.534502f,
	-2.173626f, 0.155509f, 0.534502f,
	-2.121728f, 0.611631f, 0.567338f,
	-2.177448f, 0.641749f, 0.281195f,
	-2.160502f, 0.632109f, 0.414885f,
	-2.062028f, 0.078784f, 0.665846f,
	-1.972135f, 0.009739f, 0.684609f,
	-1.997498f, 0.059726f, 0.731518f,
	-2.012985f, 0.136482f, 0.776081f,
	-2.086493f, 0.152530f, 0.701027f,
	-1.869058f, -0.019770f, 0.738554f,
	-1.885724f, 0.026126f, 0.785463f,
	-1.899527f, 0.199694f, 0.822990f,
	-1.897956f, 0.113451f, 0.813608f,
	-1.609873f, -0.073190f, 0.773736f,
	-1.609873f, -0.036088f, 0.820644f,
	-1.602466f, 0.048227f, 0.844098f,
	-1.591019f, 0.158077f, 0.853480f,
	-2.086381f, 0.213848f, 0.708063f,
	-1.869797f, 0.404386f, 0.827680f,
	-2.014444f, 0.209992f, 0.787808f,
	-1.955984f, 0.523570f, 0.787808f,
	-2.001882f, 0.549951f, 0.754972f,
	-2.051428f, 0.575211f, 0.703373f,
	-1.883039f, 0.455455f, 0.822990f,
	-1.679862f, 0.538178f, 0.811263f,
	-0.873980f, -0.174415f, -0.474514f,
	-0.820557f, -0.160951f, -0.810294f,
	-0.647632f, -0.136693f, -0.866585f,
	-0.195411f, 0.544052f, -0.957154f,
	-0.190473f, 0.573679f, -0.944809f,
	-0.776203f, 0.906084f, -0.289608f,
	-0.899646f, 0.869051f, -0.416262f,
	-0.884833f, 0.841893f, -0.505388f,
	-0.608319f, 0.893740f, -0.411571f,
	-0.637946f, 0.849300f, -0.510079f,
	-0.598444f, 0.913491f, -0.289608f,
	-0.460186f, 0.888802f, -0.420953f,
	-0.430560f, 0.898678f, -0.289608f,
	-0.233049f, 0.861644f, -0.289608f,
	-0.312053f, 0.822142f, -0.538224f,
	-0.647671f, -0.136729f, 0.858171f,
	-0.873980f, -0.174415f, 0.460584f,
	-0.820596f, -0.160987f, 0.801881f,
	-0.190443f, 0.573669f, 0.946512f,
	-0.195381f, 0.544042f, 0.958857f,
	-0.909561f, 0.883828f, 0.281195f,
	-0.884872f, 0.841858f, 0.496975f,
	-0.899685f, 0.869015f, 0.407848f,
	-0.776242f, 0.906048f, 0.281195f,
	-0.637985f, 0.849264f, 0.501666f,
	-0.608358f, 0.893704f, 0.403157f,
	-0.460225f, 0.888766f, 0.412539f,
	-0.598483f, 0.913455f, 0.281195f,
	-0.312092f, 0.822107f, 0.529811f,
	-0.430599f, 0.898642f, 0.281195f,
	-0.233088f, 0.861608f, 0.281195f,
	0.117520f, -0.138259f, -0.866585f,
	1.340888f, 0.463538f, -0.819634f,
	1.291734f, 0.413010f, -0.871234f,
	1.354360f, 0.482077f, -0.746887f,
	1.385667f, 0.377836f, -0.871233f,
	1.677863f, -0.111093f, -0.804863f,
	1.552669f, -0.109036f, -0.833008f,
	1.552669f, -0.179011f, -0.798946f,
	1.681051f, -0.173143f, -0.753263f,
	1.677863f, -0.027152f, -0.822021f,
	1.552669f, 0.008220f, -0.857203f,
	1.481395f, 0.432761f, -0.806734f,
	1.502558f, 0.441707f, -0.739300f,
	1.460072f, 0.314659f, -0.866543f,
	1.652595f, 0.349023f, -0.740117f,
	1.563207f, 0.400902f, -0.780441f,
	1.562309f, 0.371512f, -0.806240f,
	1.685256f, 0.382538f, -0.595420f,
	1.641193f, 0.435786f, -0.046576f,
	1.776618f, -0.059247f, -0.768076f,
	1.885249f, -0.111093f, -0.641423f,
	1.776618f, -0.111093f, -0.755856f,
	1.885249f, -0.083936f, -0.643768f,
	1.934626f, -0.111093f, -0.547606f,
	1.771681f, -0.172815f, -0.599205f,
	1.984004f, -0.093811f, -0.457986f,
	1.949440f, -0.091343f, -0.537730f,
	1.786493f, 0.212329f, -0.723513f,
	1.707487f, 0.182703f, -0.782149f,
	1.964256f, 0.155545f, -0.504771f,
	1.922281f, 0.182703f, -0.561555f,
	1.794557f, 0.324334f, -0.551185f,
	2.063006f, -0.111093f, -0.046670f,
	1.865498f, -0.172815f, -0.313062f,
	2.033379f, -0.111093f, -0.289608f,
	1.843267f, 0.322184f, -0.436140f,
	1.869187f, 0.322218f, -0.289566f,
	1.996345f, 0.185172f, -0.411447f,
	2.035848f, 0.185172f, -0.289608f,
	1.949437f, 0.239487f, -0.423174f,
	1.973628f, 0.271935f, -0.032095f,
	2.063007f, 0.185172f, -0.046670f,
	1.841534f, 0.351845f, -0.046643f,
	0.117481f, -0.138295f, 0.858171f,
	1.385628f, 0.377800f, 0.862904f,
	1.460033f, 0.314623f, 0.858213f,
	1.340849f, 0.463502f, 0.811305f,
	1.291695f, 0.412974f, 0.862904f,
	1.354360f, 0.482077f, 0.738639f,
	1.552630f, -0.179047f, 0.790533f,
	1.677824f, -0.111129f, 0.796450f,
	1.681012f, -0.173179f, 0.744850f,
	1.552630f, -0.109072f, 0.824595f,
	1.677824f, -0.027188f, 0.813608f,
	1.552630f, 0.008185f, 0.848790f,
	1.685217f, 0.382502f, 0.587091f,
	1.696539f, 0.409827f, 0.126789f,
	1.707448f, 0.182667f, 0.773736f,
	1.652556f, 0.348987f, 0.741579f,
	1.481356f, 0.432725f, 0.798405f,
	1.563168f, 0.400866f, 0.772112f,
	1.502519f, 0.441671f, 0.730971f,
	1.562270f, 0.371476f, 0.797911f,
	1.865459f, -0.172852f, 0.304649f,
	1.900023f, -0.172852f, 0.000730f,
	2.033340f, -0.111129f, 0.281195f,
	2.062967f, -0.111129f, 0.000730f,
	2.062969f, 0.185136f, 0.038257f,
	1.973628f, 0.271935f, 0.027158f,
	2.020538f, 0.232433f, 0.027158f,
	1.869148f, 0.322182f, 0.281237f,
	2.035809f, 0.185136f, 0.281195f,
	1.983965f, -0.093847f, 0.449572f,
	1.771642f, -0.172852f, 0.590792f,
	1.949401f, -0.091379f, 0.529317f,
	1.969151f, -0.111129f, 0.459448f,
	1.776579f, -0.059283f, 0.759663f,
	1.776580f, -0.111129f, 0.747442f,
	1.885210f, -0.111129f, 0.633010f,
	1.843229f, 0.322148f, 0.427811f,
	1.949398f, 0.239451f, 0.414761f,
	1.996306f, 0.185136f, 0.403034f,
	1.964217f, 0.155509f, 0.496358f,
	1.922243f, 0.182667f, 0.553142f,
	1.786454f, 0.212294f, 0.715100f,
	1.794518f, 0.324298f, 0.552648f
};



#if 0
class BasicCarControllerManager: public dCustomVehicleControllerManager
{
	public:
	BasicCarControllerManager (NewtonWorld* const world, int materialsCount, int* const materialList)
		:dCustomVehicleControllerManager (world, materialsCount, materialList)
//		,m_externalView(true)
//		,m_player (NULL) 
	{
		// hook a callback for 2d help display
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);
		scene->Set2DDisplayRenderFunction (RenderVehicleHud, NULL, this);
	}

	~BasicCarControllerManager ()
	{
	}

	static void RenderVehicleHud (DemoEntityManager* const scene, void* const context)
	{
		BasicCarControllerManager* const me = (BasicCarControllerManager*) context;
		//me->RenderVehicleHud (scene);
		me->DrawHelp (scene);
	}

	void DrawHelp(DemoEntityManager* const scene) const
	{
//		if (m_player->m_helpKey.GetPushButtonState()) 
		{
			dVector color(1.0f, 1.0f, 0.0f, 0.0f);
			scene->Print (color, "Vehicle driving keyboard control");
			scene->Print (color, "key switch          : 'I'");
			scene->Print (color, "accelerator         : 'W'");
			scene->Print (color, "reverse             : 'S'");
			scene->Print (color, "turn left           : 'A'");
			scene->Print (color, "turn right          : 'D'");
			scene->Print (color, "engage clutch       : 'K'");
			scene->Print (color, "hand brakes         : 'space'");
			scene->Print (color, "hide help           : 'H'");
		}
	}
/*
	void RenderVehicleHud (DemoEntityManager* const scene) const
	{
//		dAssert(0);

		if (m_player) {
			// set to transparent color
			glEnable (GL_BLEND);
			glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

			//print controllers help 
			DrawHelp(scene, lineNumber);
	
			glDisable(GL_LIGHTING);
			glDisable(GL_TEXTURE_2D);
			glDisable(GL_DEPTH_TEST);

			dFloat scale = 100.0f;
			dFloat width = scene->GetWidth();
			dFloat height = scene->GetHeight();

			dMatrix origin(dGetIdentityMatrix());
			origin.m_posit = dVector(width -300, height -200, 0.0f, 1.0f);

			glPushMatrix();
			glMultMatrix(&origin[0][0]);
			DrawSchematic(m_player->m_controller, scale);
			glPopMatrix();

			glLineWidth(1.0f);
			
			// restore color and blend mode
			glEnable(GL_DEPTH_TEST);
			glEnable(GL_TEXTURE_2D);
			glEnable(GL_LIGHTING);
			glDisable(GL_BLEND);
		}
	}
*/

	void ApplyPlayerControl (dFloat timestep) const 
	{
		NewtonBody* const body = m_player->GetBody();
		NewtonWorld* const world = NewtonBodyGetWorld(body);
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);

/*
		// get the throttler input
		dFloat joyPosX;
		dFloat joyPosY;
		int joyButtons;

		int gear = engine ? engine->GetGear() : 0;
		bool hasJopytick = mainWindow->GetJoytickPosition (joyPosX, joyPosY, joyButtons);
		if (hasJopytick) {
			// apply a cubic attenuation to the joystick inputs
//			joyPosX = joyPosX * joyPosX * joyPosX;
//			joyPosY = joyPosY * joyPosY * joyPosY;
//			steeringVal = joyPosX;
//			brakePedal = (joyPosY < 0.0f) ? -joyPosY: 0.0f;
//			engineGasPedal = (joyPosY >= 0.0f) ? joyPosY: 0.0f;
//			gear += int (m_gearUpKey.UpdateTriggerJoystick(mainWindow, joyButtons & 2)) - int (m_gearDownKey.UpdateTriggerJoystick(mainWindow, joyButtons & 4));
//			handBrakePedal = (joyButtons & 1) ? 1.0f : 0.0f;
			}
*/

		dVehicleDriverInput driverInput;

		driverInput.m_throttle = scene->GetKeyState('W') ? 1.0f : 0.0f;
		driverInput.m_brakePedal = scene->GetKeyState('S') ? 1.0f : 0.0f;
		driverInput.m_clutchPedal = scene->GetKeyState('K') ? 1.0f : 0.0f;
		//driverInput.m_manualTransmission = !m_automaticTransmission.UpdatePushButton (scene, 0x0d);
		driverInput.m_steeringValue = (dFloat(scene->GetKeyState('D')) - dFloat(scene->GetKeyState('A')));
		//gear += int(m_gearUpKey.UpdateTriggerButton(scene, '.')) - int(m_gearDownKey.UpdateTriggerButton(scene, ','));

		driverInput.m_handBrakeValue = scene->GetKeyState(' ') ? 1.0f : 0.0f;
		//driverInput.m_ignitionKey = m_engineKeySwitch.UpdatePushButton(scene, 'I');
		//driverInput.m_lockDifferential = m_engineDifferentialLock.UpdatePushButton(scene, 'L');

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

		m_player->ApplyDefualtDriver(driverInput, timestep);
	}

	void UpdateDriverInput(dCustomVehicleController* const vehicle, dFloat timestep) const
	{
		//NewtonBody* const body = vehicle->GetBody();
		//DemoEntity* const vehicleEntity = (SuperCarEntity*)NewtonBodyGetUserData(body);

		if (vehicle == m_player) {
			// do player control
			ApplyPlayerControl(timestep);
		} else {
			// do no player control
			//vehicleEntity->ApplyNPCControl (timestep, m_raceTrackPath);
		}
	}

	void SetAsPlayer (dCustomVehicleController* const player)
	{
		m_player = player;
	}

	virtual void PreUpdate (dFloat timestep)
	{
		// apply the vehicle controls, and all simulation time effect
		//NewtonWorld* const world = GetWorld();
		//DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		// set the help key
		//m_helpKey.UpdatePushButton(scene->GetRootWindow(), 'H');

		// do the base class post update
		dCustomVehicleControllerManager::PreUpdate(timestep);
	}

	virtual void PostUpdate (dFloat timestep)
	{
		// do the base class post update
		dCustomVehicleControllerManager::PostUpdate(timestep);

		// update the camera 
		UpdateCamera (timestep);
	}


	void UpdateCamera (dFloat timestep)
	{
		if (m_player) {

			DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(GetWorld());
			DemoCamera* const camera = scene->GetCamera();
			dMatrix camMatrix (camera->GetNextMatrix ());

			DemoEntity* const vehicleEntity = (DemoEntity*)NewtonBodyGetUserData(m_player->GetBody());

			dMatrix playerMatrix (vehicleEntity->GetNextMatrix());

			dVector frontDir (camMatrix[0]);
			dVector camOrigin(0.0f); 
			if (m_externalView) {
				camOrigin = playerMatrix.m_posit + dVector(0.0f, VEHICLE_THIRD_PERSON_VIEW_HIGHT, 0.0f, 0.0f);
				camOrigin -= frontDir.Scale (VEHICLE_THIRD_PERSON_VIEW_DIST);
			} else {
				dAssert (0);
				//camMatrix = camMatrix * playerMatrix;
				//camOrigin = playerMatrix.TransformVector(dVector(-0.8f, ARTICULATED_VEHICLE_CAMERA_EYEPOINT, 0.0f, 0.0f));
			}

			camera->SetNextMatrix (*scene, camMatrix, camOrigin);
		}
	}


	// use this to display debug information about vehicle 
	void Debug () const
	{
	}

	dCustomVehicleController* LoadVehicle(const char* const name)
	{
		dAssert (0);
		return NULL;
/*
		char fileName[2048];
		dGetWorkingFileName(name, fileName);

		char* const oldloc = setlocale(LC_ALL, 0);
		setlocale(LC_ALL, "C");
		FILE* const inputFile = fopen(fileName, "rt");
		dAssert(inputFile);
		
		dBasicVehicleLoader loader(GetWorld(), inputFile, 0);
		dCustomVehicleController* const vehicle = Load(&loader);

		fclose(inputFile);
		setlocale(LC_ALL, oldloc);
		return vehicle;
*/
	}

	bool m_externalView;
	dCustomVehicleController* m_player;
};
#endif

class SingleBodyVehicleManager: public dVehicleManager
{
	public:
	SingleBodyVehicleManager(NewtonWorld* const world)
		:dVehicleManager(world)
	{
	}

	~SingleBodyVehicleManager()
	{
	}

	void MakeVisualEntity(dVehicleChassis* const chassis)
	{
		NewtonBody* const chassiBody = chassis->GetBody();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(GetWorld());

		NewtonCollision* const chassisCollision = NewtonBodyGetCollision(chassiBody);
		DemoMesh* const chassisMesh = new DemoMesh("chassis", chassisCollision, "metal_30.tga", "metal_30.tga", "metal_30.tga");

		dMatrix chassisMatrix;
		NewtonBodyGetMatrix(chassiBody, &chassisMatrix[0][0]);
		DemoEntity* const entity = new DemoEntity(chassisMatrix, NULL);
		scene->Append(entity);
		entity->SetMesh(chassisMesh, dGetIdentityMatrix());
		chassisMesh->Release();

		// set the user data
		NewtonBodySetUserData(chassiBody, entity);

		// set the transform callback
		NewtonBodySetTransformCallback(chassiBody, DemoEntity::TransformCallback);
	}

	dVehicleChassis* CreateVehicle(const dMatrix& location)
	{
		NewtonWorld* const world = GetWorld();

		// make chassis collision shape;
		int chassisVertexCount = sizeof(chassisShape) / (3 * sizeof(chassisShape[0]));
		NewtonCollision* const chassisCollision = NewtonCreateConvexHull(world, chassisVertexCount, &chassisShape[0], 3 * sizeof(dFloat), 0.001f, 0, NULL);

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

		// get body from player
		NewtonBody* const body = vehicle->GetBody();

		// set the player matrix 
		NewtonBodySetMatrix(body, &location[0][0]);

//		for (int i = 0; i < int((sizeof(m_gearMap) / sizeof(m_gearMap[0]))); i++) {
//			m_gearMap[i] = i;
//		}

		// create the visual representation
		MakeVisualEntity(vehicle);

		// destroy chassis collision shape 
		NewtonDestroyCollision(chassisCollision);

		return vehicle;
	}
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
	location.m_posit.m_y += 2.0f;

	NewtonWorld* const world = scene->GetNewton();

	// create a vehicle controller manager
//	int defaulMaterial = NewtonMaterialGetDefaultGroupID(scene->GetNewton());
//	int materialList[] = {defaulMaterial };
	SingleBodyVehicleManager* const manager = new SingleBodyVehicleManager(world);
	
	// load 
//	dCustomVehicleController* const player = manager->LoadVehicle("simpleVehicle.txt");
	dVehicleChassis* const player = manager->CreateVehicle(location);

/*
	// set this vehicle as the player
	manager->SetAsPlayer(player);

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
	dVector origin(-10.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}

