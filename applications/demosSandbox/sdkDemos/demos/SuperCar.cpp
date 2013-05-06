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
#include "../DemoMesh.h"
//#include "../DemoAIAgent.h"
#include "../DemoEntityManager.h"
#include "../DemoCamera.h"
#include "PhysicsUtils.h"
#include "../toolBox/DebugDisplay.h"


#if 0
#include "CustomWheelVehicle.h"
#define VIPER_MASS						(2000.0f)
#define VIPER_TIRE_FRICTION				(1.0f)
#define VIPER_TIRE_TYRE_MASS			(20.0f)
#define VIPER_SUSPENSION_SPRING			(250.0f)
#define VIPER_SUSPENSION_DAMPER			(10.0f)
#define VIPER_SUSPENSION_LENGTH			(0.10f)

#define LAMBORGHINI_MASS				(1700.0f)
#define LAMBORGHINI_TIRE_FRICTION		(1.0f)
#define LAMBORGHINI_TIRE_TYRE_MASS		(20.0f)
#define LAMBORGHINI_SUSPENSION_SPRING	(250.0f)
#define LAMBORGHINI_SUSPENSION_DAMPER	(10.0f)
#define LAMBORGHINI_SUSPENSION_LENGTH	(0.10f)


// an ID for finding this joint in a callback
#define CUSTOM_VEHICLE_JOINT_ID			0xD9A7F4ED



// car mass (can be any value)
struct CarParam
{
	CarParam(float mass, float tireMass, float tireFriction, float suspensionSpring, float suspensionDamper, float suspensionLength)
	{
		m_mass = mass;
		m_tireMass = tireMass;
		m_tireFriction = tireFriction;
		m_suspensionSpring = suspensionSpring;
		m_suspensionDamper = suspensionDamper;
		m_suspensionLength = suspensionLength;
	}

	float m_mass;
	float m_tireMass;
	float m_tireFriction;
	float m_suspensionSpring;
	float m_suspensionDamper;
	float m_suspensionLength;
};


#if 0
class BasicVehicle
{
	//static char* engineSounds[] = {"engine_start.wav", "engine_rpm.wav", "tire_skid.wav"};
	public:
	enum
	{
		m_engine_start,
		m_engine_rpm,
		m_engine_skid,
	};

	class VehicleIterpolateTireMatrices: public DemoEntity::UserData
	{
		public:
		VehicleIterpolateTireMatrices (BasicVehicle* const car)
			:m_car(car)
		{
		}

		virtual void OnInterpolateMatrix (DemoEntityManager& world, dFloat param) const
		{
			// iterate over all tires and calculate the interpolated matrix between current and next physics frame at parameter param.
			CustomWheelVehicle* const vehicle = m_car->m_vehicle;
			int count = vehicle->GetTiresCount();
			for (int i = 0; i < count; i ++) {
				const CustomWheelVehicle::dTire& tire = vehicle->GetTire (i);
				DemoEntity* const tireNode = (DemoEntity*) tire.m_userData;
				tireNode->InterpolateMatrix (world, param);
			}
		}

		virtual void OnRender (dFloat timestep) const
		{
		}

		BasicVehicle* m_car;
	};

	class VehicleShowTire: public DemoEntity::UserData
	{
		public:
		VehicleShowTire (BasicVehicle* const car, DemoEntity* const tirePart, int tireIndex)
			:m_tireIndex(tireIndex)
			,m_me(tirePart)
			,m_car(car)
		{
		}

		virtual void OnInterpolateMatrix (DemoEntityManager& world, dFloat param) const
		{

		}

		virtual void OnRender (dFloat timestep) const
		{
			if (DebugDisplayOn()) {
				CustomWheelVehicle* const vehicle = m_car->m_vehicle;
				
				dMatrix carMatrix;
				NewtonBodyGetMatrix(m_car->m_body, &carMatrix[0][0]);
				
				glDisable (GL_LIGHTING);
				glDisable(GL_TEXTURE_2D);

				// we need to undo the vehicle local matrices and render the tire info in global space 
				dMatrix matrix (m_me->CalculateInterpolatedGlobalMatrix().Inverse());
				glPushMatrix();
				glMultMatrix(&matrix[0][0]);

				// draw the the tire collision shape
				const CustomWheelVehicle::dTire& tire = vehicle->GetTire (m_tireIndex);
				dMatrix tireBaseMatrix (vehicle->CalculateSuspensionMatrix(m_tireIndex, tire.m_posit) * vehicle->GetChassisMatrixLocal() * carMatrix);

				// draw the contact when the tire is touching something
				if (!vehicle->GetTireOnAir(m_tireIndex)) {
					glColor3f(1.0f, 1.0f, 0.0f);
					DebugDrawCollision (tire.m_shape, tireBaseMatrix, m_solid);

					glColor3f(1.0f, 1.0f, 1.0f);
					dVector span (tire.m_contactPoint + tire.m_contactNormal.Scale (1.0f));
					DebugDrawLine (tire.m_contactPoint, span);

					glColor3f(1.0f, 0.0f, 0.0f);
					DebugDrawPoint (tire.m_contactPoint, 5.0f);
				}

				glColor3f(1.0f, 1.0f, 1.0f);

				glPopMatrix();
			}
		}

		int m_tireIndex;
		DemoEntity* m_me;
		BasicVehicle* m_car;
	};

	public:
	BasicVehicle (DemoEntity* const entity, int maxTiresCount, int materialId, DemoEntityManager* const scene, CarParam& params)
		:m_body(NULL)
		,m_camera(NULL)
		,m_vehicle(NULL)
		,m_myScene(scene)
	{
		NewtonWorld* const world = scene->GetNewton();

		// create the vehicle body
		CreateChassisBody (entity, world, materialId, params);

		// create the vehicle joint
		CreateVehicleJoint (params);

		// add a use data top the entity so that we can interpolate the tire positions at render time 
		entity->SetUserData (new VehicleIterpolateTireMatrices (this));

		// create the vehicle sound 
		static char* engineSounds[] = {"engine_start.wav", "engine_rpm.wav", "tire_skid.wav"};
		dSoundManager& soundManager = scene->GetSoundManager();
		for (int i = 0; i < sizeof (engineSounds) / sizeof (engineSounds[0]); i ++) {
			void* sound = soundManager.CreateSound(engineSounds[i]);
			soundManager.SetSoundPlayMode (sound, true);
			m_engineSounds[i] = sound;
		}

		// set a sync point before the end of non looping sounds
		float duration = soundManager.GetSoundlength (m_engineSounds[0]) * 0.95f;
		soundManager.SetSoundEvent (m_engineSounds[0], duration);

		// load texture for displaying the dashboard
		m_gears = LoadTexture ("gears_font.tga");
		m_rpmDial = LoadTexture ("rpm_dial.tga");
		m_rpmNeedle = LoadTexture ("needle_green.tga");
		m_kmhDial = LoadTexture ("kmh_dial.tga");
		m_kmhNeedle = LoadTexture ("needle_red.tga");
	}

	~BasicVehicle()
	{
		if (m_vehicle->GetUserData()) {
			m_vehicle->SetUserData(NULL);
			delete m_vehicle;
		}
	}


	static void Destructor (const NewtonUserJoint* joint)
	{
		CustomWheelVehicle* const vehicleJoint = (CustomWheelVehicle*)joint;
		BasicVehicle* const me =  (BasicVehicle*) vehicleJoint->GetUserData();

		if (me) {
			vehicleJoint->SetUserData(NULL);
			vehicleJoint->SetUserDestructorCallback(NULL);
			delete me;
		}
	}

	void CreateVehicleJoint (CarParam& param)
	{
		// set the vehicle local coordinate system 
		dMatrix chassisMatrix;
		// if your vehicle moves along the z direction you can use 
		// chassisMatrix.m_front = dVector (0.0f, 0.0f, 1.0f, 0.0);
		chassisMatrix.m_front = dVector (1.0f, 0.0f, 0.0f, 0.0f);			// this is the vehicle direction of travel
		chassisMatrix.m_up	  = dVector (0.0f, 1.0f, 0.0f, 0.0f);			// this is the downward vehicle direction
		chassisMatrix.m_right = chassisMatrix.m_front * chassisMatrix.m_up;	// this is in the side vehicle direction (the plane of the wheels)
		chassisMatrix.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);

		// create the newton car joint
		m_vehicle = new CustomWheelVehicle (chassisMatrix, m_body);
		dAssert (m_vehicle);

		// assign a type information to locate the joint in the callbacks
		m_vehicle->SetJointID (CUSTOM_VEHICLE_JOINT_ID);

		// save the pointer to this class as the vehicle user data  
		m_vehicle->SetUserData (this);

		// link a destructor so that when the body of the vehicle is destroyed we can destroy this object
		m_vehicle->SetUserDestructorCallback(Destructor);

		// link a user constraint callback to be used as the driver
		//m_vehicle->SetUserSubmintConstraintCallback (VehicleSubmitConstraintCallback);
		

		dFloat width;
		dFloat radius;
		// calculate the front tire dimensions, and add the tires
		CalculateTireDimensions ("fl_tire", width, radius);
		int leftFrontTireIndex = AddTire ("fl_tire", param.m_tireMass, width, radius, param.m_tireFriction, param.m_suspensionLength, param.m_suspensionSpring, param.m_suspensionDamper);
		int rightFrontTireIndex = AddTire ("fr_tire", param.m_tireMass, width, radius, param.m_tireFriction, param.m_suspensionLength, param.m_suspensionSpring, param.m_suspensionDamper);

		// calculate the rear tire dimensions, and add the tires
		CalculateTireDimensions ("rl_tire", width, radius);
		int leftRearTireIndex = AddTire ("rl_tire", param.m_tireMass, width, radius, param.m_tireFriction, param.m_suspensionLength, param.m_suspensionSpring, param.m_suspensionDamper);
		int rightRearTireIndex = AddTire ("rr_tire", param.m_tireMass, width, radius, param.m_tireFriction, param.m_suspensionLength, param.m_suspensionSpring, param.m_suspensionDamper);

		// Build an engine base of the Lamborghini Diablo specifications
		//1st gear ratio : 2.313:1 
		//2nd gear ratio : 1.524:1 
		//3rd gear ratio : 1.125:1 
		//4th gear ratio : 0.889:1 
		//5th gear ratio :  0.697:1 
		//Rev. gear ratio: 2.125:1 
		//Rear transfer ratio : 1.783:1 
		//Rear differential ratio : 2.529:1 
		//Max. power : 492 bhp at 7000 rpm 

		// this car will be rear wheel drive only
		// create one differentials
		CustomWheelVehicle::dVehicleDifferencial* const differencial = new CustomWheelVehicle::dVehicleDifferencial (leftRearTireIndex, rightRearTireIndex, 2.529f, 0.5f, 0.5f);

		dFloat gearBoxRatios[] = {0.0, -2.125f, 2.313f, 1.524f, 0.889f, 0.697f, 0.697f}; 
		CustomWheelVehicle::dVehicleEngineTransmission* const transmission = new CustomWheelVehicle::dVehicleEngineTransmission(differencial, sizeof (gearBoxRatios)/sizeof (dFloat), gearBoxRatios);
		m_vehicle->SetEngine(new CustomWheelVehicle::dVehicleEngine(transmission, 10000.0f, 1000.0f, 492.0));

		// now set the steering wheel component
		CustomWheelVehicle::dVehicleSteering* const steering = new CustomWheelVehicle::dVehicleSteering (45.0f * 3.141592f / 180.0f);
		steering->AddSteerinWheel(leftFrontTireIndex);
		steering->AddSteerinWheel(rightFrontTireIndex);
		m_vehicle->SetSteering(steering);
	}

	void CalculateTireDimensions (const char* const tireName, dFloat& width, dFloat& radius) const
	{
		// find the the tire visual mesh 
		NewtonWorld* const world = NewtonBodyGetWorld(m_body);
		DemoEntity* const entity = (DemoEntity*) NewtonBodyGetUserData(m_body);

		DemoEntity* const tirePart = entity->dHierarchy::Find (tireName);
		dAssert (tirePart);


		// make a convex hull collision shape to assist in calculation of the tire shape size
		DemoMesh* const tireMesh = tirePart->GetMesh();
		NewtonCollision* const collision = NewtonCreateConvexHull(world, tireMesh->m_vertexCount, tireMesh->m_vertex, 3 * sizeof (dFloat), 0, 0, NULL);

		dMatrix tireLocalMatrix (entity->GetCurrentMatrix());
		dMatrix tireMatrix (tirePart->CalculateGlobalMatrix() * tireLocalMatrix);

		// find the support points that can be used to define the tire collision mesh
		dVector extremePoint;
		dVector upDir (tireMatrix.UnrotateVector(dVector (0.0f, 1.0f, 0.0f, 0.0f)));
		NewtonCollisionSupportVertex (collision, &upDir[0], &extremePoint[0]);
		radius = dAbs (upDir % extremePoint);

		//dVector widthDir (tireMatrix.UnrotateVector(tireLocalMatrix.m_right));
		dVector widthDir (tireMatrix.UnrotateVector(tireLocalMatrix.m_front));
		NewtonCollisionSupportVertex (collision, &widthDir[0], &extremePoint[0]);
		width = widthDir % extremePoint;

		widthDir = widthDir.Scale (-1.0f);
		NewtonCollisionSupportVertex (collision, &widthDir[0], &extremePoint[0]);
		width += widthDir % extremePoint;

		NewtonReleaseCollision (world, collision);	
	}


	int AddTire (const char* const tireName, dFloat mass, dFloat width, dFloat radius, 
				 dFloat friction, dFloat suspensionLength, dFloat suspensionSpring, dFloat suspensionDamper) 
	{
		DemoEntity* const entity = (DemoEntity*) NewtonBodyGetUserData(m_body);
		DemoEntity* const tirePart = entity->dHierarchy::Find (tireName);

		// add this tire, get local position and rise it by the suspension length 
		dMatrix tireMatrix (tirePart->CalculateGlobalMatrix(entity));
		dVector tirePosition (tireMatrix.m_posit);
		dFloat rollingResistance = 1.0e-2f;
		int tireIndex = m_vehicle->AddSingleSuspensionTire (tirePart, tirePosition, mass, radius, width, friction, suspensionLength, suspensionSpring, suspensionDamper, rollingResistance, 1);

		// attach a use data to the car for debug display purpose
		tirePart->SetUserData( new VehicleShowTire (this, tirePart, tireIndex));

		return tireIndex;
	}

	static void TransformCallback (const NewtonBody* body, const dFloat* matrix, int threadIndex)
	{
		// set the transform of the main body
		DemoEntity::TransformCallback (body, matrix, threadIndex);

		NewtonWorld* const world = NewtonBodyGetWorld(body);
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world );
	
		// find the car joint attached to the body
		BasicVehicle* car = NULL;
		for (NewtonJoint* joint = NewtonBodyGetFirstJoint(body); joint; joint = NewtonBodyGetNextJoint(body, joint)) {
			NewtonCustomJoint* const customJoint = (NewtonCustomJoint*) NewtonJointGetUserData (joint);  
			if (customJoint->GetJointID() == CUSTOM_VEHICLE_JOINT_ID) {
				CustomWheelVehicle* const vehicle = (CustomWheelVehicle*) customJoint;
				car = (BasicVehicle*)vehicle->GetUserData();
				break;
			}
		}

		if (car) {
			// position all visual tires matrices
			CustomWheelVehicle* const vehicle = car->m_vehicle;
			const dMatrix& carMatrix = *((dMatrix*)matrix);
			dMatrix rootMatrixInv (car->m_bodyBindMatrix * carMatrix);
			rootMatrixInv = rootMatrixInv.Inverse();
			int count = vehicle->GetTiresCount();

			// for each tire get the global matrix position, and calculate the local matrix relative to the main body   
			for (int i = 0; i < count; i ++) {
				//dBone* tireNode;
				//TireAnimation* tireAnim;
				const CustomWheelVehicle::dTire& tire = vehicle->GetTire (i);

				DemoEntity* const tireNode = (DemoEntity*) tire.m_userData;

				// calculate the tire local matrix
				dMatrix matrix (car->m_tireAlign * vehicle->CalculateTireMatrix(i) * rootMatrixInv);

				//tireNode->SetMatrix(matrix);
				dQuaternion rot (matrix);
				tireNode->SetMatrix (*scene, rot, matrix.m_posit);
			}
		}
	}


	NewtonBody* m_body;
	DemoCamera* m_camera;
	CustomWheelVehicle* m_vehicle;
	JointUserDestructorCallback m_destructorCallback;
	dFloat m_steerAngle;
	dMatrix m_tireAlign;
	dMatrix m_bodyBindMatrix;

	void* m_engineSounds[10];
	DemoEntityManager* m_myScene;
	GLuint m_gears;
	GLuint m_rpmDial;
	GLuint m_rpmNeedle;
	GLuint m_kmhDial;
	GLuint m_kmhNeedle;
};




// ***************************************************************************
// 
//  Vehicle control state machine declaration
//
// ***************************************************************************
class VehicleAIAgent: public DemoAIAgent
{
	public:
	//transitions of the vehicle state machine
	enum 
	{
		m_engine_on,
		m_engine_off,
	};

	VehicleAIAgent (DemoEntityManager* const scene, BasicVehicle* const car);

	virtual void Update (dFloat timestep, int threadID);

	static void SoundCallback (void* const channelHandle, void* const context, void* enventHandle);
	
	BasicVehicle* m_car;
	bool m_vehicleIsPlayer;
	dFloat m_startTime;
	dFloat m_startLength;
	dFloat m_rpmVolume;
	void* m_engineRpmSoundChannel;
	void* m_startEngineSoundChannel;

	PushButtonKey m_engineGearUp;
	PushButtonKey m_engineGearDown;
};


class VehicleAIAgentStateEngineOn: public DemoAIState
{
	public:
	VehicleAIAgentStateEngineOn (VehicleAIAgent* const agent, DemoEntityManager* const scene)
		:DemoAIState (agent, scene)
	{
	}
	virtual void Enter(int threadID);
	virtual void Exit(int threadID);

};

// this state do nothing
class VehicleAIAgentStateEngineOff: public DemoAIState
{
	public:
	VehicleAIAgentStateEngineOff (VehicleAIAgent* const agent, DemoEntityManager* const scene)
		:DemoAIState (agent, scene)
	{
	}
};

// ***************************************************************************
// 
//  Vehicle control state machine implementation
//
// ***************************************************************************
VehicleAIAgent::VehicleAIAgent (DemoEntityManager* const scene, BasicVehicle* const car)
	:DemoAIAgent (scene)
	,m_car (car)
	,m_vehicleIsPlayer(false)
	,m_engineRpmSoundChannel(NULL)
	,m_startEngineSoundChannel(NULL)
{
	VehicleAIAgentStateEngineOn* const engineOnState = new VehicleAIAgentStateEngineOn (this, scene);
	VehicleAIAgentStateEngineOff* const engineOffState = new VehicleAIAgentStateEngineOff (this, scene);

	LinkStates (engineOnState, engineOffState, m_engine_off);
	LinkStates (engineOffState, engineOnState, m_engine_on);

	// set this state as the start state
	SetStartState(engineOnState);

	// all vehicles start with the engine off
	GoToState(engineOffState, 0);
}


// this function serve a the vehicle driver, for both Player and NPC
void VehicleAIAgent::Update (dFloat timestep, int threadID)
{
	dMatrix matrix;
	dVector velocity;

	// the position and velocity of vehicle
	BasicVehicle* const vehicle = m_car;
	NewtonBodyGetMatrix(vehicle->m_body, &matrix[0][0]);
	NewtonBodyGetVelocity(vehicle->m_body, &velocity[0]);

	dSoundManager& soundManager = m_scene->GetSoundManager();
	if (m_engineRpmSoundChannel && m_startEngineSoundChannel) {
		//we are mixing start and rpm sound

		dFloat mixingRatio = m_startTime / m_startLength;
		soundManager.SetChannelMixerVolume (m_engineRpmSoundChannel, mixingRatio);
		soundManager.SetChannelMixerVolume (m_startEngineSoundChannel, 1.0f - mixingRatio);

		soundManager.SetChannelVolume (m_engineRpmSoundChannel, m_rpmVolume);
		soundManager.SetChannelLocationAndVelocity(m_engineRpmSoundChannel, matrix.m_posit, velocity);
		soundManager.SetChannelLocationAndVelocity(m_startEngineSoundChannel, matrix.m_posit, velocity);
		m_startTime += timestep;

	} else if (m_engineRpmSoundChannel) {
		// play rpm sound only
		soundManager.SetChannelMixerVolume (m_engineRpmSoundChannel, 1.0f);
		soundManager.SetChannelVolume (m_engineRpmSoundChannel, m_rpmVolume);
		soundManager.SetChannelLocationAndVelocity(m_engineRpmSoundChannel, matrix.m_posit, velocity);
		m_startTime += timestep;
	} 

	CustomWheelVehicle* const vehJoint = vehicle->m_vehicle;
	CustomWheelVehicle::dVehicleEngine* const engine = vehJoint->GetEngine();
	if (m_engineRpmSoundChannel) {
		// set the Gas pedal

		dFloat gasPedal = 0.0f;
		if (m_vehicleIsPlayer) {
//			if (engine->GetGear() == CustomWheelVehicle::dVehicleEngineTransmission::m_vehicleNeutralGear) {
				// if the engine is in neutral this will only rev the engine 
				int mouseX;
				int mouseY;
				const NewtonDemos* const mainWin = m_scene->GetRootWindow();

				mainWin->GetMousePosition (mouseX, mouseY);
				gasPedal = float(mouseY) / m_scene->GetHeight();
				if (gasPedal < 0.0f) {
					gasPedal = 0.0f;
				}
				if (gasPedal > 1.0) {
					gasPedal = 1.0;
				}
				engine->SetGasPedal(1.0f-gasPedal);
//			} else {
//				gasPedal*=1;
//			}
		} else {
			dAssert (0);
		}

		// modulate engine rpm frequency
		dFloat minfreqScale = 0.25f;
		dFloat maxfreqScale = 4.0f;
		dFloat rpm = engine->GetRPM();
		//dFloat minRpm = engine->GetIdleRMP();
		dFloat minRpm = 0.0f;
		dFloat maxRpm = engine->GetMaxRMP();
		dFloat rpmfactor = minfreqScale + (maxfreqScale - minfreqScale) * (rpm - minRpm) / (maxRpm - minRpm);
		if (rpmfactor < minfreqScale) {
			rpmfactor = minfreqScale;
		}
		soundManager.SetChannelFrequenceScale (m_engineRpmSoundChannel, rpmfactor);

		// modulate engine rpm volume  
		dFloat volume = (rpm-minfreqScale) / (maxRpm - minRpm);
		if (volume < 0.1f) {
			volume = 0.1f;
		}
		m_rpmVolume = volume;
	}


	// update the steering wheel
	if (m_vehicleIsPlayer) {
		// driven by player
		int mouseX;
		int mouseY;
		const NewtonDemos* const mainWin = m_scene->GetRootWindow();

		mainWin->GetMousePosition (mouseX, mouseY);
		dFloat steerParam = 2.0f * float(mouseX) / m_scene->GetWidth() - 1.0f;

		CustomWheelVehicle::dVehicleSteering* const steering = vehJoint->GetSteering();
		steering->SetSteering(vehJoint, -steerParam);

		// change gear
		if (m_engineGearUp.IsMouseKeyDown(mainWin, 0)) {
			int gear = engine->GetGear();
			gear ++;
			if (gear == CustomWheelVehicle::dVehicleEngineTransmission::m_vehicleReverseGear) {
				gear ++;
			}
gear = CustomWheelVehicle::dVehicleEngineTransmission::m_vehicleFirstGear;
			engine->SetGear(gear);
		}
		if (m_engineGearDown.IsMouseKeyDown(mainWin, 1)) {
			int gear = engine->GetGear();
			gear --;
gear = CustomWheelVehicle::dVehicleEngineTransmission::m_vehicleFirstGear;
			engine->SetGear(gear);
		}


	} else {
		// driven by AI
		dAssert (0);
	}
}


void VehicleAIAgent::SoundCallback (void* const channelHandle, void* const context, void* const eventHandle)
{
	VehicleAIAgent* const me = (VehicleAIAgent*) context;
	// this sound channel is shooting down
	if (channelHandle == me->m_startEngineSoundChannel) {
		me->m_startEngineSoundChannel = NULL;
	}
}


// vehicle states
void VehicleAIAgentStateEngineOn::Enter(int threadID)
{
	VehicleAIAgent* const vehAgent = (VehicleAIAgent*) GetAgent();
	BasicVehicle* const vehicle = vehAgent->m_car;
	dSoundManager& soundManager = m_scene->GetSoundManager();

	// start playing the start engine sound
	if (!vehAgent->m_startEngineSoundChannel) {
		void* const sound = vehicle->m_engineSounds[BasicVehicle::m_engine_start];
		soundManager.SetSoundPlayMode (sound, false);
		void* const channel = soundManager.CreatePlayChannel(sound);
		soundManager.SetChannelLoops (channel, 1);
		soundManager.PlayChannel(channel);
		soundManager.SetChannelVolume (channel, 1.0);
		soundManager.SetChannelSetUserData (channel, vehAgent);
		soundManager.SetChannelEventCallback (channel, VehicleAIAgent::SoundCallback);
		vehAgent->m_startEngineSoundChannel = channel;
	}

	// if the engine rpm is off start a new channel
	if (!vehAgent->m_engineRpmSoundChannel) {
		void* const sound = vehicle->m_engineSounds[BasicVehicle::m_engine_rpm];
		soundManager.SetSoundPlayMode (sound, true);
		void* const channel = soundManager.CreatePlayChannel(sound);
		soundManager.PlayChannel(channel);
		soundManager.SetChannelVolume (channel, 1.0f);
		vehAgent->m_engineRpmSoundChannel = channel;
	}

	CustomWheelVehicle* const vehJoint = vehicle->m_vehicle;
	CustomWheelVehicle::dVehicleEngine* const engine = vehJoint->GetEngine();
	engine->SetEngineKey(true);
	engine->SetGear (CustomWheelVehicle::dVehicleEngineTransmission::m_vehicleNeutralGear);

	// set the mixer blend factor
	vehAgent->m_startTime = 0.0f;
	vehAgent->m_rpmVolume = 0.1f;
	vehAgent->m_startLength = soundManager.GetSoundlength (vehicle->m_engineSounds[BasicVehicle::m_engine_start]);
}


// the driver is simple waiting for the signal to start racing
void VehicleAIAgentStateEngineOn::Exit(int threadID)
{
	// the position and velocity of vehicle
	VehicleAIAgent* const vehAgent = (VehicleAIAgent*) GetAgent();

	// stops start engine sound if it is playing
	dSoundManager& soundManager = m_scene->GetSoundManager();
	if (vehAgent->m_startEngineSoundChannel) {
		soundManager.StopsChannel (vehAgent->m_startEngineSoundChannel);
		vehAgent->m_startEngineSoundChannel = NULL;
	}

	// stops rpm sound
	if (vehAgent->m_engineRpmSoundChannel) {
		soundManager.StopsChannel (vehAgent->m_engineRpmSoundChannel);
		vehAgent->m_engineRpmSoundChannel = NULL;
	}

	BasicVehicle* const vehicle = vehAgent->m_car;
	CustomWheelVehicle* const vehJoint = vehicle->m_vehicle;
	CustomWheelVehicle::dVehicleEngine* const engine = vehJoint->GetEngine();
	engine->SetEngineKey(false);
	engine->SetGear (CustomWheelVehicle::dVehicleEngineTransmission::m_vehicleNeutralGear);
}
#endif


class DemoVehicleController: public dAIAgentVehicleController
{
	public:
	DemoVehicleController (DemoEntityManager* const scene, CustomWheelVehicle* const vehicle, CarParam& params)
		:dAIAgentVehicleController (scene->GetAI(), vehicle)
		,m_camera(NULL)
	{
		// calculate the binding matrix
		const NewtonBody* const body = vehicle->GetBody0();
		DemoEntity* const entity = (DemoEntity*) NewtonBodyGetUserData(body);
		DemoEntity* const chassis = entity->dHierarchy<DemoEntity>::Find("car_body");
		dAssert (chassis);
		m_bodyBindMatrix = chassis->CalculateGlobalMatrix(entity);
		m_tireAlign = dYawMatrix(90.0f * 3.141592f/180.0f);

		// assign a type information to locate the joint in the callbacks
		vehicle->SetJointID (CUSTOM_VEHICLE_JOINT_ID);

		// save the pointer to this class as the vehicle user data  
		vehicle->SetUserData (this);

		// hook a transform call back for transforming the tires
		NewtonBodySetTransformCallback (vehicle->GetBody0(), TransformCallback);
/*
		// link a destructor so that when the body of the vehicle is destroyed we can destroy this object
		m_vehicle->SetUserDestructorCallback(Destructor);

		// link a user constraint callback to be used as the driver
		dFloat width;
		dFloat radius;
		// calculate the front tire dimensions, and add the tires
		CalculateTireDimensions ("fl_tire", width, radius);
		int leftFrontTireIndex = AddTire ("fl_tire", param.m_tireMass, width, radius, param.m_tireFriction, param.m_suspensionLength, param.m_suspensionSpring, param.m_suspensionDamper);
		int rightFrontTireIndex = AddTire ("fr_tire", param.m_tireMass, width, radius, param.m_tireFriction, param.m_suspensionLength, param.m_suspensionSpring, param.m_suspensionDamper);

		// calculate the rear tire dimensions, and add the tires
		CalculateTireDimensions ("rl_tire", width, radius);
		int leftRearTireIndex = AddTire ("rl_tire", param.m_tireMass, width, radius, param.m_tireFriction, param.m_suspensionLength, param.m_suspensionSpring, param.m_suspensionDamper);
		int rightRearTireIndex = AddTire ("rr_tire", param.m_tireMass, width, radius, param.m_tireFriction, param.m_suspensionLength, param.m_suspensionSpring, param.m_suspensionDamper);

		// Build an engine base of the Lamborghini Diablo specifications
		//1st gear ratio : 2.313:1 
		//2nd gear ratio : 1.524:1 
		//3rd gear ratio : 1.125:1 
		//4th gear ratio : 0.889:1 
		//5th gear ratio :  0.697:1 
		//Rev. gear ratio: 2.125:1 
		//Rear transfer ratio : 1.783:1 
		//Rear differential ratio : 2.529:1 
		//Max. power : 492 bhp at 7000 rpm 

		// this car will be rear wheel drive only
		// create one differentials
		CustomWheelVehicle::dVehicleDifferencial* const differencial = new CustomWheelVehicle::dVehicleDifferencial (leftRearTireIndex, rightRearTireIndex, 2.529f, 0.5f, 0.5f);

		dFloat gearBoxRatios[] = {0.0, -2.125f, 2.313f, 1.524f, 0.889f, 0.697f, 0.697f}; 
		CustomWheelVehicle::dVehicleEngineTransmission* const transmission = new CustomWheelVehicle::dVehicleEngineTransmission(differencial, sizeof (gearBoxRatios)/sizeof (dFloat), gearBoxRatios);
		m_vehicle->SetEngine(new CustomWheelVehicle::dVehicleEngine(transmission, 10000.0f, 1000.0f, 492.0));

		// now set the steering wheel component
		CustomWheelVehicle::dVehicleSteering* const steering = new CustomWheelVehicle::dVehicleSteering (45.0f * 3.141592f / 180.0f);
		steering->AddSteerinWheel(leftFrontTireIndex);
		steering->AddSteerinWheel(rightFrontTireIndex);
		m_vehicle->SetSteering(steering);
*/


		// create the vehicle sound 
		static char* engineSounds[] = {"engine_start.wav", "engine_rpm.wav", "tire_skid.wav"};
		dSoundManager* const soundManager = scene->GetSoundManager();
		for (int i = 0; i < sizeof (engineSounds) / sizeof (engineSounds[0]); i ++) {
			void* sound = soundManager->CreateSound(engineSounds[i]);
			soundManager->SetSoundPlayMode (sound, true);
			m_engineSounds[i] = sound;
		}

		// set a sync point before the end of non looping sounds
		float duration = soundManager->GetSoundlength (m_engineSounds[0]) * 0.95f;
		soundManager->SetSoundEvent (m_engineSounds[0], duration);

		// load texture for displaying the dashboard
		m_gears = LoadTexture ("gears_font.tga");
		m_rpmDial = LoadTexture ("rpm_dial.tga");
		m_rpmNeedle = LoadTexture ("needle_green.tga");
		m_kmhDial = LoadTexture ("kmh_dial.tga");
		m_kmhNeedle = LoadTexture ("needle_red.tga");

	}

	~DemoVehicleController ()
	{
	}

	void Update (dFloat timestep, int threadID)
	{
	}


	static void TransformCallback (const NewtonBody* body, const dFloat* matrix, int threadIndex)
	{
		// set the transform of the main body
		DemoEntity::TransformCallback (body, matrix, threadIndex);


		// find the car joint attached to the body
		DemoVehicleController* car = NULL;
		for (NewtonJoint* joint = NewtonBodyGetFirstJoint(body); joint; joint = NewtonBodyGetNextJoint(body, joint)) {
			CustomJoint* const customJoint = (CustomJoint*) NewtonJointGetUserData (joint);  
			if (customJoint->GetJointID() == CUSTOM_VEHICLE_JOINT_ID) {
				CustomWheelVehicle* const vehicle = (CustomWheelVehicle*) customJoint;
				car = (DemoVehicleController*)vehicle->GetUserData();
				break;
			}
		}

		if (car) {
			NewtonWorld* const world = NewtonBodyGetWorld(body);
			DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world );

			// position all visual tires matrices
			CustomWheelVehicle* const vehicle = car->GetJoint();
			const dMatrix& carMatrix = *((dMatrix*)matrix);
			dMatrix rootMatrixInv (car->m_bodyBindMatrix * carMatrix);
			rootMatrixInv = rootMatrixInv.Inverse();
			int count = vehicle->GetTiresCount();

			// for each tire get the global matrix position, and calculate the local matrix relative to the main body   
			for (int i = 0; i < count; i ++) {
				const CustomWheelVehicle::dTire& tire = vehicle->GetTire (i);

				DemoEntity* const tireNode = (DemoEntity*) tire.m_userData;

				// calculate the tire local matrix
				dMatrix matrix (car->m_tireAlign * vehicle->CalculateTireMatrix(i) * rootMatrixInv);

				//tireNode->SetMatrix(matrix);
				dQuaternion rot (matrix);
				tireNode->SetMatrix (*scene, rot, matrix.m_posit);
			}
		}
	}

	static NewtonBody* CreateChassisBody (DemoEntity* const entity, NewtonWorld* const world, int materialID, CarParam& param)
	{
	dAssert (0);
	return NULL;
/*
		DemoEntity* const chassis = entity->dHierarchy<DemoEntity>::Find("car_body");
		dAssert (chassis);

		DemoMesh* const mesh = chassis->GetMesh();
		float* const vertexList = mesh->m_vertex;
		NewtonCollision* const rootBone = NewtonCreateConvexHull(world, mesh->m_vertexCount, vertexList, 3 * sizeof (float), 0.001f, 0, NULL);

		dMatrix rootBoneOffsset(GetIdentityMatrix());
		NewtonCollision* const chassisCollision = NewtonCreateArticulatedSkeletonCollision(world, 0);
		NewtonArticulatedSkeletonAddBone (chassisCollision, NULL, rootBone, &rootBoneOffsset[0][0]);
		NewtonReleaseCollision(world, rootBone);


		// create the rigid body for this vehicle
		dMatrix matrix (entity->GetCurrentMatrix());
		NewtonBody* const rigidBody = NewtonCreateBody (world, chassisCollision, &matrix[0][0]);

		// save the pointer to the graphic object with the body.
		NewtonBodySetUserData (rigidBody, entity);

		// set the material group id for vehicle
		//	NewtonBodySetMaterialGroupID (m_vehicleBody, vehicleID);
		NewtonBodySetMaterialGroupID (rigidBody, materialID);

		// set a destructor for this rigid body
		NewtonBodySetDestructorCallback (rigidBody, PhysicsBodyDestructor);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (rigidBody, PhysicsApplyGravityForce);

		// set the transform call back function
//		NewtonBodySetTransformCallback (rigidBody, BasicVehicle::TransformCallback);
		NewtonBodySetTransformCallback (rigidBody, DemoEntity::TransformCallback);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (rigidBody, &matrix[0][0]);

		dVector origin(0, 0, 0, 1);
		dVector inertia(0, 0, 0, 1);

		// calculate the moment of inertia and the relative center of mass of the solid
		NewtonConvexCollisionCalculateInertialMatrix (chassisCollision, &inertia[0], &origin[0]);


		// set vehicle mass matrix, 
		dFloat mass = param.m_mass;
		dFloat Ixx = mass * inertia[0];
		dFloat Iyy = mass * inertia[1];
		dFloat Izz = mass * inertia[2];
		NewtonBodySetMassMatrix (rigidBody, mass, Ixx, Iyy, Izz);

		// the cog may be off by a car, we can add some offset here
		// Set the vehicle Center of mass
		// the rear spoilers race the center of mass by a lot for a race car
		// we need to lower some more for the geometrical value of the y axis
		//origin += config.m_comOffset;
		NewtonBodySetCentreOfMass (rigidBody, &origin[0]);

		// do not forget to release the collision after done
		NewtonReleaseCollision(world, chassisCollision);

		return rigidBody;
*/
	}

	static DemoVehicleController* MakeSuperCar (DemoEntityManager* const scene, const char* const name, const dMatrix& location, int materialID, CarParam& params)
	{
		char fileName[2048];
		GetWorkingFileName (name, fileName);
		scene->LoadScene (fileName);

		DemoEntity* carEntity = NULL;
		// find the car visual entity, it should be the last load ent with a name (just a heuristic for this demo not a rule)
		for (DemoEntityManager::dListNode* node = scene->GetLast(); node; node = node->GetPrev()) {
			DemoEntity* const ent = node->GetInfo();
			DemoMesh* const mesh = ent->GetMesh();
			if (mesh) {
				char name[2048];
				mesh->GetName(name);
				if (!strcmp (name, "car_body_mesh")) {
					carEntity = ent->dHierarchy<DemoEntity>::GetRoot();
					break;
				}
			}
		}

		DemoVehicleController* vehicle = NULL;
		if (carEntity) {
			//  place entity in the world
			carEntity->ResetMatrix (*scene, location);

			NewtonWorld* const world = scene->GetNewton();

			// create the car rigid body
			NewtonBody* const body = CreateChassisBody (carEntity, world, materialID, params);

			// create the car joint
			dMatrix chassisMatrix;
			// if your vehicle moves along the z direction you can use 
			// chassisMatrix.m_front = dVector (0.0f, 0.0f, 1.0f, 0.0);
			chassisMatrix.m_front = dVector (1.0f, 0.0f, 0.0f, 0.0f);			// this is the vehicle direction of travel
			chassisMatrix.m_up	  = dVector (0.0f, 1.0f, 0.0f, 0.0f);			// this is the downward vehicle direction
			chassisMatrix.m_right = chassisMatrix.m_front * chassisMatrix.m_up;	// this is in the side vehicle direction (the plane of the wheels)
			chassisMatrix.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);

			// create the newton car joint
			CustomWheelVehicle* const joint = new CustomWheelVehicle (chassisMatrix, body);
			vehicle = new DemoVehicleController (scene, joint, params);
		}
		return vehicle;
	}

	DemoCamera* m_camera;
	dMatrix m_tireAlign;
	dMatrix m_bodyBindMatrix;
//	DemoEntityManager* m_myScene;
	dFloat m_steerAngle;
	void* m_engineSounds[10];
	GLuint m_gears;
	GLuint m_rpmDial;
	GLuint m_rpmNeedle;
	GLuint m_kmhDial;
	GLuint m_kmhNeedle;
};

class AIAgentGameLogic: public DemoAIAgent
{
	public:
	//transitions of the vehicle state machine
	enum 
	{
		m_nextLogicState,
	};

	class CarGameLogicBaseState: public DemoAIState
	{
		public:
		CarGameLogicBaseState (AIAgentGameLogic* const agent, DemoEntityManager* const scene)
			:DemoAIState (agent, scene)
		{
		}

		virtual void Render2DDisplay(DemoVehicleController* const car)
		{
/*
			GLint viewport[4]; 
			glGetIntegerv(GL_VIEWPORT, viewport); 
			int witdh = viewport[2];
			int height = viewport[3];

			glMatrixMode(GL_PROJECTION);
			glPushMatrix();
			glLoadIdentity();
			glOrtho(-witdh/2, witdh/2, -height/2, height/2, 0.0, 1.0);

			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();

			dMatrix origin (GetIdentityMatrix());
			origin.m_posit = dVector (-witdh/2 + 120.0f, -height/2 + 120.0f, 0.0f, 1.0f);
			glLoadIdentity();
			glMultMatrix (&origin[0][0]);

			glDisable(GL_LIGHTING);
			glDisable(GL_DEPTH_TEST);
			glEnable(GL_TEXTURE_2D);	

			glEnable (GL_BLEND);
			glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

			glColor4f(1, 1, 1, 1);

			float x = 100;
			float y = 100;
			float z = 0;

			CustomWheelVehicle::dVehicleEngine* const engine = car->GetJoint()->GetEngine();

			// tachometer
			glBindTexture(GL_TEXTURE_2D, car->m_rpmDial);
			glBegin(GL_QUADS);
			glTexCoord2f(0.0f, 1.0f); glVertex3f(-x,  y, z);
			glTexCoord2f(0.0f, 0.0f); glVertex3f(-x, -y, z);
			glTexCoord2f(1.0f, 0.0f); glVertex3f( x, -y, z);
			glTexCoord2f(1.0f, 1.0f); glVertex3f( x,  y, z);
			glEnd();

			// map engine rpm to a dial angle 
			dFloat minAngle = 180.0f * 3.141592f / 180.0f;
			dFloat maxAngle = -90.0f * 3.141592f / 180.0f;
			dFloat rpm = engine->GetRPM();
			dFloat maxRpm = engine->GetMaxRMP();
			dAssert (rpm >= 0.0f);
			dFloat angle = minAngle + (maxAngle - minAngle) * rpm / maxRpm;
			dMatrix needleMatrix (dRollMatrix (angle));


			glPushMatrix();
			glMultMatrix (&needleMatrix[0][0]);
			glBindTexture(GL_TEXTURE_2D, car->m_rpmNeedle);
			float x1 = 70;
			float y1 = 70;
			glBegin(GL_QUADS);
			glTexCoord2f(0.0f, 1.0f); glVertex3f(-x1,  y1, z);
			glTexCoord2f(0.0f, 0.0f); glVertex3f(-x1, -y1, z);
			glTexCoord2f(1.0f, 0.0f); glVertex3f( x1, -y1, z);
			glTexCoord2f(1.0f, 1.0f); glVertex3f( x1,  y1, z);
			glEnd();
			glPopMatrix();

			// speedometer
			origin.m_posit = dVector (-witdh/2 + 340.0f, -height/2 + 120.0f, 0.0f, 1.0f);
			glLoadIdentity();
			glMultMatrix (&origin[0][0]);

			glBindTexture(GL_TEXTURE_2D, car->m_kmhDial);
			glBegin(GL_QUADS);
			glTexCoord2f(0.0f, 1.0f); glVertex3f(-x,  y, z);
			glTexCoord2f(0.0f, 0.0f); glVertex3f(-x, -y, z);
			glTexCoord2f(1.0f, 0.0f); glVertex3f( x, -y, z);
			glTexCoord2f(1.0f, 1.0f); glVertex3f( x,  y, z);
			glEnd();


			// map engine speed to a dial angle 
			float speed = fabsf (car->GetJoint()->GetSpeed()) * 3.61f; //(mps to kmh)
			minAngle = 180.0 * 3.141592f / 180.0f;
			//dFloat maxAngle = -90.0f * 3.141592f / 180.0f;
			dFloat minSpeed = 0.0f;
			dFloat maxSpeed = 360.0f;
			angle = minAngle + (maxAngle - minAngle) * (speed - minSpeed)/ (maxSpeed - minSpeed);

			needleMatrix = dRollMatrix (angle);

			glPushMatrix();
			glMultMatrix (&needleMatrix[0][0]);
			glBindTexture(GL_TEXTURE_2D, car->m_kmhNeedle);
			glBegin(GL_QUADS);
			glTexCoord2f(0.0f, 1.0f); glVertex3f(-x1,  y1, z);
			glTexCoord2f(0.0f, 0.0f); glVertex3f(-x1, -y1, z);
			glTexCoord2f(1.0f, 0.0f); glVertex3f( x1, -y1, z);
			glTexCoord2f(1.0f, 1.0f); glVertex3f( x1,  y1, z);
			glEnd();
			glPopMatrix();


			// display current Gear
			int gear = engine->GetGear();
			dFloat uwith = 0.1f;
			float u0 = uwith * gear;
			float u1 = u0 + uwith;


			origin.m_posit = dVector (-witdh/2 + 390.0f, -height/2 + 70.0f, 0.0f, 1.0f);
			glLoadIdentity();
			glMultMatrix (&origin[0][0]);
			x1 = 10;
			y1 = 10;
			glColor4f(1, 1, 0, 1);
			glBindTexture(GL_TEXTURE_2D, car->m_gears);
			glBegin(GL_QUADS);
			glTexCoord2f(u0, 1.0f); glVertex3f(-x1,  y1, z);
			glTexCoord2f(u0, 0.0f); glVertex3f(-x1, -y1, z);
			glTexCoord2f(u1, 0.0f); glVertex3f( x1, -y1, z);
			glTexCoord2f(u1, 1.0f); glVertex3f( x1,  y1, z);
			glEnd();

			glColor4f(1, 1, 1, 1);
			glDisable (GL_BLEND);
			glPopMatrix();
			glMatrixMode(GL_PROJECTION);
			glPopMatrix();
*/
		}
	};

	class CountDownGameState: public CarGameLogicBaseState
	{
		public:
		CountDownGameState (AIAgentGameLogic* const agent, DemoEntityManager* const scene, DemoVehicleController* const playerAgent)
			:CarGameLogicBaseState(agent, scene)
			,m_playerAgent(playerAgent)
			,m_counter (10)
			,m_startCount(90)
			,m_timerInSeconds(0)
		{
		}

		void Enter (int threadID)
		{
			dAI* const ai = m_scene->GetAI();
			for (dAIAgent* agent = ai->GetFirstAgent(); agent; agent = ai->GetNextAgent (agent)) {
				agent->GoToState(agent->GetStartState(), threadID);
			}

			// the player start with his engine off
			m_playerAgent->ApplyTransition(dAIAgentVehicleController::m_engineOff, threadID);

			// initialize the counter for the 
			m_counter = 7;
			m_timerInSeconds = 0;
		}

		// call each tick on the current state of this AI Agent 
		void Update (dFloat timestep, int threadID)
		{
			m_timerInSeconds --;
			if (m_timerInSeconds < 0) {
				m_timerInSeconds = m_startCount;
				m_counter --;
				if (m_counter <= 1) {
					m_counter *=1;
					//NewtonAIAgentApplyTransition (m_agent, AIAgentGameLogic::m_nextLogicState, threadID);
					GetAgent()->ApplyTransition (AIAgentGameLogic::m_nextLogicState, threadID);
				}
			}
		}


		virtual void Render2DDisplay(DemoVehicleController* const car)
		{
			// render the hud display
			CarGameLogicBaseState::Render2DDisplay(car);

			// render the count down
			GLint viewport[4]; 
			glGetIntegerv(GL_VIEWPORT, viewport); 
			int witdh = viewport[2];
			int height = viewport[3];

			glMatrixMode(GL_PROJECTION);
			glPushMatrix();
			glLoadIdentity();
			glOrtho(-witdh/2, witdh/2, -height/2, height/2, 0.0, 1.0);

			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();

			dMatrix origin (GetIdentityMatrix());
			origin.m_posit = dVector (-witdh/2 + 120.0f, -height/2 + 120.0f, 0.0f, 1.0f);
			glLoadIdentity();
			glMultMatrix (&origin[0][0]);

			glDisable(GL_LIGHTING);
			glDisable(GL_DEPTH_TEST);
			glEnable(GL_TEXTURE_2D);	

			glEnable (GL_BLEND);
			glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

			glColor4f(1, 1, 1, 1);

			// speedometer
			origin.m_posit = dVector (0, 0, 0.0f, 1.0f);
			glLoadIdentity();
			glMultMatrix (&origin[0][0]);


			// display current Gear
			dFloat uwith = 0.1f;
			float u0 = uwith * m_counter;
			float u1 = u0 + uwith;

			float x1 = 60;
			float y1 = 60;
			glColor4f(1, 1, 0, 1);
			glBindTexture(GL_TEXTURE_2D, car->m_gears);
			glBegin(GL_QUADS);
			glTexCoord2f(u0, 1.0f); glVertex3f(-x1,  y1, 0);
			glTexCoord2f(u0, 0.0f); glVertex3f(-x1, -y1, 0);
			glTexCoord2f(u1, 0.0f); glVertex3f( x1, -y1, 0);
			glTexCoord2f(u1, 1.0f); glVertex3f( x1,  y1, 0);
			glEnd();

			glColor4f(1, 1, 1, 1);
			glDisable (GL_BLEND);

			glPopMatrix();

			glMatrixMode(GL_PROJECTION);
			glPopMatrix();
		}


		int m_counter;
		int m_startCount;
		int m_timerInSeconds;
		DemoVehicleController* m_playerAgent;
	};


	class StartEnginesState: public CarGameLogicBaseState
	{
		public:
		StartEnginesState (AIAgentGameLogic* const agent, DemoEntityManager* const scene, DemoVehicleController* const playerAgent)
			:CarGameLogicBaseState(agent, scene)
			,m_playerAgent (playerAgent)
			,m_playerEngineOn(false)
			,m_engineOnSwitch()
			,m_cameraModeSwitch()
		{
		}

		void Enter (int threadID)
		{
			// go over every AI agent in the Game and send the to the start state
			dAI* const ai = m_scene->GetAI();
			for (dAIAgent* agent = ai->GetFirstAgent(); agent; agent = ai->GetNextAgent (agent)) {
				agent->GoToState(agent->GetStartState(), threadID);
			}

			// send the player to engine of state
			m_playerEngineOn = false;
			m_playerAgent->ApplyTransition(dAIAgentVehicleController::m_engineOff, threadID);
		}

		void Update (dFloat timestep, int threadID)
		{
			const NewtonDemos* const mainWin = m_scene->GetRootWindow();

			if (m_cameraModeSwitch.IsKeyDown(mainWin, ' ')) {
				VehicleCamera* const camera = (VehicleCamera*) m_scene->GetCamera();
				camera->m_toggleFreeCamera = !camera->m_toggleFreeCamera;
			}

			// do the start engine logic
			if (m_engineOnSwitch.IsKeyDown(mainWin, 'I')) {
				m_playerEngineOn = !m_playerEngineOn;
				if (m_playerEngineOn) {
					m_playerAgent->ApplyTransition (dAIAgentVehicleController::m_engineOn, threadID);
				} else {
					m_playerAgent->ApplyTransition (dAIAgentVehicleController::m_engineOff, threadID);
				}
			}
		}

		DemoVehicleController* m_playerAgent;
		bool m_playerEngineOn;
		PushButtonKey m_engineOnSwitch;
		PushButtonKey m_cameraModeSwitch;
	};

	AIAgentGameLogic (DemoEntityManager* const scene, DemoVehicleController* const playerAgent)
		:DemoAIAgent(scene, scene->GetAI()->GetGameLogicNewtonAgent())
		,m_playerAgent(playerAgent)
	{
		// set a function callback for rendering vehicle information
		scene->Set2DDisplayRenderFunction (RenderVehicleHud, playerAgent);

		// for this level we will use the Newton AI engine to implement some game logic
		CountDownGameState* const countdownState = new CountDownGameState (this, scene, playerAgent);
		StartEnginesState* const startEngineState = new StartEnginesState (this, scene, playerAgent);
		LinkStates (countdownState, startEngineState, m_nextLogicState);

		//	NewtonAIAgentConnectStates (gameStateAgent, mainLoopState, endStateState);
		//	NewtonAIAgentConnectStates (gameStateAgent, endStateState, initialState);
		//NewtonAIAgent* xxx0 = NewtonAICreateAgent (ai);
		//NewtonAIAgent* xxx1 = NewtonAICreateAgent (ai);
		//NewtonAIDestroyAgent(xxx0);

		// set this state as the start state
		SetStartState (countdownState);

		// after everything ios Set we go to the start state
		GoToState(countdownState, 0);
	}


	static void RenderVehicleHud (DemoEntityManager* scene, void* context)
	{
		dAI* const ai = scene->GetAI();
		AIAgentGameLogic* const gameLogic = (AIAgentGameLogic*) ai->GetGameLogicAgent();

		CarGameLogicBaseState* const aiState = (CarGameLogicBaseState*)gameLogic->GetCurrentState();
		if (aiState) {
			DemoVehicleController* const player = (DemoVehicleController*) context;
			aiState->Render2DDisplay (player);
		}
	}


	virtual void Update (dFloat timestep, int threadID) {}
	DemoVehicleController* m_playerAgent;
};

#endif


// *************************************************************************************************
// 
//  create a simple racing game with a simple controlled by a Newton AI engine and newton physics
//
// *************************************************************************************************
void SuperCar (DemoEntityManager* const scene)
{
	dAssert (0);
/*
	// load the sky box
	scene->CreateSkyBox();


	NewtonWorld* const world = scene->GetNewton();

	// create the sky box and the floor,
//	CreateLevelMesh (scene, "flatPlane.ngd", true);
//	CreateLevelMesh (scene, "raceTrack1.ngd", true);
//	CreateLevelMesh (scene, "raceTrack2.ngd", false);
	CreateLevelMesh (scene, "raceTrack2.ngd", true);

	// create a material 
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (world);

	// find the start point
//	AddBoxes(system, 10.0f, location, size, 3, 3, 10.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddBoxes(system, 10.0f, location, size, 3, 3, 10.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddBoxes(system, 10.0f, location, size, 3, 3, 10.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddBoxes(system, 10.0f, location, size, 3, 3, 10.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddBoxes(system, 10.0f, location, size, 3, 3, 10.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddBoxes(system, 10.0f, location, size, 3, 3, 10.0f, _CHAMFER_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddBoxes(system, 10.0f, location, size, 3, 3, 10.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddBoxes(system, 10.0f, location, size, 3, 3, 10.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);

	dMatrix matrix (dYawMatrix( 90.0f * 3.1416f/ 180.0f));
	
	matrix.m_posit = dVector (126.0f, 4.0f, 50.0f, 1.0f);
	CarParam viperParam (VIPER_MASS, VIPER_TIRE_TYRE_MASS, VIPER_TIRE_FRICTION, VIPER_SUSPENSION_SPRING, VIPER_SUSPENSION_DAMPER, VIPER_SUSPENSION_LENGTH);
	DemoVehicleController* const viper = DemoVehicleController::MakeSuperCar (scene, "viper.ngd", matrix, defaultMaterialID, viperParam); 
	
	// add a driver for this vehicle
//	VehicleAIAgent* const playerAgent = new VehicleAIAgent (scene, viper);
	// make this the player vehicle 
//	playerAgent->m_vehicleIsPlayer = true;
	

	// link the camera to the player
	// change the Camera to a third person viewer attach to the player vehicle
	ThirdPersonCamera* const camera = new ThirdPersonCamera ((DemoEntity*) NewtonBodyGetUserData (viper->GetJoint()->GetBody0()));
	viper->m_camera = camera;
	scene->SetNewCamera(camera);


//	matrix.m_posit = dVector (122.0f, 4.0f, 40.0f, 1.0f);
//	CarParam lamborghiniParam (LAMBORGHINI_MASS, LAMBORGHINI_TIRE_TYRE_MASS, LAMBORGHINI_TIRE_FRICTION, LAMBORGHINI_SUSPENSION_SPRING, LAMBORGHINI_SUSPENSION_DAMPER, LAMBORGHINI_SUSPENSION_LENGTH);
//	BasicVehicle* const lamborghini = MakeSuperCar (scene, "lambDiablo.ngd", matrix, defaultMaterialID, lamborghiniParam); 
//	lamborghini->m_camera = NULL;
	
	dQuaternion rot (dYawMatrix(90.0f * 3.1416f / 180.0f));
	dVector origin (126.0f, 5.0f, 60.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);

	// configure Game Logic AI agent
	new AIAgentGameLogic (scene, viper);
*/
}


