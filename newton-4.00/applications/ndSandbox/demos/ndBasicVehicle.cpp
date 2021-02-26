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

/*
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
*/

static void AddSomeObstacles(ndDemoEntityManager* const scene, const dVector& origin)
{
/*
	dFloat32 diameter = 1.0f;
	ndShapeInstance shape(new ndShapeCapsule(diameter * 0.5f, diameter * 0.5f, diameter * 1.0f));
	ndDemoMeshIntance* const instanceMesh = new ndDemoMeshIntance("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga");

	ndDemoInstanceEntity* const rootEntity = new ndDemoInstanceEntity(instanceMesh);
	scene->AddEntity(rootEntity);

	const dInt32 n = 1;
	const dInt32 stackHigh = 1;
	//const dInt32 n = 2;
	//const dInt32 stackHigh = 7;
	for (dInt32 i = 0; i < n; i++)
	{
		for (dInt32 j = 0; j < n; j++)
		{
			dVector location((j - n / 2) * 4.0f, 0.0f, (i - n / 2) * 4.0f, 0.0f);
			AddShape(scene, rootEntity, shape, 10.0f, location + origin, diameter, stackHigh);
		}
	}

	instanceMesh->Release();
*/
}


static void PlaceRampRamp(ndDemoEntityManager* const scene, 
	const dMatrix& location, ndDemoMesh* const geometry, ndShapeInstance ramp)
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

	void OnTranform(dInt32 threadIndex, const dMatrix& matrix)
	{
		dMatrix parentMatrix(m_chassis->GetMatrix());
		dMatrix localMatrix(matrix * parentMatrix.Inverse());

		dQuaternion rot(localMatrix);
		dScopeSpinLock lock(m_entity->GetLock());
		m_entity->SetMatrixUsafe(rot, localMatrix.m_posit);
	}

	ndBodyDynamic* m_chassis;
};

class ndBasicMultiBodyVehicle : public ndMultiBodyVehicle
{
	public:
	ndBasicMultiBodyVehicle(ndDemoEntityManager* const scene, const dMatrix& matrix)
		:ndMultiBodyVehicle(dVector(1.0f, 0.0f, 0.0f, 0.0f), dVector(0.0f, 1.0f, 0.0f, 0.0f))
		,m_steerAngle(0.0f)
		,m_prevKey(false)
	{
		//ndDemoEntity* const vehicleEntity = LoadMeshModel(scene, "viper.fbx");
		ndDemoEntity* const vehicleEntity = LoadMeshModel(scene, "viper1.fbx");
		vehicleEntity->ResetMatrix(*scene, vehicleEntity->CalculateGlobalMatrix() * matrix);

		ndWorld* const world = scene->GetWorld();

		// create the vehicle chassis as a normal rigid body
		ndBodyDynamic* const chassis = CreateChassis(scene, vehicleEntity);

		// lower vehicle com;
		dVector com(chassis->GetCentreOfMass());
		com -= m_localFrame.m_up.Scale(0.35f);
		com += m_localFrame.m_front.Scale(0.25f);
		chassis->SetCentreOfMass(com);

		// create the tire chassis as a normal rigid body
		ndBodyDynamic* const rr_tire_body = CreateTireBody(scene, chassis, "rr_tire");
		ndBodyDynamic* const rl_tire_body = CreateTireBody(scene, chassis, "rl_tire");
		ndBodyDynamic* const fr_tire_body = CreateTireBody(scene, chassis, "fr_tire");
		ndBodyDynamic* const fl_tire_body = CreateTireBody(scene, chassis, "fl_tire");
		
		// 1- add chassis to the vehicle mode 
		AddChassis(chassis);

		// 2- each tire to the mode, this function will create the tire joints

		ndJointWheel::ndWheelDescriptor tireInfo;
		tireInfo.m_springK = 5000.0f;
		tireInfo.m_damperC = 100.0f;
		tireInfo.m_minLimit = -0.05f;
		//tireInfo.m_maxLimit = 0.2f;
		tireInfo.m_maxLimit = 0.1f;
		tireInfo.m_laterialStiffeness = 1.0f;
		tireInfo.m_longitudinalStiffeness = 1.0f;

		ndJointWheel* const rr_tire = AddTire(world, tireInfo, rr_tire_body);
		ndJointWheel* const rl_tire = AddTire(world, tireInfo, rl_tire_body);
		ndJointWheel* const fr_tire = AddTire(world, tireInfo, fr_tire_body);
		ndJointWheel* const fl_tire = AddTire(world, tireInfo, fl_tire_body);

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
		ndMultiBodyVehicleDifferential* const differential = AddDifferential(world, 20.0f, 0.25f, rl_tire, rr_tire);

		// add a motor
		AddMotor(world, 20, 0.25f, differential);
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
		scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
	}

	static void RenderUI(ndDemoEntityManager* const scene, void* const context)
	{
		ndBasicMultiBodyVehicle* const me = (ndBasicMultiBodyVehicle*)context;
		me->RenderUI(scene);
	}

	private:
	ndBodyDynamic* CreateChassis(ndDemoEntityManager* const scene, ndDemoEntity* const chassisEntity)
	{
		dFloat32 mass = 1000.0f;
		dMatrix matrix(chassisEntity->CalculateGlobalMatrix(nullptr));

		ndWorld* const world = scene->GetWorld();
		ndShapeInstance* const chassisCollision = chassisEntity->CreateCollisionFromchildren(scene->GetWorld());

		ndBodyDynamic* const body = new ndBodyDynamic();
		body->SetNotifyCallback(new ndDemoEntityNotify(scene, chassisEntity));
		body->SetMatrix(matrix);
		body->SetCollisionShape(*chassisCollision);
		body->SetMassMatrix(mass, *chassisCollision);
		//body->SetGyroMode(true);
		//body->SetGyroMode(false);

		world->AddBody(body);
		delete chassisCollision;
		return body;
	}

	void CalculateTireDimensions(const char* const tireName, dFloat32& width, dFloat32& radius, ndWorld* const world, ndDemoEntity* const vehEntity)
	{
		// find the the tire visual mesh 
		ndDemoEntity* const tirePart = vehEntity->Find(tireName);
		dAssert(tirePart);

		// make a convex hull collision shape to assist in calculation of the tire shape size
		ndDemoMesh* const tireMesh = (ndDemoMesh*)tirePart->GetMesh();

		dArray<dVector> temp;
		tireMesh->GetVertexArray(temp);

		dVector minVal(1.0e10f);
		dVector maxVal(-1.0e10f);
		for (dInt32 i = 0; i < temp.GetCount(); i++)
		{
			minVal = minVal.GetMin(temp[i]);
			maxVal = maxVal.GetMax(temp[i]);
		}

		dVector size(maxVal - minVal);
		width = size.m_y;
		radius = size.m_x * 0.5f;
	}

	ndBodyDynamic* CreateTireBody(ndDemoEntityManager* const scene, ndBodyDynamic* const chassis, const char* const tireName)
	{
		//rr_tire
		dFloat32 width;
		dFloat32 radius;
		ndWorld* const world = scene->GetWorld();
		ndDemoEntity* const chassisEntity = (ndDemoEntity*)chassis->GetNotifyCallback()->GetUserData();
		CalculateTireDimensions(tireName, width, radius, world, chassisEntity);

		dFloat32 mass(20.0f);
		ndShapeInstance tireCollision(CreateTireShape(radius, width));

		ndDemoEntity* const tireEntity = chassisEntity->Find(tireName);
		dMatrix matrix(tireEntity->CalculateGlobalMatrix(nullptr));

		ndBodyDynamic* const tireBody = new ndBodyDynamic();
		tireBody->SetNotifyCallback(new ndTireNotifyNotify(scene, tireEntity, chassis));
		tireBody->SetMatrix(matrix);
		tireBody->SetCollisionShape(tireCollision);
		tireBody->SetMassMatrix(mass, tireCollision);
		//tireBody->SetGyroMode(false);

		world->AddBody(tireBody);
		return tireBody;
	}

	void Update(const ndWorld* const world, dFloat32 timestep)
	{
		ndDemoEntityManager* const scene = ((ndPhysicsWorld*)world)->GetManager();

		bool start = scene->GetKeyState('I');
		dFloat32 brake = 1500.0f * dFloat32(scene->GetKeyState('S'));
		dFloat32 handBrake = 1500.0f * dFloat32(scene->GetKeyState(' '));
		dFloat32 throttle = dFloat32 (scene->GetKeyState('W')) ? 1.0f : 0.0f;
		dFloat32 steerAngle = 35.0f * (dFloat32(scene->GetKeyState('A')) - dFloat32(scene->GetKeyState('D')));
     		m_steerAngle = m_steerAngle + (steerAngle - m_steerAngle) * 0.15f;

		static dInt32 xxxx;
 		xxxx++;
		if (xxxx > 300)
		{
			start = true;
			throttle = 0.25f;
		}

		SetBrakeTorque(brake);
		SetHandBrakeTorque(handBrake);
		SetSteeringAngle(m_steerAngle * dDegreeToRad);

		if (!m_prevKey & start)
		{
			m_rotor->SetStart (!m_rotor->GetStart());
		}
		m_prevKey = start;

		m_rotor->SetThrottle(throttle);

		ndMultiBodyVehicle::Update(world, timestep);
	}

	virtual dFloat32 GetFrictionCoeficient(const ndJointWheel* const tire, const ndContactMaterial& contactPoint) const
	{
		return dFloat32(2.0f);
	}

	static void UpdateCameraCallback(ndDemoEntityManager* const manager, void* const context, dFloat32 timestep)
	{
		ndBasicMultiBodyVehicle* const me = (ndBasicMultiBodyVehicle*)context;
		me->SetCamera(manager, timestep);
	}

	void SetCamera(ndDemoEntityManager* const manager, dFloat32 timestep)
	{
		ndDemoCamera* const camera = manager->GetCamera();
		ndDemoEntity* const chassisEntity = (ndDemoEntity*)m_chassis->GetNotifyCallback()->GetUserData();
		dMatrix camMatrix(camera->GetNextMatrix());
		dMatrix playerMatrix(chassisEntity->GetNextMatrix());

		dVector frontDir(camMatrix[0]);
		dVector camOrigin(0.0f);
		camOrigin = playerMatrix.m_posit + dVector(0.0f, 1.0f, 0.0f, 0.0f);
		camOrigin -= frontDir.Scale(10.0f);

		camera->SetNextMatrix(*manager, camMatrix, camOrigin);
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

	void RenderUI(ndDemoEntityManager* const scene)
	{
		//dAssert(0);

		//dMultiBodyVehicleEngine* const engine = m_player->GetEngineControl() ? m_player->GetEngineControl()->GetEngine() : NULL;

		ndMultiBodyVehicleRotor* const motor = m_rotor;
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
		DrawGage(m_odometer, m_greenNeedle, speed, x, y, gageSize, -180.0f, 90.0f);
	}

	dFloat32 m_steerAngle;

	GLuint m_gears;
	GLuint m_odometer;
	GLuint m_redNeedle;
	GLuint m_tachometer;
	GLuint m_greenNeedle;

	bool m_prevKey;
};

void ndBasicVehicle (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene);

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

	ndBasicMultiBodyVehicle* const vehicle = new ndBasicMultiBodyVehicle(scene, matrix);
	scene->GetWorld()->AddModel(vehicle);
	vehicle->SetAsPlayer(scene);

	scene->Set2DDisplayRenderFunction(nullptr, ndBasicMultiBodyVehicle::RenderUI, vehicle);

	location.m_z = 4.0f;
	location.m_y = 0.5f;
	AddRamps(scene, location);

	AddSomeObstacles(scene, location);

	dQuaternion rot;
	dVector origin(-10.0f, 2.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
