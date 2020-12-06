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
#include "ndTargaToOpenGl.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndLoadFbxMesh.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

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
	{
		//fbxDemoEntity* const vehicleEntity = LoadFbxMesh("viper.fbx");
		fbxDemoEntity* const vehicleEntity = LoadFbxMesh("viper1.fbx");
		vehicleEntity->ResetMatrix(*scene, vehicleEntity->CalculateGlobalMatrix() * matrix);
		vehicleEntity->BuildRenderMeshes(scene);
		scene->AddEntity(vehicleEntity);

		ndWorld* const world = scene->GetWorld();

		// create the vehicle chassis as a normal rigid body
		ndBodyDynamic* const chassis = CreateChassis(scene, vehicleEntity);

		// lower vehicle com;
		dVector com(chassis->GetCentreOfMass());
		com -= m_localFrame.m_up.Scale(0.25f);
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
		tireInfo.m_maxLimit = 0.2f;
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
	}

	void SetAsPlayer(ndDemoEntityManager* const scene)
	{
		scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
	}

	private:
	ndBodyDynamic* CreateChassis(ndDemoEntityManager* const scene, fbxDemoEntity* const chassisEntity)
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
		body->SetGyroMode(false);

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

		dFloat32 brake = 1000.0f * dFloat32(scene->GetKeyState('S'));
		dFloat32 handBrake = 1000.0f * dFloat32(scene->GetKeyState(' '));
		dFloat32 steerAngle = 35.0f * (dFloat32(scene->GetKeyState('A')) - dFloat32(scene->GetKeyState('D')));
		m_steerAngle = m_steerAngle + (steerAngle - m_steerAngle) * 0.15f;

		SetBrakeTorque(brake);
		SetHandBrakeTorque(handBrake);
		SetSteeringAngle(m_steerAngle * dDegreeToRad);

		ndMultiBodyVehicle::Update(world, timestep);
	}

	virtual dFloat32 GetFrictionCoeficient(const ndJointWheel* const tire, const ndContactMaterial& contactPoint) const
	{
		return dFloat32(1.0f);
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

	dFloat32 m_steerAngle;
};

void ndBasicVehicle (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene);

	dVector location(0.0f, 1.0f, 0.0f, 1.0f);

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = location;

	ndBasicMultiBodyVehicle* const vehicle = new ndBasicMultiBodyVehicle(scene, matrix);
	scene->GetWorld()->AddModel(vehicle);
	vehicle->SetAsPlayer(scene);

	dQuaternion rot;
	//dVector origin(-80.0f, 5.0f, 0.0f, 0.0f);
	dVector origin(-10.0f, 2.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
