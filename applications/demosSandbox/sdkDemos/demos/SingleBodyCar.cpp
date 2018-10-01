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
	SingleBodyVehicleManager(NewtonWorld* const world)
		:dVehicleManager(world)
	{
	}

	~SingleBodyVehicleManager()
	{
	}

	void BindVehicleAndVisualEntity (dVehicleChassis* const chassis, DemoEntity* const chassisEntity)
	{
		NewtonBody* const chassiBody = chassis->GetBody();

		// set the user data
		NewtonBodySetUserData(chassiBody, chassisEntity);

		// set the transform callback
		NewtonBodySetTransformCallback(chassiBody, DemoEntity::TransformCallback);

		dVehicleInterface* const vehicle = chassis->GetVehicle();
		dMatrix chassisMatrixInv (vehicle->GetMatrix().Inverse());
		//DemoEntity* const chassisEntity = (DemoEntity*)NewtonBodyGetUserData(chassis->GetBody());

		for (dList<dVehicleNode*>::dListNode* node = vehicle->m_children.GetFirst(); node; node = node->GetNext()) {
			dVehicleTireInterface* const tire = node->GetInfo()->GetAsTire();
			if (tire) {
				dMatrix tireMatrix (tire->GetGlobalMatrix() * chassisMatrixInv);

				NewtonCollision* const chassisCollision = tire->GetCollisionShape();
				DemoMesh* const tireMesh = new DemoMesh("chassis", chassisCollision, "metal_30.tga", "metal_30.tga", "metal_30.tga");
				DemoEntity* const tireEntity = new DemoEntity(tireMatrix, chassisEntity);
				tireEntity->SetMesh(tireMesh, dGetIdentityMatrix());
				tireMesh->Release();
			}
		}
	}
	
	DemoEntity* const LoadVisualModel (const char* const modelName)
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		char pathName[2048];
		char chassisName [256];
		sprintf (chassisName, "%s_chassis.obj", modelName);
		//sprintf (chassisName, "%s_frontRightTire.obj", modelName);
		dGetWorkingFileName(chassisName, pathName);

		dNewtonMesh newtonMesh (world);
		char chassisMaterialLibrary[1024];
		newtonMesh.LoadObjFile(pathName, chassisMaterialLibrary);

		dMatrix scale (dGetIdentityMatrix());
		scale[0][0] = 1.0f/40.0f;
		scale[1][1] = 1.0f/40.0f;
		scale[2][2] = 1.0f/40.0f;
		scale = scale * dPitchMatrix(-dPi * 0.5f) * dYawMatrix(-dPi * 0.5f);
		newtonMesh.ApplyTransform (&scale[0][0]);
		newtonMesh.CalculateVertexNormals(45.0f * dDegreeToRad);

		DemoMesh* const chassinMesh = new DemoMesh(newtonMesh.GetMesh());
		DemoEntity* const chassisEntity = new DemoEntity(dGetIdentityMatrix(), NULL);
		chassisEntity->SetMesh(chassinMesh, dGetIdentityMatrix());
		chassinMesh->Release();
		scene->Append(chassisEntity);

		chassisEntity->SetNameID("car_body");



		return chassisEntity;
	}

	NewtonCollision* CreateChassisCollision(const DemoEntity* const carEntity, NewtonWorld* const world) const
	{
		DemoEntity* const chassis = carEntity->Find("car_body");
		dAssert(chassis);

		DemoMesh* const mesh = (DemoMesh*)chassis->GetMesh();
		dAssert(mesh->IsType(DemoMesh::GetRttiType()));
		//dAssert (chassis->GetMeshMatrix().TestIdentity());
		const dMatrix& meshMatrix = chassis->GetMeshMatrix();

		dFloat* const temp = new dFloat[mesh->m_vertexCount * 3];
		meshMatrix.TransformTriplex(&temp[0], 3 * sizeof (dFloat), mesh->m_vertex, 3 * sizeof (dFloat), mesh->m_vertexCount);
		NewtonCollision* const shape = NewtonCreateConvexHull(world, mesh->m_vertexCount, &temp[0], 3 * sizeof (dFloat), 0.001f, 0, NULL);
		delete[] temp;
		return shape;
	}


	dVehicleChassis* CreateVehicle(const char* const carModelName, const dMatrix& location)
	{
		NewtonWorld* const world = GetWorld();
		
		DemoEntity* const carEntity = LoadVisualModel (carModelName);

		// make chassis collision shape;
		//int chassisVertexCount = sizeof(chassisShape) / (3 * sizeof(chassisShape[0]));
		NewtonCollision* const chassisCollision = CreateChassisCollision(carEntity, world);

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

		// destroy chassis collision shape 
		NewtonDestroyCollision(chassisCollision);

		// add Tires
		dVector tireLocation(location.m_posit);
		dVehicleTireInterface::dTireInfo tireInfo;
	
		tireInfo.m_mass = 25.0f;
		tireInfo.m_radio = 0.33f;
		tireInfo.m_width = 0.36f;
		tireInfo.m_pivotOffset = 0.0f;
	
		dVehicleTireInterface* const frontLeft = vehicle->AddTire(tireLocation + dVector (1.2f, 0.0f, -0.8f, 0.0f), tireInfo);
		dVehicleTireInterface* const frontRight = vehicle->AddTire(tireLocation + dVector (1.2f, 0.0f,  0.8f, 0.0f), tireInfo);

		dVehicleTireInterface* const rearLeft = vehicle->AddTire(tireLocation + dVector(-1.2f, 0.0f, -0.8f, 0.0f), tireInfo);
		dVehicleTireInterface* const rearRight = vehicle->AddTire(tireLocation + dVector(-1.2f, 0.0f, 0.8f, 0.0f), tireInfo);

		// create the visual representation from the collision shapes
		BindVehicleAndVisualEntity (vehicle, carEntity);

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
	dVehicleChassis* const player = manager->CreateVehicle("porche918", location);

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

