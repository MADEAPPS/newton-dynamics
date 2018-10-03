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
		}

		void OnRender(dFloat timestep) const
		{
		}

		void OnInterpolateMatrix(DemoEntityManager& world, dFloat param) const
		{
			dVehicleInterface* const vehicle = m_vehicleChassis->GetVehicle();
			for (dList<dVehicleNode*>::dListNode* node = vehicle->m_children.GetFirst(); node; node = node->GetNext()) {
				dVehicleTireInterface* const tire = node->GetInfo()->GetAsTire();
				if (tire) {
					DemoEntity* const tireMesh = (DemoEntity*)tire->GetUserData();
					tireMesh->InterpolateMatrixUsafe(param);
				}
			}
		}

		void OnTransformCallback(DemoEntityManager& world) const
		{
			// calculate tire Matrices
			dVehicleInterface* const vehicle = m_vehicleChassis->GetVehicle();
			dMatrix chassisMatrixInv(vehicle->GetMatrix().Inverse());
			for (dList<dVehicleNode*>::dListNode* node = vehicle->m_children.GetFirst(); node; node = node->GetNext()) {
				dVehicleTireInterface* const tire = node->GetInfo()->GetAsTire();
				if (tire) {
					DemoEntity* const tireMesh = (DemoEntity*)tire->GetUserData();
					dMatrix tireMatrix(tire->GetGlobalMatrix() * chassisMatrixInv);
					dQuaternion rotation(tireMatrix);
					tireMesh->SetMatrixUsafe(rotation, tireMatrix.m_posit);
				}
			}
		}

		dVehicleChassis* m_vehicleChassis;
	};

	SingleBodyVehicleManager(NewtonWorld* const world)
		:dVehicleManager(world)
	{
	}

	~SingleBodyVehicleManager()
	{
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
		dVector* const temp = new dVector[tireMesh->m_vertexCount];
		meshMatrix.TransformTriplex(&temp[0].m_x, sizeof (dVector), tireMesh->m_vertex, 3 * sizeof (dFloat), tireMesh->m_vertexCount);
		NewtonCollision* const collision = NewtonCreateConvexHull(world, tireMesh->m_vertexCount, &temp[0].m_x, sizeof (dVector), 0, 0, NULL);
		delete[] temp;

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

		dFloat* const temp = new dFloat[mesh->m_vertexCount * 3];
		meshMatrix.TransformTriplex(&temp[0], 3 * sizeof (dFloat), mesh->m_vertex, 3 * sizeof (dFloat), mesh->m_vertexCount);
		NewtonCollision* const shape = NewtonCreateConvexHull(world, mesh->m_vertexCount, &temp[0], 3 * sizeof (dFloat), 0.001f, 0, NULL);
		delete[] temp;
		return shape;
	}

	DemoEntity* const LoadModel_obj(const char* const modelName)
	{
		dMatrix scale(dGetIdentityMatrix());
		scale[0][0] = 1.0f / 40.0f;
		scale[1][1] = 1.0f / 40.0f;
		scale[2][2] = 1.0f / 40.0f;
		scale = scale * dPitchMatrix(-dPi * 0.5f) * dYawMatrix(-dPi * 0.5f);

		char name[1024];
		sprintf(name, "%s_chassis.obj", modelName);
		DemoEntity* const chassisEntity = DemoEntity::LoadOBJ_mesh(name, GetWorld(), scale);
		chassisEntity->SetNameID("car_body");

		return chassisEntity;
	}

	DemoEntity* const LoadModel_ndg(const char* const modelName)
	{
		DemoEntity* const entity = DemoEntity::LoadNGD_mesh(modelName, GetWorld());
		return entity;
	}

	DemoEntity* const LoadVisualModel (const char* const modelName)
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);
		DemoEntity* const entity = LoadModel_ndg(modelName);
		//DemoEntity* const entity = LoadModel_obj(modelName);
		dAssert (entity);
		scene->Append(entity);
		return entity;
	}

	//dVehicleTireInterface* AddTire(dFloat width, dFloat radius, dFloat pivotOffset, dFloat maxSteerAngle, const CarDefinition& definition)
	dVehicleTireInterface* AddTire(dVehicleChassis* const vehicle, const char* const tireName, dFloat width, dFloat radius)
	{
		DemoEntity* const entity = (DemoEntity*)vehicle->GetVehicle()->GetUserData();
		DemoEntity* const tirePart = entity->Find(tireName);

		// save the controller with the tire so that we can use it a callback 
//		TireVehControllerSaved* const savedVehController = new TireVehControllerSaved(m_controller);
//		tirePart->SetUserData(savedVehController);

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

		//tireInfo.m_pivotOffset = pivotOffset;
		//tireInfo.m_maxSteeringAngle = maxSteerAngle * dDegreeToRad;
		//tireInfo.m_dampingRatio = definition.m_tireSuspensionDamperConstant;
		//tireInfo.m_springStrength = definition.m_tireSuspensionSpringConstant;
		//tireInfo.m_suspensionLength = definition.m_tireSuspensionLength;
		//tireInfo.m_corneringStiffness = definition.m_corneringStiffness;
		//tireInfo.m_aligningMomentTrail = definition.m_tireAligningMomemtTrail;
		//tireInfo.m_hasFender = definition.m_wheelHasCollisionFenders;
		//tireInfo.m_suspentionType = definition.m_tireSuspensionType;

		dVehicleTireInterface* const tire = vehicle->AddTire(tireMatrix, tireInfo);
		// add the user data and a tire transform callback that calculate the tire local space matrix
		//NewtonBodySetUserData(tireBody, tirePart);
		//NewtonBodySetTransformCallback(tireBody, TireTransformCallback);
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
		dVehicleTireInterface* const frontLeft = AddTire(vehicle, "fl_tire", width, radio);
		dVehicleTireInterface* const frontRight = AddTire(vehicle, "fr_tire", width, radio);

		CalculateTireDimensions ("rl_tire", width, radio, world, vehicleEntity);
		dVehicleTireInterface* const rearLeft = AddTire(vehicle, "rl_tire", width, radio);
		dVehicleTireInterface* const rearRight = AddTire(vehicle, "rr_tire", width, radio);

		// do not forget to call finalize after all components are added or after any change is made to the vehicle
		vehicle->Finalize();
		
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
	//dVehicleChassis* const player = manager->CreateVehicle("porche918", location);
	dVehicleChassis* const player = manager->CreateVehicle("viper.ngd", location);

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

