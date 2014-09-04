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
#include "HeightFieldPrimitive.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "../toolBox/DebugDisplay.h"

#if 0
/*
#include "CustomWheelVehicle.h"
class TireUserData: public DemoEntity::UserData
{
	public:
	TireUserData(CustomWheelVehicle* const carJoint, DemoEntity* const me, void* const tireHandle)
		:UserData()
		,m_tireHandle (tireHandle)
		,m_me(me)
		,m_carJoint(carJoint)
	{
	}

	virtual ~TireUserData()
	{
	}

	virtual void OnRender (dFloat timestep) const
	{
	}


	virtual void OnInterpolateMatrix (DemoEntityManager& world, dFloat param) const
	{
		// iterate over all tires and calculate the interpolated matrix between current and next physics frame at parameter param.
//		CustomWheelVehicle* const vehicle = m_car->m_vehicle;
//		int count = vehicle->GetTiresCount();
//		for (int i = 0; i < count; i ++) {
//			const CustomWheelVehicle::dTire& tire = vehicle->GetTire (i);
//			DemoEntity* const tireNode = (DemoEntity*) tire.m_userData;
//			tireNode->InterpolateMatrix (world, param);
//		}
	}

	void* m_tireHandle;
	DemoEntity* m_me;
	CustomWheelVehicle* m_carJoint;

};

*/



// car need to apply local transform to all it children
static void TransformCallback (const NewtonBody* body, const dFloat* matrix, int threadIndex)
{
	DemoEntity::TransformCallback (body, matrix, threadIndex);

	
	for (NewtonJoint* joint = NewtonBodyGetFirstJoint (body); joint; joint = NewtonBodyGetNextJoint(body, joint)) {
		NewtonCustomJoint* const customJoint = (NewtonCustomJoint*) NewtonJointGetUserData(joint);
		if (customJoint->IsType(CustomWheelVehicle::GetRttiType())) {

			dMatrix chassisInvMatrix (dMatrix(matrix).Inverse());
			//static dMatrix tireAligmentRotation (dYawMatrix(3.121592f * 0.0f));
			dMatrix tireAligmentRotation (GetIdentityMatrix());

			CustomWheelVehicle* const vehicleJoint = (CustomWheelVehicle*) customJoint;
			for (void* handle = vehicleJoint->GetFirstTireHandle(); handle; handle = vehicleJoint->GetNextTireHandle(handle)) {
				NewtonBody* const tireBody = vehicleJoint->GetTireBody(handle);
				dMatrix tireGlobalMatrix;
				NewtonBodyGetMatrix(tireBody, &tireGlobalMatrix[0][0]);
				dMatrix tireLocalMatrix (tireAligmentRotation * tireGlobalMatrix * chassisInvMatrix);
				DemoEntity::TransformCallback (tireBody, &tireLocalMatrix[0][0], threadIndex);
			}
		}
	}
}


static NewtonBody* CreateVehicleBody (DemoEntityManager* const scene, const char* const name, const dMatrix& location)
{
	dAssert (0);
	return 0;
/*
	char fileName[2048];
	GetWorkingFileName (name, fileName);
	scene->LoadScene (fileName);

	dMatrix matrix (location);

	NewtonWorld* const world = scene->GetNewton();

	matrix.m_posit = FindFloor (scene->GetNewton(), dVector (location.m_posit.m_x, 100.0f, location.m_posit.m_z, 0.0f), 200.0f);
	matrix.m_posit.m_y += 4.0f;

	DemoEntity* const entity = scene->GetLast()->GetInfo();
	DemoEntity* const chassis = entity->dHierarchy<DemoEntity>::Find("chassis");
	dAssert (chassis);
	dMatrix bodyBindMatrix = chassis->CalculateGlobalMatrix(entity);

	DemoMesh* const mesh = chassis->GetMesh();
	float* const vertexList = mesh->m_vertex;
	NewtonCollision* chassisCollision = NewtonCreateConvexHull(world, mesh->m_vertexCount, vertexList, 3 * sizeof (float), 0.001f, 0, NULL);

	// create the rigid body for this vehicle
	NewtonBody* const rigidBody = NewtonCreateBody (world, chassisCollision, &matrix[0][0]);

	// do not forget to release the collision after done
	NewtonDestroyCollision (chassisCollision);

	// get the collision from the rigid body
	chassisCollision = NewtonBodyGetCollision(rigidBody);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (rigidBody, entity);

	// set the material group id for vehicle
	//	NewtonBodySetMaterialGroupID (m_vehicleBody, vehicleID);
	int materialID = 0;
	NewtonBodySetMaterialGroupID (rigidBody, materialID);

	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (rigidBody, PhysicsBodyDestructor);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback (rigidBody, PhysicsApplyGravityForce);

	// set the transform call back function
	//NewtonBodySetTransformCallback (rigidBody, DemoEntity::TransformCallback);
	NewtonBodySetTransformCallback (rigidBody, TransformCallback);
	

	// set the matrix for both the rigid body and the graphic body
	NewtonBodySetMatrix (rigidBody, &matrix[0][0]);

	dVector origin(0, 0, 0, 1);
	dVector inertia(0, 0, 0, 1);

	// calculate the moment of inertia and the relative center of mass of the solid
	NewtonConvexCollisionCalculateInertialMatrix (chassisCollision, &inertia[0], &origin[0]);


	// set vehicle mass matrix, 
	dFloat mass = 1000.0f;
	dFloat Ixx = mass * inertia[0];
	dFloat Iyy = mass * inertia[1];
	dFloat Izz = mass * inertia[2];
	NewtonBodySetMassMatrix (rigidBody, mass, Ixx, Iyy, Izz);

	// the cog may be off by a car, we can add some offset here
	// Set the vehicle Center of mass
	//origin += config.m_comOffset;
	NewtonBodySetCentreOfMass (rigidBody, &origin[0]);
	return rigidBody;
*/
}


static void CalculateTireDimensions (NewtonBody* const carBody, const char* const name, dFloat& width, dFloat& radius)
{
	// find the the tire visual mesh 
	NewtonWorld* const world = NewtonBodyGetWorld(carBody);
	DemoEntity* const entity = (DemoEntity*) NewtonBodyGetUserData(carBody);

	DemoEntity* const tirePart = entity->dHierarchy<DemoEntity>::Find (name);
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

	dVector widthDir (tireMatrix.UnrotateVector(dVector (0.0f, 0.0f, 1.0f, 0.0f)));
	NewtonCollisionSupportVertex (collision, &widthDir[0], &extremePoint[0]);
	width = widthDir % extremePoint;

	widthDir = widthDir.Scale (-1.0f);
	NewtonCollisionSupportVertex (collision, &widthDir[0], &extremePoint[0]);
	width += widthDir % extremePoint;

	NewtonDestroyCollision(collision);	
}





static CustomWheelVehicle* CreateWheelVehicle (DemoEntityManager* const scene, const char* const name, const dMatrix& location)
{
	// create the car chassis body
	NewtonBody* const carBody = CreateVehicleBody (scene, name, location);

	// create the car joint
	CustomWheelVehicle* const carJoint = new CustomWheelVehicle(carBody);
	DemoEntity* const chassisEntity = (DemoEntity*) NewtonBodyGetUserData(carBody);
	
	const char* tireNames[] = {"ltire_0", "ltire_1", "ltire_2", "ltire_3", "rtire_0", "rtire_1", "rtire_2", "rtire_3"};
	// calculate tire size from a representative tire mesh
	dFloat width;
	dFloat radius;
	CalculateTireDimensions (carBody, tireNames[0], width, radius);

	dFloat tireMass = 60.0f;
	dFloat suspentionLenght = 0.7f;
	dFloat suspensionSpring = 60.0f;
	dFloat suspensionDamper = 5.0f;
	dFloat tireRollResistance = 0.0f;
	// add all tires
	carJoint->BeginAddTire();
	for (int i = 0; i < int (sizeof (tireNames) / sizeof (tireNames[0])); i ++) {
		const char* const tireName = tireNames[i];
		DemoEntity* const tireEntity = chassisEntity->Find(tireName);
		dAssert (tireEntity);

		dMatrix location (tireEntity->CalculateGlobalMatrix(chassisEntity));
//		void* const tireHandle = carJoint->AddSingleSuspensionTire (tireEntity, location.m_posit, tireMass, radius, width, suspentionLenght, suspensionSpring, suspensionDamper, tireRollResistance);
//		tireEntity->SetUserData (new TireUserData (carJoint, tireEntity, tireHandle));
		carJoint->AddSingleSuspensionTire (tireEntity, location.m_posit, tireMass, radius, width, suspentionLenght, suspensionSpring, suspensionDamper, tireRollResistance);
	}
	carJoint->EndAddTire();

	return carJoint;
}



// *************************************************************************************************
// 
//  create a simple racing game with a simple controlled by a Newton AI engine and newton physics
//
// *************************************************************************************************
void MilitaryTransport (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	CreateHeightFieldTerrain (scene, 10, 8.0f, 1.5f, 0.2f, 200.0f, -50.0f);


//	dMatrix camMatrix (dRollMatrix(-20.0f * 3.1416f /180.0f) * dYawMatrix(-45.0f * 3.1416f /180.0f));
	dMatrix camMatrix (GetIdentityMatrix());
	camMatrix.m_posit = dVector (1000.0f, 100.0f, 1000.0f, 1.0f);
	

	CustomWheelVehicle* const carJoint = CreateWheelVehicle (scene, "lav-25.ngd", camMatrix);
	NewtonBody* const carBody = carJoint->GetCarBody();

	NewtonBodyGetMatrix (carBody, &camMatrix[0][0]);
	camMatrix.m_posit.m_x -= 20.0f;
	camMatrix.m_posit.m_z -= 10.0f;
	camMatrix.m_posit.m_y += 2.0f;
	scene->SetCameraMatrix(dQuaternion (camMatrix), camMatrix.m_posit);

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

#endif