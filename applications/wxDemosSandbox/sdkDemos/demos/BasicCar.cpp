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
#include "NewtonDemos.h"
#include "TargaToOpenGl.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "HeightFieldPrimitive.h"
#include "DebugDisplay.h"


class dBasicVehicleLoader: public dCustomJointSaveLoad
{
	public:
	dBasicVehicleLoader(NewtonWorld* const world, FILE* const file, int materialID)
		:dCustomJointSaveLoad(world, file)
		, m_material(materialID)
	{
	}

	const char* GetUserDataName(const NewtonBody* const body) const
	{
		dAssert(0);
		return NULL;
	}

	virtual const void InitRigiBody(const NewtonBody* const body, const char* const bodyName) const
	{
		dMatrix matrix;
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(NewtonBodyGetWorld(body));

		NewtonCollision* const collision = NewtonBodyGetCollision(body);
		DemoMesh* mesh;
		if (!strcmp(bodyName, "tireMesh")) {
			mesh = new DemoMesh(bodyName, collision, "wood_4.tga", "wood_4.tga", "wood_4.tga");
		}
		else {
			mesh = new DemoMesh(bodyName, collision, "wood_1.tga", "wood_1.tga", "wood_1.tga");
		}

		NewtonBodyGetMatrix(body, &matrix[0][0]);
		DemoEntity* const entity = new DemoEntity(matrix, NULL);
		entity->SetMesh(mesh, dGetIdentityMatrix());
		scene->Append(entity);
		mesh->Release();

		// save the pointer to the graphic object with the body.
		NewtonBodySetUserData(body, entity);

		// assign the wood id
		NewtonBodySetMaterialGroupID(body, m_material);

		//set continuous collision mode
		//NewtonBodySetContinuousCollisionMode (body, continueCollisionMode);

		// set a destructor for this rigid body
		NewtonBodySetDestructorCallback(body, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback(body, DemoEntity::TransformCallback);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);
	}

	int m_material;
	dCustomVehicleController* m_vehicle;
};



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
		scene->Set2DDisplayRenderFunction (RenderVehicleHud, this);
	}

	~BasicCarControllerManager ()
	{
	}

	static void RenderVehicleHud (DemoEntityManager* const scene, void* const context, int lineNumber)
	{
		BasicCarControllerManager* const me = (BasicCarControllerManager*) context;
		me->RenderVehicleHud (scene, lineNumber);
		
	}

	void DrawHelp(DemoEntityManager* const scene, int lineNumber) const
	{
		dAssert(0);
/*
		if (m_player->m_helpKey.GetPushButtonState()) {
			dVector color(1.0f, 1.0f, 0.0f, 0.0f);
			lineNumber = scene->Print (color, 10, lineNumber + 20, "Vehicle driving keyboard control");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "key switch          : 'I'");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "accelerator         : 'W'");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "reverse             : 'S'");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "turn left           : 'A'");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "turn right          : 'D'");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "engage clutch       : 'K'");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "hand brakes         : 'space'");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "hide help           : 'H'");
		}
*/
	}

	void RenderVehicleHud (DemoEntityManager* const scene, int lineNumber) const
	{
//		dAssert(0);
/*
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
			origin.m_posit = dVector(width - 300, height - 200, 0.0f, 1.0f);

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
*/
	}

/*
	void SetAsPlayer (BasicCarEntity* const player)
	{
		dAssert(0);
		m_player = player;
	}
*/

	virtual void PreUpdate (dFloat timestep)
	{
		// apply the vehicle controls, and all simulation time effect
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		// set the help key
		//m_helpKey.UpdatePushButton(scene->GetRootWindow(), 'H');

		// do the base class post update
		dCustomVehicleControllerManager::PreUpdate(timestep);
	}

	virtual void PostUpdate (dFloat timestep)
	{
//		dAssert(0);
/*
		// do the base class post update
		dCustomVehicleControllerManager::PostUpdate(timestep);

		// update the visual transformation matrices for all vehicle tires
		for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
			dCustomVehicleController* const controller = &node->GetInfo();
			BasicCarEntity* const vehicleEntity = (BasicCarEntity*)NewtonBodyGetUserData (controller->GetBody());
			vehicleEntity->UpdateTireTransforms();
		}

		UpdateCamera (timestep);
*/
	}


	void UpdateCamera (dFloat timestep)
	{
		dAssert(0);
/*
		if (m_player) {
			DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(GetWorld());
			DemoCamera* const camera = scene->GetCamera();
			dMatrix camMatrix (camera->GetNextMatrix ());
			dMatrix playerMatrix (m_player->GetNextMatrix());

			dVector frontDir (camMatrix[0]);
			dVector camOrigin(0.0f); 
			if (m_externalView) {
				camOrigin = playerMatrix.m_posit + dVector(0.0f, VEHICLE_THIRD_PERSON_VIEW_HIGHT, 0.0f, 0.0f);
				camOrigin -= frontDir.Scale (VEHICLE_THIRD_PERSON_VIEW_DIST);
			} else {
				dAssert (0);
				//            camMatrix = camMatrix * playerMatrix;
				//            camOrigin = playerMatrix.TransformVector(dVector(-0.8f, ARTICULATED_VEHICLE_CAMERA_EYEPOINT, 0.0f, 0.0f));
			}

			camera->SetNextMatrix (*scene, camMatrix, camOrigin);
		}
*/
	}

/*
	// use this to display debug information about vehicle 
	void Debug () const
	{
		for (dListNode* ptr = GetFirst(); ptr; ptr = ptr->GetNext()) {
			dCustomVehicleController* const controller = &ptr->GetInfo();
			BasicCarEntity* const vehicleEntity = (BasicCarEntity*)NewtonBodyGetUserData (controller->GetBody());
			vehicleEntity->Debug();
		}
	}

*/


	dCustomVehicleController* LoadVehicle(const char* const name)
	{
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
	}

	bool m_externalView;
//	BasicCarEntity* m_player;
};



void BasicCar (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	CreateLevelMesh (scene, "flatPlane.ngd", 1);
//	CreateLevelMesh (scene, "flatPlane1.ngd", 0);
//	CreateHeightFieldTerrain (scene, 10, 8.0f, 5.0f, 0.2f, 200.0f, -50.0f);
//	AddPrimitiveArray (scene, 0.0f, dVector (0.0f, 0.0f, 0.0f, 0.0f), dVector (100.0f, 1.0f, 100.0f, 0.0f), 1, 1, 0, _BOX_PRIMITIVE, 0, dGetIdentityMatrix());

	dMatrix location (dGetIdentityMatrix());
	location.m_posit = dVector (0.0f, 10.0f, 0.0f, 1.0f);

	location.m_posit = FindFloor (scene->GetNewton(), location.m_posit, 100.0f);
	location.m_posit.m_y += 2.0f;

	NewtonWorld* const world = scene->GetNewton();

	// create a vehicle controller manager
	int defaulMaterial = NewtonMaterialGetDefaultGroupID(scene->GetNewton());
	int materialList[] = {defaulMaterial };
	BasicCarControllerManager* const manager = new BasicCarControllerManager (world, 1, materialList);
	
	// load 
	manager->LoadVehicle("simpleVehicle.txt");

	// set this vehicle as the player
//	manager->SetAsPlayer(basicVehicle);

//	dMatrix camMatrix (manager->m_player->GetNextMatrix());
//	scene->SetCameraMouseLock (true);

	dMatrix camMatrix(dGetIdentityMatrix());
	camMatrix.m_posit = dVector (-20.0f, 1.5f, 102.f, 1.0f);
	scene->SetCameraMatrix(camMatrix, camMatrix.m_posit);

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
		
}

