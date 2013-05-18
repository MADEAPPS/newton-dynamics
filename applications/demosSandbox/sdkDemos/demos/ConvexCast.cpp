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
#include "DemoMesh.h"
#include "NewtonDemos.h"
#include "PhysicsUtils.h"
#include "DemoCamera.h"
#include "OpenGlUtil.h"
#include "DemoEntityManager.h"


class StupidComplexOfConvexShapes: public DemoEntity
{
	public:
	StupidComplexOfConvexShapes (DemoEntityManager* const scene, int count)
		:DemoEntity (GetIdentityMatrix(), NULL)
	{
count = 1;
		scene->Append(this);

		const dFloat size = 0.5f;

		DemoMesh* gemetries[32];
		NewtonCollision* collisionArray[32];
		NewtonWorld* const world = scene->GetNewton();
		int materialID = NewtonMaterialGetDefaultGroupID(world);

		// create a pool of predefined convex mesh
		PrimitiveType selection[] = {_SPHERE_PRIMITIVE,	_BOX_PRIMITIVE,	_CAPSULE_PRIMITIVE, _CYLINDER_PRIMITIVE, _CONE_PRIMITIVE, _TAPERED_CAPSULE_PRIMITIVE, _TAPERED_CYLINDER_PRIMITIVE, _CHAMFER_CYLINDER_PRIMITIVE, _RANDOM_CONVEX_HULL_PRIMITIVE, _REGULAR_CONVEX_HULL_PRIMITIVE};
		for (int i = 0; i < sizeof (collisionArray) / sizeof (collisionArray[0]); i ++) {
			int index = dRand() % (sizeof (selection) / sizeof (selection[0]));
			dVector shapeSize (size + RandomVariable (size / 2.0f), size + RandomVariable (size / 2.0f), size + RandomVariable (size / 2.0f), 0.0f);
			collisionArray[i] = CreateConvexCollision (world, GetIdentityMatrix(), shapeSize, selection[index], materialID);
			gemetries[i] = new DemoMesh("geometry", collisionArray[i], "wood_4.tga", "wood_4.tga", "wood_1.tga");
		}

		// make a large complex of plane by adding lost of these shapes at a random location and oriention; 
		NewtonCollision* const compound = NewtonCreateCompoundCollision (world, materialID);
		NewtonCompoundCollisionBeginAddRemove(compound);	
		for (int i = 0 ; i < count; i ++) {
			for (int j = 0 ; j < count; j ++) {
				float pitch = RandomVariable (1.0f) * 2.0f * 3.1416f;
				float yaw = RandomVariable (1.0f) * 2.0f * 3.1416f;
				float roll = RandomVariable (1.0f) * 2.0f * 3.1416f;

				float x = size * (j - count / 2) + RandomVariable (size * 0.5f);
				float y = RandomVariable (size * 2.0f);
				float z = size * (i - count / 2) + RandomVariable (size * 0.5f);

				dMatrix matrix (dPitchMatrix (pitch) * dYawMatrix (yaw) * dRollMatrix (roll));
				matrix.m_posit = dVector (x, y, z, 1.0f);

				int index = dRand() % (sizeof (selection) / sizeof (selection[0]));
				DemoEntity* const entity = new DemoEntity(matrix, this);
				entity->SetMesh(gemetries[index]);

				NewtonCollisionSetMatrix (collisionArray[index], &matrix[0][0]);
				NewtonCompoundCollisionAddSubCollision (compound, collisionArray[index]);
			}
		}
		NewtonCompoundCollisionEndAddRemove(compound);	

		CreateSimpleBody (world, NULL, 0.0f, GetIdentityMatrix(), compound, 0);

		// destroy all collision shapes after they are used
		NewtonDestroyCollision(compound);
		for (int i = 0; i < sizeof (collisionArray) / sizeof (collisionArray[0]); i ++) {
			gemetries[i]->Release();
			NewtonDestroyCollision(collisionArray[i]);
		}

		// now make and array of collision shapes for convex casting by mouse point click an drag
		CreateCastingShapes(scene, size * 2.0f);
	}

	~StupidComplexOfConvexShapes()
	{
		m_castingEntity->Release();

		for (int i = 0; i < m_count; i ++) {
			m_castingGeometries[i]->Release();
			NewtonDestroyCollision(m_castingShapeArray[i]);
		}
		delete[] m_castingGeometries;
		delete[] m_castingShapeArray;
	}

	void CreateCastingShapes(DemoEntityManager* const scene, dFloat size)
	{
		NewtonWorld* const world = scene->GetNewton();
		int materialID = NewtonMaterialGetDefaultGroupID(world);

		dVector shapeSize (size, size, size, 0.0f);
		PrimitiveType castSelection[] = {_SPHERE_PRIMITIVE,	_CAPSULE_PRIMITIVE, _BOX_PRIMITIVE, _CYLINDER_PRIMITIVE, _REGULAR_CONVEX_HULL_PRIMITIVE, _CHAMFER_CYLINDER_PRIMITIVE};
		m_count =  sizeof (castSelection) / sizeof (castSelection[0]);
		m_castingGeometries = new DemoMesh*[m_count];
		m_castingShapeArray = new NewtonCollision*[m_count];

		for (int i = 0; i < m_count; i ++) {
			dVector shapeSize (size + RandomVariable (size / 2.0f), size + RandomVariable (size / 2.0f), size + RandomVariable (size / 2.0f), 0.0f);
			m_castingShapeArray[i] = CreateConvexCollision (world, GetIdentityMatrix(), shapeSize, castSelection[i], materialID);
			m_castingGeometries[i] = new DemoMesh("geometry", m_castingShapeArray[i], "smilli.tga", "smilli.tga", "smilli.tga");
		}

		// make and entity for showing the result of the convex cast
		dMatrix matrix (GetIdentityMatrix());
		matrix.m_posit.m_y = 2.0f;

		m_currentCastingShape = 0;
		m_castingEntity = new DemoEntity (matrix, NULL);
		m_castingEntity->SetMesh(m_castingGeometries[0]);
	}

	NewtonCollision* GetCurrentShape() const 
	{
		return m_castingShapeArray[m_currentCastingShape];
	}

	virtual void Render(dFloat timeStep) const
	{
		DemoEntity::Render(timeStep);
		m_castingEntity->Render(timeStep);
	}

	int m_count;
	int m_currentCastingShape;
	DemoEntity* m_castingEntity;
	DemoMesh** m_castingGeometries;
	NewtonCollision** m_castingShapeArray;
};


class dConvexCastRecord
{
	CUSTOM_CONTROLLER_GLUE(dConvexCastRecord);
/*
	class ConvexCastEntity: public DemoEntity
	{
		public:
		ConvexCastEntity (DemoEntityManager* const scene, dMatrix& matrix, int materialID, PrimitiveType castingShapeType)
			:DemoEntity (matrix, NULL)
			,m_contactsCount(0)
		{
			NewtonWorld* const world = scene->GetNewton();

			dVector size(1.0f, 1.0f, 1.0f, 0.0f);
			m_castingShape = CreateConvexCollision (world, GetIdentityMatrix(), size, castingShapeType, 0);

			DemoMesh* const geometry = new DemoMesh("convexShape", m_castingShape, "smilli.tga", "smilli.tga", "smilli.tga");
			SetMesh(geometry);
			geometry->Release();

			scene->Append(this);
		}

		~ConvexCastEntity()
		{
			NewtonDestroyCollision (m_castingShape);
		}

		virtual void Render(dFloat timestep) const
		{
			DemoEntity::Render(timestep);
			for (int i = 0; i <  m_contactsCount; i ++) {
				dVector n (m_normals[i]);
				dVector p0 (m_contacts[i]);
				dVector p1 (p0 + n.Scale (-1.0f));
				ShowMousePicking (p0, p1, dVector (1, 0, 0, 0), dVector (0, 1, 0, 0));
			}
		}

		dVector m_normals[4];
		dVector m_contacts[4];
		NewtonCollision* m_castingShape;
		int m_contactsCount;
	};
*/

	public:
	void PreUpdate(dFloat timestep, int threadIndex)
	{
	}

	void PostUpdate(dFloat timestep, int threadIndex)
	{
/*
		// get the matrices of the two entities
		dMatrix matrixA;
		NewtonBodyGetMatrix(m_bodyToCast, &matrixA[0][0]);

		dFloat speed = m_step * timestep * 60.0f; 
		m_pith = dMod (m_pith + speed, 3.1416f * 2.0f);
		m_yaw = dMod (m_yaw + speed, 3.1416f * 2.0f);
		m_roll = dMod (m_roll + speed, 3.1416f * 2.0f);

		m_pith = 0.0;
		m_yaw = 0.0;
		m_roll = 0.0;

		dMatrix matrixB(dPitchMatrix(m_pith) * dYawMatrix(m_yaw) * dRollMatrix(m_roll));
		//		dMatrix matrixB (matrixA);
		matrixB.m_posit = matrixA.m_posit;
		matrixB.m_posit.m_y += 10.0f;

		dVector targetPosit (matrixA.m_posit);
		targetPosit.m_y -= 10.0f;

		NewtonWorld* const world = NewtonBodyGetWorld(m_bodyToCast);
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData (world);

		// now do the convex cast form shape casting shape to this shape 
		m_castingVisualEntity->m_contactsCount = 0;
		dFloat hitParam;
		NewtonWorldConvexCastReturnInfo info[16];

		// do a convex cast and save the hit point to rendering later 
		int count = NewtonWorldConvexCast (world, &matrixB[0][0], &targetPosit[0], m_castingVisualEntity->m_castingShape, &hitParam, NULL, NULL, &info[0], 4, 0);		
		if (count) {
			matrixB.m_posit += (targetPosit - matrixB.m_posit).Scale (hitParam);

			m_castingVisualEntity->m_contactsCount = dMin(count, int (sizeof (m_castingVisualEntity->m_contacts)/sizeof (m_castingVisualEntity->m_contacts[0])));
			for (int i = 0; i < m_castingVisualEntity->m_contactsCount; i ++) {
				m_castingVisualEntity->m_normals[i] = dVector (info[i].m_normal[0], info[i].m_normal[1], info[i].m_normal[2], 0.0f);
				m_castingVisualEntity->m_contacts[i] = dVector (info[i].m_point[0], info[i].m_point[1], info[i].m_point[2], 0.0f);
			}
		}
		m_castingVisualEntity->ResetMatrix(*scene, matrixB);
*/
	}

	void Init (dFloat location_x, dFloat location_z, PrimitiveType shapeType, int materialID, PrimitiveType castingShapeType)
	{
/*
		m_pith = RandomVariable(3.1416f * 2.0f);
		m_yaw = RandomVariable(3.1416f * 2.0f);
		m_roll = RandomVariable(3.1416f * 2.0f);
		m_step = 15.0f * (dAbs (RandomVariable(0.25f)) + 0.0001f) * 3.1416f/180.0f;

		CreatCasterBody(location_x, location_z, shapeType, materialID);

		NewtonWorld* const world = GetManager()->GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		dMatrix matrix;
		NewtonBodyGetMatrix(m_bodyToCast, &matrix[0][0]);
		matrix.m_posit.m_y += 10.0f;
		m_castingVisualEntity = new ConvexCastEntity (scene, matrix, materialID, castingShapeType);
*/
	}


	private:
	void CreatCasterBody(dFloat location_x, dFloat location_z, PrimitiveType shapeType, int materialID)
	{
/*
		NewtonWorld* const world = GetManager()->GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		//dMatrix matrix (GetIdentityMatrix());
		dMatrix matrix (dRollMatrix(3.141592f/2.0f));

		matrix.m_posit.m_x = location_x;
		matrix.m_posit.m_y = 2.0f;
		matrix.m_posit.m_z = location_z;

		// create the shape and visual mesh as a common data to be re used
		dVector size(0.5f, 0.5f, 0.75f, 0.0f);
		NewtonCollision* const collision = CreateConvexCollision (world, GetIdentityMatrix(), size, shapeType, materialID);

		//	DemoMesh* const geometry = new DemoMesh("cylinder_1", collision, "wood_0.tga", "wood_0.tga", "wood_1.tga");
		DemoMesh* const geometry = new DemoMesh("convexShape", collision, "smilli.tga", "smilli.tga", "smilli.tga");
		m_bodyToCast = CreateSimpleSolid (scene, geometry, 1.0f, matrix, collision, materialID);

		// disable gravity and apply a force that only spin the body 
		//NewtonBodySetForceAndTorqueCallback(m_myBody, PhysicsSpinBody);
		//NewtonBodySetAutoSleep (m_myBody, 0);

		geometry->Release(); 
		NewtonDestroyCollision (collision);
*/
	}
/*
	NewtonBody* m_bodyToCast;
	ConvexCastEntity* m_castingVisualEntity;

	dFloat m_pith;
	dFloat m_yaw;
	dFloat m_roll;
	dFloat m_step;
*/
	
};


class dConvexCastManager: public CustomControllerManager<dConvexCastRecord> 
{
	public:
	dConvexCastManager(DemoEntityManager* const scene, StupidComplexOfConvexShapes* const stupidLevel)
		:CustomControllerManager<dConvexCastRecord>(scene->GetNewton(), "dConvexCastManager")
		,m_helpKey (true)
		,m_stupidLevel(stupidLevel)
	{
		scene->Set2DDisplayRenderFunction (RenderHelp, this);
	}

	static void RenderHelp (DemoEntityManager* const scene, void* const context)
	{
		dConvexCastManager* const me = (dConvexCastManager*) context;
		me->RenderHelp (scene);
	}

	void RenderHelp (DemoEntityManager* const scene)
	{
		if (m_helpKey.GetPushButtonState()) {
			glDisable(GL_TEXTURE_2D);

			dVector color(1.0f, 1.0f, 0.0f, 0.0f);

			scene->Print (color, 10, 100, "mouse point and right click to cast current convex shape");
			scene->Print (color, 10, 120, "space to change casting shape");
			scene->Print (color, 10, 140, "hide help   : H");
		}
	}

	virtual void Debug () const 
	{
	};

	virtual void PreUpdate (dFloat timestep)
	{
		//CustomControllerManager<dConvexCastRecord>::PreUpdate (timestep);
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);
		NewtonDemos* const mainWindow = scene->GetRootWindow();
		DemoCamera* const camera = scene->GetCamera();

		m_helpKey.UpdatePushButton (mainWindow, 'H');

		bool buttonState = mainWindow->GetMouseKeyState(1);
		if (buttonState) {
			int mouseX;
			int mouseY;

			buttonState = false;
			mainWindow->GetMousePosition (mouseX, mouseY);

			float x = dFloat (mouseX);
			float y = dFloat (mouseX);
			dVector p0 (camera->ScreenToWorld(dVector (x, y, 0.0f, 0.0f)));
			dVector p1 (camera->ScreenToWorld(dVector (x, y, 1.0f, 0.0f)));

			dMatrix matrix (GetIdentityMatrix());
			matrix.m_posit = p0;

			dFloat hitParam;
			NewtonWorldConvexCastReturnInfo info[16];
			NewtonCollision* const shape = m_stupidLevel->GetCurrentShape();
			int count = NewtonWorldConvexCast (world, &matrix[0][0], &p1[0], shape, &hitParam, NULL, NULL, &info[0], 4, 0);		
			if (count) {
				_ASSERTE (0);
				//matrixB.m_posit += (targetPosit - matrixB.m_posit).Scale (hitParam);
				//m_castingVisualEntity->m_contactsCount = dMin(count, int (sizeof (m_castingVisualEntity->m_contacts)/sizeof (m_castingVisualEntity->m_contacts[0])));
				//for (int i = 0; i < m_castingVisualEntity->m_contactsCount; i ++) {
				//	m_castingVisualEntity->m_normals[i] = dVector (info[i].m_normal[0], info[i].m_normal[1], info[i].m_normal[2], 0.0f);
				//	m_castingVisualEntity->m_contacts[i] = dVector (info[i].m_point[0], info[i].m_point[1], info[i].m_point[2], 0.0f);
				//}
			}
			//m_castingVisualEntity->ResetMatrix(*scene, matrixB);


		}

		// do the convex cast here 
	}

	DemoEntityManager::ButtonKey m_helpKey;
	StupidComplexOfConvexShapes* m_stupidLevel;
};





// create physics scene
void ConvexCast (DemoEntityManager* const scene)
{
	scene->CreateSkyBox();

	// customize the scene after loading
	// set a user friction variable in the body for variable friction demos
	// later this will be done using LUA script
	//NewtonWorld* const world = scene->GetNewton();
	dMatrix offsetMatrix (GetIdentityMatrix());

//	CreateLevelMesh (scene, "flatPlane.ngd", 0);
//	CreateLevelMesh (scene, "playground.ngd", 0);


	StupidComplexOfConvexShapes* const stupidLevel = new StupidComplexOfConvexShapes (scene, 10);

	new dConvexCastManager (scene, stupidLevel);


	// place camera into position
	dMatrix camMatrix (GetIdentityMatrix());
	dQuaternion rot (camMatrix);
	dVector origin (-30.0f, 10.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}






