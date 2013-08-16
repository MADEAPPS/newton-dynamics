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
		,m_rayP0(0.0f, 0.0f, 0.0f, 0.0f)
		,m_rayP1(0.0f, 0.0f, 0.0f, 0.0f)
	{
		scene->Append(this);

		const dFloat size = 0.5f;

		DemoMesh* gemetries[32];
		NewtonCollision* collisionArray[32];
		NewtonWorld* const world = scene->GetNewton();
		int materialID = NewtonMaterialGetDefaultGroupID(world);

		// create a pool of predefined convex mesh
		PrimitiveType selection[] = {_SPHERE_PRIMITIVE,	_BOX_PRIMITIVE,	_CAPSULE_PRIMITIVE, _CYLINDER_PRIMITIVE, _CONE_PRIMITIVE, _TAPERED_CAPSULE_PRIMITIVE, _TAPERED_CYLINDER_PRIMITIVE, _CHAMFER_CYLINDER_PRIMITIVE, _RANDOM_CONVEX_HULL_PRIMITIVE, _REGULAR_CONVEX_HULL_PRIMITIVE};
		for (int i = 0; i < int (sizeof (collisionArray) / sizeof (collisionArray[0])); i ++) {
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
		for (int i = 0; i < int (sizeof (collisionArray) / sizeof (collisionArray[0])); i ++) {
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


	void SetCastingLine (const dVector& p0, const dVector& p1)
	{
		m_rayP0 = p0;
		m_rayP1 = p1;
	}

	void ChangeCastingShape()
	{
		m_currentCastingShape = (m_currentCastingShape + 1) % m_count;
		m_castingEntity->SetMesh(m_castingGeometries[m_currentCastingShape]);
	}

	void SetCastEntityMatrix (DemoEntityManager* const scene, const dMatrix& matrix) const
	{
		m_castingEntity->ResetMatrix(*scene, matrix);
	}

	virtual void Render(dFloat timeStep) const
	{
		DemoEntity::Render(timeStep);
		m_castingEntity->Render(timeStep);

		// draw the last casting line
		glDisable(GL_TEXTURE_2D);
		glColor3f(1.0f, 1.0f, 1.0f);
		glBegin(GL_LINES);
			glVertex3f (m_rayP0.m_x, m_rayP0.m_y, m_rayP0.m_z);
			glVertex3f (m_rayP1.m_x, m_rayP1.m_y, m_rayP1.m_z);
		glEnd();
	}

	int m_count;
	int m_currentCastingShape;

	dVector m_rayP0;
	dVector m_rayP1;
	DemoEntity* m_castingEntity;
	DemoMesh** m_castingGeometries;
	NewtonCollision** m_castingShapeArray;
};


class dConvexCastRecord: public CustomControllerBase
{
	public:
	void PreUpdate(dFloat timestep, int threadIndex)
	{
	}

	void PostUpdate(dFloat timestep, int threadIndex)
	{
	}

//	void Init (dFloat location_x, dFloat location_z, PrimitiveType shapeType, int materialID, PrimitiveType castingShapeType)
//	{
//	}
};


class dConvexCastManager: public CustomControllerManager<dConvexCastRecord> 
{
	public:
	dConvexCastManager(DemoEntityManager* const scene, StupidComplexOfConvexShapes* const stupidLevel)
		:CustomControllerManager<dConvexCastRecord>(scene->GetNewton(), "dConvexCastManager")
		,m_helpKey (true)
		,m_selectShape (true)
		,m_stupidLevel(stupidLevel)
	{
		scene->Set2DDisplayRenderFunction (RenderHelp, this);
	}

	static void RenderHelp (DemoEntityManager* const scene, void* const context, int lineNumber)
	{
		dConvexCastManager* const me = (dConvexCastManager*) context;
		me->RenderHelp (scene, lineNumber);
	}

	void RenderHelp (DemoEntityManager* const scene, int lineNumber)
	{
		if (m_helpKey.GetPushButtonState()) {
			dVector color(1.0f, 1.0f, 0.0f, 0.0f);
			lineNumber = scene->Print (color, 10, lineNumber + 20, "mouse point and right click to cast current convex shape");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "space	   : change casting shape");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "h		   : hide help");
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
		if (m_selectShape.UpdateTriggerButton (mainWindow, ' ')) {
			m_stupidLevel->ChangeCastingShape();
		}

		bool buttonState = mainWindow->GetMouseKeyState(1);
		if (buttonState) {
			int mouseX;
			int mouseY;

			buttonState = false;
			mainWindow->GetMousePosition (mouseX, mouseY);

			float x = dFloat (mouseX);
			float y = dFloat (mouseY);
			dVector p0 (camera->ScreenToWorld(dVector (x, y, 0.0f, 0.0f)));
			dVector p1 (camera->ScreenToWorld(dVector (x, y, 1.0f, 0.0f)));

			// do the convex cast here 
			dMatrix matrix (GetIdentityMatrix());
			matrix.m_posit = p0;

			dFloat hitParam;
			NewtonWorldConvexCastReturnInfo info[16];
			NewtonCollision* const shape = m_stupidLevel->GetCurrentShape();
			int count = NewtonWorldConvexCast (world, &matrix[0][0], &p1[0], shape, &hitParam, NULL, NULL, &info[0], 4, 0);		
			if (count) {
				matrix.m_posit += (p1 - matrix.m_posit).Scale (hitParam);
				m_stupidLevel->SetCastEntityMatrix (scene, matrix);
			}

			m_stupidLevel->SetCastingLine (p0, p1);
		}
	}

	DemoEntityManager::ButtonKey m_helpKey;
	DemoEntityManager::ButtonKey m_selectShape;
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


	StupidComplexOfConvexShapes* const stupidLevel = new StupidComplexOfConvexShapes (scene, 100);

	new dConvexCastManager (scene, stupidLevel);


	// place camera into position
	dMatrix camMatrix (GetIdentityMatrix());
	dQuaternion rot (camMatrix);
	dVector origin (-30.0f, 10.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}






