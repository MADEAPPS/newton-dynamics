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
#include "UserPlaneCollision.h"

class StupidComplexOfConvexShapes: public DemoEntity
{
	public:
	StupidComplexOfConvexShapes (DemoEntityManager* const scene, int count)
	:DemoEntity (dGetIdentityMatrix(), NULL)
	,m_rayP0(0.0f, 0.0f, 0.0f, 0.0f)
	,m_rayP1(0.0f, 0.0f, 0.0f, 0.0f)
	{
		scene->Append(this);

		count = 40;
		const dFloat size = 0.5f;

		DemoMesh* gemetries[32];
		NewtonCollision* collisionArray[32];
		NewtonWorld* const world = scene->GetNewton();
		int materialID = NewtonMaterialGetDefaultGroupID(world);

		// create a pool of predefined convex mesh
		//		PrimitiveType selection[] = {_SPHERE_PRIMITIVE,	_BOX_PRIMITIVE,	_CAPSULE_PRIMITIVE, _CYLINDER_PRIMITIVE, _CONE_PRIMITIVE, _TAPERED_CAPSULE_PRIMITIVE, _TAPERED_CYLINDER_PRIMITIVE, _CHAMFER_CYLINDER_PRIMITIVE, _RANDOM_CONVEX_HULL_PRIMITIVE, _REGULAR_CONVEX_HULL_PRIMITIVE};
		PrimitiveType selection[] = {_SPHERE_PRIMITIVE};
		for (int i = 0; i < int (sizeof (collisionArray) / sizeof (collisionArray[0])); i ++) {
			int index = dRand() % (sizeof (selection) / sizeof (selection[0]));
			dVector shapeSize (size + RandomVariable (size / 2.0f), size + RandomVariable (size / 2.0f), size + RandomVariable (size / 2.0f), 0.0f);
			shapeSize = dVector(size, size, size, 0.0f);
			collisionArray[i] = CreateConvexCollision (world, dGetIdentityMatrix(), shapeSize, selection[index], materialID);
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

				entity->SetMesh(gemetries[index], dGetIdentityMatrix());

				NewtonCollisionSetMatrix (collisionArray[index], &matrix[0][0]);
				NewtonCompoundCollisionAddSubCollision (compound, collisionArray[index]);
			}
		}
		NewtonCompoundCollisionEndAddRemove(compound);

		CreateSimpleBody (world, NULL, 0.0f, dGetIdentityMatrix(), compound, 0);

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

		dSetRandSeed (0);

		dVector shapeSize (size, size, size, 0.0f);
		PrimitiveType castSelection[] = {_SPHERE_PRIMITIVE,	_CAPSULE_PRIMITIVE, _BOX_PRIMITIVE, _CYLINDER_PRIMITIVE, _REGULAR_CONVEX_HULL_PRIMITIVE, _CHAMFER_CYLINDER_PRIMITIVE};
		m_count =  sizeof (castSelection) / sizeof (castSelection[0]);
		m_castingGeometries = new DemoMesh*[m_count];
		m_castingShapeArray = new NewtonCollision*[m_count];

		dMatrix alignMatrix (dRollMatrix(3.141592f * 90.0f / 180.0f));
		for (int i = 0; i < m_count; i ++) {
			dVector shapeSize (size + RandomVariable (size / 2.0f), size + RandomVariable (size / 2.0f), size + RandomVariable (size / 2.0f), 0.0f);
#if 1
			m_castingShapeArray[i] = CreateConvexCollision (world, &alignMatrix[0][0], shapeSize, castSelection[i], materialID);
#else
			m_castingShapeArray[i] = NewtonCreateCompoundCollision (world, materialID);
			NewtonCompoundCollisionBeginAddRemove(m_castingShapeArray[i]);
			NewtonCollision* const collision = CreateConvexCollision (world, &alignMatrix[0][0], shapeSize, castSelection[i], materialID);
			NewtonCompoundCollisionAddSubCollision (m_castingShapeArray[i], collision);
			NewtonDestroyCollision(collision);
			NewtonCompoundCollisionEndAddRemove(m_castingShapeArray[i]);
#endif
			m_castingGeometries[i] = new DemoMesh("geometry", m_castingShapeArray[i], "smilli.tga", "smilli.tga", "smilli.tga");
		}

		// make and entity for showing the result of the convex cast
		dMatrix matrix (dGetIdentityMatrix());
		matrix.m_posit.m_y = 2.0f;

		m_currentCastingShape = 0;
		m_castingEntity = new DemoEntity (matrix, NULL);
		m_castingEntity->SetMesh(m_castingGeometries[m_currentCastingShape], dGetIdentityMatrix());
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
		m_castingEntity->SetMesh(m_castingGeometries[m_currentCastingShape], dGetIdentityMatrix());
	}

	void SetCastEntityMatrix (DemoEntityManager* const scene, const dMatrix& matrix) const
	{
		m_castingEntity->ResetMatrix(*scene, matrix);
	}


	virtual void Render(dFloat timeStep, DemoEntityManager* const scene) const
	{
		DemoEntity::Render(timeStep, scene);
		m_castingEntity->Render(timeStep, scene);

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
		(void)timestep;
		(void)threadIndex;
	}

	void PostUpdate(dFloat timestep, int threadIndex)
	{
		(void)timestep;
		(void)threadIndex;
	}
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
		(void)timestep;

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

			//p0 = dVector (-29.990000, 9.996307, 0.000510, 1.0f);
			//p1 = dVector (1967.297607, -727.473450, 101.905418, 1.0f);
				
			//dTrace (("%f, %f, %f\n", p0[0], p0[1], p0[2]));
			//dTrace (("%f, %f, %f\n", p1[0], p1[1], p1[2]));

			// do the convex cast here
			dMatrix matrix (dGetIdentityMatrix());
			matrix.m_posit = p0;

			dFloat hitParam;
			NewtonWorldConvexCastReturnInfo info[16];
			NewtonCollision* const shape = m_stupidLevel->GetCurrentShape();
			int count = NewtonWorldConvexCast (world, &matrix[0][0], &p1[0], shape, &hitParam, NULL, NULL, &info[0], 4, 0);
			//if (!count) {
				//dTrace(("%f, %f, %f\n", p0[0], p0[1], p0[2]));
				//dTrace(("%f, %f, %f\n", p1[0], p1[1], p1[2]));
			//}

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


static void AddSingleCompound(DemoEntityManager* const scene)
{
	NewtonWorld* const world = scene->GetNewton();

	NewtonCollision* compoundCollision = NewtonCreateCompoundCollision(world, 0);
	NewtonCompoundCollisionBeginAddRemove(compoundCollision);

	NewtonCollision* boxCollision = NewtonCreateBox(world, 50, 50, 50, 0, NULL);
	NewtonCompoundCollisionAddSubCollision(compoundCollision, boxCollision);
	NewtonDestroyCollision(boxCollision);

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit.m_y = 10.0f;

	NewtonCompoundCollisionEndAddRemove(compoundCollision);
	NewtonBody* compoundBody = NewtonCreateDynamicBody(world, compoundCollision, &matrix[0][0]);
	NewtonDestroyCollision(compoundCollision);

	// scale after creating body slows everything down. Without the scale it runs fine even though the body is huge
	NewtonCollisionSetScale(NewtonBodyGetCollision(compoundBody), 0.05f, 0.05f, 0.05f);


	// adding some visualization
	NewtonBodySetMassProperties (compoundBody, 1.0f, NewtonBodyGetCollision(compoundBody));
	NewtonBodySetTransformCallback(compoundBody, DemoEntity::TransformCallback);
	NewtonBodySetForceAndTorqueCallback(compoundBody, PhysicsApplyGravityForce);

	DemoMesh* mesh = new DemoMesh("geometry", NewtonBodyGetCollision(compoundBody), "smilli.tga", "smilli.tga", "smilli.tga");
	DemoEntity* const entity = new DemoEntity(matrix, NULL);
	entity->SetMesh(mesh, dGetIdentityMatrix());
	mesh->Release();
	NewtonBodySetUserData(compoundBody, entity);
	scene->Append(entity);

	NewtonBodySetSimulationState(compoundBody, 0);
	NewtonBodySetSimulationState(compoundBody, 1);
}


static void AddStaticMesh(DemoEntityManager* const scene)
{
	char fileName[2048];

	NewtonWorld* const world = scene->GetNewton();
	GetWorkingFileName ("ramp.off", fileName);
	NewtonMesh* const ntMesh = NewtonMeshLoadOFF(world, fileName);

	dMatrix matrix (dGetIdentityMatrix());
	DemoMesh* mesh = new DemoMesh(ntMesh);
	DemoEntity* const entity = new DemoEntity(matrix, NULL);
	entity->SetMesh(mesh, dGetIdentityMatrix());
	mesh->Release();

	scene->Append(entity);

	CreateLevelMeshBody (world, entity, true);

	NewtonMeshDestroy (ntMesh);
}

static void AddUserDefineStaticMesh(DemoEntityManager* const scene)
{
	dVector planeEquation (0.0f, 1.0f, 0.0f, 0.0f);
	NewtonCollision* const planeCollision = CreateInfinitePlane (scene->GetNewton(), planeEquation);

	// create the the rigid body for
	dMatrix matrix (dGetIdentityMatrix());
	NewtonBody* const body = NewtonCreateDynamicBody(scene->GetNewton(), planeCollision, &matrix[0][0]);

	// create a visual mesh
	DemoMesh* const mesh = CreateVisualPlaneMesh (planeEquation);
	DemoEntity* const entity = new DemoEntity(matrix, NULL);

	NewtonCollisionGetMatrix(planeCollision, &matrix[0][0]);

	scene->Append (entity);
	entity->SetMesh(mesh, matrix);
	mesh->Release();

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (body, entity);

	// destroy the collision shape
	NewtonDestroyCollision (planeCollision);
}


// create physics scene
void ConvexCast (DemoEntityManager* const scene)
{
	scene->CreateSkyBox();

	//char fileName[2048];
	//NewtonWorld* const world = scene->GetNewton();
	//GetWorkingFileName ("low_rez_rim.OFF", fileName);
	//NewtonMesh* const mesh = NewtonMeshLoadOFF(world, fileName);
	//NewtonCollision* coll = NewtonCreateConvexHullFromMesh (world, mesh, 0.0f, 0);
	//NewtonDestroyCollision(coll);


	// customize the scene after loading
	// set a user friction variable in the body for variable friction demos
	// later this will be done using LUA script
	//NewtonWorld* const world = scene->GetNewton();
//	dMatrix offsetMatrix (dGetIdentityMatrix());

	//	CreateLevelMesh (scene, "flatPlane.ngd", 0);
	//	CreateLevelMesh (scene, "playground.ngd", 0);

	StupidComplexOfConvexShapes* const stupidLevel = new StupidComplexOfConvexShapes (scene, 100);
	new dConvexCastManager (scene, stupidLevel);

	// add a single compound Box test
	AddSingleCompound(scene);
	AddStaticMesh(scene);
	AddUserDefineStaticMesh(scene);

	// place camera into position
	dMatrix camMatrix (dGetIdentityMatrix());
	dQuaternion rot (camMatrix);
	dVector origin (-30.0f, 10.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}






