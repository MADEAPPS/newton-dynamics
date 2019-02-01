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
#include "DemoMesh.h"
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
		,m_rayP0(0.0f)
		,m_rayP1(0.0f)
	{
		scene->Append(this);

		count = 40;
		//count = 1;
		const dFloat size = 0.5f;

		DemoMesh* gemetries[32];
		NewtonCollision* collisionArray[32];
		NewtonWorld* const world = scene->GetNewton();
		int materialID = NewtonMaterialGetDefaultGroupID(world);

		// create a pool of predefined convex mesh
		// PrimitiveType selection[] = {_SPHERE_PRIMITIVE,	_BOX_PRIMITIVE,	_CAPSULE_PRIMITIVE, _CYLINDER_PRIMITIVE, _CONE_PRIMITIVE, _CHAMFER_CYLINDER_PRIMITIVE, _RANDOM_CONVEX_HULL_PRIMITIVE, _REGULAR_CONVEX_HULL_PRIMITIVE};
		PrimitiveType selection[] = {_SPHERE_PRIMITIVE};
		for (int i = 0; i < int (sizeof (collisionArray) / sizeof (collisionArray[0])); i ++) {
			int index = dRand() % (sizeof (selection) / sizeof (selection[0]));
			dVector shapeSize (size + dGaussianRandom  (size / 2.0f), size + dGaussianRandom  (size / 2.0f), size + dGaussianRandom  (size / 2.0f), 0.0f);
			shapeSize = dVector(size, size, size, 0.0f);
			collisionArray[i] = CreateConvexCollision (world, dGetIdentityMatrix(), shapeSize, selection[index], materialID);
			gemetries[i] = new DemoMesh("geometry", scene->GetShaderCache(), collisionArray[i], "wood_4.tga", "wood_4.tga", "wood_1.tga");
		}

		// make a large complex of plane by adding lost of these shapes at a random location and oriention;
		NewtonCollision* const compound = NewtonCreateCompoundCollision (world, materialID);
		NewtonCompoundCollisionBeginAddRemove(compound);
		
		for (int i = 0 ; i < count; i ++) {
			for (int j = 0 ; j < count; j ++) {
				dFloat pitch = dGaussianRandom  (1.0f) * 2.0f * dPi;
				dFloat yaw = dGaussianRandom  (1.0f) * 2.0f * dPi;
				dFloat roll = dGaussianRandom  (1.0f) * 2.0f * dPi;

				dFloat x = size * (j - count / 2) + dGaussianRandom  (size * 0.5f);
				dFloat y = dGaussianRandom  (size * 2.0f);
				dFloat z = size * (i - count / 2) + dGaussianRandom  (size * 0.5f);

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

		dMatrix alignMatrix (dRollMatrix(dPi * 90.0f / 180.0f));
		for (int i = 0; i < m_count; i ++) {
			shapeSize = dVector (size + dGaussianRandom  (size / 2.0f), size + dGaussianRandom  (size / 2.0f), size + dGaussianRandom  (size / 2.0f), 0.0f);
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
			m_castingGeometries[i] = new DemoMesh("geometry", scene->GetShaderCache(), m_castingShapeArray[i], "smilli.tga", "smilli.tga", "smilli.tga");
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
		glVertex3f (GLfloat(m_rayP0.m_x), GLfloat(m_rayP0.m_y), GLfloat(m_rayP0.m_z));
		glVertex3f (GLfloat(m_rayP1.m_x), GLfloat(m_rayP1.m_y), GLfloat(m_rayP1.m_z));
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


class dConvexCastRecord: public dCustomControllerBase
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


class dConvexCastManager: public dCustomControllerManager<dConvexCastRecord>
{
	public:
	dConvexCastManager(DemoEntityManager* const scene, StupidComplexOfConvexShapes* const stupidLevel)
		:dCustomControllerManager<dConvexCastRecord>(scene->GetNewton(), "dConvexCastManager")
		,m_selectShape (true)
		,m_stupidLevel(stupidLevel)
	{
		scene->Set2DDisplayRenderFunction (RenderHelp, NULL, this);
	}

	static void RenderHelp (DemoEntityManager* const scene, void* const context)
	{
		dConvexCastManager* const me = (dConvexCastManager*) context;
		me->RenderHelp (scene);
	}

	void RenderHelp (DemoEntityManager* const scene)
	{
		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print (color, "mouse point and right click to cast current convex shape");
		scene->Print (color, "space	   : change casting shape");
	}				

	virtual void Debug () const
	{
	};


	static unsigned Prefilter(const NewtonBody* const body, const NewtonCollision* const collision, void* const userData)
	{
		return 1;
	}

	virtual void PreUpdate (dFloat timestep)
	{
		(void)timestep;

		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);
		DemoCamera* const camera = scene->GetCamera();

		//m_helpKey.UpdatePushButton (scene, 'H');
//		bool state = m_engineKeySwitch.UpdatePushButton(scene->GetKeyState('H'));
		if (m_selectShape.UpdateTrigger(scene->GetKeyState(' '))) {
			m_stupidLevel->ChangeCastingShape();
		}

		bool buttonState = scene->GetMouseKeyState(1);
		if (buttonState) {
			int mouseX;
			int mouseY;

			buttonState = false;
			scene->GetMousePosition (mouseX, mouseY);

			dFloat x = dFloat (mouseX);
			dFloat y = dFloat (mouseY);
			dVector p0 (camera->ScreenToWorld(dVector (x, y, 0.0f, 0.0f)));
			dVector p1 (camera->ScreenToWorld(dVector (x, y, 1.0f, 0.0f)));

			//p0 = dVector(-4.426113f, 7.244503f, 15.269794f);
			//p1 = dVector(878.254150f, -742.662842f, -1708.758545f);

			// do the convex cast here
			dMatrix matrix (dGetIdentityMatrix());
			matrix.m_posit = p0;

			dFloat param = 1.2f;
			NewtonCollision* const shape = m_stupidLevel->GetCurrentShape();
			//int count = NewtonWorldConvexCast (world, &matrix[0][0], &p1[0], shape, ConvexCastCallBack::Filter, &filter, ConvexCastCallBack::Prefilter, &filter.m_contacts[0], 4, 0);
			NewtonWorldConvexCast (world, &matrix[0][0], &p1[0], shape, &param, NULL, Prefilter, NULL, 0, 0);

			//dTrace(("%ff, %ff, %ff\n", p0[0], p0[1], p0[2]));
			//dTrace(("%ff, %ff, %ff\n", p1[0], p1[1], p1[2]));

			if (param < 1.0f) {
				matrix.m_posit += (p1 - matrix.m_posit).Scale (param);
				m_stupidLevel->SetCastEntityMatrix (scene, matrix);
			}
			m_stupidLevel->SetCastingLine (p0, p1);
		}
	}

	StupidComplexOfConvexShapes* m_stupidLevel;
	DemoEntityManager::ButtonKey m_selectShape;
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

	DemoMesh* mesh = new DemoMesh("geometry", scene->GetShaderCache(), NewtonBodyGetCollision(compoundBody), "smilli.tga", "smilli.tga", "smilli.tga");
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
	dGetWorkingFileName ("ramp.off", fileName);
	NewtonMesh* const ntMesh = NewtonMeshLoadOFF(world, fileName);

	dMatrix matrix (dGetIdentityMatrix());
	DemoMesh* mesh = new DemoMesh(ntMesh, scene->GetShaderCache());
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
	DemoMesh* const mesh = CreateVisualPlaneMesh (planeEquation, scene->GetShaderCache().m_diffuseEffect);
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
	//dGetWorkingFileName ("low_rez_rim.OFF", fileName);
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






