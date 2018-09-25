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
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "DemoMesh.h"
#include "../toolBox/OpenGlUtil.h"



class dClosestDistanceRecord: public dCustomControllerBase
{
	class ClosestDistanceEntity: public DemoEntity
	{
		public:
		ClosestDistanceEntity (DemoEntityManager* const scene, dMatrix& matrix, int materialID, PrimitiveType castingShapeType)
			:DemoEntity (matrix, NULL)
			,m_normal(0.0f)
			,m_contact0(0.0f)
			,m_contact1(0.0f)
			,m_contactsCount(0)
		{
			NewtonWorld* const world = scene->GetNewton();

			dVector size(1.0f, 1.0f, 1.0f, 0.0f);
			m_castingShape = CreateConvexCollision (world, dGetIdentityMatrix(), size, castingShapeType, 0);

			DemoMesh* const geometry = new DemoMesh("convexShape", m_castingShape, "smilli.tga", "smilli.tga", "smilli.tga");
			SetMesh(geometry, dGetIdentityMatrix());
			geometry->Release();

			scene->Append(this);
		}

		~ClosestDistanceEntity()
		{
			NewtonDestroyCollision (m_castingShape);
		}

		virtual void Render(dFloat timeStep, DemoEntityManager* const scene) const
		{
			DemoEntity::Render(timeStep, scene);

			ShowMousePicking (m_contact0, m_contact1, dVector (1, 0, 0, 0), dVector (0, 1, 0, 0));
			ShowMousePicking (m_contact1, m_contact0, dVector (1, 0, 0, 0), dVector (0, 1, 0, 0));
		}

		dVector m_normal;
		dVector m_contact0;
		dVector m_contact1;
		NewtonCollision* m_castingShape;
		int m_contactsCount;
	};


	public:
	void PreUpdate(dFloat timestep, int threadIndex)
	{
	}

	void PostUpdate(dFloat timestep, int threadIndex)
	{
		dMatrix matrixA;
		NewtonBodyGetMatrix(m_body, &matrixA[0][0]);

		dFloat speed = m_step * timestep * 60.0f; 
//speed = 0;
		m_pith = dMod (m_pith + speed, dPi * 2.0f);
		m_yaw = dMod (m_yaw + speed, dPi * 2.0f);
		m_roll = dMod (m_roll + speed, dPi * 2.0f);

		dMatrix matrixB(dPitchMatrix(m_pith) * dYawMatrix(m_yaw) * dRollMatrix(m_roll));
		matrixB.m_posit = matrixA.m_posit;

		//matrixB.m_posit.m_y = 5.0f;
		matrixB.m_posit.m_y = 1.5f;

		NewtonWorld* const world = NewtonBodyGetWorld(m_body);
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData (world);

		m_castingVisualEntity->ResetMatrix(*scene, matrixB);

		NewtonCollision* const collisionA = NewtonBodyGetCollision(m_body);

		int res = NewtonCollisionClosestPoint(world, collisionA, &matrixA[0][0], m_castingVisualEntity->m_castingShape, &matrixB[0][0], &m_castingVisualEntity->m_contact0[0], &m_castingVisualEntity->m_contact1[0], &m_castingVisualEntity->m_normal[0], 0);

		//just test the center of collisionB against collisionA to see if the point is inside or not:
		//int res = NewtonCollisionPointDistance(world, &matrixA.m_posit[0], collisionA, &matrixA[0][0], &m_castingVisualEntity->m_contact0[0], &m_castingVisualEntity->m_normal[0], 0);
		
		if (res == 0){
			printf("Point Inside Body!\n");
			//dTrace(("Point Inside Body!\n"));
		}
		else
		{
			printf("Point point oustide Body!\n");
		}
	}

	void Init (dFloat location_x, dFloat location_z, PrimitiveType shapeType, int materialID, PrimitiveType castingShapeType)
	{
		m_pith = dGaussianRandom (dPi * 2.0f);
		m_yaw = dGaussianRandom (dPi * 2.0f);
		m_roll = dGaussianRandom (dPi * 2.0f);
		m_step = 15.0f * (dAbs (dGaussianRandom (0.25f)) + 0.0001f) * dDegreeToRad;
//m_pith -= 0.5f;

		CreatCasterBody(location_x, location_z, shapeType, materialID);

		NewtonWorld* const world = ((dCustomControllerManager<dClosestDistanceRecord>*)GetManager())->GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		dMatrix matrix;
		NewtonBodyGetMatrix(m_body, &matrix[0][0]);
		matrix.m_posit.m_y += 10.0f;
		m_castingVisualEntity = new ClosestDistanceEntity (scene, matrix, materialID, castingShapeType);
	}

	void Debug(dCustomJoint::dDebugDisplay* const debugContext) const
	{
		dAssert(0);
	}


	private:
	void CreatCasterBody(dFloat location_x, dFloat location_z, PrimitiveType shapeType, int materialID)
	{
		NewtonWorld* const world = ((dCustomControllerManager<dClosestDistanceRecord>*)GetManager())->GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		//dMatrix matrix (GetIdentityMatrix());
		dMatrix matrix (dRollMatrix(dPi/2.0f));

		matrix.m_posit.m_x = location_x;
		matrix.m_posit.m_y = 2.0f;
		matrix.m_posit.m_z = location_z;

		// create the shape and visual mesh as a common data to be re used
		dVector size(0.5f, 0.5f, 0.75f, 0.0f);
		NewtonCollision* const collision = CreateConvexCollision (world, dGetIdentityMatrix(), size, shapeType, materialID);

		DemoMesh* const geometry = new DemoMesh("convexShape", collision, "smilli.tga", "smilli.tga", "smilli.tga");
		m_body = CreateSimpleSolid (scene, geometry, 1.0f, matrix, collision, materialID);


		geometry->Release(); 
		NewtonDestroyCollision (collision);
	}

	NewtonBody* m_body;
	ClosestDistanceEntity* m_castingVisualEntity;

	dFloat m_pith;
	dFloat m_yaw;
	dFloat m_roll;
	dFloat m_step;
};


class dClosestDistanceManager: public dCustomControllerManager<dClosestDistanceRecord> 
{
	public:
	dClosestDistanceManager(DemoEntityManager* const scene)
		:dCustomControllerManager<dClosestDistanceRecord>(scene->GetNewton(), "dConvexCastManager")
	{
	}

	virtual void Debug () const {};


	void AddPrimitives (int count, const dVector& location, PrimitiveType shapeType, int materialID, PrimitiveType castingShapeType)
	{
		dFloat step = 4.0f;
		dFloat z = location.m_x - step * count / 2;
		for (int i = 0; i < count; i ++) {
			dFloat x = location.m_x - step * count / 2;
			for (int j = 0; j < count; j ++) {
				dClosestDistanceRecord* const caster = (dClosestDistanceRecord*)CreateController();
				caster->Init(x, z, shapeType, materialID, castingShapeType);
				x += step;
			}
			z += step;
		}
	}
};


// create physics scene
void ClosestDistance (DemoEntityManager* const scene)
{
	scene->CreateSkyBox();

	// customize the scene after loading
	// set a user friction variable in the body for variable friction demos
	// later this will be done using LUA script
	NewtonWorld* const world = scene->GetNewton();
	dMatrix offsetMatrix (dGetIdentityMatrix());

	CreateLevelMesh (scene, "flatPlane.ngd", 1);
	//	CreateLevelMesh (scene, "playground.ngd", 0);

	int materialID = NewtonMaterialGetDefaultGroupID (world);

	dClosestDistanceManager* const castManager = new dClosestDistanceManager (scene);

	//PrimitiveType castinShapeType = _SPHERE_PRIMITIVE;
	//PrimitiveType castinShapeType = _BOX_PRIMITIVE;
	//PrimitiveType castinShapeType = _CAPSULE_PRIMITIVE;
	//PrimitiveType castinShapeType = _CYLINDER_PRIMITIVE;
	//PrimitiveType castinShapeType = _CONE_PRIMITIVE;
	//PrimitiveType castinShapeType = _CHAMFER_CYLINDER_PRIMITIVE;
	//PrimitiveType castinShapeType = _RANDOM_CONVEX_HULL_PRIMITIVE;
	//PrimitiveType castinShapeType = _REGULAR_CONVEX_HULL_PRIMITIVE;
	PrimitiveType castinShapeType = _COMPOUND_CONVEX_CRUZ_PRIMITIVE;

	int count = 1;
	castManager->AddPrimitives (count, dVector (  0, 0, 0), _SPHERE_PRIMITIVE, materialID, castinShapeType);
	//castManager->AddPrimitives (count, dVector (  2, 0, 2), _BOX_PRIMITIVE, materialID, castinShapeType);
	//castManager->AddPrimitives (count, dVector (  4, 0, 4), _CAPSULE_PRIMITIVE, materialID, castinShapeType);
	//castManager->AddPrimitives (count, dVector (  8, 0, 8), _CYLINDER_PRIMITIVE, materialID, castinShapeType);
	//castManager->AddPrimitives (count, dVector ( 10, 0, 10), _CHAMFER_CYLINDER_PRIMITIVE, materialID, castinShapeType);
	//castManager->AddPrimitives (count, dVector (- 4, 0, -4), _CONE_PRIMITIVE, materialID, castinShapeType);
	//castManager->AddPrimitives (count, dVector (-10, 0, -10), _REGULAR_CONVEX_HULL_PRIMITIVE, materialID, castinShapeType);
	//castManager->AddPrimitives (count, dVector (-12, 0, -12), _COMPOUND_CONVEX_CRUZ_PRIMITIVE, materialID, castinShapeType);


	// place camera into position
	//dMatrix camMatrix (dYawMatrix(90.0f * dDegreeToRad));
	dMatrix camMatrix (dGetIdentityMatrix());
	dQuaternion rot (camMatrix);
	dVector origin (-30.0f, 10.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}




