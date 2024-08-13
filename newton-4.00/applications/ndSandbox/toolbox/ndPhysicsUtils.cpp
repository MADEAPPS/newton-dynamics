/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software./Users/juliojerez/Desktop/newton-dynamics/applications/demosSandbox/sdkDemos/toolBox/PhysicsUtils.cpp
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndDemoMesh.h"
#include "ndDemoEntity.h"
#include "ndPhysicsUtils.h"
#include "ndDemoEntityManager.h"
#include "ndOpenGlUtil.h"
#include "ndDebugDisplay.h"
#include "ndPhysicsWorld.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"
#include "ndHighResolutionTimer.h"

ndVector FindFloor(const ndWorld& world, const ndVector& origin, ndFloat32 dist)
{
	ndVector p0(origin);
	ndVector p1(origin - ndVector(0.0f, ndAbs(dist), 0.0f, 0.0f));

	ndRayCastClosestHitCallback rayCaster;
	if (world.RayCast(rayCaster, p0, p1))
	{
		return rayCaster.m_contact.m_point;
	}
	return p1;
}

ndMatrix FindFloor(const ndWorld& world, const ndMatrix& origin, const ndShapeInstance& shape, ndFloat32 dist)
{
	class ndFindFloorConvexCast : public ndConvexCastNotify
	{
		public:
		ndFindFloorConvexCast(const ndWorld& world, const ndMatrix& origin, const ndShapeInstance& shape, ndFloat32 dist)
			:ndConvexCastNotify()
		{
			ndVector dst(origin.m_posit);
			dst.m_y -= dist;
			world.ConvexCast(*this, shape, origin, dst);
		}

		virtual ndUnsigned32 OnRayPrecastAction(const ndBody* const, const ndShapeInstance* const)
		{
			return 1;
		}
	};

	ndMatrix matrix(origin);
	matrix.m_posit.m_y += dist * 0.5f;
	ndFindFloorConvexCast castShape(world, matrix, shape, dist);
	matrix.m_posit.m_y -= dist * castShape.m_param;
	return matrix;
}

ndBodyKinematic* MousePickBody(ndWorld* const world, const ndVector& origin, const ndVector& end, ndFloat32& paramterOut, ndVector& positionOut, ndVector& normalOut)
{
	class ndRayPickingCallback: public ndRayCastClosestHitCallback
	{
		public: 
		ndRayPickingCallback()
			:ndRayCastClosestHitCallback()
		{
		}

		ndFloat32 OnRayCastAction(const ndContactPoint& contact, ndFloat32 intersetParam)
		{
			if (contact.m_body0->GetInvMass() == ndFloat32(0.0f)) 
			{
				return 1.2f;
			}
			return ndRayCastClosestHitCallback::OnRayCastAction(contact, intersetParam);
		}
	};

	ndRayPickingCallback rayCaster;
	if (world->RayCast(rayCaster, origin, end))
	{
		paramterOut = rayCaster.m_param;
		positionOut = rayCaster.m_contact.m_point;
		normalOut = rayCaster.m_contact.m_normal;
		return (ndBodyKinematic* )rayCaster.m_contact.m_body0;
	}

	return nullptr;
}

void AddPlanks(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndInt32 count)
{
	ndMatrix matrix(location);
	for (ndInt32 i = 0; i < count; ++i)
	{
		for (ndInt32 j = 0; j < count; ++j)
		{
			matrix.m_posit = location.m_posit + ndVector(((ndFloat32)i - 2)* 5.0f, 0.0f, ((ndFloat32)j - 2) * 5.0f, 0.0f);
			AddBox(scene, matrix, mass, 4.0f, 0.25f, 3.0f);
		}
	}
}

static void AddShape(ndDemoEntityManager* const scene,
	ndDemoInstanceEntity* const rootEntity, const ndShapeInstance& shape,
	ndFloat32 mass, const ndMatrix& location, const ndFloat32 high, ndInt32 count)
{
	ndMatrix matrix(location);
	ndPhysicsWorld* const world = scene->GetWorld();

	ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y = floor.m_y + high + 7.0f;

	for (ndInt32 i = 0; i < count; ++i)
	{
		ndBodyKinematic* const body = new ndBodyDynamic();
		ndDemoEntity* const entity = new ndDemoEntity(matrix, rootEntity);

		body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
		body->SetMatrix(matrix);
		body->SetCollisionShape(shape);
		body->SetMassMatrix(mass, shape);
		body->SetAngularDamping(ndVector(ndFloat32(0.5f)));
		
		ndSharedPtr<ndBody> bodyPtr(body);
		world->AddBody(bodyPtr);
		matrix.m_posit.m_y += high * 2.5f;
	}
}

void AddCapsulesStacks(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 radius0, ndFloat32 radius1, ndFloat32 high, ndInt32 rows_x, ndInt32 rows_z, ndInt32 columHigh)
{
	ndShapeInstance shape(new ndShapeCapsule(radius0, radius1, high));
	ndSharedPtr<ndDemoMeshIntance> instanceMesh (new ndDemoMeshIntance("shape", scene->GetShaderCache(), &shape, "marble.png", "marble.png", "marble.png"));
	
	ndDemoInstanceEntity* const rootEntity = new ndDemoInstanceEntity(instanceMesh);
	scene->AddEntity(rootEntity);
	
	ndFloat32 spacing = 2.0f;
	ndMatrix matrix(ndRollMatrix(90.0f * ndDegreeToRad));
	for (ndInt32 z = 0; z < rows_z; ++z)
	{
		for (ndInt32 x = 0; x < rows_x; ++x)
		{
			matrix.m_posit = location.m_posit + ndVector ((ndFloat32)(x - rows_x / 2) * spacing, 0.0f, (ndFloat32)(z - rows_z / 2) * spacing, 0.0f);
			AddShape(scene, rootEntity, shape, mass, matrix, high, columHigh);
		}
	}
}

static ndBodyKinematic* CreateBody(ndDemoEntityManager* const scene, const ndShapeInstance& shape, const ndMatrix& location, ndFloat32 mass, const char* const textName)
{
	ndPhysicsWorld* const world = scene->GetWorld();

	ndMatrix matrix(FindFloor(*world, location, shape, 200.0f));
	ndSharedPtr<ndDemoMeshInterface> mesh (new ndDemoMesh("shape", scene->GetShaderCache(), &shape, textName, textName, textName));

	ndBodyKinematic* const kinBody = new ndBodyDynamic();
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(mesh);
	
	kinBody->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	kinBody->SetMatrix(matrix);
	kinBody->SetCollisionShape(shape);
	kinBody->SetMassMatrix(mass, shape);
	return kinBody;
}

ndBodyKinematic* CreateBox(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 sizex, ndFloat32 sizey, ndFloat32 sizez, const char* const textName)
{
	ndShapeInstance shape(new ndShapeBox(sizex, sizey, sizez));
	ndBodyKinematic* const body = CreateBody(scene, shape, location, mass, textName);
	return body;
}

ndBodyKinematic* AddBox(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 sizex, ndFloat32 sizey, ndFloat32 sizez, const char* const textName)
{
	ndBodyKinematic* const body = CreateBox(scene, location, mass, sizex, sizey, sizez, textName);

	ndPhysicsWorld* const world = scene->GetWorld();
	world->AddBody(body);

	ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)body->GetNotifyCallback();
	scene->AddEntity(notify->m_entity);
	return body;
}

ndBodyKinematic* CreateSphere(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 radius, const char* const textName)
{
	ndShapeInstance shape(new ndShapeSphere(radius));
	ndBodyKinematic* const body = CreateBody(scene, shape, location, mass, textName);
	return body;
}

ndBodyKinematic* AddSphere(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 radius, const char* const textName)
{
	ndBodyKinematic* const body = CreateSphere(scene, location, mass, radius, textName);

	ndPhysicsWorld* const world = scene->GetWorld();
	world->AddBody(body);

	ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)body->GetNotifyCallback();
	scene->AddEntity(notify->m_entity);
	return body;
}

ndBodyKinematic* CreateCapsule(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 radius0, ndFloat32 radius1, ndFloat32 high, const char* const textName)
{
	ndShapeInstance shape(new ndShapeCapsule(radius0, radius1, high));
	ndBodyKinematic* const body = CreateBody(scene, shape, location, mass, textName);
	return body;
}

ndBodyKinematic* AddCapsule(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 radius0, ndFloat32 radius1, ndFloat32 high, const char* const textName)
{
	ndBodyKinematic* const body = CreateCapsule(scene, location, mass, radius0, radius1, high, textName);

	ndPhysicsWorld* const world = scene->GetWorld();
	world->AddBody(body);

	ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)body->GetNotifyCallback();
	scene->AddEntity(notify->m_entity);
	return body;
}

ndBodyKinematic* AddConvexHull(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 radius, ndFloat32 high, ndInt32 segments, const char* const textName)
{
	ndArray<ndVector> points;
	ndFloat32 den = ndFloat32(segments);
	for (ndInt32 i = 0; i < segments; ++i)
	{
		ndFloat32 angle = ndFloat32(2.0f) * ndPi * ndFloat32(i) / den;
		ndFloat32 x = radius * ndCos(angle);
		ndFloat32 z = radius * ndSin(angle);
		points.PushBack(ndVector(0.7f * x, -high * 0.5f, 0.7f * z, 0.0f));
		points.PushBack(ndVector(0.7f * x,  high * 0.5f, 0.7f * z, 0.0f));
		points.PushBack(ndVector(x, -high * 0.25f, z, 0.0f));
		points.PushBack(ndVector(x, high * 0.25f, z, 0.0f));
	}

	ndShapeInstance shape(new ndShapeConvexHull(ndInt32 (points.GetCount()), sizeof(ndVector), 0.0f, &points[0].m_x));
	ndBodyKinematic* const body = CreateBody(scene, shape, location, mass, textName);

	ndPhysicsWorld* const world = scene->GetWorld();
	world->AddBody(body);

	ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)body->GetNotifyCallback();
	scene->AddEntity(notify->m_entity);

	return body;
}

void SetModelVisualMesh(ndDemoEntityManager* const scene, ndModelArticulation* const model)
{
	for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
	{
		ndSharedPtr<ndBody> body(node->m_body);

		ndUrdfBodyNotify* const urdfNotify = body->GetNotifyCallback()->GetAsUrdfBodyNotify();
		if (urdfNotify)
		{
			ndSharedPtr<ndDemoMeshInterface> mesh(new ndDemoMesh("urdfMesh", *urdfNotify->m_mesh, scene->GetShaderCache()));
			ndDemoEntity* const entity = new ndDemoEntity(body->GetMatrix(), nullptr);
			entity->SetMesh(mesh);
			entity->SetMeshMatrix(urdfNotify->m_offset);
			scene->AddEntity(entity);
			body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
		}
	}
}