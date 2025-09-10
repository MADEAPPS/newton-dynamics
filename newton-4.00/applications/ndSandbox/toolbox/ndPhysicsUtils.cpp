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
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
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

#if 0
void SetModelVisualMesh(ndDemoEntityManager* const scene, ndModelArticulation* const model)
{
	for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
	{
		ndSharedPtr<ndBody> body(node->m_body);

		ndUrdfBodyNotify* const urdfNotify = body->GetNotifyCallback()->GetAsUrdfBodyNotify();
		if (urdfNotify)
		{
			ndSharedPtr<ndDemoMeshInterface> mesh(new ndDemoMesh("urdfMesh", *urdfNotify->m_mesh, scene->GetShaderCache()));
			ndSharedPtr<ndDemoEntity> entity (new ndDemoEntity(body->GetMatrix()));
			entity->SetMesh(mesh);
			entity->SetMeshMatrix(urdfNotify->m_offset);

			scene->AddEntity(entity);
			body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
		}
	}
}
#endif

//******************************************************************************
// Create simple rigi body with a collsion shspe and render primitev but is not added to the scene
//******************************************************************************
ndSharedPtr<ndBody> CreateBody(
	ndDemoEntityManager* const scene, 
	const ndShapeInstance& shape, 
	const ndMatrix& location, 
	ndFloat32 mass, 
	const char* const textName,
	ndRenderPrimitiveMesh::ndUvMapingMode mappingMode)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	ndRender* const render = *scene->GetRenderer();

	const ndMatrix matrix(FindFloor(*world, location, shape, 200.0f));

	ndRenderPrimitiveMeshMaterial material;
	material.m_texture = render->GetTextureCache()->GetTexture(ndGetWorkingFileName(textName));

	ndSharedPtr<ndRenderPrimitive> mesh(ndRenderPrimitiveMesh::CreateFromCollisionShape(render, &shape, material, mappingMode));
	ndSharedPtr<ndBody> body (new ndBodyDynamic());
	ndSharedPtr<ndRenderSceneNode>entity(new ndRenderSceneNode(matrix));
	entity->SetPrimitive(mesh);

	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->GetAsBodyKinematic()->SetCollisionShape(shape);
	body->GetAsBodyKinematic()->SetMassMatrix(mass, shape);
	return body;
}

//******************************************************************************
// Create simple primitive but is no added to the scene
//******************************************************************************
ndSharedPtr<ndBody> CreateBox(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 sizex, ndFloat32 sizey, ndFloat32 sizez, const char* const textName)
{
	ndShapeInstance shape(new ndShapeBox(sizex, sizey, sizez));
	ndSharedPtr<ndBody> body(CreateBody(scene, shape, location, mass, textName, ndRenderPrimitiveMesh::m_box));
	return body;
}

ndSharedPtr<ndBody> CreateSphere(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 radius, const char* const textName)
{
	ndShapeInstance shape(new ndShapeSphere(radius));
	ndSharedPtr<ndBody> body(CreateBody(scene, shape, location, mass, textName, ndRenderPrimitiveMesh::m_spherical));
	return body;
}

ndSharedPtr<ndBody> CreateCapsule(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 radius0, ndFloat32 radius1, ndFloat32 high, const char* const textName)
{
	ndShapeInstance shape(new ndShapeCapsule(radius0, radius1, high));
	ndSharedPtr<ndBody> body(CreateBody(scene, shape, location, mass, textName, ndRenderPrimitiveMesh::m_cylindrical));
	return body;
}

//******************************************************************************
// Create simple primitive and add to the scene
//******************************************************************************
ndSharedPtr<ndBody> AddSphere(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 radius, const char* const textName)
{
	ndSharedPtr<ndBody> body(CreateSphere(scene, location, mass, radius, textName));
	
	ndPhysicsWorld* const world = scene->GetWorld();
	world->AddBody(body);
	
	ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)body->GetNotifyCallback();
	scene->AddEntity(notify->m_entity);
	return body;
}

ndSharedPtr<ndBody> AddBox(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 sizex, ndFloat32 sizey, ndFloat32 sizez, const char* const textName)
{
	ndSharedPtr<ndBody> body(CreateBox(scene, location, mass, sizex, sizey, sizez, textName));

	ndPhysicsWorld* const world = scene->GetWorld();
	world->AddBody(body);

	ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)body->GetNotifyCallback();
	scene->AddEntity(notify->m_entity);
	return body;
}

ndSharedPtr<ndBody> AddCapsule(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 radius0, ndFloat32 radius1, ndFloat32 high, const char* const textName)
{
	ndSharedPtr<ndBody> body = CreateCapsule(scene, location, mass, radius0, radius1, high, textName);

	ndPhysicsWorld* const world = scene->GetWorld();
	world->AddBody(body);

	ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)body->GetNotifyCallback();
	scene->AddEntity(notify->m_entity);
	return body;
}

ndSharedPtr<ndBody> AddConvexHull(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 radius, ndFloat32 high, ndInt32 segments, const char* const textName)
{
	ndArray<ndVector> points;
	ndFloat32 den = ndFloat32(segments);
	for (ndInt32 i = 0; i < segments; ++i)
	{
		ndFloat32 angle = ndFloat32(2.0f) * ndPi * ndFloat32(i) / den;
		ndFloat32 x = radius * ndCos(angle);
		ndFloat32 z = radius * ndSin(angle);
		points.PushBack(ndVector(0.7f * x, -high * 0.5f, 0.7f * z, 0.0f));
		points.PushBack(ndVector(0.7f * x, high * 0.5f, 0.7f * z, 0.0f));
		points.PushBack(ndVector(x, -high * 0.25f, z, 0.0f));
		points.PushBack(ndVector(x, high * 0.25f, z, 0.0f));
	}

	ndShapeInstance shape(new ndShapeConvexHull(ndInt32(points.GetCount()), sizeof(ndVector), 0.0f, &points[0].m_x));
	ndSharedPtr<ndBody> body(CreateBody(scene, shape, location, mass, textName, ndRenderPrimitiveMesh::m_box));

	ndPhysicsWorld* const world = scene->GetWorld();
	world->AddBody(body);

	ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)body->GetNotifyCallback();
	scene->AddEntity(notify->m_entity);
	return body;
}

// ************************************************************************
// add array of some primitive to the scene
// ************************************************************************
void AddPlanks(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndInt32 count)
{
	ndMatrix matrix(location);

	for (ndInt32 i = 0; i < count; ++i)
	{
		for (ndInt32 j = 0; j < count; ++j)
		{
			matrix.m_posit = location.m_posit + ndVector(((ndFloat32)i - 2) * 5.0f, 0.0f, ((ndFloat32)j - 2) * 5.0f, 0.0f);
			ndSharedPtr<ndBody> body (AddBox(scene, matrix, mass, 4.0f, 0.25f, 3.0f));
		}
	}
}

void AddCapsulesStacks(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 radius0, ndFloat32 radius1, ndFloat32 high, ndInt32 rows_x, ndInt32 rows_z, ndInt32 columHigh)
{
	ndShapeInstance shape(new ndShapeCapsule(radius0, radius1, high));

	ndRenderPrimitiveMeshMaterial material;
	ndRender* const render = *scene->GetRenderer();
	material.m_texture = render->GetTextureCache()->GetTexture(ndGetWorkingFileName("marble.png"));
	ndSharedPtr<ndRenderPrimitive> mesh(ndRenderPrimitiveMesh::CreateFromCollisionShape(render, &shape, material, ndRenderPrimitiveMesh::m_spherical));

	ndSharedPtr<ndRenderSceneNode>root(new ndRenderSceneNodeInstance(location));
	scene->AddEntity(root);

	ndFloat32 spacing = 2.0f;
	const ndMatrix startMatrix(ndRollMatrix(90.0f * ndDegreeToRad));

	const ndMatrix invLocation(location.OrthoInverse());
	ndPhysicsWorld* const world = scene->GetWorld();
	for (ndInt32 z = 0; z < rows_z; ++z)
	{
		for (ndInt32 x = 0; x < rows_x; ++x)
		{
			ndMatrix matrix(startMatrix);
			matrix.m_posit = location.m_posit + ndVector((ndFloat32)(x - rows_x / 2) * spacing, 0.0f, (ndFloat32)(z - rows_z / 2) * spacing, ndFloat32(0.0f));

			ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
			matrix.m_posit.m_y = floor.m_y + high + 7.0f;

			for (ndInt32 i = 0; i < columHigh; ++i)
			{
				const ndMatrix localMatrix(matrix * invLocation);
				ndSharedPtr<ndRenderSceneNode>instance(new ndRenderSceneNode(localMatrix));
				instance->SetPrimitive(mesh);
				root->AddChild(instance);

				ndSharedPtr<ndBody> body(new ndBodyDynamic());
				body->SetNotifyCallback(new ndDemoEntityNotify(scene, instance));
				body->SetMatrix(matrix);
				body->GetAsBodyKinematic()->SetCollisionShape(shape);
				body->GetAsBodyKinematic()->SetMassMatrix(mass, shape);
				body->GetAsBodyDynamic()->SetAngularDamping(ndVector(ndFloat32(0.5f)));
				world->AddBody(body);
				matrix.m_posit.m_y += high * 2.5f;
			}
		}
	}
}
