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
	ndVector dir(ndVector::m_zero);
	dir.m_y = dist;

	ndVector p0(origin + dir.Scale (ndFloat32 (0.5f)));
	ndVector p1(p0 - dir);
	ndRayCastClosestHitCallback rayCaster;
	if (world.RayCast(rayCaster, p0, p1))
	{
		rayCaster.m_contact.m_point.m_w = ndFloat32(1.0f);
		return rayCaster.m_contact.m_point;
	}
	p0.m_w = ndFloat32(1.0f);
	return p0;
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
			dst.m_w = ndFloat32(1.0f);
			m_floor = origin.m_posit;
			if (((ndShape*)shape.GetShape())->GetAsShapeCompound())
			{
				//ndVector size;
				//ndVector obbOrigin;
				//shape.CalculateObb(obbOrigin, size);
				//size = size.Scale(ndFloat32 (2.0f));
				//ndShapeInstance sorrugate(new ndShapeBox(size.m_x, size.m_y, size.m_z));
				//ndMatrix surrugateMatrix(ndGetIdentityMatrix());
				//
				////ndMatrix xxxxx(shape.CalculateInertia());
				////ndVector xxxxxx(xxxxx.EigenVectors());
				//
				//surrugateMatrix.m_posit = obbOrigin;
				//surrugateMatrix.m_posit.m_w = ndFloat32(1.0f);
				//sorrugate.SetLocalMatrix(surrugateMatrix);

				ndFloat32 radios = shape.GetBoxMinRadius();
				ndShapeInstance sorrugate(new ndShapeSphere(radios));
				world.ConvexCast(*this, sorrugate, origin, dst);
				if (m_param < ndFloat32(1.0f))
				{
					ndVector size;
					ndVector obbOrigin;
					shape.CalculateObb(obbOrigin, size);
					m_floor.m_y -= dist * m_param;
					m_floor.m_y += (size.m_y - radios);
				}
			}
			else
			{
				world.ConvexCast(*this, shape, origin, dst);
				if (m_param < ndFloat32(1.0f))
				{
					m_floor.m_y -= dist * m_param;
				}
			}
		}

		virtual ndUnsigned32 OnRayPrecastAction(const ndBody* const, const ndShapeInstance* const)
		{
			return 1;
		}
		ndVector m_floor;
	};

	ndMatrix matrix(origin);
	matrix.m_posit.m_y += dist * ndFloat32 (0.5f);
	ndFindFloorConvexCast castShape(world, matrix, shape, dist);
	matrix.m_posit = castShape.m_floor;
	return matrix;
}

ndVector FindFloor(const ndWorld& world, const ndVector& origin, const ndShapeInstance& shape, ndFloat32 dist)
{
	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit = origin;
	const ndMatrix castMatrix(FindFloor(world, matrix, shape, dist));
	return castMatrix.m_posit;
}

//******************************************************************************
// Create simple rigid body with a collsion shspe and render primitev but 
// is not added to the scene
//******************************************************************************
ndSharedPtr<ndBody> CreateRigidbody(
	ndDemoEntityManager* const scene, 
	ndSharedPtr<ndShapeInstance>& shape,
	const ndMatrix& location, 
	ndFloat32 mass, 
	const char* const textName,
	ndRenderPrimitive::ndUvMapingMode mappingMode)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	ndRender* const render = *scene->GetRenderer();

	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_mapping = mappingMode;
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName(textName)));
	ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));

	const ndMatrix matrix(FindFloor(*world, location, **shape, 200.0f));
	ndSharedPtr<ndRenderSceneNode>entity(new ndRenderSceneNode(matrix));
	entity->SetPrimitive(mesh);

	ndSharedPtr<ndBody> body (new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->GetAsBodyKinematic()->SetCollisionShape(**shape);
	body->GetAsBodyKinematic()->SetMassMatrix(mass, **shape);
	return body;
}

//******************************************************************************
// Create simple primitive but is no added to the scene
//******************************************************************************
ndSharedPtr<ndBody> CreateBox(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 sizex, ndFloat32 sizey, ndFloat32 sizez, const char* const textName)
{
	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeBox(sizex, sizey, sizez)));
	ndSharedPtr<ndBody> body(CreateRigidbody(scene, shape, location, mass, textName, ndRenderPrimitive::m_box));
	return body;
}

ndSharedPtr<ndBody> CreateSphere(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 radius, const char* const textName)
{
	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeSphere(radius)));
	ndSharedPtr<ndBody> body(CreateRigidbody(scene, shape, location, mass, textName, ndRenderPrimitive::m_spherical));
	return body;
}

ndSharedPtr<ndBody> CreateCapsule(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 radius0, ndFloat32 radius1, ndFloat32 high, const char* const textName)
{
	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeCapsule(radius0, radius1, high)));
	ndSharedPtr<ndBody> body(CreateRigidbody(scene, shape, location, mass, textName, ndRenderPrimitive::m_capsule));
	return body;
}

ndSharedPtr<ndBody> CreateCylinder(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 radius0, ndFloat32 radius1, ndFloat32 high, const char* const textName)
{
	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeCylinder(radius0, radius1, high)));
	ndSharedPtr<ndBody> body(CreateRigidbody(scene, shape, location, mass, textName, ndRenderPrimitive::m_cylindrical));
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
	ndSharedPtr<ndBody> body (CreateCapsule(scene, location, mass, radius0, radius1, high, textName));

	ndPhysicsWorld* const world = scene->GetWorld();
	world->AddBody(body);

	ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)body->GetNotifyCallback();
	scene->AddEntity(notify->m_entity);
	return body;
}

ndSharedPtr<ndBody> AddCylinder(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 radius0, ndFloat32 radius1, ndFloat32 high, const char* const textName)
{
	ndSharedPtr<ndBody> body (CreateCylinder(scene, location, mass, radius0, radius1, high, textName));
	//ndSharedPtr<ndBody> body(CreateCapsule(scene, location, mass, radius0, radius1, high, textName));

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

	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeConvexHull(ndInt32(points.GetCount()), sizeof(ndVector), 0.0f, &points[0].m_x)));
	ndSharedPtr<ndBody> body(CreateRigidbody(scene, shape, location, mass, textName, ndRenderPrimitive::m_box));

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
			//matrix.m_posit = location.m_posit + ndVector(((ndFloat32)i - 2) * 5.0f, 0.0f, ((ndFloat32)j - 2) * 5.0f, 0.0f);
			ndSharedPtr<ndBody> body (AddBox(scene, matrix, mass, 4.0f, 0.25f, 3.0f));
		}
	}
}

void AddCapsuleStacks(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 radius0, ndFloat32 radius1, ndFloat32 high, ndInt32 rows_x, ndInt32 rows_z, ndInt32 columHigh)
{
	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeCapsule(radius0, radius1, high)));

	ndRender* const render = *scene->GetRenderer();
	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_stretchMaping = false;
	descriptor.m_mapping = ndRenderPrimitive::m_spherical;
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("marble.png")));
	ndSharedPtr<ndRenderSceneNode>root(new ndRenderSceneNodeInstance(location, descriptor));
	scene->AddEntity(root);

	ndFloat32 spacing = 2.0f;
	const ndMatrix startMatrix(ndRollMatrix(90.0f * ndDegreeToRad));

	const ndMatrix bindMatrix(location.OrthoInverse());
	ndPhysicsWorld* const world = scene->GetWorld();
	for (ndInt32 z = 0; z < rows_z; ++z)
	{
		for (ndInt32 x = 0; x < rows_x; ++x)
		{
			ndMatrix matrix(startMatrix);
			matrix.m_posit = location.m_posit + ndVector((ndFloat32)(x - rows_x / 2) * spacing, 0.0f, (ndFloat32)(z - rows_z / 2) * spacing, ndFloat32(0.0f));

			ndVector floor(FindFloor(*world, matrix.m_posit, **shape, 200.0f));
			matrix.m_posit.m_y = floor.m_y + high + 7.0f;
			for (ndInt32 i = 0; i < columHigh; ++i)
			{
				const ndMatrix localMatrix(matrix * bindMatrix);
				ndSharedPtr<ndRenderSceneNode>instance(new ndRenderSceneNode(localMatrix));
				root->AddChild(instance);
			
				ndSharedPtr<ndBody> body(new ndBodyDynamic());
				body->SetNotifyCallback(new ndDemoEntityNotify(scene, instance));
				body->SetMatrix(matrix);
				body->GetAsBodyKinematic()->SetCollisionShape(**shape);
				body->GetAsBodyKinematic()->SetMassMatrix(mass, **shape);
				body->GetAsBodyDynamic()->SetAngularDamping(ndVector(ndFloat32(0.5f)));
				world->AddBody(body);
				matrix.m_posit.m_y += high * 2.5f;
			}
		}
	}

	ndRenderSceneNodeInstance* const instanceRoot = (ndRenderSceneNodeInstance*)*root;
	instanceRoot->Finalize();
}
