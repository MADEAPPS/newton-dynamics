/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndContactCallback.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndArchimedesBuoyancyVolume.h"

static void AddTrigger(ndDemoEntityManager* const scene)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	
	// make the matrix for this body
	ndVector floor(FindFloor(*world, ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit = floor;
	matrix.m_posit.m_w = 1.0f;
	matrix.m_posit.m_y += 2.0f;

	// make a visual mesh
	ndShapeInstance shape(new ndShapeBox(20.0f, 10.0f, 20.0f));
	ndRenderPrimitiveMeshMaterial material;
	material.m_texture = scene->GetRenderer()->GetTextureCache()->GetTexture(ndGetWorkingFileName("metal_30.png"));
	ndSharedPtr<ndRenderPrimitive> mesh(ndRenderPrimitiveMesh::CreateFromCollisionShape(*scene->GetRenderer(), &shape, material, ndRenderPrimitiveMesh::m_box));
	ndSharedPtr<ndRenderSceneNode>entity(new ndRenderSceneNode(matrix));
	entity->SetPrimitive(mesh);

	// make a trigger rigid body and asign mesh and collsion to the rigid body
	ndSharedPtr<ndBody> body(new ndArchimedesBuoyancyVolume());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->GetAsBodyKinematic()->SetCollisionShape(shape);
	world->AddBody(body);
	scene->AddEntity(entity);
}

//static void AddBox(ndDemoEntityManager* const scene, const ndMatrix& origin, ndFloat32 density, ndFloat32 mass)
//{
//	ndSharedPtr<ndBody> body = AddBox(scene, origin, mass, 1.0f, 1.0f, 1.0f);
//	ndShapeMaterial material;
//	material.m_userParam[ndDemoContactCallback::m_density].m_floatData = density;
//	body->GetAsBodyKinematic()->GetCollisionShape().SetMaterial(material);
//}
//
//static void AddSphere1(ndDemoEntityManager* const scene, const ndMatrix& origin, ndFloat32 density, ndFloat32 mass)
//{
//	ndSharedPtr<ndBody> body = AddSphere(scene, origin, mass, 0.5f);
//	ndShapeMaterial material;
//	material.m_userParam[ndDemoContactCallback::m_density].m_floatData = density;
//	body->GetAsBodyKinematic()->GetCollisionShape().SetMaterial(material);
//}
//
//static void AddCapsule(ndDemoEntityManager* const scene, const ndMatrix& origin, ndFloat32 density, ndFloat32 mass)
//{
//	ndSharedPtr<ndBody> body = AddCapsule(scene, origin, mass, 0.5f, 0.5f, 1.0f);
//	ndShapeMaterial material;
//	material.m_userParam[ndDemoContactCallback::m_density].m_floatData = density;
//	body->GetAsBodyKinematic()->GetCollisionShape().SetMaterial(material);
//}
//
//static void AddConvexHull(ndDemoEntityManager* const scene, const ndMatrix& origin, const ndInt32 segments, ndFloat32 radius, ndFloat32 high, ndFloat32 density, ndFloat32 mass)
//{
//	ndSharedPtr<ndBody> body = AddConvexHull(scene, origin, mass, radius, high, segments);
//	ndShapeMaterial material;
//	material.m_userParam[ndDemoContactCallback::m_density].m_floatData = density;
//	body->GetAsBodyKinematic()->GetCollisionShape().SetMaterial(material);
//}

void ndBasicTrigger (ndDemoEntityManager* const scene)
{
	// build a floor
	ndSharedPtr<ndBody> bodyFloor(BuildFloorBox(scene, ndGetIdentityMatrix(), "blueCheckerboard.png", 0.1f, true));

	// add a trigger and make trasparent
	AddTrigger(scene);

	//class PlaceMatrix : public ndMatrix
	//{
	//	public:
	//	PlaceMatrix(ndFloat32 x, ndFloat32 y, ndFloat32 z)
	//		:ndMatrix(ndGetIdentityMatrix())
	//	{
	//		m_posit.m_x = x;
	//		m_posit.m_y = y;
	//		m_posit.m_z = z;
	//	}
	//};
	//
	//AddBox(scene, PlaceMatrix(0.0f, 20.0f, -3.0f), 0.6f, 10.0f);
	//AddSphere1(scene, PlaceMatrix(0.0f, 5.0f, 0.0f), 0.5f, 10.0f);
	//AddCapsule(scene, PlaceMatrix(0.0f, 5.0f, 3.0f), 0.7f, 10.0f);
	//AddConvexHull(scene, PlaceMatrix(-2.0f, 5.0f, -2.0f), 7, 1.0f, 1.5f, 0.8f, 10.0f);
	//AddConvexHull(scene, PlaceMatrix(-2.0f, 5.0f,  2.0f), 21, 1.0f, 1.5f, 0.7f, 10.0f);
	//AddConvexHull(scene, PlaceMatrix( 2.0f, 5.0f,  3.0f), 210, 1.0f, 1.5f, 0.9f, 10.0f);

	ndQuaternion rot;
	ndVector origin(-40.0f, 5.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}
