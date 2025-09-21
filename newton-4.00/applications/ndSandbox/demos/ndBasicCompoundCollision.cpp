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
#include "ndMeshLoader.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"

static ndSharedPtr<ndBody> AddRigidBody(
	ndDemoEntityManager* const scene,
	const ndMatrix& matrix, const ndShapeInstance& shape,
	const ndSharedPtr<ndRenderSceneNode>& entity, ndFloat32 mass)
{
	ndPhysicsWorld* const world = scene->GetWorld();

	ndSharedPtr<ndBody> body(new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->GetAsBodyDynamic()->SetCollisionShape(shape);
	body->GetAsBodyDynamic()->SetMassMatrix(mass, shape);

	world->AddBody(body);
	scene->AddEntity(entity);
	return body;
}

static void AddSphere(ndDemoEntityManager* const scene)
{
	ndRender* const render = *scene->GetRenderer();

	ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeSphere(0.125f)));
	ndRenderPrimitiveMesh::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_mapping = ndRenderPrimitiveMesh::m_spherical;
	descriptor.AddMaterial (render->GetTextureCache()->GetTexture(ndGetWorkingFileName("earthmap.png")));
	ndSharedPtr<ndRenderPrimitive> mesh(ndRenderPrimitiveMesh::CreateMeshPrimitive(descriptor));

	ndSharedPtr<ndRenderSceneNode>origEntity(new ndRenderSceneNode(ndGetIdentityMatrix()));
	origEntity->SetPrimitive(mesh);

	ndMatrix originMatrix = ndGetIdentityMatrix();
	originMatrix.m_posit.m_x = 2.0f;
	for (ndInt32 i = 0; i < 4; ++i)
	{
		ndSharedPtr<ndRenderSceneNode> entity(origEntity->Clone());
		ndVector floor(FindFloor(*scene->GetWorld(), originMatrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		originMatrix.m_posit.m_y = floor.m_y + 1.0f;

		// make sure the visual is initialized properlly
		entity->SetTransform(originMatrix, originMatrix.m_posit);
		entity->SetTransform(originMatrix, originMatrix.m_posit);
		AddRigidBody(scene, originMatrix, **shape, entity, 1.0);
	}
}

static void CreateBoxCompoundShape(ndShapeInstance& parentInstance)
{
	ndShapeInstance wall1(new ndShapeBox(2.0f, 2.0f, 0.1f));
	ndShapeInstance wall2(new ndShapeBox(0.1f, 2.0f, 2.0f));
	ndShapeInstance wall3(new ndShapeBox(2.0f, 2.0f, 0.1f));
	ndShapeInstance wall4(new ndShapeBox(0.1f, 2.0f, 2.0f));
	ndShapeInstance floor(new ndShapeBox(2.0f, 0.1f, 2.0f));

	ndMatrix wall1_local = ndGetIdentityMatrix();
	wall1_local.m_posit = ndVector(0.0f, 0.0f, -1.0f, 1.0f);

	ndMatrix wall2_local = ndGetIdentityMatrix();
	wall2_local.m_posit = ndVector(1.0f, 0.0f, 0.0f, 1.0f);

	ndMatrix wall3_local = ndGetIdentityMatrix();
	wall3_local.m_posit = ndVector(0.0f, 0.0f, 1.0f, 1.0f);

	ndMatrix wall4_local = ndGetIdentityMatrix();
	wall4_local.m_posit = ndVector(-1.0f, 0.0f, 0.0f, 1.0f);

	ndMatrix floor_local = ndGetIdentityMatrix();
	floor_local.m_posit = ndVector(0.0f, -1.0f, 0.0f, 1.0f);

	ndShapeCompound* const compoundShape = parentInstance.GetShape()->GetAsShapeCompound();
	compoundShape->BeginAddRemove();
		auto AddToCompoundShape = [](const ndMatrix& localMatrix, ndShapeInstance& parentShape, ndShapeInstance& childInstance)
		{
			ndShapeCompound* const compoundShape = parentShape.GetShape()->GetAsShapeCompound();
			childInstance.SetLocalMatrix(localMatrix);
			compoundShape->AddCollision(&childInstance);
		};
		AddToCompoundShape(wall1_local, parentInstance, wall1);
		AddToCompoundShape(wall2_local, parentInstance, wall2);
		AddToCompoundShape(wall3_local, parentInstance, wall3);
		AddToCompoundShape(wall4_local, parentInstance, wall4);
		AddToCompoundShape(floor_local, parentInstance, floor);
	compoundShape->EndAddRemove();
}

static void AddEmptyBox(ndDemoEntityManager* const scene)
{
	ndRender* const render = *scene->GetRenderer();
	ndSharedPtr<ndShapeInstance>compoundShapeInstance(new ndShapeInstance(new ndShapeCompound()));
	CreateBoxCompoundShape(**compoundShapeInstance);

	ndRenderPrimitiveMesh::ndDescriptor descriptor(render);
	descriptor.m_collision = compoundShapeInstance;
	descriptor.m_mapping = ndRenderPrimitiveMesh::m_box;
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("wood_0.png")));
	ndSharedPtr<ndRenderPrimitive> mesh(ndRenderPrimitiveMesh::CreateMeshPrimitive(descriptor));

	ndSharedPtr<ndRenderSceneNode>entity(new ndRenderSceneNode(ndGetIdentityMatrix()));
	entity->SetPrimitive(mesh);

	ndMatrix mBodyMatrix = ndGetIdentityMatrix();
	mBodyMatrix.m_posit = ndVector(-2.0f, 5.0f, -5.0f, 1.0f);
	ndVector floor(FindFloor(*scene->GetWorld(), mBodyMatrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	mBodyMatrix.m_posit.m_y = floor.m_y + 1.5f;

	AddRigidBody(scene, mBodyMatrix, **compoundShapeInstance, entity, 10.0f);
}

static void AddSimpleConcaveMesh(ndDemoEntityManager* const scene, const ndMatrix& matrix, const char* const meshName, int count = 1)
{
	ndMeshLoader loader;
	ndSharedPtr<ndRenderSceneNode> rootEntity(loader.LoadEntity(*scene->GetRenderer(), ndGetWorkingFileName(meshName)));
	ndSharedPtr<ndShapeInstance>compoundShapeInstance(loader.m_mesh->CreateCollision());
	
	ndMatrix originMatrix(matrix);
	for (ndInt32 i = 0; i < count; ++i)
	{
		ndSharedPtr<ndRenderSceneNode> cloneEntity(rootEntity->Clone());
		originMatrix.m_posit.m_z += 2.0f;
		ndVector floor(FindFloor(*scene->GetWorld(), originMatrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		originMatrix.m_posit.m_y = floor.m_y + 2.0f;
		AddRigidBody(scene, originMatrix, *(*compoundShapeInstance), cloneEntity, 5.0f);
	}
}

void ndBasicCompoundCollision(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> ground(BuildFlatPlane(scene, ndGetIdentityMatrix(), "grass.png", true));

	ndMatrix location(ndGetIdentityMatrix());

	AddSphere(scene);
	AddEmptyBox(scene);

	location.m_posit.m_y = 0.5f;
	location.m_posit.m_z = -3.0f;
	AddSimpleConcaveMesh(scene, location, "bowl.fbx", 1);

	location.m_posit.m_z = -5.0f;
	AddSimpleConcaveMesh(scene, location, "testConcave.fbx", 1);
	
	location.m_posit.m_x = 5.0f;
	location.m_posit.m_z = -2.0f;
	location.m_posit.m_y = 1.7f;
	AddSimpleConcaveMesh(scene, location, "camel.fbx", 4);
	
	location.m_posit.m_x = 10.0f;
	location.m_posit.m_z = 5.0f;
	location.m_posit.m_y = 2.0f;
	AddSimpleConcaveMesh(scene, location, "dino.fbx", 4);

	ndVector origin(ndVector::m_zero);
	origin.m_x -= 10.0f;
	origin.m_y += 2.0f;
	origin.m_z += 10.0f;
	origin.m_w = 1.0f;
	ndQuaternion rot(ndYawMatrix(45.0f * ndDegreeToRad));
	scene->SetCameraMatrix(rot, origin);
}
