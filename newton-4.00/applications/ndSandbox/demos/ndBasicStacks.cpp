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
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"

static ndSharedPtr<ndBody> AddRigidBody(ndDemoEntityManager* const scene,
	const ndMatrix& matrix, const ndShapeInstance& shape, 
	ndRenderSceneNodeInstance* const rootEntity, ndFloat32 mass)
{
	ndWorld* const world = scene->GetWorld();

	const ndMatrix bindMatrix(rootEntity->GetTransform().GetMatrix());
	const ndMatrix localMatrix(matrix* bindMatrix);
	ndSharedPtr<ndRenderSceneNode>sceneNode(new ndRenderSceneNode(localMatrix));
	rootEntity->AddChild(sceneNode);

	ndSharedPtr<ndBody> body(new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, sceneNode));
	body->SetMatrix(matrix);
	body->GetAsBodyKinematic()->SetCollisionShape(shape);
	body->GetAsBodyKinematic()->SetMassMatrix(mass, shape);
	body->GetAsBodyDynamic()->SetAngularDamping(ndVector(ndFloat32(0.5f)));
	world->AddBody(body);
	return body;
}

static void BuildPyramid(ndDemoEntityManager* const scene,
	ndRenderSceneNodeInstance* const rootEntity, const ndShapeInstance& shape,
	ndFloat32 mass, const ndVector& origin, const ndVector& boxSize, ndInt32 count)
{
	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	// create the shape and visual mesh as a common data to be re used
	ndFloat32 stepz = boxSize.m_z + 1.0e-2f;
	ndFloat32 stepy = boxSize.m_y + 1.0e-2f;
	stepy = boxSize.m_y;

	ndFloat32 y0 = matrix.m_posit.m_y;
	ndFloat32 z0 = matrix.m_posit.m_z - stepz * (ndFloat32)count / 2;

	matrix.m_posit.m_y = y0;
	matrix.m_posit.m_y -= 0.01f;

	ndWorld* const world = scene->GetWorld();
	for (ndInt32 j = 0; j < count; ++j)
	{
		matrix.m_posit.m_z = z0;
		matrix = FindFloor(*world, matrix, shape, 100.0f);
		for (ndInt32 i = 0; i < (count - j); ++i)
		{
			AddRigidBody(scene, matrix, shape, rootEntity, mass);
			matrix.m_posit.m_z += stepz;
		}
		z0 += stepz * 0.5f;
	}
}

void BuildPyramidStacks(ndDemoEntityManager* const scene, ndFloat32 mass, const ndVector& origin, const ndVector& boxSize, ndInt32 stackHigh)
{
	ndVector origin1(origin);
	ndVector size(boxSize.Scale(1.0f));

	ndRender* const render = *scene->GetRenderer();

	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeBox(size.m_x, size.m_y, size.m_z)));
	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_mapping = ndRenderPrimitive::m_box;
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("wood_0.png")));
	ndSharedPtr<ndRenderSceneNode>root(new ndRenderSceneNodeInstance(ndGetIdentityMatrix(), descriptor));
	scene->AddEntity(root);
	
	origin1.m_z = 0.0f;
	origin1.m_x += 3.0f;
	ndRenderSceneNodeInstance* const instanceRoot = root->GetAsInstance();
	BuildPyramid(scene, instanceRoot, **shape, mass, origin1, boxSize, stackHigh);
	
	instanceRoot->Finalize();
}

static void BuildSphereColumn(ndDemoEntityManager* const scene, ndFloat32 mass, const ndVector& origin, const ndVector& size, ndInt32 count)
{
	// build a standard block stack of 20 * 3 boxes for a total of 60
	ndWorld* const world = scene->GetWorld();
	ndRender* const render = *scene->GetRenderer();

	ndVector blockBoxSize(size);
	// create the stack
	ndMatrix baseMatrix(ndGetIdentityMatrix());
	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeSphere(blockBoxSize.m_x)));

	// add a instance root scene node
	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_mapping = ndRenderPrimitive::m_spherical;
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("earthmap.png")));
	ndSharedPtr<ndRenderSceneNode>root(new ndRenderSceneNodeInstance(ndGetIdentityMatrix(), descriptor));
	scene->AddEntity(root);

	baseMatrix.m_posit.m_x = origin.m_x;
	baseMatrix.m_posit.m_z = origin.m_z;

	ndRenderSceneNodeInstance* const instanceRoot = root->GetAsInstance();
	for (ndInt32 i = 0; i < count; ++i)
	{
		baseMatrix = FindFloor(*world, baseMatrix, **shape, 100.0f);
		AddRigidBody(scene, baseMatrix, **shape, instanceRoot, mass);
	}
	instanceRoot->Finalize();
}

static void BuildBoxColumn(ndDemoEntityManager* const scene, ndFloat32 mass, const ndVector& origin, const ndVector& size, ndInt32 count)
{
	// build a standard block stack of 20 * 3 boxes for a total of 60
	ndWorld* const world = scene->GetWorld();
	ndRender* const render = *scene->GetRenderer();

	ndVector blockBoxSize(size);
	blockBoxSize = blockBoxSize.Scale(2.0f);

	// create the stack
	ndMatrix baseMatrix(ndGetIdentityMatrix());
	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeBox(blockBoxSize.m_x, blockBoxSize.m_y, blockBoxSize.m_z)));

	// add a instance root scene node
	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_mapping = ndRenderPrimitive::m_box;
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("wood_0.png")));
	ndSharedPtr<ndRenderSceneNode>root(new ndRenderSceneNodeInstance(ndGetIdentityMatrix(), descriptor));
	scene->AddEntity(root);

	// for the elevation of the floor at the stack position
	baseMatrix.m_posit.m_x = origin.m_x;
	baseMatrix.m_posit.m_z = origin.m_z;

	baseMatrix.m_posit.m_y -= ndFloat32 (0.01f);
	ndMatrix rotation(ndYawMatrix(20.0f * ndDegreeToRad));

	ndRenderSceneNodeInstance* const instanceRoot = root->GetAsInstance();
	for (ndInt32 i = 0; i < count; ++i)
	{
		baseMatrix = FindFloor(*world, baseMatrix, **shape, 100.0f);
		AddRigidBody(scene, baseMatrix, **shape, instanceRoot, mass);
		baseMatrix.m_posit += baseMatrix.m_up.Scale(blockBoxSize.m_x);
		baseMatrix = rotation * baseMatrix;
	}
	instanceRoot->Finalize();
}

static void BuildCylinderColumn(ndDemoEntityManager* const scene, ndFloat32 mass, const ndVector& origin, const ndVector& size, ndInt32 count)
{
	// build a standard block stack of 20 * 3 boxes for a total of 60
	ndWorld* const world = scene->GetWorld();
	ndRender* const render = *scene->GetRenderer();

	ndVector blockBoxSize(size);

	// create the stack
	ndMatrix baseMatrix(ndGetIdentityMatrix());

	// for the elevation of the floor at the stack position
	baseMatrix.m_posit.m_x = origin.m_x;
	baseMatrix.m_posit.m_z = origin.m_z;

	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeCylinder(blockBoxSize.m_x, blockBoxSize.m_y, blockBoxSize.m_z)));
	shape->SetLocalMatrix(ndRollMatrix(ndPi * 0.5f));

	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_mapping = ndRenderPrimitive::m_cylindrical;
	descriptor.m_uvMatrix = shape->GetLocalMatrix();
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("smilli.png")));
	ndSharedPtr<ndRenderSceneNode>root(new ndRenderSceneNodeInstance(ndGetIdentityMatrix(), descriptor));
	scene->AddEntity(root);

	ndMatrix rotation(ndYawMatrix(20.0f * ndDegreeToRad));

	ndRenderSceneNodeInstance* const instanceRoot = root->GetAsInstance();
	for (ndInt32 i = 0; i < count; ++i)
	{
		baseMatrix = FindFloor(*world, baseMatrix, **shape, 100.0f);
		AddRigidBody(scene, baseMatrix, **shape, instanceRoot, mass);
		baseMatrix.m_posit += baseMatrix.m_up.Scale(blockBoxSize.m_z);
		baseMatrix = rotation * baseMatrix;
	}
	instanceRoot->Finalize();
}

static void BuildCapsuleStack(ndDemoEntityManager* const scene, ndFloat32 mass, const ndVector& origin, const ndVector& size, ndInt32 stackHigh)
{
	// build a standard block stack of 20 * 3 boxes for a total of 60
	ndWorld* const world = scene->GetWorld();
	ndRender* const render = *scene->GetRenderer();

	ndVector blockBoxSize(size);

	//ndShapeInstance shape(new ndShapeCapsule(blockBoxSize.m_x, blockBoxSize.m_x, blockBoxSize.m_z));
	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeCapsule(blockBoxSize.m_x, blockBoxSize.m_x, blockBoxSize.m_z)));
	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_uvMatrix = ndPitchMatrix(ndPi);
	descriptor.m_mapping = ndRenderPrimitive::m_capsule;
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("smilli.png")));
	ndSharedPtr<ndRenderSceneNode>root(new ndRenderSceneNodeInstance(ndGetIdentityMatrix(), descriptor));
	scene->AddEntity(root);

	ndFloat32 vertialStep = blockBoxSize.m_x * 2.0f;
	ndFloat32 horizontalStep = blockBoxSize.m_z * 0.8f;

	ndMatrix matrix0(ndGetIdentityMatrix());
	matrix0.m_posit = origin;
	matrix0.m_posit.m_y += blockBoxSize.m_x;
	matrix0.m_posit.m_w = 1.0f;
	matrix0 = FindFloor(*world, matrix0, **shape, 100.0f);

	ndMatrix matrix1(matrix0);
	matrix1.m_posit.m_z += horizontalStep;

	ndMatrix matrix2(ndYawMatrix(ndPi * 0.5f) * matrix0);
	matrix2.m_posit.m_x += horizontalStep * 0.5f;
	matrix2.m_posit.m_z += horizontalStep * 0.5f;
	matrix2.m_posit.m_y += vertialStep;

	ndMatrix matrix3(matrix2);
	matrix3.m_posit.m_x -= horizontalStep;

	ndRenderSceneNodeInstance* const instanceRoot = root->GetAsInstance();
	for (ndInt32 i = 0; i < stackHigh / 2; ++i)
	{
		AddRigidBody(scene, matrix0, **shape, instanceRoot, mass);
		AddRigidBody(scene, matrix1, **shape, instanceRoot, mass);
		AddRigidBody(scene, matrix2, **shape, instanceRoot, mass);
		AddRigidBody(scene, matrix3, **shape, instanceRoot, mass);

		matrix0.m_posit.m_y += vertialStep * 2.0f;
		matrix1.m_posit.m_y += vertialStep * 2.0f;
		matrix2.m_posit.m_y += vertialStep * 2.0f;
		matrix3.m_posit.m_y += vertialStep * 2.0f;
	}
	instanceRoot->Finalize();
}

void ndBasicStacks (ndDemoEntityManager* const scene)
{
	// build a floor
	ndSharedPtr<ndBody> body (BuildFlatPlane(scene, ndGetIdentityMatrix(), "blueCheckerboard.png", true));
	ndVector origin(ndVector::m_zero);

	//ndInt32 pyramidHigh = 1;
	//ndInt32 pyramidHigh = 10;
	//ndInt32 pyramidHigh = 20;
	ndInt32 pyramidHigh = 30;
	//ndInt32 pyramidHigh = 50;
	//for (ndInt32 i = 0; i < 4; ++i)
	for (ndInt32 i = 0; i < 4; ++i)
	{
		BuildPyramidStacks(scene, 1.0f, origin, ndVector(0.5f, 0.25f, 0.8f, 0.0f), pyramidHigh);
		origin.m_x += 4.0f;
	}

	origin = ndVector::m_zero;
	origin.m_x -= 2.0f;
	origin.m_z -= 3.0f;
	BuildSphereColumn(scene, 10.0f, origin, ndVector(0.5f, 0.5f, 0.5f, 0.0f), 20);

	origin.m_z += 6.0f;
	BuildBoxColumn(scene, 10.0f, origin, ndVector(0.5f, 0.5f, 0.5f, 0.0f), 20);

	origin.m_z += 10.0f;
	BuildCylinderColumn(scene, 10.0f, origin, ndVector(0.75f, 0.6f, 1.0f, 0.0f), 20);

	origin.m_x -= 6.0f;
	origin.m_z -= 10.0f;
	BuildCapsuleStack(scene, 10.0f, origin, ndVector(0.25f, 0.25f, 2.0f, 0.0f), 20);

	origin = ndVector::m_wOne;
	origin.m_x -= 3.0f;
	origin.m_y += 5.0f;

	origin.m_x -= 15.0f;
	origin.m_z -= 15.0f;

	ndQuaternion rot(ndYawMatrix(-30.0f * ndDegreeToRad));
	scene->SetCameraMatrix(rot, origin);
}
