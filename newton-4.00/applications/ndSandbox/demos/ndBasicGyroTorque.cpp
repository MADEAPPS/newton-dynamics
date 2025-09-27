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

static ndSharedPtr<ndBody> CreateFlyWheel(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndFloat32 speed, ndFloat32 radius, ndFloat32 lenght, ndFloat32 tiltAnsgle)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	ndRender* const render = *scene->GetRenderer();
	
	ndFloat32 smallRadius = 0.0625f;
	ndShapeInstance rod(new ndShapeCapsule(smallRadius * 0.5f, smallRadius * 0.5f, lenght));
	ndShapeInstance wheel(new ndShapeCylinder(radius, radius, 0.125f));
	
	ndMatrix offset(ndGetIdentityMatrix());
	offset.m_posit.m_x = lenght * 0.5f;
	wheel.SetLocalMatrix(offset);
	//ndShapeInstance flyWheelShape(new ndShapeCompound());
	ndSharedPtr<ndShapeInstance>flyWheelShape(new ndShapeInstance(new ndShapeCompound()));
	ndShapeCompound* const compound = flyWheelShape->GetShape()->GetAsShapeCompound();
	compound->BeginAddRemove();
		compound->AddCollision(&rod);
		compound->AddCollision(&wheel);
	compound->EndAddRemove();
	
	ndMatrix matrix(ndRollMatrix(tiltAnsgle * ndDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_y += 5.0f;
	matrix.m_posit.m_w = 1.0f;

	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = flyWheelShape;
	descriptor.m_mapping = ndRenderPrimitive::m_box;
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("smilli.png")));

	ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));
	ndSharedPtr<ndRenderSceneNode>entity(new ndRenderSceneNode(ndGetIdentityMatrix()));
	entity->SetPrimitive(mesh);
	
	ndSharedPtr<ndBody> body (new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->GetAsBodyDynamic()->SetCollisionShape(**flyWheelShape);
	body->GetAsBodyDynamic()->SetMassMatrix(mass, **flyWheelShape);
	ndVector omega(matrix.m_front.Scale (speed));
	body->SetOmega(omega);
	
	matrix.m_posit -= matrix.m_front.Scale(lenght * 0.5f);
	ndSharedPtr<ndJointBilateralConstraint> joint (new ndJointSpherical(matrix, body->GetAsBodyDynamic(), world->GetSentinelBody()));
	
	world->AddBody(body);
	world->AddJoint(joint);
	scene->AddEntity(entity);
	return body;
}

static ndSharedPtr<ndBody> DzhanibekovEffect(ndDemoEntityManager* const scene, ndFloat32 mass, ndFloat32 angularSpeed, const ndVector& origin)
{
	ndRender* const render = *scene->GetRenderer();

	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();
	matrix.m_posit.m_y += 5.0f;

	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeBox(2.0f, 0.5f, 1.0)));
	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_mapping = ndRenderPrimitive::m_box;
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("smilli.png")));

	ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));
	ndSharedPtr<ndRenderSceneNode>entity(new ndRenderSceneNode(ndGetIdentityMatrix()));
	entity->SetPrimitive(mesh);

	ndVector omega(0.1f, 0.0f, angularSpeed, 0.0f);
	
	ndSharedPtr<ndBody> body(new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity, nullptr, ndVector (ndFloat32 (0.0f))));
	body->SetOmega(omega);
	body->SetMatrix(matrix);
	body->GetAsBodyDynamic()->SetCollisionShape(**shape);
	body->GetAsBodyDynamic()->SetMassMatrix(mass, **shape);

	world->AddBody(body);
	scene->AddEntity(entity);
	return body;
}

static ndSharedPtr<ndBody> Phitop(ndDemoEntityManager* const scene, ndFloat32 mass, ndFloat32 angularSpeed, const ndVector& origin)
{
	ndRender* const render = *scene->GetRenderer();
	ndMatrix matrix(ndPitchMatrix(15.0f * ndDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();
	matrix.m_posit.m_y += 0.5f;

	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeSphere(1.0f)));
	shape->SetScale(ndVector(0.5f, 0.5f, 1.0f, 0.0f));

	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_mapping = ndRenderPrimitive::m_spherical;
	descriptor.m_uvMatrix = ndPitchMatrix(ndPi);
	//descriptor.m_material.m_texture = render->GetTextureCache()->GetTexture(ndGetWorkingFileName("marble.png"));
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("smilli.png")));

	ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));
	ndSharedPtr<ndRenderSceneNode>entity(new ndRenderSceneNode(ndGetIdentityMatrix()));
	entity->SetPrimitive(mesh);

	ndVector omega(0.0f, angularSpeed, 0.0f, 0.0f);
	ndSharedPtr<ndBody> body(new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));

	body->SetOmega(omega);
	body->SetMatrix(matrix);
	body->GetAsBodyDynamic()->SetCollisionShape(**shape);
	body->GetAsBodyDynamic()->SetMassMatrix(mass, **shape);

	world->AddBody(body);
	scene->AddEntity(entity);
	return body;
}

static ndSharedPtr<ndBody> PrecessingTop(ndDemoEntityManager* const scene, const ndVector& origin)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	ndRender* const render = *scene->GetRenderer();

	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeCone(0.7f, 1.0f)));
	shape->SetLocalMatrix(ndRollMatrix(-90.0f * ndDegreeToRad));

	ndMatrix matrix(ndPitchMatrix(15.0f * ndDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_uvMatrix = shape->GetLocalMatrix();
	descriptor.m_mapping = ndRenderPrimitive::m_cylindrical;
	descriptor.m_stretchMaping = false;
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("smilli.png")));

	ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));
	ndSharedPtr<ndRenderSceneNode>entity(new ndRenderSceneNode(ndGetIdentityMatrix()));
	entity->SetPrimitive(mesh);

	matrix.m_posit.m_y += 1.0f;

	const ndFloat32 mass = 1.0f;
	ndSharedPtr<ndBody> body(new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->GetAsBodyDynamic()->SetCollisionShape(**shape);
	body->GetAsBodyDynamic()->SetMassMatrix(mass, **shape);
	body->SetOmega(matrix.m_up.Scale(40.0f));

	world->AddBody(body);
	scene->AddEntity(entity);
	return body;
}

static ndSharedPtr<ndBody> RattleBack(ndDemoEntityManager* const scene, ndFloat32 mass, ndFloat32 speed, const ndVector& origin)
{
	ndRender* const render = *scene->GetRenderer();
	ndMatrix matrix(ndPitchMatrix(10.0f * ndDegreeToRad) * ndYawMatrix(45.0f * ndDegreeToRad));

	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();

	ndMatrix shapeMatrix(ndYawMatrix(5.0f * ndDegreeToRad));

	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeSphere(1.0f)));
	shape->SetScale(ndVector(0.3f, 0.25f, 1.0f, 0.0f));

	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_mapping = ndRenderPrimitive::m_spherical;

	descriptor.m_uvMatrix = ndRollMatrix(-ndPi * 0.5f) * ndPitchMatrix(ndPi * 0.5f);
	//descriptor.m_material.m_texture = render->GetTextureCache()->GetTexture(ndGetWorkingFileName("marble.png"));
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("smilli.png")));

	ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));
	ndSharedPtr<ndRenderSceneNode>entity(new ndRenderSceneNode(ndGetIdentityMatrix()));
	entity->SetPrimitive(mesh);

	matrix = FindFloor(*world, matrix, **shape, 200.0f);

	ndSharedPtr<ndBody> body(new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));

	body->SetMatrix(matrix);

	// skew the inertia matrix for rattle effect
	shape->SetLocalMatrix(shapeMatrix);
	body->GetAsBodyDynamic()->SetCollisionShape(**shape);
	body->GetAsBodyDynamic()->SetMassMatrix(mass, **shape, true);
	body->SetCentreOfMass(ndVector(0.0f, -0.1f, 0.0f, 0.0f));

	ndVector omega(0.0f, speed, 0.0f, 0.0f);
	body->SetOmega(omega);

	world->AddBody(body);
	scene->AddEntity(entity);
	return body;
}

#if 0
static void TippeTop(ndDemoEntityManager* const scene, ndFloat32 mass, ndFloat32 angularSpeed, const ndVector& origin)
{
	class TippeTopShape : public ndShapeUserDefinedImplicit
	{
	public:
		TippeTopShape()
			:ndShapeUserDefinedImplicit()
		{
		}
	};


	//ndMatrix matrix(ndGetIdentityMatrix());
	ndMatrix matrix(ndPitchMatrix(15.0f * ndDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;
	matrix.m_posit.m_y += 1.0f;

	ndMeshLoader loader;
	ndSharedPtr<ndDemoEntity> modelMesh(loader.LoadEntity("tippeTop.fbx", scene));
	//ndSharedPtr<ndDemoMeshInterface> mesh (modelMesh->GetFirstChild()->GetMesh());
	ndSharedPtr<ndDemoMeshInterface> mesh(modelMesh->GetChildren().GetFirst()->GetInfo()->GetMesh());

	ndArray<ndVector> meshPoints;
	((ndDemoMesh*)(*mesh))->GetVertexArray(meshPoints);
	//ndArray<ndBigVector> points;
	//for (ndInt32 i = ndInt32 (meshPoints.GetCount() - 1); i >= 0; --i)
	//{
	//	points.PushBack(meshPoints[i]);
	//}
	//ndConvexHull3d hull(&points[0].m_x, sizeof(ndBigVector), ndInt32 (points.GetCount()), 1.0e-3f);
	//ndShapeInstance shape(new ndShapeSphere(0.64f));
	//ndShapeInstance shape (new TippeTopShape());
	ndShapeInstance shape(new ndShapeConvexHull(ndInt32(meshPoints.GetCount()), sizeof(ndVector), 0.0f, &meshPoints[0].m_x));

	ndVector omega(matrix.m_up.Scale(angularSpeed));
	ndSharedPtr<ndBody> body(new ndBodyDynamic());
	ndSharedPtr<ndDemoEntity>entity(modelMesh->CreateClone());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));

	body->SetOmega(omega);
	body->SetMatrix(matrix);
	body->GetAsBodyDynamic()->SetCollisionShape(shape);
	body->GetAsBodyDynamic()->SetMassMatrix(mass, shape);

	ndVector inertia(body->GetAsBodyDynamic()->GetMassMatrix());
	inertia.m_y *= .5f;
	//inertia.m_x *= 1.75f;
	//inertia.m_z *= 1.75f;
	body->GetAsBodyDynamic()->SetMassMatrix(inertia);

	ndVector com(body->GetCentreOfMass());
	com.m_y -= 0.2f;
	body->SetCentreOfMass(com);

	ndPhysicsWorld* const world = scene->GetWorld();
	world->AddBody(body);
	scene->AddEntity(entity);
}
#endif

void ndBasicAngularMomentum (ndDemoEntityManager* const scene)
{
	// build a floor
	ndSharedPtr<ndBody> bodyFloor(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));
	//ndSharedPtr<ndBody> bodyFloor(BuildFloorBox(scene, ndGetIdentityMatrix(), "blueCheckerboard.png", 0.1f, true));

	// should spins very slowly, with a tilt angle of 30 degrees
	CreateFlyWheel(scene, ndVector(15.0f, 0.0f, -12.0f, 0.0f), 10.0f, 50.0f, 0.6f, 0.5f, 30.0f);
	CreateFlyWheel(scene, ndVector(15.0f, 0.0f, -10.0f, 0.0f), 10.0f, 100.0f, 0.6f, 0.5f, 5.0f);
	CreateFlyWheel(scene, ndVector(15.0f, 0.0f,  -8.0f, 0.0f), 10.0f, -60.0f, 0.6f, 0.5f, 5.0f);
	
	DzhanibekovEffect(scene, 10.0f, 5.0f, ndVector(15.0f, 0.0f, -4.0f, 0.0f));
	DzhanibekovEffect(scene, 10.0f, -5.0f, ndVector(15.0f, 0.0f, 0.0f, 0.0f));
	DzhanibekovEffect(scene, 10.0f, 10.0f, ndVector(15.0f, 0.0f, 4.0f, 0.0f));
	
	Phitop(scene, 10.0f,  25.0f, ndVector(25.0f, 0.0f, -10.0f, 0.0f));
	Phitop(scene, 10.0f, -25.0f, ndVector(25.0f, 0.0f, 0.0f, 0.0f));
	Phitop(scene, 10.0f,  35.0f, ndVector(25.0f, 0.0f, 10.0f, 0.0f));
	
	PrecessingTop(scene, ndVector(5.0f, 0.0f, -4.0f, 0.0f));
	PrecessingTop(scene, ndVector(5.0f, 0.0f, 0.0f, 0.0f));
	PrecessingTop(scene, ndVector(5.0f, 0.0f, 4.0f, 0.0f));
	
	RattleBack(scene, 10.0f, -5.0f, ndVector(0.0f, 0.0f, -4.0f, 0.0f));
	RattleBack(scene, 10.0f, 5.0f, ndVector(0.0f, 0.0f, 0.0f, 0.0f));
	RattleBack(scene, 10.0f, -5.0f, ndVector(0.0f, 0.0f,  4.0f, 0.0f));

	// I can't get this to work.
	//TippeTop(scene, 10.0f, 50.0f, ndVector(0.0f, 0.0f, -4.0f, 0.0f));
	
	ndQuaternion rot;
	ndVector origin(-15.0f, 5.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}
