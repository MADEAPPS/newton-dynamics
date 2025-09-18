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
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"

static void BuildPlaygroundHangingBridge(ndDemoEntityManager* const scene, const ndSharedPtr<ndMesh>& mesh, const ndSharedPtr<ndBody>& playgroundBody)
{
	//ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)playgroundBody->GetNotifyCallback();
	//ndSharedPtr<ndRenderSceneNode> entity (notify->GetUserData());

	// add a hanging bridge as a feature.
	const ndMesh* const end(mesh->FindChild("rampEnd"));
	const ndMesh* const start(mesh->FindChild("rampStart"));
	
	ndMatrix endMatrix(end->CalculateGlobalMatrix());
	ndMatrix startMatrix(start->CalculateGlobalMatrix());
	ndFloat32 dist = ndAbs(startMatrix.m_right.DotProduct(endMatrix.m_posit - startMatrix.m_posit).GetScalar());

	ndInt32 numberOfPlank = 1;
	while (dist > 2.0f)
	{
		numberOfPlank *= 2;
		dist *= ndFloat32(0.5f);
	}

	ndFloat32 plankSickness = 0.2f;
	ndFloat32 slackDist = dist * 1.01f;
	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeBox(11.0f, plankSickness, slackDist)));

	ndRender* const render = *scene->GetRenderer();
	ndRenderPrimitiveMesh::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_mapping = ndRenderPrimitiveMesh::m_box;
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("wood_1.png")));

	ndSharedPtr<ndRenderSceneNode> bridgeMesh(new ndRenderSceneNodeInstance(startMatrix, descriptor));
	scene->AddEntity(bridgeMesh);

	ndMatrix localMatrix(ndGetIdentityMatrix());
	localMatrix.m_posit.m_z = dist * ndFloat32(0.5f);
	localMatrix.m_posit.m_y = -ndFloat32(0.5f) * plankSickness;

	ndPhysicsWorld* const world = scene->GetWorld();

	ndFloat32 linkMass = 100.0f;
	ndFixSizeArray<ndBodyDynamic*, 256> bodyLinks;

	// create all the links and attach the to the parent scene node
	for (ndInt32 i = 0; i < numberOfPlank; ++i)
	{
		ndSharedPtr<ndRenderSceneNode>visualLink(new ndRenderSceneNode(localMatrix));
		bridgeMesh->AddChild(visualLink);

		ndSharedPtr<ndBody> body(new ndBodyDynamic());
		body->SetNotifyCallback(new ndDemoEntityNotify(scene, visualLink));
		body->SetMatrix(visualLink->CalculateGlobalMatrix());
		body->GetAsBodyKinematic()->SetCollisionShape(**shape);
		body->GetAsBodyKinematic()->SetMassMatrix(linkMass, **shape);
		body->GetAsBodyDynamic()->SetAngularDamping(ndVector(ndFloat32(0.5f)));
		world->AddBody(body);
		bodyLinks.PushBack(body->GetAsBodyDynamic());

		localMatrix.m_posit.m_z += dist;
	}
	ndRenderSceneNodeInstance* const instanceRoot = (ndRenderSceneNodeInstance*)*bridgeMesh;
	instanceRoot->Finalize();

	//connect every two adjecent link with a hinge joint
	ndMatrix matrix0(ndGetIdentityMatrix());
	ndMatrix matrix1(ndGetIdentityMatrix());

	matrix0.m_posit.m_z = slackDist * 0.5f;
	matrix0.m_posit.m_y = ndFloat32(0.5f) * plankSickness;

	matrix1.m_posit.m_z = -slackDist * 0.5f;
	matrix1.m_posit.m_y = ndFloat32(0.5f) * plankSickness;

	for (ndInt32 i = 0; i < bodyLinks.GetCount() - 1; ++i)
	{
		ndBodyDynamic* const body0 = bodyLinks[i];
		ndBodyDynamic* const body1 = bodyLinks[i + 1];
		ndSharedPtr<ndJointBilateralConstraint> joint (new ndJointHinge(matrix0 * body0->GetMatrix(), matrix1 * body1->GetMatrix(), body0, body1));
		ndJointHinge* const hinge = (ndJointHinge*)*joint;
		hinge->SetAsSpringDamper(0.02f, 0.0f, 20.0f);
		world->AddJoint(joint);
	}

	// connect the two ends
	{
		ndBodyDynamic* const body0 = bodyLinks[0];
		ndBodyDynamic* const body1 = playgroundBody->GetAsBodyDynamic();
		ndMatrix body0Matrix(matrix1 * body0->GetMatrix());
		ndMatrix body1Matrix(body0Matrix);
		body1Matrix.m_posit += body1Matrix.m_right.Scale(ndFloat32(0.5f) * (slackDist - dist));

		ndSharedPtr<ndJointBilateralConstraint> joint(new ndJointHinge(body0Matrix, body1Matrix, body0, body1));
		ndJointHinge* const hinge = (ndJointHinge*)*joint;
		hinge->SetAsSpringDamper(0.02f, 0.0f, 20.0f);
		world->AddJoint(joint);
	}

	{
		ndBodyDynamic* const body0 = bodyLinks[bodyLinks.GetCount() - 1];
		ndBodyDynamic* const body1 = playgroundBody->GetAsBodyDynamic();
		ndMatrix body0Matrix(matrix0 * body0->GetMatrix());
		ndMatrix body1Matrix(body0Matrix);
		body1Matrix.m_posit -= body1Matrix.m_right.Scale(ndFloat32(0.5f) * (slackDist - dist));

		ndSharedPtr<ndJointBilateralConstraint> joint(new ndJointHinge(body0Matrix, body1Matrix, body0, body1));
		ndJointHinge* const hinge = (ndJointHinge*)*joint;
		hinge->SetAsSpringDamper(0.02f, 0.0f, 20.0f);
		world->AddJoint(joint);
	}
}

static void BuildPlayground(ndDemoEntityManager* const scene)
{
	// build the background collsion mesh
	ndMeshLoader loader;
	ndSharedPtr<ndRenderSceneNode> playground(loader.LoadEntity(*scene->GetRenderer(), ndGetWorkingFileName("playground.fbx")));
	const ndMesh* const levelMesh = loader.m_mesh->FindChild("levelGeometry");
	ndAssert(levelMesh);
	ndSharedPtr<ndShapeInstance>collision(loader.m_mesh->CreateCollisionTree());

	ndMatrix location(loader.m_mesh->CalculateGlobalMatrix());
	// generate a rigibody and added to the scene and world
	ndPhysicsWorld* const world = scene->GetWorld();
	ndSharedPtr<ndBody> body(new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, playground));
	body->SetMatrix(location);
	body->GetAsBodyDynamic()->SetCollisionShape(**collision);
	
	world->AddBody(body);
	scene->AddEntity(playground);

	BuildPlaygroundHangingBridge(scene, loader.m_mesh, body);
}

void ndBasicStaticMeshCollision (ndDemoEntityManager* const scene)
{
	BuildPlayground(scene);

	//ndMatrix location(ndGetIdentityMatrix());
	//location.m_posit.m_y += 2.0f;
	//ndMatrix localAxis(ndGetIdentityMatrix());
	//localAxis[0] = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
	//localAxis[1] = ndVector(1.0f, 0.0f, 0.0f, 0.0f);
	//localAxis[2] = localAxis[0].CrossProduct(localAxis[1]);
	//
	//ndMeshLoader loader;
	//ndSharedPtr<ndDemoEntity> man(loader.LoadEntity("walker.fbx", scene));
	//
	////ndFloat32 height = 1.9f;
	////ndFloat32 radio = 0.5f;
	////ndFloat32 mass = 100.0f;
	////new ndBasicPlayerCapsule(scene, man, localAxis, location, mass, radio, height, height/4.0f, true);
	//
	//location.m_posit.m_x += 8.0f;
	//location.m_posit.m_z -= 2.0f;
	////new ndBasicPlayerCapsule(scene, man, localAxis, location, mass, radio, height, height / 4.0f);
	//
	//location.m_posit.m_z += 4.0f;
	////new ndBasicPlayerCapsule(scene, man, localAxis, location, mass, radio, height, height / 4.0f);
	
	class PlaceMatrix : public ndMatrix
	{
		public:
		PlaceMatrix(ndFloat32 x, ndFloat32 y, ndFloat32 z)
			:ndMatrix(ndGetIdentityMatrix())
		{
			m_posit.m_x = x;
			m_posit.m_y = y;
			m_posit.m_z = z;
		}
	};
	
	AddBox(scene, PlaceMatrix(10.0f, 1.0f, 0.0f), 30.0f, 2.0f, 0.25f, 2.5f);
	AddBox(scene, PlaceMatrix(12.0f, 1.5f, 1.125f), 30.0f, 2.0f, 0.25f, 2.5f);
	AddBox(scene, PlaceMatrix(13.0f, 1.0f, 0.0f), 30.0f, 1.0f, 0.25f, 1.0f);
	AddConvexHull(scene, PlaceMatrix(11.0f, 1.0f, 2.0f), 40.0f, 0.6f, 1.0f, 15);
	AddConvexHull(scene, PlaceMatrix(11.0f, 1.0f, 0.0f), 40.0f, 0.7f, 1.0f, 10);
	AddConvexHull(scene, PlaceMatrix(12.0f, 1.0f, 0.0f), 40.0f, 0.5f, 1.2f, 6);
	AddConvexHull(scene, PlaceMatrix(13.0f, 1.0f, 2.0f), 40.0f, 0.6f, 1.0f, 15);
	AddConvexHull(scene, PlaceMatrix(13.0f, 1.0f, 0.0f), 40.0f, 0.7f, 1.0f, 10);
	AddConvexHull(scene, PlaceMatrix(12.0f, 1.0f, 1.0f), 40.0f, 0.5f, 1.2f, 6);
	//AddCapsulesStacks(scene, PlaceMatrix(45.0f, 0.0f, 0.0f), 10.0f, 0.5f, 0.5f, 1.0f, 5, 8, 7);
	
	ndQuaternion rot(ndYawMatrix(30.0f * ndDegreeToRad));
	ndVector origin(-40.0f, 15.0f, 20.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}

