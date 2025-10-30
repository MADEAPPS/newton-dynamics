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
#include "ndDemoCameraNodeFollow.h"
#include "ndHeightFieldPrimitive.h"


class ndExcavatorController : public ndModelNotify
{
	public:
	ndExcavatorController(
		ndDemoEntityManager* const scene, 
		const ndMatrix& location, ndSharedPtr<ndMesh>& mesh, 
		ndSharedPtr<ndModel>& model, ndSharedPtr<ndRenderSceneNode>& visualMesh)
		:ndModelNotify()
		,m_scene(scene)
	{
		ndModelArticulation* const articulation = model->GetAsModelArticulation();
		ndAssert(articulation);

		MakeChassis(articulation, location, mesh, visualMesh);
		MakeCabinAndUpperBody(articulation, location, mesh, visualMesh);
	}

	static ndSharedPtr<ndModelNotify> CreateExcavator(ndDemoEntityManager* const scene, const ndMatrix& location, ndRenderMeshLoader& mesh)
	{
		ndSharedPtr<ndRenderSceneNode> vehicleMesh(mesh.m_renderMesh->Clone());
		ndMatrix matrix(location);
		matrix.m_posit = FindFloor(*scene->GetWorld(), matrix.m_posit, 200.0f);

		// using a model articulation for this vehicle
		ndSharedPtr<ndModel> vehicleModel(new ndModelArticulation());
		ndSharedPtr<ndModelNotify> controller(new ndExcavatorController(scene, matrix, mesh.m_mesh, vehicleModel, vehicleMesh));

		ndWorld* const world = scene->GetWorld();
		scene->AddEntity(vehicleMesh);
		world->AddModel(vehicleModel);
		vehicleModel->AddBodiesAndJointsToWorld();

		return controller;
	}

	private:
	void MakeChassis(
		ndModelArticulation* const articulation, 
		const ndMatrix& location, 
		ndSharedPtr<ndMesh>& mesh, 
		ndSharedPtr<ndRenderSceneNode>& visualMeshRoot)
	{
		ndMesh* const chassisMesh = mesh->FindByName("base");
		ndAssert(chassisMesh);

		// create the collision and the world matrix
		ndMatrix matrix(chassisMesh->CalculateGlobalMatrix() * location);
		ndSharedPtr<ndShapeInstance> collision (chassisMesh->CreateCollisionFromChildren());

		// find the visual mesh for thsi body
		ndRenderSceneNode* const visualMeshNode = visualMeshRoot->FindByName(chassisMesh->GetName());
		ndSharedPtr<ndRenderSceneNode> visualMesh((visualMeshNode == *visualMeshRoot) ? visualMeshRoot : visualMeshRoot->GetSharedPtr());

		ndFloat32 chassisMass = ndFloat32(4000.0f);

		// create the rigid that represent the chassis
		ndSharedPtr<ndBody> chassisBody(new ndBodyDynamic());
		chassisBody->SetNotifyCallback(new ndDemoEntityNotify(m_scene, visualMesh));
		chassisBody->SetMatrix(matrix);
		chassisBody->GetAsBodyDynamic()->SetCollisionShape(**collision);
		chassisBody->GetAsBodyDynamic()->SetMassMatrix(chassisMass, **collision);
		
		articulation->AddRootBody(chassisBody);
	}

	ndSharedPtr<ndBody> MakeBodyPart(
		const ndMatrix& location,
		ndSharedPtr<ndMesh>& mesh, ndSharedPtr<ndRenderSceneNode>& visualMeshRoot,
		const char* const name, ndFloat32 mass)
	{
		ndMesh* const meshNode = mesh->FindByName(name);
		ndAssert(meshNode);

		// calculate matrix
		ndMatrix matrix(meshNode->CalculateGlobalMatrix() * location);

		// build collision mesh
		ndSharedPtr<ndShapeInstance> collision(meshNode->CreateCollisionFromChildren());

		// find the visual mesh
		ndRenderSceneNode* const visualMeshNode = visualMeshRoot->FindByName(meshNode->GetName());
		ndSharedPtr<ndRenderSceneNode> visualMesh(visualMeshNode->GetSharedPtr());

		ndSharedPtr<ndBody> body(new ndBodyDynamic());
		body->SetNotifyCallback(new ndDemoEntityNotify(m_scene, visualMesh));
		body->SetMatrix(matrix);
		body->GetAsBodyDynamic()->SetCollisionShape(**collision);
		body->GetAsBodyDynamic()->SetMassMatrix(mass, **collision);
		return body;
	}

	void MakeCabinAndUpperBody(
		ndModelArticulation* const articulation,
		const ndMatrix& location,
		ndSharedPtr<ndMesh>& mesh, ndSharedPtr<ndRenderSceneNode>& visualMeshRoot)
	{
		ndSharedPtr<ndBody> cabinBody (MakeBodyPart(location, mesh, visualMeshRoot, "EngineBody", 400.0f));

		ndMatrix hingeFrame(cabinBody->GetMatrix());
		ndModelArticulation::ndNode* const rootNode = articulation->GetRoot();

		ndSharedPtr<ndJointBilateralConstraint> cabinPivot(new ndJointHinge(hingeFrame, cabinBody->GetAsBodyDynamic(), rootNode->m_body->GetAsBodyDynamic()));
		
		//// set the center of mass of engine
		//dVector com(hingeFrame.UnrotateVector(dVector(-2.0f, 0.0f, 0.0f, 0.0f)));
		//NewtonBodySetCentreOfMass(cabinBody, &com[0]);
		ndModelArticulation::ndNode* const cabinNode = articulation->AddLimb(rootNode, cabinBody, cabinPivot);
	}

	ndDemoEntityManager* m_scene;
};


void ndComplexModel(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "blueCheckerboard.png", 0.1f, true));
	//ndSharedPtr<ndBody> mapBody(BuildHeightFieldTerrain(scene, "grass.png", ndGetIdentityMatrix()));

	ndMatrix matrix(ndGetIdentityMatrix());

	//// add a help menu
	//ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndBackGroundVehicleController::ndHelpLegend());
	//scene->SetDemoHelp(demoHelper);

	ndRenderMeshLoader excavatorLoaded(*scene->GetRenderer());
	excavatorLoaded.LoadMesh(ndGetWorkingFileName("excavator.nd"));
	//ndSharedPtr<ndModelNotify> controller(ndBackGroundVehicleController::CreateAiVehicleProp(scene, ndVector::m_wOne, vmwLoaderRedPaint));

	ndSharedPtr<ndModelNotify> controller(ndExcavatorController::CreateExcavator(scene, matrix, excavatorLoaded));

	//// set this player as the active camera
	//ndBackGroundVehicleController* const playerController = (ndBackGroundVehicleController*)*controller;
	//ndRender* const renderer = *scene->GetRenderer();
	//renderer->SetCamera(playerController->GetCamera());

	matrix.m_posit.m_x -= 10.0f;
	matrix.m_posit.m_y += 4.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
