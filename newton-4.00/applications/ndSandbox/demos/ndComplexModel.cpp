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

	static void CreateExcavator(ndDemoEntityManager* const scene, const ndMatrix& location, ndRenderMeshLoader& mesh)
	{
		ndSharedPtr<ndRenderSceneNode> vehicleMesh(mesh.m_renderMesh->Clone());
		ndMatrix matrix(location);
		matrix.m_posit = FindFloor(*scene->GetWorld(), matrix.m_posit, 200.0f);


		scene->AddEntity(vehicleMesh);
	}
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

	ndExcavatorController::CreateExcavator(scene, matrix, excavatorLoaded);

	//// set this player as the active camera
	//ndBackGroundVehicleController* const playerController = (ndBackGroundVehicleController*)*controller;
	//ndRender* const renderer = *scene->GetRenderer();
	//renderer->SetCamera(playerController->GetCamera());

	matrix.m_posit.m_x -= 10.0f;
	matrix.m_posit.m_y += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
