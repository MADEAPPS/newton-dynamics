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
#include "ndHeightFieldPrimitive.h"

void ndBasicHeighfieldCollision(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildHeightFieldTerrain(scene, "grass.png", ndGetIdentityMatrix()));

	ndVector floor(FindFloor(*scene->GetWorld(), ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	ndQuaternion rot(ndYawMatrix(180.0f * ndDegreeToRad));

	ndMatrix origin(ndCalculateMatrix(rot, floor));
	origin.m_posit += origin.m_front.Scale (40.0f);
	AddCapsuleStacks(scene, origin, 10.0f, 0.5f, 0.5f, 1.0f, 10, 10, 7);

	origin.m_posit += origin.m_right.Scale(20.0f);
	AddPlanks(scene, origin, 1.0f, 4);

	floor.m_y += 15.0f;
	scene->SetCameraMatrix(rot, floor);
}
