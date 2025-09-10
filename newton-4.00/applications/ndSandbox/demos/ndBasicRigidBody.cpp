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
#include "ndDemoEntityManager.h"

void ndBasicRigidBody(ndDemoEntityManager* const scene)
{
	// build a floor
	//BuildFloorBox(scene, ndGetIdentityMatrix());
	//ndSharedPtr<ndBody> body(BuildFlatPlane(scene, true, true));
	//ndSharedPtr<ndBody> bodyFloor(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));
	ndSharedPtr<ndBody> bodyFloor(BuildFloorBox(scene, ndGetIdentityMatrix(), "blueCheckerboard.png", 0.1f, true));

	ndMatrix matrix(ndGetIdentityMatrix());

	ndMatrix origin1(matrix);
	origin1.m_posit.m_x = 20.0f;
	origin1.m_posit.m_y = 5.0f;

	ndSharedPtr<ndBody> body(AddSphere(scene, origin1, 100.0f, 2.0f));
	body->SetOmega(ndVector(0.0f, 2.0f, 0.0f, 0.0f));
	//body->SetMatrix(origin1);
	body->GetAsBodyKinematic()->SetMatrixUpdateScene(origin1);
	
	AddPlanks(scene, origin1, 1.0f, 4);

	origin1.m_posit.m_x += 20.0f;
	origin1.m_posit.m_z += 15.0f;
	//AddCapsulesStacks(scene, origin1, 10.0f, 0.5f, 0.5f, 1.0f, 1, 2, 7);
	AddCapsulesStacks(scene, origin1, 10.0f, 0.5f, 0.5f, 1.0f, 10, 10, 7);
	//AddCapsulesStacks(scene, origin1, 10.0f, 0.5f, 0.5f, 1.0f, 4, 4, 4);
	//AddCapsulesStacks(scene, origin1, 10.0f, 0.5f, 0.5f, 1.0f, 2, 2, 7);

	matrix.m_posit.m_x -= 10.0f;
	matrix.m_posit.m_y += 4.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
