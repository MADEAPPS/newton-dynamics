/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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
//#include "SkyBox.h"
//#include "TargaToOpenGl.h"
//#include "DemoMesh.h"

//#include "DemoCamera.h"
//#include "PhysicsUtils.h"
#include "ndDemoEntityManager.h"


void ndBasicSetup(ndDemoEntityManager* const scene)
{
#if 0
	// load the skybox
	scene->CreateSkyBox();

	// load the scene from a ngd file format
	CreateLevelMesh (scene, "flatPlane.ngd", true);

	NewtonSetContactMergeTolerance (scene->GetNewton(), 1.0e-3f);

	// test soft contacts
//	NewtonMaterialSetCollisionCallback(scene->GetNewton(), 0, 0, NULL, TestSoftContacts);
//	AddPrimitiveArray (scene, 1.0f, dVector (0.0f, 1.0f, -2.0f), dVector (1.0f, 1.0f, 1.0f), 1, 1, 0.1f, _SPHERE_PRIMITIVE, 0, dGetIdentityMatrix(), 0.0f, 0.0f);

	// make a box using low lever NetwonMesh
	NewtonBody* const body0 = CreateSimpleBox_NewtonMesh (scene, dVector (0.0f, 2.0f, -2.0f), dVector (1.0f, 0.5f, 2.0f, 0.0f), 2500.0f);
	body0;

	// make a box using the dNetwonMesh Class
	NewtonBody* const body1 = CreateSimpledBox_dNetwonMesh (scene, dVector (0.0f, 2.0f, 2.0f), dVector (1.0f, 0.5f, 2.0f, 0.0f), 500.0f);
	body1;

	dMatrix pinAndPivotFrame(dGetIdentityMatrix());
	pinAndPivotFrame.m_posit = dVector(0.0f, 2.5f, 0.0f, 1.0f);

	dCustomBallAndSocket* const joint = new dCustomBallAndSocket(pinAndPivotFrame, body0, body1);
	joint->SetBodyMassScale(5.0f, 1.0f);

	dQuaternion rot;
	dVector origin(-10.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);

//	NewtonSerializeToFile(scene->GetNewton(), "xxx.bin", NULL, NULL);
#endif
}
