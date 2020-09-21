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
#include "ndSkyBox.h"
#include "ndTargaToOpenGl.h"
#include "ndDemoMesh.h"
#include "ndDemoEntityManager.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"

// the vertex array, vertices's has for values, x, y, z, w
// w is use as a id to have multiple copy of the same very, like for example mesh that share more than two edges.
// in most case w can be set to 0.0

void ndBasicSetup (ndDemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();

	dQuaternion rot;
	dVector origin(-10.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
