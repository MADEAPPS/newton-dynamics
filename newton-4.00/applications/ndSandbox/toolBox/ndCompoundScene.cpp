/* Copyright (c) <2003-2021> <Newton Game Dynamics>
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
#include "ndDemoEntity.h"
//#include "ndDemoMesh.h"
//#include "ndDemoCamera.h"
//#include "ndPhysicsUtils.h"
//#include "ndDebugDisplay.h"
#include "ndPhysicsWorld.h"
//#include "ndTargaToOpenGl.h"
#include "ndCompoundScene.h"
#include "ndDemoEntityManager.h"
#include "ndHeightFieldPrimitive.h"


//ndBodyKinematic* BuildCompoundScene(ndDemoEntityManager* const scene, const dMatrix& location)
ndBodyKinematic* BuildCompoundScene(ndDemoEntityManager* const scene, const dMatrix& )
{
	ndShapeInstance sceneInstance (new ndShapeCompound());

	ndShapeCompound* const compound = sceneInstance.GetShape()->GetAsShapeCompound();
	compound->BeginAddRemove();

	AddHeightfield(scene, sceneInstance);

	compound->EndAddRemove();

	return nullptr;
}